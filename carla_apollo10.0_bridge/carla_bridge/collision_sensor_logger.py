#!/usr/bin/env python
#
# Collision logging support for CARLA ego vehicle.
#

import csv
import json
import math
import os
import struct
import time
from datetime import datetime
from threading import Lock, Thread


class CollisionSensorLogger:
    """
    Attach a CARLA collision sensor to the ego vehicle and persist collision events.
    """

    # Fixed-width history storage keeps the pre-collision memory allocation
    # deterministic.  One frame contains a 24-byte header followed by a fixed
    # number of 32-byte actor samples.
    _HISTORY_FRAME = struct.Struct("<IqdB3x")
    _HISTORY_ACTOR = struct.Struct("<I7f")

    HISTORY_FIELDS = [
        "role",
        "actor_id",
        "actor_type",
        "role_name",
        "carla_frame",
        "carla_timestamp_sec",
        "wall_time_unix_ns",
        "wall_time_iso",
        "location_x",
        "location_y",
        "location_z",
        "rotation_yaw",
        "velocity_x",
        "velocity_y",
        "velocity_z",
    ]

    JSON_FIELDS = [
        "event_type",
        "collision_seq",
        "wall_time_unix_ns",
        "wall_time_iso",
        "carla_frame",
        "carla_timestamp_sec",
        "map_name",
        "ego_actor_id",
        "ego_type_id",
        "ego_role_name",
        "other_actor_id",
        "other_type_id",
        "other_role_name",
        "other_semantic_tags",
        "normal_impulse_x",
        "normal_impulse_y",
        "normal_impulse_z",
        "normal_impulse_norm",
        "ego_location_x",
        "ego_location_y",
        "ego_location_z",
        "ego_rotation_roll",
        "ego_rotation_pitch",
        "ego_rotation_yaw",
        "ego_velocity_x",
        "ego_velocity_y",
        "ego_velocity_z",
        "ego_speed_mps",
        "other_location_x",
        "other_location_y",
        "other_location_z",
        "other_rotation_roll",
        "other_rotation_pitch",
        "other_rotation_yaw",
        "other_velocity_x",
        "other_velocity_y",
        "other_velocity_z",
        "other_speed_mps",
        "collision_history_status",
        "collision_history_path",
        "collision_history_row_count",
        "collision_history_buffer_bytes",
    ]

    def __init__(self, world, config, logger, node=None):
        self.world = world
        self.config = config or {}
        self.log = logger
        self.node = node
        self.enabled = self._config_value("enable_collision_logger", True)
        self.publish_cyber = self._config_value("collision_publish_cyber", True)
        self.topic = self._config_value("collision_topic", "/apollo/carla/collision")
        self.min_impulse = float(self._config_value("collision_min_impulse", 0.0))
        self.dedup_same_frame_same_other_actor = self._config_value(
            "dedup_same_frame_same_other_actor", False
        )
        self.first_event_only = bool(
            self._config_value("collision_first_event_only", True)
        )
        self.configured_log_dir = self._config_value(
            "collision_log_dir", "/apollo/data/collision_log"
        )
        self.json_name_template = self._config_value(
            "collision_log_jsonl", "carla_collision_events_{wall_time_iso}.jsonl"
        )
        self.csv_name_template = self._config_value(
            "collision_log_csv", "carla_collision_events_{wall_time_iso}.csv"
        )
        self.history_name_template = self._config_value(
            "collision_history_csv",
            "carla_collision_actor_history_{wall_time_iso}.csv",
        )

        fixed_delta_seconds = max(
            0.001,
            float(self.config.get("fixed_delta_seconds", 0.1) or 0.1),
        )
        self.history_enabled = bool(
            self._config_value("collision_history_enabled", True)
        )
        self.history_sec = max(
            fixed_delta_seconds,
            float(self._config_value("collision_history_sec", 10.0)),
        )
        self.history_max_actor_slots = max(
            2,
            min(
                255,
                int(self._config_value("collision_history_max_actor_slots", 8)),
            ),
        )
        self.history_candidate_refresh_frames = max(
            1,
            int(
                self._config_value(
                    "collision_history_candidate_refresh_frames", 10
                )
            ),
        )
        self.history_max_distance_m = max(
            0.0,
            float(self._config_value("collision_history_max_distance_m", 80.0)),
        )
        role_prefixes = self._config_value(
            "collision_history_priority_role_prefixes", ["scenario"]
        )
        if isinstance(role_prefixes, str):
            role_prefixes = [role_prefixes]
        self.history_priority_role_prefixes = tuple(
            str(value) for value in (role_prefixes or []) if str(value)
        )

        self._history_capacity = max(
            2, int(math.ceil(self.history_sec / fixed_delta_seconds))
        )
        self._history_stride = (
            self._HISTORY_FRAME.size
            + self.history_max_actor_slots * self._HISTORY_ACTOR.size
        )
        self._history_buffer = (
            bytearray(self._history_capacity * self._history_stride)
            if self.history_enabled
            else None
        )
        self._history_write_index = 0
        self._history_count = 0
        self._history_selected_actor_ids = []
        self._history_actor_meta = {}
        self._history_last_refresh_frame = None
        self._history_lock = Lock()
        self._collision_latched = False
        self._writer_threads = []

        self.sensor = None
        self.ego_vehicle = None
        self.collision_seq = 0
        self._last_dedup_key = None
        self._lock = Lock()
        self.json_file = None
        self.csv_file = None
        self.csv_writer = None
        self.cyber_writer = None
        self.cyber_message_type = None

        self.log_dir = None
        self.json_path = None
        self.csv_path = None
        self.history_path = None
        try:
            self.map_name = self.world.get_map().name
        except Exception:  # pylint: disable=broad-except
            self.map_name = ""

        if not self.enabled:
            self._log_info("Collision logger disabled by config.")
            return

        self._log_info(
            "Collision logger file output will be created on first collision: "
            "dir={}, jsonl={}, csv={}".format(
                self.configured_log_dir,
                self.json_name_template,
                self.csv_name_template,
            )
        )
        if self.history_enabled:
            self._log_info(
                "Collision history enabled: {:.1f}s, {} frames, {} actor slots, "
                "{} bytes fixed buffer.".format(
                    self.history_sec,
                    self._history_capacity,
                    self.history_max_actor_slots,
                    len(self._history_buffer),
                )
            )
        self._open_cyber_writer()

    def _config_value(self, key, default=None):
        nested = self.config.get("collision_logger", {})
        if key in self.config:
            return self.config.get(key)
        short_key = key.replace("collision_", "", 1)
        if short_key in nested:
            return nested.get(short_key)
        if key == "enable_collision_logger" and "enable" in nested:
            return nested.get("enable")
        return default

    def _log_info(self, message):
        if self.log:
            self.log.info(message)

    def _log_warning(self, message):
        if self.log:
            self.log.warning(message)

    def _log_error(self, message):
        if self.log:
            self.log.error(message)

    def _ensure_log_files(self, wall_time_iso):
        if self.json_file:
            return

        json_name = self._resolve_log_name(self.json_name_template, wall_time_iso)
        csv_name = self._resolve_log_name(self.csv_name_template, wall_time_iso)

        try:
            self._prepare_log_files(self.configured_log_dir, json_name, csv_name)
        except OSError as err:
            self._close_log_files()
            fallback_dir = os.path.join(os.getcwd(), "data", "log")
            self._log_warning(
                "Cannot open collision log dir {}: {}. Falling back to {}".format(
                    self.configured_log_dir, err, fallback_dir
                )
            )
            self._prepare_log_files(fallback_dir, json_name, csv_name)

        self._log_info(
            "Collision logger file output: jsonl={}, csv={}".format(
                self.json_path, self.csv_path
            )
        )

    def _resolve_log_name(self, template, wall_time_iso):
        if not template:
            return ""
        wall_time_iso_filename = self._format_wall_time_for_filename(wall_time_iso)
        return template.format(
            wall_time_iso=wall_time_iso_filename,
            first_wall_time_iso=wall_time_iso_filename,
        )

    @staticmethod
    def _format_wall_time_for_filename(value):
        try:
            return datetime.fromisoformat(str(value)).strftime("%Y%m%d%H%M%S")
        except ValueError:
            digits = "".join(char for char in str(value) if char.isdigit())
            return digits[:14] if len(digits) >= 14 else digits

    def _prepare_log_files(self, log_dir, json_name, csv_name):
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir
        self.json_path = os.path.join(log_dir, json_name)
        self.csv_path = os.path.join(log_dir, csv_name) if csv_name else None

        self.json_file = open(self.json_path, "a", encoding="utf-8")

        if self.csv_path:
            csv_exists = os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0
            self.csv_file = open(self.csv_path, "a", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.JSON_FIELDS)
            if not csv_exists:
                self.csv_writer.writeheader()
                self.csv_file.flush()

    def _close_log_files(self):
        for file_obj in (self.json_file, self.csv_file):
            if file_obj:
                try:
                    file_obj.close()
                except Exception:  # pylint: disable=broad-except
                    pass
        self.json_file = None
        self.csv_file = None
        self.csv_writer = None

    def _open_cyber_writer(self):
        if not self.publish_cyber or not self.node:
            return
        try:
            from google.protobuf.wrappers_pb2 import StringValue

            self.cyber_message_type = StringValue
            self.cyber_writer = self.node.new_writer(self.topic, StringValue, qos_depth=10)
            self._log_info(
                "Collision events will be published to Cyber topic {} as StringValue JSON.".format(
                    self.topic
                )
            )
        except Exception as err:  # pylint: disable=broad-except
            self.cyber_writer = None
            self.cyber_message_type = None
            self._log_warning(
                "Cyber collision topic {} is unavailable: {}. File logs remain enabled.".format(
                    self.topic, err
                )
            )

    def capture_snapshot(
        self, world_snapshot, frame_id, frame_wall_ns, actor_factory=None
    ):
        """Capture a compact pre-collision frame without CARLA RPCs.

        ``world_snapshot`` is the snapshot already fetched by the synchronous
        bridge loop.  ActorSnapshot access is local to that snapshot; this
        method deliberately never calls world.get_actors() or Actor getters.
        """
        if (
            not self.enabled
            or not self.history_enabled
            or self._history_buffer is None
            or self.ego_vehicle is None
            or self._collision_latched
            or world_snapshot is None
        ):
            return

        frame_id = int(frame_id)
        if self._history_selection_needs_refresh(frame_id):
            self._refresh_history_selection(world_snapshot, actor_factory, frame_id)

        actor_samples = []
        actor_meta = {}
        for actor_id in self._history_selected_actor_ids:
            actor_snapshot = self._snapshot_find(world_snapshot, actor_id)
            if actor_snapshot is None:
                continue
            try:
                transform = actor_snapshot.get_transform()
                velocity = actor_snapshot.get_velocity()
                actor_samples.append(
                    (
                        int(actor_id),
                        float(transform.location.x),
                        float(transform.location.y),
                        float(transform.location.z),
                        float(transform.rotation.yaw),
                        float(velocity.x),
                        float(velocity.y),
                        float(velocity.z),
                    )
                )
            except Exception:  # pylint: disable=broad-except
                continue
            meta = self._history_actor_meta.get(int(actor_id))
            if meta:
                actor_meta[int(actor_id)] = meta

        if not actor_samples:
            return

        timestamp = self._safe_get(world_snapshot, "timestamp")
        elapsed_seconds = float(
            self._safe_get(timestamp, "elapsed_seconds", 0.0) or 0.0
        )
        actor_samples = actor_samples[: self.history_max_actor_slots]

        with self._history_lock:
            if self._collision_latched:
                return
            offset = self._history_write_index * self._history_stride
            self._HISTORY_FRAME.pack_into(
                self._history_buffer,
                offset,
                frame_id,
                int(frame_wall_ns),
                elapsed_seconds,
                len(actor_samples),
            )
            actor_offset = offset + self._HISTORY_FRAME.size
            for sample in actor_samples:
                self._HISTORY_ACTOR.pack_into(
                    self._history_buffer, actor_offset, *sample
                )
                actor_offset += self._HISTORY_ACTOR.size
            self._history_actor_meta.update(actor_meta)
            self._history_write_index = (
                self._history_write_index + 1
            ) % self._history_capacity
            self._history_count = min(
                self._history_count + 1, self._history_capacity
            )

    def _history_selection_needs_refresh(self, frame_id):
        if not self._history_selected_actor_ids:
            return True
        if self._history_last_refresh_frame is None:
            return True
        return (
            frame_id - self._history_last_refresh_frame
            >= self.history_candidate_refresh_frames
        )

    def _refresh_history_selection(self, world_snapshot, actor_factory, frame_id):
        ego_id = self._safe_get(self.ego_vehicle, "id")
        if ego_id is None:
            return
        ego_id = int(ego_id)
        ego_snapshot = self._snapshot_find(world_snapshot, ego_id)
        if ego_snapshot is None:
            return
        try:
            ego_location = ego_snapshot.get_transform().location
        except Exception:  # pylint: disable=broad-except
            return

        actor_wrappers = []
        if actor_factory is not None:
            factory_lock = self._safe_get(actor_factory, "lock")
            actors = self._safe_get(actor_factory, "actors", {}) or {}
            if factory_lock is not None:
                with factory_lock:
                    actor_wrappers = list(actors.values())
            else:
                actor_wrappers = list(actors.values())

        candidates = []
        meta = {}
        ego_type = self._safe_get(self.ego_vehicle, "type_id", "")
        ego_role = self._actor_attribute(self.ego_vehicle, "role_name", "")
        meta[ego_id] = {"actor_type": ego_type, "role_name": ego_role}

        for wrapper in actor_wrappers:
            carla_actor = self._safe_get(wrapper, "carla_actor")
            actor_id = self._safe_get(carla_actor, "id")
            if carla_actor is None or actor_id is None or int(actor_id) == ego_id:
                continue
            type_id = str(self._safe_get(carla_actor, "type_id", "") or "")
            if not (
                type_id.startswith("vehicle.") or type_id.startswith("walker.")
            ):
                continue
            actor_snapshot = self._snapshot_find(world_snapshot, int(actor_id))
            if actor_snapshot is None:
                continue
            try:
                location = actor_snapshot.get_transform().location
            except Exception:  # pylint: disable=broad-except
                continue
            dx = float(location.x) - float(ego_location.x)
            dy = float(location.y) - float(ego_location.y)
            dz = float(location.z) - float(ego_location.z)
            distance_sq = dx * dx + dy * dy + dz * dz
            if (
                self.history_max_distance_m > 0.0
                and distance_sq > self.history_max_distance_m ** 2
            ):
                continue
            role_name = self._actor_attribute(carla_actor, "role_name", "")
            priority = 1
            if any(
                role_name.startswith(prefix)
                for prefix in self.history_priority_role_prefixes
            ):
                priority = 0
            actor_id = int(actor_id)
            candidates.append((priority, distance_sq, actor_id))
            meta[actor_id] = {
                "actor_type": type_id,
                "role_name": role_name,
            }

        candidates.sort(key=lambda value: (value[0], value[1], value[2]))
        selected = [ego_id]
        selected.extend(
            value[2]
            for value in candidates[: self.history_max_actor_slots - 1]
        )
        selected_meta = {
            actor_id: meta.get(actor_id, {}) for actor_id in selected
        }
        with self._history_lock:
            self._history_selected_actor_ids = selected
            # Bound metadata to the fixed actor-slot count as well.  Collision
            # event metadata fills any historical row whose actor later left
            # the selected set.
            self._history_actor_meta = selected_meta
            self._history_last_refresh_frame = frame_id

    @staticmethod
    def _snapshot_find(world_snapshot, actor_id):
        try:
            return world_snapshot.find(int(actor_id))
        except Exception:  # pylint: disable=broad-except
            return None

    def _freeze_history_bundle(self):
        """Transfer the fixed buffer to the one-shot writer without copying."""
        with self._history_lock:
            bundle = (
                self._history_buffer,
                self._history_write_index,
                self._history_count,
                self._history_capacity,
                self._history_stride,
                self._history_actor_meta,
            )
            # Capture is latched off after the first collision, so ownership of
            # the 28 KB bytearray can move directly to the writer thread.  This
            # avoids constructing hundreds of Python dicts in the sensor callback.
            self._history_buffer = None
            return bundle

    def _iter_history_rows(self, history_bundle, record):
        ego_id = self._to_int(record.get("ego_actor_id"))
        other_id = self._to_int(record.get("other_actor_id"))
        wanted = {actor_id for actor_id in (ego_id, other_id) if actor_id is not None}
        role_by_id = {ego_id: "ego", other_id: "other"}
        event_frame = self._to_int(record.get("carla_frame")) or 0
        event_frame_seen = {"ego": False, "other": False}

        if history_bundle and wanted:
            buffer_obj, write_index, count, capacity, stride, metadata_by_id = (
                history_bundle
            )
            if buffer_obj is not None:
                start = (write_index - count) % capacity
                for index in range(count):
                    slot = (start + index) % capacity
                    offset = slot * stride
                    frame_id, wall_ns, elapsed, actor_count = self._HISTORY_FRAME.unpack_from(
                        buffer_obj, offset
                    )
                    actor_offset = offset + self._HISTORY_FRAME.size
                    for _ in range(actor_count):
                        sample = self._HISTORY_ACTOR.unpack_from(
                            buffer_obj, actor_offset
                        )
                        actor_offset += self._HISTORY_ACTOR.size
                        actor_id = int(sample[0])
                        if actor_id not in wanted:
                            continue
                        role = role_by_id.get(actor_id, "other")
                        if int(frame_id) == event_frame:
                            event_frame_seen[role] = True
                        metadata = metadata_by_id.get(actor_id, {})
                        prefix = "ego" if actor_id == ego_id else "other"
                        yield {
                            "role": role,
                            "actor_id": actor_id,
                            "actor_type": metadata.get("actor_type")
                            or record.get(prefix + "_type_id", ""),
                            "role_name": metadata.get("role_name")
                            or record.get(prefix + "_role_name", ""),
                            "carla_frame": int(frame_id),
                            "carla_timestamp_sec": float(elapsed),
                            "wall_time_unix_ns": int(wall_ns),
                            "wall_time_iso": self._format_wall_time_ns_iso(wall_ns),
                            "location_x": float(sample[1]),
                            "location_y": float(sample[2]),
                            "location_z": float(sample[3]),
                            "rotation_yaw": float(sample[4]),
                            "velocity_x": float(sample[5]),
                            "velocity_y": float(sample[6]),
                            "velocity_z": float(sample[7]),
                        }

        for role in ("ego", "other"):
            if not event_frame_seen[role]:
                row = self._collision_state_row(record, role)
                if row is not None:
                    yield row

    def _collision_state_row(self, record, role):
        prefix = "ego" if role == "ego" else "other"
        actor_id = self._to_int(record.get(prefix + "_actor_id"))
        if actor_id is None:
            return None
        frame_id = self._to_int(record.get("carla_frame")) or 0
        return {
            "role": role,
            "actor_id": actor_id,
            "actor_type": record.get(prefix + "_type_id", ""),
            "role_name": record.get(prefix + "_role_name", ""),
            "carla_frame": frame_id,
            "carla_timestamp_sec": record.get("carla_timestamp_sec"),
            "wall_time_unix_ns": record.get("wall_time_unix_ns"),
            "wall_time_iso": self._format_wall_time_ns_iso(
                record.get("wall_time_unix_ns")
            ),
            "location_x": record.get(prefix + "_location_x"),
            "location_y": record.get(prefix + "_location_y"),
            "location_z": record.get(prefix + "_location_z"),
            "rotation_yaw": record.get(prefix + "_rotation_yaw"),
            "velocity_x": record.get(prefix + "_velocity_x"),
            "velocity_y": record.get(prefix + "_velocity_y"),
            "velocity_z": record.get(prefix + "_velocity_z"),
        }

    @staticmethod
    def _to_int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _format_wall_time_ns_iso(value):
        """Format Unix nanoseconds as local-time ISO 8601 without float loss."""
        try:
            unix_ns = int(value)
        except (TypeError, ValueError):
            return ""
        seconds, nanoseconds = divmod(unix_ns, 1000000000)
        local_time = datetime.fromtimestamp(seconds).astimezone()
        offset = local_time.strftime("%z")
        if len(offset) == 5:
            offset = offset[:3] + ":" + offset[3:]
        return "{}.{:09d}{}".format(
            local_time.strftime("%Y-%m-%dT%H:%M:%S"),
            nanoseconds,
            offset,
        )

    def attach_to(self, ego_vehicle):
        if not self.enabled or ego_vehicle is None:
            return

        ego_id = self._safe_get(ego_vehicle, "id")
        previous_ego_id = self._safe_get(self.ego_vehicle, "id")
        if (
            self.sensor is not None
            and self.ego_vehicle is not None
            and previous_ego_id == ego_id
        ):
            return

        self.detach()
        if previous_ego_id is not None and previous_ego_id != ego_id:
            self._reset_case_state()
        try:
            import carla

            blueprint = self.world.get_blueprint_library().find("sensor.other.collision")
            try:
                blueprint.set_attribute("role_name", "carla_collision_logger")
            except Exception:  # pylint: disable=broad-except
                pass

            try:
                self.sensor = self.world.spawn_actor(
                    blueprint,
                    carla.Transform(),
                    attach_to=ego_vehicle,
                    attachment_type=carla.AttachmentType.Rigid,
                )
            except TypeError:
                self.sensor = self.world.spawn_actor(
                    blueprint, carla.Transform(), attach_to=ego_vehicle
                )
            self.ego_vehicle = ego_vehicle
            self.sensor.listen(self._on_collision)
            self._log_info(
                "Collision sensor attached to ego vehicle id={}, type_id={}, log_path={}".format(
                    self._safe_get(ego_vehicle, "id"),
                    self._safe_get(ego_vehicle, "type_id", ""),
                    self.json_path or "pending_first_collision",
                )
            )
        except Exception as err:  # pylint: disable=broad-except
            self.sensor = None
            self.ego_vehicle = None
            self._log_error("Failed to attach CARLA collision sensor: {}".format(err))

    def _reset_case_state(self):
        """Reset the bounded history when a new ego starts a new case."""
        for thread in list(self._writer_threads):
            if thread.is_alive():
                thread.join(timeout=2.0)
        self._writer_threads = []
        with self._lock:
            self._close_log_files()
            self.log_dir = None
            self.json_path = None
            self.csv_path = None
            self.history_path = None
            self.collision_seq = 0
        with self._history_lock:
            if self.history_enabled and self._history_buffer is None:
                self._history_buffer = bytearray(
                    self._history_capacity * self._history_stride
                )
            self._history_write_index = 0
            self._history_count = 0
            self._history_selected_actor_ids = []
            self._history_actor_meta = {}
            self._history_last_refresh_frame = None
            self._last_dedup_key = None
            self._collision_latched = False

    def detach_from_ego(self, ego_actor_id):
        if self.ego_vehicle is None:
            return
        if self._safe_get(self.ego_vehicle, "id") == ego_actor_id:
            self.detach()

    def detach(self):
        sensor = self.sensor
        self.sensor = None
        self.ego_vehicle = None
        if sensor is None:
            return
        try:
            if self._safe_get(sensor, "is_listening", False):
                sensor.stop()
        except Exception as err:  # pylint: disable=broad-except
            self._log_warning("Failed to stop collision sensor: {}".format(err))
        try:
            sensor.destroy()
        except Exception as err:  # pylint: disable=broad-except
            self._log_warning("Failed to destroy collision sensor: {}".format(err))

    def is_sensor_actor_id(self, actor_id):
        return self.sensor is not None and self._safe_get(self.sensor, "id") == actor_id

    def destroy(self):
        self.detach()
        for thread in list(self._writer_threads):
            if thread.is_alive():
                thread.join(timeout=2.0)
        with self._lock:
            for file_obj in (self.json_file, self.csv_file):
                if file_obj:
                    try:
                        file_obj.flush()
                    except Exception:  # pylint: disable=broad-except
                        pass
            self._close_log_files()

    def _on_collision(self, event):
        try:
            impulse = self._safe_get(event, "normal_impulse")
            ix = float(self._safe_get(impulse, "x", 0.0) or 0.0)
            iy = float(self._safe_get(impulse, "y", 0.0) or 0.0)
            iz = float(self._safe_get(impulse, "z", 0.0) or 0.0)
            impulse_norm = math.sqrt(ix * ix + iy * iy + iz * iz)
            if impulse_norm < self.min_impulse:
                return

            other_actor = self._safe_get(event, "other_actor")
            dedup_key = (
                self._safe_get(event, "frame"),
                self._safe_get(other_actor, "id"),
            )
            with self._history_lock:
                if self.first_event_only and self._collision_latched:
                    return
                if (
                    self.dedup_same_frame_same_other_actor
                    and dedup_key == self._last_dedup_key
                ):
                    return
                self._last_dedup_key = dedup_key
                if self.first_event_only:
                    self._collision_latched = True

            # These actor calls now happen only once, after the collision has
            # already occurred.  The pre-collision hot path uses snapshots only.
            record = self.build_event_record(event)
            history_bundle = self._freeze_history_bundle()
            writer = Thread(
                target=self._persist_collision_bundle,
                args=(record, history_bundle),
                name="carla-collision-writer",
            )
            writer.daemon = True
            self._writer_threads.append(writer)
            writer.start()
        except Exception as err:  # pylint: disable=broad-except
            with self._history_lock:
                if self.first_event_only:
                    self._collision_latched = False
            self._log_error("CARLA_COLLISION logger callback failed: {}".format(err))

    def _persist_collision_bundle(self, record, history_bundle):
        try:
            with self._lock:
                output_dir = self._write_history_before_event_locked(
                    record["wall_time_iso"], history_bundle, record
                )
                json_name = self._resolve_log_name(
                    self.json_name_template, record["wall_time_iso"]
                )
                csv_name = self._resolve_log_name(
                    self.csv_name_template, record["wall_time_iso"]
                )
                try:
                    self._prepare_log_files(output_dir, json_name, csv_name)
                except OSError as err:
                    fallback_dir = os.path.join(os.getcwd(), "data", "log")
                    self._log_warning(
                        "Cannot open collision event files in {}: {}. "
                        "Falling back to {}".format(output_dir, err, fallback_dir)
                    )
                    self._close_log_files()
                    self._prepare_log_files(fallback_dir, json_name, csv_name)

                json_line = json.dumps(
                    record, ensure_ascii=False, separators=(",", ":")
                )
                if self.json_file:
                    self.json_file.write(json_line + "\n")
                if self.csv_writer and self.csv_file:
                    self.csv_writer.writerow(record)
                for file_obj in (self.json_file, self.csv_file):
                    if file_obj:
                        file_obj.flush()

            # Publish at most one compact collision event.  History is never
            # sent through Cyber and therefore never adds bridge/Orin traffic.
            self._publish_cyber(json_line)
            self._log_error(self._format_error_log(record))
        except Exception as err:  # pylint: disable=broad-except
            self._log_error(
                "Failed to persist CARLA collision bundle: {}".format(err)
            )

    def _write_history_before_event_locked(
        self, wall_time_iso, history_bundle, record
    ):
        """Persist history atomically before exposing the collision event file.

        Some collection workflows stop the bridge as soon as the event CSV
        appears.  Writing history first guarantees that the event file acts as
        a completion marker rather than a start marker.
        """
        record["collision_history_buffer_bytes"] = (
            len(history_bundle[0])
            if history_bundle and history_bundle[0] is not None
            else 0
        )
        if not self.history_enabled or not self.history_name_template:
            record["collision_history_status"] = "DISABLED"
            record["collision_history_path"] = ""
            record["collision_history_row_count"] = 0
            return self.configured_log_dir

        history_name = self._resolve_log_name(
            self.history_name_template, wall_time_iso
        )
        fallback_dir = os.path.join(os.getcwd(), "data", "log")
        candidate_dirs = []
        for value in (self.configured_log_dir, fallback_dir):
            value = os.path.abspath(value)
            if value not in candidate_dirs:
                candidate_dirs.append(value)

        errors = []
        for output_dir in candidate_dirs:
            history_path = os.path.join(output_dir, history_name)
            temp_path = history_path + ".tmp.{}".format(os.getpid())
            try:
                os.makedirs(output_dir, exist_ok=True)
                row_count = 0
                with open(
                    temp_path, "w", encoding="utf-8", newline=""
                ) as file_obj:
                    writer = csv.DictWriter(
                        file_obj, fieldnames=self.HISTORY_FIELDS
                    )
                    writer.writeheader()
                    for row in self._iter_history_rows(history_bundle, record):
                        writer.writerow(row)
                        row_count += 1
                os.replace(temp_path, history_path)
                self.log_dir = output_dir
                self.history_path = history_path
                record["collision_history_status"] = "WRITTEN"
                record["collision_history_path"] = history_path
                record["collision_history_row_count"] = row_count
                self._log_info(
                    "Collision history file output: path={}, rows={}, "
                    "buffer_bytes={}".format(
                        history_path,
                        row_count,
                        record["collision_history_buffer_bytes"],
                    )
                )
                return output_dir
            except Exception as err:  # pylint: disable=broad-except
                errors.append("{}: {}".format(output_dir, err))
                try:
                    if os.path.exists(temp_path):
                        os.remove(temp_path)
                except OSError:
                    pass

        record["collision_history_status"] = "FAILED"
        record["collision_history_path"] = ""
        record["collision_history_row_count"] = 0
        self._log_error(
            "Collision history output failed in all directories: {}".format(
                "; ".join(errors)
            )
        )
        return fallback_dir

    def build_event_record(self, event):
        event_frame = self._safe_get(event, "frame")
        wall_time_unix_ns = self._history_wall_time_ns(event_frame)
        if wall_time_unix_ns is None:
            wall_time_unix_ns = time.time_ns()
        wall_time_iso = datetime.fromtimestamp(
            wall_time_unix_ns / 1000000000.0
        ).astimezone().isoformat()

        actor = self._safe_get(event, "actor")
        other_actor = self._safe_get(event, "other_actor")
        impulse = self._safe_get(event, "normal_impulse")
        ix = self._safe_get(impulse, "x", 0.0)
        iy = self._safe_get(impulse, "y", 0.0)
        iz = self._safe_get(impulse, "z", 0.0)
        impulse_norm = math.sqrt(ix * ix + iy * iy + iz * iz)

        ego_transform = self._safe_call(actor, "get_transform")
        ego_location = self._safe_get(ego_transform, "location")
        ego_rotation = self._safe_get(ego_transform, "rotation")
        ego_velocity = self._safe_call(actor, "get_velocity")
        evx = self._safe_get(ego_velocity, "x", 0.0)
        evy = self._safe_get(ego_velocity, "y", 0.0)
        evz = self._safe_get(ego_velocity, "z", 0.0)
        ego_speed = math.sqrt(evx * evx + evy * evy + evz * evz)

        other_transform = self._safe_call(other_actor, "get_transform")
        other_location = self._safe_get(other_transform, "location")
        other_rotation = self._safe_get(other_transform, "rotation")
        other_velocity = self._safe_call(other_actor, "get_velocity")
        ovx = self._safe_get(other_velocity, "x", 0.0)
        ovy = self._safe_get(other_velocity, "y", 0.0)
        ovz = self._safe_get(other_velocity, "z", 0.0)
        other_speed = math.sqrt(ovx * ovx + ovy * ovy + ovz * ovz)

        with self._lock:
            self.collision_seq += 1
            collision_seq = self.collision_seq

        return {
            "event_type": "CARLA_COLLISION",
            "collision_seq": collision_seq,
            "wall_time_unix_ns": wall_time_unix_ns,
            "wall_time_iso": wall_time_iso,
            "carla_frame": event_frame,
            "carla_timestamp_sec": self._safe_get(event, "timestamp"),
            "map_name": self._get_map_name(),
            "ego_actor_id": self._safe_get(actor, "id"),
            "ego_type_id": self._safe_get(actor, "type_id", ""),
            "ego_role_name": self._actor_attribute(actor, "role_name", ""),
            "other_actor_id": self._safe_get(other_actor, "id"),
            "other_type_id": self._safe_get(other_actor, "type_id", ""),
            "other_role_name": self._actor_attribute(other_actor, "role_name", ""),
            "other_semantic_tags": self._semantic_tags(other_actor),
            "normal_impulse_x": ix,
            "normal_impulse_y": iy,
            "normal_impulse_z": iz,
            "normal_impulse_norm": impulse_norm,
            "ego_location_x": self._safe_get(ego_location, "x"),
            "ego_location_y": self._safe_get(ego_location, "y"),
            "ego_location_z": self._safe_get(ego_location, "z"),
            "ego_rotation_roll": self._safe_get(ego_rotation, "roll"),
            "ego_rotation_pitch": self._safe_get(ego_rotation, "pitch"),
            "ego_rotation_yaw": self._safe_get(ego_rotation, "yaw"),
            "ego_velocity_x": evx,
            "ego_velocity_y": evy,
            "ego_velocity_z": evz,
            "ego_speed_mps": ego_speed,
            "other_location_x": self._safe_get(other_location, "x"),
            "other_location_y": self._safe_get(other_location, "y"),
            "other_location_z": self._safe_get(other_location, "z"),
            "other_rotation_roll": self._safe_get(other_rotation, "roll"),
            "other_rotation_pitch": self._safe_get(other_rotation, "pitch"),
            "other_rotation_yaw": self._safe_get(other_rotation, "yaw"),
            "other_velocity_x": ovx,
            "other_velocity_y": ovy,
            "other_velocity_z": ovz,
            "other_speed_mps": other_speed,
        }

    def _history_wall_time_ns(self, frame_id):
        frame_id = self._to_int(frame_id)
        if (
            frame_id is None
            or not self.history_enabled
            or self._history_buffer is None
        ):
            return None
        with self._history_lock:
            start = (
                self._history_write_index - self._history_count
            ) % self._history_capacity
            for index in range(self._history_count):
                slot = (start + index) % self._history_capacity
                offset = slot * self._history_stride
                stored_frame, wall_ns, _, _ = self._HISTORY_FRAME.unpack_from(
                    self._history_buffer, offset
                )
                if int(stored_frame) == frame_id:
                    return int(wall_ns)
        return None

    def _publish_cyber(self, json_line):
        if not self.cyber_writer or not self.cyber_message_type:
            return
        try:
            self.cyber_writer.write(self.cyber_message_type(value=json_line))
        except Exception as err:  # pylint: disable=broad-except
            self._log_warning("Failed to publish collision event to Cyber: {}".format(err))

    def _format_error_log(self, record):
        return (
            "CARLA_COLLISION seq={collision_seq} frame={carla_frame} "
            "carla_time={carla_timestamp_sec} ego_id={ego_actor_id} "
            "other_id={other_actor_id} other_type={other_type_id} "
            "impulse_norm={normal_impulse_norm:.6f} ego_x={ego_location_x} "
            "ego_y={ego_location_y} speed={ego_speed_mps:.6f}"
        ).format(**record)

    def _get_map_name(self):
        return self.map_name

    @staticmethod
    def _safe_get(obj, attr, default=None):
        if obj is None:
            return default
        try:
            return getattr(obj, attr)
        except Exception:  # pylint: disable=broad-except
            return default

    @staticmethod
    def _safe_call(obj, method, default=None):
        if obj is None:
            return default
        try:
            return getattr(obj, method)()
        except Exception:  # pylint: disable=broad-except
            return default

    @staticmethod
    def _actor_attribute(actor, name, default=""):
        if actor is None:
            return default
        try:
            attributes = actor.attributes
            return attributes.get(name, default)
        except Exception:  # pylint: disable=broad-except
            return default

    @staticmethod
    def _semantic_tags(actor):
        if actor is None:
            return []
        try:
            return list(actor.semantic_tags)
        except Exception:  # pylint: disable=broad-except
            return []
