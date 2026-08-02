#!/usr/bin/env python
"""Small non-blocking control delay injector for SCB deadline experiments."""

from __future__ import print_function

import csv
import heapq
import os
import threading
import time
from datetime import datetime


class ControlDelayInjector(object):
    """Delay the first effective brake and every later ControlCommand.

    The Cyber callback only copies and enqueues the protobuf.  One daemon worker
    performs the delayed calls to CARLA, so Apollo/Bridge communication is never
    blocked by ``sleep``.  Until the trigger condition is met, commands retain
    the original direct-apply path.
    """

    LOG_FIELDS = [
        "schema_version",
        "bridge_entry_file",
        "settings_source_file",
        "injector_source_file",
        "process_id",
        "process_working_directory",
        "activation_speed_mps",
        "brake_threshold_percentage",
        "log_all_delayed_commands",
        "sequence",
        "control_header_time_sec",
        "receive_wall_time_unix_ns",
        "receive_monotonic_ns",
        "receive_carla_frame",
        "receive_carla_elapsed_sec",
        "release_wall_time_unix_ns",
        "release_monotonic_ns",
        "apply_call_start_wall_time_unix_ns",
        "apply_call_start_monotonic_ns",
        "apply_call_end_wall_time_unix_ns",
        "apply_call_end_monotonic_ns",
        "apply_wall_time_unix_ns",
        "apply_monotonic_ns",
        "apply_carla_frame",
        "apply_carla_elapsed_sec",
        "requested_delay_ms",
        "actual_delay_ms",
        "api_completion_delay_ms",
        "api_call_duration_ms",
        "actual_frame_delay",
        "actual_sim_delay_ms",
        "ego_speed_mps_at_receive",
        "throttle_percentage",
        "brake_percentage",
        "steering_target_percentage",
        "first_effective_brake",
        "queue_depth",
        "status",
    ]

    @staticmethod
    def _as_bool(value):
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "yes", "on", "1"}:
                return True
            if normalized in {"false", "no", "off", "0", ""}:
                return False
            raise ValueError("invalid boolean value: {!r}".format(value))
        return bool(value)

    @classmethod
    def normalize_bridge_parameters(
        cls,
        parameters,
        bridge_entry_file="",
        settings_source_file="",
        logger=None,
    ):
        """Return CARLA parameters and normalize the delay-config location.

        The Bridge historically stores settings below ``carla:``, while some
        deployment scripts append experiment blocks at the YAML root.  Accept
        either location, but reject conflicting duplicates so an experiment
        can never run with an silently selected configuration.
        """
        if not isinstance(parameters, dict):
            raise RuntimeError("Bridge settings must be a YAML mapping")
        carla_parameters = parameters.get("carla")
        if not isinstance(carla_parameters, dict):
            raise RuntimeError("Bridge settings are missing the 'carla' mapping")

        nested = carla_parameters.get("control_delay_injection")
        root = parameters.get("control_delay_injection")
        if nested is not None and not isinstance(nested, dict):
            raise RuntimeError("carla.control_delay_injection must be a mapping")
        if root is not None and not isinstance(root, dict):
            raise RuntimeError("root control_delay_injection must be a mapping")
        config_location = "carla"
        if nested is not None and root is not None and nested != root:
            raise RuntimeError(
                "Conflicting control_delay_injection blocks exist at YAML root "
                "and under carla; keep exactly one"
            )
        if nested is None and root is not None:
            carla_parameters["control_delay_injection"] = root
            nested = root
            message = (
                "SCB config found at YAML root; normalized to "
                "carla.control_delay_injection"
            )
            print(message, flush=True)
            if logger:
                logger.warning(message)
            config_location = "yaml_root_normalized_to_carla"
        elif nested is None:
            config_location = "missing"

        cfg = nested or {}
        if bridge_entry_file:
            cfg["_bridge_entry_file"] = os.path.abspath(bridge_entry_file)
        if settings_source_file:
            cfg["_settings_source_file"] = os.path.abspath(settings_source_file)
        if "enabled" in cfg:
            # Validate now.  In particular, the string "False" must not be
            # treated as truthy by Python's normal bool(str) behavior.
            cfg["enabled"] = cls._as_bool(cfg.get("enabled"))
        if "log_all_delayed_commands" in cfg:
            cfg["log_all_delayed_commands"] = cls._as_bool(
                cfg.get("log_all_delayed_commands")
            )
        if not cfg.get("_resolution_reported"):
            enabled = cls._as_bool(cfg.get("enabled", False))
            message = (
                "SCB CONFIG RESOLVED: location={} enabled={} settings={} "
                "injector={}".format(
                    config_location,
                    enabled,
                    cfg.get("_settings_source_file", "") or "<not supplied>",
                    os.path.abspath(__file__),
                )
            )
            print(message, flush=True)
            if logger:
                logger.info(message)
            if cfg:
                cfg["_resolution_reported"] = True
        return carla_parameters

    def __init__(self, config, apply_callback, speed_callback, logger=None):
        cfg = (config or {}).get("control_delay_injection", {}) or {}
        self.enabled = self._as_bool(cfg.get("enabled", False))
        self.delay_ms = max(0.0, float(cfg.get("delay_ms", 0.0)))
        self.activation_speed_mps = max(
            0.0, float(cfg.get("activation_speed_mps", 21.5))
        )
        self.brake_threshold_percentage = max(
            0.0, float(cfg.get("brake_threshold_percentage", 1.0))
        )
        self.queue_max_messages = max(
            8, int(cfg.get("queue_max_messages", 64))
        )
        self.log_all_delayed_commands = self._as_bool(
            cfg.get("log_all_delayed_commands", False)
        )
        self.log_dir = str(cfg.get("log_dir", "/apollo/data/log"))
        self.log_name = str(
            cfg.get("log_csv", "scb_control_delay_{wall_time_iso}.csv")
        )
        self._resolved_log_path = cfg.get("_resolved_log_path")
        self._bridge_entry_file = str(cfg.get("_bridge_entry_file", ""))
        self._settings_source_file = str(cfg.get("_settings_source_file", ""))
        self.apply_callback = apply_callback
        self.speed_callback = speed_callback
        self.logger = logger

        self._condition = threading.Condition(threading.RLock())
        self._queue = []
        self._sequence = 0
        self._armed = False
        self._triggered = False
        self._stopping = False
        self._latest_frame = None
        self._latest_elapsed_sec = None
        self._latest_speed_mps = None
        self._file = None
        self._writer = None
        self.log_path = None
        self._thread = None
        if self.enabled:
            # Create and flush the header at startup.  An empty file now means
            # that the injector was enabled but no effective brake was seen;
            # a missing file means that this injector instance never started.
            self._ensure_writer()
            self._write_lifecycle_record("INITIALIZED")
            self._thread = threading.Thread(
                target=self._worker, name="scb-control-delay"
            )
            self._thread.daemon = True
            self._thread.start()
            self._info(
                "SCB control delay enabled: delay_ms={} speed_mps>={} brake%>={}"
                .format(
                    self.delay_ms,
                    self.activation_speed_mps,
                    self.brake_threshold_percentage,
                )
            )
            if self.log_all_delayed_commands:
                self._warning(
                    "SCB log_all_delayed_commands=true: /apollo/control may run "
                    "near 100 Hz; use this only for short diagnostics."
                )

    @classmethod
    def prepare_startup_evidence(cls, config, logger=None):
        """Create the experiment CSV as soon as Bridge loads its settings.

        This deliberately runs before CARLA connection and ego discovery.  An
        enabled experiment is therefore not allowed to continue silently when
        its evidence file cannot be created.
        """
        cfg = (config or {}).get("control_delay_injection", {}) or {}
        if not cls._as_bool(cfg.get("enabled", False)):
            return None
        existing = cfg.get("_resolved_log_path")
        if existing:
            return str(existing)

        log_dir = str(cfg.get("log_dir", "/apollo/data/log"))
        log_name = str(
            cfg.get("log_csv", "scb_control_delay_{wall_time_iso}.csv")
        )
        stamp = datetime.now().strftime("%Y%m%d%H%M%S_%f")
        name = log_name.format(wall_time_iso=stamp)
        attempts = [log_dir]
        fallback = os.path.join(os.getcwd(), "data", "log")
        if os.path.abspath(fallback) != os.path.abspath(log_dir):
            attempts.append(fallback)

        failures = []
        for directory in attempts:
            try:
                os.makedirs(directory, exist_ok=True)
                path = os.path.abspath(os.path.join(directory, name))
                with open(path, "x", encoding="utf-8", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=cls.LOG_FIELDS)
                    writer.writeheader()
                    values = dict((field, "") for field in cls.LOG_FIELDS)
                    values.update(cls._config_log_values(cfg))
                    values.update({
                        "requested_delay_ms": max(
                            0.0, float(cfg.get("delay_ms", 0.0))
                        ),
                        "queue_depth": 0,
                        "status": "BRIDGE_CONFIG_LOADED",
                    })
                    writer.writerow(values)
                    handle.flush()
                    os.fsync(handle.fileno())
                cfg["_resolved_log_path"] = path
                message = (
                    "SCB STARTUP EVIDENCE CREATED: {} "
                    "(enabled=True; no speed/brake trigger required)".format(path)
                )
                print(message, flush=True)
                if logger:
                    logger.info(message)
                return path
            except (OSError, ValueError) as exc:
                failures.append("{}: {}".format(directory, exc))

        message = (
            "SCB EVIDENCE CREATION FAILED while enabled=True; Bridge will not "
            "start. Attempts: {}".format(" | ".join(failures))
        )
        print(message, flush=True)
        if logger:
            logger.error(message)
        raise RuntimeError(message)

    @classmethod
    def _config_log_values(cls, cfg):
        return {
            "schema_version": "scb_control_delay_v3",
            "bridge_entry_file": str(cfg.get("_bridge_entry_file", "")),
            "settings_source_file": str(cfg.get("_settings_source_file", "")),
            "injector_source_file": os.path.abspath(__file__),
            "process_id": os.getpid(),
            "process_working_directory": os.getcwd(),
            "activation_speed_mps": max(
                0.0, float(cfg.get("activation_speed_mps", 21.5))
            ),
            "brake_threshold_percentage": max(
                0.0, float(cfg.get("brake_threshold_percentage", 1.0))
            ),
            "log_all_delayed_commands": int(
                cls._as_bool(cfg.get("log_all_delayed_commands", False))
            ),
        }

    def update_carla_time(self, frame, elapsed_sec, ego_speed_mps=None):
        """Receive the latest tick without performing any CARLA RPC."""
        newly_armed = False
        with self._condition:
            self._latest_frame = int(frame) if frame is not None else None
            self._latest_elapsed_sec = (
                float(elapsed_sec) if elapsed_sec is not None else None
            )
            self._latest_speed_mps = (
                float(ego_speed_mps) if ego_speed_mps is not None else None
            )
            newly_armed = self._arm_for_speed_locked(self._latest_speed_mps)
        if newly_armed:
            self._info(
                "SCB control delay armed: speed_mps={} threshold_mps={}".format(
                    self._latest_speed_mps, self.activation_speed_mps
                )
            )

    def _arm_for_speed_locked(self, speed_mps):
        """Latch the experiment once the ego has reached activation speed."""
        if (
            not self._armed
            and speed_mps is not None
            and speed_mps >= self.activation_speed_mps
        ):
            self._armed = True
            return True
        return False

    @staticmethod
    def _number(message, field, default=0.0):
        try:
            return float(getattr(message, field))
        except (AttributeError, TypeError, ValueError):
            return float(default)

    @staticmethod
    def _header_time(message):
        try:
            return float(message.header.timestamp_sec)
        except (AttributeError, TypeError, ValueError):
            return 0.0

    @staticmethod
    def _copy_message(message):
        copied = type(message)()
        if hasattr(copied, "CopyFrom"):
            copied.CopyFrom(message)
        else:
            copied.__dict__.update(getattr(message, "__dict__", {}))
        return copied

    def submit(self, message):
        """Apply immediately or enqueue; returns True when delayed."""
        if not self.enabled:
            self.apply_callback(message)
            return False

        with self._condition:
            speed = self._latest_speed_mps
        if speed is None:
            try:
                speed = float(self.speed_callback())
            except Exception:  # pylint: disable=broad-except
                speed = 0.0
        brake = self._number(message, "brake")
        first_brake = False
        newly_armed = False
        with self._condition:
            # The speed condition is intentionally latched.  Once the vehicle
            # has reached the experiment speed, a later effective brake still
            # triggers injection even if speed has already started to fall.
            newly_armed = self._arm_for_speed_locked(speed)
            if not self._triggered:
                if (
                    not self._armed
                    or brake < self.brake_threshold_percentage
                ):
                    direct = True
                else:
                    self._triggered = True
                    first_brake = True
                    direct = False
            else:
                direct = False
            frame = self._latest_frame
            elapsed = self._latest_elapsed_sec

        if newly_armed:
            self._info(
                "SCB control delay armed: speed_mps={} threshold_mps={}".format(
                    speed, self.activation_speed_mps
                )
            )

        if direct:
            self.apply_callback(message)
            return False

        receive_mono_ns = time.monotonic_ns()
        record = {
            "message": self._copy_message(message),
            "receive_wall_ns": time.time_ns(),
            "receive_mono_ns": receive_mono_ns,
            "release_mono_ns": receive_mono_ns
            + int(round(self.delay_ms * 1000000.0)),
            "receive_frame": frame,
            "receive_elapsed_sec": elapsed,
            "speed": speed,
            "first_brake": first_brake,
        }
        with self._condition:
            self._sequence += 1
            record["sequence"] = self._sequence
            record["queue_depth"] = len(self._queue) + 1
            if len(self._queue) >= self.queue_max_messages:
                dropped = heapq.heappop(self._queue)[2]
                self._write_record(dropped, None, None, "DROPPED_QUEUE_FULL")
            heapq.heappush(
                self._queue,
                (record["release_mono_ns"], record["sequence"], record),
            )
            self._condition.notify()
        return True

    def _worker(self):
        while True:
            with self._condition:
                while not self._queue and not self._stopping:
                    self._condition.wait()
                if self._stopping and not self._queue:
                    return
                release_ns, _, record = self._queue[0]
                wait_sec = (release_ns - time.monotonic_ns()) / 1000000000.0
                if wait_sec > 0.0:
                    self._condition.wait(timeout=wait_sec)
                    continue
                heapq.heappop(self._queue)
            release_mono_ns = time.monotonic_ns()
            release_wall_ns = time.time_ns()
            apply_start_mono_ns = time.monotonic_ns()
            apply_start_wall_ns = time.time_ns()
            status = "APPLIED"
            try:
                self.apply_callback(record["message"])
            except Exception as exc:  # pylint: disable=broad-except
                status = "APPLY_FAILED:{}".format(exc)
                self._warning(status)
            apply_end_mono_ns = time.monotonic_ns()
            apply_end_wall_ns = time.time_ns()
            record.update({
                "release_mono_ns": release_mono_ns,
                "release_wall_ns": release_wall_ns,
                "apply_start_mono_ns": apply_start_mono_ns,
                "apply_start_wall_ns": apply_start_wall_ns,
                "apply_end_mono_ns": apply_end_mono_ns,
                "apply_end_wall_ns": apply_end_wall_ns,
            })
            if (
                record["first_brake"]
                or self.log_all_delayed_commands
                or status != "APPLIED"
            ):
                # Legacy apply_* columns now mean CARLA API call completion,
                # not physical brake onset.  The offline analyzer derives the
                # latter from post-command localization/velocity evidence.
                self._write_record(
                    record, apply_end_wall_ns, apply_end_mono_ns, status
                )

    def _ensure_writer(self):
        if self._writer is not None:
            return
        path = None
        append_existing = False
        if self._resolved_log_path:
            path = os.path.abspath(str(self._resolved_log_path))
            append_existing = os.path.isfile(path) and os.path.getsize(path) > 0
        try:
            if path is None:
                stamp = datetime.now().strftime("%Y%m%d%H%M%S_%f")
                name = self.log_name.format(wall_time_iso=stamp)
                os.makedirs(self.log_dir, exist_ok=True)
                path = os.path.abspath(os.path.join(self.log_dir, name))
            else:
                os.makedirs(os.path.dirname(path), exist_ok=True)
            self._file = open(
                path,
                "a" if append_existing else "w",
                encoding="utf-8",
                newline="",
            )
        except OSError as exc:
            if self._resolved_log_path:
                raise RuntimeError(
                    "SCB startup evidence existed but cannot be reopened: {}: {}"
                    .format(path, exc)
                )
            stamp = datetime.now().strftime("%Y%m%d%H%M%S_%f")
            name = self.log_name.format(wall_time_iso=stamp)
            fallback = os.path.join(os.getcwd(), "data", "log")
            os.makedirs(fallback, exist_ok=True)
            path = os.path.abspath(os.path.join(fallback, name))
            self._file = open(path, "w", encoding="utf-8", newline="")
            self._warning("SCB log fallback after {}: {}".format(exc, path))
        self.log_path = path
        self._writer = csv.DictWriter(self._file, fieldnames=self.LOG_FIELDS)
        if not append_existing:
            self._writer.writeheader()
        self._file.flush()
        self._info("SCB control-delay evidence: {}".format(path))

    def _base_log_values(self):
        return {
            "schema_version": "scb_control_delay_v3",
            "bridge_entry_file": self._bridge_entry_file,
            "settings_source_file": self._settings_source_file,
            "injector_source_file": os.path.abspath(__file__),
            "process_id": os.getpid(),
            "process_working_directory": os.getcwd(),
            "activation_speed_mps": self.activation_speed_mps,
            "brake_threshold_percentage": self.brake_threshold_percentage,
            "log_all_delayed_commands": int(self.log_all_delayed_commands),
        }

    def _write_lifecycle_record(self, status):
        """Write one small startup row proving which config/code was loaded."""
        with self._condition:
            self._ensure_writer()
            values = dict((field, "") for field in self.LOG_FIELDS)
            values.update(self._base_log_values())
            values.update({
                "requested_delay_ms": self.delay_ms,
                "queue_depth": len(self._queue),
                "status": status,
            })
            self._writer.writerow(values)
            self._file.flush()

    def _write_record(self, record, apply_wall_ns, apply_mono_ns, status):
        with self._condition:
            self._ensure_writer()
            apply_frame = self._latest_frame
            apply_elapsed = self._latest_elapsed_sec
            actual_delay_ms = None
            release_mono_ns = record.get("release_mono_ns")
            if release_mono_ns is not None:
                actual_delay_ms = (
                    release_mono_ns - record["receive_mono_ns"]
                ) / 1000000.0
            elif apply_mono_ns is not None:
                actual_delay_ms = (
                    apply_mono_ns - record["receive_mono_ns"]
                ) / 1000000.0
            api_completion_delay_ms = None
            if apply_mono_ns is not None:
                api_completion_delay_ms = (
                    apply_mono_ns - record["receive_mono_ns"]
                ) / 1000000.0
            api_call_duration_ms = None
            if (
                record.get("apply_start_mono_ns") is not None
                and record.get("apply_end_mono_ns") is not None
            ):
                api_call_duration_ms = (
                    record["apply_end_mono_ns"]
                    - record["apply_start_mono_ns"]
                ) / 1000000.0
            frame_delay = None
            if apply_frame is not None and record["receive_frame"] is not None:
                frame_delay = apply_frame - record["receive_frame"]
            sim_delay_ms = None
            if (
                apply_elapsed is not None
                and record["receive_elapsed_sec"] is not None
            ):
                sim_delay_ms = (
                    apply_elapsed - record["receive_elapsed_sec"]
                ) * 1000.0
            message = record["message"]
            values = self._base_log_values()
            values.update({
                "sequence": record["sequence"],
                "control_header_time_sec": self._header_time(message),
                "receive_wall_time_unix_ns": record["receive_wall_ns"],
                "receive_monotonic_ns": record["receive_mono_ns"],
                "receive_carla_frame": record["receive_frame"],
                "receive_carla_elapsed_sec": record["receive_elapsed_sec"],
                "release_wall_time_unix_ns": record.get("release_wall_ns"),
                "release_monotonic_ns": record.get("release_mono_ns"),
                "apply_call_start_wall_time_unix_ns": record.get(
                    "apply_start_wall_ns"
                ),
                "apply_call_start_monotonic_ns": record.get(
                    "apply_start_mono_ns"
                ),
                "apply_call_end_wall_time_unix_ns": record.get(
                    "apply_end_wall_ns"
                ),
                "apply_call_end_monotonic_ns": record.get(
                    "apply_end_mono_ns"
                ),
                "apply_wall_time_unix_ns": apply_wall_ns,
                "apply_monotonic_ns": apply_mono_ns,
                "apply_carla_frame": apply_frame,
                "apply_carla_elapsed_sec": apply_elapsed,
                "requested_delay_ms": self.delay_ms,
                "actual_delay_ms": actual_delay_ms,
                "api_completion_delay_ms": api_completion_delay_ms,
                "api_call_duration_ms": api_call_duration_ms,
                "actual_frame_delay": frame_delay,
                "actual_sim_delay_ms": sim_delay_ms,
                "ego_speed_mps_at_receive": record["speed"],
                "throttle_percentage": self._number(message, "throttle"),
                "brake_percentage": self._number(message, "brake"),
                "steering_target_percentage": self._number(
                    message, "steering_target"
                ),
                "first_effective_brake": int(record["first_brake"]),
                "queue_depth": record["queue_depth"],
                "status": status,
            })
            self._writer.writerow(values)
            self._file.flush()

    def destroy(self):
        if not self.enabled:
            return
        with self._condition:
            self._stopping = True
            while self._queue:
                record = heapq.heappop(self._queue)[2]
                self._write_record(record, None, None, "DROPPED_SHUTDOWN")
            self._condition.notify_all()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._file:
            self._file.close()
        self._file = None
        self._writer = None

    def _info(self, message):
        if self.logger:
            self.logger.info(message)

    def _warning(self, message):
        if self.logger:
            self.logger.warning(message)
