#!/usr/bin/env python
#
# Copyright (c) 2023 synkrotron.ai
#
import time
import os
import sys
import threading
from threading import Event, Thread

import carla
import yaml
from cyber.proto.clock_pb2 import Clock

from carla_bridge.collision_sensor_logger import CollisionSensorLogger
from carla_bridge.control_delay_injector import ControlDelayInjector
from carla_bridge.core.actor_factory import ActorFactory
from carla_bridge.core.carla_spawn_objects import CarlaSpawnObjects
from carla_bridge.core.node import CyberNode
from carla_bridge.utils.logurus import init_log


sys.path.append("../")


def _prepare_scb_control_delay_evidence(parameters, log, settings_file=None):
    """Add SCB startup evidence without replacing the original main flow."""
    carla_parameters = parameters["carla"]
    nested_cfg = carla_parameters.get("control_delay_injection")
    root_cfg = parameters.get("control_delay_injection")
    if nested_cfg is not None and root_cfg is not None and nested_cfg != root_cfg:
        raise RuntimeError(
            "Conflicting control_delay_injection blocks exist at YAML root "
            "and under carla; keep exactly one"
        )
    if nested_cfg is None and root_cfg is not None:
        carla_parameters["control_delay_injection"] = root_cfg
        nested_cfg = root_cfg
        message = (
            "SCB config found at YAML root; normalized to "
            "carla.control_delay_injection"
        )
        print(message, flush=True)
        if log:
            log.warning(message)

    delay_cfg = nested_cfg or {}
    delay_cfg["_bridge_entry_file"] = os.path.abspath(__file__)
    if settings_file:
        delay_cfg["_settings_source_file"] = os.path.abspath(settings_file)

    prepare = getattr(ControlDelayInjector, "prepare_startup_evidence", None)
    if not callable(prepare):
        message = (
            "SCB injector version mismatch: {} does not provide "
            "prepare_startup_evidence; deploy control_delay_injector.py from "
            "the same revision as main.py"
        ).format(os.path.abspath(sys.modules[ControlDelayInjector.__module__].__file__))
        print(message, flush=True)
        if log:
            log.error(message)
        return None
    return prepare(carla_parameters, log)


class CarlaCyberBridge(CyberNode):

    # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
    # wait for this time until a next tick is triggered.
    VEHICLE_CONTROL_TIMEOUT = 1.0

    def __init__(self):
        super().__init__("cyber_bridge_node")
        self.spawn_objects_node = None
        self.timestamp = None
        self._registered_actors = None
        self.on_tick_id = None
        self.synchronous_mode_update_thread = None
        self.clock_writer = None
        self.actor_factory = None
        self.sync_mode = None
        self.carla_settings = None
        self.shutdown = None
        self.carla_world = None
        self.carla_parameters = None
        self.log = None
        self.collision_sensor_logger = None

    def initialize_bridge(self, carla_world, params, log):
        """
        Initialize the bridge
        """
        self.log = log
        self.carla_parameters = params["carla"]
        _prepare_scb_control_delay_evidence(params, log)
        self.carla_world = carla_world
        self.configure_time_sync(
            enable_frame_lock=self.carla_parameters.get("enable_frame_lock", True)
        )
        self.collision_sensor_logger = CollisionSensorLogger(
            world=carla_world,
            config=self.carla_parameters,
            logger=log,
            node=self,
        )

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()

        self.carla_settings = carla_world.get_settings()
        if not self.carla_parameters["passive"]:
            # workaround: settings can only applied within non-sync mode
            if self.carla_settings.synchronous_mode:
                self.carla_settings.synchronous_mode = False
                carla_world.apply_settings(self.carla_settings)

            self.carla_settings.synchronous_mode = self.carla_parameters[
                "synchronous_mode"
            ]

            self.carla_settings.fixed_delta_seconds = self.carla_parameters[
                "fixed_delta_seconds"
            ]
            carla_world.apply_settings(self.carla_settings)

        self.sync_mode = (
            self.carla_settings.synchronous_mode
            and not self.carla_parameters["passive"]
        )

        # actor factory
        self.actor_factory = ActorFactory(self, carla_world, self.log, self.sync_mode)

        # Communication topics
        self.clock_writer = self.new_writer("/clock", Clock, 10)

        if self.sync_mode:
            self.synchronous_mode_update_thread = Thread(
                target=self._synchronous_mode_update
            )
            self.synchronous_mode_update_thread.daemo = True
            self.synchronous_mode_update_thread.start()
        self._registered_actors = []

        carla_spawn_objects_thread = threading.Thread(
            target=self.carla_spawn_objects
        )
        carla_spawn_objects_thread.daemo = True
        carla_spawn_objects_thread.start()
        self.spin()

    def carla_spawn_objects(self):
        self.spawn_objects_node = CarlaSpawnObjects(self)
        self.spawn_objects_node.spawn_objects()
        self.spawn_objects_node.bridge_node.spin()

    def spawn_object(self, req):
        if not self.shutdown.is_set():
            id_ = self.actor_factory.spawn_actor(req)
            self._registered_actors.append(id_)

    def destroy_object(self, id_):
        destroyed_actors = self.actor_factory.destroy_actor(id_)
        success = bool(destroyed_actors)
        for actor in destroyed_actors:
            if actor in self._registered_actors:
                self._registered_actors.remove(actor)
        return success

    def _synchronous_mode_update(self):
        """
        execution loop for synchronous mode
        """
        # while not self.shutdown.is_set():
        #     frame = self.carla_world.tick()

        #     world_snapshot = self.carla_world.get_snapshot()
        #     self.on_world_tick(frame, world_snapshot.timestamp.elapsed_seconds)

        #     self.update_clock(world_snapshot.timestamp)
        #     self._update(frame, world_snapshot.timestamp.elapsed_seconds)
        #     self.actor_factory.update_available_objects()
        fixed_delta_seconds = float(
            self.carla_parameters.get("fixed_delta_seconds", 0.0) or 0.0
        )
        next_tick_wall_time = time.monotonic()

        while not self.shutdown.is_set():
            if fixed_delta_seconds > 0.0:
                now = time.monotonic()
                sleep_time = next_tick_wall_time - now
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            frame = self.carla_world.tick()

            world_snapshot = self.carla_world.get_snapshot()
            self.on_world_tick(frame, world_snapshot.timestamp.elapsed_seconds)

            self.update_clock(world_snapshot.timestamp)
            self._update(frame, world_snapshot.timestamp.elapsed_seconds)
            self.actor_factory.update_available_objects()

            # Capture only after the current frame's Apollo messages have been
            # dispatched.  The logger reuses this already-fetched snapshot, so
            # it adds neither CARLA RPCs nor Apollo/Cyber traffic.
            if self.collision_sensor_logger:
                self.collision_sensor_logger.capture_snapshot(
                    world_snapshot=world_snapshot,
                    frame_id=frame,
                    frame_wall_ns=self.get_clock_ns(frame_id=frame),
                    actor_factory=self.actor_factory,
                )

            if fixed_delta_seconds > 0.0:
                next_tick_wall_time += fixed_delta_seconds
                now = time.monotonic()
                if next_tick_wall_time < now - fixed_delta_seconds:
                    next_tick_wall_time = now

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        self.actor_factory.update_actor_states(frame_id, timestamp)

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if self.ok():
            _ = carla_timestamp
            self.clock_writer.write(Clock(clock=self.get_clock_ns()))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.log.info("Shutting down...")
        if not self.sync_mode:
            if self.on_tick_id:
                self.carla_world.remove_on_tick(self.on_tick_id)
        else:
            self.synchronous_mode_update_thread.join()
        self.log.info("Object update finished.")
        if self.collision_sensor_logger:
            self.collision_sensor_logger.destroy()
        for uid in self._registered_actors:
            self.actor_factory.destroy_actor(uid)
        self.actor_factory.update_available_objects()
        self.actor_factory.clear()
        super().destroy()


def main():
    """
    main function for carla simulator Cyber bridge
    maintaining the communication client and the CarlaBridge object
    """
    log = init_log("bridge.log")
    carla_client = None
    carla_bridge = CarlaCyberBridge()

    config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/settings.yaml"
    with open(config_file, encoding="utf-8") as f:
        parameters = yaml.safe_load(f)
    carla_parameters = parameters["carla"]
    # Fail fast before connecting to CARLA.  When enabled=True, a Bridge run
    # without its SCB evidence file is invalid and must not continue silently.
    _prepare_scb_control_delay_evidence(parameters, log, config_file)

    log.info(
        f"Trying to connect to {carla_parameters['host']}:{carla_parameters['port']}"
    )

    try:
        carla_client = carla.Client(
            host=carla_parameters["host"], port=carla_parameters["port"]
        )
        carla_client.set_timeout(carla_parameters["timeout"])

        carla_client.load_world(carla_parameters["town"])
        carla_world = carla_client.get_world()

        log.info(f"Connect to {carla_parameters['host']}:{carla_parameters['port']} successfully.")

        carla_bridge.initialize_bridge(carla_world, parameters, log)

    except (IOError, RuntimeError) as e:
        log.error(f"Error: {e}")
    except KeyboardInterrupt as e:
        log.error(f"Error: {e}")
    except Exception as e:  # pylint: disable=W0718
        log.error(e)
    finally:
        carla_world = carla_client.get_world()
        settings = carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        carla_world.apply_settings(settings)
        log.warning("Shutting down.")
        if carla_bridge.shutdown :
            carla_bridge.shutdown.set()
        carla_bridge.destroy()
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
