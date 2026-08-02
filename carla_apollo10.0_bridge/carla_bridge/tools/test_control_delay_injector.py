#!/usr/bin/env python3
"""Unit tests for the SCB Bridge control delay path."""

import csv
import tempfile
import time
import unittest
from pathlib import Path

from carla_bridge.control_delay_injector import ControlDelayInjector


class _Header(object):
    def __init__(self):
        self.timestamp_sec = 0.0


class _Command(object):
    def __init__(self):
        self.header = _Header()
        self.throttle = 0.0
        self.brake = 0.0
        self.acceleration = 0.0
        self.steering_target = 0.0

    def CopyFrom(self, other):
        self.header.timestamp_sec = other.header.timestamp_sec
        self.throttle = other.throttle
        self.brake = other.brake
        self.acceleration = other.acceleration
        self.steering_target = other.steering_target


class ControlDelayInjectorTest(unittest.TestCase):
    def test_root_level_delay_config_is_normalized(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root_cfg = {
                "enabled": True,
                "delay_ms": 0.0,
                "log_dir": temp_dir,
            }
            parameters = {"carla": {}, "control_delay_injection": root_cfg}
            carla_parameters = ControlDelayInjector.normalize_bridge_parameters(
                parameters,
                bridge_entry_file="/test/main.py",
                settings_source_file="/test/settings.yaml",
            )
            self.assertIs(
                carla_parameters["control_delay_injection"], root_cfg
            )
            path = ControlDelayInjector.prepare_startup_evidence(
                carla_parameters
            )
            self.assertTrue(Path(path).is_file())

    def test_conflicting_delay_config_locations_are_rejected(self):
        parameters = {
            "carla": {"control_delay_injection": {"enabled": True}},
            "control_delay_injection": {"enabled": False},
        }
        with self.assertRaises(RuntimeError):
            ControlDelayInjector.normalize_bridge_parameters(parameters)

    def test_enabled_creates_header_before_trigger(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = {"control_delay_injection": {
                "enabled": True,
                "delay_ms": 0.0,
                "activation_speed_mps": 20.0,
                "brake_threshold_percentage": 1.0,
                "log_dir": temp_dir,
            }}
            injector = ControlDelayInjector(
                config,
                lambda command: None,
                lambda: 0.0,
            )
            injector.destroy()

            paths = list(Path(temp_dir).glob("*.csv"))
            self.assertEqual(len(paths), 1)
            with paths[0].open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["status"], "INITIALIZED")
            self.assertEqual(rows[0]["schema_version"], "scb_control_delay_v3")
            self.assertEqual(rows[0]["requested_delay_ms"], "0.0")

    def test_main_startup_creates_and_ego_reuses_same_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = {"control_delay_injection": {
                "enabled": True,
                "delay_ms": 0.0,
                "activation_speed_mps": 5.0,
                "brake_threshold_percentage": 1.0,
                "log_dir": temp_dir,
                "_bridge_entry_file": "/test/carla_bridge/main.py",
                "_settings_source_file": "/test/config/settings.yaml",
            }}
            path = ControlDelayInjector.prepare_startup_evidence(config)
            self.assertTrue(Path(path).is_file())

            injector = ControlDelayInjector(
                config,
                lambda command: None,
                lambda: 0.0,
            )
            injector.destroy()

            paths = list(Path(temp_dir).glob("*.csv"))
            self.assertEqual(len(paths), 1)
            with paths[0].open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(
                [row["status"] for row in rows],
                ["BRIDGE_CONFIG_LOADED", "INITIALIZED"],
            )
            self.assertEqual(rows[0]["schema_version"], "scb_control_delay_v3")
            self.assertEqual(rows[0]["bridge_entry_file"], "/test/carla_bridge/main.py")
            self.assertEqual(
                rows[0]["settings_source_file"], "/test/config/settings.yaml"
            )

    def test_activation_speed_is_latched_before_effective_brake(self):
        applied = []
        with tempfile.TemporaryDirectory() as temp_dir:
            config = {"control_delay_injection": {
                "enabled": True,
                "delay_ms": 0.0,
                "activation_speed_mps": 20.0,
                "brake_threshold_percentage": 1.0,
                "log_dir": temp_dir,
            }}
            injector = ControlDelayInjector(
                config,
                lambda command: applied.append(command.brake),
                lambda: 0.0,
            )
            injector.update_carla_time(100, 10.0, 22.2)
            injector.update_carla_time(101, 10.1, 18.0)

            brake = _Command()
            brake.brake = 5.0
            self.assertTrue(injector.submit(brake))
            deadline = time.monotonic() + 1.0
            while not applied and time.monotonic() < deadline:
                time.sleep(0.005)
            injector.destroy()

            self.assertEqual(applied, [5.0])
            paths = list(Path(temp_dir).glob("*.csv"))
            self.assertEqual(len(paths), 1)
            with paths[0].open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.DictReader(fh))
            applied_rows = [row for row in rows if row["status"] == "APPLIED"]
            self.assertEqual(len(applied_rows), 1)
            self.assertEqual(applied_rows[0]["first_effective_brake"], "1")
            self.assertEqual(float(applied_rows[0]["ego_speed_mps_at_receive"]), 18.0)

    def test_only_first_brake_and_later_commands_are_delayed(self):
        applied = []
        speed = [22.2]
        with tempfile.TemporaryDirectory() as temp_dir:
            config = {"control_delay_injection": {
                "enabled": True,
                "delay_ms": 35.0,
                "activation_speed_mps": 21.5,
                "brake_threshold_percentage": 1.0,
                "log_dir": temp_dir,
            }}
            injector = ControlDelayInjector(
                config,
                lambda command: applied.append((time.monotonic(), command.brake)),
                lambda: speed[0],
            )
            injector.update_carla_time(100, 10.0, speed[0])
            cruise = _Command()
            cruise.throttle = 10.0
            start = time.monotonic()
            self.assertFalse(injector.submit(cruise))
            self.assertEqual(len(applied), 1)

            brake = _Command()
            brake.header.timestamp_sec = 123.0
            brake.brake = 5.0
            self.assertTrue(injector.submit(brake))
            self.assertEqual(len(applied), 1)
            deadline = time.monotonic() + 1.0
            while len(applied) < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            injector.update_carla_time(101, 10.1, speed[0])
            injector.destroy()

            self.assertEqual(len(applied), 2)
            self.assertGreaterEqual((applied[1][0] - start) * 1000.0, 30.0)
            paths = list(Path(temp_dir).glob("*.csv"))
            self.assertEqual(len(paths), 1)
            with paths[0].open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.DictReader(fh))
            applied_rows = [row for row in rows if row["status"] == "APPLIED"]
            self.assertEqual(len(applied_rows), 1)
            row = applied_rows[0]
            self.assertEqual(row["first_effective_brake"], "1")
            self.assertGreaterEqual(float(row["actual_delay_ms"]), 30.0)
            self.assertLessEqual(
                int(row["release_monotonic_ns"]),
                int(row["apply_call_start_monotonic_ns"]),
            )
            self.assertLessEqual(
                int(row["apply_call_start_monotonic_ns"]),
                int(row["apply_call_end_monotonic_ns"]),
            )


if __name__ == "__main__":
    unittest.main()
