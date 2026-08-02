#!/usr/bin/env python3

"""Regression tests for GNSS velocity and decision-chain topic capture."""

import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[4]
BRIDGE_ROOT = REPO_ROOT / "modules/carla_apollo10.0_bridge"
NEO_PYTHON = Path("/opt/apollo/neo/python")
for path in (str(BRIDGE_ROOT), str(NEO_PYTHON), str(REPO_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

sys.modules.setdefault("carla", types.ModuleType("carla"))

import carla_bridge  # noqa: E402

utils_module = types.ModuleType("carla_bridge.utils")
utils_module.__path__ = [str(BRIDGE_ROOT / "carla_bridge/utils")]
sys.modules.setdefault("carla_bridge.utils", utils_module)
carla_bridge.utils = utils_module


class StubSensor:
    pass


class StubGnssFrameBundle:
    def __init__(self, best_pose, odometry, heading, status, publish_cb):
        self.best_pose = best_pose
        self.odometry = odometry
        self.heading = heading
        self.status = status
        self.publish_cb = publish_cb


class StubGnssImuFrameCoordinator:
    @staticmethod
    def get_instance():
        return None


sensor_module = types.ModuleType("carla_bridge.sensor.sensor")
sensor_module.Sensor = StubSensor
sys.modules.setdefault("carla_bridge.sensor.sensor", sensor_module)

coordinator_module = types.ModuleType(
    "carla_bridge.utils.gnss_imu_frame_coordinator")
coordinator_module.GnssFrameBundle = StubGnssFrameBundle
coordinator_module.GnssImuFrameCoordinator = StubGnssImuFrameCoordinator
sys.modules.setdefault(
    "carla_bridge.utils.gnss_imu_frame_coordinator", coordinator_module)

transforms_module = types.ModuleType("carla_bridge.utils.transforms")
transforms_module.carla_rotation_to_rpy = lambda _rotation: (0.0, 0.0, 0.0)
sys.modules.setdefault("carla_bridge.utils.transforms", transforms_module)

from modules.common_msgs.localization_msgs.localization_pb2 import (  # noqa: E402
    LocalizationEstimate,
)
from modules.common_msgs.localization_msgs.pose_pb2 import Pose  # noqa: E402
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis  # noqa: E402

from carla_bridge.sensor.gnss import Gnss  # noqa: E402
from carla_bridge.tools import collect_lidar_ped_decision_chain as collector  # noqa: E402


class FakeCoordinator:
    def __init__(self):
        self.bundle = None

    def submit_gnss(self, _frame, bundle):
        self.bundle = bundle


class FakeParent:
    def __init__(self, linear=(4.0, -2.0, 0.5)):
        self.linear = linear

    def get_current_cyber_pose(self):
        pose = Pose()
        pose.position.x = 10.0
        pose.position.y = -3.0
        pose.orientation.qw = 1.0
        return pose

    def get_current_cyber_twist(self):
        return SimpleNamespace(
            linear=SimpleNamespace(
                x=self.linear[0], y=self.linear[1], z=self.linear[2]),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )


class FakeCyberNode:
    def __init__(self, name):
        self.name = name
        self.readers = {}

    def create_reader(self, topic, _message_type, callback):
        self.readers[topic] = callback
        return callback


class FakeCyber:
    def __init__(self):
        self.initialized = False
        self.node = None

    def init(self):
        self.initialized = True

    def Node(self, name):
        self.node = FakeCyberNode(name)
        return self.node


class DecisionChainFixesTest(unittest.TestCase):
    def test_gnss_odometry_contains_world_linear_velocity(self):
        gnss = Gnss.__new__(Gnss)
        gnss.node = SimpleNamespace(get_time=lambda frame_id=None: 123.5)
        gnss.parent = FakeParent()
        gnss.carla_actor = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(
                rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)))
        gnss.frame_coordinator = FakeCoordinator()

        gnss.sensor_data_updated(SimpleNamespace(
            frame=42, latitude=1.0, longitude=2.0, altitude=3.0))

        pose = gnss.frame_coordinator.bundle.odometry.localization
        self.assertTrue(pose.HasField("linear_velocity"))
        self.assertEqual((pose.linear_velocity.x,
                          pose.linear_velocity.y,
                          pose.linear_velocity.z),
                         (4.0, -2.0, 0.5))

    def test_gnss_odometry_marks_stationary_linear_velocity_present(self):
        gnss = Gnss.__new__(Gnss)
        gnss.node = SimpleNamespace(get_time=lambda frame_id=None: 123.5)
        gnss.parent = FakeParent(linear=(0.0, 0.0, 0.0))
        gnss.carla_actor = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(
                rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)))
        gnss.frame_coordinator = FakeCoordinator()

        gnss.sensor_data_updated(SimpleNamespace(
            frame=42, latitude=1.0, longitude=2.0, altitude=3.0))

        pose = gnss.frame_coordinator.bundle.odometry.localization
        self.assertTrue(pose.HasField("linear_velocity"))
        self.assertEqual((pose.linear_velocity.x,
                          pose.linear_velocity.y,
                          pose.linear_velocity.z),
                         (0.0, 0.0, 0.0))

    def test_typed_capture_uses_one_node_and_counts_messages(self):
        fake_cyber = FakeCyber()
        topic_specs = [
            ("localization", "/apollo/localization/pose", LocalizationEstimate),
            ("chassis", "/apollo/canbus/chassis", Chassis),
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            capture = collector.TypedTopicCapture(
                Path(temp_dir), fake_cyber, topic_specs=topic_specs)
            capture.start()

            message = LocalizationEstimate()
            message.header.timestamp_sec = 123.5
            fake_cyber.node.readers["/apollo/localization/pose"](message)
            chassis = Chassis()
            chassis.header.timestamp_sec = 123.5
            fake_cyber.node.readers["/apollo/canbus/chassis"](chassis)
            counts = capture.close()

            output = Path(temp_dir, "echo_localization.txt").read_text(
                encoding="utf-8")

        self.assertTrue(fake_cyber.initialized)
        self.assertTrue(fake_cyber.node.name.startswith(
            "carla_ped_decision_chain_"))
        self.assertEqual(len(fake_cyber.node.readers), 2)
        self.assertEqual(counts, {"localization": 1, "chassis": 1})
        self.assertIn("header {", output)
        self.assertNotIn("listener_node_echo", output)
        self.assertEqual(len(collector.split_echo_messages(output)), 1)

    def test_summary_marks_zero_message_topics_failed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            collector.write_skipped_live_outputs(output_dir)
            args = collector.build_arg_parser().parse_args([
                "--skip-live-capture",
                "--output-dir",
                temp_dir,
            ])

            collector.summarize_all(
                output_dir, {"skip_live_capture": 0}, {}, args)
            summary = Path(temp_dir, "decision_chain_summary.txt").read_text(
                encoding="utf-8")

        self.assertIn(
            "perception_obstacles: 0 (FAILED: zero messages)", summary)
        self.assertIn("localization: 0 (FAILED: zero messages)", summary)


if __name__ == "__main__":
    unittest.main()
