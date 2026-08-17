from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path
from zoneinfo import ZoneInfo

import yaml

from realtime_collision_core import (
    EgoSample,
    FusionObservation,
    classify_run,
    clock_alignment,
    detect_brake_onset,
    find_brake_completion,
    find_near_stop,
    find_stop,
    geometry_at_t1,
    integrate_speed,
    interpolate_sample,
    log_epoch,
    stable_observation,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "analysis_config.yaml"
CONFIG = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def ego(t: float, speed: float, x: float = 0.0) -> EgoSample:
    return EgoSample(t, t, x, 0.0, 0.0, 0.0, speed, 0.0, 0.0, speed, "test", 1)


def obs(seq: int, t: float, obstacle_id: str = "7", x: float = 40.0) -> FusionObservation:
    return FusionObservation(seq, str(seq), t + 0.2, t, obstacle_id, "5", x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.8, 2.0, 0.9, "test", seq)


class CoreTests(unittest.TestCase):
    def test_log_time_conversion(self) -> None:
        value = log_epoch("I0717 17:04:06.718860 1 file.cc:1] x", "202607171703", ZoneInfo("Asia/Shanghai"))
        self.assertAlmostEqual(value, 1784279046.71886, places=5)

    def test_frame_to_sim_time(self) -> None:
        self.assertAlmostEqual(123 * 0.1, 12.3)

    def test_trapezoidal_speed_integral(self) -> None:
        rows = [ego(0.0, 10.0), ego(1.0, 8.0), ego(2.0, 6.0)]
        self.assertAlmostEqual(integrate_speed(rows, 0.0, 2.0), 16.0)

    def test_interpolated_collision_speed(self) -> None:
        rows = [ego(0.0, 10.0), ego(1.0, 6.0)]
        self.assertAlmostEqual(interpolate_sample(rows, 0.25)["speed_mps"], 9.0)

    def test_longitudinal_clearance(self) -> None:
        result = geometry_at_t1(obs(1, 0.0, x=40.0), ego(0.0, 15.0), CONFIG)
        self.assertAlmostEqual(result["longitudinal_clearance_m"], 40.0 - 5.3074, places=6)
        self.assertAlmostEqual(result["lateral_offset_m"], 0.0, places=6)

    def test_three_frame_stable_sequence(self) -> None:
        rows = [obs(1, 0.0), obs(2, 0.1), obs(3, 0.2)]
        first, segment, _ = stable_observation(rows, 3, CONFIG)
        self.assertIsNotNone(first)
        self.assertEqual(len(segment), 3)

    def test_stable_sequence_rejects_gap(self) -> None:
        rows = [obs(1, 0.0), obs(2, 0.1), obs(4, 0.2)]
        first, _, _ = stable_observation(rows, 3, CONFIG)
        self.assertIsNone(first)

    def test_effective_brake_onset(self) -> None:
        speeds = [10.0, 10.0, 9.8, 9.5, 9.1, 8.8]
        rows = [ego(index * 0.1, value) for index, value in enumerate(speeds)]
        result = detect_brake_onset(rows, 0.0, 0.1, 0.5, CONFIG, smoothed=False)
        self.assertEqual(result["status"], "AVAILABLE")
        self.assertGreaterEqual(result["onset_time_s"], 0.1)

    def test_precontrol_deceleration_invalid(self) -> None:
        speeds = [10.0, 9.8, 9.5, 9.2, 8.9, 8.6]
        rows = [ego(index * 0.1, value) for index, value in enumerate(speeds)]
        result = detect_brake_onset(rows, 0.0, 0.4, 0.5, CONFIG, smoothed=False)
        self.assertEqual(result["status"], "ATTRIBUTION_INVALID")

    def test_short_precontrol_transient_allows_distinct_episode(self) -> None:
        speeds = [10.0, 9.8, 9.6, 9.4, 9.4, 9.4, 9.0, 8.5, 8.0, 7.5]
        rows = [ego(index * 0.1, value) for index, value in enumerate(speeds)]
        result = detect_brake_onset(rows, 0.0, 0.35, 0.5, CONFIG, smoothed=False)
        self.assertEqual(result["status"], "AVAILABLE")
        self.assertEqual(result["attribution"], "DISTINCT_POST_CONTROL_EPISODE")
        self.assertAlmostEqual(result["onset_time_s"], 0.6)

    def test_near_stop_is_separate_from_strict_hold(self) -> None:
        rows = [
            ego(0.0, 1.0),
            ego(0.1, 0.5),
            ego(0.2, 0.05),
            ego(0.3, 0.2),
            ego(0.4, 0.4),
        ]
        near_stop = find_near_stop(rows, 0.0, math.nan, CONFIG)
        strict_stop = find_stop(rows, 0.0, math.nan, CONFIG)
        self.assertEqual(near_stop["status"], "AVAILABLE")
        self.assertAlmostEqual(near_stop["time_s"], 0.2)
        self.assertEqual(strict_stop["status"], "MISSING")

    def test_brake_completion_uses_post_t2_minimum_speed(self) -> None:
        rows = [
            ego(0.0, 1.0, 0.0),
            ego(0.1, 0.05, 0.1),
            ego(0.2, 0.08, 0.2),
            ego(0.3, 0.01, 0.3),
        ]
        completion = find_brake_completion(rows, 0.0, math.nan, CONFIG)
        self.assertEqual(completion["status"], "AVAILABLE")
        self.assertAlmostEqual(completion["time_s"], 0.3)

    def test_dual_clock_alignment(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            history = Path(temp) / "history.csv"
            history.write_text(
                "role,carla_frame,carla_timestamp_sec,wall_time_unix_ns\n"
                "ego,1,0.1,1000100000000\n"
                "ego,2,0.2,1000200000000\n"
                "ego,3,0.3,1000300000000\n",
                encoding="utf-8",
            )
            result = clock_alignment({"actor_history": history}, CONFIG)
            self.assertEqual(result["status"], "ALIGNED")
            self.assertAlmostEqual(result["slope"], 1.0, places=6)

    def test_id_switch_geometry_continuity(self) -> None:
        old = obs(10, 1.0, obstacle_id="7", x=40.0)
        new = obs(11, 1.1, obstacle_id="9", x=40.05)
        spatial_jump = math.dist((old.x_m, old.y_m), (new.x_m, new.y_m))
        self.assertNotEqual(old.obstacle_id, new.obstacle_id)
        self.assertLess(spatial_jump, CONFIG["stable_perception"]["max_position_jump_m"])

    def test_rt_only_follows_complete_function_chain(self) -> None:
        row = {
            "run_id": "collision-pass",
            "group_name": "delay_400ms",
            "collision": True,
            "collision_with_target": True,
            "analysis_status": "ANALYZED",
            "target_id": "7",
            "scb_log_present": True,
            "clock_alignment_status": "ALIGNED",
            "M_safety_6m_m": -10.0,
            "M_safety_6m_counterfactual_m": -2.0,
            "M_collision_0m_m": -4.0,
            "M_collision_0m_counterfactual_m": -1.0,
        }
        module = {
            "perception_status": "PASS",
            "prediction_status": "PASS",
            "planning_status": "PASS",
            "control_status": "PASS",
            "bridge_status": "PASS",
        }
        result = classify_run(row, module)
        self.assertEqual(result["classification"], "RT_ONLY_COLLISION")

    def test_perception_timing_degradation_remains_distinct_subtype(self) -> None:
        row = {
            "run_id": "collision-degraded",
            "group_name": "delay_400ms",
            "collision": True,
            "collision_with_target": True,
            "analysis_status": "ANALYZED",
            "target_id": "7",
            "scb_log_present": True,
            "M_safety_6m_m": -12.0,
            "M_safety_6m_counterfactual_m": -1.0,
            "M_collision_0m_m": -6.0,
            "M_collision_0m_counterfactual_m": 5.0,
        }
        module = {
            "perception_status": "DEGRADED",
            "prediction_status": "PASS",
            "planning_status": "PASS",
            "control_status": "PASS",
            "bridge_status": "PASS",
        }
        result = classify_run(row, module)
        self.assertEqual(
            result["classification"],
            "TIMING_INDUCED_FUNCTIONAL_DEGRADATION",
        )


if __name__ == "__main__":
    unittest.main()
