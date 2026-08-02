from __future__ import annotations

import sys
import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from implicit_deadline_analyzer import (
    deadline_model,
    detect_effective_brake_onset,
    load_brake_apply,
    resolve_epoch_reference,
    resolve_target,
    stable_segment,
)


class ImplicitDeadlineAnalyzerTest(unittest.TestCase):
    def test_80_kmh_200_ms_distance_and_two_deadlines(self) -> None:
        model = deadline_model(80.0, 10.0, 5.0, 0.0, None, 200.0)
        self.assertAlmostEqual(24.691358, model["brake_distance_m"], places=5)
        self.assertAlmostEqual(29.691358, model["d2_safety_m"], places=5)
        self.assertAlmostEqual(34.135802, model["desired_d1_m"], places=5)
        self.assertAlmostEqual(
            29.135802,
            model["desired_d1_for_collision_deadline_m"],
            places=5,
        )

        measured = deadline_model(80.0, 10.0, 5.0, 0.0, model["desired_d1_m"], None)
        self.assertAlmostEqual(200.0, measured["safety_deadline_ms"], places=5)
        self.assertAlmostEqual(425.0, measured["collision_deadline_ms"], places=5)

    def test_stable_onset_is_backdated_to_first_of_three_frames(self) -> None:
        rows = [
            {
                "time": 100.12 + index * 0.1,
                "output_time": 100.12 + index * 0.1,
                "obs_time": 100.00 + index * 0.1,
                "closing_speed": 22.2,
            }
            for index in range(3)
        ]
        result = stable_segment(rows, [], {
            "stable_perception_frames": 3,
            "stable_max_frame_gap_sec": 0.25,
            "target_search_min_ego_speed_mps": 20.0,
        })
        self.assertIsNotNone(result)
        self.assertAlmostEqual(100.0, result["stable_observation_time_sec"])
        self.assertAlmostEqual(100.12, result["stable_output_time_sec"])
        self.assertAlmostEqual(100.32, result["stable_confirmation_output_time_sec"])

    def test_low_speed_gate_derives_from_configured_experiment_speed(self) -> None:
        rows = [
            {
                "output_time": 100.0 + index * 0.1,
                "obs_time": 100.0 + index * 0.1,
                "closing_speed": 11.1,
            }
            for index in range(3)
        ]
        result = stable_segment(rows, [], {
            "speed_kmh": 40.0,
            "stable_perception_frames": 3,
            "stable_max_frame_gap_sec": 0.25,
            "target_search_min_ego_speed_mps": None,
            "target_search_min_ego_speed_ratio": 0.70,
        })
        self.assertIsNotNone(result)

    def test_target_at_expected_stable_distance_is_resolved(self) -> None:
        fusion = []
        ego = []
        for index in range(3):
            timestamp = 100.0 + index * 0.1
            fusion.append({
                "id": "145",
                "output_time": timestamp,
                "obs_time": timestamp,
                "rel_forward": 34.135802,
                "rel_left": 0.0,
                "speed": 0.0,
                "type": "VEHICLE",
            })
            ego.append({"time": timestamp, "ego_speed": 22.2})
        result = resolve_target(SimpleNamespace(fusion_obs=fusion, ego_states=ego), {
            "speed_kmh": 80.0,
            "stable_perception_frames": 3,
            "stable_max_frame_gap_sec": 0.25,
            "target_search_min_ego_speed_mps": None,
            "target_search_min_ego_speed_ratio": 0.70,
            "expected_stable_distance_m": 34.135802,
            "target_search_max_distance_m": 55.0,
            "target_lateral_tolerance_m": 3.0,
            "static_target_max_speed_mps": 2.0,
            "target_distance_score_scale_m": 30.0,
            "target_candidate_min_score": 0.65,
            "target_candidate_min_margin": 0.10,
        }, [])
        self.assertEqual(result["status"], "RESOLVED")
        self.assertEqual(result["target_id"], "145")

    def test_pre_t1_brake_is_not_reused_as_response(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scb_control_delay_test.csv"
            fields = [
                "first_effective_brake", "status",
                "apply_wall_time_unix_ns", "receive_wall_time_unix_ns",
            ]
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                writer.writerow({
                    "first_effective_brake": 1,
                    "status": "APPLIED",
                    "apply_wall_time_unix_ns": int(100.0 * 1e9),
                    "receive_wall_time_unix_ns": int(99.9 * 1e9),
                })
            result = load_brake_apply(Path(tmp), 101.0)
        self.assertEqual(result["status"], "MISSING")
        self.assertEqual(result["reason"], "NO_EFFECTIVE_BRAKE_AT_OR_AFTER_T1")

    def test_effective_brake_onset_uses_sustained_speed_drop(self) -> None:
        artifacts = SimpleNamespace(ego_states=[
            {"ego_source": "localization", "time": 100.0, "ego_speed": 10.0},
            {"ego_source": "localization", "time": 100.1, "ego_speed": 10.0},
            {"ego_source": "localization", "time": 100.2, "ego_speed": 9.9},
            {"ego_source": "localization", "time": 100.3, "ego_speed": 9.8},
        ])
        result = detect_effective_brake_onset(artifacts, 100.15, {
            "effective_brake_min_decel_mps2": 0.5,
            "effective_brake_consecutive_intervals": 2,
            "effective_brake_max_lag_sec": 1.0,
            "effective_brake_precheck_sec": 0.2,
        })
        self.assertEqual(result["status"], "AVAILABLE")
        self.assertAlmostEqual(result["effective_brake_onset_time_sec"], 100.2)

    def test_safe_run_parse_window_prefers_brake_end_over_spawn(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scb_control_delay_test.csv"
            fields = ["status", "apply_call_end_wall_time_unix_ns"]
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                writer.writerow({
                    "status": "APPLIED",
                    "apply_call_end_wall_time_unix_ns": int(1_700_000_010.0 * 1e9),
                })
            result = resolve_epoch_reference(
                Path(tmp),
                {"obstacle_spawn_wall_time_unix_ns": int(1_700_000_000.0 * 1e9)},
                post_event_sec=5.0,
            )
        self.assertAlmostEqual(result, 1_700_000_015.0)


if __name__ == "__main__":
    unittest.main()
