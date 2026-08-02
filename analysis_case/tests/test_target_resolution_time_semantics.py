from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collision_case_classifier import (
    CaseArtifacts,
    Config,
    LogAndTableParser,
    TargetResolver,
    _stable_to_t2,
    build_physical_target_chain,
)


class TargetResolutionTimeSemanticsTest(unittest.TestCase):
    def test_fusion_keeps_observation_and_output_times_separate(self) -> None:
        parser = object.__new__(LogAndTableParser)
        row = {
            "id": "1463",
            "type": 0,
            "obs_time": 9.9,
            "header_time": 10.8,
            "_log_time": 10.81,
            "pos_x": 1.0,
            "pos_y": 2.0,
        }
        result = parser._normalize_fusion_obs(row)
        self.assertEqual(9.9, result["time"])
        self.assertEqual(9.9, result["obs_time"])
        self.assertEqual(10.8, result["output_time"])
        self.assertEqual(10.81, result["log_time"])

    def test_ambiguous_carla_history_forbids_generic_fallback(self) -> None:
        config = Config()
        artifacts = CaseArtifacts()
        artifacts.fusion_obs = [{
            "id": "1463", "time": 9.9, "obs_time": 9.9,
            "output_time": 9.95, "type": "UNKNOWN",
            "pos_x": 100.0, "pos_y": 100.0,
        }]
        carla_match = {
            "available": True,
            "resolved": False,
            "warnings": ["CARLA_HISTORY_CANDIDATE_SCORE_OR_MARGIN_INSUFFICIENT"],
        }
        result = TargetResolver(artifacts, config, 10.0, carla_match).resolve(None)
        self.assertIsNone(result["selected_target_id"])
        self.assertIn("CARLA_HISTORY_AMBIGUOUS_NO_TARGET_FALLBACK", result["warnings"])

    def test_post_collision_output_is_not_pre_collision_detection(self) -> None:
        config = Config()
        artifacts = CaseArtifacts()
        artifacts.fusion_obs = [{
            "id": "1463", "time": 9.9, "obs_time": 9.9,
            "output_time": 10.8, "header_time": 10.8,
            "type": "UNKNOWN", "pos_x": 1.0, "pos_y": 2.0,
        }]
        chain = build_physical_target_chain("1463", artifacts, 10.0, config)
        self.assertEqual("FAIL", chain["verdict"])
        self.assertEqual("PERCEPTION_OUTPUT_AFTER_COLLISION", chain["reason_code"])

        resolver = TargetResolver(artifacts, config, 10.0, {}).resolve(None)
        self.assertIsNone(resolver["selected_target_id"])
        self.assertIn("TARGET_ID_UNRESOLVED", resolver["warnings"])

    def test_large_lateral_offset_cannot_win_fallback(self) -> None:
        config = Config()
        artifacts = CaseArtifacts()
        artifacts.fusion_obs = [
            {
                "id": "99", "time": 9.7 + index * 0.1,
                "obs_time": 9.7 + index * 0.1,
                "output_time": 9.75 + index * 0.1,
                "type": "VEHICLE", "pos_x": 30.0, "pos_y": 15.0,
                "rel_forward": 30.0, "rel_left": 15.0,
                "rel_distance": 33.5, "closing_speed": 10.0, "ttc": 3.0,
            }
            for index in range(2)
        ]
        result = TargetResolver(artifacts, config, 10.0, {}).resolve(None)
        self.assertIsNone(result["selected_target_id"])
        candidate = result["candidates"][0]
        self.assertEqual(0.0, candidate["score"])
        self.assertFalse(candidate["risk_evidence"]["lateral_gate_passed"])

    def test_stability_requires_multiple_output_frames(self) -> None:
        config = Config()
        one = [{"time": 9.8, "obs_time": 9.8, "output_time": 9.9}]
        two = one + [{"time": 9.9, "obs_time": 9.9, "output_time": 9.98}]
        self.assertFalse(_stable_to_t2(one, 10.0, config))
        self.assertTrue(_stable_to_t2(two, 10.0, config))


if __name__ == "__main__":
    unittest.main()
