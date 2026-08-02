from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import main as combined_main


class MainPipelineTest(unittest.TestCase):
    def _functional_result(self, verdict: str) -> dict:
        return {
            "case_id": "case",
            "final_verdict": verdict,
            "reason_code": "test_reason",
            "target_id": "42",
        }

    def test_timing_is_skipped_when_function_is_not_normal(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case, out = root / "case", root / "out"
            case.mkdir()
            classifier = MagicMock()
            classifier.run.return_value = self._functional_result("PERCEPTION_ABNORMAL")
            with patch.object(combined_main.Config, "load", return_value=MagicMock()), patch.object(
                combined_main, "Classifier", return_value=classifier
            ), patch.object(combined_main, "run_timing_analysis") as timing:
                result = combined_main.run_pipeline(
                    case, out, root / "functional.yaml", root / "timing.yaml", True
                )
            timing.assert_not_called()
            self.assertFalse(result["timing_analysis"]["invoked"])
            self.assertEqual("SKIPPED_FUNCTION_NOT_NORMAL", result["timing_analysis"]["status"])
            self.assertFalse((out / "timing_frame_latencies.csv").exists())

    def test_timing_is_invoked_after_function_passes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case, out = root / "case", root / "out"
            case.mkdir()
            functional = self._functional_result(combined_main.FUNCTIONALLY_NORMAL_VERDICT)
            classifier = MagicMock()

            def run_classifier(*_args):
                out.mkdir(parents=True, exist_ok=True)
                (out / "classification_result.json").write_text(json.dumps(functional), encoding="utf-8")
                return functional

            classifier.run.side_effect = run_classifier
            timing_result = {
                "status": "FAIL",
                "implicit_deadline_miss": True,
                "e2e_anomaly_frame_count": 2,
                "output_files": {
                    "all_frames": "timing_frame_latencies.csv",
                    "anomaly_frames": "timing_anomaly_frames.csv",
                },
            }
            with patch.object(combined_main.Config, "load", return_value=MagicMock()), patch.object(
                combined_main, "Classifier", return_value=classifier
            ), patch.object(combined_main, "run_timing_analysis", return_value=timing_result) as timing:
                result = combined_main.run_pipeline(
                    case, out, root / "functional.yaml", root / "timing.yaml", True
                )
            timing.assert_called_once()
            self.assertTrue(result["timing_analysis"]["invoked"])
            self.assertTrue(result["timing_analysis"]["implicit_deadline_miss"])
            self.assertEqual(2, result["timing_analysis"]["e2e_anomaly_frame_count"])

    def test_physical_implicit_analysis_is_gated_by_function_result(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case, out = root / "case", root / "out"
            case.mkdir()
            classifier = MagicMock()
            classifier.run.return_value = self._functional_result("PLANNING_ABNORMAL")
            with patch.object(combined_main.Config, "load", return_value=MagicMock()), patch.object(
                combined_main, "Classifier", return_value=classifier
            ), patch.object(combined_main, "run_implicit_deadline_analysis") as implicit:
                result = combined_main.run_pipeline(
                    case,
                    out,
                    root / "functional.yaml",
                    root / "timing.yaml",
                    False,
                    enable_implicit_deadline_analysis=True,
                    implicit_deadline_config_path=root / "implicit.yaml",
                )
            implicit.assert_not_called()
            self.assertEqual(
                "SKIPPED_FUNCTION_NOT_NORMAL",
                result["implicit_deadline_analysis"]["status"],
            )

    def test_physical_implicit_analysis_runs_after_function_passes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case, out = root / "case", root / "out"
            case.mkdir()
            functional = self._functional_result(combined_main.FUNCTIONALLY_NORMAL_VERDICT)
            classifier = MagicMock()

            def run_classifier(*_args):
                out.mkdir(parents=True, exist_ok=True)
                (out / "classification_result.json").write_text(json.dumps(functional), encoding="utf-8")
                return functional

            classifier.run.side_effect = run_classifier
            implicit_result = {
                "status": "PASS",
                "implicit_safety_deadline_miss": True,
                "collision_deadline_miss": True,
                "causal_assessment": "REALTIME_INJECTION_CAUSAL_CANDIDATE_REQUIRES_GROUP_COMPARISON",
            }
            with patch.object(combined_main.Config, "load", return_value=MagicMock()), patch.object(
                combined_main, "Classifier", return_value=classifier
            ), patch.object(
                combined_main,
                "run_implicit_deadline_analysis",
                return_value=implicit_result,
            ) as implicit:
                result = combined_main.run_pipeline(
                    case,
                    out,
                    root / "functional.yaml",
                    root / "timing.yaml",
                    False,
                    enable_implicit_deadline_analysis=True,
                    implicit_deadline_config_path=root / "implicit.yaml",
                )
            implicit.assert_called_once()
            self.assertTrue(result["implicit_deadline_analysis"]["invoked"])
            self.assertTrue(result["implicit_deadline_analysis"]["collision_deadline_miss"])


if __name__ == "__main__":
    unittest.main()
