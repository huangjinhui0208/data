from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from timing_anomaly_detector import build_result, calculate_frame_latencies, write_outputs


def write_csv(path: Path, fieldnames, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class TimingAnomalyDetectorTest(unittest.TestCase):
    def test_frame_pairing_and_e2e_first_attribution(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            case = Path(tmp) / "case"
            events = case / "trace" / "events"
            context = case / "trace" / "message_context"
            log = case / "log"
            event_fields = ["trace_id", "module", "phase", "mono_ns"]
            context_fields = [
                "trace_id", "edge", "trace_valid", "primary_parent_trace_id",
                "data_ts_ns", "input_seq", "output_seq", "object_count",
            ]
            frames = [
                # Frame f1: E2E=40 ms, all deadlines pass.
                ("f1", "p1", 1_100_000_000, 0, 10, 12, 17, 20, 40),
                # Frame f2: E2E=90 ms and perception=60 ms -> perception cause.
                ("f2", "p2", 1_200_000_000, 100, 160, 165, 170, 175, 190),
                # Frame f3: E2E=70 ms, all modules pass -> handoff/accumulation.
                ("f3", "p3", 1_300_000_000, 200, 210, 230, 235, 260, 270),
            ]
            write_csv(
                context / "perception.multi_sensor_fusion.1.csv",
                context_fields,
                [
                    {
                        "trace_id": trace,
                        "edge": "out",
                        "trace_valid": 1,
                        "primary_parent_trace_id": parent,
                        "data_ts_ns": data_ts,
                        "input_seq": index,
                        "output_seq": index,
                        "object_count": 1,
                    }
                    for index, (trace, parent, data_ts, *_rest) in enumerate(frames, 1)
                ],
            )
            write_csv(
                events / "perception.pointcloud_preprocess.1.csv",
                event_fields,
                [
                    {"trace_id": parent, "module": "perception.pointcloud_preprocess", "phase": "proc_enter", "mono_ns": start * 1_000_000}
                    for _trace, parent, _data, start, *_rest in frames
                ],
            )
            write_csv(
                events / "perception.multi_sensor_fusion.1.csv",
                event_fields,
                [
                    {"trace_id": trace, "module": "perception.multi_sensor_fusion", "phase": "output_pub", "mono_ns": p_out * 1_000_000}
                    for trace, _parent, _data, _start, p_out, *_rest in frames
                ],
            )
            prediction_rows = []
            planning_rows = []
            for trace, _parent, _data, _start, _p_out, d_start, d_out, l_start, l_out in frames:
                prediction_rows.extend([
                    {"trace_id": trace, "module": "prediction", "phase": "proc_enter", "mono_ns": d_start * 1_000_000},
                    {"trace_id": trace, "module": "prediction", "phase": "output_pub", "mono_ns": d_out * 1_000_000},
                ])
                planning_rows.extend([
                    {"trace_id": trace, "module": "planning", "phase": "proc_enter", "mono_ns": l_start * 1_000_000},
                    {"trace_id": trace, "module": "planning", "phase": "output_pub", "mono_ns": l_out * 1_000_000},
                ])
            write_csv(events / "prediction.1.csv", event_fields, prediction_rows)
            write_csv(events / "planning.1.csv", event_fields, planning_rows)
            log.mkdir(parents=True, exist_ok=True)
            (log / "perception.log.INFO.test").write_text(
                "I [FUSION_OBS] trace_id=f2 id=42 type=5\n",
                encoding="utf-8",
            )

            thresholds = {"perception": 50.0, "prediction": 10.0, "planning": 40.0, "e2e": 50.0}
            rows, metadata = calculate_frame_latencies(case, 1.0, 1.4, thresholds, ["42"])
            self.assertEqual(3, len(rows))
            self.assertEqual(1.0, metadata["complete_frame_ratio"])
            self.assertFalse(rows[0]["e2e_deadline_miss"])
            self.assertEqual("perception", rows[1]["cause_modules"])
            self.assertTrue(rows[1]["target_present"])
            self.assertEqual("PIPELINE_ACCUMULATION_OR_HANDOFF", rows[2]["cause_modules"])

            window = {
                "t1": 1.0,
                "t2": 1.4,
                "functional_result_path": "synthetic",
                "functional_verdict": "PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING",
                "functional_reason_code": "FUNCTIONS_PASS_COLLISION_AFTER_PLANNING",
                "target_id": "42",
                "target_id_chain": ["42"],
            }
            result = build_result(case, window, thresholds, {"min_complete_frame_ratio": 0.95}, rows, metadata)
            self.assertEqual("FAIL", result["status"])
            self.assertTrue(result["implicit_deadline_miss"])
            self.assertEqual(2, result["e2e_anomaly_frame_count"])
            self.assertNotIn("threshold_source", result)
            worst = result["worst_e2e_frames"][0]
            self.assertIn("cause_modules_ms", worst)
            self.assertIn("cause_modules_overrun_ms", worst)
            self.assertIn("e2e_over_threshold_ms", worst)
            output = case / "out"
            write_outputs(output, rows, result)
            self.assertTrue((output / "timing_frame_latencies.csv").exists())
            with (output / "timing_frame_latencies.csv").open(encoding="utf-8-sig") as fh:
                self.assertEqual(3, len(list(csv.DictReader(fh))))
            with (output / "timing_anomaly_frames.csv").open(encoding="utf-8-sig") as fh:
                self.assertEqual(2, len(list(csv.DictReader(fh))))
            scatter = output / "timing_e2e_scatter.svg"
            self.assertTrue(scatter.exists())
            self.assertIn('stroke="#d62728"', scatter.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
