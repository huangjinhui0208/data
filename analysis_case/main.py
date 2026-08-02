#!/usr/bin/env python3
"""Run functional classification, explicit timing, and physical implicit deadline analysis."""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional

from collision_case_classifier import Classifier, Config, json_safe
from implicit_deadline_analyzer import run_implicit_deadline_analysis
from timing_anomaly_detector import run_timing_analysis


FUNCTIONALLY_NORMAL_VERDICT = "PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING"
TIMING_OUTPUT_NAMES = {
    "timing_frame_latencies.csv",
    "timing_anomaly_frames.csv",
    "timing_analysis_result.json",
    "timing_e2e_scatter.svg",
}
IMPLICIT_OUTPUT_NAMES = {
    "implicit_deadline_result.json",
    "implicit_deadline_trace_frames.csv",
}


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(json_safe(value), ensure_ascii=False, indent=2), encoding="utf-8")


def _clear_timing_outputs(out_dir: Path) -> None:
    for name in TIMING_OUTPUT_NAMES | IMPLICIT_OUTPUT_NAMES:
        path = out_dir / name
        if path.exists():
            path.unlink()


def _timing_not_run(status: str, functional_result: Dict[str, Any], reason: str) -> Dict[str, Any]:
    return {
        "case_id": functional_result.get("case_id"),
        "status": status,
        "explicit_deadline_threshold_miss": None,
        "implicit_deadline_miss": None,
        "reason": reason,
        "functional_context": {
            "verdict": functional_result.get("final_verdict"),
            "reason_code": functional_result.get("reason_code"),
            "target_id": functional_result.get("target_id"),
        },
        "output_files": {
            "all_frames": None,
            "anomaly_frames": None,
            "summary": "timing_analysis_result.json",
            "e2e_scatter": None,
        },
    }


def _implicit_not_run(status: str, functional_result: Dict[str, Any], reason: str) -> Dict[str, Any]:
    return {
        "case_id": functional_result.get("case_id"),
        "status": status,
        "implicit_safety_deadline_miss": None,
        "collision_deadline_miss": None,
        "causal_assessment": "NOT_RUN",
        "reason": reason,
        "functional_context": {
            "verdict": functional_result.get("final_verdict"),
            "reason_code": functional_result.get("reason_code"),
            "target_id": functional_result.get("target_id"),
        },
        "output_files": {
            "summary": "implicit_deadline_result.json",
            "trace_frames": "implicit_deadline_trace_frames.csv",
        },
    }


def run_pipeline(
    case_dir: Path,
    out_dir: Path,
    functional_config_path: Path,
    timing_config_path: Path,
    enable_timing_check: bool,
    target_id: Optional[str] = None,
    collision_time: Optional[float] = None,
    max_window_sec: Optional[float] = None,
    threshold_overrides: Optional[Dict[str, Optional[float]]] = None,
    enable_implicit_deadline_analysis: bool = False,
    implicit_deadline_config_path: Optional[Path] = None,
    calibration_path: Optional[Path] = None,
) -> Dict[str, Any]:
    case_dir = case_dir.resolve()
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    _clear_timing_outputs(out_dir)

    config = Config.load(functional_config_path)
    if max_window_sec is not None:
        config.analysis["pre_collision_window_sec"] = float(max_window_sec)
    functional_result = Classifier(case_dir, out_dir, config).run(target_id, collision_time)
    functional_result_path = out_dir / "classification_result.json"
    functionally_normal = functional_result.get("final_verdict") == FUNCTIONALLY_NORMAL_VERDICT

    timing_invoked = False
    if not enable_timing_check:
        timing_result = _timing_not_run("DISABLED", functional_result, "Timing check was not enabled by --enable-timing-check.")
        _write_json(out_dir / "timing_analysis_result.json", timing_result)
    elif not functionally_normal:
        timing_result = _timing_not_run(
            "SKIPPED_FUNCTION_NOT_NORMAL",
            functional_result,
            "Timing analysis is only invoked after all three functional checks pass.",
        )
        _write_json(out_dir / "timing_analysis_result.json", timing_result)
    else:
        timing_invoked = True
        timing_result = run_timing_analysis(
            case_dir,
            out_dir,
            functional_result_path,
            timing_config_path,
            threshold_overrides,
        )

    implicit_invoked = False
    if not enable_implicit_deadline_analysis:
        implicit_result = _implicit_not_run(
            "DISABLED", functional_result,
            "Implicit physical deadline analysis was not enabled.",
        )
        _write_json(out_dir / "implicit_deadline_result.json", implicit_result)
        (out_dir / "implicit_deadline_trace_frames.csv").write_text("", encoding="utf-8")
    elif not functionally_normal:
        implicit_result = _implicit_not_run(
            "SKIPPED_FUNCTION_NOT_NORMAL", functional_result,
            "Physical implicit-deadline attribution is only run after perception, prediction, and planning pass.",
        )
        _write_json(out_dir / "implicit_deadline_result.json", implicit_result)
        (out_dir / "implicit_deadline_trace_frames.csv").write_text("", encoding="utf-8")
    else:
        implicit_invoked = True
        implicit_result = run_implicit_deadline_analysis(
            case_dir,
            out_dir,
            implicit_deadline_config_path or Path(__file__).with_name("implicit_deadline_config.yaml"),
            functional_result_path,
            functional_config_path,
            calibration_path,
            target_id,
        )

    combined = {
        "case_id": case_dir.name,
        "functional_analysis": {
            "verdict": functional_result.get("final_verdict"),
            "reason_code": functional_result.get("reason_code"),
            "target_id": functional_result.get("target_id"),
            "functionally_normal": functionally_normal,
            "result_file": "classification_result.json",
        },
        "timing_analysis": {
            "enabled": enable_timing_check,
            "invoked": timing_invoked,
            "status": timing_result.get("status"),
            "explicit_deadline_threshold_miss": timing_result.get(
                "explicit_deadline_threshold_miss",
                timing_result.get("implicit_deadline_miss"),
            ),
            "implicit_deadline_miss": timing_result.get("implicit_deadline_miss"),
            "e2e_anomaly_frame_count": timing_result.get("e2e_anomaly_frame_count"),
            "result_file": "timing_analysis_result.json",
            "all_frames_file": timing_result.get("output_files", {}).get("all_frames"),
            "anomaly_frames_file": timing_result.get("output_files", {}).get("anomaly_frames"),
            "e2e_scatter_file": timing_result.get("output_files", {}).get("e2e_scatter"),
        },
        "implicit_deadline_analysis": {
            "enabled": enable_implicit_deadline_analysis,
            "invoked": implicit_invoked,
            "status": implicit_result.get("status"),
            "implicit_safety_deadline_miss": implicit_result.get("implicit_safety_deadline_miss"),
            "collision_deadline_miss": implicit_result.get("collision_deadline_miss"),
            "causal_assessment": implicit_result.get("causal_assessment"),
            "result_file": "implicit_deadline_result.json",
            "trace_frames_file": "implicit_deadline_trace_frames.csv",
        },
    }
    _write_json(out_dir / "main_result.json", combined)
    return combined


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Functional collision classifier plus explicit timing and physical implicit-deadline analysis")
    parser.add_argument("--case-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument(
        "--functional-config",
        type=Path,
        default=Path(__file__).with_name("collision_classifier_config.yaml"),
    )
    parser.add_argument(
        "--timing-config",
        type=Path,
        default=Path(__file__).with_name("timing_threshold_config.yaml"),
    )
    parser.add_argument("--enable-timing-check", action="store_true")
    parser.add_argument("--enable-implicit-deadline-analysis", action="store_true")
    parser.add_argument(
        "--implicit-deadline-config",
        type=Path,
        default=Path(__file__).with_name("implicit_deadline_config.yaml"),
    )
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--target-id", type=str)
    parser.add_argument("--collision-time", type=float)
    parser.add_argument("--max-window-sec", type=float)
    parser.add_argument("--perception-threshold-ms", type=float)
    parser.add_argument("--prediction-threshold-ms", type=float)
    parser.add_argument("--planning-threshold-ms", type=float)
    parser.add_argument("--e2e-threshold-ms", type=float)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")
    overrides = {
        "perception": args.perception_threshold_ms,
        "prediction": args.prediction_threshold_ms,
        "planning": args.planning_threshold_ms,
        "e2e": args.e2e_threshold_ms,
    }
    try:
        result = run_pipeline(
            args.case_dir,
            args.out_dir,
            args.functional_config,
            args.timing_config,
            args.enable_timing_check,
            args.target_id,
            args.collision_time,
            args.max_window_sec,
            overrides,
            args.enable_implicit_deadline_analysis,
            args.implicit_deadline_config,
            args.calibration,
        )
    except Exception as exc:
        logging.error("Combined analysis failed: %s", exc)
        return 2
    logging.info(
        "Functional=%s Timing=%s explicit_threshold_miss=%s SCB=%s",
        result["functional_analysis"]["verdict"],
        result["timing_analysis"]["status"],
        result["timing_analysis"]["explicit_deadline_threshold_miss"],
        result["implicit_deadline_analysis"]["causal_assessment"],
    )
    logging.info("Combined outputs written to %s", args.out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
