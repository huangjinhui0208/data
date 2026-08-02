#!/usr/bin/env python3
"""Aggregate repeated no-injection runs into an SCB calibration result."""

from __future__ import annotations

import argparse
import csv
import json
import logging
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from collision_case_classifier import json_safe, to_float
from implicit_deadline_analyzer import (
    deadline_model,
    load_deadline_config,
    load_experiment_metadata,
    percentile,
    run_implicit_deadline_analysis,
)


def _discover_cases(root: Path, group: str, explicit: Sequence[Path]) -> List[Path]:
    if explicit:
        return sorted({path.resolve() for path in explicit})
    cases = []
    for path in sorted(root.iterdir() if root.exists() else []):
        if not path.is_dir():
            continue
        metadata = load_experiment_metadata(path)
        if str(metadata.get("group") or "") == group:
            cases.append(path.resolve())
    return cases


def _find_functional_result(case_dir: Path) -> Optional[Path]:
    candidates = [
        case_dir / "analysis" / "classification_result.json",
        case_dir / "scb_analysis" / "classification_result.json",
        case_dir / "classification_result.json",
    ]
    return next((path for path in candidates if path.exists()), None)


def _median(values: Sequence[Optional[float]]) -> Optional[float]:
    usable = [float(value) for value in values if value is not None]
    return statistics.median(usable) if usable else None


def calibrate(
    cases_root: Path,
    out_dir: Path,
    config_path: Path,
    functional_config_path: Path,
    case_dirs: Sequence[Path],
    group: str = "baseline",
) -> Dict[str, Any]:
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = _discover_cases(cases_root.resolve(), group, case_dirs)
    cfg = load_deadline_config(config_path)
    rows: List[Dict[str, Any]] = []
    for case_dir in cases:
        case_out = case_dir / "scb_analysis"
        metadata = load_experiment_metadata(case_dir)
        result = run_implicit_deadline_analysis(
            case_dir,
            case_out,
            config_path,
            _find_functional_result(case_dir),
            functional_config_path,
            None,
            str(metadata.get("apollo_target_id")) if metadata.get("apollo_target_id") else None,
        )
        stable = result.get("stable_perception") or {}
        physical = result.get("physical_response") or {}
        trace = result.get("apollo_trace_internal") or {}
        trace_e2e = ((trace.get("statistics_ms") or {}).get("e2e") or {})
        deceleration = result.get("measured_deceleration") or {}
        checks = result.get("condition_checks") or {}
        semantic = result.get("calibration_semantic_chain") or {}
        requested_delay = to_float((result.get("brake_application") or {}).get("requested_delay_ms"))
        valid = (
            result.get("status") == "ANALYZED"
            and result.get("collision", {}).get("occurred") is False
            and checks.get("speed_matches_config") is True
            and checks.get("target_static") is True
            and checks.get("bridge_injection_evidence_present") is True
            and checks.get("brake_threshold_matches") is True
            and checks.get("bridge_and_analyzer_brake_threshold_match") is True
            and checks.get("effective_brake_onset_present") is True
            and checks.get("d1_reference_verified_for_physical_model") is True
            and checks.get("trace_complete_ratio_sufficient") is True
            and semantic.get("complete") is True
            and requested_delay is not None
            and abs(requested_delay) <= 1.0
        )
        rows.append({
            "case_id": case_dir.name,
            "valid_for_calibration": valid,
            "invalid_reason": "" if valid else "see_case_result",
            "collision": result.get("collision", {}).get("occurred"),
            "target_id": stable.get("target_id"),
            "speed_kmh": stable.get("ego_speed_kmh"),
            "d1_m": stable.get("d1_m"),
            "stable_detection_lag_ms": stable.get("stable_detection_lag_ms"),
            "stable_detection_lag_distance_m": stable.get("stable_detection_lag_distance_m"),
            "internal_e2e_median_ms": trace_e2e.get("median_ms"),
            "internal_e2e_p90_ms": trace_e2e.get("p90_ms"),
            "physical_response_ms": physical.get("stable_observation_to_brake_apply_ms"),
            "api_response_ms": physical.get("stable_observation_to_carla_api_return_ms"),
            "effective_brake_onset_status": (result.get("effective_brake_onset") or {}).get("status"),
            "stable_to_control_receive_ms": physical.get("stable_observation_to_control_receive_ms"),
            "requested_delay_ms": requested_delay,
            "actual_delay_ms": (result.get("brake_application") or {}).get("actual_delay_ms"),
            "effective_decel_mps2": deceleration.get("recommended_run_decel_mps2"),
            "semantic_chain_complete": semantic.get("complete"),
            "result_file": str((case_out / "implicit_deadline_result.json").resolve()),
        })

    valid_rows = [row for row in rows if row["valid_for_calibration"]]
    physical_values = [to_float(row["physical_response_ms"]) for row in valid_rows]
    physical_values = [value for value in physical_values if value is not None]
    internal_medians = [to_float(row["internal_e2e_median_ms"]) for row in valid_rows]
    internal_medians = [value for value in internal_medians if value is not None]
    d1_values = [to_float(row["d1_m"]) for row in valid_rows]
    d1_values = [value for value in d1_values if value is not None]
    speed_values = [to_float(row["speed_kmh"]) for row in valid_rows]
    speed_values = [value for value in speed_values if value is not None]
    lag_values = [to_float(row["stable_detection_lag_ms"]) for row in valid_rows]
    lag_values = [value for value in lag_values if value is not None]
    lag_distance_values = [to_float(row["stable_detection_lag_distance_m"]) for row in valid_rows]
    lag_distance_values = [value for value in lag_distance_values if value is not None]
    decel_values = [to_float(row["effective_decel_mps2"]) for row in valid_rows]
    decel_values = [value for value in decel_values if value is not None and value > 0.0]

    conservative_decel = percentile(decel_values, 0.10)
    if conservative_decel is None:
        conservative_decel = float(cfg.get("decel_mps2", 10.0))
    baseline_physical_median = _median(physical_values)
    model = deadline_model(
        _median(speed_values) or float(cfg.get("speed_kmh", 80.0)),
        conservative_decel,
        float(cfg.get("safety_margin_m", 5.0)),
        float(cfg.get("actuator_delay_s", 0.0)),
        _median(d1_values),
        float(cfg.get("target_safety_deadline_ms", 200.0)),
    )

    suggestions = {}
    for target in (200.0, 300.0):
        suggestions[str(int(target))] = {
            "target_total_physical_response_ms": target,
            "initial_bridge_delay_ms": max(0.0, target - baseline_physical_median) if baseline_physical_median is not None else None,
            "note": "Initial value only; accept a run by measured physical response, not by configured delay.",
        }
    enough_runs = len(valid_rows) >= 8 and len(cases) >= 10
    enough_decel = len(decel_values) >= 8
    result = {
        "status": "PASS" if enough_runs and enough_decel else (
            "INSUFFICIENT_DECELERATION_SAMPLES" if enough_runs else "INSUFFICIENT_VALID_BASELINE_RUNS"
        ),
        "group": group,
        "case_count": len(cases),
        "valid_case_count": len(valid_rows),
        "required_case_count": 10,
        "recommended": {
            "baseline_internal_e2e_median_ms": _median(internal_medians),
            "baseline_physical_response_median_ms": baseline_physical_median,
            "baseline_physical_response_p90_ms": percentile(physical_values, 0.90),
            "stable_detection_lag_median_ms": _median(lag_values),
            "stable_detection_lag_distance_median_m": _median(lag_distance_values),
            "stable_detection_distance_d1_median_m": _median(d1_values),
            "stable_detection_distance_d1_p10_m": percentile(d1_values, 0.10),
            "stable_detection_distance_d1_p90_m": percentile(d1_values, 0.90),
            "measured_speed_median_kmh": _median(speed_values),
            "measured_speed_p10_kmh": percentile(speed_values, 0.10),
            "measured_speed_p90_kmh": percentile(speed_values, 0.90),
            "conservative_decel_mps2": conservative_decel,
            "effective_decel_median_mps2": _median(decel_values),
            "decel_sample_count": len(decel_values),
        },
        "deadline_model_at_baseline_medians": model,
        "suggested_bridge_delays": suggestions,
        "important_checks": {
            "baseline_collision_count": sum(bool(row["collision"]) for row in rows),
            "all_valid_runs_have_zero_injection": all(abs(float(row["requested_delay_ms"])) <= 1.0 for row in valid_rows) if valid_rows else False,
            "spawn_time_available_count": len(lag_values),
            "spawn_distance_available_count": len(lag_distance_values),
            "deceleration_sample_count_sufficient": enough_decel,
            "d1_p10_to_p90_span_m": (
                percentile(d1_values, 0.90) - percentile(d1_values, 0.10)
                if d1_values else None
            ),
            "speed_p10_to_p90_span_kmh": (
                percentile(speed_values, 0.90) - percentile(speed_values, 0.10)
                if speed_values else None
            ),
        },
        "runs": rows,
    }
    with (out_dir / "scb_calibration_runs.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        fields = list(rows[0]) if rows else ["case_id", "valid_for_calibration"]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (out_dir / "scb_calibration_result.json").write_text(
        json.dumps(json_safe(result), ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate SCB deadline evidence from repeated baseline runs")
    parser.add_argument("--cases-root", type=Path, default=Path(r"D:\data\scb_data"))
    parser.add_argument("--case-dir", type=Path, action="append", default=[])
    parser.add_argument("--group", default="baseline")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("implicit_deadline_config.yaml"))
    parser.add_argument("--functional-config", type=Path, default=Path(__file__).with_name("collision_classifier_config.yaml"))
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")
    try:
        result = calibrate(
            args.cases_root, args.out_dir, args.config, args.functional_config,
            args.case_dir, args.group,
        )
    except Exception as exc:
        logging.error("SCB calibration failed: %s", exc)
        return 2
    logging.info("Calibration status=%s valid=%s/%s", result["status"], result["valid_case_count"], result["case_count"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
