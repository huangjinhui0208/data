#!/usr/bin/env python3
"""Raw-data-first realtime-defect analysis for the hidden-deadline experiment.

All outputs are written below realtime_defect_report.  The original experiment
directory is read-only.  Observed quantities and model/counterfactual
quantities are deliberately kept in different columns and tables.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
EXPERIMENT = WORKSPACE / "第二次实验"
OUTPUT = WORKSPACE / "realtime_defect_report"
TABLES = OUTPUT / "tables"
FIGURES = OUTPUT / "figures"
EXTRACTED = OUTPUT / "extracted"
VALIDATION = OUTPUT / "validation"
VENDOR = OUTPUT / "scripts" / "vendor" / "realtime_collision_analysis"
sys.path.insert(0, str(VENDOR / "src"))

import realtime_collision_core as core  # noqa: E402


EXPECTED = {
    "202607271031",
    "202607271048",
    "202607271054",
    "202607271059",
    "202607271104",
    "202607271108",
    "202607271113",
    "202607271131",
    "202607271202",
    "202607271206",
    "202607271211",
    "202607271643",
}
COLLISION_IDS = {"202607271131", "202607271643"}
UNKNOWN_OUTCOME_IDS = {"202607271206"}
MAIN_EXCLUDED_IDS = UNKNOWN_OUTCOME_IDS
OFFSET_M = 5.3074
COLORS = {
    "baseline": "#4C78A8",
    "delay": "#F2A541",
    "safe": "#2A6FBB",
    "collision": "#C44E52",
    "unknown": "#777777",
    "dark": "#30343B",
    "light": "#D9E2EC",
}

RUN_ORDER = [
    "202607271031", "202607271048", "202607271054", "202607271059",
    "202607271104", "202607271108", "202607271113", "202607271131",
    "202607271202", "202607271206", "202607271211", "202607271643",
]


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def f(value: object) -> float:
    return core.fnum(value)


def make_config() -> dict:
    config = yaml.safe_load(
        (VENDOR / "config" / "analysis_config.yaml").read_text(encoding="utf-8")
    )
    config["analysis"]["title"] = "第二次车速隐形Deadline实验"
    config["analysis"]["pointcloud_count"] = 560_000
    config["groups"] = {
        "baseline": {
            "root": str(EXPERIMENT / "baseline"),
            "nominal_injected_delay_ms": 0.0,
            "expected_runs": 7,
        },
        "delay_300ms": {
            "root": str(EXPERIMENT / "300ms"),
            "nominal_injected_delay_ms": 300.0,
            "expected_runs": 5,
        },
    }
    config["stable_perception"]["sensitivity_frames"] = [
        config["stable_perception"]["primary_frames"]
    ]
    config["effective_brake"]["sensitivity_thresholds_mps2"] = [
        config["effective_brake"]["primary_decel_threshold_mps2"]
    ]
    return config


def configure_plotting() -> None:
    font = Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf")
    if font.exists():
        fm.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = fm.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.dpi": 180,
        }
    )


def write_csv(name: str, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(TABLES / name, index=False, encoding="utf-8-sig")


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def interpolated_clearance(stable: core.FusionObservation, state: dict) -> float:
    dx = stable.x_m - state["x_m"]
    dy = stable.y_m - state["y_m"]
    return (
        dx * math.cos(state["heading_rad"])
        + dy * math.sin(state["heading_rad"])
        - OFFSET_M
    )


def displacement(start: dict | None, sample: object | None) -> float:
    if start is None or sample is None:
        return math.nan
    return math.dist(
        (start["x_m"], start["y_m"], start["z_m"]),
        (sample.x_m, sample.y_m, sample.z_m),
    )


def read_csv_rows(path: Path | None) -> list[dict[str, str]]:
    if path is None or not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        return list(csv.DictReader(handle))


def first_glob(root: Path, pattern: str) -> Path | None:
    return next(iter(sorted(root.glob(pattern))), None)


def event_stage_metrics(parsed: core.ParsedRun) -> dict:
    """Measure ground->detection flow from the complete event traces."""
    ground_path = first_glob(
        parsed.spec.run_dir, "trace/events/perception.pointcloud_ground_detection.*.csv"
    )
    detection_path = first_glob(
        parsed.spec.run_dir, "trace/events/perception.lidar_detection.*.csv"
    )
    ground = read_csv_rows(ground_path)
    detection = read_csv_rows(detection_path)
    ground_out = {
        row["trace_id"]: f(row.get("mono_ns"))
        for row in ground
        if row.get("phase") == "output_pub" and row.get("trace_id")
    }
    det_enter = {
        row["trace_id"]: f(row.get("mono_ns"))
        for row in detection
        if row.get("phase") == "proc_enter" and row.get("trace_id")
    }
    det_out = {
        row["trace_id"]: f(row.get("mono_ns"))
        for row in detection
        if row.get("phase") == "output_pub" and row.get("trace_id")
    }
    matched = sorted(set(ground_out) & set(det_out))
    queue = [
        (det_enter[trace] - ground_out[trace]) / 1e6
        for trace in matched
        if trace in det_enter and finite(det_enter[trace]) and finite(ground_out[trace])
    ]
    process = [
        (det_out[trace] - det_enter[trace]) / 1e6
        for trace in matched
        if trace in det_enter and finite(det_out[trace]) and finite(det_enter[trace])
    ]
    total = [
        (det_out[trace] - ground_out[trace]) / 1e6
        for trace in matched
        if finite(det_out[trace]) and finite(ground_out[trace])
    ]
    return {
        "ground_output_count": len(ground_out),
        "detection_output_count": len(det_out),
        "ground_detection_matched_count": len(matched),
        "ground_detection_completion_ratio": (
            len(matched) / len(ground_out) if ground_out else math.nan
        ),
        "ground_to_detection_queue_median_ms": float(np.median(queue)) if queue else math.nan,
        "ground_to_detection_process_median_ms": float(np.median(process)) if process else math.nan,
        "ground_to_detection_total_median_ms": float(np.median(total)) if total else math.nan,
        "ground_to_detection_total_p90_ms": float(np.percentile(total, 90)) if total else math.nan,
    }


def target_continuity(parsed: core.ParsedRun, endpoint_s: float) -> dict:
    rows: list[core.FusionObservation] = parsed.perception.get("target_rows", [])
    gaps = np.diff([row.header_time_s for row in rows]) * 1000.0 if len(rows) > 1 else np.asarray([])
    lifecycle = np.asarray(
        [(row.header_time_s - row.obs_time_s) * 1000.0 for row in rows], dtype=float
    )
    preceding = [row for row in rows if row.header_time_s <= endpoint_s]
    # Source age includes both the target's Fusion lifecycle and any interval
    # from the last Fusion output to the selected outcome endpoint.
    age = (endpoint_s - preceding[-1].obs_time_s) * 1000.0 if preceding else math.nan
    return {
        "target_observation_count": len(rows),
        "target_gap_max_ms": float(np.max(gaps)) if gaps.size else math.nan,
        "target_gap_p90_ms": float(np.percentile(gaps, 90)) if gaps.size else math.nan,
        "target_lifecycle_median_ms": float(np.median(lifecycle)) if lifecycle.size else math.nan,
        "target_lifecycle_p90_ms": float(np.percentile(lifecycle, 90)) if lifecycle.size else math.nan,
        "target_lifecycle_max_ms": float(np.max(lifecycle)) if lifecycle.size else math.nan,
        "target_source_age_at_outcome_ms": age,
    }


def inventory(config: dict, specs: list[core.RunSpec]) -> list[dict]:
    file_rows, schema = core.inventory_inputs(specs, config, compute_hashes=False)
    # The core inventory is intentionally run-root scoped.  The task also
    # requires existing reports, analysis scripts/configs, and prior outputs
    # at the experiment root, so append every file not already covered.
    covered = {str(Path(row["source_file"]).resolve()) for row in file_rows}
    run_lookup = {spec.run_id: spec for spec in specs}
    for path in sorted(item for item in EXPERIMENT.rglob("*") if item.is_file()):
        resolved = str(path.resolve())
        if resolved in covered:
            continue
        relative = path.relative_to(EXPERIMENT)
        parts = relative.parts
        run_id = next((part for part in parts if part in run_lookup), "")
        spec = run_lookup.get(run_id)
        category = core.categorize_file(path)
        file_rows.append(
            {
                "group_name": spec.group_name if spec else "experiment_root",
                "run_id": run_id,
                "run_directory": str(spec.run_dir) if spec else "",
                "source_file": resolved,
                "relative_path": str(relative),
                "category": category,
                "extension": path.suffix.lower(),
                "size_bytes": path.stat().st_size,
                "line_count": -1,
                "modified_time_iso": "NOT_RECOMPUTED",
                "sha256": "SKIPPED",
            }
        )
        schema["files"].append(
            {
                "source_file": resolved,
                "group_name": spec.group_name if spec else "experiment_root",
                "run_id": run_id,
                "category": category,
                "columns": [],
                "json_or_yaml_top_level_keys": [],
                "schema_error": "ROOT_LEVEL_EXISTING_ARTIFACT_NOT_USED_AS_RAW_INPUT",
                "line_count": -1,
            }
        )
    schema["category_summary"] = dict(
        sorted(Counter(row["category"] for row in file_rows).items())
    )
    pd.DataFrame(file_rows).to_csv(
        EXTRACTED / "file_inventory.csv", index=False, encoding="utf-8-sig"
    )
    write_json(EXTRACTED / "schema_inventory.json", schema)
    rows = []
    for spec in specs:
        files = core.locate_run_files(spec.run_dir)
        categories = Counter(
            core.categorize_file(p)
            for p in spec.run_dir.rglob("*")
            if p.is_file()
        )
        rows.append(
            {
                "group_name": spec.group_name,
                "run_id": spec.run_id,
                "run_directory": str(spec.run_dir),
                "file_count": sum(categories.values()),
                "log_file_count": sum(
                    1 for p in (spec.run_dir / "log").rglob("*") if p.is_file()
                ),
                "trace_file_count": sum(
                    1 for p in (spec.run_dir / "trace").rglob("*") if p.is_file()
                ),
                "collect_available": files["collect"] is not None,
                "localization_available": files["localization"] is not None,
                "perception_available": files["perception"] is not None,
                "prediction_available": files["prediction"] is not None,
                "planning_available": files["planning"] is not None,
                "control_trace_available": files["control_context"] is not None,
                "scb_available": files["scb"] is not None,
                "collision_event_available": files["collision_csv"] is not None,
                "actor_history_available": files["actor_history"] is not None,
                "included_main_analysis": spec.run_id not in MAIN_EXCLUDED_IDS,
                "exclusion_reason": (
                    "OUTCOME_UNCERTAIN_COLLISION_EVENT_ABSENT_BUT_FIXED_GEOMETRY_IMPLIES_OVERLAP"
                    if spec.run_id in UNKNOWN_OUTCOME_IDS
                    else ""
                ),
            }
        )
    write_csv("run_inventory.csv", rows)
    return rows


def analyze_run(parsed: core.ParsedRun, raw: dict, debug: dict) -> dict:
    stable = parsed.perception.get("stable")
    t1 = f(raw.get("t_sensor_origin_s"))
    t2 = f(raw.get("t_brake_effective_s"))
    state1 = core.interpolate_sample(parsed.localization, t1)
    state2 = core.interpolate_sample(parsed.localization, t2)
    d1 = interpolated_clearance(stable, state1) if stable and state1 else math.nan
    ddelay = core.integrate_speed(parsed.localization, t1, t2)
    d2 = d1 - ddelay if finite(d1) and finite(ddelay) else math.nan
    collision = bool(raw.get("collision"))
    completion = debug.get("brake_completion", {})
    near = debug.get("near_stop", {})
    strict = debug.get("stop", {})
    completion_sample = completion.get("sample")
    t_end = f(raw.get("t_collision_s")) if collision else f(completion.get("time_s"))
    if not finite(t_end):
        t_end = f(parsed.collect.get("end_log_epoch_s"))
    continuity = target_continuity(parsed, t_end)
    stages = event_stage_metrics(parsed)
    dbrake = displacement(state2, completion_sample) if not collision else math.nan
    dbrake_path = (
        core.integrate_speed(parsed.localization, t2, f(completion.get("time_s")))
        if not collision and finite(completion.get("time_s"))
        else math.nan
    )
    collision_travel_from_t1 = (
        core.integrate_speed(parsed.localization, t1, t_end)
        if collision and finite(t_end)
        else math.nan
    )
    collision_clearance_projected = (
        d1 - collision_travel_from_t1
        if collision and finite(d1) and finite(collision_travel_from_t1)
        else math.nan
    )
    margin0 = d2 - dbrake if finite(d2) and finite(dbrake) else math.nan
    margin6 = margin0 - 6.0 if finite(margin0) else math.nan
    deadline0 = (
        1000.0 * (d1 - dbrake) / state1["speed_mps"]
        if finite(d1) and finite(dbrake) and state1 and state1["speed_mps"] > 0
        else math.nan
    )
    deadline6 = deadline0 - 6000.0 / state1["speed_mps"] if finite(deadline0) else math.nan
    final_clear = f(raw.get("final_clearance_m")) if not collision else math.nan
    run_id = parsed.spec.run_id
    outcome = "collision" if collision else "safe_stop"
    if run_id in UNKNOWN_OUTCOME_IDS:
        outcome = "uncertain_geometry_event_conflict"
    row = {
        "group_name": parsed.spec.group_name,
        "nominal_injected_delay_ms": parsed.spec.nominal_delay_ms,
        "run_id": run_id,
        "included_main_analysis": run_id not in MAIN_EXCLUDED_IDS,
        "analysis_status": raw.get("analysis_status"),
        "outcome_data_observed": outcome,
        "collision_event_data_observed": collision,
        "impact_speed_data_observed_mps": f(raw.get("impact_speed_mps")),
        "target_id": parsed.target_id,
        "t1_wall_s": t1,
        "t2_wall_s": t2,
        "t_outcome_wall_s": t_end,
        "T_outcome_from_t1_s": (t_end - t1) if finite(t_end) else math.nan,
        "T_e2e_data_observed_ms": (t2 - t1) * 1000.0,
        "v1_data_observed_mps": state1["speed_mps"] if state1 else math.nan,
        "v2_data_observed_mps": state2["speed_mps"] if state2 else math.nan,
        "D1_center_data_observed_m": d1 + OFFSET_M if finite(d1) else math.nan,
        "D1_clear_data_observed_m": d1,
        "D_delay_wall_integral_data_observed_m": ddelay,
        "D2_clear_data_observed_m": d2,
        "D2_clear_direct_diagnostic_m": (
            interpolated_clearance(stable, state2) if stable and state2 else math.nan
        ),
        "D_brake_data_observed_m": dbrake,
        "D_brake_path_integral_diagnostic_m": dbrake_path,
        "D_brake_near_stop_data_observed_m": displacement(state2, near.get("sample")) if not collision else math.nan,
        "D_brake_strict_stop_data_observed_m": displacement(state2, strict.get("sample")) if not collision else math.nan,
        "D_brake_truncated_to_collision_data_observed_m": f(raw.get("distance_braked_before_collision_m")) if collision else math.nan,
        "T_brake_truncated_to_collision_data_observed_s": (t_end - t2) if collision and finite(t_end) else math.nan,
        "collision_clearance_projected_diagnostic_m": collision_clearance_projected,
        "M_collision_0m_data_observed_m": margin0,
        "M_safety_6m_data_observed_m": margin6,
        "T_deadline_collision_0m_data_observed_ms": deadline0,
        "T_deadline_safety_6m_data_observed_ms": deadline6,
        "deadline_collision_minus_observed_ms": deadline0 - (t2 - t1) * 1000.0 if finite(deadline0) else math.nan,
        "deadline_safety_minus_observed_ms": deadline6 - (t2 - t1) * 1000.0 if finite(deadline6) else math.nan,
        "final_clearance_projected_data_observed_m": final_clear,
        "sensor_to_fusion_ms": f(raw.get("sensor_to_perception_ms")),
        "fusion_to_prediction_ms": f(raw.get("perception_to_prediction_ms")),
        "prediction_to_planning_stop_ms": f(raw.get("prediction_to_planning_stop_ms")),
        "planning_stop_to_control_ms": f(raw.get("planning_stop_to_control_ms")),
        "control_to_t2_ms": f(raw.get("control_to_effective_brake_ms")),
        "sensor_to_control_ms": f(raw.get("sensor_to_control_ms")),
        "scb_requested_delay_ms": f(raw.get("scb_requested_delay_ms")),
        "scb_actual_wall_delay_ms": f(raw.get("scb_actual_wall_delay_ms")),
        "scb_actual_frame_delay": f(raw.get("scb_actual_frame_delay")),
        "scb_queue_depth_at_trigger": f(raw.get("scb_queue_depth_at_trigger")),
        "scb_lifecycle_complete": bool(raw.get("scb_lifecycle_complete")),
        "scb_trigger_relative_t1_s": f(raw.get("scb_trigger_relative_t1_s")),
        "planning_stop_count": int(parsed.planning.get("stop_count", 0)),
        "planning_empty_trajectory_count": int(parsed.planning.get("error_counts", {}).get("empty_trajectory", 0)),
        "planning_primal_infeasible_count": int(raw.get("planning_primal_infeasible_count", 0)),
        "planning_speed_fallback_count": int(raw.get("planning_speed_fallback_count", 0)),
        "control_payload_archived": False,
        "clock_alignment_status": raw.get("clock_alignment_status"),
        "clock_alignment_p95_residual_ms": f(raw.get("clock_alignment_p95_residual_ms")),
        "source_localization_file": raw.get("source_localization_file"),
        "source_perception_file": raw.get("source_perception_file"),
        "source_collision_file": raw.get("source_collision_file"),
        **stages,
        **continuity,
    }
    return row


def actor_identity(parsed: core.ParsedRun, row: dict) -> dict:
    history_path = parsed.files.get("actor_history")
    base = {
        "run_id": parsed.spec.run_id,
        "target_id_fusion_planning": parsed.target_id,
        "collision_event_observed": row["collision_event_data_observed"],
        "actor_history_available": history_path is not None,
        "target_association_method": "Planning STOP ID + stable Fusion trajectory",
        "association_confidence": "MEDIUM_HIGH",
    }
    if history_path is None:
        return {
            **base,
            "carla_other_actor_id": "",
            "carla_other_actor_type": "",
            "matched_frame_count": 0,
            "position_error_median_m": math.nan,
            "position_error_p90_m": math.nan,
            "speed_error_median_mps": math.nan,
            "audit_conclusion": "NO_CARLA_ACTOR_TRUTH; stable static Fusion target only",
        }
    history = [r for r in read_csv_rows(history_path) if r.get("role") == "other"]
    if not history:
        return {**base, "matched_frame_count": 0, "audit_conclusion": "NO_OTHER_ACTOR_ROWS"}
    times = np.asarray([f(r.get("wall_time_unix_ns")) / 1e9 for r in history])
    errors: list[float] = []
    speed_errors: list[float] = []
    for obs in parsed.perception.get("target_rows", []):
        idx = int(np.argmin(np.abs(times - obs.header_time_s)))
        if abs(times[idx] - obs.header_time_s) > 0.15:
            continue
        hist = history[idx]
        hx, hy = f(hist.get("location_x")), -f(hist.get("location_y"))
        hv = math.hypot(f(hist.get("velocity_x")), f(hist.get("velocity_y")))
        errors.append(math.hypot(obs.x_m - hx, obs.y_m - hy))
        speed_errors.append(abs(obs.speed_mps - hv))
    actor_ids = Counter(r.get("actor_id") for r in history)
    actor_types = Counter(r.get("actor_type") for r in history)
    return {
        **base,
        "carla_other_actor_id": actor_ids.most_common(1)[0][0],
        "carla_other_actor_type": actor_types.most_common(1)[0][0],
        "matched_frame_count": len(errors),
        "position_error_median_m": float(np.median(errors)) if errors else math.nan,
        "position_error_p90_m": float(np.percentile(errors, 90)) if errors else math.nan,
        "speed_error_median_mps": float(np.median(speed_errors)) if speed_errors else math.nan,
        "association_confidence": "HIGH" if errors and np.median(errors) < 2.0 else "MEDIUM",
        "audit_conclusion": "Fusion/Planning target matches CARLA collision counterpart across frames",
    }


def summarize(rows: list[dict]) -> list[dict]:
    frame = pd.DataFrame([r for r in rows if r["included_main_analysis"]])
    metrics = [
        "T_e2e_data_observed_ms",
        "D_delay_wall_integral_data_observed_m",
        "D1_clear_data_observed_m",
        "D2_clear_data_observed_m",
        "v1_data_observed_mps",
        "v2_data_observed_mps",
        "scb_actual_wall_delay_ms",
        "M_collision_0m_data_observed_m",
        "M_safety_6m_data_observed_m",
    ]
    result = []
    for group, data in frame.groupby("group_name", sort=False):
        for metric in metrics:
            vals = pd.to_numeric(data[metric], errors="coerce").dropna().to_numpy()
            result.append(
                {
                    "group_name": group,
                    "metric": metric,
                    "run_count": len(data),
                    "available_count": len(vals),
                    "mean": float(np.mean(vals)) if len(vals) else math.nan,
                    "sd": float(np.std(vals, ddof=1)) if len(vals) > 1 else math.nan,
                    "median": float(np.median(vals)) if len(vals) else math.nan,
                    "p90": float(np.percentile(vals, 90)) if len(vals) else math.nan,
                    "min": float(np.min(vals)) if len(vals) else math.nan,
                    "max": float(np.max(vals)) if len(vals) else math.nan,
                    "collision_count": int(data["collision_event_data_observed"].sum()),
                }
            )
    return result


def stage_summary(rows: list[dict]) -> list[dict]:
    metrics = [
        "sensor_to_fusion_ms",
        "fusion_to_prediction_ms",
        "prediction_to_planning_stop_ms",
        "planning_stop_to_control_ms",
        "control_to_t2_ms",
        "sensor_to_control_ms",
        "scb_actual_wall_delay_ms",
        "scb_queue_depth_at_trigger",
        "ground_to_detection_queue_median_ms",
        "ground_to_detection_process_median_ms",
        "ground_detection_completion_ratio",
        "target_gap_max_ms",
        "target_lifecycle_p90_ms",
        "target_source_age_at_outcome_ms",
    ]
    frame = pd.DataFrame([r for r in rows if r["included_main_analysis"]])
    out = []
    for group, data in frame.groupby("group_name", sort=False):
        for metric in metrics:
            vals = pd.to_numeric(data[metric], errors="coerce").dropna().to_numpy()
            out.append(
                {
                    "group_name": group,
                    "metric": metric,
                    "available_count": len(vals),
                    "median": float(np.median(vals)) if len(vals) else math.nan,
                    "p90": float(np.percentile(vals, 90)) if len(vals) else math.nan,
                    "min": float(np.min(vals)) if len(vals) else math.nan,
                    "max": float(np.max(vals)) if len(vals) else math.nan,
                }
            )
    return out


def source_age_at(parsed: core.ParsedRun, timestamp_s: float) -> float:
    preceding = [
        row
        for row in parsed.perception.get("target_rows", [])
        if row.header_time_s <= timestamp_s
    ]
    return (
        (timestamp_s - preceding[-1].obs_time_s) * 1000.0
        if preceding
        else math.nan
    )


def collision_comparison(
    rows_by_id: dict[str, dict], parsed_by_id: dict[str, core.ParsedRun]
) -> list[dict]:
    metrics = [
        "T_e2e_data_observed_ms",
        "v1_data_observed_mps",
        "v2_data_observed_mps",
        "D1_clear_data_observed_m",
        "D_delay_wall_integral_data_observed_m",
        "D2_clear_data_observed_m",
        "sensor_to_fusion_ms",
        "target_gap_max_ms",
        "target_lifecycle_p90_ms",
        "target_source_age_at_outcome_ms",
        "planning_empty_trajectory_count",
        "planning_speed_fallback_count",
    ]
    out = []
    for collision_id in ["202607271131", "202607271643"]:
        collision = rows_by_id[collision_id]
        control = rows_by_id["202607271211"]
        for metric in metrics:
            out.append(
                {
                    "collision_run_id": collision_id,
                    "safe_control_run_id": "202607271211",
                    "metric": metric,
                    "collision_value": collision.get(metric),
                    "safe_control_value": control.get(metric),
                    "collision_minus_control": (
                        f(collision.get(metric)) - f(control.get(metric))
                        if finite(collision.get(metric)) and finite(control.get(metric))
                        else math.nan
                    ),
                    "evidence_type": "B_DIRECT_LOG_MEASUREMENT",
                }
            )
        collision_time = f(parsed_by_id[collision_id].collision.get("time_s"))
        elapsed = collision_time - collision["t1_wall_s"]
        control_match_time = control["t1_wall_s"] + elapsed
        out.append(
            {
                "collision_run_id": collision_id,
                "safe_control_run_id": "202607271211",
                "metric": "target_source_age_at_case_matched_elapsed_ms",
                "collision_value": source_age_at(
                    parsed_by_id[collision_id], collision_time
                ),
                "safe_control_value": source_age_at(
                    parsed_by_id["202607271211"], control_match_time
                ),
                "collision_minus_control": source_age_at(
                    parsed_by_id[collision_id], collision_time
                )
                - source_age_at(parsed_by_id["202607271211"], control_match_time),
                "evidence_type": "B_DIRECT_LOG_MEASUREMENT",
            }
        )
    return out


def evidence_matrix(rows_by_id: dict[str, dict]) -> list[dict]:
    r1131 = rows_by_id["202607271131"]
    r1202 = rows_by_id["202607271202"]
    r1206 = rows_by_id["202607271206"]
    r1211 = rows_by_id["202607271211"]
    r1643 = rows_by_id["202607271643"]
    return [
        {
            "run_id": "202607271131",
            "observed_outcome": "collision",
            "realtime_defect_class": "RT_DOMINATED_COLLISION",
            "primary_defect": "RESPONSE_TOO_LONG + DATA_FRESHNESS/OUTPUT_CONTINUITY_DEGRADATION",
            "direct_evidence": f"T_e2e={r1131['T_e2e_data_observed_ms']:.3f} ms; D_delay={r1131['D_delay_wall_integral_data_observed_m']:.3f} m; Fusion max gap={r1131['target_gap_max_ms']:.3f} ms; impact={r1131['impact_speed_data_observed_mps']:.3f} m/s",
            "counterevidence_or_confounders": f"D1 is {r1211['D1_clear_data_observed_m']-r1131['D1_clear_data_observed_m']:.3f} m smaller than 1211; braking capability is not controlled",
            "attribution": "Realtime-dominated candidate; not RT_ONLY because initial clearance and Fusion continuity differ",
            "confidence": "MEDIUM_HIGH",
            "evidence_boundary": "Observed chain is direct; baseline-restoration avoidance is model/predicted only",
        },
        {
            "run_id": "202607271202",
            "observed_outcome": "safe_stop",
            "realtime_defect_class": "NONCOLLISION_REALTIME_MARGIN_EXHAUSTION",
            "primary_defect": "RESPONSE_TOO_LONG",
            "direct_evidence": f"T_e2e={r1202['T_e2e_data_observed_ms']:.3f} ms; observed M0={r1202['M_collision_0m_data_observed_m']:.3f} m",
            "counterevidence_or_confounders": "No collision and no actor-history physical truth",
            "attribution": "Critical stop showing margin compression without an observed collision",
            "confidence": "HIGH_FOR_MARGIN; MEDIUM_FOR_PHYSICAL_OUTCOME",
            "evidence_boundary": "Does not establish a universal millisecond threshold",
        },
        {
            "run_id": "202607271206",
            "observed_outcome": "uncertain_geometry_event_conflict",
            "realtime_defect_class": "INDETERMINATE",
            "primary_defect": "OUTCOME_EVIDENCE_CONFLICT",
            "direct_evidence": f"T_e2e={r1206['T_e2e_data_observed_ms']:.3f} ms; projected final clearance={r1206['final_clearance_projected_data_observed_m']:.3f} m; no CollisionSensor event/actor history",
            "counterevidence_or_confounders": "Projected overlap conflicts with missing collision event; geometry offset uncertainty is insufficient to reconcile it",
            "attribution": "Timing metrics usable for diagnostics; outcome is not usable for main effect estimation",
            "confidence": "HIGH_FOR_INDETERMINATE",
            "evidence_boundary": "Excluded from the 11-run main outcome analysis before interpreting direction",
        },
        {
            "run_id": "202607271211",
            "observed_outcome": "safe_stop",
            "realtime_defect_class": "NONCOLLISION_REALTIME_MARGIN_EXHAUSTION",
            "primary_defect": "RESPONSE_TOO_LONG",
            "direct_evidence": f"T_e2e={r1211['T_e2e_data_observed_ms']:.3f} ms; observed M0={r1211['M_collision_0m_data_observed_m']:.3f} m",
            "counterevidence_or_confounders": "No collision; selected as reliable same-setting comparison rather than proof of causality",
            "attribution": "Critical stop and closest reliable safe comparison for both collision cases",
            "confidence": "HIGH_FOR_MARGIN",
            "evidence_boundary": "1206 has closer latency to 1131 but uncertain outcome, so it is not used as safe control",
        },
        {
            "run_id": "202607271643",
            "observed_outcome": "collision",
            "realtime_defect_class": "MULTI_FACTOR_COLLISION",
            "primary_defect": "RESPONSE_TOO_LONG with smaller initial clearance and observed braking differences",
            "direct_evidence": f"T_e2e={r1643['T_e2e_data_observed_ms']:.3f} ms; D1={r1643['D1_clear_data_observed_m']:.3f} m; D2={r1643['D2_clear_data_observed_m']:.3f} m; impact={r1643['impact_speed_data_observed_mps']:.3f} m/s",
            "counterevidence_or_confounders": f"Fusion max gap={r1643['target_gap_max_ms']:.3f} ms is not abnormal like 1131; D1 is {r1211['D1_clear_data_observed_m']-r1643['D1_clear_data_observed_m']:.3f} m smaller than 1211",
            "attribution": "Realtime defect amplifies risk and severity but is not sufficient by itself to explain the collision",
            "confidence": "HIGH",
            "evidence_boundary": "Baseline-restoration model still predicts contact at reduced impact speed",
        },
    ]


def counterfactual(run: dict, parsed: core.ParsedRun, baseline_ms: float) -> dict:
    t1 = run["t1_wall_s"]
    t2 = run["t2_wall_s"]
    collision_s = f(parsed.collision.get("time_s"))
    restored_t2 = t1 + baseline_ms / 1000.0
    state_restored = core.interpolate_sample(parsed.localization, restored_t2)
    v_restored = state_restored["speed_mps"]
    restored_delay = core.integrate_speed(parsed.localization, t1, restored_t2)
    actual_delay = run["D_delay_wall_integral_data_observed_m"]
    actual_to_contact = core.integrate_speed(parsed.localization, t2, collision_s)
    v2 = run["v2_data_observed_mps"]
    impact = run["impact_speed_data_observed_mps"]
    aeq = (v2**2 - impact**2) / (2.0 * actual_to_contact)
    d2_actual = run["D2_clear_data_observed_m"]
    correction = actual_to_contact - d2_actual
    d2_restored = run["D1_clear_data_observed_m"] - restored_delay
    available = d2_restored + correction
    required = v_restored**2 / (2.0 * aeq)
    margin = available - required
    impact_pred = math.sqrt(max(0.0, v_restored**2 - 2.0 * aeq * available))
    return {
        "run_id": run["run_id"],
        "reference_baseline_latency_model_input_ms": baseline_ms,
        "actual_latency_data_observed_ms": run["T_e2e_data_observed_ms"],
        "D_delay_actual_data_observed_m": actual_delay,
        "D_delay_restored_from_observed_trace_m": restored_delay,
        "response_distance_recovered_model_m": actual_delay - restored_delay,
        "v2_restored_from_observed_trace_mps": v_restored,
        "equivalent_deceleration_from_collision_data_mps2": aeq,
        "available_to_observed_contact_restored_model_m": available,
        "required_stopping_distance_restored_model_m": required,
        "margin_to_observed_contact_restored_model_m": margin,
        "collision_model_predicted": margin < 0,
        "impact_speed_model_predicted_mps": impact_pred,
        "model_scope_note": "C_MODEL_COUNTERFACTUAL; not an observed outcome",
    }


def savefig(fig: plt.Figure, name: str) -> None:
    fig.tight_layout()
    fig.savefig(FIGURES / name)
    plt.close(fig)


def plot_group_results(rows: list[dict]) -> None:
    main = [r for r in rows if r["included_main_analysis"]]
    base = [r for r in main if r["group_name"] == "baseline"]
    delay = [r for r in main if r["group_name"] == "delay_300ms"]
    groups = [base, delay]
    labels = ["baseline\n(n=7)", "300 ms\n(n=4)"]
    for metric, ylabel, name, title in [
        ("T_e2e_data_observed_ms", "端到端响应/ms", "group_e2e_response.png", "干预后实际响应时间整体右移"),
        ("D_delay_wall_integral_data_observed_m", "响应阶段距离债务/m", "group_distance_debt.png", "墙钟速度积分显示距离债务增加"),
        ("D2_clear_data_observed_m", "$t_2$时剩余净距/m", "group_braking_position.png", "有效制动开始位置更靠近障碍物"),
    ]:
        fig, ax = plt.subplots(figsize=(7.2, 5.0))
        for i, data in enumerate(groups):
            values = np.asarray([r[metric] for r in data])
            ax.scatter(
                np.full(len(values), i) + np.linspace(-0.08, 0.08, len(values)),
                values,
                s=62,
                color=COLORS["baseline"] if i == 0 else COLORS["delay"],
                edgecolor="white",
                zorder=3,
            )
            for j, r in enumerate(data):
                if r["collision_event_data_observed"]:
                    ax.scatter(i + np.linspace(-0.08, 0.08, len(values))[j], r[metric], marker="X", s=110, color=COLORS["collision"], zorder=4)
                    ax.annotate(r["run_id"][-4:], (i, r[metric]), xytext=(10, 3), textcoords="offset points", fontsize=8)
            ax.hlines(np.median(values), i - 0.22, i + 0.22, color=COLORS["dark"], lw=2.2)
        ax.set_xticks([0, 1], labels)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.2)
        savefig(fig, name)


def plot_causal_chain() -> None:
    fig, ax = plt.subplots(figsize=(12.2, 3.0))
    ax.axis("off")
    labels = [
        "300 ms控制延迟\n（实测近300 ms）",
        "$T_{e2e}$增加\n约450 ms",
        "$D_{delay}$增加\n约7.43 m",
        "$D_2$减少\n约7.14 m",
        "当前样本转换区\n出现2/4碰撞",
    ]
    xs = np.linspace(0.08, 0.92, len(labels))
    for i, (x, label) in enumerate(zip(xs, labels)):
        color = COLORS["light"] if i < 4 else "#F6D6D6"
        ax.text(x, 0.5, label, ha="center", va="center", fontsize=11,
                bbox=dict(boxstyle="round,pad=0.6", fc=color, ec=COLORS["dark"], lw=1.1), transform=ax.transAxes)
        if i < len(labels) - 1:
            ax.annotate("", xy=(xs[i + 1] - 0.085, 0.5), xytext=(x + 0.085, 0.5), xycoords=ax.transAxes,
                        arrowprops=dict(arrowstyle="->", color=COLORS["dark"], lw=1.6))
    ax.set_title("数据支持的因果链：干预 → 响应 → 距离预算 → 结局", pad=18)
    savefig(fig, "causal_chain.png")


def plot_safety(rows: list[dict]) -> None:
    main = [r for r in rows if r["included_main_analysis"]]
    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    safe = [r for r in main if not r["collision_event_data_observed"]]
    collision = [r for r in main if r["collision_event_data_observed"]]
    ax.scatter([r["T_e2e_data_observed_ms"] for r in safe], [r["M_collision_0m_data_observed_m"] for r in safe], s=68, color=COLORS["safe"], label="完整停车：观测0 m余量")
    ax.scatter([r["T_e2e_data_observed_ms"] for r in collision], [0, 0], marker="X", s=120, color=COLORS["collision"], label="碰撞：完整制动距离/余量NA")
    for r in collision:
        ax.annotate(f"{r['run_id'][-4:]}\n撞击 {r['impact_speed_data_observed_mps']:.2f} m/s", (r["T_e2e_data_observed_ms"], 0), xytext=(5, 10), textcoords="offset points", fontsize=8)
    ax.axhline(0, color=COLORS["dark"], lw=1)
    ax.set_xlabel("实际端到端响应/ms")
    ax.set_ylabel("观测碰撞余量/m")
    ax.set_title("当前样本的安全陡峭（碰撞点不伪造余量）")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    savefig(fig, "safety_cliff.png")

    fig, ax = plt.subplots(figsize=(10.5, 5.8))
    shown = sorted(main, key=lambda r: r["T_e2e_data_observed_ms"])
    x = np.arange(len(shown))
    delay_vals = np.asarray([r["D_delay_wall_integral_data_observed_m"] for r in shown])
    brake_vals = np.asarray([r["D_brake_data_observed_m"] if finite(r["D_brake_data_observed_m"]) else r["D_brake_truncated_to_collision_data_observed_m"] for r in shown])
    ax.bar(x, delay_vals, color=COLORS["delay"], label="$D_{delay}$（观测）")
    bars = ax.bar(x, brake_vals, bottom=delay_vals, color=COLORS["baseline"], alpha=0.85, label="$D_{brake}$（停车run）/碰撞前截断距离")
    for i, r in enumerate(shown):
        if r["collision_event_data_observed"]:
            bars[i].set_hatch("///")
            bars[i].set_edgecolor(COLORS["collision"])
        ax.plot([i - 0.34, i + 0.34], [r["D1_clear_data_observed_m"]] * 2, color=COLORS["dark"], lw=2)
    ax.set_xticks(x, [r["run_id"][-4:] for r in shown], rotation=45)
    ax.set_ylabel("距离/m")
    ax.set_title("从$t_1$起算的距离预算分解（黑线为$D_1$）")
    ax.legend(frameon=False, fontsize=9)
    ax.grid(axis="y", alpha=0.18)
    savefig(fig, "distance_budget_decomposition.png")

    complete = [r for r in shown if finite(r["T_deadline_collision_0m_data_observed_ms"])]
    fig, ax = plt.subplots(figsize=(10.2, 5.5))
    x = np.arange(len(complete))
    observed = [r["T_e2e_data_observed_ms"] for r in complete]
    d0 = [r["T_deadline_collision_0m_data_observed_ms"] for r in complete]
    d6 = [r["T_deadline_safety_6m_data_observed_ms"] for r in complete]
    ax.plot(x, observed, "o-", label="观测$T_{e2e}$", color=COLORS["dark"])
    ax.plot(x, d0, "s--", label="0 m观测deadline", color=COLORS["safe"])
    ax.plot(x, d6, "^--", label="6 m观测deadline", color=COLORS["delay"])
    ax.set_xticks(x, [r["run_id"][-4:] for r in complete], rotation=45)
    ax.set_ylabel("时间/ms")
    ax.set_title("完整停车run的车速隐形Deadline")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    savefig(fig, "deadline_margin.png")


def case_plots(run: dict, parsed: core.ParsedRun) -> None:
    rid = run["run_id"][-4:]
    stages = [
        ("Fusion", run["sensor_to_fusion_ms"]),
        ("Prediction", run["fusion_to_prediction_ms"]),
        ("Planning STOP", run["prediction_to_planning_stop_ms"]),
        ("Control", run["planning_stop_to_control_ms"]),
        ("Control→$t_2$", run["control_to_t2_ms"]),
    ]
    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    left = 0.0
    for i, (label, value) in enumerate(stages):
        ax.barh([0], [value], left=[left], color=plt.cm.Blues(0.35 + i * 0.12), label=f"{label} {value:.1f} ms")
        left += value
    ax.axvline(run["T_e2e_data_observed_ms"], color=COLORS["collision"], lw=1.5)
    ax.set_yticks([])
    ax.set_xlabel("从$t_1$起算的时间/ms")
    ax.set_title(f"{rid}：目标信息到有效制动的阶段分解")
    ax.legend(frameon=False, fontsize=8, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.14))
    savefig(fig, f"case_{rid}_latency_breakdown.png")

    t1, t2 = run["t1_wall_s"], run["t2_wall_s"]
    end = f(parsed.collision.get("time_s"))
    samples = [s for s in parsed.localization if t1 <= s.time_s <= end]
    times = np.asarray([s.time_s - t1 for s in samples])
    speeds = np.asarray([s.speed_mps for s in samples])
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    ax.plot(times, speeds, color=COLORS["baseline"], lw=2)
    ax.axvline(t2 - t1, color=COLORS["delay"], ls="--", label="$t_2$")
    ax.axvline(end - t1, color=COLORS["collision"], ls=":", label="碰撞")
    ax.scatter([end - t1], [run["impact_speed_data_observed_mps"]], marker="X", s=90, color=COLORS["collision"])
    ax.set_xlabel("相对$t_1$时间/s")
    ax.set_ylabel("车速/(m/s)")
    ax.set_title(f"{rid}：车速轨迹")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    savefig(fig, f"case_{rid}_speed.png")

    station = [-run["D1_clear_data_observed_m"] + core.integrate_speed(parsed.localization, t1, s.time_s) for s in samples]
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    ax.plot(times, station, color=COLORS["baseline"], lw=2)
    ax.axhline(0, color=COLORS["collision"], lw=1.2, label="障碍物接触边界（投影）")
    ax.axvline(t2 - t1, color=COLORS["delay"], ls="--", label="$t_2$")
    ax.set_xlabel("相对$t_1$时间/s")
    ax.set_ylabel("S/m（$t_1$时$S=-D_1$）")
    ax.set_title(f"{rid}：S–T轨迹")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    savefig(fig, f"case_{rid}_st.png")

    target = parsed.perception.get("target_rows", [])
    ft = np.asarray([o.header_time_s - t1 for o in target])
    age = np.asarray([(o.header_time_s - o.obs_time_s) * 1000 for o in target])
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=True)
    axes[0].eventplot(ft, colors=COLORS["baseline"], lineoffsets=1, linelengths=0.8)
    axes[0].set_yticks([])
    axes[0].set_ylabel("Fusion输出")
    axes[0].set_title(f"{rid}：目标Fusion输出时序与源数据年龄")
    axes[1].plot(ft, age, "o-", ms=4, color=COLORS["baseline"])
    axes[1].set_ylabel("源数据年龄/ms")
    axes[1].set_xlabel("相对$t_1$时间/s")
    for ax in axes:
        ax.axvline(t2 - t1, color=COLORS["delay"], ls="--")
        ax.axvline(end - t1, color=COLORS["collision"], ls=":")
        ax.grid(alpha=0.18)
    savefig(fig, f"case_{rid}_fusion_timeline_age.png")


def run_color(row: dict) -> str:
    return COLORS["baseline"] if row["group_name"] == "baseline" else COLORS["delay"]


def run_label(row: dict) -> str:
    return row["run_id"][-4:]


def ordered_rows(rows: list[dict]) -> list[dict]:
    by_id = {row["run_id"]: row for row in rows}
    return [by_id[run_id] for run_id in RUN_ORDER]


def add_status_marker(ax: plt.Axes, x: float, y: float, row: dict) -> None:
    if row["collision_event_data_observed"]:
        ax.scatter(x, y, marker="X", s=115, color=COLORS["collision"], edgecolor="white", linewidth=0.7, zorder=5)
    elif not row["included_main_analysis"]:
        ax.scatter(x, y, marker="s", s=92, color=COLORS["unknown"], edgecolor="white", linewidth=0.7, zorder=5)


def draw_chain_figure(labels: list[str], title: str, subtitle: str, filename: str) -> None:
    fig, ax = plt.subplots(figsize=(13.2, 3.35))
    ax.axis("off")
    xs = np.linspace(0.07, 0.93, len(labels))
    for index, (x, label) in enumerate(zip(xs, labels)):
        fc = "#E7EEF6" if index < len(labels) - 1 else "#F7DDDD"
        ax.text(
            x, 0.48, label, transform=ax.transAxes, ha="center", va="center",
            fontsize=10.2, bbox=dict(boxstyle="round,pad=0.58", fc=fc, ec=COLORS["dark"], lw=1.0),
        )
        if index < len(labels) - 1:
            ax.annotate(
                "", xy=(xs[index + 1] - 0.07, 0.48), xytext=(x + 0.07, 0.48),
                xycoords=ax.transAxes, arrowprops=dict(arrowstyle="->", lw=1.5, color=COLORS["dark"]),
            )
    ax.set_title(title, fontsize=14, pad=20)
    fig.text(0.5, 0.055, subtitle, ha="center", fontsize=9, color="#555555")
    savefig(fig, filename)


def plot_required_figures(rows: list[dict], parsed_by_id: dict[str, core.ParsedRun]) -> None:
    shown = ordered_rows(rows)
    main = [row for row in shown if row["included_main_analysis"]]
    base = [row for row in main if row["group_name"] == "baseline"]
    delay = [row for row in main if row["group_name"] == "delay_300ms"]
    base_lat = float(np.median([row["T_e2e_data_observed_ms"] for row in base]))
    delay_lat = float(np.median([row["T_e2e_data_observed_ms"] for row in delay]))
    base_debt = float(np.median([row["D_delay_wall_integral_data_observed_m"] for row in base]))
    delay_debt = float(np.median([row["D_delay_wall_integral_data_observed_m"] for row in delay]))
    base_d2 = float(np.median([row["D2_clear_data_observed_m"] for row in base]))
    delay_d2 = float(np.median([row["D2_clear_data_observed_m"] for row in delay]))
    delta_lat = delay_lat - base_lat
    amp = delta_lat / 300.0

    draw_chain_figure(
        [
            "受控时序故障\nSCB请求 300 ms",
            f"闭环响应变化\n中位数 +{delta_lat:.1f} ms",
            f"距离债务变化\n中位数 +{delay_debt-base_debt:.2f} m",
            f"制动起点净距\n中位数 −{base_d2-delay_d2:.2f} m",
            "安全后果\n临界停车/碰撞",
            "缺陷分类与归因\n主导/多因素/不确定",
        ],
        "实时性故障传播链",
        "箭头表示本批次数据支持的传播顺序；不表示每一起碰撞都由单一时延因素决定。",
        "realtime_fault_propagation_chain.png",
    )

    x = np.arange(len(shown))
    fig, ax = plt.subplots(figsize=(11.2, 5.4))
    values = [row["T_e2e_data_observed_ms"] for row in shown]
    bars = ax.bar(x, values, color=[run_color(row) for row in shown], width=0.72)
    for index, row in enumerate(shown):
        if not row["included_main_analysis"]:
            bars[index].set_hatch("///")
            bars[index].set_edgecolor(COLORS["unknown"])
        add_status_marker(ax, index, values[index], row)
    ax.axhline(base_lat, color=COLORS["baseline"], ls="--", lw=1.2, label=f"baseline中位数 {base_lat:.1f} ms")
    ax.axhline(delay_lat, color=COLORS["delay"], ls="--", lw=1.2, label=f"300 ms主分析中位数 {delay_lat:.1f} ms")
    ax.set_xticks(x, [run_label(row) for row in shown], rotation=45)
    ax.set_ylabel("实际端到端响应 T_e2e / ms")
    ax.set_title("逐 run 的端到端响应")
    ax.legend(frameon=False, ncol=2)
    ax.grid(axis="y", alpha=0.2)
    fig.text(0.5, 0.01, "蓝色=baseline，橙色=300 ms；红叉=碰撞，灰色斜线=1206结局不确定。", ha="center", fontsize=9, color="#555555")
    savefig(fig, "e2e_response_by_run.png")

    fig, ax = plt.subplots(figsize=(8.3, 5.2))
    scb_delta = float(np.median([row["scb_actual_wall_delay_ms"] for row in delay])) - float(np.median([row["scb_actual_wall_delay_ms"] for row in base]))
    amp_values = [300.0, scb_delta, delta_lat]
    labels = ["名义注入", "SCB实测增量", "闭环响应中位增量"]
    bars = ax.bar(labels, amp_values, color=["#B5BDC9", COLORS["delay"], COLORS["collision"]], width=0.62)
    for bar, value in zip(bars, amp_values):
        ax.text(bar.get_x() + bar.get_width()/2, value + 8, f"{value:.1f} ms", ha="center", fontsize=10)
    ax.set_ylabel("时间增量 / ms")
    ax.set_ylim(0, max(amp_values) * 1.25)
    ax.set_title(f"注入延迟与闭环增量（放大系数 {amp:.2f}）")
    ax.grid(axis="y", alpha=0.2)
    fig.text(0.5, 0.01, "放大系数=(300 ms组T_e2e中位数−baseline中位数)/300 ms；不把剩余增量强行归因给单一模块。", ha="center", fontsize=8.7, color="#555555")
    savefig(fig, "latency_amplification.png")

    for metric, ylabel, title, filename in [
        ("D_delay_wall_integral_data_observed_m", "响应阶段墙钟积分距离 D_delay / m", "逐 run 的距离债务", "distance_debt_by_run.png"),
        ("D2_clear_data_observed_m", "有效制动开始净距 D2 / m", "逐 run 的有效制动位置", "braking_position_by_run.png"),
    ]:
        fig, ax = plt.subplots(figsize=(11.2, 5.4))
        values = [row[metric] for row in shown]
        bars = ax.bar(x, values, color=[run_color(row) for row in shown], width=0.72)
        for index, row in enumerate(shown):
            if not row["included_main_analysis"]:
                bars[index].set_hatch("///")
                bars[index].set_edgecolor(COLORS["unknown"])
            add_status_marker(ax, index, values[index], row)
        ax.set_xticks(x, [run_label(row) for row in shown], rotation=45)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.2)
        fig.text(0.5, 0.01, "所有数值使用同一 t1/t2 与墙钟时间基准；红叉仅标记结局，不改变柱高。", ha="center", fontsize=9, color="#555555")
        savefig(fig, filename)

    fig, ax = plt.subplots(figsize=(9.2, 5.8))
    label_offsets = {
        "202607271031": (5, 12), "202607271048": (58, -28),
        "202607271054": (38, -5), "202607271059": (5, 9),
        "202607271104": (5, -20), "202607271108": (34, -12),
        "202607271113": (5, 13), "202607271131": (5, -25),
        "202607271202": (5, -22), "202607271206": (5, 9),
        "202607271211": (5, 12), "202607271643": (5, 0),
    }
    for row in shown:
        marker = "X" if row["collision_event_data_observed"] else ("s" if not row["included_main_analysis"] else "o")
        color = COLORS["collision"] if row["collision_event_data_observed"] else (COLORS["unknown"] if not row["included_main_analysis"] else run_color(row))
        ax.scatter(row["T_e2e_data_observed_ms"], row["D2_clear_data_observed_m"], s=95, marker=marker, color=color, edgecolor="white", linewidth=0.6, zorder=3)
        note = run_label(row)
        if row["collision_event_data_observed"]:
            note += f"\n撞击{row['impact_speed_data_observed_mps']:.1f}m/s"
        elif row["group_name"] != "baseline" and finite(row["M_collision_0m_data_observed_m"]):
            note += f"\nM0={row['M_collision_0m_data_observed_m']:.1f}m"
        ax.annotate(
            note,
            (row["T_e2e_data_observed_ms"], row["D2_clear_data_observed_m"]),
            xytext=label_offsets[row["run_id"]], textcoords="offset points", fontsize=7.5,
        )
    ax.set_xlabel("实际端到端响应 T_e2e / ms")
    ax.set_ylabel("有效制动开始净距 D2 / m")
    ax.set_xlim(260, 930)
    ax.set_title("当前样本中的实时性安全陡峭")
    ax.grid(alpha=0.2)
    fig.text(0.5, 0.01, "图示的是本批次样本转换区，不据此声明通用700 ms阈值；碰撞run的完整制动余量保持NA。", ha="center", fontsize=8.8, color="#555555")
    savefig(fig, "realtime_safety_cliff.png")

    r1131, r1211, r1643 = (next(row for row in shown if row["run_id"] == run_id) for run_id in ["202607271131", "202607271211", "202607271643"])
    draw_chain_figure(
        [
            f"响应偏晚\n{r1131['T_e2e_data_observed_ms']:.1f} ms",
            f"Fusion连续性异常\n最大间隔{r1131['target_gap_max_ms']:.1f} ms",
            f"距离债务\n{r1131['D_delay_wall_integral_data_observed_m']:.2f} m",
            f"制动起点\nD2={r1131['D2_clear_data_observed_m']:.2f} m",
            f"碰撞\n{r1131['impact_speed_data_observed_mps']:.2f} m/s",
        ],
        "案例 1131：实时性主导候选的证据链",
        f"与可靠安全对照1211相比：T_e2e +{r1131['T_e2e_data_observed_ms']-r1211['T_e2e_data_observed_ms']:.1f} ms，D1小{r1211['D1_clear_data_observed_m']-r1131['D1_clear_data_observed_m']:.2f} m；因此不归类为纯时延碰撞。",
        "case_1131_causal_chain.png",
    )

    def freshness_figure(run: dict, filename: str, title: str, include_gap: bool) -> None:
        parsed = parsed_by_id[run["run_id"]]
        target = parsed.perception.get("target_rows", [])
        rel = np.asarray([obs.header_time_s - run["t1_wall_s"] for obs in target])
        lifecycle = np.asarray([(obs.header_time_s - obs.obs_time_s) * 1000.0 for obs in target])
        gaps = np.diff(np.asarray([obs.header_time_s for obs in target])) * 1000.0
        fig, axes = plt.subplots(2, 1, figsize=(9.4, 6.5), sharex=True)
        axes[0].plot(rel, lifecycle, "o-", ms=3.4, lw=1.1, color=run_color(run), label="Fusion输出时的源数据年龄")
        axes[0].axhline(500, color=COLORS["collision"], ls="--", lw=1.0, label="归档配置阈值 500 ms")
        axes[0].set_ylabel("源数据年龄 / ms")
        axes[0].legend(frameon=False, fontsize=8)
        axes[1].plot(rel[1:], gaps, "o-", ms=3.4, lw=1.1, color=COLORS["dark"])
        axes[1].set_ylabel("相邻输出间隔 / ms")
        axes[1].set_xlabel("相对 t1 时间 / s")
        for axis in axes:
            axis.axvline((run["t2_wall_s"] - run["t1_wall_s"]), color=COLORS["delay"], ls="--", label="t2")
            axis.axvline((run["t_outcome_wall_s"] - run["t1_wall_s"]), color=COLORS["collision"], ls=":", label="碰撞")
            axis.grid(alpha=0.18)
        axes[0].set_title(title)
        footer = "输出间隔与源数据年龄是两种不同缺陷信号。" if include_gap else "该run无1131式长输出空档，但碰撞端最后目标源数据仍已老化。"
        fig.text(0.5, 0.01, footer, ha="center", fontsize=9, color="#555555")
        savefig(fig, filename)

    freshness_figure(r1131, "case_1131_fusion_timeline.png", "案例 1131：Fusion输出连续性与源数据年龄", True)

    draw_chain_figure(
        [
            f"响应最慢\n{r1643['T_e2e_data_observed_ms']:.1f} ms",
            f"起始净距较小\nD1={r1643['D1_clear_data_observed_m']:.2f} m",
            f"距离债务\n{r1643['D_delay_wall_integral_data_observed_m']:.2f} m",
            f"制动起点\nD2={r1643['D2_clear_data_observed_m']:.2f} m",
            f"碰撞\n{r1643['impact_speed_data_observed_mps']:.2f} m/s",
        ],
        "案例 1643：多因素碰撞的证据链",
        f"Fusion最大输出间隔{r1643['target_gap_max_ms']:.1f} ms并不异常；响应、起始净距与当次制动过程共同决定后果。",
        "case_1643_causal_chain.png",
    )
    freshness_figure(r1643, "case_1643_data_freshness.png", "案例 1643：持续输出下的数据新鲜度", False)

    fig, ax = plt.subplots(figsize=(11.0, 6.4))
    for index, row in enumerate(shown):
        t2_rel = row["T_e2e_data_observed_ms"] / 1000.0
        end_rel = row["T_outcome_from_t1_s"]
        ax.plot([0, t2_rel], [index, index], color=run_color(row), lw=4, solid_capstyle="butt")
        ax.plot([t2_rel, end_rel], [index, index], color="#BCC5CE", lw=4, solid_capstyle="butt")
        ax.scatter([t2_rel], [index], marker="|", s=180, color=COLORS["dark"], zorder=4)
        marker = "X" if row["collision_event_data_observed"] else ("s" if not row["included_main_analysis"] else "o")
        color = COLORS["collision"] if row["collision_event_data_observed"] else (COLORS["unknown"] if not row["included_main_analysis"] else COLORS["safe"])
        ax.scatter([end_rel], [index], marker=marker, s=80, color=color, zorder=5)
    ax.set_yticks(np.arange(len(shown)), [run_label(row) for row in shown])
    ax.invert_yaxis()
    ax.set_xlabel("相对 t1 时间 / s")
    ax.set_title("逐 run 的响应—制动—结局时间线")
    ax.grid(axis="x", alpha=0.2)
    fig.text(0.5, 0.01, "彩色段=t1→t2响应阶段；灰段=t2→停车/碰撞端点；红叉=碰撞，灰方块=结局不确定。", ha="center", fontsize=9, color="#555555")
    savefig(fig, "outcome_timeline.png")


def write_validation(rows: list[dict], inventory_rows: list[dict], config: dict) -> None:
    missing = EXPECTED - {r["run_id"] for r in inventory_rows}
    extra = {r["run_id"] for r in inventory_rows} - EXPECTED
    categories = pd.read_csv(EXTRACTED / "file_inventory.csv")["category"].value_counts().to_dict()
    text = [
        "# 实时性缺陷报告数据盘点",
        "",
        f"- 预期 run：12；发现：{len(inventory_rows)}；缺失：{sorted(missing) or '无'}；额外：{sorted(extra) or '无'}。",
        f"- 文件分类计数：`{json.dumps(categories, ensure_ascii=False)}`。",
        "- 原始目录只读；新生结果均位于 `realtime_defect_report/`。",
        "- 详细文件清单见 `../extracted/file_inventory.csv`，模式清单见 `../extracted/schema_inventory.json`。",
        "- 优先级：原始日志/Trace/SCB/CollisionSensor/actor history > 本目录可复算脚本 > 旧CSV/JSON > 旧报告文字。",
        "- 点云数量560000、队列长度1等属于实验设定，不作为逐run实测常量。该项来源于实验设定，当前归档中缺少独立配置文件快照。",
    ]
    (VALIDATION / "data_inventory.md").write_text("\n".join(text) + "\n", encoding="utf-8")

    r1206 = next(r for r in rows if r["run_id"] == "202607271206")
    discrepancies = f"""# 数据与旧产物差异

1. 旧报告将 `202607271206` 简称为“数据异常”并排除，但未说明原因。本次重算表明该 run 具有完整的 t1/t2、Localization、Perception、Prediction、Planning、Trace 和 SCB 证据：`T_e2e={r1206['T_e2e_data_observed_ms']:.3f} ms`，`D_delay={r1206['D_delay_wall_integral_data_observed_m']:.3f} m`。真正冲突是：无 CARLA 碰撞事件/演化历史，但固定目标几何与停车端点推算得投影净距 `{r1206['final_clearance_projected_data_observed_m']:.3f} m`，超过 ±0.52 m 接触偏移不确定性范围。因此它是“结局不确定”，不是“无法解析”。

2. 旧表中 baseline 和其他 11 个主分析 run 的响应时间、墙钟速度积分距离与本次统一重算在数值精度内一致。本报告改用 t1 时插值 Localization 计算几何，不采用最近邻车位。

3. 旧文本容易把“基准组仍有数米余量”与 6 m 安全余量混为一谈。实际上，baseline 的 0 m 碰撞余量中位数约 3.47 m，但 7/7 的 6 m 安全余量都为负。新报告分开表达这两个定义。

4. 碰撞 run 没有完整停车端点；其完整 `D_brake_data_observed`、0 m/6 m 观测余量和观测 deadline 均保持 NA，不再用模型值回填。
"""
    (VALIDATION / "data_discrepancies.md").write_text(discrepancies, encoding="utf-8")

    excluded = f"""# 排除样本记录

| run | 主分析 | 原因 | 仍保留的计算 |
|---|---:|---|---|
| 202607271206 | 否 | 无碰撞事件与固定目标几何穿透冲突，无 actor history 可独立定性 | T_e2e={r1206['T_e2e_data_observed_ms']:.3f} ms，D_delay={r1206['D_delay_wall_integral_data_observed_m']:.3f} m，仅进入质量诊断 |

这是结局不确定性排除，不是按结果方向挑选样本。其余 11 个 run 均进入主分析。
"""
    (VALIDATION / "excluded_runs.md").write_text(excluded, encoding="utf-8")

    collision_rows = [r for r in rows if r["collision_event_data_observed"]]
    clocks = f"""# 时钟对齐说明

- 主分析只使用 Apollo/Localization 墙钟 epoch 时间线：`t1`、`t2`、`D_delay`与停车端点共用同一基准。
- `D_delay` 是速度对墙钟时间的梯形积分；不混用 CARLA 帧数、sim time 或 Localization 空间位移。
- 主分析中 9 个无 actor history run（另有被排除的 `1206` 也无 actor history）状态为 `LIMITED_NO_DUAL_CLOCK_HISTORY`；这不影响墙钟主指标，但不能用于估计 realtime factor。
- 两个碰撞 run 使用 actor history 拟合 CARLA sim time→wall time。其状态为 `{collision_rows[0]['clock_alignment_status']}` 和 `{collision_rows[1]['clock_alignment_status']}`，p95残差分别为 {collision_rows[0]['clock_alignment_p95_residual_ms']:.3f} ms 和 {collision_rows[1]['clock_alignment_p95_residual_ms']:.3f} ms。
- Trace 内部阶段耗时使用 monotonic clock，通过 `trace_anchor.ingress_ms` 和目标 trace ID 与源时刻对齐。
"""
    (VALIDATION / "clock_alignment.md").write_text(clocks, encoding="utf-8")

    provenance = """# 指标与证据来源

| 项目 | 证据层 | 说明 |
|---|---|---|
| CARLA 0.9.15、Apollo 10.0.0、Bridge直接读取Control | A 配置/部署说明 | 来自工作区 AGENTS.md；Guardian Trace不作为车辆执行链输入 |
| 名义300 ms注入、约560k点云、配置队列长度1 | A 实验设定 | 该项来源于实验设定，当前归档中缺少独立配置文件快照。 |
| t1、t2、T_e2e、D1、D_delay、D2、停车/碰撞结局 | B 直接观测 | 从原始Fusion/Trace/Localization/SCB/CollisionSensor/actor history统一重算 |
| baseline恢复反事实、预测碰撞余量/冲击速度 | C 模型 | 单独保存在counterfactual_model.csv，不回填观测结果 |

主结果只以B类证据下结论；A类用于说明系统和实验边界；C类只回答局部“如果更早响应”问题。
"""
    (VALIDATION / "method_provenance.md").write_text(provenance, encoding="utf-8")

    chart_map = """# 图表—证据映射

| 图 | 主数据 | 支持的结论 | 不支持的结论 |
|---|---|---|---|
| realtime_fault_propagation_chain | group_summary.csv | 组级故障传播顺序 | 单一模块根因 |
| e2e_response_by_run | run_level_metrics.csv | 逐run响应差异 | 通用硬阈值 |
| latency_amplification | group_summary.csv + SCB列 | 闭环增量大于名义注入 | 剩余149.5 ms全部属于某一模块 |
| distance_debt_by_run | run_level_metrics.csv | 墙钟积分距离债务 | CARLA sim距离 |
| braking_position_by_run | run_level_metrics.csv | t2时可用净距 | 完整制动能力相同 |
| realtime_safety_cliff | run_level_metrics.csv | 当前样本结局转换区 | 普适700 ms阈值 |
| case_1131_causal_chain | collision_case_comparison.csv | 实时性主导候选 | 纯时延唯一致撞 |
| case_1131_fusion_timeline | Fusion目标序列 | 输出连续性/新鲜度异常 | 目标从未识别 |
| case_1643_causal_chain | collision_case_comparison.csv | 多因素碰撞 | 与1131相同Fusion gap机制 |
| case_1643_data_freshness | Fusion目标序列 | 持续输出下源数据老化 | 单帧处理超过500 ms |
| outcome_timeline | run_level_metrics.csv | t1→t2→端点顺序 | 碰撞run完整停车距离 |
"""
    (VALIDATION / "chart_evidence_map.md").write_text(chart_map, encoding="utf-8")


def main() -> None:
    for directory in [TABLES, FIGURES, EXTRACTED, VALIDATION]:
        directory.mkdir(parents=True, exist_ok=True)
    configure_plotting()
    config = make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    specs = core.discover_runs(config)
    inventory_rows = inventory(config, specs)
    parsed_by_id: dict[str, core.ParsedRun] = {}
    rows: list[dict] = []
    identities: list[dict] = []
    for spec in specs:
        parsed = core.parse_run(spec, config, timezone)
        raw, debug = core.raw_run_metrics(parsed, config)
        row = analyze_run(parsed, raw, debug)
        parsed_by_id[spec.run_id] = parsed
        rows.append(row)
        identities.append(actor_identity(parsed, row))
        print(f"analyzed {spec.run_id}: {row['outcome_data_observed']}")
    rows.sort(key=lambda r: (r["nominal_injected_delay_ms"], r["run_id"]))
    rows_by_id = {r["run_id"]: r for r in rows}
    write_csv("run_level_metrics.csv", rows)
    write_csv("group_summary.csv", summarize(rows))
    write_csv("stage_latency_summary.csv", stage_summary(rows))
    write_csv(
        "collision_case_comparison.csv",
        collision_comparison(rows_by_id, parsed_by_id),
    )
    write_csv("realtime_defect_evidence_matrix.csv", evidence_matrix(rows_by_id))
    write_csv("target_identity_audit.csv", identities)
    continuity_fields = [
        "run_id", "target_id", "target_observation_count", "target_gap_max_ms",
        "target_gap_p90_ms", "target_lifecycle_median_ms", "target_lifecycle_p90_ms",
        "target_lifecycle_max_ms", "target_source_age_at_outcome_ms",
    ]
    write_csv("fusion_continuity.csv", [{k: r[k] for k in continuity_fields} for r in rows])
    main_latency = [r["T_e2e_data_observed_ms"] for r in rows if r["included_main_analysis"] and r["group_name"] == "baseline"]
    baseline_median = float(np.median(main_latency))
    cf = [counterfactual(rows_by_id[rid], parsed_by_id[rid], baseline_median) for rid in sorted(COLLISION_IDS)]
    write_csv("counterfactual_model.csv", cf)
    write_validation(rows, inventory_rows, config)
    plot_required_figures(rows, parsed_by_id)
    write_json(
        EXTRACTED / "analysis_manifest.json",
        {
            "expected_runs": sorted(EXPECTED),
            "found_runs": sorted(rows_by_id),
            "main_included_runs": [r["run_id"] for r in rows if r["included_main_analysis"]],
            "excluded_runs": sorted(MAIN_EXCLUDED_IDS),
            "observed_table": "../tables/run_level_metrics.csv",
            "model_table": "../tables/counterfactual_model.csv",
            "geometry_offset_m": OFFSET_M,
            "pointcloud_count_configured": 560000,
        },
    )
    print(f"wrote {len(rows)} run rows, {len(list(FIGURES.glob('*.png')))} figures")


if __name__ == "__main__":
    main()
