#!/usr/bin/env python3
"""TCPS-PA v4.1 single-run diagnosis for second-experiment run 202607271211.

Raw files are read only. Observed quantities are recomputed from this run;
seven baseline runs enter only the explicitly model-tainted deadline branch.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np


WORKSPACE = Path("/Users/huangjinhui/Desktop/萨卡班/data")
RUN_ID = "202607271211"
RUN_DIR = WORKSPACE / "第二次实验/300ms" / RUN_ID
OUTPUT = WORKSPACE / "output/second_experiment_1211_tcps_pa_v4_1"
TABLES = OUTPUT / "tables"
FIGURES = OUTPUT / "figures"
REPORT = OUTPUT / "report"
VALIDATION = OUTPUT / "validation"

PARSER_DIR = WORKSPACE / "report_workspace/scripts"
sys.path.insert(0, str(PARSER_DIR))
import analyze_second_experiment as ase  # noqa: E402
import realtime_collision_core as core  # noqa: E402

SKILL_DIR = Path(
    "/Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis"
)
sys.path.insert(0, str(SKILL_DIR / "scripts"))
from construct_dynamic_deadline import OUTPUT_FIELDS as DEADLINE_FIELDS  # noqa: E402
from construct_dynamic_deadline import construct_row  # noqa: E402


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def fmt(value: object, digits: int = 3) -> str:
    return f"{float(value):.{digits}f}" if finite(value) else "不可用"


def iso(epoch_s: float) -> str:
    return datetime.fromtimestamp(epoch_s, ZoneInfo("Asia/Shanghai")).isoformat(
        timespec="microseconds"
    )


def write_csv(name: str, rows: list[dict], fields: list[str] | None = None) -> None:
    path = TABLES / name
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = []
        seen: set[str] = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    fields.append(key)
                    seen.add(key)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def quantile(values: list[float], probability: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), probability * 100.0))


def distribution(values: list[float]) -> dict[str, float]:
    median = statistics.median(values)
    mad = statistics.median(abs(value - median) for value in values)
    return {
        "n": len(values),
        "median_ms": median,
        "mad_ms": mad,
        "p90_ms": quantile(values, 0.90),
        "p95_ms": quantile(values, 0.95),
        "p99_ms": quantile(values, 0.99),
        "max_ms": max(values),
        "research_threshold_median_plus_6mad_ms": median + 6.0 * mad,
    }


def rss_required(
    tau_s: float,
    v_ego_mps: float,
    a_response_mps2: float,
    b_ego_mps2: float,
    d_safe_m: float,
) -> float:
    return (
        v_ego_mps * tau_s
        + 0.5 * a_response_mps2 * tau_s**2
        + (v_ego_mps + a_response_mps2 * tau_s) ** 2 / (2.0 * b_ego_mps2)
        + d_safe_m
    )


def solve_residual(
    d_clear_m: float,
    v_ego_mps: float,
    a_response_mps2: float,
    b_ego_mps2: float,
    d_safe_m: float,
) -> tuple[float, str]:
    if rss_required(
        0.0, v_ego_mps, a_response_mps2, b_ego_mps2, d_safe_m
    ) > d_clear_m:
        return 0.0, "ALREADY_OUTSIDE_DECLARED_ENVELOPE_AT_SAMPLE"
    low, high = 0.0, 10.0
    for _ in range(100):
        middle = (low + high) / 2.0
        if rss_required(
            middle, v_ego_mps, a_response_mps2, b_ego_mps2, d_safe_m
        ) <= d_clear_m:
            low = middle
        else:
            high = middle
    return low, "FINITE_RESIDUAL_BUDGET_AT_SAMPLE"


def configure_plotting() -> None:
    font = Path("/System/Library/Fonts/PingFang.ttc")
    if not font.exists():
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
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.dpi": 180,
        }
    )


def input_inventory() -> dict:
    files = []
    for path in sorted(item for item in RUN_DIR.rglob("*") if item.is_file()):
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        stat = path.stat()
        files.append(
            {
                "relative_path": str(path.relative_to(RUN_DIR)),
                "absolute_path": str(path),
                "size_bytes": stat.st_size,
                "mtime_ns": stat.st_mtime_ns,
                "sha256": digest.hexdigest(),
            }
        )
    result = {
        "analysis_scope": "single_run_only",
        "run_id": RUN_ID,
        "raw_run_directory": str(RUN_DIR),
        "raw_directory_policy": "read_only",
        "generated_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "file_count": len(files),
        "total_bytes": sum(row["size_bytes"] for row in files),
        "files": files,
    }
    (VALIDATION / "input_inventory.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result


def build() -> dict:
    for directory in (TABLES, FIGURES, REPORT, VALIDATION):
        directory.mkdir(parents=True, exist_ok=True)
    configure_plotting()
    inventory = input_inventory()

    config = ase.make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    specs = [item for item in core.discover_runs(config) if item.run_id == RUN_ID]
    if len(specs) != 1:
        raise RuntimeError(f"Expected one {RUN_ID} run, found {len(specs)}")
    parsed = core.parse_run(specs[0], config, timezone)
    raw, debug = core.raw_run_metrics(parsed, config)
    observed = ase.analyze_run(parsed, raw, debug)
    identity = ase.actor_identity(parsed, observed)

    t_sample = float(raw["t_sensor_origin_s"])
    t_fusion = float(raw["t_perception_stable_output_s"])
    t_prediction = float(raw["t_prediction_first_s"])
    t_stop = float(raw["t_planning_stop_s"])
    t_plan = float(raw["t_planning_decel_s"])
    t_control = float(raw["t_control_brake_command_s"])
    t_phys = float(raw["t_brake_effective_s"])
    t_near_stop = float(debug["near_stop"]["time_s"])
    t_min_speed = float(debug["brake_completion"]["time_s"])
    t_fault = t_sample + float(raw["scb_trigger_relative_t1_s"])
    preceding_phys = max(
        sample.time_s for sample in parsed.localization if sample.time_s < t_phys
    )
    tr_low_ms = (preceding_phys - t_sample) * 1000.0
    tr_high_ms = (t_phys - t_sample) * 1000.0
    d_response_low = core.integrate_speed(parsed.localization, t_sample, preceding_phys)
    d_response = core.integrate_speed(parsed.localization, t_sample, t_phys)
    d_brake_wall = core.integrate_speed(parsed.localization, t_phys, t_min_speed)
    d_brake_displacement = float(raw["empirical_braking_distance_m"])
    m0_observed = float(raw["D1_clear_m"]) - d_response - d_brake_displacement

    stable = parsed.perception["stable"]
    target_rows = list(parsed.perception["target_rows"])
    response_rows = [
        row for row in target_rows if t_sample <= row.header_time_s <= t_phys
    ]
    response_gaps = np.diff([row.header_time_s for row in response_rows]) * 1000.0
    target_gap_pairs = [
        (target_rows[index - 1], target_rows[index])
        for index in range(1, len(target_rows))
    ]
    max_gap_pair = max(
        target_gap_pairs,
        key=lambda pair: pair[1].header_time_s - pair[0].header_time_s,
    )
    max_gap_ms = (
        max_gap_pair[1].header_time_s - max_gap_pair[0].header_time_s
    ) * 1000.0
    pre_phys_latest = response_rows[-1]
    age_phys_ms = (t_phys - pre_phys_latest.obs_time_s) * 1000.0
    age_endpoint_ms = float(observed["target_source_age_at_outcome_ms"])
    lifecycle_ms = [
        (row.header_time_s - row.obs_time_s) * 1000.0 for row in target_rows
    ]

    # The static target exists no later than its source sample, but no CARLA
    # actor history or observer model archives its actual onset/observability.
    t_world_upper = t_sample

    # Trace-exact decomposition of the first stable-target chain.
    event_dir = RUN_DIR / "trace/events"
    event_files = {path.name: path for path in event_dir.glob("*.csv")}
    target_trace = str(parsed.trace["target_trace_id"])
    parent_trace = str(parsed.trace["parent_trace_id"])

    def event_rows(file_name: str) -> list[dict]:
        return read_csv(event_files[file_name])

    def first_phase(file_name: str, phase: str, trace_id: str) -> int:
        values = [
            int(row["mono_ns"])
            for row in event_rows(file_name)
            if row["phase"] == phase and row["trace_id"] == trace_id
        ]
        return min(values)

    phase_files = {
        "preprocess": "perception.pointcloud_preprocess.719157.csv",
        "roi": "perception.pointcloud_map_based_roi.719157.csv",
        "ground": "perception.pointcloud_ground_detection.719157.csv",
        "detection": "perception.lidar_detection.719157.csv",
        "filter": "perception.lidar_detection_filter.719157.csv",
        "tracking": "perception.lidar_tracking.719157.csv",
        "fusion": "perception.multi_sensor_fusion.719157.csv",
        "prediction": "prediction.719147.csv",
        "planning": "planning.719151.csv",
        "control": "control.719153.csv",
    }
    times = {
        "preproc_enter": first_phase(phase_files["preprocess"], "proc_enter", parent_trace),
        "preproc_out": first_phase(phase_files["preprocess"], "output_pub", parent_trace),
        "roi_enter": first_phase(phase_files["roi"], "proc_enter", parent_trace),
        "roi_out": first_phase(phase_files["roi"], "output_pub", parent_trace),
        "ground_enter": first_phase(phase_files["ground"], "proc_enter", parent_trace),
        "ground_out": first_phase(phase_files["ground"], "output_pub", parent_trace),
        "det_enter": first_phase(phase_files["detection"], "proc_enter", parent_trace),
        "det_out": first_phase(phase_files["detection"], "output_pub", parent_trace),
        "filter_enter": first_phase(phase_files["filter"], "proc_enter", parent_trace),
        "filter_out": first_phase(phase_files["filter"], "output_pub", parent_trace),
        "tracking_enter": first_phase(phase_files["tracking"], "proc_enter", parent_trace),
        "tracking_out": first_phase(phase_files["tracking"], "output_pub", parent_trace),
        "fusion_enter": first_phase(phase_files["fusion"], "proc_enter", parent_trace),
        "fusion_out": first_phase(phase_files["fusion"], "output_pub", target_trace),
    }
    segment_specs = [
        ("sensor_to_preprocess_ingress", float(parsed.trace["sensor_anchor"]["ingress_ms"]), "source age before Preprocess entry"),
        ("preprocess_processing", (times["preproc_out"] - times["preproc_enter"]) / 1e6, "Preprocess execution"),
        ("preprocess_to_roi_edge", (times["roi_enter"] - times["preproc_out"]) / 1e6, "edge/readiness delay"),
        ("roi_processing", (times["roi_out"] - times["roi_enter"]) / 1e6, "map ROI execution"),
        ("roi_to_ground_edge", (times["ground_enter"] - times["roi_out"]) / 1e6, "edge/readiness delay"),
        ("ground_processing", (times["ground_out"] - times["ground_enter"]) / 1e6, "Ground Detection execution"),
        ("ground_to_detection_wait", (times["det_enter"] - times["ground_out"]) / 1e6, "wait/queue before Lidar Detection"),
        ("lidar_detection_processing", (times["det_out"] - times["det_enter"]) / 1e6, "Lidar Detection execution"),
        ("detection_to_filter_edge", (times["filter_enter"] - times["det_out"]) / 1e6, "edge/readiness delay"),
        ("filter_processing", (times["filter_out"] - times["filter_enter"]) / 1e6, "Detection Filter execution"),
        ("filter_to_tracking_edge", (times["tracking_enter"] - times["filter_out"]) / 1e6, "edge/readiness delay"),
        ("tracking_processing", (times["tracking_out"] - times["tracking_enter"]) / 1e6, "Lidar Tracking execution"),
        ("tracking_to_fusion_edge", (times["fusion_enter"] - times["tracking_out"]) / 1e6, "edge/readiness delay"),
        ("fusion_processing", (times["fusion_out"] - times["fusion_enter"]) / 1e6, "Multi-Sensor Fusion execution"),
    ]
    perception_decomposition = [
        {
            "run_id": RUN_ID,
            "trace_id": target_trace,
            "parent_trace_id": parent_trace,
            "segment": name,
            "duration_ms": value,
            "semantic": semantic,
            "clock_domain": "monotonic_ns anchored to source wall epoch",
            "evidence_class": "TRACE_LINEAGE",
        }
        for name, value, semantic in segment_specs
    ]
    write_csv("perception_trace_decomposition.csv", perception_decomposition)
    decomposition_sum = sum(float(row["duration_ms"]) for row in perception_decomposition)

    def paired(file_name: str, start_phase: str, end_phase: str) -> tuple[dict[str, float], dict[str, int]]:
        grouped: dict[str, dict[str, list[int]]] = {}
        for row in event_rows(file_name):
            grouped.setdefault(row["trace_id"], {}).setdefault(row["phase"], []).append(
                int(row["mono_ns"])
            )
        values: dict[str, float] = {}
        starts: dict[str, int] = {}
        for trace_id, phases in grouped.items():
            if start_phase in phases and end_phase in phases:
                start = min(phases[start_phase])
                end = min(phases[end_phase])
                if end >= start:
                    values[trace_id] = (end - start) / 1e6
                    starts[trace_id] = start
        return values, starts

    def edge(first_file: str, first_phase_name: str, second_file: str, second_phase_name: str) -> tuple[dict[str, float], dict[str, int]]:
        first: dict[str, list[int]] = {}
        second: dict[str, list[int]] = {}
        for row in event_rows(first_file):
            if row["phase"] == first_phase_name:
                first.setdefault(row["trace_id"], []).append(int(row["mono_ns"]))
        for row in event_rows(second_file):
            if row["phase"] == second_phase_name:
                second.setdefault(row["trace_id"], []).append(int(row["mono_ns"]))
        values: dict[str, float] = {}
        starts: dict[str, int] = {}
        for trace_id in first.keys() & second.keys():
            start = min(first[trace_id])
            end = min(second[trace_id])
            if end >= start:
                values[trace_id] = (end - start) / 1e6
                starts[trace_id] = start
        return values, starts

    anchor_ns = float(parsed.trace["sensor_anchor"]["preproc_enter_ns"])
    anchor_wall = t_sample + float(parsed.trace["sensor_anchor"]["ingress_ms"]) / 1000.0

    def mono_to_wall(mono_ns: int) -> float:
        return anchor_wall + (float(mono_ns) - anchor_ns) / 1e9

    local_metrics: list[tuple[str, dict[str, float], dict[str, int], str]] = []
    local_metrics.append(
        (
            "ground_to_lidar_detection_wait",
            *edge(phase_files["ground"], "output_pub", phase_files["detection"], "proc_enter"),
            parent_trace,
        )
    )
    local_metrics.append(
        (
            "lidar_detection_processing",
            *paired(phase_files["detection"], "proc_enter", "output_pub"),
            parent_trace,
        )
    )
    local_metrics.append(
        (
            "planning_runonce",
            *paired(phase_files["planning"], "runonce_enter", "runonce_exit"),
            target_trace,
        )
    )
    anomaly_rows: list[dict] = []
    for metric_name, values, starts, selected_trace in local_metrics:
        stats = distribution(list(values.values()))
        selected = values[selected_trace]
        start_wall = mono_to_wall(starts[selected_trace])
        event_window = (
            "PRE_SAMPLE"
            if start_wall < t_sample
            else "INITIAL_RESPONSE"
            if start_wall <= t_phys
            else "BRAKING_TO_NEAR_STOP"
            if start_wall <= t_near_stop
            else "AFTER_NEAR_STOP"
        )
        anomaly_rows.append(
            {
                "run_id": RUN_ID,
                "metric_name": metric_name,
                "trace_id": selected_trace,
                "event_window": event_window,
                "start_wall_s": start_wall,
                "relative_t_sample_ms": (start_wall - t_sample) * 1000.0,
                "observed_ms": selected,
                **stats,
                "ratio_to_median": selected / stats["median_ms"],
                "research_anomaly_verdict": (
                    "OUTLIER_RESEARCH_ONLY"
                    if selected > stats["research_threshold_median_plus_6mad_ms"]
                    else "WITHIN_RESEARCH_SCREEN"
                ),
                "provenance_class": "RESEARCH",
                "qualification": "RESEARCH_ONLY_NOT_A_CONTRACT",
            }
        )
    gap_stats = distribution(
        [
            (target_rows[index].header_time_s - target_rows[index - 1].header_time_s)
            * 1000.0
            for index in range(1, len(target_rows))
        ]
    )
    anomaly_rows.append(
        {
            "run_id": RUN_ID,
            "metric_name": "target_fusion_output_gap",
            "trace_id": max_gap_pair[1].trace_id,
            "event_window": "BRAKING_TO_NEAR_STOP",
            "start_wall_s": max_gap_pair[0].header_time_s,
            "relative_t_sample_ms": (max_gap_pair[0].header_time_s - t_sample) * 1000.0,
            "observed_ms": max_gap_ms,
            **gap_stats,
            "ratio_to_median": max_gap_ms / gap_stats["median_ms"],
            "research_anomaly_verdict": "OUTLIER_RESEARCH_ONLY",
            "provenance_class": "RESEARCH",
            "qualification": "RESEARCH_ONLY_NOT_A_CONTRACT",
        }
    )
    write_csv("local_timing_anomaly_screen.csv", anomaly_rows)
    ground_wait = next(
        row for row in anomaly_rows if row["metric_name"] == "ground_to_lidar_detection_wait"
    )

    onset_rows = []
    for smoothed in (False, True):
        for threshold in (0.3, 0.5, 1.0):
            result = core.detect_brake_onset(
                parsed.localization,
                t_sample,
                t_control,
                threshold,
                config,
                smoothed=smoothed,
            )
            onset_rows.append(
                {
                    "run_id": RUN_ID,
                    "speed_filter": "median3" if smoothed else "raw",
                    "deceleration_threshold_mps2": threshold,
                    "status": result.get("status", "UNKNOWN"),
                    "t_phys_wall_s": result.get("onset_time_s", ""),
                    "T_R_sample_ms": (
                        (float(result["onset_time_s"]) - t_sample) * 1000.0
                        if finite(result.get("onset_time_s"))
                        else ""
                    ),
                    "onset_speed_mps": result.get("onset_speed_mps", ""),
                    "attribution": result.get("attribution", result.get("reason", "")),
                }
            )
    write_csv("effect_threshold_sensitivity.csv", onset_rows)
    write_csv("t2_sensitivity.csv", onset_rows)

    # SCB record: the one archived APPLIED command predates this event, so its
    # measured delay characterizes the persistent injector but is not an
    # event-local command lineage link.
    scb_path = Path(raw["source_scb_file"])
    scb_rows = read_csv(scb_path)
    scb_applied = next(row for row in scb_rows if row.get("status") == "APPLIED")
    scb_apply_wall = float(scb_applied["apply_wall_time_unix_ns"]) / 1e9
    proxy_residual_ms = float(raw["control_to_effective_brake_ms"]) - float(
        raw["scb_actual_wall_delay_ms"]
    )

    observed.update(
        {
            "time_basis_main": "wall_epoch_s",
            "t_world_status": "LEFT_CENSORED_NO_LATER_THAN_SAMPLE",
            "t_world_upper_wall_s": t_world_upper,
            "t_demand_status": "NOT_TESTABLE_NO_INDEPENDENT_DEMAND_PREDICATE",
            "t_demand_wall_s": "",
            "t_observable_status": "NOT_TESTABLE_MISSING_OBSERVER_MODEL",
            "t_observable_wall_s": "",
            "t_sample_wall_s": t_sample,
            "P_OBSERVABILITY": "NOT_TESTABLE_MISSING_OBSERVER_MODEL",
            "T_R_demand_data_observed_ms": "",
            "T_R_sample_lower_bracket_ms": tr_low_ms,
            "T_R_sample_upper_bracket_ms": tr_high_ms,
            "D_response_wall_integral_data_observed_m": d_response,
            "D_response_sample_wall_integral_data_observed_m": d_response,
            "D_response_demand_wall_integral_data_observed_m": "",
            "D_response_lower_bracket_wall_integral_data_observed_m": d_response_low,
            "D_delay_wall_integral_data_observed_m": d_response,
            "D_brake_data_observed_m": d_brake_displacement,
            "D_brake_wall_integral_diagnostic_m": d_brake_wall,
            "M_collision_0m_data_observed_m": m0_observed,
            "M_safety_6m_data_observed_m": m0_observed - 6.0,
            "outcome_data_observed": "NEAR_STOP_NO_COLLISION_FILE",
            "outcome_endpoint_type_data_observed": "MINIMUM_SPEED_PROXY",
            "t_outcome_wall_s": t_min_speed,
            "outcome_endpoint_speed_data_observed_mps": debug["brake_completion"]["sample"].speed_mps,
            "collision_event_data_observed": "UNAVAILABLE_NO_COLLISION_SENSOR_FILE",
            "collision_event_absence_interpretation": "NO_FILE_IS_NOT_DIRECT_NONCOLLISION_PROOF",
            "record_profile_available": False,
            "record_missing_reason": "NO_SAME_RUN_RECORD_OR_PARSED_RECORD_EXPORT",
            "target_output_count_response_window": len(response_rows),
            "data_age_target_at_t2_data_observed_ms": age_phys_ms,
            "data_age_target_at_outcome_data_observed_ms": age_endpoint_ms,
            "update_gap_target_response_window_max_data_observed_ms": float(np.max(response_gaps)),
            "update_gap_target_full_window_max_data_observed_ms": max_gap_ms,
            "target_lifecycle_p90_data_observed_ms": float(np.percentile(lifecycle_ms, 90)),
            "primary_dynamic_deadline_available": False,
            "primary_dynamic_deadline_missing_reason": "NO_T_DEMAND; CALIBRATION_NOT_LOCKED_BY_RUN; DOMAIN_NOT_VALIDATED",
            "D_debt_requirement_constrained_derived_m": "",
            "guarantee_status": "NOT_ESTABLISHED",
            "missing_reason": "",
        }
    )
    write_csv("run_level_observed.csv", [observed])

    velocity_rows = [
        {
            "run_id": RUN_ID,
            "sample_index": index,
            "t_wall_s": sample.time_s,
            "speed_mps": sample.speed_mps,
            "clock_domain": "wall_epoch_s",
            "source_file": sample.source_file,
            "source_locator": f"line {sample.source_line}",
            "availability": "AVAILABLE",
            "quality_flags": "",
        }
        for index, sample in enumerate(parsed.localization)
    ]
    write_csv("velocity_trajectory_observed.csv", velocity_rows)

    # Four sample-origin model scenarios. These are not observed requirements.
    baseline_path = (
        WORKSPACE
        / "output/second_experiment_baseline_dynamic_parameters/tables/baseline_parameter_sets_model_predicted.csv"
    )
    baseline_sets = read_csv(baseline_path)
    d_nominal = float(raw["D1_clear_m"])
    geometry_half_width = 0.52
    d_low, d_high = d_nominal - geometry_half_width, d_nominal + geometry_half_width
    v_sample = float(raw["t1_speed_mps"])
    model_rows: list[dict] = []
    dynamic_rows: list[dict] = []
    for parameter in baseline_sets:
        set_id = parameter["parameter_set_id"]
        a_response = float(parameter["a_resp_model_predicted_mps2"])
        b_ego = float(parameter["b_e_model_predicted_mps2"])
        d_safe = float(parameter["d_safe_requirement_m"])
        tau_low, state_low = solve_residual(d_low, v_sample, a_response, b_ego, d_safe)
        tau_center, state_center = solve_residual(d_nominal, v_sample, a_response, b_ego, d_safe)
        tau_high, state_high = solve_residual(d_high, v_sample, a_response, b_ego, d_safe)
        deadline_low = t_sample + tau_low
        deadline_center = t_sample + tau_center
        deadline_high = t_sample + tau_high
        debt_low = core.integrate_speed(parsed.localization, deadline_low, t_phys) if deadline_low < t_phys else 0.0
        debt_center = core.integrate_speed(parsed.localization, deadline_center, t_phys) if deadline_center < t_phys else 0.0
        debt_center_effect_low = core.integrate_speed(parsed.localization, deadline_center, preceding_phys) if deadline_center < preceding_phys else 0.0
        debt_high = core.integrate_speed(parsed.localization, deadline_high, t_phys) if deadline_high < t_phys else 0.0
        required_zero = rss_required(0.0, v_sample, a_response, b_ego, d_safe)
        required_observed = rss_required(tr_high_ms / 1000.0, v_sample, a_response, b_ego, d_safe)
        verdict = (
            "CLEARLY_MISSED_MODEL_SUPPORTED_ONLY"
            if tr_low_ms > tau_high * 1000.0
            else "BOUNDARY_UNCERTAIN_MODEL_SUPPORTED_ONLY"
        )
        model_rows.append(
            {
                "run_id": RUN_ID,
                "parameter_set_id": set_id,
                "analysis_status_model_predicted": "COMPUTED_DIAGNOSTIC_ONLY",
                "model_name": "RSS_LIKE_STATIC_OBSTACLE_SAMPLE_RESIDUAL",
                "model_origin": "t_sample",
                "t_sample_wall_s": t_sample,
                "d_clear_low_m": d_low,
                "d_clear_center_m": d_nominal,
                "d_clear_high_m": d_high,
                "geometry_sensitivity_half_width_m": geometry_half_width,
                "v_ego_at_sample_mps": v_sample,
                "a_response_model_predicted_mps2": a_response,
                "b_ego_model_predicted_mps2": b_ego,
                "d_safe_requirement_m": d_safe,
                "tau_residual_low_model_predicted_ms": tau_low * 1000.0,
                "tau_residual_center_model_predicted_ms": tau_center * 1000.0,
                "tau_residual_high_model_predicted_ms": tau_high * 1000.0,
                "sample_to_phys_observed_low_ms": tr_low_ms,
                "sample_to_phys_observed_high_ms": tr_high_ms,
                "model_contract_verdict": verdict,
                "D_debt_model_predicted_at_tau_low_m": debt_low,
                "D_debt_model_predicted_center_effect_low_m": debt_center_effect_low,
                "D_debt_model_predicted_center_m": debt_center,
                "D_debt_model_predicted_at_tau_high_m": debt_high,
                "required_distance_zero_delay_model_predicted_m": required_zero,
                "required_distance_observed_delay_model_predicted_m": required_observed,
                "margin_at_observed_delay_model_predicted_m": d_nominal - required_observed,
                "Delta_M_phys_model_predicted_m": required_observed - required_zero,
                "envelope_state_low": state_low,
                "envelope_state_center": state_center,
                "envelope_state_high": state_high,
                "calibration_run_ids": parameter["calibration_run_ids"],
                "evaluation_run_ids": RUN_ID,
                "calibration_evaluation_disjoint": "TRUE",
                "provenance_class": "CALIBRATED_DYNAMICS_PLUS_RESEARCH_DSAFE",
                "parameter_evidence_class": "UNVALIDATED_MODEL_CALIBRATION",
                "qualification": "MODEL_SUPPORTED_ONLY_LOCK_TIME_INVALID_AND_DOMAIN_NOT_VALIDATED",
                "reason": "Disjoint baseline calibration exists, but it was not locked by the evaluated run and its micro-ODD domain is unvalidated.",
            }
        )
        dynamic_rows.append(
            {
                "construction_id": f"DDL.1211.SAMPLE_RESIDUAL.{set_id}",
                "event_id": "EV.STATIC_OBSTACLE.1211",
                "origin_role": "SAMPLE",
                "origin_wall_s": t_sample,
                "demand_origin_available": "FALSE",
                "method": "RSS_LIKE_STATIC_OBSTACLE",
                "state_source": "OFFLINE_EVALUATION_RECONSTRUCTION_AT_SOURCE_EPOCH",
                "state_cutoff_wall_s": t_sample,
                "post_sample_outcome_used": "FALSE",
                "d_clear_low_m": d_low,
                "d_clear_center_m": d_nominal,
                "d_clear_high_m": d_high,
                "geometry_sensitivity_half_width_m": geometry_half_width,
                "v_ego_mps": v_sample,
                "v_front_mps": 0.0,
                "a_response_mps2": a_response,
                "b_ego_mps2": b_ego,
                "d_safe_m": d_safe,
                "tau_low_ms": tau_low * 1000.0,
                "tau_center_ms": tau_center * 1000.0,
                "tau_high_ms": tau_high * 1000.0,
                "deadline_low_wall_s": deadline_low,
                "deadline_center_wall_s": deadline_center,
                "deadline_high_wall_s": deadline_high,
                "monotonicity": "PROVEN_MONOTONE_IN_RESPONSE_DELAY_FOR_POSITIVE_PARAMETERS",
                "calibration_run_ids": parameter["calibration_run_ids"],
                "evaluation_run_ids": RUN_ID,
                "provenance_class": "CALIBRATED_DYNAMICS_PLUS_RESEARCH_DSAFE",
                "qualification": "MODEL_SUPPORTED_ONLY",
                "primary_use": "PROHIBITED_NO_T_DEMAND_AND_INVALID_LOCK_TIME",
            }
        )
    write_csv("run_level_model_predicted.csv", model_rows)
    write_csv("dynamic_contract_construction.csv", dynamic_rows)

    deadline_seed = {field: "" for field in DEADLINE_FIELDS}
    deadline_seed.update(
        {
            "construction_id": "DDL.1211.PRIMARY",
            "requirement_id": "REQ.DYN.1211",
            "run_id_or_group": RUN_ID,
            "method": "RSS_LIKE_LONGITUDINAL",
            "method_version": "TCPS-PA-v4.1",
            "state_time": t_sample,
            "state_time_basis": "wall_epoch_s",
            "state_available_by_t1": "TRUE",
            "input_cutoff_time": t_sample,
            "latest_input_time": t_sample,
            "parameter_selection_locked_by_t1": "FALSE",
            "current_run_post_t1_data_used": "FALSE",
            "current_run_outcome_used": "FALSE",
            "d_clear_m": d_nominal,
            "v_ego_mps": v_sample,
            "v_front_mps": 0.0,
            "target_motion_assumption": "static target in Apollo Prediction",
            "road_condition_assumption": "not archived",
            "validation_dataset_independent": "FALSE",
            "validation_scope": "NONE",
            "evaluation_run_ids": RUN_ID,
            "braking_envelope_status": "MISSING",
            "source_evidence_ids": "EV.TARGET.1211|EV.REACTION.1211",
            "notes": "No independent t_demand. Baseline parameters remain a post-run, unvalidated sample-origin model diagnostic.",
        }
    )
    deadline_row = construct_row(deadline_seed)
    write_csv("dynamic_deadline_construction.csv", [deadline_row], DEADLINE_FIELDS)
    write_csv(
        "requirement_registry.csv",
        [
            {
                "requirement_id": "REQ.DYN.1211",
                "run_id_or_group": RUN_ID,
                "requirement_name": "prospective scenario-dependent physical reaction deadline",
                "requirement_value": "",
                "unit": "ms",
                "requirement_provenance": "demand-origin primary requirement unavailable",
                "pre_registered": "FALSE",
                "external_or_internal": "NONE",
                "safety_meaning": "would bound demand to sustained physical deceleration",
                "deadline_type": "DYNAMIC_CONSTRUCTED",
                "evidence_class": "MISSING",
                "tau_req_low_ms": "",
                "tau_req_center_ms": "",
                "tau_req_high_ms": "",
                "validation_scope": "NONE",
                "p_deadline_qualification": "NOT_QUALIFIED_PRIMARY",
                "notes": "Sample-origin model rows are not promoted to this primary requirement.",
            }
        ],
    )

    event_semantics = [
        {
            "event_id": "EV.STATIC_OBSTACLE.1211",
            "run_id": RUN_ID,
            "time_role": "t_world",
            "predicate": "the physical/static obstacle exists in the ego path",
            "status": "LEFT_CENSORED_NO_LATER_THAN_SAMPLE",
            "t_low_wall_s": "",
            "t_center_wall_s": "",
            "t_high_wall_s": t_world_upper,
            "clock_domain": "wall/source epoch",
            "evidence_class": "OBSERVED_DERIVED",
            "source_evidence_ids": "EV.TIMESEM.1211",
            "limitations": "No CARLA actor history; existence is inferred no later than the stable source sample.",
        },
        {
            "event_id": "EV.STATIC_OBSTACLE.1211",
            "run_id": RUN_ID,
            "time_role": "t_demand",
            "predicate": "the physical state first requires a safety-relevant response",
            "status": "NOT_TESTABLE_NO_INDEPENDENT_DEMAND_PREDICATE",
            "t_low_wall_s": "",
            "t_center_wall_s": "",
            "t_high_wall_s": "",
            "clock_domain": "not established",
            "evidence_class": "MISSING",
            "source_evidence_ids": "EV.TIMESEM.1211",
            "limitations": "Planning STOP and t_phys are downstream events and do not substitute for t_demand.",
        },
        {
            "event_id": "EV.STATIC_OBSTACLE.1211",
            "run_id": RUN_ID,
            "time_role": "t_observable",
            "predicate": "target becomes observable under a qualified sensor/observer model",
            "status": "NOT_TESTABLE_MISSING_OBSERVER_MODEL",
            "t_low_wall_s": "",
            "t_center_wall_s": "",
            "t_high_wall_s": "",
            "clock_domain": "not established",
            "evidence_class": "MISSING",
            "source_evidence_ids": "EV.OBSERVABILITY.1211",
            "limitations": "FOV, occlusion, range, resolution and residence predicates are absent.",
        },
        {
            "event_id": "EV.STATIC_OBSTACLE.1211",
            "run_id": RUN_ID,
            "time_role": "t_sample",
            "predicate": "first source frame of the first three-frame stable target-12 sequence",
            "status": "OBSERVED_SOURCE_EPOCH_RETROSPECTIVELY_SELECTED",
            "t_low_wall_s": t_sample,
            "t_center_wall_s": t_sample,
            "t_high_wall_s": t_sample,
            "clock_domain": "Apollo source wall epoch",
            "evidence_class": "TRACE_LINEAGE",
            "source_evidence_ids": "EV.CHAIN.1211",
            "limitations": "The stable sequence is detected retrospectively; t_sample is not renamed t_demand.",
        },
    ]
    write_csv("event_time_semantics_audit.csv", event_semantics)
    write_csv(
        "observability_audit.csv",
        [
            {
                "event_id": "EV.STATIC_OBSTACLE.1211",
                "run_id": RUN_ID,
                "observer": "velodyne64 + Apollo Perception",
                "observable_state_variables": "target position, extent and static state",
                "fov_model": "MISSING",
                "occlusion_model": "MISSING",
                "range_resolution_model": "MISSING",
                "minimum_residence_predicate": "MISSING",
                "t_observable_status": "NOT_TESTABLE",
                "t_sample_status": "OBSERVED_SOURCE_EPOCH",
                "p_observability_verdict": "NOT_TESTABLE",
                "source_evidence_ids": "EV.OBSERVABILITY.1211|EV.CHAIN.1211",
                "notes": "Only sample-relative analysis is eligible.",
            }
        ],
    )

    threshold_rows = [
        {
            "threshold_id": "TH.ARCH.BRIDGE.300MS",
            "metric_name": "Bridge delay request",
            "value_or_rule": "300 ms",
            "provenance_class": "ARCHITECTURAL",
            "source": str(scb_path),
            "locked_before_run": "TRUE",
            "validation_scope": "Bridge injector configuration",
            "allowed_claim_level": "local injection fidelity only",
        },
        {
            "threshold_id": "TH.RESEARCH.MEDIAN6MAD",
            "metric_name": "within-run anomaly screen",
            "value_or_rule": "median + 6*MAD",
            "provenance_class": "RESEARCH",
            "source": str(TABLES / "local_timing_anomaly_screen.csv"),
            "locked_before_run": "FALSE",
            "validation_scope": "descriptive same-run instances",
            "allowed_claim_level": "research anomaly, not contract miss",
        },
        {
            "threshold_id": "TH.CAL.BASELINE.DYNAMICS",
            "metric_name": "a_response and b_ego envelope",
            "value_or_rule": "central/conservative baseline parameter sets",
            "provenance_class": "CALIBRATED",
            "source": str(baseline_path),
            "locked_before_run": "FALSE",
            "validation_scope": "seven disjoint baseline runs; domain not validated",
            "allowed_claim_level": "MODEL_SUPPORTED_ONLY",
        },
        {
            "threshold_id": "TH.RESEARCH.DSAFE",
            "metric_name": "residual safety margin",
            "value_or_rule": "0 m / 6 m scenarios",
            "provenance_class": "RESEARCH",
            "source": str(baseline_path),
            "locked_before_run": "FALSE",
            "validation_scope": "sensitivity only",
            "allowed_claim_level": "model scenario only",
        },
    ]
    write_csv("threshold_provenance_registry.csv", threshold_rows)

    event_timeline = [
        ("fault_first_archived_receive", t_fault, "Bridge receive/trigger of first archived delayed brake command", "DIRECT_OBSERVED", str(scb_path), "sequence=1"),
        ("t_sample", t_sample, "source epoch of first stable target sequence frame", "TRACE_LINEAGE", stable.source_file, f"line {stable.source_line}; trace={target_trace}"),
        ("Fusion", t_fusion, "stable target output", "TRACE_LINEAGE", stable.source_file, f"trace={target_trace}"),
        ("Prediction", t_prediction, "static-target output", "TRACE_LINEAGE", parsed.prediction["source_file"], f"trace={target_trace}"),
        ("Planning_STOP", t_stop, "STOP for target 12", "OBSERVED_DERIVED", raw["source_planning_file"], f"trace={target_trace}"),
        ("Planning_output", t_plan, "nonempty fallback trajectory output", "OBSERVED_DERIVED", raw["source_planning_file"], "planning seq=640"),
        ("Control_output", t_control, "same-trace Control command output", "TRACE_LINEAGE", parsed.trace["source_files"]["control_context"], f"trace={target_trace}"),
        ("t_phys", t_phys, "first sustained effective-deceleration sample", "OBSERVED_DERIVED", raw["source_localization_file"], "raw detector threshold 0.5 m/s^2"),
        ("target_gap_start", max_gap_pair[0].header_time_s, "largest target Fusion gap begins", "OBSERVED_DERIVED", stable.source_file, f"trace={max_gap_pair[0].trace_id}"),
        ("target_gap_end", max_gap_pair[1].header_time_s, "largest target Fusion gap ends", "OBSERVED_DERIVED", stable.source_file, f"trace={max_gap_pair[1].trace_id}"),
        ("near_stop", t_near_stop, "speed <= near-stop threshold", "OBSERVED_DERIVED", raw["source_localization_file"], "near-stop detector"),
        ("minimum_speed_proxy", t_min_speed, "post-response minimum-speed endpoint", "OBSERVED_DERIVED", raw["source_localization_file"], "minimum speed sample"),
    ]
    write_csv(
        "event_timeline.csv",
        [
            {
                "run_id": RUN_ID,
                "event": name,
                "t_wall_s": time_value,
                "t_iso": iso(time_value),
                "relative_t_sample_ms": (time_value - t_sample) * 1000.0,
                "semantic": semantic,
                "evidence_class": evidence_class,
                "clock_domain": "wall_epoch_s",
                "source_file": source,
                "source_locator": locator,
            }
            for name, time_value, semantic, evidence_class, source, locator in event_timeline
        ],
    )
    stage_rows = [
        ("source_to_Fusion", float(raw["sensor_to_perception_ms"]), "reaction", "same target trace"),
        ("Fusion_to_Prediction", float(raw["perception_to_prediction_ms"]), "reaction", "same target trace"),
        ("Prediction_to_Planning_STOP", float(raw["prediction_to_planning_stop_ms"]), "reaction", "same target trace"),
        ("Planning_STOP_to_Control", float(raw["planning_stop_to_control_ms"]), "reaction", "same target trace"),
        ("Control_to_t_phys", float(raw["control_to_effective_brake_ms"]), "reaction", "Grade C physical association"),
        ("target_age_at_t_phys", age_phys_ms, "age", "latest target source before t_phys"),
        ("target_gap_response_max", float(np.max(response_gaps)), "gap", "within sample-to-physical response"),
        ("target_gap_full_max", max_gap_ms, "gap", "during braking; research outlier"),
    ]
    write_csv(
        "stage_timing_and_freshness.csv",
        [
            {
                "run_id": RUN_ID,
                "stage": stage,
                "value_ms": value,
                "semantic_type": semantic_type,
                "clock_domain": "wall/trace anchored to wall",
                "lineage_grade": "A" if "same target" in note else "C",
                "interpretation": note,
            }
            for stage, value, semantic_type, note in stage_rows
        ],
    )
    write_csv(
        "target_freshness_timeline.csv",
        [
            {
                "run_id": RUN_ID,
                "target_id": row.obstacle_id,
                "sequence_index": index,
                "trace_id": row.trace_id,
                "source_time_s": row.obs_time_s,
                "output_time_s": row.header_time_s,
                "lifecycle_ms": (row.header_time_s - row.obs_time_s) * 1000.0,
                "relative_t_sample_ms": (row.header_time_s - t_sample) * 1000.0,
                "event_window": "BEFORE_T_PHYS" if row.header_time_s <= t_phys else "BRAKING_OR_LATER",
                "source_file": row.source_file,
                "source_locator": f"line {row.source_line}",
            }
            for index, row in enumerate(target_rows)
        ],
    )
    write_csv(
        "target_identity_audit.csv",
        [
            {
                **identity,
                "p_target_verdict": "PARTIAL",
                "limitation": "Apollo target-12 lineage is continuous, but no CARLA actor history/collision file provides physical actor ground truth.",
            }
        ],
    )

    write_csv(
        "temporal_fault_signature.csv",
        [
            {
                "run_id": RUN_ID,
                "fault_type": "FIXED_DELAY",
                "injection_location": "Bridge Control-command to CARLA apply path",
                "requested_magnitude": "300 ms",
                "actual_magnitude": f"{float(raw['scb_actual_wall_delay_ms']):.6f} ms",
                "actual_distribution": "one archived APPLIED event; persistent injector semantics",
                "fault_onset_wall": t_fault,
                "fault_end_wall": "",
                "t1_wall": t_sample,
                "trigger_relative_t1_s": raw["scb_trigger_relative_t1_s"],
                "duration": "NOT_ARCHIVED",
                "persistent_or_transient": "PERSISTENT_AFTER_TRIGGER_BY_IMPLEMENTATION",
                "one_shot_or_repeated": "REPEATED_COMMAND_PATH; ONE EVENT LOGGED",
                "affected_channel": "Control command consumed directly by Bridge; Guardian excluded",
                "affected_message_count": "AT_LEAST_ONE; TOTAL_NOT_ARCHIVED",
                "queue_behavior": "queue_depth=1 at archived trigger",
                "drop_status": "UNKNOWN",
                "reorder_status": "UNKNOWN",
                "evidence_class": "DIRECT_OBSERVED",
                "confidence": "HIGH",
            }
        ],
    )
    prehazard = []
    for variable, value_t1, causal_role, note in [
        ("V1", raw["t1_speed_mps"], "POST_TREATMENT_STATE", "fault predates available event state by 23.021 s"),
        ("D1", raw["D1_clear_m"], "POST_TREATMENT_STATE", "target is unavailable at fault onset"),
        ("acceleration_mps2", "", "UNKNOWN", "localization starts after fault onset"),
        ("heading_rad", "", "UNKNOWN", "localization starts after fault onset"),
        ("route_progress", "", "UNKNOWN", "not archived as a comparable scalar"),
    ]:
        prehazard.append(
            {
                "run_id": RUN_ID,
                "state_variable": variable,
                "window_start_wall": t_fault,
                "window_end_wall": t_sample,
                "value_at_fault": "",
                "value_at_t1": value_t1,
                "delta": "",
                "source_file": raw["source_localization_file"],
                "availability": "PARTIAL_T1_ONLY" if finite(value_t1) else "MISSING",
                "causal_role": causal_role,
                "evidence_class": "OBSERVED_DERIVED" if finite(value_t1) else "MISSING",
                "confidence": "LOW",
                "notes": note,
            }
        )
    write_csv("pre_hazard_state_audit.csv", prehazard)

    fallback = parsed.planning["fallback_evidence"]
    first_fallback = fallback["speed_fallback"]["first"]
    functional_row = {
        "run_id": RUN_ID,
        "physical_target_identity": "UNKNOWN_NO_CARLA_ACTOR_TRUTH",
        "perception_target_present": "PASS",
        "perception_tracking_continuity": "DEGRADED_188MS_GAP",
        "prediction_target_present": "PASS",
        "prediction_semantics_valid": "PASS_STATIC_27_OF_27",
        "planning_stop_present": "PASS",
        "planning_stop_target_correct": "PASS_TARGET_12_WITHIN_APOLLO",
        "planning_stop_location_reasonable": "UNKNOWN_NO_PHYSICAL_ACTOR_TRUTH",
        "planning_trajectory_valid": "DEGRADED_NONEMPTY_CONSTANT_DECEL_FALLBACK",
        "planning_fallback_status": f"DEGRADED_{raw['planning_constant_deceleration_fallback_count']}_EVENTS",
        "control_received_relevant_trajectory": "PASS_TRACE_ID",
        "control_braking_command_present": "PASS",
        "control_command_continuity": "UNKNOWN_NO_RECORD",
        "bridge_payload_received": "UNKNOWN_EVENT_LOCAL_PAYLOAD_NOT_ARCHIVED",
        "bridge_payload_applied": "PARTIAL_PERSISTENT_INJECTOR_ONE_PRIOR_APPLIED_ROW",
        "physical_response_observed": "PASS",
        "p_func_verdict": "PARTIAL",
        "confidence": "MEDIUM",
        "source_evidence_ids": "EV.TARGET.1211|EV.FUNC.1211|EV.CHAIN.1211|EV.REACTION.1211",
        "notes": "Relevant STOP chain exists, but speed optimization failed and Control/Bridge continuity is not archived.",
    }
    write_csv("functional_correctness_audit.csv", [functional_row])
    clock_row = {
        "run_id_or_group": RUN_ID,
        "clock_domain": "Apollo wall epoch + Orin monotonic trace; Bridge wall/monotonic only for prior APPLIED event",
        "host": "Orin and CARLA/Bridge server",
        "timestamp_type": "source/header/log wall epoch and process monotonic",
        "sync_method": "UNVERIFIED_NO_DUAL_CLOCK_HISTORY",
        "offset_estimate_ms": "",
        "offset_bound_ms": "",
        "drift_estimate_ppm": "",
        "dispersion_or_sync_distance_ms": "",
        "alignment_residual_ms": "",
        "timestamp_resolution_ms": 0.001,
        "measurement_window": f"{iso(t_sample)} to {iso(t_min_speed)}",
        "source_evidence_ids": "EV.CLOCK.1211",
        "confidence": "LOW",
        "p_clock_verdict": "NOT_TESTABLE_CROSS_HOST",
        "notes": "Same-host source-to-Control ordering is trace anchored; event-local Bridge-to-Orin alignment is unavailable.",
    }
    write_csv("clock_alignment_audit.csv", [clock_row])
    phase_row = {
        "run_id_or_group": RUN_ID,
        "producer_period_ms": "approximately 100",
        "consumer_period_ms": "approximately 10 for Control",
        "bridge_tick_period_ms": 100.0,
        "phase_definition": "relative offsets among CARLA tick, Perception, Planning, Control and injector release",
        "phase_bins_or_offsets": "NOT_SCANNED",
        "scan_performed": "FALSE",
        "matched_repeats_per_phase": 0,
        "phase_effect_metric": "T_R_sample_ms",
        "phase_effect_estimate": "",
        "uncertainty_interval": "",
        "phase_effect_verdict": "NOT_TESTABLE",
        "source_evidence_ids": "EV.PHASE.1211",
        "p_phase_verdict": "NOT_TESTABLE",
        "notes": "One run cannot isolate periodic phase effects.",
    }
    write_csv("phase_audit.csv", [phase_row])
    write_csv(
        "clock_phase_audit.csv",
        [
            {
                "run_id_or_group": RUN_ID,
                "clock_domain": clock_row["clock_domain"],
                "host": clock_row["host"],
                "timestamp_type": clock_row["timestamp_type"],
                "sync_method": clock_row["sync_method"],
                "confidence": "LOW",
                "phase_scan_performed": "FALSE",
                "phase_effect_verdict": "NOT_TESTABLE",
                "notes": "Legacy combined view; canonical audits are separated.",
            }
        ],
    )

    component_registry = [
        ("K.ARCH.R.BRIDGE.1211", "ARCHITECTURAL", "R", "Bridge requested-to-apply delay", "ControlCommand_to_CARLA_apply", "300 ms configured request", "TH.ARCH.BRIDGE.300MS"),
        ("K.PHYS.R.DEMAND.1211", "PHYSICAL", "R", "demand-to-physical response", "t_demand_to_t_phys", "qualified dynamic physical deadline", "REQ.DYN.1211"),
        ("K.MODEL.R.SAMPLE.1211", "MODEL", "R", "sample-to-physical response", "t_sample_to_t_phys", "baseline residual model scenarios", "TH.CAL.BASELINE.DYNAMICS"),
        ("K.ARCH.A.TARGET.1211", "ARCHITECTURAL", "A", "target source age at t_phys", "target12_at_t_phys", "no declared age threshold", ""),
        ("K.ARCH.G.TARGET.1211", "ARCHITECTURAL", "G", "target Fusion output gap", "target12_stream", "no declared gap threshold", ""),
        ("K.PHYS.C.BUNDLE.1211", "PHYSICAL", "C", "induced state error of Fusion input bundle", "target12_bundle", "no induced-state-error envelope", ""),
        ("K.ARCH.L.CLOSED_LOOP.1211", "ARCHITECTURAL", "L", "Closed-Loop Timing Integrity", "target12_source_to_physical", "ordered lineage + continuity + valid effect", ""),
    ]
    write_csv(
        "component_contract_registry.csv",
        [
            {
                "contract_id": cid,
                "contract_family": family,
                "component": component,
                "metric_name": metric,
                "scope_id": scope,
                "criterion": criterion,
                "threshold_or_requirement_id": threshold,
                "run_id": RUN_ID,
            }
            for cid, family, component, metric, scope, criterion, threshold in component_registry
        ],
    )
    component_eval = [
        ("K.ARCH.R.BRIDGE.1211", raw["scb_actual_wall_delay_ms"], "ms", "WITHIN_CONFIGURED_REQUEST_TOLERANCE", "DIRECT_OBSERVED", "EV.FAULT.1211", "Local injector fidelity, not an end-to-end safety verdict"),
        ("K.PHYS.R.DEMAND.1211", "", "ms", "NOT_TESTABLE", "MISSING", "EV.NODEADLINE.1211", "t_demand and qualified primary deadline unavailable"),
        ("K.MODEL.R.SAMPLE.1211", f"[{tr_low_ms:.3f},{tr_high_ms:.3f}]", "ms", "MODEL_SUPPORTED_MISS_ALL_DECLARED_SCENARIOS", "UNVALIDATED_MODEL", "EV.MODELDEADLINE.1211", "Model sensitivity only"),
        ("K.ARCH.A.TARGET.1211", age_phys_ms, "ms", "NOT_TESTABLE_NO_ARCHITECTURAL_LIMIT", "OBSERVED_DERIVED", "EV.AGEGAP.1211", "Diagnostic age"),
        ("K.ARCH.G.TARGET.1211", max_gap_ms, "ms", "RESEARCH_OUTLIER_NO_ARCHITECTURAL_LIMIT", "OBSERVED_DERIVED", "EV.AGEGAP.1211", "Occurs after t_phys"),
        ("K.PHYS.C.BUNDLE.1211", "raw skew retained separately", "status", "NOT_TESTABLE_NO_INDUCED_STATE_ERROR_MODEL", "TRACE_LINEAGE", "EV.COHERENCE.1211", "Raw skew cannot establish Physical Coherence"),
        ("K.ARCH.L.CLOSED_LOOP.1211", "fallback + 188.254 ms gap + missing event-local apply", "status", "PARTIAL_DEGRADED", "OBSERVED_DERIVED", "EV.FUNC.1211|EV.AGEGAP.1211", "Closed-loop integrity is incomplete/degraded"),
    ]
    write_csv(
        "component_contract_evaluation.csv",
        [
            {
                "contract_id": cid,
                "run_id": RUN_ID,
                "observed_value": value,
                "unit": unit,
                "verdict": verdict,
                "evidence_class": evidence_class,
                "source_evidence_ids": evidence_ids,
                "interpretation": interpretation,
            }
            for cid, value, unit, verdict, evidence_class, evidence_ids, interpretation in component_eval
        ],
    )
    write_csv(
        "physical_coherence_audit.csv",
        [
            {
                "run_id": RUN_ID,
                "bundle_id": "BUNDLE.FUSION.TARGET12.1211",
                "raw_source_skew_ms": "DIAGNOSTIC_ONLY",
                "induced_state_error": "",
                "induced_state_error_unit": "",
                "physical_error_envelope": "",
                "physical_coherence_verdict": "NOT_TESTABLE_NO_INDUCED_STATE_ERROR_MODEL",
                "architectural_coherence_verdict": "NOT_TESTABLE_NO_ARCHITECTURAL_SKEW_LIMIT",
                "source_evidence_ids": "EV.COHERENCE.1211",
                "notes": "Physical Coherence is not inferred from raw timestamp skew.",
            }
        ],
    )

    evidence_fields = [
        "evidence_id", "run_id", "layer", "metric", "value", "unit",
        "evidence_class", "clock_domain", "source_file", "source_locator",
        "availability", "confidence", "supports_claim_ids", "challenges_claim_ids",
        "limitations", "semantic_role", "reference_type", "distribution_scope",
        "causal_lineage_grade",
    ]
    evidence = [
        {
            "evidence_id": "EV.CLOCK.1211", "run_id": RUN_ID, "layer": "P_CLOCK",
            "metric": "cross_host_clock_alignment", "value": "unavailable", "unit": "status",
            "evidence_class": "MISSING", "clock_domain": "cross-host", "source_file": str(TABLES / "clock_alignment_audit.csv"),
            "source_locator": "single scoped audit row", "availability": "MISSING", "confidence": "HIGH",
            "supports_claim_ids": "P_CLOCK.1211", "challenges_claim_ids": "P_CLOCK.1211|C3.1211|C7.1211",
            "limitations": "no dual-clock history or event-local Bridge apply mapping", "semantic_role": "CLOCK_ALIGNMENT_GAP",
        },
        {
            "evidence_id": "EV.PHASE.1211", "run_id": RUN_ID, "layer": "P_PHASE",
            "metric": "active_phase_scan", "value": "not performed", "unit": "status",
            "evidence_class": "MISSING", "clock_domain": "mixed", "source_file": str(TABLES / "phase_audit.csv"),
            "source_locator": "single scoped phase row", "availability": "MISSING", "confidence": "HIGH",
            "supports_claim_ids": "P_PHASE.1211", "challenges_claim_ids": "P_PHASE.1211|C7.1211",
            "limitations": "one run cannot isolate phase", "semantic_role": "PHASE_AUDIT_GAP",
        },
        {
            "evidence_id": "EV.TARGET.1211", "run_id": RUN_ID, "layer": "P_TARGET",
            "metric": "Apollo_target12_lineage", "value": "27 Fusion/Prediction/Planning target observations", "unit": "events",
            "evidence_class": "TRACE_LINEAGE", "clock_domain": "Apollo wall + trace", "source_file": stable.source_file,
            "source_locator": f"target=12; first trace={target_trace}", "availability": "AVAILABLE", "confidence": "MEDIUM",
            "supports_claim_ids": "P_TARGET.1211|C3.1211", "challenges_claim_ids": "P_TARGET.1211|C6.1211|C7.1211",
            "limitations": "no CARLA actor truth", "semantic_role": "TARGET_LINEAGE", "causal_lineage_grade": "A_WITHIN_APOLLO_ONLY",
        },
        {
            "evidence_id": "EV.FAULT.1211", "run_id": RUN_ID, "layer": "L1/L2",
            "metric": "bridge_fixed_delay_actual_wall_ms", "value": raw["scb_actual_wall_delay_ms"], "unit": "ms",
            "evidence_class": "DIRECT_OBSERVED", "clock_domain": "Bridge wall/monotonic/CARLA frame", "source_file": str(scb_path),
            "source_locator": "APPLIED row sequence=1; requested=300 ms; 3 CARLA frames", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "C1.1211|C2.1211|C7.1211", "challenges_claim_ids": "",
            "limitations": "archived APPLIED event predates target event; later commands not individually logged", "semantic_role": "TEMPORAL_DISTURBANCE_APPLICATION",
            "reference_type": "CONFIGURED_REQUEST", "distribution_scope": "ONE_APPLIED_EVENT_WITH_PERSISTENT_INJECTOR_SEMANTICS",
        },
        {
            "evidence_id": "EV.REACTION.1211", "run_id": RUN_ID, "layer": "L3/L4",
            "metric": "T_e2e_data_observed_ms", "value": tr_high_ms, "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "Apollo wall epoch", "source_file": raw["source_localization_file"],
            "source_locator": "t_sample source epoch to sustained-deceleration t_phys", "availability": "AVAILABLE", "confidence": "MEDIUM",
            "supports_claim_ids": "C3.1211|C4.1211", "challenges_claim_ids": "",
            "limitations": f"effect bracket [{tr_low_ms:.3f},{tr_high_ms:.3f}] ms; demand origin missing", "semantic_role": "PHYSICAL_REACTION_INTERVAL",
        },
        {
            "evidence_id": "EV.CHAIN.1211", "run_id": RUN_ID, "layer": "L3",
            "metric": "sensor_to_control_trace_lineage_ms", "value": raw["sensor_to_control_ms"], "unit": "ms",
            "evidence_class": "TRACE_LINEAGE", "clock_domain": "Orin monotonic anchored to source epoch", "source_file": parsed.trace["source_files"]["control_context"],
            "source_locator": f"trace={target_trace}; parent={parent_trace}", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "P_OBSERVABILITY.1211|P_FUNC.1211|C3.1211|C7.1211", "challenges_claim_ids": "",
            "limitations": "strict lineage ends at Control; Bridge/physical suffix is not event-local", "semantic_role": "CAUSE_EFFECT_LINEAGE",
            "causal_lineage_grade": "C_FULL_CHAIN_A_SOFTWARE_PREFIX",
        },
        {
            "evidence_id": "EV.AGEGAP.1211", "run_id": RUN_ID, "layer": "L2/L3",
            "metric": "target_age_and_output_gap", "value": f"A_tphys={age_phys_ms:.3f};G_response={float(np.max(response_gaps)):.3f};G_full={max_gap_ms:.3f}", "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "Apollo wall epoch", "source_file": stable.source_file,
            "source_locator": "27 target observations", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "C3.1211", "challenges_claim_ids": "P_FUNC.1211|C7.1211",
            "limitations": "no architectural A/G threshold; max gap occurs after t_phys", "semantic_role": "FRESHNESS_CONTINUITY_DIAGNOSTIC",
        },
        {
            "evidence_id": "EV.LOCALWAIT.1211", "run_id": RUN_ID, "layer": "L2/L3",
            "metric": "ground_to_lidar_detection_wait_ms", "value": ground_wait["observed_ms"], "unit": "ms",
            "evidence_class": "TRACE_LINEAGE", "clock_domain": "Orin monotonic", "source_file": str(event_files[phase_files["detection"]]),
            "source_locator": f"parent_trace={parent_trace}", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "C3.1211", "challenges_claim_ids": "",
            "limitations": "median+6MAD is a research screen, not an architectural contract", "semantic_role": "LOCAL_TIMING_ANOMALY",
            "reference_type": "WITHIN_RUN_DISTRIBUTION", "distribution_scope": f"n={ground_wait['n']} matched edges",
        },
        {
            "evidence_id": "EV.FUNC.1211", "run_id": RUN_ID, "layer": "P_FUNC",
            "metric": "planning_constant_deceleration_fallback_count", "value": raw["planning_constant_deceleration_fallback_count"], "unit": "events",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "Planning log wall epoch", "source_file": raw["source_planning_file"],
            "source_locator": f"first relevant fallback line {first_fallback['source_line']}", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "P_FUNC.1211", "challenges_claim_ids": "P_FUNC.1211|C7.1211",
            "limitations": "fallback output is nonempty but prevents functional qualification", "semantic_role": "FUNCTIONAL_DEGRADATION",
        },
        {
            "evidence_id": "EV.DRESPONSE.1211", "run_id": RUN_ID, "layer": "L5",
            "metric": "D_response_wall_integral_data_observed_m", "value": d_response, "unit": "m",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "wall_epoch_s", "source_file": raw["source_localization_file"],
            "source_locator": "trapezoid integral of speed over [t_sample,t_phys]", "availability": "AVAILABLE", "confidence": "HIGH",
            "supports_claim_ids": "C5.1211", "challenges_claim_ids": "",
            "limitations": "sample-relative response distance is not deadline debt", "semantic_role": "OBSERVED_RESPONSE_DISTANCE",
        },
        {
            "evidence_id": "EV.OUTCOME.1211", "run_id": RUN_ID, "layer": "L6",
            "metric": "final_clearance_projected_data_observed_m", "value": raw["final_clearance_m"], "unit": "m",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "Localization wall epoch + stable Fusion geometry", "source_file": raw["source_localization_file"],
            "source_locator": f"minimum-speed proxy at {iso(t_min_speed)}", "availability": "AVAILABLE", "confidence": "MEDIUM",
            "supports_claim_ids": "C6.1211|C7.1211", "challenges_claim_ids": "C6.1211|C7.1211",
            "limitations": "no CollisionSensor/actor-history file; projected clearance has geometry uncertainty", "semantic_role": "PHYSICAL_OUTCOME",
        },
        {
            "evidence_id": "EV.NODEADLINE.1211", "run_id": RUN_ID, "layer": "P_DEADLINE/L4",
            "metric": "qualified_dynamic_deadline", "value": "unavailable", "unit": "status",
            "evidence_class": "MISSING", "clock_domain": "not established", "source_file": str(TABLES / "dynamic_deadline_construction.csv"),
            "source_locator": "DDL.1211.PRIMARY", "availability": "MISSING", "confidence": "HIGH",
            "supports_claim_ids": "P_DEADLINE.1211|C4.1211", "challenges_claim_ids": "P_DEADLINE.1211|C4.1211|C5.1211|C7.1211",
            "limitations": "no primary tau_req, guarantee-loss time or primary D_debt", "semantic_role": "DEADLINE_QUALIFICATION_GAP",
        },
        {
            "evidence_id": "EV.TIMESEM.1211", "run_id": RUN_ID, "layer": "P_OBSERVABILITY/L4",
            "metric": "four_event_time_semantics", "value": "world left-censored; demand/observable unavailable; sample observed", "unit": "status",
            "evidence_class": "OBSERVED_DERIVED", "clock_domain": "wall/source epoch", "source_file": str(TABLES / "event_time_semantics_audit.csv"),
            "source_locator": "EV.STATIC_OBSTACLE.1211 four rows", "availability": "PARTIAL", "confidence": "HIGH",
            "supports_claim_ids": "P_OBSERVABILITY.1211", "challenges_claim_ids": "P_DEADLINE.1211|C4.1211|C5.1211",
            "limitations": "no t_demand/t_observable", "semantic_role": "EVENT_TIME_SEMANTICS",
        },
        {
            "evidence_id": "EV.OBSERVABILITY.1211", "run_id": RUN_ID, "layer": "P_OBSERVABILITY",
            "metric": "qualified_observer_spec", "value": "missing", "unit": "status",
            "evidence_class": "MISSING", "clock_domain": "not established", "source_file": str(TABLES / "observability_audit.csv"),
            "source_locator": "velodyne64 observer audit", "availability": "MISSING", "confidence": "HIGH",
            "supports_claim_ids": "", "challenges_claim_ids": "P_OBSERVABILITY.1211|C4.1211",
            "limitations": "demand-to-sample exposure cannot be allocated", "semantic_role": "OBSERVABILITY_QUALIFICATION_GAP",
        },
        {
            "evidence_id": "EV.MODELDEADLINE.1211", "run_id": RUN_ID, "layer": "L4_MODEL_DIAGNOSTIC",
            "metric": "sample_origin_residual_budget_sensitivity", "value": "four declared scenarios", "unit": "ms",
            "evidence_class": "UNVALIDATED_MODEL", "clock_domain": "sample-relative wall epoch", "source_file": str(TABLES / "dynamic_contract_construction.csv"),
            "source_locator": "four baseline parameter-set rows", "availability": "AVAILABLE", "confidence": "MEDIUM",
            "supports_claim_ids": "C4.1211", "challenges_claim_ids": "P_DEADLINE.1211|C4.1211|C7.1211",
            "limitations": "post-run lock; domain unvalidated; not demand-origin", "semantic_role": "MODEL_PREDICTED_DEADLINE_DIAGNOSTIC",
        },
        {
            "evidence_id": "EV.COHERENCE.1211", "run_id": RUN_ID, "layer": "L3_C",
            "metric": "bundle_induced_state_error", "value": "unavailable", "unit": "status",
            "evidence_class": "TRACE_LINEAGE", "clock_domain": "source epoch", "source_file": parsed.trace["source_files"]["fusion_inputs"],
            "source_locator": f"fusion trace={target_trace}", "availability": "PARTIAL", "confidence": "MEDIUM",
            "supports_claim_ids": "C3.1211", "challenges_claim_ids": "C7.1211",
            "limitations": "raw skew is diagnostic; no induced-state-error model", "semantic_role": "PHYSICAL_COHERENCE_DIAGNOSTIC",
        },
    ]
    write_csv("evidence_ledger.csv", evidence, evidence_fields)

    claim_rows: list[dict] = []

    def claim(
        claim_id: str, layer: str, proposition: str, prereqs: str,
        required: str, support: str, challenge: str, defeaters: str,
        verdict: str, confidence: str, ceiling: str, level: int,
        residual: str, allowed: str, forbidden: str, criterion: str,
        next_gate: str, lineage: str = "",
    ) -> None:
        base = claim_id.split(".", 1)[0]
        claim_rows.append(
            {
                "claim_id": claim_id, "layer": layer, "run_id_or_group": RUN_ID,
                "proposition": proposition, "prerequisite_claim_ids": prereqs,
                "required_evidence_classes": required, "supporting_evidence_ids": support,
                "challenging_evidence_ids": challenge, "defeater_ids": defeaters,
                "inference_rule_id": f"IR-{base}", "verdict": verdict,
                "confidence": confidence, "confidence_ceiling": ceiling,
                "maximum_claim_level": level, "residual_uncertainty": residual,
                "allowed_language": allowed, "forbidden_language": forbidden,
                "causal_lineage_grade": lineage, "gate_inputs": support,
                "gate_metrics": criterion, "admissible_evidence": required,
                "gate_criterion": criterion, "next_gate_condition": next_gate,
            }
        )

    claim("P_CLOCK.1211", "P_CLOCK", "Cross-host timestamps are qualified for event-local subtraction.", "", "", "EV.CLOCK.1211", "EV.CLOCK.1211", "", "NOT_TESTABLE", "NONE", "NONE", 1, "No dual-clock history/event-local apply mapping.", "Same-host trace ordering is usable; cross-host suffix timing is unqualified.", "Cross-host timestamps are exact.", "bounded cross-host clock audit", "Collect PTP/NTP residuals and dual timestamps for event-local commands.")
    claim("P_PHASE.1211", "P_PHASE", "Periodic phase sensitivity is tested and isolated.", "", "", "EV.PHASE.1211", "EV.PHASE.1211", "", "NOT_TESTABLE", "NONE", "NONE", 1, "No phase scan or matched repeats.", "Phase remains an unresolved alternative.", "Phase is the root cause.", "controlled phase scan", "Collect matched repeats over phase offsets.")
    claim("P_OBSERVABILITY.1211", "P_OBSERVABILITY", "Demand observability and the first causal sample are independently qualified.", "P_CLOCK.1211", "TRACE_LINEAGE", "EV.TIMESEM.1211|EV.CHAIN.1211", "EV.OBSERVABILITY.1211", "", "NOT_TESTABLE", "NONE", "NONE", 1, "t_sample exists; t_demand/t_observable do not.", "Only sample-relative timing is qualified.", "t_sample equals t_demand.", "observer model plus source epoch", "Provide FOV/occlusion/range/residence model and independent demand predicate.")
    claim("P_TARGET.1211", "P_TARGET", "The relevant Apollo target identity is continuous and physically anchored.", "", "TRACE_LINEAGE", "EV.TARGET.1211", "EV.TARGET.1211", "D_TARGET.P_TARGET.1211", "PARTIAL", "MEDIUM", "MEDIUM", 2, "Apollo target 12 is continuous; CARLA actor truth is absent.", "Target 12 is lineage-consistent inside Apollo only.", "Target 12 is proven identical to a CARLA actor.", "same target ID/trace plus physical actor association", "Archive CARLA actor history and CollisionSensor data.")
    claim("P_FUNC.1211", "P_FUNC", "Relevant functional behavior is fully qualified and does not independently explain the low margin.", "P_TARGET.1211", "OBSERVED_DERIVED", "EV.FUNC.1211|EV.CHAIN.1211", "EV.FUNC.1211|EV.AGEGAP.1211", "D_FUNC.P_FUNC.1211", "PARTIAL", "MEDIUM", "MEDIUM", 2, "Speed optimization fallback and payload continuity gaps remain.", "STOP exists, but functional correctness is partial.", "Functionally correct, temporally wrong.", "valid target-correct trajectory and command/apply continuity", "Resolve fallback semantics and archive record/payload continuity.")
    claim("P_DEADLINE.1211", "P_DEADLINE", "An independently qualified prospective dynamic physical deadline is available.", "P_TARGET.1211", "INDEPENDENT_REQUIREMENT", "EV.NODEADLINE.1211", "EV.NODEADLINE.1211|EV.TIMESEM.1211|EV.MODELDEADLINE.1211", "D_DEADLINE.P_DEADLINE.1211", "NOT_TESTABLE", "NONE", "NONE", 1, "No t_demand and model parameters were not prospectively locked/validated.", "Primary deadline is unavailable; sample model sensitivity is separate.", "The run violated a qualified physical deadline.", "qualified prospective deadline interval", "Establish t_demand and prospectively locked, validated braking bounds.")
    claim("C1.1211", "L1", "A configured 300 ms Bridge fixed-delay stressor entered the command path.", "", "DIRECT_OBSERVED", "EV.FAULT.1211", "", "", "PASS", "HIGH", "HIGH", 3, "Only one APPLIED command is individually archived.", "The external Bridge stressor entered the deployed path.", "Apollo has an intrinsic real-time defect.", "APPLIED row with actual delay", "Per-command logging is required for event-local propagation.")
    claim("C2.1211", "L2", "The configured Bridge delay manifested as a measured local delay event.", "C1.1211", "DIRECT_OBSERVED", "EV.FAULT.1211", "", "", "PASS", "HIGH", "HIGH", 3, "A/G have no architectural threshold; research anomalies are separate.", "Local Bridge delay fidelity is established.", "All observed gaps are requirement violations.", "actual delay against configured request", "Add architectural A/G contracts for strong freshness/continuity verdicts.")
    claim("C3.1211", "L3", "The selected target propagates through Control and is temporally associated with physical response.", "C1.1211|C2.1211|P_CLOCK.1211|P_TARGET.1211", "TRACE_LINEAGE", "EV.CHAIN.1211|EV.REACTION.1211|EV.LOCALWAIT.1211|EV.TARGET.1211|EV.COHERENCE.1211", "EV.CLOCK.1211", "D_PAYLOAD.C3.1211", "PARTIAL_PASS", "NONE", "NONE", 2, "Grade A software prefix; cross-host/event-local suffix remains Grade C/unqualified.", "Software propagation and sample-relative physical association are supported.", "The exact Control command caused t_phys.", "same-trace software prefix plus compatible physical endpoint", "Archive event-local Bridge apply and qualify cross-host clocks.", "C")
    claim("C4.1211", "L4", "Observed physical response exceeded a qualified dynamic temporal contract.", "C3.1211|P_DEADLINE.1211", "OBSERVED_DERIVED", "EV.REACTION.1211|EV.NODEADLINE.1211|EV.MODELDEADLINE.1211", "EV.NODEADLINE.1211|EV.TIMESEM.1211|EV.OBSERVABILITY.1211|EV.MODELDEADLINE.1211", "D_DEADLINE.C4.1211", "NOT_TESTABLE", "NONE", "NONE", 1, "Only sample-relative model crossings exist.", "All declared sample-origin model scenarios miss, but primary C4 is not testable.", "The 700.167 ms response is a qualified deadline violation.", "observed demand-relative T_R versus qualified tau_req", "Qualify a demand-origin prospective deadline.")
    claim("C5.1211", "L5", "A requirement-constrained deadline-excess distance debt is established.", "C4.1211", "REQUIREMENT_CONSTRAINED_DERIVED", "EV.DRESPONSE.1211", "EV.NODEADLINE.1211|EV.TIMESEM.1211", "", "NOT_TESTABLE", "NONE", "NONE", 1, "Observed response distance exists; primary deadline debt does not.", "12.0 m is response distance, not primary debt.", "12.0 m is timing-caused safety loss.", "qualified C4 plus post-deadline velocity integral", "Qualify tau_req and recompute primary debt.")
    claim("C6.1211", "L6", "The run reached a low-clearance near-stop physical outcome.", "", "OBSERVED_DERIVED", "EV.OUTCOME.1211", "EV.OUTCOME.1211|EV.TARGET.1211", "D_OUTCOME.C6.1211", "PARTIAL_PASS", "MEDIUM", "MEDIUM", 2, "Near-zero speed and projected clearance are observed-derived; direct collision/noncollision sensor truth is absent.", "A near-stop with about 1 m projected 0 m margin is supported.", "A direct safe-stop outcome is proven by absence of a file.", "compatible Localization endpoint and target geometry", "Archive CollisionSensor and actor history; meet strict stop-hold endpoint.")
    c7_types = {
        "D_INITIAL_CLEARANCE": "INITIAL_CLEARANCE", "D_INITIAL_SPEED": "INITIAL_SPEED",
        "D_BRAKING_CAPABILITY": "BRAKING_CAPABILITY", "D_FUNCTIONAL_FAILURE": "FUNCTIONAL_FAILURE",
        "D_TARGET_MISMATCH": "TARGET_MISMATCH", "D_DATA_FRESHNESS": "DATA_FRESHNESS",
        "D_UPDATE_GAP": "UPDATE_GAP", "D_SOLVER_FALLBACK": "SOLVER_FALLBACK",
        "D_CLOCK": "CLOCK", "D_PHASE": "PHASE", "D_PREHAZARD_STATE": "PREHAZARD_STATE",
        "D_GEOMETRY": "GEOMETRY", "D_OUTCOME_CONFLICT": "OUTCOME_CONFLICT",
    }
    c7_defeaters = "|".join(f"{prefix}.C7.1211" for prefix in c7_types)
    claim("C7.1211", "ATTRIBUTION", "Observed timing anomalies are established as the unique cause and quantified loss of the low-margin outcome.", "C4.1211|C5.1211|C6.1211|P_FUNC.1211", "DIRECT_OBSERVED|TRACE_LINEAGE", "EV.FAULT.1211|EV.CHAIN.1211|EV.OUTCOME.1211", "EV.CLOCK.1211|EV.PHASE.1211|EV.FUNC.1211|EV.AGEGAP.1211|EV.NODEADLINE.1211|EV.MODELDEADLINE.1211|EV.COHERENCE.1211", c7_defeaters, "UNCERTAIN", "NONE", "NONE", 2, "Deadline, cross-host lineage, functionality, phase, geometry and direct outcome truth remain open.", "Timing/functional/physical candidates coexist; unique causation is unresolved.", "Timing uniquely caused the low margin.", "C4+C5+C6 with P_FUNC and closed defeaters", "Collect event-local record/apply, direct actor outcome, validated deadline and controlled repeats.")
    claim_fields = [
        "claim_id", "layer", "run_id_or_group", "proposition", "prerequisite_claim_ids",
        "required_evidence_classes", "supporting_evidence_ids", "challenging_evidence_ids",
        "defeater_ids", "inference_rule_id", "verdict", "confidence", "confidence_ceiling",
        "maximum_claim_level", "residual_uncertainty", "allowed_language", "forbidden_language",
        "causal_lineage_grade", "gate_inputs", "gate_metrics", "admissible_evidence",
        "gate_criterion", "next_gate_condition",
    ]
    write_csv("claim_ledger.csv", claim_rows, claim_fields)
    edges = []
    for row in claim_rows:
        for parent in str(row["prerequisite_claim_ids"]).split("|"):
            if parent:
                edges.append(
                    {
                        "parent_claim_id": parent, "child_claim_id": row["claim_id"],
                        "relation": "REQUIRES", "required": "TRUE",
                        "notes": "canonical TCPS-PA gate dependency",
                    }
                )
    write_csv("claim_edges.csv", edges)

    defeaters = [
        {
            "defeater_id": "D_TARGET.P_TARGET.1211", "claim_id": "P_TARGET.1211",
            "description": "No CARLA actor history or collision truth anchors Apollo target 12 physically.",
            "type": "TARGET_MISMATCH", "evidence_ids": "EV.TARGET.1211", "status": "OPEN",
            "resolution": "Archive actor history and target association.", "residual_risk": "physical identity uncertain",
            "impact_on_claim": "CAPS_AT_PARTIAL", "notes": "Apollo-internal lineage remains usable.",
        },
        {
            "defeater_id": "D_FUNC.P_FUNC.1211", "claim_id": "P_FUNC.1211",
            "description": "Planning speed optimization failed and emitted fallback.", "type": "FUNCTIONAL_FAILURE",
            "evidence_ids": "EV.FUNC.1211", "status": "OPEN", "resolution": "Resolve fallback cause/acceptability.",
            "residual_risk": "functional and temporal factors coexist", "impact_on_claim": "CAPS_AT_PARTIAL", "notes": "",
        },
        {
            "defeater_id": "D_DEADLINE.P_DEADLINE.1211", "claim_id": "P_DEADLINE.1211",
            "description": "No qualified demand-origin deadline.", "type": "DEADLINE",
            "evidence_ids": "EV.NODEADLINE.1211", "status": "OPEN", "resolution": "Qualify t_demand and braking envelope.",
            "residual_risk": "no primary deadline verdict", "impact_on_claim": "INVALIDATES", "notes": "",
        },
        {
            "defeater_id": "D_PAYLOAD.C3.1211", "claim_id": "C3.1211",
            "description": "No event-local Control payload/Bridge apply row for the selected trace.", "type": "PAYLOAD_LINEAGE",
            "evidence_ids": "EV.CHAIN.1211|EV.CLOCK.1211", "status": "OPEN", "resolution": "Enable record and per-command Bridge logging.",
            "residual_risk": "suffix is temporally associated only", "impact_on_claim": "CAPS_AT_PARTIAL", "notes": "",
        },
        {
            "defeater_id": "D_DEADLINE.C4.1211", "claim_id": "C4.1211",
            "description": "Model residual budgets are not qualified requirements.", "type": "DEADLINE",
            "evidence_ids": "EV.NODEADLINE.1211|EV.MODELDEADLINE.1211", "status": "OPEN",
            "resolution": "Prospectively lock/validate deadline construction.", "residual_risk": "model taint",
            "impact_on_claim": "INVALIDATES", "notes": "",
        },
        {
            "defeater_id": "D_OUTCOME.C6.1211", "claim_id": "C6.1211",
            "description": "No direct CollisionSensor/actor-history outcome evidence.", "type": "OUTCOME_CONFLICT",
            "evidence_ids": "EV.OUTCOME.1211", "status": "OPEN", "resolution": "Archive direct outcome files.",
            "residual_risk": "noncollision conclusion is observed-derived", "impact_on_claim": "CAPS_AT_PARTIAL", "notes": "",
        },
    ]
    for prefix, defeater_type in c7_types.items():
        status = "RESOLVED" if prefix == "D_TARGET_MISMATCH" and False else "OPEN"
        evidence_ids = {
            "D_FUNCTIONAL_FAILURE": "EV.FUNC.1211", "D_SOLVER_FALLBACK": "EV.FUNC.1211",
            "D_DATA_FRESHNESS": "EV.AGEGAP.1211", "D_UPDATE_GAP": "EV.AGEGAP.1211",
            "D_CLOCK": "EV.CLOCK.1211", "D_PHASE": "EV.PHASE.1211",
            "D_TARGET_MISMATCH": "EV.TARGET.1211", "D_GEOMETRY": "EV.OUTCOME.1211",
            "D_OUTCOME_CONFLICT": "EV.OUTCOME.1211", "D_PREHAZARD_STATE": "EV.FAULT.1211",
            "D_INITIAL_CLEARANCE": "EV.OUTCOME.1211", "D_INITIAL_SPEED": "EV.REACTION.1211",
            "D_BRAKING_CAPABILITY": "EV.OUTCOME.1211",
        }.get(prefix, "")
        defeaters.append(
            {
                "defeater_id": f"{prefix}.C7.1211", "claim_id": "C7.1211",
                "description": f"Alternative not isolated: {defeater_type}.", "type": defeater_type,
                "evidence_ids": evidence_ids, "status": status,
                "resolution": "Requires targeted measurement/control test.",
                "residual_risk": "unique attribution unavailable", "impact_on_claim": "CAPS_AT_PARTIAL",
                "notes": "Single-run backward diagnosis cannot refute this alternative.",
            }
        )
    write_csv("defeater_ledger.csv", defeaters)

    guarantee_rows = [
        {
            "run_id": RUN_ID,
            "guarantee_status": "NOT_ESTABLISHED",
            "guarantee_loss_time_wall_s": "",
            "reason": "No qualified demand-origin deadline, WCRT/path bound or suffix bound.",
            "model_crossings_available": "TRUE",
            "model_crossings_are_guarantee_loss": "FALSE",
        }
    ]
    write_csv("guarantee_status.csv", guarantee_rows)
    write_csv(
        "empirical_headroom.csv",
        [
            {
                "run_id": RUN_ID,
                "metric": "observed 0 m projected clearance margin",
                "value": m0_observed,
                "unit": "m",
                "evidence_class": "OBSERVED_DERIVED",
                "qualification": "EMPIRICAL_HEADROOM_NOT_DEADLINE_GUARANTEE",
            }
        ],
    )

    diagnosis_fields = [
        "hypothesis_id", "run_id_or_group", "seed_claim_id", "seed_evidence_ids",
        "candidate_layer", "candidate_component", "candidate_fault_type", "hypothesis",
        "path_claim_ids", "supporting_evidence_ids", "challenging_evidence_ids",
        "alternative_hypothesis_ids", "required_prerequisite_claim_ids", "diagnosability_class",
        "equivalence_class_id", "status", "rank_score", "rank_method",
        "maximum_diagnosis_strength", "discriminating_test", "residual_uncertainty",
        "allowed_language", "forbidden_language",
    ]
    diagnoses = [
        ("H1.BRIDGE", "L1/L3", "R", "EXTERNAL_FIXED_DELAY", "Persistent 300 ms Bridge delay consumes most of the Control-to-physical interval.", "EV.FAULT.1211|EV.REACTION.1211", "EV.CLOCK.1211", "H2.SUFFIX|H3.FALLBACK", 0.90, "CONSISTENT_BUT_UNRESOLVED", "LOCALIZED_TO_SEGMENT", "Log every selected event-local command receive/release/apply and align Orin/server clocks."),
        ("H2.SUFFIX", "L3", "R/L", "ACTUATION_OR_SAMPLING_SUFFIX", "The arithmetic 107.419 ms residual contains CARLA tick, dynamics, Localization sampling and hold confirmation.", "EV.REACTION.1211", "EV.CLOCK.1211", "H1.BRIDGE", 0.75, "CONSISTENT_BUT_UNRESOLVED", "LOCALIZED_TO_SEGMENT", "Archive apply completion, actuator feedback and higher-rate physical state."),
        ("H3.FALLBACK", "P_FUNC/L3", "L", "PLANNING_FALLBACK", "Speed optimizer failure and constant-deceleration fallback may alter the commanded braking profile.", "EV.FUNC.1211", "EV.CHAIN.1211", "H1.BRIDGE|H2.SUFFIX", 0.80, "CONSISTENT_BUT_UNRESOLVED", "LOCALIZED_TO_SEGMENT", "Compare record-extracted nominal/fallback trajectories and Control commands under matched state."),
        ("H4.GROUND_WAIT", "L2/L3", "R", "QUEUE_OR_READINESS_WAIT", "The selected chain waits 38.871 ms from Ground output to Lidar Detection entry.", "EV.LOCALWAIT.1211", "", "H5.PREBACKLOG", 0.85, "CONSISTENT_BUT_UNRESOLVED", "LOCALIZED_TO_SEGMENT", "Collect scheduler/queue/resource traces around parent trace and adjacent frames."),
        ("H5.PREBACKLOG", "L2", "R/G", "LIDAR_DETECTION_BACKLOG", "Three preceding Lidar Detection instances take about 226-232 ms and may create pre-sample backlog.", "EV.LOCALWAIT.1211", "", "H4.GROUND_WAIT", 0.70, "CONSISTENT_BUT_UNRESOLVED", "LOCALIZED_TO_SEGMENT", "Correlate DAG queue occupancy and GPU execution with preceding trace IDs."),
        ("H6.POST_GAP", "L2/L3", "G/L", "TARGET_UPDATE_GAP", "A 188.254 ms target output gap degrades braking-phase update continuity but starts after t_phys.", "EV.AGEGAP.1211", "", "H1.BRIDGE|H4.GROUND_WAIT", 0.55, "CONSISTENT_BUT_UNRESOLVED", "DETECTED", "Enable record and identify missing producer/consumer sequence; test physical-command change over the gap."),
    ]
    diagnosis_rows = []
    for hid, layer, component, fault_type, hypothesis, support, challenge, alternatives, score, status, strength, test in diagnoses:
        diagnosis_rows.append(
            {
                "hypothesis_id": hid, "run_id_or_group": RUN_ID, "seed_claim_id": "C6.1211",
                "seed_evidence_ids": "EV.OUTCOME.1211", "candidate_layer": layer,
                "candidate_component": component, "candidate_fault_type": fault_type,
                "hypothesis": hypothesis, "path_claim_ids": "C6.1211|C3.1211|C2.1211|C1.1211",
                "supporting_evidence_ids": support, "challenging_evidence_ids": challenge,
                "alternative_hypothesis_ids": alternatives, "required_prerequisite_claim_ids": "P_CLOCK.1211|P_TARGET.1211|P_FUNC.1211",
                "diagnosability_class": "OBSERVATIONALLY_EQUIVALENT_WITH_MISSING_RECORD",
                "equivalence_class_id": "EQ.1211.TIMING_FUNCTION_PHYSICS", "status": status,
                "rank_score": score, "rank_method": "evidence proximity + temporal compatibility; descriptive only",
                "maximum_diagnosis_strength": strength, "discriminating_test": test,
                "residual_uncertainty": "single run; no qualified deadline/counterfactual",
                "allowed_language": "candidate consistent with observed evidence",
                "forbidden_language": "isolated intrinsic Apollo root cause",
            }
        )
    write_csv("diagnosis_hypothesis_ledger.csv", diagnosis_rows, diagnosis_fields)
    diagnosis_edges = [
        {
            "parent_id": "C6.1211", "child_id": row["hypothesis_id"],
            "relation": "SEEDS_DIAGNOSIS", "time_direction": "BACKWARD_DIAGNOSTIC",
            "required": "TRUE", "notes": "Outcome/low margin seeds hypotheses; it does not prove them.",
        }
        for row in diagnosis_rows
    ]
    write_csv("diagnosis_edges.csv", diagnosis_edges)

    write_csv(
        "record_timing_diagnostics.csv",
        [
            {
                "run_id": RUN_ID, "record_dir": "", "record_evidence_class": "MISSING",
                "l2_reference_qualification": "NOT_TESTABLE", "l3_causal_lineage_grade": "NO_RECORD",
                "availability": "MISSING", "reason": "recording was not enabled for this run",
            }
        ],
    )
    rag_values = [
        ("R", "sample-relative physical response", "T_e2e_data_observed_ms", tr_high_ms, "t_sample to t_phys; demand-relative R unavailable"),
        ("A", "target data age at t_phys", "data_age_target_at_t2_data_observed_ms", age_phys_ms, "latest target source represented before t_phys"),
        ("G", "target output gap in response window", "update_gap_target_response_window_max_data_observed_ms", float(np.max(response_gaps)), "maximum in sample-to-physical window; full-window 188.254 ms separately reported"),
    ]
    write_csv(
        "realtime_rag_summary.csv",
        [
            {
                "dimension": dimension, "metric": metric, "source_column": column,
                "unit": "ms", "semantics": semantics, "group_name": "single_run_1211_descriptive",
                "n_total_runs": 1, "n_available_runs": 1, "p50": value, "p90": value,
                "p95": value, "p99": value, "max": value, "iqr": 0.0,
            }
            for dimension, metric, column, value, semantics in rag_values
        ],
    )
    write_csv(
        "group_summary_observed.csv",
        [
            {
                "scope": "single_run_only_not_group_comparison", "run_id": RUN_ID, "n": 1,
                "T_R_sample_observed_ms": tr_high_ms, "D_response_observed_m": d_response,
                "M0_observed_m": m0_observed, "notes": "Baseline data are confined to model sensitivity.",
            }
        ],
    )
    write_csv(
        "space_budget_decomposition_observed.csv",
        [
            {
                "run_id": RUN_ID, "group_name": "single_run_1211", "included_main_analysis": True,
                "outcome_data_observed": "near_stop_no_collision_file",
                "D1_clear_data_observed_m": d_nominal,
                "D_response_wall_integral_data_observed_m": d_response,
                "D_brake_data_observed_m": d_brake_displacement,
                "M0_recomputed_observed_m": m0_observed,
                "endpoint_compatible_full_stop": "MINIMUM_SPEED_PROXY_NOT_STRICT_STOP",
                "decomposition_scope": "D_response uses wall-speed integral; D_brake uses Localization displacement to the minimum-speed proxy; no model substitution.",
            }
        ],
    )
    write_csv(
        "space_budget_group_decomposition.csv",
        [
            {
                "comparison_group": "single_run_1211_not_comparison", "n": 1,
                "D1_mean_m": d_nominal, "D_response_mean_m": d_response,
                "D_brake_mean_m": d_brake_displacement, "M0_mean_m": m0_observed,
            }
        ],
    )
    method_rows = [
        ("single-run observed scope", "PASS", "observed branch parses only 202607271211"),
        ("raw input inventory and hashes", "PASS", f"{inventory['file_count']} files with SHA-256"),
        ("four event-time semantics", "PARTIAL", "t_sample observed; t_world left-censored; t_demand/t_observable unavailable"),
        ("P_OBSERVABILITY", "NOT_TESTABLE", "missing observer/FOV/occlusion/range model"),
        ("P_CLOCK cross-host audit", "NOT_TESTABLE", "no dual-clock/event-local apply mapping"),
        ("P_PHASE active scan", "MISSING", "one run and no phase intervention"),
        ("target identity continuity", "PARTIAL", "Apollo target 12 continuous; no CARLA actor truth"),
        ("functional correctness qualification", "PARTIAL", "STOP exists; fallback and payload gaps remain"),
        ("same-instance source-to-Control trace", "PASS", f"trace {target_trace}"),
        ("Control-to-physical strict lineage", "PARTIAL", "Grade C association; event-local apply missing"),
        ("component-wise R/A/G/C/L", "PASS", "separate component registry/evaluation"),
        ("Physical Coherence induced state error", "NOT_TESTABLE", "no induced-state-error model"),
        ("primary demand-origin dynamic deadline", "MISSING", "no t_demand; lock time/domain invalid"),
        ("sample-origin residual model", "MODEL_SUPPORTED_ONLY", "four baseline sensitivity scenarios"),
        ("timing guarantee bound", "MISSING", "no WCRT/path/suffix bound"),
        ("canonical D_response wall integral", "PASS", f"{d_response:.6f} m"),
        ("requirement-constrained D_debt", "NOT_TESTABLE", "no qualified tau_req"),
        ("minimum-speed physical space budget", "PARTIAL", f"M0={m0_observed:.3f} m; strict stop/direct actor truth absent"),
        ("direct physical outcome", "MISSING", "no CollisionSensor/actor-history files"),
        ("causal attribution", "UNCERTAIN", "functional/phase/clock/geometry/deadline alternatives open"),
        ("record-derived message audit", "MISSING", "recording was not enabled"),
        ("observed/model separation", "PASS", "separate observed and model tables"),
    ]
    write_csv(
        "method_completeness_matrix.csv",
        [{"requirement": a, "status": b, "evidence_or_gap": c} for a, b, c in method_rows],
    )
    exclusions = [
        ("parsed Apollo record", "MISSING", "recording was not enabled", "payload/receive/send/reuse audit unavailable"),
        ("CARLA CollisionSensor and actor history", "MISSING", "no corresponding files in run", "direct noncollision and physical identity not provable"),
        ("qualified dynamic deadline", "MISSING", "no t_demand and invalid parameter lock/domain", "C4/primary debt/guarantee-loss not testable"),
        ("strict stop hold", "MISSING", "minimum-speed proxy used", "physical endpoint capped at partial"),
        ("event-local Bridge applied command", "MISSING_BY_LOGGING_POLICY", "log_all_delayed_commands=false", "Control-to-physical lineage capped"),
        ("multi-run comparison", "EXCLUDED_BY_SCOPE", "single-run request", "no cross-run anomaly claims"),
    ]
    write_csv(
        "exclusions_and_missing.csv",
        [
            {"run_id": RUN_ID, "item": item, "status": status, "reason": reason, "effect": effect}
            for item, status, reason, effect in exclusions
        ],
    )
    layer_rows = [
        ("L1", "Bridge fixed delay", raw["scb_actual_wall_delay_ms"], "ms", "DIRECT_OBSERVED", str(scb_path), "AVAILABLE"),
        ("L2", "local wait/freshness/gap diagnostics", f"wait={ground_wait['observed_ms']:.3f};A={age_phys_ms:.3f};G={max_gap_ms:.3f}", "mixed", "OBSERVED_DERIVED", str(TABLES / "local_timing_anomaly_screen.csv"), "PARTIAL"),
        ("L3", "source-to-Control lineage", raw["sensor_to_control_ms"], "ms", "TRACE_LINEAGE", parsed.trace["source_files"]["control_context"], "PARTIAL"),
        ("L4", "primary deadline / sample model", "primary missing; sample model only", "status", "UNVALIDATED_MODEL", str(TABLES / "dynamic_contract_construction.csv"), "MODEL_SUPPORTED_ONLY"),
        ("L5", "D_response wall integral", d_response, "m", "OBSERVED_DERIVED", raw["source_localization_file"], "AVAILABLE"),
        ("L6", "minimum-speed projected clearance", raw["final_clearance_m"], "m", "OBSERVED_DERIVED", raw["source_localization_file"], "PARTIAL"),
    ]
    write_csv(
        "layer_evidence_matrix.csv",
        [
            {
                "layer": layer, "metric": metric, "run_id_or_group": RUN_ID,
                "value": value, "unit": unit, "evidence_type": evidence_type,
                "time_basis": "wall_epoch_s" if layer != "L3" else "trace anchored to wall",
                "source_file": source, "availability": availability,
            }
            for layer, metric, value, unit, evidence_type, source, availability in layer_rows
        ],
    )
    write_csv(
        "l5_recomputation.csv",
        [
            {
                "run_id": RUN_ID, "requirement_id": "", "t1_wall_s": t_sample,
                "t_deadline_wall_s": "", "te_wall_s": t_phys,
                "D_response_recomputed_m": "", "D_debt_recomputed_m": "",
                "D_response_reported_m": d_response, "D_debt_reported_m": "",
                "D1_observed_m": d_nominal, "D_brake_observed_m": d_brake_displacement,
                "M0_recomputed_m": m0_observed, "M0_reported_m": m0_observed,
                "endpoint_coverage": "PENDING_VALIDATOR", "max_abs_error_m": "",
                "tolerance_m": 0.02, "recomputation_status": "PENDING_VALIDATOR",
                "notes": "validator overwrites this row",
            }
        ],
    )

    # Figures.
    timeline = read_csv(TABLES / "event_timeline.csv")
    plot_events = [row for row in timeline if row["event"] != "fault_first_archived_receive"]
    x = [float(row["relative_t_sample_ms"]) for row in plot_events]
    labels = [row["event"] for row in plot_events]
    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.hlines(0, min(x) - 50, max(x) + 50, color="#9aa0a6", lw=1.5)
    colors = ["#3B82F6" if value <= tr_high_ms else "#F59E0B" for value in x]
    ax.scatter(x, np.zeros(len(x)), c=colors, s=48, zorder=3)
    for index, (value, label) in enumerate(zip(x, labels)):
        y = 0.18 if index % 2 == 0 else -0.24
        ax.annotate(f"{label}\n{value:.1f} ms", (value, 0), xytext=(value, y),
                    textcoords="data", ha="center", va="center", fontsize=8,
                    arrowprops={"arrowstyle": "-", "color": "#9aa0a6"})
    ax.set_yticks([])
    ax.set_xlabel("relative to t_sample (ms)")
    ax.set_title("1211 event timeline: initial response and braking-phase gap")
    ax.set_ylim(-0.45, 0.40)
    fig.savefig(FIGURES / "event_timeline.png")
    plt.close(fig)

    stage_names = ["source→Fusion", "Fusion→Prediction", "Prediction→STOP", "STOP→Control", "Control→t_phys"]
    stage_values = [float(raw["sensor_to_perception_ms"]), float(raw["perception_to_prediction_ms"]), float(raw["prediction_to_planning_stop_ms"]), float(raw["planning_stop_to_control_ms"]), float(raw["control_to_effective_brake_ms"])]
    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    bars = ax.barh(stage_names, stage_values, color=["#2563EB"] * 4 + ["#DC2626"])
    ax.invert_yaxis()
    for bar, value in zip(bars, stage_values):
        ax.text(value + 5, bar.get_y() + bar.get_height() / 2, f"{value:.3f} ms", va="center")
    ax.axvline(300, color="#F59E0B", ls="--", label="configured Bridge delay 300 ms")
    ax.set_xlabel("duration (ms)")
    ax.set_title("1211 sample-to-physical response-stage decomposition")
    ax.legend()
    fig.savefig(FIGURES / "response_stage_decomposition.png")
    plt.close(fig)

    samples = [sample for sample in parsed.localization if t_sample - 0.5 <= sample.time_s <= t_min_speed + 0.2]
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot([(sample.time_s - t_sample) * 1000 for sample in samples], [sample.speed_mps for sample in samples], color="#1D4ED8", lw=2)
    for time_value, label, color in [(t_control, "Control", "#7C3AED"), (t_phys, "t_phys", "#DC2626"), (t_near_stop, "near stop", "#059669")]:
        ax.axvline((time_value - t_sample) * 1000, color=color, ls="--", label=label)
    ax.axvspan((max_gap_pair[0].header_time_s - t_sample) * 1000, (max_gap_pair[1].header_time_s - t_sample) * 1000, color="#F59E0B", alpha=0.22, label="188.254 ms target gap")
    ax.set_xlabel("relative to t_sample (ms)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("1211 observed wall-clock speed and critical events")
    ax.legend(ncol=4, fontsize=8)
    fig.savefig(FIGURES / "speed_and_events.png")
    plt.close(fig)

    # Report generated from recomputed values and ledgers.
    model_by_id = {row["parameter_set_id"]: row for row in model_rows}
    model_table_lines = []
    for row in model_rows:
        model_table_lines.append(
            f"| {row['parameter_set_id']} | {float(row['tau_residual_center_model_predicted_ms']):.3f} "
            f"[{float(row['tau_residual_low_model_predicted_ms']):.3f}, {float(row['tau_residual_high_model_predicted_ms']):.3f}] | "
            f"{row['model_contract_verdict']} | {float(row['D_debt_model_predicted_center_m']):.3f} | "
            f"{float(row['Delta_M_phys_model_predicted_m']):.3f} |"
        )
    report_text = f"""# 第二次实验 1211 run 实时性异常诊断

**方法：TCPS-PA v4.1；范围：单次 run `202607271211`**  
**observed/data 与 model/predicted 分开；7 个 baseline 只进入模型敏感性分支**

## Six-Layer Inference Status Matrix

| 层/主张 | 判定 | 可支持的结论 |
|---|---|---|
| L1 / C1 | PASS | 300 ms Bridge 固定延时确实进入部署命令路径 |
| L2 / C2 | PASS | 局部 Bridge 延时实测为 {float(raw['scb_actual_wall_delay_ms']):.3f} ms；A/G 无工程契约 |
| L3 / C3 | PARTIAL_PASS | source→Control 同 trace；Control→physical 缺 event-local apply/跨主机时钟闭环 |
| L4 / C4 | NOT_TESTABLE | 无 qualified demand-origin deadline；仅 sample-origin 模型 crossing |
| L5 / C5 | NOT_TESTABLE | $D_{{response}}$ 可观测；主 $D_{{debt}}$ 不可用 |
| L6 / C6 | PARTIAL_PASS | 近零速+约 1 m 投影余量；无直接 CollisionSensor/actor truth |
| Attribution / C7 | UNCERTAIN | 外部时延、局部排队、fallback、物理后缀共存，未唯一隔离 |

## 结论先行

1211 **存在实时性异常，只是没有在当次数据中发展成可直接确认的碰撞**。从 target 12 稳定序列首帧 source epoch $t_{{sample}}$ 到持续有效减速 $t_{{phys}}$，端点夹取为 **[{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms**；上端前车辆仍从 **{float(raw['t1_speed_mps']):.3f} m/s** 升至 **{float(raw['brake_start_speed_mps']):.3f} m/s**，响应阶段墙钟速度梯形积分为 **{d_response:.3f} m**。

与首次响应直接相关的三个问题是：

1. **Perception 入口/排队异常**：目标链 Ground output→Lidar Detection entry 等待 **{float(ground_wait['observed_ms']):.3f} ms**，而同 run {int(ground_wait['n'])} 个可匹配边的中位数只有 **{float(ground_wait['median_ms']):.3f} ms**，约为中位数的 **{float(ground_wait['ratio_to_median']):.1f}倍**，明显超过 research `median+6MAD={float(ground_wait['research_threshold_median_plus_6mad_ms']):.3f} ms`。
2. **Planning 功能退化与时间问题并存**：首个相关目标周期中，Planning 虽然产生 target-12 STOP，但速度优化失败，转入 non-empty constant-deceleration fallback；全 run 计数 **{int(raw['planning_constant_deceleration_fallback_count'])}**。
3. **Control→物理起效段是主要耗时段**：共 **{float(raw['control_to_effective_brake_ms']):.3f} ms**。其中持续 Bridge 注入器的已归档实测延时为 **{float(raw['scb_actual_wall_delay_ms']):.3f} ms**；算术剩余 **{proxy_residual_ms:.3f} ms** 还混合 CARLA tick、车辆动力学、Localization 采样和 effect hold 确认。由于无合格的该段工程上限，除已知 300 ms 外部注入外，不把总值单独判为某个 Apollo 模块超限。

另外，target Fusion 在 $t_{{phys}}$ 后制动段出现 **{max_gap_ms:.3f} ms** 更新缺口（run 内目标间隔中位数 {float(gap_stats['median_ms']):.3f} ms）。它会削弱 Closed-Loop Timing Integrity，但它开始于 $t_{{sample}}+{(max_gap_pair[0].header_time_s-t_sample)*1000:.3f}$ ms，因而**在时间上不能解释首次 $t_{{phys}}$ 为何为 700.167 ms**。

## 四时刻、P_OBSERVABILITY 与端点

| 角色 | 结果 | 含义 |
|---|---|---|
| $t_{{world}}$ | $\le {iso(t_world_upper)}$ | 左截尾；无 CARLA actor history |
| $t_{{demand}}$ | 不可用 | 无独立的物理响应需求 predicate |
| $t_{{observable}}$ | 不可用 | 无 FOV/遮挡/距离/分辨率/驻留模型 |
| $t_{{sample}}$ | {iso(t_sample)} | 稳定 target-12 序列首帧 source epoch，稳定性是事后检测 |
| Fusion | +{(t_fusion-t_sample)*1000:.3f} ms | 同 trace target 12 输出 |
| Planning STOP | +{(t_stop-t_sample)*1000:.3f} ms | STOP 存在，随后 fallback |
| Control | +{(t_control-t_sample)*1000:.3f} ms | 同 trace Control 输出 |
| $t_{{phys}}$ | +[{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms | 首个持续减速样本夹取 |
| near stop | +{(t_near_stop-t_sample)*1000:.3f} ms | {debug['near_stop']['sample'].speed_mps:.6f} m/s |
| minimum-speed proxy | +{(t_min_speed-t_sample)*1000:.3f} ms | {debug['brake_completion']['sample'].speed_mps:.6f} m/s；非严格 stop-hold |

P_OBSERVABILITY 为 `NOT_TESTABLE`；因此需求到可观测、可观测到采样的暴露不能分摊到软件模块。

![event timeline](../figures/event_timeline.png)

## R/A/G/C/L 分量契约集

| 分量 | observed/data | 契约判定 |
|---|---:|---|
| R | sample→physical = [{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms | demand-relative `NOT_TESTABLE`；sample 模型另列 |
| A | $t_{{phys}}$ 的 target source age = {age_phys_ms:.3f} ms | 无 Architectural A threshold，`NOT_TESTABLE` |
| G | 响应窗 max = {float(np.max(response_gaps)):.3f} ms；制动段 max = {max_gap_ms:.3f} ms | 后者为 RESEARCH outlier，不是工程 deadline miss |
| C | induced state error 不可用 | raw skew 只保留为诊断，Physical Coherence `NOT_TESTABLE` |
| L | source→Control 顺序完整；fallback、后段 gap、apply/feedback 缺失 | Closed-Loop Timing Integrity `PARTIAL_DEGRADED` |

## 异常定位

### 首次目标链

source→Fusion 为 **{float(raw['sensor_to_perception_ms']):.3f} ms**，trace 分段求和 **{decomposition_sum:.3f} ms**，与 Fusion E2E {float(parsed.trace['e2e_ms']['fusion']):.3f} ms 一致。其中：

| 段 | 时间/ms | 诊断 |
|---|---:|---|
| source→Preprocess entry | {float(parsed.trace['sensor_anchor']['ingress_ms']):.3f} | 入口数据年龄，不是单模块 execution |
| Preprocess | {dict((a, b) for a, b, _ in segment_specs)['preprocess_processing']:.3f} | run 内常规量级 |
| Ground Detection | {dict((a, b) for a, b, _ in segment_specs)['ground_processing']:.3f} | run 内常规量级 |
| Ground→Lidar Detection wait | **{float(ground_wait['observed_ms']):.3f}** | 清晰的 run 内 RESEARCH outlier |
| Lidar Detection | {dict((a, b) for a, b, _ in segment_specs)['lidar_detection_processing']:.3f} | 本次目标链未超 research 阈值，但是主要执行成本 |
| Filter+Tracking+Fusion suffix | {(dict((a, b) for a, b, _ in segment_specs)['filter_processing'] + dict((a, b) for a, b, _ in segment_specs)['tracking_processing'] + dict((a, b) for a, b, _ in segment_specs)['fusion_processing']):.3f} | 较小 |

在 $t_{{sample}}$ 之前约 0.84–0.15 s，连续三个 Lidar Detection 实例耗时约 226–232 ms，而目标链恢复至 100.329 ms。它与 38.871 ms 等待一起支持“目标采样附近存在 Detection 积压/资源干扰”候选，但无 queue/GPU/scheduler 证据，未能唯一隔离根因。

![response stages](../figures/response_stage_decomposition.png)

### Planning、Bridge 与物理后缀

- Planning STOP 与非空 fallback trajectory 表明系统不是“完全没反应”；但 fallback 阻止 P_FUNC 达到 `QUALIFIED_PASS`。
- 归档 APPLIED 行发生在目标事件前 23.021 s，只能证明持续注入器的延时实现。因 `log_all_delayed_commands=false`，本事件命令的 receive/release/apply 无法逐条串起。
- Guardian 不进入实际执行命令链；当前 Bridge 直接读 Control。

![speed](../figures/speed_and_events.png)

## 动态物理 deadline 模型敏感性

正式 demand-origin deadline 不可构造。以 $t_{{sample}}$ 状态 $d_0={d_nominal:.3f}$ m、$v={v_sample:.3f}$ m/s 建立的四组 RSS-like 剩余预算均为 **model/predicted**；几何带仅采用已归档 offset 不确定性 $\pm0.52$ m，没有伪造 actor 关联误差。

| 参数集 | $\\tau_{{residual}}$ 中心 [low,high] /ms | 与 effect 夹取比较 | 中心 $D_{{debt,model}}$/m | $\Delta M_{{phys,model}}$/m |
|---|---:|---|---:|---:|
{chr(10).join(model_table_lines)}

四组模型在当前几何与 effect 夹取下都给出 `CLEARLY_MISSED_MODEL_SUPPORTED_ONLY`；最宽松的 central/0 m 上界约 {max(float(row['tau_residual_high_model_predicted_ms']) for row in model_rows):.3f} ms，仍早于 effect 下夹取 {tr_low_ms:.3f} ms。这是模型 crossing，不是已建立时间保证的 loss timestamp。

## Space Budget / 空间预算与物理结果

| 量 | data/observed | 性质 |
|---|---:|---|
| $D_1$ | {d_nominal:.3f} m | Fusion/几何派生，offset 不确定性 0.52 m |
| $D_{{response}}$ | **{d_response:.3f} m** | $\int_{{t_{{sample}}}}^{{t_{{phys}}}}vdt_{{wall}}$，主响应距离口径 |
| $D_{{brake}}$ | {d_brake_displacement:.3f} m | t_phys 到 minimum-speed proxy 的 Localization 位移 |
| $D_{{brake,wall}}$ | {d_brake_wall:.3f} m | 独立墙钟速度积分诊断量 |
| $M_0=D_1-D_{{response}}-D_{{brake}}$ | **{m0_observed:.3f} m** | 与旧 observed 结果一致；minimum-speed proxy，非严格 stop-hold |
| final projected clearance | {float(raw['final_clearance_m']):.3f} m | 直接终点几何诊断 |
| 6 m research margin | {m0_observed-6.0:.3f} m | RESEARCH 阈值，不是认证安全要求 |
| 主 $D_{{debt}}$ | 不可用 | 无 qualified $\\tau_{{req}}$ |

数据中没有 CollisionSensor 事件文件和 actor history。Localization 显示车辆降至 0.001335 m/s，并保留约 1 m 投影净距，所以可写为“近零速低余量停车候选”；**不能仅由缺少碰撞文件就把“无碰撞”提升为直接观测事实**。

## 对核心问题的回答

| 问题 | 1211 可支持的回答 |
|---|---|
| 哪里出了实时性问题？ | 目标链 Ground→Lidar Detection 等待尖峰 38.871 ms；Control→physical 段受 300 ms 外部 Bridge 注入而成为 407.567 ms 主耗时段；制动段 target Fusion gap 188.254 ms。 |
| 它是什么性质？ | 外部 Bridge 固定延时 stressor + Perception 局部排队/资源干扰候选 + Planning fallback + 后段更新连续性退化；不是单 run 能证明的 Apollo 内生根因。 |
| 什么时候失去时间保证？ | 无法确定。正式保证从未建立，因为无 demand-origin qualified deadline/WCRT/suffix bound；模型 crossing 不是 guarantee-loss point。 |
| 为什么？ | 300 ms 注入器是直接观测的主要时间消耗；38.871 ms 等待和采样前 226–232 ms Detection 实例支持积压/干扰候选；fallback 使功能因素不能排除；缺 record/资源/事件命令证据，未能唯一隔离。 |
| 造成多少物理安全损失？ | observed：响应距离 11.999 m，到 minimum-speed proxy 剩余 0 m 余量约 1.018 m，没有直接碰撞严重度证据。model：模型 deadline debt 按场景另列。无 qualified deadline/反事实，timing 的现实因果份额不可定量。 |

## 方法完备性、限制与复现

- 原始输入 {inventory['file_count']} 个文件、{inventory['total_bytes']} bytes，逐文件 SHA-256 保存于 `validation/input_inventory.json`；原始目录未修改。
- `run_level_observed.csv` 与 `run_level_model_predicted.csv` 分表；模型值未回填 observed 缺失。
- 完整缺失性和阈值来源见 `method_completeness_matrix.csv`、`exclusions_and_missing.csv`和 `threshold_provenance_registry.csv`。
- 自动语义验证见 `validation/validation.json`；L5 由独立验证器从 `velocity_trajectory_observed.csv` 重算。

复现：

```bash
python3 {OUTPUT / 'scripts/analyze_1211_single_run_v4_1.py'}
python3 {SKILL_DIR / 'scripts/recompute_l5_metrics.py'} --analysis-dir {OUTPUT}
python3 {SKILL_DIR / 'scripts/validate_analysis_outputs.py'} --analysis-dir {OUTPUT}
```
"""
    (REPORT / "six_layer_analysis_report.md").write_text(report_text, encoding="utf-8")
    (OUTPUT / "README.md").write_text(
        "# Run 202607271211 TCPS-PA v4.1 analysis\n\n"
        "Main report: [six_layer_analysis_report.md](report/six_layer_analysis_report.md)\n\n"
        "Observed results use only run 1211; baseline calibration is model-only.\n",
        encoding="utf-8",
    )
    audit = f"""# Data quality audit

## Scope and immutability

- Scope: `{RUN_DIR}` only.
- Raw files: {inventory['file_count']}; SHA-256 saved for every file.
- Raw files were not modified.

## Time and endpoint quality

- t_world is left-censored no later than t_sample.
- t_demand and t_observable are unavailable.
- t_sample is the source epoch of the first frame in a retrospectively identified three-frame stable sequence.
- Same-host source-to-Control trace is available; event-local cross-host Bridge apply is unavailable.
- t_phys 0.5/1.0 m/s^2 and all median3 sensitivities agree at {tr_high_ms:.3f} ms; raw 0.3 m/s^2 gives {tr_low_ms:.3f} ms.

## Physical outcome quality

- Canonical D_response is the wall-clock speed trapezoid: {d_response:.12f} m.
- Minimum-speed proxy is used; strict stop-hold is absent.
- No CARLA CollisionSensor or actor-history file exists, so direct noncollision/physical identity is unavailable.
- No parsed record exists.

## Model boundary

- Four dynamic residual-budget rows use seven disjoint baseline runs, but calibration was not locked by the evaluated run and the domain is unvalidated.
- Model crossings do not establish a primary deadline or guarantee-loss time.
"""
    (VALIDATION / "data_quality_audit.md").write_text(audit, encoding="utf-8")
    summary = {
        "run_id": RUN_ID,
        "T_R_sample_bracket_ms": [tr_low_ms, tr_high_ms],
        "D_response_sample_m": d_response,
        "ground_to_detection_wait_ms": ground_wait["observed_ms"],
        "planning_fallback_count": raw["planning_constant_deceleration_fallback_count"],
        "control_to_physical_ms": raw["control_to_effective_brake_ms"],
        "target_gap_max_ms": max_gap_ms,
        "M0_observed_m": m0_observed,
        "primary_deadline_status": "NOT_QUALIFIED_PRIMARY",
        "guarantee_status": "NOT_ESTABLISHED",
        "attribution_status": "UNCERTAIN",
    }
    (VALIDATION / "analysis_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


if __name__ == "__main__":
    print(json.dumps(build(), ensure_ascii=False, indent=2))
