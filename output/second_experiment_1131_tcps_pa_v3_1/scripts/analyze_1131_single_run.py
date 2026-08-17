#!/usr/bin/env python3
"""TCPS-PA v3.1 single-run analysis for second-experiment run 202607271131.

The script reads only the selected run's raw data. It intentionally does not
use measurements, braking models, or baselines from any other run.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np


WORKSPACE = Path("/Users/huangjinhui/Desktop/萨卡班/data")
RUN_DIR = WORKSPACE / "第二次实验/300ms/202607271131"
OUTPUT = WORKSPACE / "output/second_experiment_1131_tcps_pa_v3_1"
TABLES = OUTPUT / "tables"
FIGURES = OUTPUT / "figures"
REPORT = OUTPUT / "report"
VALIDATION = OUTPUT / "validation"
RUN_ID = "202607271131"

PARSER_DIR = WORKSPACE / "report_workspace/scripts"
sys.path.insert(0, str(PARSER_DIR))
import analyze_second_experiment as ase  # noqa: E402
import realtime_collision_core as core  # noqa: E402

SKILL_SCRIPTS = Path(
    "/Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts"
)
sys.path.insert(0, str(SKILL_SCRIPTS))
from construct_dynamic_deadline import OUTPUT_FIELDS as DEADLINE_FIELDS  # noqa: E402
from construct_dynamic_deadline import construct_row  # noqa: E402


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def fmt(value: object, digits: int = 3) -> str:
    if not finite(value):
        return "不可用"
    return f"{float(value):.{digits}f}"


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


def read_events(path: Path, trace_ids: set[str]) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return [row for row in csv.DictReader(handle) if row.get("trace_id") in trace_ids]


def event_value(rows: list[dict], phase: str) -> float:
    values = [float(row["mono_ns"]) for row in rows if row.get("phase") == phase]
    if not values:
        return math.nan
    return min(values)


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
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.dpi": 180,
            "font.size": 10,
        }
    )


def input_inventory() -> dict:
    rows = []
    for path in sorted(item for item in RUN_DIR.rglob("*") if item.is_file()):
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        stat = path.stat()
        rows.append(
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
        "file_count": len(rows),
        "total_bytes": sum(row["size_bytes"] for row in rows),
        "files": rows,
    }
    (VALIDATION / "input_inventory.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result


def build() -> dict:
    for directory in [TABLES, FIGURES, REPORT, VALIDATION]:
        directory.mkdir(parents=True, exist_ok=True)
    configure_plotting()
    inventory = input_inventory()

    config = ase.make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    specs = [item for item in core.discover_runs(config) if item.run_id == RUN_ID]
    if len(specs) != 1:
        raise RuntimeError(f"Expected exactly one {RUN_ID} run, found {len(specs)}")
    parsed = core.parse_run(specs[0], config, timezone)
    raw, debug = core.raw_run_metrics(parsed, config)
    observed = ase.analyze_run(parsed, raw, debug)
    identity = ase.actor_identity(parsed, observed)

    t1 = float(raw["t_sensor_origin_s"])
    t_fusion = float(raw["t_perception_stable_output_s"])
    t_prediction = float(raw["t_prediction_first_s"])
    t_stop = float(raw["t_planning_stop_s"])
    t_plan_out = float(raw["t_planning_decel_s"])
    t_control = float(raw["t_control_brake_command_s"])
    t2 = float(raw["t_brake_effective_s"])
    t_collision = float(raw["t_collision_s"])
    t_fault = t1 + float(raw["scb_trigger_relative_t1_s"])

    stable = parsed.perception["stable"]
    target_rows = list(parsed.perception["target_rows"])
    response_rows = [row for row in target_rows if t1 <= row.header_time_s <= t2]
    pre_t2_latest = response_rows[-1]
    response_gaps = np.diff([row.header_time_s for row in response_rows]) * 1000.0
    target_gap_pairs = [
        (target_rows[index - 1], target_rows[index])
        for index in range(1, len(target_rows))
    ]
    max_gap_pair = max(
        target_gap_pairs, key=lambda pair: pair[1].header_time_s - pair[0].header_time_s
    )
    max_gap_ms = (
        max_gap_pair[1].header_time_s - max_gap_pair[0].header_time_s
    ) * 1000.0
    max_lifecycle_row = max(
        target_rows, key=lambda row: row.header_time_s - row.obs_time_s
    )
    age_t2_ms = (t2 - pre_t2_latest.obs_time_s) * 1000.0
    age_collision_ms = float(observed["target_source_age_at_outcome_ms"])

    # Primary observed response endpoint uncertainty: the previous Localization
    # sample is a conservative lower bracket; t2 is the first qualifying sample.
    preceding_t2 = max(sample.time_s for sample in parsed.localization if sample.time_s < t2)
    tr_low_ms = (preceding_t2 - t1) * 1000.0
    tr_high_ms = (t2 - t1) * 1000.0
    d_response_low = core.integrate_speed(parsed.localization, t1, preceding_t2)
    d_response = core.integrate_speed(parsed.localization, t1, t2)
    d_brake_truncated = core.integrate_speed(parsed.localization, t2, t_collision)

    onset_sensitivity = []
    for smoothed in [False, True]:
        for threshold in [0.3, 0.5, 1.0]:
            result = core.detect_brake_onset(
                parsed.localization,
                t1,
                t_control,
                threshold,
                config,
                smoothed=smoothed,
            )
            onset_sensitivity.append(
                {
                    "run_id": RUN_ID,
                    "speed_filter": "median3" if smoothed else "raw",
                    "deceleration_threshold_mps2": threshold,
                    "status": result.get("status", "UNKNOWN"),
                    "t2_wall_s": result.get("onset_time_s", ""),
                    "T_R_ms": (
                        (float(result["onset_time_s"]) - t1) * 1000.0
                        if finite(result.get("onset_time_s"))
                        else ""
                    ),
                    "onset_speed_mps": result.get("onset_speed_mps", ""),
                    "attribution": result.get("attribution", result.get("reason", "")),
                }
            )
    write_csv("t2_sensitivity.csv", onset_sensitivity)

    # Trace-exact first target instance decomposition.
    parent_id = str(parsed.trace["parent_trace_id"])
    target_trace_id = str(parsed.trace["target_trace_id"])
    trace_ids = {parent_id, target_trace_id}
    event_files = parsed.spec.run_dir / "trace/events"
    module_events = {
        path.name: read_events(path, trace_ids) for path in sorted(event_files.glob("*.csv"))
    }
    module_events = {key: value for key, value in module_events.items() if value}

    def ev(file_key: str, phase: str) -> float:
        return event_value(module_events[file_key], phase)

    preproc_enter = ev("perception.pointcloud_preprocess.476004.csv", "proc_enter")
    preproc_out = ev("perception.pointcloud_preprocess.476004.csv", "output_pub")
    roi_enter = ev("perception.pointcloud_map_based_roi.476004.csv", "proc_enter")
    roi_out = ev("perception.pointcloud_map_based_roi.476004.csv", "output_pub")
    ground_enter = ev("perception.pointcloud_ground_detection.476004.csv", "proc_enter")
    ground_out = ev("perception.pointcloud_ground_detection.476004.csv", "output_pub")
    det_enter = ev("perception.lidar_detection.476004.csv", "proc_enter")
    det_out = ev("perception.lidar_detection.476004.csv", "output_pub")
    filter_enter = ev("perception.lidar_detection_filter.476004.csv", "proc_enter")
    filter_out = ev("perception.lidar_detection_filter.476004.csv", "output_pub")
    tracking_enter = ev("perception.lidar_tracking.476004.csv", "proc_enter")
    tracking_out = ev("perception.lidar_tracking.476004.csv", "output_pub")
    fusion_enter = ev("perception.multi_sensor_fusion.476004.csv", "proc_enter")
    fusion_out = ev("perception.multi_sensor_fusion.476004.csv", "output_pub")
    anchor_enter = float(parsed.trace["sensor_anchor"]["preproc_enter_ns"])

    perception_decomposition = [
        {
            "segment": "sensor_to_preprocess_ingress",
            "duration_ms": float(parsed.trace["sensor_anchor"]["ingress_ms"]),
            "semantic": "source data age before Preprocess entry",
        },
        {
            "segment": "preprocess_processing",
            "duration_ms": (preproc_out - preproc_enter) / 1e6,
            "semantic": "Preprocess execution",
        },
        {
            "segment": "preprocess_to_roi_edge",
            "duration_ms": (roi_enter - preproc_out) / 1e6,
            "semantic": "edge/readiness delay",
        },
        {
            "segment": "roi_processing",
            "duration_ms": (roi_out - roi_enter) / 1e6,
            "semantic": "map ROI execution",
        },
        {
            "segment": "roi_to_ground_edge",
            "duration_ms": (ground_enter - roi_out) / 1e6,
            "semantic": "edge/readiness delay",
        },
        {
            "segment": "ground_processing",
            "duration_ms": (ground_out - ground_enter) / 1e6,
            "semantic": "ground detection execution",
        },
        {
            "segment": "ground_to_detection_wait",
            "duration_ms": (det_enter - ground_out) / 1e6,
            "semantic": "wait/queue before Lidar Detection",
        },
        {
            "segment": "lidar_detection_processing",
            "duration_ms": (det_out - det_enter) / 1e6,
            "semantic": "CenterPoint/Lidar Detection execution",
        },
        {
            "segment": "detection_to_filter_edge",
            "duration_ms": (filter_enter - det_out) / 1e6,
            "semantic": "edge/readiness delay",
        },
        {
            "segment": "filter_processing",
            "duration_ms": (filter_out - filter_enter) / 1e6,
            "semantic": "detection filter execution",
        },
        {
            "segment": "filter_to_tracking_edge",
            "duration_ms": (tracking_enter - filter_out) / 1e6,
            "semantic": "edge/readiness delay",
        },
        {
            "segment": "tracking_processing",
            "duration_ms": (tracking_out - tracking_enter) / 1e6,
            "semantic": "Lidar Tracking execution",
        },
        {
            "segment": "tracking_to_fusion_edge",
            "duration_ms": (fusion_enter - tracking_out) / 1e6,
            "semantic": "edge/readiness delay",
        },
        {
            "segment": "fusion_processing",
            "duration_ms": (fusion_out - fusion_enter) / 1e6,
            "semantic": "Multi-Sensor Fusion execution",
        },
    ]
    for row in perception_decomposition:
        row.update(
            {
                "run_id": RUN_ID,
                "trace_id": target_trace_id,
                "parent_trace_id": parent_id,
                "clock_domain": "monotonic_ns anchored to sensor wall/source time",
                "evidence_class": "TRACE_LINEAGE",
            }
        )
    write_csv("perception_trace_decomposition.csv", perception_decomposition)

    # Reconcile the sum against the trace E2E value.
    decomposition_sum = sum(float(row["duration_ms"]) for row in perception_decomposition)
    fusion_trace_e2e = float(parsed.trace["e2e_ms"]["fusion"])

    first_plan = parsed.planning["first_output"]
    fallback = parsed.planning["fallback_evidence"]
    fallback_first = fallback["speed_fallback"]["first"]
    scb_path = Path(raw["source_scb_file"])
    with scb_path.open(encoding="utf-8-sig", newline="") as handle:
        scb_rows = list(csv.DictReader(handle))
    scb_applied = next(row for row in scb_rows if row.get("status") == "APPLIED")
    fault_apply = float(scb_applied["apply_wall_time_unix_ns"]) / 1e9
    event_delay_proxy_apply = t_control + float(raw["scb_actual_wall_delay_ms"]) / 1000.0
    post_proxy_residual_ms = (t2 - event_delay_proxy_apply) * 1000.0

    observed.update(
        {
            "time_basis_main": "wall_epoch_s",
            "D_response_wall_integral_data_observed_m": d_response,
            "D_response_lower_bracket_wall_integral_data_observed_m": d_response_low,
            "T_R_lower_bracket_ms": tr_low_ms,
            "T_R_upper_bracket_ms": tr_high_ms,
            "t_outcome_wall_s": t_collision,
            "outcome_endpoint_type_data_observed": "COLLISION",
            "source_scb_file": str(raw["source_scb_file"]),
            "source_planning_file": str(raw["source_planning_file"]),
            "record_profile_available": False,
            "record_missing_reason": "NO_SAME_RUN_RECORD_OR_PARSED_RECORD_EXPORT",
            "target_output_count_response_window": len(response_rows),
            "update_gap_target_response_window_max_data_observed_ms": float(
                np.max(response_gaps)
            ),
            "update_gap_target_response_window_p90_data_observed_ms": float(
                np.percentile(response_gaps, 90)
            ),
            "data_age_target_at_t2_data_observed_ms": age_t2_ms,
            "target_lifecycle_response_window_p90_data_observed_ms": float(
                np.percentile(
                    [
                        (row.header_time_s - row.obs_time_s) * 1000.0
                        for row in response_rows
                    ],
                    90,
                )
            ),
            "full_stop_endpoint_available": False,
            "full_stop_missing_reason": "COLLISION_RIGHT_CENSORING",
            "primary_dynamic_deadline_available": False,
            "primary_dynamic_deadline_missing_reason": (
                "NO_PROSPECTIVE_INDEPENDENT_BRAKING_ENVELOPE_OR_QUALIFIED_REQUIREMENT"
            ),
            "D_debt_requirement_constrained_derived_m": "",
            "guarantee_status": "NOT_ESTABLISHED",
            "observed_contract_status": "NOT_TESTABLE",
            "impact_impulse_norm_data_observed": raw["collision_impulse_norm"],
            "D_brake_truncated_to_collision_data_observed_m": d_brake_truncated,
            "fault_onset_wall_s": t_fault,
            "fault_apply_wall_s": fault_apply,
            "fault_onset_relative_t1_s": t_fault - t1,
            "event_command_proxy_apply_wall_s": event_delay_proxy_apply,
            "control_to_t2_after_300ms_proxy_residual_ms": post_proxy_residual_ms,
            "physical_outcome_confidence": "HIGH_DIRECT_COLLISION_EVENT_AND_ACTOR_HISTORY",
            "missing_reason": (
                "COLLISION_RIGHT_CENSORING; NO_QUALIFIED_DYNAMIC_DEADLINE; "
                "NO_RECORD; NO_EVENT_LOCAL_BRIDGE_COMMAND_PAYLOAD"
            ),
        }
    )
    # Collision runs must not expose a full-stop D_brake or full-stop margin.
    observed["D_brake_data_observed_m"] = ""
    observed["M_collision_0m_data_observed_m"] = ""
    observed["M_safety_6m_data_observed_m"] = ""
    write_csv("run_level_observed.csv", [observed])

    model_row = {
        "run_id": RUN_ID,
        "analysis_status_model_predicted": "NOT_COMPUTED",
        "model_predicted_deadline_ms": "",
        "model_predicted_braking_distance_m": "",
        "model_predicted_collision_margin_m": "",
        "model_name": "",
        "qualification": "NO_QUALIFIED_INDEPENDENT_MODEL",
        "reason": (
            "Single-run analysis does not borrow other-run calibration; current-run "
            "collision right-censors full stopping and cannot independently calibrate a model."
        ),
    }
    write_csv("run_level_model_predicted.csv", [model_row])

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
            "quality_flags": "STRICTLY_MONOTONIC" if index else "START",
        }
        for index, sample in enumerate(parsed.localization)
    ]
    write_csv("velocity_trajectory_observed.csv", velocity_rows)

    # Dynamic requirement is deliberately unqualified and invalid for primary use.
    deadline_seed = {field: "" for field in DEADLINE_FIELDS}
    deadline_seed.update(
        {
            "construction_id": "DDL.1131.PRIMARY",
            "requirement_id": "REQ.DYN.1131",
            "run_id_or_group": RUN_ID,
            "method": "RSS_LIKE_LONGITUDINAL",
            "method_version": "TCPS-PA-v3.1",
            "state_time": t1,
            "state_time_basis": "wall_epoch_s",
            "state_available_by_t1": "TRUE",
            "input_cutoff_time": t1,
            "latest_input_time": t1,
            "parameter_selection_locked_by_t1": "FALSE",
            "current_run_post_t1_data_used": "FALSE",
            "current_run_outcome_used": "FALSE",
            "d_clear_m": observed["D1_clear_data_observed_m"],
            "v_ego_mps": observed["v1_data_observed_mps"],
            "v_front_mps": 0.0,
            "target_motion_assumption": "static target observed by Prediction",
            "road_condition_assumption": "not archived",
            "validation_dataset_independent": "FALSE",
            "validation_scope": "NONE",
            "evaluation_run_ids": RUN_ID,
            "braking_envelope_status": "MISSING",
            "source_evidence_ids": "EV.TARGET.1131|EV.REACTION.1131",
            "notes": (
                "No pre-t1 locked safety buffer, response-acceleration bound, independently "
                "validated ego braking lower bound, or target braking bound. No primary tau is emitted."
            ),
        }
    )
    deadline_row = construct_row(deadline_seed)
    write_csv("dynamic_deadline_construction.csv", [deadline_row], DEADLINE_FIELDS)

    requirement_rows = [
        {
            "requirement_id": "REQ.DYN.1131",
            "run_id_or_group": RUN_ID,
            "requirement_name": "prospective scenario-dependent physical reaction deadline",
            "requirement_value": "",
            "unit": "ms",
            "requirement_provenance": "not available for this run",
            "pre_registered": "FALSE",
            "external_or_internal": "NONE",
            "safety_meaning": "would bound sensor-source to sustained physical deceleration",
            "deadline_type": "DYNAMIC_CONSTRUCTED",
            "evidence_class": "MISSING",
            "tau_req_low_ms": "",
            "tau_req_center_ms": "",
            "tau_req_high_ms": "",
            "validation_scope": "NONE",
            "p_deadline_qualification": "NOT_QUALIFIED_PRIMARY",
            "notes": (
                "No independent prospective braking envelope; collision-truncated same-run "
                "trajectory is not used to construct a primary requirement."
            ),
        }
    ]
    write_csv("requirement_registry.csv", requirement_rows)

    event_timeline = [
        {
            "run_id": RUN_ID,
            "event_id": "E00",
            "event_kind": "temporal_stressor_onset",
            "label": "SCB first effective-brake trigger/receive",
            "t_wall_s": t_fault,
            "relative_t1_ms": (t_fault - t1) * 1000.0,
            "clock_domain": "bridge wall epoch",
            "lineage_grade": "A for logged APPLIED row; persistence from implementation documentation",
            "source_file": str(scb_path),
            "source_locator": "CSV APPLIED row, sequence=1",
            "uncertainty_or_limit": "subsequent delayed commands suppressed from CSV",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E01",
            "event_kind": "sensor_sample/physical_cause",
            "label": "t1: first frame of 3-frame stable target sequence",
            "t_wall_s": t1,
            "relative_t1_ms": 0.0,
            "clock_domain": "sensor source/wall epoch",
            "lineage_grade": "A",
            "source_file": stable.source_file,
            "source_locator": f"line {stable.source_line}; trace_id={target_trace_id}",
            "uncertainty_or_limit": "event definition, not a safety deadline",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E02",
            "event_kind": "publish",
            "label": "stable Fusion output",
            "t_wall_s": t_fusion,
            "relative_t1_ms": (t_fusion - t1) * 1000.0,
            "clock_domain": "wall epoch aligned from trace",
            "lineage_grade": "A",
            "source_file": stable.source_file,
            "source_locator": f"line {stable.source_line}; trace_id={target_trace_id}",
            "uncertainty_or_limit": "log-header and trace E2E differ by 0.094 ms",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E03",
            "event_kind": "publish",
            "label": "Prediction static-target output",
            "t_wall_s": t_prediction,
            "relative_t1_ms": (t_prediction - t1) * 1000.0,
            "clock_domain": "wall epoch aligned from trace",
            "lineage_grade": "A",
            "source_file": parsed.prediction["source_file"],
            "source_locator": (
                f"line {parsed.prediction['first_output']['source_line']}; "
                f"trace_id={target_trace_id}"
            ),
            "uncertainty_or_limit": "static semantics verified for 20/20 target observations",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E04",
            "event_kind": "decision",
            "label": "Planning STOP for target 11",
            "t_wall_s": t_stop,
            "relative_t1_ms": (t_stop - t1) * 1000.0,
            "clock_domain": "wall epoch",
            "lineage_grade": "A",
            "source_file": parsed.planning["first_stop"]["source_file"],
            "source_locator": f"line {parsed.planning['first_stop']['source_line']}",
            "uncertainty_or_limit": "STOP exists; subsequent speed optimizer falls back",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E05",
            "event_kind": "publish",
            "label": "Planning fallback trajectory output",
            "t_wall_s": t_plan_out,
            "relative_t1_ms": (t_plan_out - t1) * 1000.0,
            "clock_domain": "wall epoch",
            "lineage_grade": "A",
            "source_file": first_plan["source_file"],
            "source_locator": f"line {first_plan['source_line']}",
            "uncertainty_or_limit": "status_ok=1, 98 points, but constant-deceleration fallback",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E06",
            "event_kind": "command",
            "label": "Control output for causal trace",
            "t_wall_s": t_control,
            "relative_t1_ms": (t_control - t1) * 1000.0,
            "clock_domain": "monotonic trace anchored to source wall time",
            "lineage_grade": "A to Control output",
            "source_file": parsed.trace["source_files"]["control_context"],
            "source_locator": f"trace_id={target_trace_id}; cmd_write_enter/output_pub",
            "uncertainty_or_limit": "command payload and event-local Bridge apply row absent",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E07",
            "event_kind": "physical_effect",
            "label": "t2: first sustained deceleration sample",
            "t_wall_s": t2,
            "relative_t1_ms": (t2 - t1) * 1000.0,
            "clock_domain": "wall epoch",
            "lineage_grade": "C from Control output to physical effect",
            "source_file": raw["source_localization_file"],
            "source_locator": "first 2 consecutive intervals <= -0.5 m/s^2 plus 0.3 m/s confirmation",
            "uncertainty_or_limit": f"sample bracket [{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms",
        },
        {
            "run_id": RUN_ID,
            "event_id": "E08",
            "event_kind": "freshness_degradation",
            "label": "largest target Fusion output gap begins",
            "t_wall_s": max_gap_pair[0].header_time_s,
            "relative_t1_ms": (max_gap_pair[0].header_time_s - t1) * 1000.0,
            "clock_domain": "wall epoch",
            "lineage_grade": "A for target Fusion outputs",
            "source_file": max_gap_pair[0].source_file,
            "source_locator": (
                f"trace {max_gap_pair[0].trace_id} -> {max_gap_pair[1].trace_id}"
            ),
            "uncertainty_or_limit": (
                f"gap={max_gap_ms:.3f} ms; occurs after t2, not explanatory for initial response"
            ),
        },
        {
            "run_id": RUN_ID,
            "event_id": "E09",
            "event_kind": "outcome",
            "label": "CARLA collision",
            "t_wall_s": t_collision,
            "relative_t1_ms": (t_collision - t1) * 1000.0,
            "clock_domain": "bridge wall epoch",
            "lineage_grade": "direct physical outcome; causal attribution unresolved",
            "source_file": raw["source_collision_file"],
            "source_locator": "CSV row 2, collision_seq=1",
            "uncertainty_or_limit": "collision proves outcome, not deadline miss or sole cause",
        },
    ]
    write_csv("event_timeline.csv", event_timeline)

    stage_rows = [
        {
            "run_id": RUN_ID,
            "chain_instance": target_trace_id,
            "stage": "sensor_source_to_fusion_output",
            "start_event": "t1 source sample",
            "end_event": "Fusion output",
            "reaction_ms": (t_fusion - t1) * 1000.0,
            "age_ms": (t_fusion - t1) * 1000.0,
            "gap_ms": "",
            "time_basis": "wall/trace anchored",
            "lineage_grade": "A",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "NO_INDEPENDENT_LOCAL_BUDGET",
            "interpretation": "observed hotspot; not by itself a violation",
        },
        {
            "run_id": RUN_ID,
            "chain_instance": target_trace_id,
            "stage": "fusion_to_prediction",
            "start_event": "Fusion output",
            "end_event": "Prediction output",
            "reaction_ms": raw["perception_to_prediction_ms"],
            "age_ms": (t_prediction - t1) * 1000.0,
            "gap_ms": "",
            "time_basis": "trace anchored",
            "lineage_grade": "A",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "NO_INDEPENDENT_LOCAL_BUDGET",
            "interpretation": "small share of observed response",
        },
        {
            "run_id": RUN_ID,
            "chain_instance": target_trace_id,
            "stage": "prediction_to_planning_stop",
            "start_event": "Prediction output",
            "end_event": "Planning STOP",
            "reaction_ms": raw["prediction_to_planning_stop_ms"],
            "age_ms": (t_stop - t1) * 1000.0,
            "gap_ms": "",
            "time_basis": "wall/trace anchored",
            "lineage_grade": "A",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "NO_INDEPENDENT_LOCAL_BUDGET",
            "interpretation": "STOP exists for the same target",
        },
        {
            "run_id": RUN_ID,
            "chain_instance": target_trace_id,
            "stage": "planning_stop_to_control_output",
            "start_event": "Planning STOP",
            "end_event": "Control output",
            "reaction_ms": raw["planning_stop_to_control_ms"],
            "age_ms": (t_control - t1) * 1000.0,
            "gap_ms": "",
            "time_basis": "wall/trace anchored",
            "lineage_grade": "A",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "NO_INDEPENDENT_LOCAL_BUDGET",
            "interpretation": "contains speed-fallback planning output and Control work",
        },
        {
            "run_id": RUN_ID,
            "chain_instance": target_trace_id,
            "stage": "control_output_to_physical_t2",
            "start_event": "Control output",
            "end_event": "sustained physical deceleration",
            "reaction_ms": raw["control_to_effective_brake_ms"],
            "age_ms": (t2 - t1) * 1000.0,
            "gap_ms": "",
            "time_basis": "wall with aligned Control trace",
            "lineage_grade": "C",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "300_MS_CONFIGURED_STRESSOR_PLUS_UNRESOLVED_RESIDUAL",
            "interpretation": (
                f"dominant observed segment; subtracting 300.047 ms leaves {post_proxy_residual_ms:.3f} ms "
                "diagnostic residual that mixes scheduling, CARLA tick, vehicle dynamics, and onset sampling"
            ),
        },
        {
            "run_id": RUN_ID,
            "chain_instance": "target 11 response window",
            "stage": "target_freshness_at_t2",
            "start_event": "latest source consumed before t2",
            "end_event": "t2",
            "reaction_ms": "",
            "age_ms": age_t2_ms,
            "gap_ms": float(np.max(response_gaps)),
            "time_basis": "wall epoch",
            "lineage_grade": "A to Fusion; C to physical effect",
            "event_relevance": "INITIAL_RESPONSE",
            "reference_status": "NO_QUALIFIED_FRESHNESS_REQUIREMENT",
            "interpretation": "descriptive A/G quantities; no miss verdict",
        },
        {
            "run_id": RUN_ID,
            "chain_instance": "target 11 continuous response",
            "stage": "post_t2_target_continuity",
            "start_event": "Fusion output before largest gap",
            "end_event": "next Fusion output",
            "reaction_ms": "",
            "age_ms": (max_lifecycle_row.header_time_s - max_lifecycle_row.obs_time_s)
            * 1000.0,
            "gap_ms": max_gap_ms,
            "time_basis": "wall epoch",
            "lineage_grade": "A to Fusion; downstream command semantics absent",
            "event_relevance": "POST_T2_BEFORE_COLLISION",
            "reference_status": "NO_QUALIFIED_FRESHNESS_REQUIREMENT",
            "interpretation": "late closed-loop degradation candidate; cannot explain first t2",
        },
    ]
    write_csv("stage_timing_and_freshness.csv", stage_rows)

    target_timeline = []
    previous = None
    for index, row in enumerate(target_rows):
        gap = (row.header_time_s - previous.header_time_s) * 1000.0 if previous else ""
        target_timeline.append(
            {
                "run_id": RUN_ID,
                "target_id": row.obstacle_id,
                "observation_index": index,
                "trace_id": row.trace_id,
                "source_time_wall_s": row.obs_time_s,
                "fusion_output_wall_s": row.header_time_s,
                "lifecycle_age_at_output_ms": (row.header_time_s - row.obs_time_s) * 1000.0,
                "output_gap_from_previous_ms": gap,
                "event_window": (
                    "INITIAL_RESPONSE"
                    if row.header_time_s <= t2
                    else "POST_T2_PRE_COLLISION"
                    if row.header_time_s <= t_collision
                    else "AFTER_COLLISION"
                ),
                "speed_mps": row.speed_mps,
                "confidence": row.confidence,
                "source_file": row.source_file,
                "source_locator": f"line {row.source_line}",
            }
        )
        previous = row
    write_csv("target_freshness_timeline.csv", target_timeline)

    identity_row = {
        **identity,
        "evidence_class": "TRACE_LINEAGE_PLUS_ACTOR_HISTORY_ASSOCIATION",
        "history_coverage_note": (
            "other-actor history begins at t1+399.215 ms; target association uses 20 matched Fusion rows, "
            "not a t1 ground-truth obstacle row"
        ),
    }
    write_csv("target_identity_audit.csv", [identity_row])

    fault_rows = [
        {
            "run_id": RUN_ID,
            "fault_type": "FIXED_CONTROL_COMMAND_DELAY",
            "injection_location": "Bridge ControlDelayInjector after Apollo Control",
            "requested_magnitude": raw["scb_requested_delay_ms"],
            "actual_magnitude": raw["scb_actual_wall_delay_ms"],
            "actual_distribution": "one logged APPLIED sample; later applications suppressed",
            "fault_onset_wall": t_fault,
            "fault_end_wall": "",
            "t1_wall": t1,
            "trigger_relative_t1_s": t_fault - t1,
            "duration": f">={t1 - t_fault:.6f} s through t1; end not logged",
            "persistent_or_transient": "PERSISTENT_AFTER_TRIGGER_BY_IMPLEMENTATION",
            "one_shot_or_repeated": "REPEATED_AFTER_TRIGGER_BY_IMPLEMENTATION",
            "affected_channel": "Apollo ControlCommand -> CARLA apply_control",
            "affected_message_count": "UNKNOWN_LOGGING_SUPPRESSED",
            "queue_behavior": "FIFO delayed worker; event-local queue depth unavailable",
            "drop_status": "UNKNOWN_NO_EVENT_LOCAL_ROWS",
            "reorder_status": "implementation preserves order; not event-locally re-observed",
            "evidence_class": "DIRECT_OBSERVED",
            "confidence": "MEDIUM",
        }
    ]
    write_csv("temporal_fault_signature.csv", fault_rows)

    pre_hazard_rows = [
        {
            "run_id": RUN_ID,
            "state_variable": "D1",
            "window_start_wall": t_fault,
            "window_end_wall": t1,
            "value_at_fault": "",
            "value_at_t1": observed["D1_clear_data_observed_m"],
            "delta": "",
            "source_file": raw["source_perception_file"],
            "availability": "PARTIAL",
            "causal_role": "POST_TREATMENT_STATE",
            "evidence_class": "OBSERVED_DERIVED",
            "confidence": "MEDIUM",
            "notes": (
                "Fault was active 25.997 s before t1. Obstacle clearance at fault onset is not observed; "
                "D1 cannot be treated as an untreated baseline."
            ),
        },
        {
            "run_id": RUN_ID,
            "state_variable": "V1",
            "window_start_wall": t_fault,
            "window_end_wall": t1,
            "value_at_fault": scb_applied["ego_speed_mps_at_receive"],
            "value_at_t1": observed["v1_data_observed_mps"],
            "delta": float(observed["v1_data_observed_mps"])
            - float(scb_applied["ego_speed_mps_at_receive"]),
            "source_file": f"{scb_path}|{raw['source_localization_file']}",
            "availability": "AVAILABLE",
            "causal_role": "POST_TREATMENT_STATE",
            "evidence_class": "OBSERVED_DERIVED",
            "confidence": "HIGH",
            "notes": "v1 occurs after 25.997 s of active fault and is not an untreated pre-fault state.",
        },
    ]
    write_csv("pre_hazard_state_audit.csv", pre_hazard_rows)

    functional_row = {
        "run_id": RUN_ID,
        "physical_target_identity": "PASS",
        "perception_target_present": "PASS",
        "perception_tracking_continuity": "PASS_IN_INITIAL_RESPONSE_WINDOW",
        "prediction_target_present": "PASS",
        "prediction_semantics_valid": "PASS_STATIC_20_OF_20",
        "planning_stop_present": "PASS",
        "planning_stop_target_correct": "PASS_TARGET_11",
        "planning_stop_location_reasonable": "PARTIAL_TARGET_STOP_GEOMETRY_PRESENT",
        "planning_trajectory_valid": "DEGRADED_BUT_NONEMPTY",
        "planning_fallback_status": "DEGRADED_CONSTANT_DECELERATION_FALLBACK",
        "control_received_relevant_trajectory": "PASS_TRACE_LINEAGE",
        "control_braking_command_present": "UNKNOWN_PAYLOAD_NOT_ARCHIVED",
        "control_command_continuity": "UNKNOWN_NO_RECORD_OR_CONTROL_PAYLOAD",
        "bridge_payload_received": "UNKNOWN_EVENT_LOCAL_ROW_NOT_LOGGED",
        "bridge_payload_applied": "UNKNOWN_EVENT_LOCAL_ROW_NOT_LOGGED",
        "physical_response_observed": "PASS",
        "p_func_verdict": "PARTIAL",
        "confidence": "MEDIUM",
        "source_evidence_ids": "EV.TARGET.1131|EV.FUNC.1131|EV.CHAIN.1131|EV.REACTION.1131",
        "notes": (
            "Planning issued STOP for target 11, then speed optimization failed and emitted a "
            "constant-deceleration fallback trajectory (status_ok=1, 98 points). Control payload "
            "and event-local Bridge apply evidence are absent; functional correctness is not qualified."
        ),
    }
    write_csv("functional_correctness_audit.csv", [functional_row])

    clock_row = {
        "run_id_or_group": RUN_ID,
        "clock_domain": "Apollo/CARLA propagated source epoch + monotonic trace + Bridge wall epoch",
        "host": "Orin and CARLA server",
        "timestamp_type": "epoch seconds, monotonic nanoseconds, CARLA simulation seconds",
        "sync_method": parsed.clock["method"] + "; trace_anchor source-to-monotonic mapping",
        "offset_estimate_ms": 0.0,
        "offset_bound_ms": parsed.clock["p95_abs_residual_ms"],
        "drift_estimate_ppm": (float(parsed.clock["slope"]) - 1.0) * 1e6,
        "dispersion_or_sync_distance_ms": parsed.clock["p95_abs_residual_ms"],
        "alignment_residual_ms": parsed.clock["median_abs_residual_ms"],
        "timestamp_resolution_ms": 0.001,
        "measurement_window": f"{iso(t1)} to {iso(t_collision)}; 152 inliers/156 pairs",
        "source_evidence_ids": "EV.CLOCK.1131",
        "confidence": "HIGH",
        "p_clock_verdict": "PASS",
        "notes": (
            f"p95 absolute residual={parsed.clock['p95_abs_residual_ms']:.3f} ms; "
            "impact-adjacent outliers do not alter t1/t2 ordering."
        ),
    }
    write_csv("clock_alignment_audit.csv", [clock_row])

    phase_row = {
        "run_id_or_group": RUN_ID,
        "producer_period_ms": 100.0,
        "consumer_period_ms": "mixed",
        "bridge_tick_period_ms": 100.0,
        "phase_definition": "relative sensor/Control/CARLA tick offset",
        "phase_bins_or_offsets": "",
        "scan_performed": "FALSE",
        "matched_repeats_per_phase": 0,
        "phase_effect_metric": "",
        "phase_effect_estimate": "",
        "uncertainty_interval": "",
        "phase_effect_verdict": "NOT_TESTABLE",
        "source_evidence_ids": "EV.PHASE.1131",
        "p_phase_verdict": "NOT_TESTABLE",
        "notes": "No active phase scan or matched repeats in this single run; tick quantization remains an alternative.",
    }
    write_csv("phase_audit.csv", [phase_row])

    # Evidence ledger.
    evidence = [
        {
            "evidence_id": "EV.CLOCK.1131",
            "run_id": RUN_ID,
            "layer": "P_CLOCK",
            "metric": "clock_alignment_p95_residual_ms",
            "value": parsed.clock["p95_abs_residual_ms"],
            "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "cross-domain alignment audit",
            "source_file": parsed.clock["source_file"],
            "source_locator": "156 CARLA simulation/wall pairs; 152 robust-fit inliers",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "P_CLOCK.1131",
            "challenges_claim_ids": "",
            "limitations": "p95 bound; max residual includes impact-adjacent outliers",
            "semantic_role": "CLOCK_ALIGNMENT",
        },
        {
            "evidence_id": "EV.PHASE.1131",
            "run_id": RUN_ID,
            "layer": "P_PHASE",
            "metric": "active_phase_scan",
            "value": "not performed",
            "unit": "status",
            "evidence_class": "MISSING",
            "clock_domain": "mixed",
            "source_file": str(RUN_DIR),
            "source_locator": "single-run input inventory",
            "availability": "MISSING",
            "confidence": "HIGH",
            "supports_claim_ids": "",
            "challenges_claim_ids": "P_PHASE.1131|C7.1131",
            "limitations": "phase effect cannot be isolated",
            "semantic_role": "PHASE_AUDIT_GAP",
        },
        {
            "evidence_id": "EV.TARGET.1131",
            "run_id": RUN_ID,
            "layer": "P_TARGET",
            "metric": "target_identity_matched_frames",
            "value": identity["matched_frame_count"],
            "unit": "frames",
            "evidence_class": "TRACE_LINEAGE",
            "clock_domain": "wall epoch and trace ids",
            "source_file": f"{stable.source_file}|{parsed.files['actor_history']}",
            "source_locator": (
                f"target=11, trace={target_trace_id}, actor=155; "
                f"median position error={identity['position_error_median_m']:.3f} m"
            ),
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "P_TARGET.1131",
            "challenges_claim_ids": "",
            "limitations": "other-actor history begins 399.215 ms after t1",
            "semantic_role": "TARGET_LINEAGE",
        },
        {
            "evidence_id": "EV.FAULT.1131",
            "run_id": RUN_ID,
            "layer": "L1",
            "metric": "bridge_fixed_delay_actual_wall_ms",
            "value": raw["scb_actual_wall_delay_ms"],
            "unit": "ms",
            "evidence_class": "DIRECT_OBSERVED",
            "clock_domain": "Bridge wall/monotonic/CARLA frame",
            "source_file": str(scb_path),
            "source_locator": "APPLIED row sequence=1; requested=300 ms; 3 CARLA frames",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C1.1131|C2.1131|C7.1131",
            "challenges_claim_ids": "",
            "limitations": "subsequent commands are delayed by implementation but not logged individually",
            "semantic_role": "TEMPORAL_DISTURBANCE_APPLICATION",
            "reference_type": "CONFIGURED_REQUEST",
            "distribution_scope": "SINGLE_APPLIED_EVENT_WITH_EXPLICIT_REQUEST",
        },
        {
            "evidence_id": "EV.REACTION.1131",
            "run_id": RUN_ID,
            "layer": "L3/L4",
            "metric": "T_R_observed_physical",
            "value": tr_high_ms,
            "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "wall epoch",
            "source_file": raw["source_localization_file"],
            "source_locator": "t1 stable source timestamp to first sustained-deceleration sample t2",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C3.1131|C4.1131",
            "challenges_claim_ids": "",
            "limitations": f"physical-effect sample bracket [{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms",
            "semantic_role": "PHYSICAL_REACTION_INTERVAL",
        },
        {
            "evidence_id": "EV.CHAIN.1131",
            "run_id": RUN_ID,
            "layer": "L3",
            "metric": "sensor_to_control_trace_lineage_ms",
            "value": raw["sensor_to_control_ms"],
            "unit": "ms",
            "evidence_class": "TRACE_LINEAGE",
            "clock_domain": "monotonic trace anchored to source epoch",
            "source_file": parsed.trace["source_files"]["control_context"],
            "source_locator": f"trace_id={target_trace_id}; parent_trace_id={parent_id}",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C3.1131|P_FUNC.1131|C7.1131",
            "challenges_claim_ids": "",
            "limitations": "strict lineage ends at Control output; physical t2 is Grade C association",
            "semantic_role": "CAUSE_EFFECT_LINEAGE",
            "causal_lineage_grade": "C_FULL_CHAIN_A_SOFTWARE_PREFIX",
        },
        {
            "evidence_id": "EV.FRESH.1131",
            "run_id": RUN_ID,
            "layer": "L2/L3",
            "metric": "target_age_at_t2_and_response_gap",
            "value": f"A={age_t2_ms:.3f};Gmax={float(np.max(response_gaps)):.3f}",
            "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "wall epoch",
            "source_file": stable.source_file,
            "source_locator": "5 target Fusion outputs in [t1,t2]",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131|C3.1131",
            "challenges_claim_ids": "P_FUNC.1131|C7.1131",
            "limitations": "no independent freshness/gap requirement",
            "semantic_role": "FRESHNESS_CONTINUITY_DIAGNOSTIC",
        },
        {
            "evidence_id": "EV.POSTGAP.1131",
            "run_id": RUN_ID,
            "layer": "L2/L3",
            "metric": "post_t2_target_gap_max_ms",
            "value": max_gap_ms,
            "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "wall epoch",
            "source_file": max_gap_pair[0].source_file,
            "source_locator": f"trace {max_gap_pair[0].trace_id} to {max_gap_pair[1].trace_id}",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "",
            "challenges_claim_ids": "P_FUNC.1131|C7.1131",
            "limitations": "starts after t2; cannot explain initial response endpoint",
            "semantic_role": "POST_T2_FRESHNESS_DEGRADATION",
        },
        {
            "evidence_id": "EV.FUNC.1131",
            "run_id": RUN_ID,
            "layer": "P_FUNC",
            "metric": "planning_constant_deceleration_fallback_count",
            "value": raw["planning_constant_deceleration_fallback_count"],
            "unit": "events",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "planning log wall epoch",
            "source_file": first_plan["source_file"],
            "source_locator": (
                f"first fallback line {fallback_first['source_line']}; "
                f"first output line {first_plan['source_line']}"
            ),
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "P_FUNC.1131",
            "challenges_claim_ids": "P_FUNC.1131|C7.1131",
            "limitations": "fallback trajectory is nonempty/status_ok but prevents functional qualification",
            "semantic_role": "FUNCTIONAL_DEGRADATION",
        },
        {
            "evidence_id": "EV.DRESPONSE.1131",
            "run_id": RUN_ID,
            "layer": "L5",
            "metric": "D_response_wall_integral_data_observed_m",
            "value": d_response,
            "unit": "m",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "wall epoch",
            "source_file": raw["source_localization_file"],
            "source_locator": "trapezoidal integral of speed over [t1,t2] with endpoint samples",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C5.1131",
            "challenges_claim_ids": "",
            "limitations": "response distance is not requirement-constrained deadline debt",
            "semantic_role": "OBSERVED_RESPONSE_DISTANCE",
        },
        {
            "evidence_id": "EV.COLLISION.1131",
            "run_id": RUN_ID,
            "layer": "L6",
            "metric": "impact_speed_data_observed_mps",
            "value": raw["impact_speed_mps"],
            "unit": "m/s",
            "evidence_class": "DIRECT_OBSERVED",
            "clock_domain": "Bridge wall epoch/CARLA",
            "source_file": raw["source_collision_file"],
            "source_locator": "CSV row 2; collision_seq=1; other_actor_id=155",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C6.1131|C7.1131",
            "challenges_claim_ids": "",
            "limitations": "outcome does not establish a timing deadline violation or timing causation",
            "semantic_role": "PHYSICAL_OUTCOME",
        },
        {
            "evidence_id": "EV.NODEADLINE.1131",
            "run_id": RUN_ID,
            "layer": "P_DEADLINE/L4",
            "metric": "qualified_dynamic_deadline",
            "value": "unavailable",
            "unit": "status",
            "evidence_class": "MISSING",
            "clock_domain": "not applicable",
            "source_file": str(TABLES / "dynamic_deadline_construction.csv"),
            "source_locator": "DDL.1131.PRIMARY",
            "availability": "MISSING",
            "confidence": "HIGH",
            "supports_claim_ids": "P_DEADLINE.1131|C4.1131",
            "challenges_claim_ids": "P_DEADLINE.1131|C4.1131|C5.1131|C7.1131",
            "limitations": "no qualified tau_req, guarantee-loss point, or primary D_debt",
            "semantic_role": "DEADLINE_QUALIFICATION_GAP",
        },
    ]
    evidence_fields = [
        "evidence_id",
        "run_id",
        "layer",
        "metric",
        "value",
        "unit",
        "evidence_class",
        "clock_domain",
        "source_file",
        "source_locator",
        "availability",
        "confidence",
        "supports_claim_ids",
        "challenges_claim_ids",
        "limitations",
        "semantic_role",
        "reference_type",
        "distribution_scope",
        "causal_lineage_grade",
    ]
    write_csv("evidence_ledger.csv", evidence, evidence_fields)

    # Claims use legacy C1-C7 compatibility IDs plus explicit v3.1 G1/E1 tables.
    claim_rows: list[dict] = []

    def claim(
        claim_id: str,
        layer: str,
        proposition: str,
        prereqs: str,
        required_classes: str,
        support: str,
        challenge: str,
        defeaters: str,
        rule: str,
        verdict: str,
        confidence: str,
        ceiling: str,
        level: int,
        residual: str,
        allowed: str,
        forbidden: str,
        gate: str,
        next_gate: str,
        lineage: str = "",
    ) -> None:
        claim_rows.append(
            {
                "claim_id": claim_id,
                "layer": layer,
                "run_id_or_group": RUN_ID,
                "proposition": proposition,
                "prerequisite_claim_ids": prereqs,
                "required_evidence_classes": required_classes,
                "supporting_evidence_ids": support,
                "challenging_evidence_ids": challenge,
                "defeater_ids": defeaters,
                "inference_rule_id": rule,
                "verdict": verdict,
                "confidence": confidence,
                "confidence_ceiling": ceiling,
                "maximum_claim_level": level,
                "residual_uncertainty": residual,
                "allowed_language": allowed,
                "forbidden_language": forbidden,
                "causal_lineage_grade": lineage,
                "gate_inputs": support,
                "gate_metrics": gate,
                "admissible_evidence": required_classes,
                "gate_criterion": gate,
                "next_gate_condition": next_gate,
            }
        )

    claim(
        "P_CLOCK.1131",
        "P_CLOCK",
        "The event timing domains are sufficiently aligned for this event-chain ordering and millisecond intervals.",
        "",
        "OBSERVED_DERIVED",
        "EV.CLOCK.1131",
        "",
        "",
        "IR-P-CLOCK",
        "PASS",
        "HIGH",
        "HIGH",
        3,
        "p95 alignment residual is a bound, not zero error.",
        "Clock alignment supports event ordering and interval measurement.",
        "All timestamps are exact and error-free.",
        "p95 residual <= 1 ms with stable mapping",
        "Preserve endpoint uncertainty in physical interpretation.",
    )
    claim(
        "P_PHASE.1131",
        "P_PHASE",
        "A phase effect has been actively tested and isolated.",
        "",
        "",
        "",
        "EV.PHASE.1131",
        "",
        "IR-P-PHASE",
        "NOT_TESTABLE",
        "NONE",
        "NONE",
        1,
        "No active phase scan or matched repeats.",
        "Tick/phase is an unresolved alternative.",
        "Phase is the root cause.",
        "active phase scan with matched repeats",
        "Collect controlled phase-offset repeats.",
    )
    claim(
        "P_TARGET.1131",
        "P_TARGET",
        "Fusion/Prediction/Planning target 11 corresponds to CARLA collision actor 155.",
        "",
        "TRACE_LINEAGE",
        "EV.TARGET.1131",
        "",
        "",
        "IR-P-TARGET",
        "PASS",
        "HIGH",
        "HIGH",
        3,
        "CARLA other-actor history starts 399.215 ms after t1.",
        "Target identity is strongly supported across 20 matched frames.",
        "Target identity is mathematically exact at t1.",
        "explicit trace plus multi-frame actor association",
        "Archive pre-t1 other-actor history for full t1 ground truth.",
    )
    claim(
        "P_FUNC.1131",
        "P_FUNC",
        "Relevant functional behavior is fully qualified and does not independently explain the outcome.",
        "P_TARGET.1131",
        "OBSERVED_DERIVED",
        "EV.FUNC.1131|EV.CHAIN.1131",
        "EV.FUNC.1131",
        "D_FUNC.P_FUNC.1131",
        "IR-P-FUNC",
        "PARTIAL",
        "MEDIUM",
        "MEDIUM",
        2,
        "Planning fallback and absent Control/Bridge payload continuity keep functionality unresolved.",
        "The STOP chain exists, but functional correctness remains partial.",
        "functionally correct, temporally wrong|功能正确但时间错误",
        "target-correct valid trajectory and command/apply continuity",
        "Archive parsed record and event-local Control/Bridge payloads; resolve fallback semantics.",
    )
    claim(
        "P_DEADLINE.1131",
        "P_DEADLINE",
        "An independently qualified prospective dynamic physical deadline is available.",
        "P_TARGET.1131",
        "INDEPENDENT_REQUIREMENT",
        "EV.NODEADLINE.1131",
        "EV.NODEADLINE.1131",
        "D_DEADLINE.P_DEADLINE.1131",
        "IR-P-DEADLINE",
        "NOT_TESTABLE",
        "LOW",
        "LOW",
        1,
        "No independent braking envelope or pre-t1 locked safety policy.",
        "Primary tau_req is unavailable for this run.",
        "The run missed its physical safety deadline.",
        "qualified prospective construction with uncertainty bounds",
        "Provide independently validated braking bounds and locked safety buffer.",
    )
    claim(
        "C1.1131",
        "L1",
        "A 300 ms Bridge fixed-delay stressor was actually applied before the selected event.",
        "",
        "DIRECT_OBSERVED",
        "EV.FAULT.1131",
        "",
        "",
        "IR-C1",
        "PASS",
        "HIGH",
        "HIGH",
        3,
        "Only the first applied command is logged; persistence follows deployed implementation semantics.",
        "The external Bridge temporal stressor entered the system.",
        "Apollo has an intrinsic real-time defect.",
        "APPLIED row with actual delay",
        "Per-command logging is required for event-local propagation strength.",
    )
    claim(
        "C2.1131",
        "L2",
        "The configured Bridge delay manifested as a measured local delay event.",
        "C1.1131",
        "DIRECT_OBSERVED",
        "EV.FAULT.1131|EV.FRESH.1131",
        "",
        "",
        "IR-C2",
        "PASS",
        "HIGH",
        "HIGH",
        3,
        "Freshness quantities have no qualified local requirement; the Bridge delay itself has an explicit 300 ms reference.",
        "A measured local Bridge delay manifestation is established.",
        "All observed age/gap values are requirement violations.",
        "actual delay against configured request",
        "Add local freshness/gap requirements for A/G verdicts.",
    )
    claim(
        "C3.1131",
        "L3",
        "The target instance propagates from source through Control and is temporally associated with physical t2.",
        "C1.1131|C2.1131|P_CLOCK.1131|P_TARGET.1131",
        "TRACE_LINEAGE",
        "EV.CHAIN.1131|EV.REACTION.1131|EV.FRESH.1131",
        "",
        "D_PAYLOAD.C3.1131",
        "IR-C3",
        "PARTIAL_PASS",
        "MEDIUM",
        "MEDIUM",
        3,
        "Grade A lineage stops at Control output; Control-to-physical link is Grade C.",
        "Strict software propagation and system-level physical association are supported.",
        "The exact event-local command payload caused t2.",
        "same-trace source-to-Control plus aligned physical effect",
        "Archive event-local Bridge apply and Control payload to upgrade full-chain lineage.",
        "C",
    )
    claim(
        "C4.1131",
        "L4",
        "Observed physical reaction time violated a qualified dynamic temporal contract.",
        "C3.1131|P_DEADLINE.1131",
        "OBSERVED_DERIVED",
        "EV.REACTION.1131|EV.NODEADLINE.1131",
        "EV.NODEADLINE.1131",
        "D_DEADLINE.C4.1131",
        "IR-C4",
        "NOT_TESTABLE",
        "LOW",
        "LOW",
        1,
        "T_R is observed, but tau_req is not qualified.",
        "Observed contract status is not testable for this run.",
        "A collision proves a deadline miss.|The 799.636 ms response is a deadline violation.",
        "observed T_R compared with qualified tau_req interval",
        "Qualify an independent prospective deadline.",
    )
    claim(
        "C5.1131",
        "L5",
        "A requirement-constrained deadline-excess distance debt is established.",
        "C4.1131",
        "REQUIREMENT_CONSTRAINED_DERIVED",
        "EV.DRESPONSE.1131",
        "EV.NODEADLINE.1131",
        "",
        "IR-C5",
        "NOT_TESTABLE",
        "LOW",
        "LOW",
        1,
        "D_response is observed; primary D_debt requires qualified tau_req and is unavailable.",
        "13.432 m is observed response distance, not deadline-excess debt.",
        "13.432 m is timing-caused safety loss.|13.432 m is deadline debt.",
        "qualified C4 miss plus wall-clock velocity integration after deadline",
        "Qualify tau_req, then recompute D_debt from the saved velocity path.",
    )
    claim(
        "C6.1131",
        "L6",
        "A direct physical collision outcome occurred with actor 155.",
        "",
        "DIRECT_OBSERVED",
        "EV.COLLISION.1131",
        "",
        "",
        "IR-C6",
        "PASS",
        "HIGH",
        "HIGH",
        3,
        "Collision establishes outcome but not timing attribution.",
        "Collision and impact severity are directly observed.",
        "The collision proves a temporal contract miss.",
        "direct collision event and impact metric",
        "Causal attribution still requires C4/C5/P_FUNC closure.",
    )

    c7_default_types = {
        "D_INITIAL_CLEARANCE": "INITIAL_CLEARANCE",
        "D_INITIAL_SPEED": "INITIAL_SPEED",
        "D_BRAKING_CAPABILITY": "BRAKING_CAPABILITY",
        "D_FUNCTIONAL_FAILURE": "FUNCTIONAL_FAILURE",
        "D_TARGET_MISMATCH": "TARGET_MISMATCH",
        "D_DATA_FRESHNESS": "DATA_FRESHNESS",
        "D_UPDATE_GAP": "UPDATE_GAP",
        "D_SOLVER_FALLBACK": "SOLVER_FALLBACK",
        "D_CLOCK": "CLOCK",
        "D_PHASE": "PHASE",
        "D_PREHAZARD_STATE": "PREHAZARD_STATE",
        "D_GEOMETRY": "GEOMETRY",
        "D_OUTCOME_CONFLICT": "OUTCOME_CONFLICT",
    }
    c7_defeater_ids = "|".join(f"{prefix}.C7.1131" for prefix in c7_default_types)
    claim(
        "C7.1131",
        "ATTRIBUTION",
        "The observed timing behavior is established as the cause of the collision and quantified physical loss.",
        "C4.1131|C5.1131|C6.1131|P_FUNC.1131",
        "DIRECT_OBSERVED|TRACE_LINEAGE",
        "EV.FAULT.1131|EV.CHAIN.1131|EV.COLLISION.1131",
        "EV.FUNC.1131|EV.FRESH.1131|EV.POSTGAP.1131|EV.NODEADLINE.1131|EV.PHASE.1131",
        c7_defeater_ids,
        "IR-C7",
        "UNCERTAIN",
        "LOW",
        "LOW",
        2,
        "Unqualified deadline, partial functionality, event-local Bridge payload gap, phase, braking, and geometry alternatives remain.",
        "An injected temporal stressor, delayed physical response, later freshness degradation, and collision co-occur in one chain; causal share is unresolved.",
        "The delay caused the collision.|Timing was the sole cause.|The injected delay caused 13.432 m of safety loss.",
        "C4+C5+C6 with P_FUNC and closed defeaters",
        "Resolve deadline, payload, fallback, phase, braking-envelope, and counterfactual evidence.",
    )

    claim_fields = [
        "claim_id",
        "layer",
        "run_id_or_group",
        "proposition",
        "prerequisite_claim_ids",
        "required_evidence_classes",
        "supporting_evidence_ids",
        "challenging_evidence_ids",
        "defeater_ids",
        "inference_rule_id",
        "verdict",
        "confidence",
        "confidence_ceiling",
        "maximum_claim_level",
        "residual_uncertainty",
        "allowed_language",
        "forbidden_language",
        "causal_lineage_grade",
        "gate_inputs",
        "gate_metrics",
        "admissible_evidence",
        "gate_criterion",
        "next_gate_condition",
    ]
    write_csv("claim_ledger.csv", claim_rows, claim_fields)

    edges = []
    for row in claim_rows:
        for parent in str(row["prerequisite_claim_ids"]).split("|"):
            if parent:
                edges.append(
                    {
                        "parent_claim_id": parent,
                        "child_claim_id": row["claim_id"],
                        "relation": "REQUIRES",
                        "required": "TRUE",
                        "notes": "canonical TCPS-PA gate dependency",
                    }
                )
    write_csv(
        "claim_edges.csv",
        edges,
        ["parent_claim_id", "child_claim_id", "relation", "required", "notes"],
    )

    defeaters = [
        {
            "defeater_id": "D_FUNC.P_FUNC.1131",
            "claim_id": "P_FUNC.1131",
            "description": "Planning speed optimization falls back and command payload continuity is absent.",
            "type": "FUNCTIONAL_FAILURE",
            "evidence_ids": "EV.FUNC.1131",
            "status": "OPEN",
            "resolution": "Archive and validate target-specific Control/Bridge payload continuity and fallback safety semantics.",
            "residual_risk": "Functional behavior may independently affect braking outcome.",
            "impact_on_claim": "CAPS_AT_PARTIAL",
            "notes": "",
        },
        {
            "defeater_id": "D_DEADLINE.P_DEADLINE.1131",
            "claim_id": "P_DEADLINE.1131",
            "description": "Prospective braking envelope and safety policy are absent.",
            "type": "DEADLINE_PROVENANCE",
            "evidence_ids": "EV.NODEADLINE.1131",
            "status": "OPEN",
            "resolution": "Provide a pre-t1 locked, independently validated parameter envelope.",
            "residual_risk": "No qualified temporal requirement.",
            "impact_on_claim": "INVALIDATES",
            "notes": "",
        },
        {
            "defeater_id": "D_PAYLOAD.C3.1131",
            "claim_id": "C3.1131",
            "description": "Strict lineage ends at Control output; event-local Bridge payload/apply row is absent.",
            "type": "LINEAGE_GAP",
            "evidence_ids": "EV.CHAIN.1131",
            "status": "OPEN",
            "resolution": "Enable per-command event-local Bridge logging and archive Control payloads.",
            "residual_risk": "Physical t2 is only Grade C-associated with the exact command.",
            "impact_on_claim": "CAPS_AT_PARTIAL",
            "notes": "",
        },
        {
            "defeater_id": "D_DEADLINE.C4.1131",
            "claim_id": "C4.1131",
            "description": "No qualified tau_req exists for comparison with observed T_R.",
            "type": "DEADLINE_PROVENANCE",
            "evidence_ids": "EV.NODEADLINE.1131",
            "status": "OPEN",
            "resolution": "Construct and validate a prospective deadline independently of this outcome.",
            "residual_risk": "Observed miss verdict and miss time are not testable.",
            "impact_on_claim": "INVALIDATES",
            "notes": "",
        },
    ]
    c7_status = {
        "D_INITIAL_CLEARANCE": (
            "OPEN",
            "EV.REACTION.1131",
            "D1 is observed after pre-event fault exposure but no qualified safe threshold exists.",
        ),
        "D_INITIAL_SPEED": (
            "OPEN",
            "EV.REACTION.1131",
            "v1 is observed after 25.997 s of active delay and can mediate outcome severity.",
        ),
        "D_BRAKING_CAPABILITY": (
            "OPEN",
            "EV.COLLISION.1131",
            "Collision right-censors a full stopping trajectory; independent braking capability is absent.",
        ),
        "D_FUNCTIONAL_FAILURE": (
            "OPEN",
            "EV.FUNC.1131",
            "Planning fallback and missing command semantics remain viable contributors.",
        ),
        "D_TARGET_MISMATCH": (
            "RESOLVED",
            "EV.TARGET.1131",
            "Twenty matched frames and target-consistent STOP support actor identity.",
        ),
        "D_DATA_FRESHNESS": (
            "OPEN",
            "EV.FRESH.1131|EV.POSTGAP.1131",
            "Age at t2 and late lifecycle peaks remain unqualified but potentially relevant.",
        ),
        "D_UPDATE_GAP": (
            "OPEN",
            "EV.POSTGAP.1131",
            "507.439 ms gap is after t2: refuted for initial response, unresolved for continuing braking.",
        ),
        "D_SOLVER_FALLBACK": (
            "OPEN",
            "EV.FUNC.1131",
            "Twenty speed/constant-deceleration fallback events are observed.",
        ),
        "D_CLOCK": (
            "BOUNDED",
            "EV.CLOCK.1131",
            "p95 residual is 0.661 ms and does not change event ordering.",
        ),
        "D_PHASE": (
            "OPEN",
            "EV.PHASE.1131",
            "No active phase scan exists.",
        ),
        "D_PREHAZARD_STATE": (
            "OPEN",
            "EV.FAULT.1131|EV.REACTION.1131",
            "The delay was already active 25.997 s before t1; D1/v1 are post-treatment states.",
        ),
        "D_GEOMETRY": (
            "OPEN",
            "EV.TARGET.1131",
            "Fusion-to-actor median position error and 0.52 m offset uncertainty limit metric reconciliation.",
        ),
        "D_OUTCOME_CONFLICT": (
            "RESOLVED",
            "EV.COLLISION.1131",
            "Direct collision event and actor history agree on actor 155 and impact speed.",
        ),
    }
    for prefix, dtype in c7_default_types.items():
        status, evidence_ids, resolution = c7_status[prefix]
        defeaters.append(
            {
                "defeater_id": f"{prefix}.C7.1131",
                "claim_id": "C7.1131",
                "description": f"C7 alternative/limitation: {dtype.lower().replace('_', ' ')}.",
                "type": dtype,
                "evidence_ids": evidence_ids,
                "status": status,
                "resolution": resolution,
                "residual_risk": "Attribution remains limited." if status == "OPEN" else "Bounded by cited evidence.",
                "impact_on_claim": "CAPS_AT_PARTIAL" if status == "OPEN" else "BOUNDED",
                "notes": "",
            }
        )
    write_csv(
        "defeater_ledger.csv",
        defeaters,
        [
            "defeater_id",
            "claim_id",
            "description",
            "type",
            "evidence_ids",
            "status",
            "resolution",
            "residual_risk",
            "impact_on_claim",
            "notes",
        ],
    )

    guarantee_rows = [
        {
            "claim_id": "G1_GUARANTEE.1131",
            "run_id": RUN_ID,
            "guarantee_status": "NOT_ESTABLISHED",
            "qualified_path_bound_available": False,
            "qualified_suffix_bounds_available": False,
            "conditional_guarantee_loss_time_wall_s": "",
            "observed_contract_miss_time_wall_s": "",
            "reason": (
                "No formal/WCRT path bound, qualified dynamic deadline, or qualified suffix bounds. "
                "A fault onset and an observed reaction endpoint are not guarantee-loss points."
            ),
            "allowed_statement": "The run does not support a timing-guarantee-loss timestamp.",
        }
    ]
    write_csv("guarantee_status.csv", guarantee_rows)
    empirical_rows = [
        {
            "claim_id": "E1_EMPIRICAL.1131",
            "run_id": RUN_ID,
            "metric": "observed reaction time without requirement comparison",
            "value": tr_high_ms,
            "unit": "ms",
            "status": "DESCRIPTIVE_ONLY",
            "reason": "One event observation does not establish headroom, WCRT, or a guarantee.",
        }
    ]
    write_csv("empirical_headroom.csv", empirical_rows)

    diagnosis_rows = [
        {
            "hypothesis_id": "H.BRIDGE_DELAY.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "L1/L3",
            "candidate_component": "Bridge ControlDelayInjector and Control-to-physical segment",
            "candidate_fault_type": "fixed command delay",
            "hypothesis": "Persistent 300 ms Bridge delay contributed to the long Control-to-t2 segment.",
            "path_claim_ids": "C1.1131|C2.1131|C3.1131|C6.1131",
            "supporting_evidence_ids": "EV.FAULT.1131|EV.CHAIN.1131|EV.REACTION.1131",
            "challenging_evidence_ids": "EV.NODEADLINE.1131|EV.FUNC.1131",
            "alternative_hypothesis_ids": "H.FUNCTIONAL.1131|H.PHASE.1131|H.BRAKING.1131|H.FRESHNESS.1131",
            "required_prerequisite_claim_ids": "P_CLOCK.1131|P_TARGET.1131",
            "diagnosability_class": "NON_UNIQUE_SINGLE_RUN",
            "equivalence_class_id": "EQ.CONTROL_TO_PHYSICAL.1131",
            "status": "CONSISTENT_BUT_UNRESOLVED",
            "rank_score": 0.75,
            "rank_method": "event relevance + direct stressor evidence - open alternatives",
            "maximum_diagnosis_strength": "SYSTEM_LEVEL_ASSOCIATION",
            "discriminating_test": "event-local per-command Bridge apply logging and zero-delay replay under matched state",
            "residual_uncertainty": "event command payload and counterfactual are absent",
            "allowed_language": "Bridge delay is a leading event-relevant contributor candidate.",
            "forbidden_language": "Bridge delay caused the collision.",
        },
        {
            "hypothesis_id": "H.FUNCTIONAL.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "P_FUNC/L3",
            "candidate_component": "Planning speed optimization/fallback and downstream command semantics",
            "candidate_fault_type": "functional degradation",
            "hypothesis": "Planning fallback or command semantics materially affected braking despite a STOP decision.",
            "path_claim_ids": "P_FUNC.1131|C3.1131|C6.1131",
            "supporting_evidence_ids": "EV.FUNC.1131",
            "challenging_evidence_ids": "EV.CHAIN.1131",
            "alternative_hypothesis_ids": "H.BRIDGE_DELAY.1131|H.BRAKING.1131|H.FRESHNESS.1131",
            "required_prerequisite_claim_ids": "P_TARGET.1131",
            "diagnosability_class": "NON_UNIQUE_SINGLE_RUN",
            "equivalence_class_id": "EQ.CONTROL_TO_PHYSICAL.1131",
            "status": "CONSISTENT_BUT_UNRESOLVED",
            "rank_score": 0.65,
            "rank_method": "functional anomaly proximity + missing payload penalty",
            "maximum_diagnosis_strength": "FUNCTIONAL_OR_MULTI_FACTOR_CANDIDATE",
            "discriminating_test": "decode record/Control payload and verify commanded brake/trajectory continuity",
            "residual_uncertainty": "fallback output is nonempty and status_ok; semantic effect unknown",
            "allowed_language": "Planning fallback prevents timing-only attribution.",
            "forbidden_language": "Fallback caused the collision.",
        },
        {
            "hypothesis_id": "H.FRESHNESS.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "L2/L3",
            "candidate_component": "Fusion target update continuity after t2",
            "candidate_fault_type": "staleness/update gap",
            "hypothesis": "Post-t2 target staleness degraded continuing braking updates before collision.",
            "path_claim_ids": "C2.1131|C3.1131|C6.1131",
            "supporting_evidence_ids": "EV.POSTGAP.1131|EV.FRESH.1131",
            "challenging_evidence_ids": "EV.NODEADLINE.1131",
            "alternative_hypothesis_ids": "H.BRIDGE_DELAY.1131|H.FUNCTIONAL.1131|H.BRAKING.1131",
            "required_prerequisite_claim_ids": "P_CLOCK.1131|P_TARGET.1131",
            "diagnosability_class": "POST_T2_ONLY",
            "equivalence_class_id": "EQ.POST_T2_CONTINUOUS_CONTROL.1131",
            "status": "CONSISTENT_BUT_UNRESOLVED",
            "rank_score": 0.50,
            "rank_method": "temporal order + downstream payload gap",
            "maximum_diagnosis_strength": "LATE_CHAIN_CANDIDATE",
            "discriminating_test": "trace each post-t2 target source to the actual applied ControlCommand",
            "residual_uncertainty": "no event-local applied commands or freshness requirement",
            "allowed_language": "Late freshness degradation is relevant only after t2.",
            "forbidden_language": "The 507 ms gap explains the initial response delay.",
        },
        {
            "hypothesis_id": "H.INITIAL_GAP.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "L2/L3",
            "candidate_component": "Fusion target updates",
            "candidate_fault_type": "update gap",
            "hypothesis": "The run-wide 507.439 ms target gap caused the first t1-to-t2 response delay.",
            "path_claim_ids": "C2.1131|C3.1131",
            "supporting_evidence_ids": "",
            "challenging_evidence_ids": "EV.POSTGAP.1131",
            "alternative_hypothesis_ids": "",
            "required_prerequisite_claim_ids": "P_CLOCK.1131|P_TARGET.1131",
            "diagnosability_class": "ORDER_REFUTED",
            "equivalence_class_id": "EQ.INITIAL_RESPONSE.1131",
            "status": "REFUTED",
            "rank_score": 0.0,
            "rank_method": "event-order test",
            "maximum_diagnosis_strength": "REFUTED_FOR_INITIAL_RESPONSE",
            "discriminating_test": "already resolved by timestamp ordering",
            "residual_uncertainty": "may still affect post-t2 continuing control",
            "allowed_language": "The largest gap starts after t2 and cannot explain first response onset.",
            "forbidden_language": "The largest gap caused t2 delay.",
        },
        {
            "hypothesis_id": "H.PHASE.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "L1/L2",
            "candidate_component": "100 ms CARLA/Localization tick phase",
            "candidate_fault_type": "phase/tick quantization",
            "hypothesis": "Unfavorable phase amplified Control-to-physical observation latency.",
            "path_claim_ids": "P_PHASE.1131|C3.1131",
            "supporting_evidence_ids": "",
            "challenging_evidence_ids": "EV.PHASE.1131",
            "alternative_hypothesis_ids": "H.BRIDGE_DELAY.1131|H.FUNCTIONAL.1131",
            "required_prerequisite_claim_ids": "P_CLOCK.1131",
            "diagnosability_class": "NO_PHASE_SCAN",
            "equivalence_class_id": "EQ.CONTROL_TO_PHYSICAL.1131",
            "status": "NOT_TESTABLE",
            "rank_score": 0.25,
            "rank_method": "open alternative without intervention",
            "maximum_diagnosis_strength": "HYPOTHESIS_ONLY",
            "discriminating_test": "controlled phase scan with matched repeats",
            "residual_uncertainty": "phase is unobserved as an intervention",
            "allowed_language": "Phase remains an alternative.",
            "forbidden_language": "Phase caused the response delay.",
        },
        {
            "hypothesis_id": "H.BRAKING.1131",
            "run_id_or_group": RUN_ID,
            "seed_claim_id": "C6.1131",
            "seed_evidence_ids": "EV.COLLISION.1131",
            "candidate_layer": "L4/L6",
            "candidate_component": "vehicle/controller braking capability",
            "candidate_fault_type": "insufficient or uncertain braking envelope",
            "hypothesis": "Available braking capability or actuation dynamics were insufficient for the encountered state.",
            "path_claim_ids": "P_FUNC.1131|P_DEADLINE.1131|C6.1131",
            "supporting_evidence_ids": "",
            "challenging_evidence_ids": "EV.NODEADLINE.1131",
            "alternative_hypothesis_ids": "H.BRIDGE_DELAY.1131|H.FUNCTIONAL.1131|H.FRESHNESS.1131",
            "required_prerequisite_claim_ids": "P_TARGET.1131",
            "diagnosability_class": "RIGHT_CENSORED",
            "equivalence_class_id": "EQ.PHYSICAL_ENVELOPE.1131",
            "status": "NOT_TESTABLE",
            "rank_score": 0.35,
            "rank_method": "physical alternative with right-censored full stop",
            "maximum_diagnosis_strength": "HYPOTHESIS_ONLY",
            "discriminating_test": "independent prospective braking-envelope calibration in the same micro-ODD",
            "residual_uncertainty": "collision prevents full-stop observation",
            "allowed_language": "Braking capability remains unresolved.",
            "forbidden_language": "The vehicle could not have stopped in time.",
        },
    ]
    diagnosis_fields = [
        "hypothesis_id",
        "run_id_or_group",
        "seed_claim_id",
        "seed_evidence_ids",
        "candidate_layer",
        "candidate_component",
        "candidate_fault_type",
        "hypothesis",
        "path_claim_ids",
        "supporting_evidence_ids",
        "challenging_evidence_ids",
        "alternative_hypothesis_ids",
        "required_prerequisite_claim_ids",
        "diagnosability_class",
        "equivalence_class_id",
        "status",
        "rank_score",
        "rank_method",
        "maximum_diagnosis_strength",
        "discriminating_test",
        "residual_uncertainty",
        "allowed_language",
        "forbidden_language",
    ]
    write_csv("diagnosis_hypothesis_ledger.csv", diagnosis_rows, diagnosis_fields)
    diagnosis_edges = [
        {
            "parent_id": "C6.1131",
            "child_id": row["hypothesis_id"],
            "relation": "SEEDS_DIAGNOSIS",
            "time_direction": "BACKWARD_DIAGNOSTIC",
            "required": "TRUE",
            "notes": "diagnostic traversal is not reversed causal proof",
        }
        for row in diagnosis_rows
    ]
    write_csv(
        "diagnosis_edges.csv",
        diagnosis_edges,
        ["parent_id", "child_id", "relation", "time_direction", "required", "notes"],
    )

    record_rows = [
        {
            "run_id": RUN_ID,
            "record_profile_available": False,
            "record_timing_diagnostic_status": "NOT_AVAILABLE",
            "reaction_time_message_diagnostic_ms": "",
            "data_age_record_diagnostic_ms": "",
            "channel": "",
            "message_count": "",
            "source_file": "",
            "evidence_class": "MISSING",
            "scope_warning": (
                "No record was captured for 1131. No other-run record profile is borrowed. "
                "Control payload continuity and module receive/send details remain unavailable."
            ),
        }
    ]
    write_csv("record_timing_diagnostics.csv", record_rows)

    rag_rows = []
    rag_values = [
        ("R", "physical reaction", "T_e2e_data_observed_ms", "ms", tr_high_ms, "t1 to t2"),
        ("A", "target data age at t2", "data_age_target_at_t2_data_observed_ms", "ms", age_t2_ms, "latest source represented before t2"),
        ("G", "target output gap in response window", "update_gap_target_response_window_max_data_observed_ms", "ms", float(np.max(response_gaps)), "maximum of four within-window gaps"),
    ]
    for dimension, metric, column, unit, value, semantics in rag_values:
        rag_rows.append(
            {
                "dimension": dimension,
                "metric": metric,
                "source_column": column,
                "unit": unit,
                "semantics": semantics,
                "group_name": "single_run_1131_descriptive",
                "n_total_runs": 1,
                "n_available_runs": 1,
                "p50": value,
                "p90": value,
                "p95": value,
                "p99": value,
                "max": value,
                "iqr": 0.0,
            }
        )
    write_csv("realtime_rag_summary.csv", rag_rows)
    write_csv(
        "group_summary_observed.csv",
        [
            {
                "scope": "single_run_only_not_group_comparison",
                "run_id": RUN_ID,
                "n": 1,
                "T_R_observed_ms": tr_high_ms,
                "D_response_observed_m": d_response,
                "impact_speed_observed_mps": raw["impact_speed_mps"],
                "notes": "No cross-run baseline or group statistic is used.",
            }
        ],
    )
    write_csv(
        "space_budget_decomposition_observed.csv",
        [
            {
                "run_id": RUN_ID,
                "group_name": "single_run_1131",
                "included_main_analysis": True,
                "outcome_data_observed": "collision",
                "D1_clear_data_observed_m": observed["D1_clear_data_observed_m"],
                "D_response_wall_integral_data_observed_m": d_response,
                "D_brake_data_observed_m": "",
                "M0_recomputed_observed_m": "",
                "endpoint_compatible_full_stop": False,
                "decomposition_scope": (
                    "COLLISION_RIGHT_CENSORED; D_brake_truncated is reported separately and is not inserted into full-stop M0"
                ),
            }
        ],
    )
    write_csv(
        "space_budget_group_decomposition.csv",
        [
            {
                "comparison_group": "single_run_1131_not_comparison",
                "n": 1,
                "D1_mean_m": observed["D1_clear_data_observed_m"],
                "D_response_mean_m": d_response,
                "D_brake_mean_m": "",
                "M0_mean_m": "",
            }
        ],
    )

    method_rows = [
        ("single-run scope; no other-run evidence", "PASS", "parser discovers only 202607271131 for computation"),
        ("raw input inventory and hashes", "PASS", f"{inventory['file_count']} files; sha256 saved"),
        ("wall-clock t1/t2 endpoint definitions", "PASS", "event_timeline.csv E01/E07"),
        ("P_CLOCK alignment audit", "PASS", f"p95 residual {parsed.clock['p95_abs_residual_ms']:.3f} ms"),
        ("P_PHASE active scan", "MISSING", "no matched phase intervention"),
        ("target identity continuity", "PASS", f"{identity['matched_frame_count']} matched frames"),
        ("functional correctness qualification", "PARTIAL", "STOP exists; fallback and payload gaps remain"),
        ("same-instance source-to-Control trace", "PASS", f"trace {target_trace_id}"),
        ("Control-to-physical strict lineage", "PARTIAL", "Grade C temporal/semantic association"),
        ("R/A/G explicit outputs", "PASS", "realtime_rag_summary.csv and stage table"),
        ("prospective dynamic deadline", "MISSING", "no independent braking envelope or locked safety buffer"),
        ("timing guarantee bound", "MISSING", "no WCRT/suffix bound"),
        ("canonical D_response wall integral", "PASS", f"{d_response:.6f} m"),
        ("requirement-constrained D_debt", "NOT_TESTABLE", "no qualified tau_req"),
        ("full-stop observed space budget", "NOT_TESTABLE", "collision right-censoring"),
        ("direct physical outcome", "PASS", f"collision at {raw['impact_speed_mps']:.3f} m/s"),
        ("causal attribution", "UNCERTAIN", "open functional, phase, braking, geometry and deadline defeaters"),
        ("record-derived message audit", "MISSING", "record was not enabled for run 1131"),
        ("observed/model separation", "PASS", "model table explicitly reports NOT_COMPUTED"),
    ]
    write_csv(
        "method_completeness_matrix.csv",
        [
            {"requirement": requirement, "status": status, "evidence_or_gap": detail}
            for requirement, status, detail in method_rows
        ],
    )
    write_csv(
        "exclusions_and_missing.csv",
        [
            {
                "run_id": RUN_ID,
                "item": "parsed Apollo record",
                "status": "MISSING",
                "reason": "simulation recording was not enabled",
                "effect": "module message payload/receive/send and Control continuity not auditable",
            },
            {
                "run_id": RUN_ID,
                "item": "qualified dynamic deadline",
                "status": "MISSING",
                "reason": "no prospective independent braking envelope/safety policy",
                "effect": "C4, guarantee-loss time, primary D_debt are NOT_TESTABLE",
            },
            {
                "run_id": RUN_ID,
                "item": "full-stop braking endpoint",
                "status": "RIGHT_CENSORED",
                "reason": "collision occurs before full stop",
                "effect": "full observed D_brake and M0 unavailable",
            },
            {
                "run_id": RUN_ID,
                "item": "event-local Bridge applied command rows",
                "status": "MISSING_BY_LOGGING_POLICY",
                "reason": "log_all_delayed_commands=false",
                "effect": "Control-to-physical lineage capped at Grade C",
            },
            {
                "run_id": RUN_ID,
                "item": "multi-run comparison",
                "status": "EXCLUDED_BY_SCOPE",
                "reason": "user requested one-run analysis",
                "effect": "no baseline restoration/counterfactual imported",
            },
        ],
    )

    # Compatibility view: all six layers are explicit.
    layer_rows = [
        ("L1", "Bridge fixed delay", raw["scb_actual_wall_delay_ms"], "ms", "DIRECT_OBSERVED", str(scb_path), "AVAILABLE"),
        ("L2", "R/A/G diagnostics", f"R={tr_high_ms:.3f};A={age_t2_ms:.3f};G={float(np.max(response_gaps)):.3f}", "ms", "OBSERVED_DERIVED", str(TABLES / "realtime_rag_summary.csv"), "AVAILABLE"),
        ("L3", "source-to-Control lineage", raw["sensor_to_control_ms"], "ms", "TRACE_LINEAGE", parsed.trace["source_files"]["control_context"], "AVAILABLE"),
        ("L4", "qualified dynamic deadline", "", "ms", "MISSING", str(TABLES / "dynamic_deadline_construction.csv"), "NOT_AVAILABLE"),
        ("L5", "D_response wall integral", d_response, "m", "OBSERVED_DERIVED", raw["source_localization_file"], "AVAILABLE"),
        ("L6", "impact speed", raw["impact_speed_mps"], "m/s", "DIRECT_OBSERVED", raw["source_collision_file"], "AVAILABLE"),
    ]
    write_csv(
        "layer_evidence_matrix.csv",
        [
            {
                "layer": layer,
                "metric": metric,
                "run_id_or_group": RUN_ID,
                "value": value,
                "unit": unit,
                "evidence_type": evidence_type,
                "time_basis": "wall_epoch_s" if layer != "L3" else "trace anchored to wall",
                "source_file": source,
                "availability": availability,
            }
            for layer, metric, value, unit, evidence_type, source, availability in layer_rows
        ],
    )

    # Placeholders are generated now and overwritten by validator-authoritative L5 recomputation.
    write_csv(
        "l5_recomputation.csv",
        [
            {
                "run_id": RUN_ID,
                "requirement_id": "",
                "t1_wall_s": t1,
                "t_deadline_wall_s": "",
                "te_wall_s": t2,
                "D_response_recomputed_m": d_response,
                "D_debt_recomputed_m": "",
                "D_response_reported_m": d_response,
                "D_debt_reported_m": "",
                "D1_observed_m": observed["D1_clear_data_observed_m"],
                "D_brake_observed_m": "",
                "M0_recomputed_m": "",
                "M0_reported_m": "",
                "endpoint_coverage": "COVERED_WITH_LINEAR_INTERPOLATION",
                "max_abs_error_m": 0.0,
                "tolerance_m": 0.02,
                "recomputation_status": "RESPONSE_ONLY",
                "notes": "No qualified per-run tau_req; primary D_debt is not recomputed.",
            }
        ],
    )

    # Chart contracts / map.
    chart_map = [
        {
            "figure": "event_chain_timeline.png",
            "question": "Where is time consumed from t1 to collision?",
            "takeaway": "799.636 ms to physical braking; Control-to-t2 is the largest segment.",
            "family": "ordered event timeline",
            "data_rows": len(event_timeline) - 1,
            "palette_policy": "hard two-root cap",
            "non_color_encoding": "direct labels and event markers",
            "source": "event_timeline.csv",
        },
        {
            "figure": "response_stage_decomposition.png",
            "question": "Which initial-response segments dominate?",
            "takeaway": "Control-to-t2 and source-to-Fusion account for nearly all observed T_R.",
            "family": "ordered horizontal bar",
            "data_rows": 5,
            "palette_policy": "hard two-root cap",
            "non_color_encoding": "exact values and ordered labels",
            "source": "stage_timing_and_freshness.csv",
        },
        {
            "figure": "speed_and_events.png",
            "question": "How did the physical state evolve around t1, t2 and collision?",
            "takeaway": "Speed rises until the t2 sample, then falls but remains 7.988 m/s at impact.",
            "family": "line with event references",
            "data_rows": len(velocity_rows),
            "palette_policy": "single-root preferred",
            "non_color_encoding": "line plus labeled vertical markers",
            "source": "velocity_trajectory_observed.csv",
        },
        {
            "figure": "target_freshness_timeline.png",
            "question": "When did target freshness and continuity degrade?",
            "takeaway": "The largest gap and ~700 ms lifecycle occur after t2, before collision.",
            "family": "two-panel line/stem",
            "data_rows": len(target_timeline),
            "palette_policy": "hard two-root cap",
            "non_color_encoding": "markers, line styles and t2/collision references",
            "source": "target_freshness_timeline.csv",
        },
    ]
    write_csv("chart_map.csv", chart_map)

    # Figures.
    blue = "#2563A7"
    orange = "#D97706"
    charcoal = "#263238"
    light_blue = "#DCEAF7"
    grey = "#8A949B"
    red = "#A33A3A"

    timeline_plot_rows = [row for row in event_timeline if row["event_id"] != "E00"]
    x = np.asarray([float(row["relative_t1_ms"]) for row in timeline_plot_rows])
    labels = [row["label"] for row in timeline_plot_rows]
    fig, ax = plt.subplots(figsize=(11.5, 4.8))
    ax.axvspan(0, tr_high_ms, color=light_blue, alpha=0.75, label="t1→t2 observed response")
    ax.hlines(0, 0, (t_collision - t1) * 1000.0, color=charcoal, linewidth=1.5)
    colors = [red if row["event_id"] == "E09" else orange if row["event_id"] == "E07" else blue for row in timeline_plot_rows]
    ax.scatter(x, np.zeros_like(x), c=colors, s=55, zorder=3, edgecolor="white", linewidth=0.8)
    levels = [0.55, -0.65, 0.85, -0.95, 1.15, -1.25, 1.45, -1.55, 1.75]
    for xpos, label, level in zip(x, labels, levels):
        short = label.replace("stable ", "").replace(" for causal trace", "")
        ax.vlines(xpos, 0, level, color=grey, linewidth=0.8)
        ax.text(xpos, level, f"{short}\n{xpos:.1f} ms", ha="center", va="bottom" if level > 0 else "top", fontsize=8.2)
    ax.set_ylim(-2.0, 2.15)
    ax.set_xlim(-60, (t_collision - t1) * 1000.0 + 120)
    ax.set_yticks([])
    ax.set_xlabel("Relative to t1 (wall-clock ms)")
    ax.set_title("Run 1131 event-chain timeline")
    ax.text(0.01, 0.98, "Single target trace; physical endpoint and outcome shown separately", transform=ax.transAxes, va="top", color=grey)
    ax.grid(axis="x", color="#E5E7EB", linewidth=0.7)
    fig.savefig(FIGURES / "event_chain_timeline.png")
    plt.close(fig)

    stage_names = [
        "Source→Fusion",
        "Fusion→Prediction",
        "Prediction→Planning STOP",
        "Planning STOP→Control",
        "Control→physical t2",
    ]
    stage_values = [
        float(raw["sensor_to_perception_ms"]),
        float(raw["perception_to_prediction_ms"]),
        float(raw["prediction_to_planning_stop_ms"]),
        float(raw["planning_stop_to_control_ms"]),
        float(raw["control_to_effective_brake_ms"]),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 5.1))
    y = np.arange(len(stage_names))
    bar_colors = [blue, blue, blue, blue, orange]
    bars = ax.barh(y, stage_values, color=bar_colors, edgecolor=charcoal, linewidth=0.4)
    ax.set_yticks(y, stage_names)
    ax.invert_yaxis()
    ax.set_xlim(0, max(stage_values) * 1.18)
    ax.set_xlabel("Observed duration (ms)")
    fig.suptitle("Run 1131 initial-response stage decomposition", y=0.98, fontsize=16)
    ax.text(
        0.0,
        1.015,
        f"Ordered same-instance chain; total T_R = {tr_high_ms:.3f} ms",
        transform=ax.transAxes,
        color=grey,
        va="bottom",
    )
    for bar, value in zip(bars, stage_values):
        ax.text(value + 6, bar.get_y() + bar.get_height() / 2, f"{value:.3f} ms", va="center", fontsize=9)
    ax.grid(axis="x", color="#E5E7EB", linewidth=0.7)
    fig.subplots_adjust(top=0.84)
    fig.savefig(FIGURES / "response_stage_decomposition.png")
    plt.close(fig)

    plot_samples = [sample for sample in parsed.localization if t1 - 0.5 <= sample.time_s <= t_collision + 0.2]
    tx = np.asarray([(sample.time_s - t1) for sample in plot_samples])
    speeds = np.asarray([sample.speed_mps for sample in plot_samples])
    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.plot(tx, speeds, color=blue, linewidth=2.0, marker="o", markersize=3.5, label="Localization speed")
    refs = [(0.0, "t1", charcoal), (t2 - t1, "t2", orange), (t_collision - t1, "collision", red)]
    for xpos, label, color in refs:
        ax.axvline(xpos, color=color, linestyle="--", linewidth=1.3)
        ax.text(xpos, ax.get_ylim()[1], label, color=color, ha="right", va="top", fontsize=9)
    ax.scatter([0, t2 - t1, t_collision - t1], [observed["v1_data_observed_mps"], observed["v2_data_observed_mps"], raw["impact_speed_mps"]], c=[charcoal, orange, red], s=55, zorder=4)
    ax.set_xlabel("Relative to t1 (s, wall clock)")
    ax.set_ylabel("Speed (m/s)")
    fig.suptitle("Run 1131 speed trajectory and physical events", y=0.98, fontsize=16)
    ax.text(
        0.0,
        1.015,
        "Collision marker uses direct event pre-impact speed; the Localization line includes impact response",
        transform=ax.transAxes,
        color=grey,
        va="bottom",
    )
    ax.grid(color="#E5E7EB", linewidth=0.7)
    fig.subplots_adjust(top=0.84)
    fig.savefig(FIGURES / "speed_and_events.png")
    plt.close(fig)

    target_x = np.asarray([row.header_time_s - t1 for row in target_rows])
    lifecycles = np.asarray([(row.header_time_s - row.obs_time_s) * 1000.0 for row in target_rows])
    gaps = np.asarray([math.nan] + [(target_rows[index].header_time_s - target_rows[index - 1].header_time_s) * 1000.0 for index in range(1, len(target_rows))])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True)
    axes[0].plot(target_x, lifecycles, color=blue, marker="o", linewidth=1.8, label="source age at Fusion output")
    axes[0].set_ylabel("Lifecycle age (ms)")
    axes[0].grid(color="#E5E7EB", linewidth=0.7)
    axes[1].stem(target_x, gaps, linefmt=orange, markerfmt="o", basefmt=" ")
    axes[1].set_ylabel("Output gap (ms)")
    axes[1].set_xlabel("Fusion output time relative to t1 (s)")
    axes[1].grid(color="#E5E7EB", linewidth=0.7)
    for ax in axes:
        ax.axvline(t2 - t1, color=orange, linestyle="--", linewidth=1.2)
        ax.axvline(t_collision - t1, color=red, linestyle="--", linewidth=1.2)
    axes[0].text(t2 - t1, max(lifecycles) * 0.98, "t2", color=orange, ha="right", va="top")
    axes[0].text(t_collision - t1, max(lifecycles) * 0.98, "collision", color=red, ha="right", va="top")
    fig.suptitle("Run 1131 target freshness and update continuity", y=0.985, fontsize=16)
    axes[0].text(
        0.0,
        1.02,
        "Largest gap/lifecycle degradation begins after t2 and remains pre-collision",
        transform=axes[0].transAxes,
        color=grey,
        va="bottom",
    )
    fig.subplots_adjust(top=0.88, hspace=0.20)
    fig.savefig(FIGURES / "target_freshness_timeline.png")
    plt.close(fig)

    # Report and quality audit.
    stage_share_fusion = float(raw["sensor_to_perception_ms"]) / tr_high_ms * 100.0
    stage_share_control = float(raw["control_to_effective_brake_ms"]) / tr_high_ms * 100.0
    report_text = f"""# 第二次实验 1131 run 单次实时性—物理安全双向诊断

**方法：TCPS-PA v3.1 单-run、事件中心、Claim–Evidence 诊断**  
**范围：仅 `202607271131`；未使用其他 run 的基线、制动模型、deadline 或反事实轨迹**

## 结论先行

1131 run 存在清楚的**事件级时间退化现象**：从首个稳定目标源帧 $t_1$ 到首个持续减速采样 $t_2$ 经过 **{tr_high_ms:.3f} ms**，期间车速由 **{float(observed['v1_data_observed_mps']):.3f} m/s** 升至 **{float(observed['v2_data_observed_mps']):.3f} m/s**，按墙钟速度梯形积分前进 **{d_response:.3f} m**。时间消耗集中在两处：

- source→Fusion 为 **{float(raw['sensor_to_perception_ms']):.3f} ms**，占 $T_R$ 的 **{stage_share_fusion:.1f}%**；
- Control 输出→物理 $t_2$ 为 **{float(raw['control_to_effective_brake_ms']):.3f} ms**，占 **{stage_share_control:.1f}%**，是最大单段。

本 run 确认 Bridge 的 300 ms 固定延时在 $t_1$ 前约 **{abs(t_fault - t1):.3f} s** 已触发；实现语义是触发后持续延时后续 ControlCommand，但本 run 因 `log_all_delayed_commands=false` 没有事件对应命令的逐条 Bridge apply 记录。因此可将 Bridge 延时定位为**主要事件相关候选因素**，但不能从单 run 证明它导致碰撞。

Planning 对同一目标 11 产生 STOP，但速度优化失败并进入常减速 fallback（首个事件输出 `status_ok=1`、98 轨迹点、`max_abs_decel=4 m/s²`）；加上 Control 载荷和事件级 Bridge apply 证据缺失，`P_FUNC=PARTIAL`。所以不得把此案表述为“功能正确但仅时间错误”。

物理后果是直接观测的：在 $t_1+{(t_collision-t1):.3f}$ s 与 actor 155 碰撞，冲击速度 **{float(raw['impact_speed_mps']):.3f} m/s**，冲量模 **{float(raw['collision_impulse_norm']):.1f}**。$t_2$到碰撞的墙钟速度积分是 **{d_brake_truncated:.3f} m**，但它是碰撞右截尾距离，不是完整制动距离。

最重要的否定性结论是：**本 run 无合格、事前锁定且独立验证的动态物理 deadline**，也无 WCRT/后缀上界。因此 `C4_OBS=NOT_TESTABLE`、`G1_GUARANTEE=NOT_ESTABLISHED`，不存在可报告的“失去时间保证时刻”；主 `D_debt` 也不可用。

## Six-Layer Inference Status Matrix

| 层/主张 | 判定 | 本 run 证据上限 |
|---|---|---|
| L1 / C1 | PASS | 300 ms Bridge 时间干预实际进入系统；不等于 Apollo 内生缺陷 |
| L2 / C2 | PASS | Bridge 局部延时显化；A/G 无独立要求，仅作诊断量 |
| L3 / C3 | PARTIAL_PASS | source→Control 为 Grade A，Control→物理 $t_2$ 为 Grade C |
| L4 / C4 | NOT_TESTABLE | 已观测 $T_R$，但无合格 $\\tau_{{req}}$ |
| L5 / C5 | NOT_TESTABLE | $D_{{response}}$ 可用；requirement-constrained $D_{{debt}}$ 不可用 |
| L6 / C6 | PASS | 碰撞、对方 actor、冲击速度和冲量直接观测 |
| Attribution / C7 | UNCERTAIN | 只能报告系统级关联和候选机制，不能定量因果份额 |

v3.1 独立状态：`P_CLOCK=PASS`、`P_TARGET=PASS`、`P_FUNC=PARTIAL`、`P_PHASE=NOT_TESTABLE`、`P_DEADLINE=NOT_TESTABLE`、`G1_GUARANTEE=NOT_ESTABLISHED`、`E1_EMPIRICAL=DESCRIPTIVE_ONLY`。

## 事件和端点定义

| 端点 | 墙钟时间（Asia/Shanghai） | 相对 $t_1$ | 定义 |
|---|---|---:|---|
| fault onset | {iso(t_fault)} | {(t_fault-t1)*1000:.3f} ms | SCB 首条有效制动命令 receive/trigger |
| $t_1$ | {iso(t1)} | 0 ms | 目标 11 三帧稳定序列首帧的 source timestamp |
| Fusion | {iso(t_fusion)} | {(t_fusion-t1)*1000:.3f} ms | 同 trace 的稳定 Fusion 输出 |
| Prediction | {iso(t_prediction)} | {(t_prediction-t1)*1000:.3f} ms | 同 trace 静态目标预测 |
| Planning STOP | {iso(t_stop)} | {(t_stop-t1)*1000:.3f} ms | target 11 STOP |
| Control | {iso(t_control)} | {(t_control-t1)*1000:.3f} ms | 同 trace `cmd_write_enter/output_pub` |
| $t_2$ | {iso(t2)} | {tr_high_ms:.3f} ms | 首个持续减速采样：连续 2 区间 $a\\le-0.5$ m/s² 且 0.3 s 内掉速≥0.3 m/s |
| collision | {iso(t_collision)} | {(t_collision-t1)*1000:.3f} ms | CARLA collision event，不使用 history 首帧替代 |

$t_2$ 受约 100 ms Localization 采样粒度，保守采样夹取为 **[{tr_low_ms:.3f}, {tr_high_ms:.3f}] ms**；对 raw $v(t)$ 使用 0.3/0.5/1.0 m/s² 门限都得到同一 $t_2$，median-3 平滑则延至 899.502 ms，见 `t2_sensitivity.csv`。

![事件链时间线](../figures/event_chain_timeline.png)

## R/A/G 与时间消耗位置

| 维度 | 观测值 | 事件范围 | 判定 |
|---|---:|---|---|
| Reaction $R$ | {tr_high_ms:.3f} ms | $t_1\\to t_2$ | 可观测；无 deadline，不可判 miss |
| Age $A$ | {age_t2_ms:.3f} ms | $t_2$ 时最新已 Fusion 目标源数据 | 可观测；无 freshness requirement |
| Gap $G$ | {float(np.max(response_gaps)):.3f} ms | $[t_1,t_2]$ 内 5 个目标输出 | 可观测；无 gap requirement |
| 后续 $G_{{max}}$ | {max_gap_ms:.3f} ms | $t_2$ 后、碰撞前 | 不能解释首次 $t_2$；可影响持续闭环的候选 |
| 碰撞时目标源数据年龄 | {age_collision_ms:.3f} ms | 最后已 Fusion 目标源帧到碰撞 | 后续新鲜度退化的直接诊断量 |

同一 trace 的 source→Fusion 细分中，sensor→Preprocess 入口年龄 **{float(parsed.trace['sensor_anchor']['ingress_ms']):.3f} ms**、Ground 输出→Detection 进入等待 **{(det_enter-ground_out)/1e6:.3f} ms**、Lidar Detection 处理 **{(det_out-det_enter)/1e6:.3f} ms**；三者是 Perception 时间的主要组成。细分和与 trace E2E 差 **{abs(decomposition_sum-fusion_trace_e2e):.6f} ms**。

![首次响应分段](../figures/response_stage_decomposition.png)

Control→$t_2$ 的 486.037 ms 是最大分段。以已记录的 300.047 ms 注入精度做**非因果算术分解**，剩余 **{post_proxy_residual_ms:.3f} ms**；它混合事件命令未记录的排队/释放、CARLA tick、车辆动力学、Localization 采样及持续减速确认窗，不得命名为 Apollo 计算延时。

## 双向定位：哪里、什么性质、为什么

1. **Bridge/SCB（L1）**：直接证据确认的 300.047 ms 外部注入型固定时延；它表明 SUT 在注入干领下的行为，不是 Apollo 内生实时缺陷证明。
2. **Perception 源数据年龄（L2/L3）**：首帧 source→Fusion 为 292.885 ms，主要由 101.614 ms 入口年龄、61.478 ms Detection 前等待和 98.163 ms Detection 处理组成。这是精确段级定位，但缺局部要求/基准，不升格为 violation。
3. **Planning 功能退化（P_FUNC）**：STOP 后在 $t_1+{(float(fallback_first['time_s'])-t1)*1000:.3f}$ ms 出现 speed fallback，后续常减速 fallback 共 {int(raw['planning_constant_deceleration_fallback_count'])} 次。它在首次因果窗内，是事件相关功能候选，但不是引起 486 ms 的处理时间瓶颈。
4. **Control→物理效应（L3）**：486.037 ms 是对初始响应最有关联的位置，但精确命令载荷/apply 不在归档中，全链只能 Grade C。
5. **$t_2$ 后持续闭环（L2/L3）**：507.439 ms 输出缺口与约 700 ms lifecycle 峰值发生在 $t_2$ 之后。它们对“为何首次制动晚”已被时序反证，但对碰撞前持续制动新鲜度仍是未解候选。

![目标新鲜度](../figures/target_freshness_timeline.png)

## 时间保证与动态契约

本次可以观测 $T_R$，却不能合格构造 $\\tau_{{req}}$，因为缺少：

- $t_1$ 前锁定的 $d_{{safe}}$ 政策；
- 独立验证的 ego 最小制动能力/响应期加速上界；
- 目标行为与路面条件的有界包络；
- 与评估 run 独立的校准/验证数据。

碰撞已把完整停车轨迹右截尾，同 run 事后制动量不能被反用为本 run 的独立 deadline。因此：

- observed contract verdict：`NOT_TESTABLE`；
- guarantee verdict：`NOT_ESTABLISHED`；
- conditional guarantee-loss point：不可用；
- observed miss time：不可用；
- primary deadline debt：不可用。

300 ms 干预触发时刻是**故障进入时刻**，不是时间保证丧失时刻；799.636 ms 是**观测物理反应端点**，不是未定义 deadline 的 miss 时刻。

## Space budget / 空间预算、物理传播与安全损失

![速度与物理事件](../figures/speed_and_events.png)

| 量 | data/observed | 含义与边界 |
|---|---:|---|
| $D_1$ Fusion/几何净距 | {float(observed['D1_clear_data_observed_m']):.3f} m | Apollo Fusion 目标与校准 offset；对方 CARLA history 未覆盖 $t_1$ |
| $D_{{response}}$ | **{d_response:.3f} m** | $\\int_{{t_1}}^{{t_2}}v(t)dt_{{wall}}$，主口径 |
| $D_{{response}}$ 采样下夹取 | {d_response_low:.3f} m | 积分到 $t_2$ 前一 Localization 样本 |
| $D_{{brake,truncated}}$ | {d_brake_truncated:.3f} m | $t_2$→collision 墙钟速度积分；非完整制动距离 |
| 冲击速度 | **{float(raw['impact_speed_mps']):.3f} m/s** | CARLA collision event 直接观测 |
| 冲量模 | {float(raw['collision_impulse_norm']):.1f} | CARLA collision event 直接观测 |
| 主 $D_{{debt}}$ | 不可用 | 无 qualified $\\tau_{{req}}$ |
| 完整 $D_{{brake}}$ / $M_0$ | 不可用 | 碰撞右截尾，不以模型值填充 |
| timing 因果物理损失 | 不可定量 | 无 qualified deadline 和事前锁定的现实反事实轨迹 |

几何不确定性不得隐藏：Fusion/计划目标与 CARLA actor 155 的 20 帧匹配中，位置误差中位数 **{float(identity['position_error_median_m']):.3f} m**、P90 **{float(identity['position_error_p90_m']):.3f} m**；净距 offset 另有 0.52 m 不确定度。碰撞事件本身作为实际接触的主证据，不由这些距离推导替代。

## 核心问题的最终回答

| 问题 | 1131 run 可支持的回答 |
|---|---|
| 哪里出了实时性问题？ | 观测时间消耗主要在 source→Fusion（292.885 ms）和 Control→物理 $t_2$（486.037 ms）；Bridge 300 ms 注入是后者的主要候选构成，但事件命令未逐条归档。 |
| 它是什么性质？ | 已证明的是外部注入型 Bridge 固定时延 + 事件级时间/新鲜度退化；不是 Apollo 内生实时缺陷的单-run证明。Planning fallback 使它同时是功能/时间多因素候选。 |
| 什么时候失去时间保证？ | 不可判定。没有已建立的合格 deadline/WCRT/suffix bound，因而没有合法的 guarantee-loss 时刻。 |
| 为什么？ | 候选机制依次是持续 300 ms Bridge 延时、Perception 入口/等待/Detection 时间、Planning fallback、Control-to-physical 未分解残差、$t_2$ 后新鲜度空档。仅最后一项已被反证为“首次 $t_2$ 延迟”的原因；其他仍未唯一隔离。 |
| 造成多少物理安全损失？ | 可直接报告 13.432 m 响应距离、27.148 m 碰撞截尾制动距离、7.988 m/s 冲击速度和 16817.2 冲量模。但不能把 13.432 m 写成 deadline debt 或 timing 因果损失；该因果份额在本 run 不可定量。 |

## 验证、方法完备性与复现

- 原始输入库存：`validation/input_inventory.json`，{inventory['file_count']} 个文件、总计 {inventory['total_bytes']} bytes，保存 SHA-256；原始目录未写入。
- 标准墙钟响应距离由 `velocity_trajectory_observed.csv` 重算，只使用 Localization 速度与墙钟端点。
- data/observed 与 model/predicted 分表；模型表明确写 `NOT_COMPUTED`，没有以模型补观测缺失。
- 方法完备性：见 `tables/method_completeness_matrix.csv`。
- Claim–Evidence–Defeater 帐本：见 `claim_ledger.csv`、`evidence_ledger.csv`、`defeater_ledger.csv`。
- 反向诊断候选及区分检验：见 `diagnosis_hypothesis_ledger.csv`。
- 自动验证结果：见 `validation/validation.json`和 `validation/claim_audit.md`。

复现命令：

```bash
python3 {OUTPUT / 'scripts/analyze_1131_single_run.py'}
python3 {SKILL_SCRIPTS / 'recompute_l5_metrics.py'} --analysis-dir {OUTPUT}
python3 {SKILL_SCRIPTS / 'validate_analysis_outputs.py'} --analysis-dir {OUTPUT}
```
"""
    (REPORT / "six_layer_analysis_report.md").write_text(report_text, encoding="utf-8")
    (OUTPUT / "README.md").write_text(
        "# Run 202607271131 TCPS-PA v3.1 analysis\n\n"
        "Main report: [six_layer_analysis_report.md](report/six_layer_analysis_report.md)\n\n"
        "This directory is generated from the selected run's raw data only.\n",
        encoding="utf-8",
    )

    audit_text = f"""# Data quality audit

## Scope and immutability

- Scope: only `{RUN_DIR}`.
- Raw-file count: {inventory['file_count']}; SHA-256 recorded for every file.
- No file was written beneath the raw run directory.

## Clocks and endpoints

- Clock alignment: `{parsed.clock['status']}`; p95 absolute residual {parsed.clock['p95_abs_residual_ms']:.6f} ms; median {parsed.clock['median_abs_residual_ms']:.6f} ms.
- t1: source timestamp of the first frame in the first qualifying 3-frame stable target-11 sequence.
- t2: first raw-Localization sample satisfying the declared sustained-deceleration detector.
- t2 sensitivity: raw thresholds 0.3/0.5/1.0 m/s² agree; median-3 shifts the endpoint by one 100 ms sample.
- Collision endpoint: direct CARLA collision event, never actor-history first row.

## Distance and physical data

- Canonical D_response: wall-clock speed trapezoid, {d_response:.12f} m.
- Collision truncates full stopping; full D_brake and M0 remain blank.
- Fusion clearance and CARLA collision truth use different measurement sources; identity/geometry uncertainty is preserved.

## Missing data and claim ceilings

- No parsed record or Control payload archive.
- Event-local Bridge apply rows are suppressed after first APPLIED row.
- No qualified prospective physical deadline or WCRT/suffix bound.
- No cross-run measurements, calibrations, or counterfactuals were imported.
"""
    (VALIDATION / "data_quality_audit.md").write_text(audit_text, encoding="utf-8")

    summary = {
        "run_id": RUN_ID,
        "t1_wall_s": t1,
        "t2_wall_s": t2,
        "collision_wall_s": t_collision,
        "T_R_ms": tr_high_ms,
        "D_response_m": d_response,
        "impact_speed_mps": raw["impact_speed_mps"],
        "primary_deadline_status": "NOT_QUALIFIED_PRIMARY",
        "observed_contract_status": "NOT_TESTABLE",
        "guarantee_status": "NOT_ESTABLISHED",
        "attribution_status": "UNCERTAIN",
    }
    (VALIDATION / "analysis_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


if __name__ == "__main__":
    print(json.dumps(build(), ensure_ascii=False, indent=2))
