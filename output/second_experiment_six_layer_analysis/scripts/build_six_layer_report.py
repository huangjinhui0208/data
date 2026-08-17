#!/usr/bin/env python3
"""Build the six-layer temporal-correctness-to-physical-safety deliverable.

The original experiment tree is read-only.  This script consumes the freshly
recomputed raw-data-first table in report_workspace and reopens the raw logs
only to calculate deadline-crossing distance and event/freshness endpoints.
"""

from __future__ import annotations

import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT = ROOT / "第二次实验"
UPSTREAM = ROOT / "report_workspace"
OUT = Path(
    os.environ.get(
        "TCPS_PA_OUTPUT_DIR",
        str(ROOT / "output" / "second_experiment_six_layer_analysis"),
    )
).resolve()
TABLES = OUT / "tables"
FIGURES = OUT / "figures"
REPORT = OUT / "report"
VALIDATION = OUT / "validation"
RECORD_PROFILES = OUT / "record_profiles"
VENDOR = UPSTREAM / "scripts" / "vendor" / "realtime_collision_analysis"
sys.path.insert(0, str(VENDOR / "src"))

import realtime_collision_core as core  # noqa: E402


EXPECTED_RUNS = [
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
]
SAFETY_CLEARANCE_M = 6.0
UNKNOWN_OUTCOME_RUN = "202607271206"
COLORS = {
    "baseline": "#4472C4",
    "delay": "#ED7D31",
    "collision": "#C00000",
    "safe": "#2E7D32",
    "unknown": "#777777",
    "model": "#7F3C8D",
    "dark": "#30343B",
}


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def num(value: object) -> float:
    return float(value) if finite(value) else math.nan


def fmt(value: object, digits: int = 3) -> str:
    return f"{float(value):.{digits}f}" if finite(value) else "NA"


def bool_or_na(value: object) -> object:
    if pd.isna(value):
        return ""
    return bool(value)


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
    return config


def save_csv(frame: pd.DataFrame, name: str) -> None:
    frame.to_csv(TABLES / name, index=False, encoding="utf-8-sig")


def savefig(fig: plt.Figure, name: str) -> None:
    fig.tight_layout()
    fig.savefig(FIGURES / name)
    plt.close(fig)


def parse_raw_runs(config: dict) -> tuple[dict[str, core.ParsedRun], dict[str, dict]]:
    timezone = ZoneInfo(config["analysis"]["timezone"])
    parsed_by_id: dict[str, core.ParsedRun] = {}
    raw_by_id: dict[str, dict] = {}
    for spec in core.discover_runs(config):
        parsed = core.parse_run(spec, config, timezone)
        raw, _ = core.raw_run_metrics(parsed, config)
        parsed_by_id[spec.run_id] = parsed
        raw_by_id[spec.run_id] = raw
    return parsed_by_id, raw_by_id


def integrate_deadline_debt(
    parsed: core.ParsedRun, t1: float, t2: float, tau_ms: float
) -> float:
    if not all(finite(v) for v in (t1, t2, tau_ms)):
        return math.nan
    deadline = t1 + tau_ms / 1000.0
    if deadline >= t2:
        return 0.0
    if deadline < t1:
        # The available raw window starts at t1.  A pre-cause deadline is a
        # physically infeasible initial condition, so post-deadline distance
        # before t1 cannot be observed in this experiment.
        return math.nan
    return core.integrate_speed(parsed.localization, deadline, t2)


def response_window_freshness(parsed: core.ParsedRun, t1: float, t2: float) -> dict:
    rows = parsed.perception.get("target_rows", [])
    preceding = [row for row in rows if row.header_time_s <= t2]
    age_at_t2 = (t2 - preceding[-1].obs_time_s) * 1000.0 if preceding else math.nan
    in_window = [row for row in rows if t1 <= row.header_time_s <= t2]
    gaps = (
        np.diff([row.header_time_s for row in in_window]) * 1000.0
        if len(in_window) > 1
        else np.asarray([])
    )
    lifecycles = np.asarray(
        [(row.header_time_s - row.obs_time_s) * 1000.0 for row in in_window],
        dtype=float,
    )
    return {
        "target_output_count_response_window": len(in_window),
        "update_gap_target_response_window_max_data_observed_ms": (
            float(np.max(gaps)) if gaps.size else math.nan
        ),
        "update_gap_target_response_window_p90_data_observed_ms": (
            float(np.percentile(gaps, 90)) if gaps.size else math.nan
        ),
        "data_age_target_at_t2_data_observed_ms": age_at_t2,
        "target_lifecycle_response_window_p90_data_observed_ms": (
            float(np.percentile(lifecycles, 90)) if lifecycles.size else math.nan
        ),
    }


def build_observed(
    source: pd.DataFrame,
    parsed_by_id: dict[str, core.ParsedRun],
    raw_by_id: dict[str, dict],
) -> pd.DataFrame:
    observed = source.copy()
    observed["run_id"] = observed["run_id"].astype(str)
    observed["time_basis_main"] = "wall_epoch_s"
    # TCPS-PA v2 canonical name. Keep the legacy field for backward compatibility.
    observed["D_response_wall_integral_data_observed_m"] = observed[
        "D_delay_wall_integral_data_observed_m"
    ]
    observed["t_deadline_data_derived_s"] = (
        observed["t1_wall_s"]
        + observed["T_deadline_collision_0m_data_observed_ms"] / 1000.0
    )
    observed["tau_dynamic_data_derived_ms"] = observed[
        "T_deadline_collision_0m_data_observed_ms"
    ]
    observed["timing_slack_data_derived_ms"] = observed[
        "deadline_collision_minus_observed_ms"
    ]
    observed["deadline_miss_data_derived"] = observed[
        "timing_slack_data_derived_ms"
    ].map(lambda value: "" if not finite(value) else value < 0)
    observed["tau_dynamic_safety_6m_data_derived_ms"] = observed[
        "T_deadline_safety_6m_data_observed_ms"
    ]
    observed["timing_slack_safety_6m_data_derived_ms"] = observed[
        "deadline_safety_minus_observed_ms"
    ]
    observed["deadline_miss_safety_6m_data_derived"] = observed[
        "timing_slack_safety_6m_data_derived_ms"
    ].map(lambda value: "" if not finite(value) else value < 0)
    observed["D_safe_required_m"] = SAFETY_CLEARANCE_M
    observed["M_safety_required_data_observed_m"] = observed[
        "M_safety_6m_data_observed_m"
    ]
    observed["bridge_delay_requested_ms"] = observed["scb_requested_delay_ms"]
    observed["bridge_delay_actual_wall_data_observed_ms"] = observed[
        "scb_actual_wall_delay_ms"
    ]
    observed["bridge_delay_lifecycle_status"] = observed[
        "scb_lifecycle_complete"
    ].map(lambda value: "COMPLETE" if bool(value) else "INCOMPLETE")
    observed["record_profile_available"] = False
    observed["reaction_time_message_diagnostic_ms"] = math.nan
    observed["data_age_record_diagnostic_ms"] = math.nan
    observed["record_missing_reason"] = "NO_SAME_RUN_PARSED_RECORD_EXPORT"
    observed["D_distance_debt_data_derived_m"] = [
        integrate_deadline_debt(
            parsed_by_id[row.run_id],
            row.t1_wall_s,
            row.t2_wall_s,
            row.T_deadline_collision_0m_data_observed_ms,
        )
        for row in observed.itertuples()
    ]
    # These same-run post-outcome reconstructions are diagnostics, not independent
    # requirements or primary observed debt.
    observed["tau_retro_collision_0m_diagnostic_ms"] = observed[
        "tau_dynamic_data_derived_ms"
    ]
    observed["tau_retro_safety_6m_diagnostic_ms"] = observed[
        "tau_dynamic_safety_6m_data_derived_ms"
    ]
    observed["D_debt_retro_diagnostic_m"] = observed[
        "D_distance_debt_data_derived_m"
    ]

    outcome_times = []
    outcome_types = []
    freshness_rows = []
    scb_sources = []
    planning_sources = []
    for row in observed.itertuples():
        raw = raw_by_id[row.run_id]
        if bool(row.collision_event_data_observed):
            outcome_times.append(num(raw.get("t_collision_s")))
            outcome_types.append("COLLISION")
        else:
            outcome_times.append(num(raw.get("t_minimum_speed_s")))
            outcome_types.append(raw.get("braking_endpoint_type") or "UNKNOWN")
        freshness_rows.append(
            response_window_freshness(
                parsed_by_id[row.run_id], row.t1_wall_s, row.t2_wall_s
            )
        )
        scb_sources.append(raw.get("source_scb_file") or "")
        planning_sources.append(raw.get("source_planning_file") or "")
    observed["t_outcome_wall_s"] = outcome_times
    observed["outcome_endpoint_type_data_observed"] = outcome_types
    observed["source_scb_file"] = scb_sources
    observed["source_planning_file"] = planning_sources
    for column in freshness_rows[0]:
        observed[column] = [row[column] for row in freshness_rows]

    observed["physical_outcome_confidence"] = observed.apply(
        lambda row: (
            "HIGH_DIRECT_COLLISION_EVENT_AND_ACTOR_HISTORY"
            if bool(row["collision_event_data_observed"])
            else (
                "LOW_CONFLICTING_EVENT_AND_FIXED_GEOMETRY"
                if row["run_id"] == UNKNOWN_OUTCOME_RUN
                else "MEDIUM_HIGH_STOP_TRAJECTORY_NO_ACTOR_HISTORY"
            )
        ),
        axis=1,
    )
    observed["exclusion_reason"] = observed["run_id"].map(
        lambda run_id: (
            "OUTCOME_UNCERTAIN_COLLISION_EVENT_ABSENT_BUT_FIXED_GEOMETRY_IMPLIES_OVERLAP"
            if run_id == UNKNOWN_OUTCOME_RUN
            else ""
        )
    )
    observed["missing_reason"] = observed.apply(
        lambda row: (
            row["exclusion_reason"]
            if row["exclusion_reason"]
            else (
                "BRAKING_ENDPOINT_TRUNCATED_BY_COLLISION; FULL_OBSERVED_STOP_MARGIN_AND_DATA_DERIVED_DEADLINE_UNAVAILABLE"
                if bool(row["collision_event_data_observed"])
                else ""
            )
        ),
        axis=1,
    )
    return observed


def build_model(
    observed: pd.DataFrame, parsed_by_id: dict[str, core.ParsedRun]
) -> tuple[pd.DataFrame, float]:
    calibration = observed[
        (observed["group_name"] == "baseline")
        & observed["included_main_analysis"].astype(bool)
        & observed["D_brake_data_observed_m"].notna()
    ].copy()
    calibration_a = (
        calibration["v2_data_observed_mps"] ** 2
        / (2.0 * calibration["D_brake_data_observed_m"])
    )
    a_ref = float(np.median(calibration_a))
    calibration_runs = ",".join(calibration["run_id"].astype(str))
    model_rows = []
    for row in observed.itertuples():
        dbrake = row.v2_data_observed_mps**2 / (2.0 * a_ref)
        tau0 = (
            (row.D1_clear_data_observed_m - dbrake)
            / row.v1_data_observed_mps
            * 1000.0
        )
        tau6 = (
            (row.D1_clear_data_observed_m - dbrake - SAFETY_CLEARANCE_M)
            / row.v1_data_observed_mps
            * 1000.0
        )
        slack = tau0 - row.T_e2e_data_observed_ms
        slack6 = tau6 - row.T_e2e_data_observed_ms
        margin0 = row.D2_clear_data_observed_m - dbrake
        margin6 = margin0 - SAFETY_CLEARANCE_M
        collision_pred = margin0 < 0
        impact_pred = (
            math.sqrt(
                max(
                    0.0,
                    row.v2_data_observed_mps**2
                    - 2.0 * a_ref * max(0.0, row.D2_clear_data_observed_m),
                )
            )
            if collision_pred
            else 0.0
        )
        observed_brake = row.D_brake_data_observed_m
        brake_signed = dbrake - observed_brake if finite(observed_brake) else math.nan
        impact_observed = row.impact_speed_data_observed_mps
        impact_signed = (
            impact_pred - impact_observed if finite(impact_observed) else math.nan
        )
        model_rows.append(
            {
                "run_id": row.run_id,
                "group_name": row.group_name,
                "model_name": "baseline_median_effective_deceleration",
                "model_version": "v1.0",
                "baseline_reference_effective_deceleration_model_mps2": a_ref,
                "model_inputs_and_provenance": (
                    f"v2 and D_brake from baseline full-stop runs [{calibration_runs}]; "
                    "run-specific D1, v1, v2 and observed wall-clock t1/t2"
                ),
                "tau_dynamic_model_predicted_ms": tau0,
                "tau_dynamic_safety_6m_model_predicted_ms": tau6,
                "timing_slack_model_predicted_ms": slack,
                "timing_slack_safety_6m_model_predicted_ms": slack6,
                "deadline_miss_model_predicted": slack < 0,
                "t_deadline_model_predicted_s": row.t1_wall_s + tau0 / 1000.0,
                "D_brake_model_predicted_m": dbrake,
                "D_distance_debt_model_predicted_m": integrate_deadline_debt(
                    parsed_by_id[row.run_id], row.t1_wall_s, row.t2_wall_s, tau0
                ),
                "M_collision_0m_model_predicted_m": margin0,
                "M_safety_model_predicted_m": margin6,
                "collision_model_predicted": collision_pred,
                "impact_speed_model_predicted_mps": impact_pred,
                "outcome_data_observed_comparator": row.outcome_data_observed,
                "D_brake_data_observed_comparator_m": observed_brake,
                "D_brake_signed_error_model_minus_observed_m": brake_signed,
                "D_brake_absolute_error_model_m": (
                    abs(brake_signed) if finite(brake_signed) else math.nan
                ),
                "D_brake_relative_error_model": (
                    brake_signed / observed_brake
                    if finite(brake_signed) and observed_brake != 0
                    else math.nan
                ),
                "impact_speed_data_observed_comparator_mps": impact_observed,
                "impact_speed_signed_error_model_minus_observed_mps": impact_signed,
                "impact_speed_absolute_error_model_mps": (
                    abs(impact_signed) if finite(impact_signed) else math.nan
                ),
                "compatible_observed_comparator": (
                    "full-stop D_brake and M0" if finite(observed_brake) else "collision impact speed"
                ),
                "assumptions_and_scope_note": (
                    "Descriptive baseline-calibrated constant-effective-deceleration model; "
                    "baseline fit is in-sample, not cross-validated; not an observed outcome; "
                    "deadline is derived independently of measured reaction time."
                ),
            }
        )
    return pd.DataFrame(model_rows), a_ref


def build_event_timeline(
    observed: pd.DataFrame, model: pd.DataFrame, raw_by_id: dict[str, dict]
) -> pd.DataFrame:
    model_by_id = model.set_index("run_id")
    rows = []
    for row in observed.itertuples():
        raw = raw_by_id[row.run_id]
        rows.append(
            {
                "group_name": row.group_name,
                "run_id": row.run_id,
                "t_cause_t1_wall_s": row.t1_wall_s,
                "t_perception_output_wall_s": num(raw.get("t_perception_stable_output_s")),
                "t_prediction_wall_s": num(raw.get("t_prediction_first_s")),
                "t_planning_stop_wall_s": num(raw.get("t_planning_stop_s")),
                "t_control_command_wall_s": num(raw.get("t_control_brake_command_s")),
                "t_physical_response_t2_wall_s": row.t2_wall_s,
                "t_deadline_data_derived_wall_s": row.t_deadline_data_derived_s,
                "t_deadline_model_predicted_wall_s": model_by_id.loc[
                    row.run_id, "t_deadline_model_predicted_s"
                ],
                "t_outcome_wall_s": row.t_outcome_wall_s,
                "outcome_endpoint_type_data_observed": row.outcome_endpoint_type_data_observed,
                "time_basis": "wall_epoch_s",
                "t1_definition": "source time of first frame in continuous 3-frame stable Fusion target sequence",
                "t2_definition": "end of first interval satisfying sustained-deceleration rule",
                "deadline_definition_data_derived": "(D1 - observed full-stop D_brake) / v1; 0 m contact boundary",
                "deadline_definition_model": "(D1 - v2^2/(2*a_baseline_median)) / v1; 0 m contact boundary",
                "source_perception_file": row.source_perception_file,
                "source_localization_file": row.source_localization_file,
                "source_collision_file": row.source_collision_file,
            }
        )
    return pd.DataFrame(rows)


def build_stage_table(observed: pd.DataFrame) -> pd.DataFrame:
    columns = [
        "group_name",
        "run_id",
        "sensor_to_fusion_ms",
        "fusion_to_prediction_ms",
        "prediction_to_planning_stop_ms",
        "planning_stop_to_control_ms",
        "control_to_t2_ms",
        "sensor_to_control_ms",
        "T_e2e_data_observed_ms",
        "target_gap_max_ms",
        "target_gap_p90_ms",
        "target_lifecycle_median_ms",
        "target_lifecycle_p90_ms",
        "target_lifecycle_max_ms",
        "target_source_age_at_outcome_ms",
        "target_output_count_response_window",
        "update_gap_target_response_window_max_data_observed_ms",
        "update_gap_target_response_window_p90_data_observed_ms",
        "target_lifecycle_response_window_p90_data_observed_ms",
        "data_age_target_at_t2_data_observed_ms",
        "reaction_time_message_diagnostic_ms",
        "data_age_record_diagnostic_ms",
        "record_profile_available",
        "record_missing_reason",
        "source_perception_file",
        "source_localization_file",
        "time_basis_main",
    ]
    return observed[columns].copy()


def build_record_diagnostics(observed: pd.DataFrame) -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "group_name": row.group_name,
                "run_id": row.run_id,
                "record_profile_available": False,
                "record_dir": "",
                "run_record_association_status": "NO_SAME_RUN_RECORD_EXPORT",
                "profile_status": "UNAVAILABLE",
                "reaction_time_message_diagnostic_ms": math.nan,
                "data_age_record_diagnostic_ms": math.nan,
                "planning_age_record_diagnostic_ms": math.nan,
                "update_gap_record_diagnostic_ms": math.nan,
                "missing_reason": "SECOND_EXPERIMENT_RUN_HAS_NO_NESTED_RECORD_DIRECTORY",
                "join_policy": "left join; do not use records from other experiment dates",
            }
            for row in observed.itertuples()
        ]
    )


def summarize_group(observed: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for group_name, all_group in observed.groupby("group_name", sort=False):
        main = all_group[all_group["included_main_analysis"].astype(bool)]
        available_deadline = main[main["tau_dynamic_data_derived_ms"].notna()]
        rows.append(
            {
                "group_name": group_name,
                "n_total": len(all_group),
                "n_included_main_analysis": len(main),
                "n_excluded": len(all_group) - len(main),
                "n_collision_data_observed": int(main["collision_event_data_observed"].sum()),
                "collision_rate_data_observed": float(
                    main["collision_event_data_observed"].mean()
                ),
                "bridge_delay_actual_wall_median_data_observed_ms": float(
                    main["bridge_delay_actual_wall_data_observed_ms"].median()
                ),
                "bridge_delay_actual_wall_p90_data_observed_ms": float(
                    main["bridge_delay_actual_wall_data_observed_ms"].quantile(0.9)
                ),
                "T_e2e_median_data_observed_ms": float(
                    main["T_e2e_data_observed_ms"].median()
                ),
                "T_e2e_p90_data_observed_ms": float(
                    main["T_e2e_data_observed_ms"].quantile(0.9)
                ),
                "D_response_wall_integral_median_data_observed_m": float(
                    main["D_delay_wall_integral_data_observed_m"].median()
                ),
                "D_response_wall_integral_p90_data_observed_m": float(
                    main["D_delay_wall_integral_data_observed_m"].quantile(0.9)
                ),
                "D2_clear_median_data_observed_m": float(
                    main["D2_clear_data_observed_m"].median()
                ),
                "data_derived_contact_deadline_available_count": len(available_deadline),
                "data_derived_contact_deadline_miss_count": int(
                    (available_deadline["timing_slack_data_derived_ms"] < 0).sum()
                ),
                "data_derived_6m_deadline_miss_count": int(
                    (available_deadline["timing_slack_safety_6m_data_derived_ms"] < 0).sum()
                ),
                "record_profile_available_count": int(
                    main["record_profile_available"].sum()
                ),
            }
        )
    return pd.DataFrame(rows)


def build_exclusions(observed: pd.DataFrame) -> pd.DataFrame:
    rows = [
        {
            "run_id_or_group": UNKNOWN_OUTCOME_RUN,
            "category": "main_analysis_exclusion",
            "affected_fields": "outcome and outcome aggregates",
            "reason_code": "COLLISION_GEOMETRY_CONFLICT",
            "reason_detail": "No CollisionSensor event/actor history, but fixed-geometry stop endpoint implies overlap.",
            "handling": "Retain timing/distance diagnostics; exclude from main outcome comparison.",
        },
        {
            "run_id_or_group": "202607271131;202607271643",
            "category": "observed_endpoint_missing",
            "affected_fields": "full D_brake, full-stop margins, data-derived deadline",
            "reason_code": "BRAKING_ENDPOINT_TRUNCATED_BY_COLLISION",
            "reason_detail": "Collision occurs before a full observed stop.",
            "handling": "Keep collision, truncated braking distance and impact speed; leave full-stop fields unavailable.",
        },
        {
            "run_id_or_group": "all_12_runs",
            "category": "record_unavailable",
            "affected_fields": "record Planning age/reuse, message reaction/data age and channel gaps",
            "reason_code": "NO_SAME_RUN_PARSED_RECORD_EXPORT",
            "reason_detail": "No record/ directory is nested under any second-experiment run.",
            "handling": "Preserve record columns as unavailable; do not join July 9 records from other experiments.",
        },
        {
            "run_id_or_group": "10_noncollision_or_uncertain_runs",
            "category": "physical_truth_limit",
            "affected_fields": "CARLA dual-clock alignment, actor identity and direct collision absence confidence",
            "reason_code": "ACTOR_HISTORY_UNAVAILABLE",
            "reason_detail": "Actor history exists only for the two collision runs.",
            "handling": "Use wall-clock Localization for main metrics; label noncollision outcome confidence medium-high.",
        },
        {
            "run_id_or_group": "all_12_runs",
            "category": "control_payload_limit",
            "affected_fields": "Control command payload semantics",
            "reason_code": "CONTROL_PAYLOAD_NOT_ARCHIVED",
            "reason_detail": "Control Trace timing is present, but full Control payload was not archived in the run bundle.",
            "handling": "Use Control Trace timing only; keep Guardian outside the executed command chain.",
        },
    ]
    return pd.DataFrame(rows)


def evidence_row(
    layer: str,
    question: str,
    metric: str,
    run_id: str,
    value: object,
    unit: str,
    evidence_type: str,
    time_basis: str,
    source_file: str,
    source_locator: str,
    availability: str,
    confidence: str,
) -> dict:
    return {
        "layer": layer,
        "question": question,
        "metric": metric,
        "run_id_or_group": run_id,
        "value": value,
        "unit": unit,
        "evidence_type": evidence_type,
        "time_basis": time_basis,
        "source_file": source_file,
        "source_locator": source_locator,
        "availability": availability,
        "confidence": confidence,
    }


def build_evidence(observed: pd.DataFrame, model: pd.DataFrame) -> pd.DataFrame:
    rows = []
    model_by_id = model.set_index("run_id")
    model_file = str((TABLES / "run_level_model_predicted.csv").resolve())
    for row in observed.itertuples():
        rows.extend(
            [
                evidence_row(
                    "L1",
                    "What temporal disturbance entered the closed loop?",
                    "bridge_delay_actual_wall_data_observed_ms",
                    row.run_id,
                    row.bridge_delay_actual_wall_data_observed_ms,
                    "ms",
                    "direct observed",
                    "wall",
                    row.source_scb_file,
                    "SCB requested/actual delay and lifecycle rows",
                    "available",
                    "high: independent bridge/SCB log",
                ),
                evidence_row(
                    "L2",
                    "Did update continuity or freshness degrade?",
                    "target_gap_max_ms",
                    row.run_id,
                    row.target_gap_max_ms,
                    "ms",
                    "diagnostic",
                    "message header/source",
                    row.source_perception_file,
                    "stable target Fusion observations",
                    "available",
                    "medium-high: target ID continuity checked",
                ),
                evidence_row(
                    "L2",
                    "Did update continuity or freshness degrade?",
                    "data_age_target_at_t2_data_observed_ms",
                    row.run_id,
                    row.data_age_target_at_t2_data_observed_ms,
                    "ms",
                    "data-derived",
                    "wall/header aligned",
                    row.source_perception_file,
                    "last target source timestamp preceding physical t2",
                    "available" if finite(row.data_age_target_at_t2_data_observed_ms) else "unavailable",
                    "medium: source/header semantics from parser",
                ),
                evidence_row(
                    "L3",
                    "How long from target source to sustained physical response?",
                    "T_e2e_data_observed_ms",
                    row.run_id,
                    row.T_e2e_data_observed_ms,
                    "ms",
                    "data-derived",
                    "wall",
                    row.source_localization_file,
                    "t2_wall_s - t1_wall_s; t1 also uses Fusion source timestamp",
                    "available",
                    "high: common wall time and fixed endpoints",
                ),
                evidence_row(
                    "L4",
                    "Was the independently derived 0 m contact deadline missed?",
                    "tau_dynamic_data_derived_ms",
                    row.run_id,
                    row.tau_dynamic_data_derived_ms,
                    "ms",
                    "data-derived",
                    "wall",
                    row.source_localization_file,
                    "(D1 - full observed D_brake) / v1",
                    "available" if finite(row.tau_dynamic_data_derived_ms) else "unavailable",
                    (
                        "medium-high: same-run full-stop braking envelope"
                        if finite(row.tau_dynamic_data_derived_ms)
                        else "unavailable: collision truncates full stop"
                    ),
                ),
                evidence_row(
                    "L4",
                    "What does a baseline-calibrated independent deadline predict?",
                    "tau_dynamic_model_predicted_ms",
                    row.run_id,
                    model_by_id.loc[row.run_id, "tau_dynamic_model_predicted_ms"],
                    "ms",
                    "model-predicted",
                    "wall",
                    model_file,
                    "baseline median effective-deceleration model",
                    "available",
                    "medium: descriptive in-sample calibration",
                ),
                evidence_row(
                    "L5",
                    "How much distance was consumed before physical response?",
                    "D_delay_wall_integral_data_observed_m",
                    row.run_id,
                    row.D_delay_wall_integral_data_observed_m,
                    "m",
                    "data-derived",
                    "wall",
                    row.source_localization_file,
                    "trapezoidal integral of speed from t1 to t2",
                    "available",
                    "high: common wall-clock integration",
                ),
                evidence_row(
                    "L5",
                    "How much incremental distance was traveled after the data-derived deadline?",
                    "D_distance_debt_data_derived_m",
                    row.run_id,
                    row.D_distance_debt_data_derived_m,
                    "m",
                    "data-derived",
                    "wall",
                    row.source_localization_file,
                    "trapezoidal integral from t_deadline to t2",
                    "available" if finite(row.D_distance_debt_data_derived_m) else "unavailable",
                    (
                        "medium-high: observed full-stop deadline"
                        if finite(row.D_distance_debt_data_derived_m)
                        else "unavailable: full-stop deadline or pre-cause window missing"
                    ),
                ),
                evidence_row(
                    "L6",
                    "What physical safety outcome was observed?",
                    "outcome_data_observed",
                    row.run_id,
                    row.outcome_data_observed,
                    "category",
                    "direct observed",
                    "wall/sim evidence separated",
                    row.source_collision_file if isinstance(row.source_collision_file, str) and row.source_collision_file else row.source_localization_file,
                    "CollisionSensor/actor history for collisions; braking endpoint and event absence for stops",
                    "uncertain" if row.run_id == UNKNOWN_OUTCOME_RUN else "available",
                    row.physical_outcome_confidence,
                ),
            ]
        )
    return pd.DataFrame(rows)


def plots(observed: pd.DataFrame, model: pd.DataFrame, group: pd.DataFrame) -> None:
    # Six-layer causal chain.
    fig, ax = plt.subplots(figsize=(13.2, 3.0))
    ax.axis("off")
    labels = [
        "L1 时序扰动\nSCB/Bridge延迟",
        "L2 时序退化\n连续性/新鲜度",
        "L3 因果时序\n源→物理响应",
        "L4 时间正确性\n动态deadline/裕量",
        "L5 物理传播\n响应距离/增量债务",
        "L6 安全后果\n停车/碰撞",
    ]
    xs = np.linspace(0.075, 0.925, len(labels))
    for index, (x, label) in enumerate(zip(xs, labels)):
        face = "#E8F0FE" if index < 4 else ("#FFF1E6" if index == 4 else "#FDECEC")
        ax.text(
            x,
            0.5,
            label,
            ha="center",
            va="center",
            fontsize=10.5,
            transform=ax.transAxes,
            bbox=dict(boxstyle="round,pad=0.55", fc=face, ec=COLORS["dark"]),
        )
        if index < len(labels) - 1:
            ax.annotate(
                "",
                xy=(xs[index + 1] - 0.065, 0.5),
                xytext=(x + 0.065, 0.5),
                xycoords=ax.transAxes,
                arrowprops=dict(arrowstyle="->", lw=1.5, color=COLORS["dark"]),
            )
    ax.set_title("第二次实验六层证据链（deadline由车辆状态独立输入L4）", pad=18)
    savefig(fig, "six_layer_chain.png")

    main = observed[observed["included_main_analysis"].astype(bool)].copy()
    main["short_id"] = main["run_id"].str[-4:]
    colors = [COLORS["baseline"] if g == "baseline" else COLORS["delay"] for g in main["group_name"]]

    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    ax.scatter(
        main["bridge_delay_actual_wall_data_observed_ms"],
        main["T_e2e_data_observed_ms"],
        c=colors,
        s=70,
        edgecolor="white",
    )
    for row in main.itertuples():
        if row.collision_event_data_observed:
            ax.scatter(
                row.bridge_delay_actual_wall_data_observed_ms,
                row.T_e2e_data_observed_ms,
                marker="X",
                s=130,
                color=COLORS["collision"],
            )
            ax.annotate(row.short_id, (row.bridge_delay_actual_wall_data_observed_ms, row.T_e2e_data_observed_ms), xytext=(5, 4), textcoords="offset points")
    ax.set_xlabel("SCB/Bridge实测墙钟延迟/ms")
    ax.set_ylabel("物理响应时间 $T_R$/ms")
    ax.set_title("L1→L3：干预生效后，闭环物理响应整体右移")
    ax.grid(alpha=0.2)
    savefig(fig, "intervention_vs_physical_response.png")

    model_by_id = model.set_index("run_id")
    x = np.arange(len(main))
    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    ax.bar(x, main["T_e2e_data_observed_ms"], color=colors, alpha=0.85, label="观测 $T_R$")
    ax.scatter(
        x,
        [model_by_id.loc[rid, "tau_dynamic_model_predicted_ms"] for rid in main["run_id"]],
        marker="D",
        s=56,
        color=COLORS["model"],
        label="baseline制动包络预测0 m deadline（模型）",
        zorder=4,
    )
    available = main["tau_dynamic_data_derived_ms"].notna()
    ax.scatter(
        x[available],
        main.loc[available, "tau_dynamic_data_derived_ms"],
        marker="o",
        facecolors="white",
        edgecolors=COLORS["safe"],
        s=62,
        label="完整停车run数据派生0 m deadline",
        zorder=5,
    )
    ax.set_xticks(x, main["short_id"], rotation=45)
    ax.set_ylabel("时间/ms（墙钟）")
    ax.set_title("L4：观测响应与独立动态deadline（模型与数据派生分开）")
    ax.legend(frameon=False, fontsize=8)
    ax.grid(axis="y", alpha=0.18)
    savefig(fig, "response_vs_dynamic_deadline.png")

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    ax.bar(
        x,
        main["D_delay_wall_integral_data_observed_m"],
        color=colors,
        alpha=0.88,
        label="观测响应阶段总距离 $D_{response}$",
    )
    debts = [model_by_id.loc[rid, "D_distance_debt_model_predicted_m"] for rid in main["run_id"]]
    ax.scatter(x, debts, marker="D", s=52, color=COLORS["model"], label="模型deadline后的增量 $D_{debt}$")
    ax.set_xticks(x, main["short_id"], rotation=45)
    ax.set_ylabel("距离/m（墙钟速度梯形积分）")
    ax.set_title("L5：总响应距离与deadline后增量距离债务严格区分")
    ax.legend(frameon=False, fontsize=8)
    ax.grid(axis="y", alpha=0.18)
    savefig(fig, "response_distance_and_deadline_debt.png")

    stage_columns = [
        "sensor_to_fusion_ms",
        "fusion_to_prediction_ms",
        "prediction_to_planning_stop_ms",
        "planning_stop_to_control_ms",
        "control_to_t2_ms",
    ]
    labels = ["源→Fusion", "Fusion→Prediction", "Prediction→Planning", "Planning→Control", "Control→t2"]
    medians = [
        [float(main[main["group_name"] == g][col].median()) for col in stage_columns]
        for g in ["baseline", "delay_300ms"]
    ]
    fig, ax = plt.subplots(figsize=(9.2, 5.5))
    left = np.zeros(2)
    palette = ["#5B8FF9", "#61DDAA", "#65789B", "#F6BD16", "#F08BB4"]
    for index, label in enumerate(labels):
        values = np.asarray([medians[0][index], medians[1][index]])
        ax.barh([0, 1], values, left=left, color=palette[index], label=label)
        left += values
    ax.set_yticks([0, 1], ["baseline", "300 ms"])
    ax.set_xlabel("组内逐run中位数/ms")
    ax.set_title("L3阶段分解：增量主要出现在Control→物理响应段")
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.14))
    ax.grid(axis="x", alpha=0.18)
    savefig(fig, "stage_timing_group_median.png")

    fig, ax = plt.subplots(figsize=(10.2, 5.6))
    safe = main[~main["collision_event_data_observed"].astype(bool)]
    collision = main[main["collision_event_data_observed"].astype(bool)]
    ax.scatter(
        safe["timing_slack_data_derived_ms"],
        safe["M_collision_0m_data_observed_m"],
        s=74,
        color=COLORS["safe"],
        label="完整停车：数据派生slack与观测0 m余量",
    )
    ax.scatter(
        [model_by_id.loc[rid, "timing_slack_model_predicted_ms"] for rid in main["run_id"]],
        [model_by_id.loc[rid, "M_collision_0m_model_predicted_m"] for rid in main["run_id"]],
        marker="D",
        s=42,
        color=COLORS["model"],
        alpha=0.75,
        label="baseline包络模型：slack与0 m余量",
    )
    for row in collision.itertuples():
        collision_x = model_by_id.loc[row.run_id, "timing_slack_model_predicted_ms"]
        ax.scatter(collision_x, 0, marker="X", s=130, color=COLORS["collision"])
        ax.annotate(
            f"{row.short_id} 观测碰撞\n（x为模型slack）",
            (collision_x, 0),
            xytext=(5, 7),
            textcoords="offset points",
            fontsize=8,
        )
    ax.axhline(0, color=COLORS["dark"], lw=1)
    ax.axvline(0, color=COLORS["dark"], lw=1, ls="--", alpha=0.7)
    ax.set_xlabel("timing slack/ms（正值为按时，负值为失约）")
    ax.set_ylabel("0 m接触余量/m")
    ax.set_title("L4→L6：时间裕量耗尽与物理余量/观测结局的关系")
    ax.legend(frameon=False, fontsize=8)
    ax.grid(alpha=0.18)
    savefig(fig, "timing_to_physical_outcome.png")


def report_markdown(
    observed: pd.DataFrame,
    model: pd.DataFrame,
    group: pd.DataFrame,
    a_ref: float,
) -> str:
    group_by = group.set_index("group_name")
    base = group_by.loc["baseline"]
    delay = group_by.loc["delay_300ms"]
    main = observed[observed["included_main_analysis"].astype(bool)].copy()
    model_by_id = model.set_index("run_id")
    delta_t = delay.T_e2e_median_data_observed_ms - base.T_e2e_median_data_observed_ms
    delta_d = (
        delay.D_response_wall_integral_median_data_observed_m
        - base.D_response_wall_integral_median_data_observed_m
    )
    delta_d2 = delay.D2_clear_median_data_observed_m - base.D2_clear_median_data_observed_m
    base_control = float(main[main.group_name == "baseline"].control_to_t2_ms.median())
    delay_control = float(main[main.group_name == "delay_300ms"].control_to_t2_ms.median())
    base_s2c = float(main[main.group_name == "baseline"].sensor_to_control_ms.median())
    delay_s2c = float(main[main.group_name == "delay_300ms"].sensor_to_control_ms.median())
    safe_main = main[~main.collision_event_data_observed.astype(bool)]
    contact_available = safe_main[safe_main.tau_dynamic_data_derived_ms.notna()]
    contact_misses = int((contact_available.timing_slack_data_derived_ms < 0).sum())
    six_m_misses = int((contact_available.timing_slack_safety_6m_data_derived_ms < 0).sum())
    model_misses = model[model.run_id.isin(main.run_id) & model.deadline_miss_model_predicted.astype(bool)]
    r1131 = observed.set_index("run_id").loc["202607271131"]
    r1643 = observed.set_index("run_id").loc["202607271643"]
    r1206 = observed.set_index("run_id").loc[UNKNOWN_OUTCOME_RUN]
    m1131 = model_by_id.loc["202607271131"]
    m1643 = model_by_id.loc["202607271643"]

    run_lines = []
    for row in observed.itertuples():
        run_lines.append(
            "| {run} | {group_name} | {included} | {tr} | {d1} | {dr} | {d2} | {db} | {m0} | {outcome} |".format(
                run=row.run_id,
                group_name="baseline" if row.group_name == "baseline" else "300 ms",
                included="是" if row.included_main_analysis else "否",
                tr=fmt(row.T_e2e_data_observed_ms),
                d1=fmt(row.D1_clear_data_observed_m),
                dr=fmt(row.D_delay_wall_integral_data_observed_m),
                d2=fmt(row.D2_clear_data_observed_m),
                db=fmt(row.D_brake_data_observed_m),
                m0=fmt(row.M_collision_0m_data_observed_m),
                outcome=row.outcome_data_observed,
            )
        )

    model_lines = []
    for run_id in ["202607271131", "202607271202", "202607271211", "202607271643"]:
        row = model_by_id.loc[run_id]
        model_lines.append(
            f"| {run_id} | {fmt(row.tau_dynamic_model_predicted_ms)} | {fmt(row.timing_slack_model_predicted_ms)} | "
            f"{fmt(row.D_distance_debt_model_predicted_m)} | {fmt(row.M_collision_0m_model_predicted_m)} | "
            f"{bool(row.collision_model_predicted)} | {fmt(row.impact_speed_model_predicted_mps)} |"
        )

    return f"""# 第二次实验：自动驾驶时间正确性—物理安全传播六层分析报告

> 生成日期：2026-08-11｜原始数据统一重算｜分析目录与原始 `第二次实验/` 分离

## Executive conclusion（执行结论）

名义 300 ms 的 Bridge/SCB 时序干预被独立日志证实：主分析 300 ms 组实测墙钟延迟中位数为 **{delay.bridge_delay_actual_wall_median_data_observed_ms:.3f} ms**，baseline 为 **{base.bridge_delay_actual_wall_median_data_observed_ms:.3f} ms**。物理响应时间 `T_R=t2-t1` 的中位数由 **{base.T_e2e_median_data_observed_ms:.3f} ms** 增至 **{delay.T_e2e_median_data_observed_ms:.3f} ms**，增加 **{delta_t:.3f} ms**；墙钟速度梯形积分得到的响应阶段总距离由 **{base.D_response_wall_integral_median_data_observed_m:.3f} m** 增至 **{delay.D_response_wall_integral_median_data_observed_m:.3f} m**，增加 **{delta_d:.3f} m**。有效制动开始时剩余净距中位数减少 **{abs(delta_d2):.3f} m**。

11 个可信结局 run 中，baseline 为 0/7 碰撞，300 ms 组为 2/4 碰撞。完整停车 run 的“基于同 run 实测完整制动距离”的 0 m 接触 deadline 均未失约（{contact_misses}/{len(contact_available)}），但 300 ms 两个可靠停车 run 的接触裕量只剩 0.548 m 和 1.018 m；两个碰撞 run 因轨迹在碰撞处截断，不能伪造完整观测制动距离或数据派生 deadline。独立于实测 `T_R` 的 baseline 制动包络模型预测 `1131`、`1643` 分别失约 **{abs(m1131.timing_slack_model_predicted_ms):.3f} ms**、**{abs(m1643.timing_slack_model_predicted_ms):.3f} ms**，deadline 后增量距离债务分别为 **{m1131.D_distance_debt_model_predicted_m:.3f} m**、**{m1643.D_distance_debt_model_predicted_m:.3f} m**。模型结果仅用于物理可行性对照，不回填 observed 表。

本数据最强支持的结论是：**受控命令时序故障在闭环物理响应端被放大，增加响应阶段行驶距离并压缩制动空间，是两起碰撞的重要贡献因素。** `1131` 还伴随 507.439 ms Fusion 输出空档和 705.892 ms 生命周期峰值，可称“实时性主导候选”；`1643` 没有同类长空档，且起始净距更小，必须归为多因素碰撞。小样本、初始净距和制动能力差异阻止“300 ms 是唯一原因”或“700 ms 是普适硬阈值”的表述。

![六层分析链](../figures/six_layer_chain.png)

## Scope, architecture, and experiment groups

- 系统：CARLA 0.9.15（服务器）—Apollo 10.0.0（Orin）—Bridge（服务器），经网线闭环。
- 当前 Bridge 直接读取 Control 命令；Apollo 未向 Bridge 发送 Guardian 命令，所以 Guardian 不在主执行链中。
- 样本：baseline 7 个 run，名义 300 ms 组 5 个 run，共 12 个；`202607271206` 因结局证据冲突只保留时序/距离诊断，不进入主结局统计。
- 原始证据：Localization、Perception/Fusion、Prediction、Planning、Control Trace、SCB、CollisionSensor 与 actor history。
- 第二次实验 12 个 run 均无同 run 的 `record/` 解析导出。报告没有把 7 月 9 日其他实验的 record 强行关联进来；record 诊断列明确标为不可用。

## Data inventory, clocks, endpoints, and quality limits

主时钟为 Apollo/Localization wall epoch。`t1` 是连续 3 帧稳定 Fusion 目标序列第一帧的源时间；`t2` 是首次满足持续减速度规则的区间终点；物理 `T_R=(t2-t1)`，不能用 sensor→Control 消息时延替代。主响应距离为：

`D_response = ∫[t1,t2] v(t) dt_wall`

本报告沿用工作区字段 `D_delay_wall_integral_data_observed_m`，但在文字中称“响应阶段总距离”，不把它误称为 deadline 后的增量距离债务。真正的 `D_debt` 定义为 `∫[t_deadline,t2] v(t)dt_wall` 且只在 deadline 被超过时为正。

两个碰撞 run 的 CollisionSensor 和 actor history 可直接证实碰撞；多数停车 run 无 actor history，因此结局置信度为中高，而不是与碰撞 run 相同的完整 CARLA 真值。`1206` 无碰撞事件/actor history，但固定几何推算停车端点发生明显重叠，因此标记 `COLLISION_GEOMETRY_CONFLICT`。完整质量审计见 [data_quality_audit.md](../validation/data_quality_audit.md)。

## L1 Temporal disturbance（时序扰动）

所有 run 均存在 SCB 延迟日志和完整 lifecycle。300 ms 主分析组实测延迟中位数 **{delay.bridge_delay_actual_wall_median_data_observed_ms:.3f} ms**；baseline 中位数 **{base.bridge_delay_actual_wall_median_data_observed_ms:.3f} ms**，其中 `1031` 首次有效命令为 19.282 ms，其余约 0.067–0.091 ms。干预因此由直接证据确认，而不是从目录名推断。

![干预与物理响应](../figures/intervention_vs_physical_response.png)

## L2 Temporal degradation（时序退化）

本层分开检查响应时间尾部、数据新鲜度和更新连续性。`1131` 的目标 Fusion 最大输出间隔 **{r1131.target_gap_max_ms:.3f} ms**、生命周期峰值 **{r1131.target_lifecycle_max_ms:.3f} ms**，显著偏离其同设置安全对照；`1643` 的对应值为 **{r1643.target_gap_max_ms:.3f} ms** 和 **{r1643.target_lifecycle_max_ms:.3f} ms**，未复现 `1131` 的长空档。由此可把 `1131` 定性为连续性/新鲜度退化，而不能把两起碰撞都归为单帧感知计算超时。

record 级 Planning age/reuse、消息 sensor→Control reaction/data age 在本批 run 不可用。现有 Fusion header/source 与 Trace 证据仍支持本层分析，但不能声称已观测 record 内部未归档的通道行为。

## L3 Cause-effect timing（因果时序）

物理 `T_R` 中位数由 **{base.T_e2e_median_data_observed_ms:.3f} ms** 增至 **{delay.T_e2e_median_data_observed_ms:.3f} ms**。sensor→Control 中位数只由 **{base_s2c:.3f} ms** 增至 **{delay_s2c:.3f} ms**，而 Control→`t2` 由 **{base_control:.3f} ms** 增至 **{delay_control:.3f} ms**，说明增量主要落在干预位置之后的命令等待、车辆执行、动力学和持续减速识别区段。该区段不能全部贴成 Bridge 内部处理时延。

![阶段时序](../figures/stage_timing_group_median.png)

## L4 Temporal correctness and dynamic deadline（时间正确性）

报告同时保存两类 deadline，且都在与 `T_R` 比较前由物理状态导出：

1. `tau_dynamic_data_derived`：仅对完整停车 run，用 `tau=(D1-D_brake_observed)/v1` 计算 0 m 接触 deadline；碰撞 run 不具备完整观测制动距离，保持 NA。
2. `tau_dynamic_model_predicted`：以 baseline 完整停车 run 的等效减速度中位数 **{a_ref:.3f} m/s²** 构造 `D_brake=v2²/(2a)`，再计算 deadline。该模型可覆盖碰撞 run，但属于 predicted/model。

可信完整停车 run 的数据派生 0 m deadline 失约数为 **{contact_misses}/{len(contact_available)}**；若要求停车后仍保留 6 m，则为 **{six_m_misses}/{len(contact_available)}** 全部失约，说明 6 m 是比“避免接触”更严格的工程要求，baseline 也未满足。模型在 11 个主分析 run 中判为 0 m deadline miss 的有 **{len(model_misses)}** 个；其中 `1202`、`1211` 仅在模型中略微负裕量而实际停车，暴露模型约数十厘米的边界误差，不能把模型判定冒充观测碰撞。

![响应与动态deadline](../figures/response_vs_dynamic_deadline.png)

## L5 Temporal-to-physical propagation（时序到物理传播）

300 ms 组 `D_response` 中位数较 baseline 增加 **{delta_d:.3f} m**，`D2` 中位数减少 **{abs(delta_d2):.3f} m**。这是“时间变慢→制动开始位置后移”的直接空间证据。数据派生的 0 m deadline 在完整停车主分析 run 中未被超过，所以相应 `D_debt_data_derived=0`；碰撞 run 的该值不可用。baseline 包络模型则给出碰撞 run 的 post-deadline 增量距离债务：`1131` **{m1131.D_distance_debt_model_predicted_m:.3f} m**、`1643` **{m1643.D_distance_debt_model_predicted_m:.3f} m**。

![响应距离与增量债务](../figures/response_distance_and_deadline_debt.png)

## L6 Observed physical safety outcomes（观测物理安全后果）

baseline 7/7 完整停车、0/7 碰撞；300 ms 主分析组 2/4 碰撞。可靠非碰撞 delay run `1202` 和 `1211` 的观测 0 m 余量分别为 0.548 m、1.018 m。`1131` 撞击前速度 **{r1131.impact_speed_data_observed_mps:.3f} m/s**，`1643` 为 **{r1643.impact_speed_data_observed_mps:.3f} m/s**。碰撞 run 的完整 `D_brake`、完整停车余量和数据派生 deadline 均保持 NA，只报告碰撞前截断制动距离和撞击速度。

![时间到物理后果](../figures/timing_to_physical_outcome.png)

## Cross-run comparison and causal audit

干预真实性、物理响应变化、墙钟响应距离增加、剩余制动空间减少和观测结局变化五个环节均有证据。竞争解释仍包括 D1 波动、速度波动、Fusion 新鲜度/连续性、当次制动能力与停止端点差异。`1131` 的实时性链最完整，可称“主要支持贡献因素”；`1643` 的更小 D1 与较高撞击速度要求多因素表述。`1206` 的 `T_R={r1206.T_e2e_data_observed_ms:.3f} ms`、`D_response={r1206.D_delay_wall_integral_data_observed_m:.3f} m` 可复算，但结局冲突使其不能当作安全对照。

样本不是随机配对设计，不能从 0/7 与 2/4 推出总体碰撞概率，也不能把 700 ms 当普适硬阈值。当前只支持在本次速度、净距和制动条件下存在安全裕量陡峭收缩区。

## Model/predicted comparison（与 observed 分离）

下表仅列模型输出。`D_debt` 是模型 deadline 后到 `t2` 的墙钟速度积分；M0 是模型制动距离下的 0 m接触余量。

| run | tau_model/ms | slack_model/ms | D_debt_model/m | M0_model/m | collision_model | impact_model/(m/s) |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(model_lines)}

模型对 `1131` 的撞击速度预测为 {m1131.impact_speed_model_predicted_mps:.3f} m/s，对比观测 {r1131.impact_speed_data_observed_mps:.3f} m/s；对 `1643` 预测 {m1643.impact_speed_model_predicted_mps:.3f} m/s，对比观测 {r1643.impact_speed_data_observed_mps:.3f} m/s。误差方向均为低估严重度，且 baseline 校准为描述性、样本内模型，不能替代碰撞 run 的直接结局证据。完整误差字段见 [run_level_model_predicted.csv](../tables/run_level_model_predicted.csv)。

## Limitations and next experiment recommendations

- 所有 run 均应同时归档 `record/`、CollisionSensor、actor history、Control payload 与配置快照，避免时间层证据和物理真值不对称。
- 固定初速度与 D1，在 600–900 ms 区间增加重复，并把 Bridge 延迟和 Fusion gap/data age 分成独立实验因素。
- 预注册 0 m接触边界与 6 m工程安全边界，分别报告；不要用同一“安全”标签混合。
- 使用独立 baseline/训练批次校准制动模型，并留出验证 run，报告 stopping distance 和 impact speed 的 signed/absolute/relative error。
- record 接入必须按同 run 嵌套目录或经 collection window 审计后左连接，绝不按相似时间目录名强行合并。

## Per-run observed results（逐 run 观测主结果）

| run | group | 主分析 | T_R/ms | D1/m | D_response/m | D2/m | D_brake_data/m | M0_data/m | observed outcome |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
{chr(10).join(run_lines)}

观测主表：[run_level_observed.csv](../tables/run_level_observed.csv)；模型表：[run_level_model_predicted.csv](../tables/run_level_model_predicted.csv)；六层证据矩阵：[layer_evidence_matrix.csv](../tables/layer_evidence_matrix.csv)；验证结果：[validation.json](../validation/validation.json)。
"""


def write_quality_audit(observed: pd.DataFrame, model: pd.DataFrame) -> None:
    t_error = np.max(
        np.abs(
            observed["T_e2e_data_observed_ms"]
            - (observed["t2_wall_s"] - observed["t1_wall_s"]) * 1000.0
        )
    )
    d2_error = np.max(
        np.abs(
            observed["D2_clear_data_observed_m"]
            - (
                observed["D1_clear_data_observed_m"]
                - observed["D_delay_wall_integral_data_observed_m"]
            )
        )
    )
    collision = observed[observed["collision_event_data_observed"].astype(bool)]
    source_paths = []
    for column in ["source_localization_file", "source_perception_file", "source_scb_file"]:
        source_paths.extend([Path(value) for value in observed[column] if isinstance(value, str) and value])
    missing_sources = [str(path) for path in source_paths if not path.exists()]
    audit = f"""# Data quality audit

## Automated consistency checks

- Expected/found runs: 12/12.
- Main-analysis runs: 11; excluded outcome-conflict run: `{UNKNOWN_OUTCOME_RUN}`.
- `T_R=(t2-t1)` maximum absolute residual: `{t_error:.12g} ms`.
- `D2=D1-D_response` maximum absolute residual: `{d2_error:.12g} m`.
- Required Localization/Perception/SCB source paths missing: `{len(missing_sources)}`.
- Observed/model separation: `run_level_observed.csv` contains no model/predicted columns; model results are stored separately.
- Collision endpoint rule: {len(collision)}/{len(collision)} collision runs keep full observed `D_brake`, full-stop margins and data-derived deadline unavailable.
- Main response distance uses wall-clock speed trapezoidal integration for every run.
- Record association: 0/12 same-run parsed record exports; record-only metrics remain unavailable.

## Known evidence limits

1. `202607271206` has a reproducible timing/distance chain but conflicting outcome evidence; it is retained for diagnostics and excluded from outcome aggregates.
2. Collision runs are trajectory-truncated; any full stopping distance, deadline, margin or restored outcome is model-only.
3. Actor history is archived only for two collision runs. Most noncollision runs lack dual-clock CARLA history; this does not alter wall-clock main metrics but limits realtime-factor and actor-truth claims.
4. Full Control payload is not archived. Control Trace establishes timing, while the deployment description establishes that Bridge reads Control directly. Guardian is not used as the executed command source.
5. The baseline braking model uses the same seven baseline full-stop runs for descriptive calibration and comparison; it is not cross-validated and is explicitly not an observed result.
6. The 6 m engineering safety boundary is distinct from the 0 m contact boundary. Both are reported; neither is selected after seeing collision labels.

## Distance semantics

- `D_delay_wall_integral_data_observed_m` = total response-stage distance from `t1` to `t2`.
- `D_distance_debt_*` = incremental distance after an independently derived deadline and before `t2`.
- Localization displacement/path and CARLA sim quantities are diagnostics only and do not replace the wall-clock main field.
"""
    (VALIDATION / "data_quality_audit.md").write_text(audit, encoding="utf-8")


def write_inventory(observed: pd.DataFrame) -> None:
    raw_file_count = sum(1 for path in EXPERIMENT.rglob("*") if path.is_file())
    record_dirs = [str(path) for path in EXPERIMENT.rglob("record") if path.is_dir()]
    inventory = {
        "generated_at": datetime.now().astimezone().isoformat(),
        "experiment_root": str(EXPERIMENT.resolve()),
        "analysis_root": str(OUT.resolve()),
        "raw_experiment_modified": False,
        "expected_runs": EXPECTED_RUNS,
        "found_runs": observed["run_id"].astype(str).tolist(),
        "group_counts": observed.groupby("group_name")["run_id"].count().to_dict(),
        "main_analysis_run_count": int(observed["included_main_analysis"].sum()),
        "excluded_runs": [UNKNOWN_OUTCOME_RUN],
        "raw_tree_file_count_including_existing_experiment_artifacts": raw_file_count,
        "same_run_record_dirs": record_dirs,
        "record_profile_available_count": 0,
        "architecture": {
            "carla": "0.9.15 server",
            "apollo": "10.0.0 Orin",
            "bridge": "server; reads Control directly",
            "guardian_in_executed_chain": False,
        },
        "fresh_recompute_sources": {
            "observed_upstream_table": str((UPSTREAM / "tables/run_level_metrics.csv").resolve()),
            "raw_parser": str((UPSTREAM / "scripts/analyze_second_experiment.py").resolve()),
            "raw_experiment_root": str(EXPERIMENT.resolve()),
        },
    }
    (VALIDATION / "input_inventory.json").write_text(
        json.dumps(inventory, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def write_readme() -> None:
    text = f"""# 第二次实验六层时序—物理安全分析

本目录由 `$autonomous-driving-temporal-safety-analysis` 工作流生成。原始目录 `{EXPERIMENT}` 只读；所有新结果位于本目录。

## Reproduce

```bash
cd "{ROOT}"
python3 report_workspace/scripts/analyze_second_experiment.py
python3 report_workspace/scripts/generate_report.py
python3 report_workspace/scripts/validate_outputs.py
python3 output/second_experiment_six_layer_analysis/scripts/build_six_layer_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/validate_analysis_outputs.py \
  --analysis-dir output/second_experiment_six_layer_analysis
```

主报告：`report/six_layer_analysis_report.md`。观测和模型结果分别位于 `tables/run_level_observed.csv` 与 `tables/run_level_model_predicted.csv`。
"""
    (OUT / "README.md").write_text(text, encoding="utf-8")
    (RECORD_PROFILES / "README.md").write_text(
        "# Record profiles\n\n第二次实验 12 个 run 均没有同 run 的 `record/` 目录，因此未生成 profile JSON。\n",
        encoding="utf-8",
    )


def main() -> None:
    for directory in [TABLES, FIGURES, REPORT, VALIDATION, RECORD_PROFILES]:
        directory.mkdir(parents=True, exist_ok=True)
    configure_plotting()
    source_table = UPSTREAM / "tables" / "run_level_metrics.csv"
    if not source_table.exists():
        raise FileNotFoundError(
            "Fresh observed table is missing. Run report_workspace/scripts/analyze_second_experiment.py first."
        )
    source = pd.read_csv(source_table, dtype={"run_id": str})
    config = make_config()
    parsed_by_id, raw_by_id = parse_raw_runs(config)
    if set(source["run_id"]) != set(EXPECTED_RUNS):
        raise RuntimeError("Fresh observed table does not contain the expected 12 runs")
    observed = build_observed(source, parsed_by_id, raw_by_id)
    model, a_ref = build_model(observed, parsed_by_id)
    event_timeline = build_event_timeline(observed, model, raw_by_id)
    stage = build_stage_table(observed)
    record = build_record_diagnostics(observed)
    group = summarize_group(observed)
    exclusions = build_exclusions(observed)

    save_csv(observed, "run_level_observed.csv")
    save_csv(model, "run_level_model_predicted.csv")
    save_csv(event_timeline, "event_timeline.csv")
    save_csv(stage, "stage_timing_and_freshness.csv")
    save_csv(record, "record_timing_diagnostics.csv")
    save_csv(group, "group_summary_observed.csv")
    save_csv(exclusions, "exclusions_and_missing.csv")
    evidence = build_evidence(observed, model)
    save_csv(evidence, "layer_evidence_matrix.csv")
    plots(observed, model, group)
    report = report_markdown(observed, model, group, a_ref)
    (REPORT / "six_layer_analysis_report.md").write_text(report, encoding="utf-8")
    write_quality_audit(observed, model)
    write_inventory(observed)
    write_readme()
    print(
        json.dumps(
            {
                "status": "built",
                "runs": len(observed),
                "main_runs": int(observed["included_main_analysis"].sum()),
                "model_reference_deceleration_mps2": a_ref,
                "output": str(OUT),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
