#!/usr/bin/env python3
"""Observed-only safety margins and hidden-deadline calculations.

Collision runs remain right-censored: complete braking distance, observed-data
margins, and observed-data deadlines are unavailable and are never filled from
another run or a predictive model.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from zoneinfo import ZoneInfo


EXPERIMENT_ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = EXPERIMENT_ROOT.parent
CORE_ROOT = WORKSPACE_ROOT / "realtime_collision_analysis" / "src"
RUNTIME_PACKAGES = (
    WORKSPACE_ROOT / "realtime_collision_analysis" / ".python_packages"
)
sys.path.insert(0, str(RUNTIME_PACKAGES))
sys.path.insert(0, str(CORE_ROOT))

import numpy as np
import pandas as pd
import yaml

import realtime_collision_core as core


CONFIG_PATH = (
    WORKSPACE_ROOT
    / "realtime_collision_analysis"
    / "config"
    / "analysis_config.yaml"
)
OUTPUT_ROOT = EXPERIMENT_ROOT / "analysis_results"
EXCLUDED_RUN_IDS = {"202607271206"}


def make_config() -> dict[str, object]:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    config["analysis"]["title"] = "第二次车速隐形Deadline实验结果分析"
    config["analysis"]["pointcloud_count"] = 560_000
    config["groups"] = {
        "baseline": {
            "root": str(EXPERIMENT_ROOT / "baseline"),
            "nominal_injected_delay_ms": 0.0,
            "expected_runs": 7,
        },
        "delay_300ms": {
            "root": str(EXPERIMENT_ROOT / "300ms"),
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


def interpolated_clearance(
    stable: core.FusionObservation,
    state: dict[str, float],
    offset_m: float,
) -> float:
    dx = stable.x_m - state["x_m"]
    dy = stable.y_m - state["y_m"]
    return (
        dx * math.cos(state["heading_rad"])
        + dy * math.sin(state["heading_rad"])
        - offset_m
    )


def finite(values: pd.Series) -> np.ndarray:
    array = pd.to_numeric(values, errors="coerce").to_numpy(dtype=float)
    return array[np.isfinite(array)]


def main() -> None:
    config = make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    offset_m = float(config["geometry"]["combined_center_to_surface_offset_m"])
    rows: list[dict[str, object]] = []

    for spec in core.discover_runs(config):
        if spec.run_id in EXCLUDED_RUN_IDS:
            continue
        parsed = core.parse_run(spec, config, timezone)
        raw, debug = core.raw_run_metrics(parsed, config)
        stable = parsed.perception.get("stable")
        t1_s = core.fnum(raw.get("t_sensor_origin_s"))
        t2_s = core.fnum(raw.get("t_brake_effective_s"))
        state_t1 = core.interpolate_sample(parsed.localization, t1_s)
        v1_mps = state_t1["speed_mps"] if state_t1 is not None else math.nan
        d1_clear_m = (
            interpolated_clearance(stable, state_t1, offset_m)
            if stable is not None and state_t1 is not None
            else math.nan
        )
        latency_ms = (
            (t2_s - t1_s) * 1000.0
            if math.isfinite(t1_s) and math.isfinite(t2_s)
            else math.nan
        )
        d_delay_m = (
            core.integrate_speed(parsed.localization, t1_s, t2_s)
            if math.isfinite(t1_s) and math.isfinite(t2_s)
            else math.nan
        )
        response_mean_speed_mps = (
            d_delay_m / (latency_ms / 1000.0)
            if math.isfinite(d_delay_m)
            and math.isfinite(latency_ms)
            and latency_ms > 0.0
            else math.nan
        )

        collision = bool(raw.get("collision"))
        braking = debug.get("braking", {})
        state_t2 = core.interpolate_sample(parsed.localization, t2_s)
        near_stop_sample = debug.get("near_stop", {}).get("sample")
        strict_stop_sample = debug.get("stop", {}).get("sample")
        completion_sample = debug.get("brake_completion", {}).get("sample")

        def displacement_to(sample: object) -> float:
            if state_t2 is None or sample is None:
                return math.nan
            return math.dist(
                (state_t2["x_m"], state_t2["y_m"], state_t2["z_m"]),
                (sample.x_m, sample.y_m, sample.z_m),
            )

        final_lateral_offset_m = math.nan
        final_euclidean_surface_m = math.nan
        if (
            not collision
            and stable is not None
            and state_t1 is not None
            and completion_sample is not None
        ):
            endpoint_dx = stable.x_m - completion_sample.x_m
            endpoint_dy = stable.y_m - completion_sample.y_m
            endpoint_dz = stable.z_m - completion_sample.z_m
            final_lateral_offset_m = (
                -endpoint_dx * math.sin(state_t1["heading_rad"])
                + endpoint_dy * math.cos(state_t1["heading_rad"])
            )
            final_euclidean_surface_m = (
                math.sqrt(
                    endpoint_dx**2 + endpoint_dy**2 + endpoint_dz**2
                )
                - offset_m
            )
        d_brake_data_m = (
            core.fnum(braking.get("distance_m"))
            if not collision
            and braking.get("status") == "AVAILABLE"
            and braking.get("endpoint") == "MINIMUM_SPEED_PROXY"
            else math.nan
        )
        margin_collision_m = (
            d1_clear_m - d_delay_m - d_brake_data_m
            if all(
                math.isfinite(value)
                for value in [d1_clear_m, d_delay_m, d_brake_data_m]
            )
            else math.nan
        )
        margin_safety_6m = (
            margin_collision_m - 6.0
            if math.isfinite(margin_collision_m)
            else math.nan
        )
        deadline_collision_ms = (
            1000.0 * (d1_clear_m - d_brake_data_m) / v1_mps
            if all(
                math.isfinite(value)
                for value in [
                    d1_clear_m,
                    d_brake_data_m,
                    v1_mps,
                ]
            )
            and v1_mps > 0.0
            else math.nan
        )
        deadline_safety_6m_ms = (
            1000.0
            * (d1_clear_m - d_brake_data_m - 6.0)
            / v1_mps
            if all(
                math.isfinite(value)
                for value in [
                    d1_clear_m,
                    d_brake_data_m,
                    v1_mps,
                ]
            )
            and v1_mps > 0.0
            else math.nan
        )

        rows.append(
            {
                "group_name": spec.group_name,
                "run_id": spec.run_id,
                "v1_mps": v1_mps,
                "D1_clear_m": d1_clear_m,
                "actual_e2e_latency_ms": latency_ms,
                "D_delay_wall_integral_m": d_delay_m,
                "D_brake_data_m": d_brake_data_m,
                "D_brake_to_first_near_stop_m": displacement_to(
                    near_stop_sample
                ),
                "D_brake_to_strict_stop_m": displacement_to(
                    strict_stop_sample
                ),
                "D_brake_path_integral_m": (
                    core.fnum(braking.get("path_length_m"))
                    if not collision and braking.get("status") == "AVAILABLE"
                    else math.nan
                ),
                "M_safety_6m_data_m": margin_safety_6m,
                "M_collision_0m_data_m": margin_collision_m,
                "final_clearance_direct_projected_m": (
                    core.fnum(raw.get("final_clearance_m"))
                    if not collision
                    else math.nan
                ),
                "final_lateral_offset_m": final_lateral_offset_m,
                "final_euclidean_surface_m": final_euclidean_surface_m,
                "margin_minus_direct_clearance_m": (
                    margin_collision_m
                    - core.fnum(raw.get("final_clearance_m"))
                    if math.isfinite(margin_collision_m)
                    and math.isfinite(core.fnum(raw.get("final_clearance_m")))
                    else math.nan
                ),
                "collision": collision,
                "impact_speed_mps": core.fnum(raw.get("impact_speed_mps")),
                "response_mean_speed_mps": response_mean_speed_mps,
                "T_deadline_safety_6m_data_ms": deadline_safety_6m_ms,
                "T_deadline_collision_0m_data_ms": deadline_collision_ms,
                "deadline_safety_minus_observed_ms": (
                    deadline_safety_6m_ms - latency_ms
                    if math.isfinite(deadline_safety_6m_ms)
                    and math.isfinite(latency_ms)
                    else math.nan
                ),
                "deadline_collision_minus_observed_ms": (
                    deadline_collision_ms - latency_ms
                    if math.isfinite(deadline_collision_ms)
                    and math.isfinite(latency_ms)
                    else math.nan
                ),
                "braking_endpoint": braking.get("endpoint"),
                "braking_status": braking.get("status"),
                "t_near_stop_s": core.fnum(
                    debug.get("near_stop", {}).get("time_s")
                ),
                "t_strict_stop_s": core.fnum(
                    debug.get("stop", {}).get("time_s")
                ),
                "t_minimum_speed_s": core.fnum(
                    debug.get("brake_completion", {}).get("time_s")
                ),
                "source_localization_file": raw.get("source_localization_file"),
                "source_collision_file": raw.get("source_collision_file"),
            }
        )

    frame = pd.DataFrame(rows).sort_values(["group_name", "run_id"])
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    frame.to_csv(
        OUTPUT_ROOT / "chapter4_run_observed_safety_margins.csv",
        index=False,
        encoding="utf-8-sig",
    )

    summary_rows: list[dict[str, object]] = []
    metrics = [
        "D_brake_data_m",
        "M_safety_6m_data_m",
        "M_collision_0m_data_m",
        "T_deadline_safety_6m_data_ms",
        "T_deadline_collision_0m_data_ms",
        "deadline_safety_minus_observed_ms",
        "deadline_collision_minus_observed_ms",
    ]
    for group_name, group in frame.groupby("group_name", sort=False):
        for metric in metrics:
            values = finite(group[metric])
            summary_rows.append(
                {
                    "group_name": group_name,
                    "metric": metric,
                    "run_count": int(len(group)),
                    "available_run_count": int(values.size),
                    "mean": float(np.mean(values)) if values.size else math.nan,
                    "median": (
                        float(np.median(values)) if values.size else math.nan
                    ),
                    "min": float(np.min(values)) if values.size else math.nan,
                    "max": float(np.max(values)) if values.size else math.nan,
                }
            )
    pd.DataFrame(summary_rows).to_csv(
        OUTPUT_ROOT / "chapter4_group_observed_safety_summary.csv",
        index=False,
        encoding="utf-8-sig",
    )

    available = frame[
        np.isfinite(
            pd.to_numeric(
                frame["M_collision_0m_data_m"], errors="coerce"
            ).to_numpy(dtype=float)
        )
    ]
    audit = {
        "run_count": int(len(frame)),
        "complete_braking_run_count": int(len(available)),
        "collision_run_count": int(frame["collision"].sum()),
        "collision_margin_available_count": int(
            frame.loc[frame["collision"], "M_collision_0m_data_m"]
            .notna()
            .sum()
        ),
        "margin_identity_max_abs_error_m": float(
            np.max(
                np.abs(
                    finite(available["M_collision_0m_data_m"])
                    - finite(available["M_safety_6m_data_m"])
                    - 6.0
                )
            )
        ),
        "sensitivity_analysis": False,
        "inferential_tests": False,
        "predictive_model": False,
    }
    (OUTPUT_ROOT / "chapter4_observed_safety_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(frame.to_string(index=False))
    print("\nAUDIT")
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
