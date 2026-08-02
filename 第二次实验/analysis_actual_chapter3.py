#!/usr/bin/env python3
"""Compute the observed-only metrics used by Chapter 3.

The script reuses the established log parsers, but deliberately does not run
the braking model, counterfactual analysis, or sensitivity analysis.  Every
run uses the same t1/t2 detector and the same wall-clock speed integration.
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


CONFIG_PATH = (
    WORKSPACE_ROOT
    / "realtime_collision_analysis"
    / "config"
    / "analysis_config.yaml"
)
OUTPUT_ROOT = EXPERIMENT_ROOT / "analysis_results"
EXCLUDED_RUN_IDS = {"202607271206"}

import realtime_collision_core as core  # noqa: E402


def interpolated_clearance(
    stable: core.FusionObservation,
    state: dict[str, float],
    offset_m: float,
) -> float:
    """Longitudinal ego-front to obstacle-near-surface clearance."""

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


def describe(values: np.ndarray) -> dict[str, float | int]:
    return {
        "n": int(values.size),
        "mean": float(np.mean(values)),
        "sd": float(np.std(values, ddof=1)) if values.size > 1 else math.nan,
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def main() -> None:
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
    # The report has one fixed definition only; no alternate thresholds or
    # stable-frame counts are evaluated.
    config["stable_perception"]["sensitivity_frames"] = [
        config["stable_perception"]["primary_frames"]
    ]
    config["effective_brake"]["sensitivity_thresholds_mps2"] = [
        config["effective_brake"]["primary_decel_threshold_mps2"]
    ]

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
        state_t2 = core.interpolate_sample(parsed.localization, t2_s)

        d1_clear_m = math.nan
        d2_direct_m = math.nan
        v1_mps = math.nan
        v2_mps = math.nan
        if stable is not None and state_t1 is not None:
            d1_clear_m = interpolated_clearance(stable, state_t1, offset_m)
            v1_mps = state_t1["speed_mps"]
        if stable is not None and state_t2 is not None:
            d2_direct_m = interpolated_clearance(stable, state_t2, offset_m)
            v2_mps = state_t2["speed_mps"]

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
        d2_wall_budget_m = (
            d1_clear_m - d_delay_m
            if math.isfinite(d1_clear_m) and math.isfinite(d_delay_m)
            else math.nan
        )

        rows.append(
            {
                "group_name": spec.group_name,
                "nominal_injected_delay_ms": spec.nominal_delay_ms,
                "run_id": spec.run_id,
                "analysis_status": raw.get("analysis_status"),
                "target_id": parsed.target_id,
                "t1_wall_s": t1_s,
                "t2_wall_s": t2_s,
                "actual_e2e_latency_ms": latency_ms,
                "v1_interpolated_mps": v1_mps,
                "v2_interpolated_mps": v2_mps,
                "D1_clear_interpolated_m": d1_clear_m,
                "D_delay_wall_integral_m": d_delay_m,
                "response_mean_speed_mps": (
                    d_delay_m / (latency_ms / 1000.0)
                    if math.isfinite(d_delay_m)
                    and math.isfinite(latency_ms)
                    and latency_ms > 0.0
                    else math.nan
                ),
                "D2_clear_wall_budget_m": d2_wall_budget_m,
                "D2_clear_direct_diagnostic_m": d2_direct_m,
                "D2_budget_minus_direct_m": (
                    d2_wall_budget_m - d2_direct_m
                    if math.isfinite(d2_wall_budget_m)
                    and math.isfinite(d2_direct_m)
                    else math.nan
                ),
                "collision": bool(raw.get("collision")),
                "impact_speed_mps": core.fnum(raw.get("impact_speed_mps")),
                "scb_requested_delay_ms": core.fnum(
                    raw.get("scb_requested_delay_ms")
                ),
                "scb_actual_wall_delay_ms": core.fnum(
                    raw.get("scb_actual_wall_delay_ms")
                ),
                "brake_onset_attribution": raw.get("brake_onset_attribution"),
                "clock_alignment_status": raw.get("clock_alignment_status"),
                "source_localization_file": raw.get("source_localization_file"),
                "source_perception_file": raw.get("source_perception_file"),
                "source_collision_file": raw.get("source_collision_file"),
                "brake_onset_debug_status": debug.get("brake_onset", {}).get(
                    "status"
                ),
            }
        )

    frame = pd.DataFrame(rows).sort_values(
        ["nominal_injected_delay_ms", "run_id"]
    )
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    frame.to_csv(
        OUTPUT_ROOT / "chapter3_run_observed_metrics.csv",
        index=False,
        encoding="utf-8-sig",
    )

    summary_rows: list[dict[str, object]] = []
    metrics = [
        "scb_actual_wall_delay_ms",
        "actual_e2e_latency_ms",
        "v1_interpolated_mps",
        "v2_interpolated_mps",
        "response_mean_speed_mps",
        "D_delay_wall_integral_m",
        "D1_clear_interpolated_m",
        "D2_clear_wall_budget_m",
    ]
    for group_name, group in frame.groupby("group_name", sort=False):
        for metric in metrics:
            summary_rows.append(
                {
                    "group_name": group_name,
                    "metric": metric,
                    **describe(finite(group[metric])),
                    "collision_count": int(group["collision"].sum()),
                    "run_count": int(len(group)),
                }
            )
    summary = pd.DataFrame(summary_rows)
    summary.to_csv(
        OUTPUT_ROOT / "chapter3_group_observed_summary.csv",
        index=False,
        encoding="utf-8-sig",
    )

    description = {
        "n": int(len(frame)),
        "response_mean_speed_min_mps": float(
            np.min(finite(frame["response_mean_speed_mps"]))
        ),
        "response_mean_speed_max_mps": float(
            np.max(finite(frame["response_mean_speed_mps"]))
        ),
    }
    (OUTPUT_ROOT / "chapter3_observed_description.json").write_text(
        json.dumps(description, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(frame.to_string(index=False))
    print("\nGROUP SUMMARY")
    print(summary.to_string(index=False))
    print("\nDESCRIPTION")
    print(json.dumps(description, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
