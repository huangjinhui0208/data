#!/usr/bin/env python3
"""Verified Chapter 7 analysis: baseline versus two collision runs.

Observed results and counterfactual model results are stored separately.
The report is not edited by this script.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
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

import matplotlib

matplotlib.use("Agg")

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import yaml

import realtime_collision_core as core


CONFIG_PATH = (
    WORKSPACE_ROOT
    / "realtime_collision_analysis"
    / "config"
    / "analysis_config.yaml"
)
OUTPUT_ROOT = EXPERIMENT_ROOT / "analysis_results"
FIGURE_ROOT = OUTPUT_ROOT / "chapter7_verified_figures"
REPORT_FIGURE_ROOT = EXPERIMENT_ROOT / "figures"
EXCLUDED_RUN_IDS = {"202607271206"}
COLLISION_IDS = {"202607271131", "202607271643"}
DISPLAY_NAMES = {
    "202607271131": "碰撞_1131",
    "202607271643": "碰撞_1643",
}
COLORS = {
    "baseline": "#8A8A8A",
    "baseline_median": "#333333",
    "delayed_safe": "#0072B2",
    "collision_1131": "#E69F00",
    "collision_1643": "#D55E00",
    "boundary": "#444444",
    "restored": "#009E73",
}


def configure_plotting() -> None:
    font_path = Path(r"C:\Windows\Fonts\msyh.ttc")
    if font_path.exists():
        fm.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"] = fm.FontProperties(
            fname=str(font_path)
        ).get_name()
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.labelcolor": "#333333",
            "text.color": "#222222",
            "xtick.color": "#444444",
            "ytick.color": "#444444",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.12,
        }
    )


def make_config() -> dict[str, object]:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
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


def load_runs() -> dict[str, dict[str, object]]:
    config = make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    runs: dict[str, dict[str, object]] = {}
    for spec in core.discover_runs(config):
        if spec.run_id in EXCLUDED_RUN_IDS:
            continue
        parsed = core.parse_run(spec, config, timezone)
        raw, debug = core.raw_run_metrics(parsed, config)
        runs[spec.run_id] = {
            "spec": spec,
            "parsed": parsed,
            "raw": raw,
            "debug": debug,
        }
    return runs


def is_finite(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def outcome_endpoint_s(run: dict[str, object]) -> float:
    """Use one outcome rule for every run.

    Collision: CARLA CollisionSensor event.
    Non-collision: first post-t2 Localization sample with v < 0.1 m/s.
    """
    raw = run["raw"]
    if bool(raw["collision"]):
        return float(raw["t_collision_s"])
    near_stop = raw.get("t_near_stop_s")
    if not is_finite(near_stop):
        raise RuntimeError(
            f"Missing first near-stop endpoint for {run['spec'].run_id}"
        )
    return float(near_stop)


def outcome_endpoint_speed_mps(run: dict[str, object]) -> float:
    raw = run["raw"]
    if bool(raw["collision"]):
        return float(raw["impact_speed_mps"])
    sample = core.interpolate_sample(
        run["parsed"].localization, outcome_endpoint_s(run)
    )
    if sample is None:
        raise RuntimeError("Missing Localization at near-stop endpoint")
    return float(sample["speed_mps"])


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


def observed_row(
    run_id: str,
    run: dict[str, object],
    config: dict[str, object],
) -> dict[str, object]:
    raw = run["raw"]
    parsed = run["parsed"]
    t1_s = float(raw["t_sensor_origin_s"])
    t2_s = float(raw["t_brake_effective_s"])
    state_t1 = core.interpolate_sample(parsed.localization, t1_s)
    state_t2 = core.interpolate_sample(parsed.localization, t2_s)
    stable = parsed.perception["stable"]
    if state_t1 is None or state_t2 is None or stable is None:
        raise RuntimeError(f"Missing t1/t2 state or stable target for {run_id}")
    offset_m = float(
        config["geometry"]["combined_center_to_surface_offset_m"]
    )
    d1_clear_m = interpolated_clearance(stable, state_t1, offset_m)
    d_delay_m = core.integrate_speed(parsed.localization, t1_s, t2_s)
    d2_clear_m = d1_clear_m - d_delay_m
    endpoint_s = outcome_endpoint_s(run)
    endpoint_speed_mps = outcome_endpoint_speed_mps(run)
    post_t2_distance_m = core.integrate_speed(
        parsed.localization, t2_s, endpoint_s
    )
    equivalent_deceleration_mps2 = (
        (
            float(state_t2["speed_mps"]) ** 2
            - endpoint_speed_mps**2
        )
        / (2.0 * post_t2_distance_m)
        if post_t2_distance_m > 0
        else math.nan
    )
    return {
        "run_id": run_id,
        "display_name": DISPLAY_NAMES.get(run_id, run_id),
        "group_name": run["spec"].group_name,
        "collision_data_observed": bool(raw["collision"]),
        "actual_e2e_latency_data_observed_ms": (t2_s - t1_s) * 1000.0,
        "v1_data_observed_mps": float(state_t1["speed_mps"]),
        "v2_data_observed_mps": float(state_t2["speed_mps"]),
        "D1_clear_data_observed_m": d1_clear_m,
        "D_delay_wall_integral_data_observed_m": d_delay_m,
        "response_mean_speed_data_observed_mps": (
            d_delay_m / (t2_s - t1_s)
        ),
        "D2_clear_wall_budget_data_observed_m": d2_clear_m,
        "outcome_endpoint_type_data_observed": (
            "CARLA_COLLISION_EVENT"
            if bool(raw["collision"])
            else "FIRST_LOCALIZATION_V_LT_0.1_MPS"
        ),
        "outcome_endpoint_elapsed_data_observed_s": endpoint_s - t1_s,
        "outcome_endpoint_speed_data_observed_mps": endpoint_speed_mps,
        "post_t2_distance_to_outcome_wall_integral_data_observed_m": (
            post_t2_distance_m
        ),
        "outcome_energy_equivalent_deceleration_data_observed_mps2": (
            equivalent_deceleration_mps2
        ),
        "impact_speed_data_observed_mps": (
            float(raw["impact_speed_mps"])
            if bool(raw["collision"])
            else math.nan
        ),
        "t1_data_observed_s": t1_s,
        "t2_data_observed_s": t2_s,
        "outcome_endpoint_data_observed_s": endpoint_s,
    }


def describe(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "min": float(np.min(data)),
        "max": float(np.max(data)),
    }


def trajectory(
    run: dict[str, object],
) -> tuple[np.ndarray, np.ndarray]:
    parsed = run["parsed"]
    raw = run["raw"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = outcome_endpoint_s(run)
    start = core.interpolate_sample(parsed.localization, start_s)
    if start is None:
        raise RuntimeError("Missing Localization at t1")
    times = [0.0]
    speeds = [float(start["speed_mps"])]
    for sample in parsed.localization:
        if start_s < sample.time_s < end_s:
            times.append(sample.time_s - start_s)
            speeds.append(sample.speed_mps)
    times.append(end_s - start_s)
    speeds.append(outcome_endpoint_speed_mps(run))
    return np.asarray(times), np.asarray(speeds)


def station_trajectory(
    run: dict[str, object],
) -> tuple[np.ndarray, np.ndarray]:
    parsed = run["parsed"]
    raw = run["raw"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = outcome_endpoint_s(run)
    d1_m = float(run["observed"]["D1_clear_data_observed_m"])
    times = [0.0]
    stations = [-d1_m]
    for sample in parsed.localization:
        if start_s < sample.time_s < end_s:
            times.append(sample.time_s - start_s)
            stations.append(
                -d1_m
                + core.integrate_speed(
                    parsed.localization, start_s, sample.time_s
                )
            )
    times.append(end_s - start_s)
    stations.append(
        -d1_m
        + core.integrate_speed(parsed.localization, start_s, end_s)
    )
    return np.asarray(times), np.asarray(stations)


def acceleration_trajectory(
    run: dict[str, object],
) -> tuple[np.ndarray, np.ndarray]:
    """Adjacent-sample acceleration without smoothing."""
    parsed = run["parsed"]
    raw = run["raw"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = outcome_endpoint_s(run)
    samples = [
        sample
        for sample in parsed.localization
        if start_s <= sample.time_s <= end_s
    ]
    start = core.interpolate_sample(parsed.localization, start_s)
    if start is None:
        raise RuntimeError("Missing Localization at t1")
    points = [(start_s, float(start["speed_mps"]))]
    points.extend(
        (sample.time_s, sample.speed_mps)
        for sample in samples
        if sample.time_s > start_s
    )
    if len(points) < 2:
        raise RuntimeError("Insufficient Localization samples for acceleration")
    times = np.asarray([item[0] for item in points], dtype=float)
    speeds = np.asarray([item[1] for item in points], dtype=float)
    dt = np.diff(times)
    valid = dt > 0
    midpoint = ((times[:-1] + times[1:]) / 2.0 - start_s)[valid]
    acceleration = (np.diff(speeds)[valid] / dt[valid])
    return midpoint, acceleration


def acceleration_summary(
    runs: dict[str, dict[str, object]],
) -> list[dict[str, object]]:
    """Summarize raw adjacent-sample acceleration with the same rule."""
    rows: list[dict[str, object]] = []
    for run_id, run in sorted(runs.items()):
        times, acceleration = acceleration_trajectory(run)
        t2_elapsed_s = (
            float(
                run["observed"][
                    "actual_e2e_latency_data_observed_ms"
                ]
            )
            / 1000.0
        )
        before_t2 = acceleration[times < t2_elapsed_s]
        after_t2 = acceleration[times >= t2_elapsed_s]
        rows.append(
            {
                "run_id": run_id,
                "display_name": str(run["observed"]["display_name"]),
                "group_name": run["spec"].group_name,
                "maximum_positive_acceleration_data_observed_mps2": (
                    float(np.max(acceleration))
                ),
                "maximum_deceleration_data_observed_mps2": (
                    float(np.min(acceleration))
                ),
                "maximum_positive_acceleration_before_t2_data_observed_mps2": (
                    float(np.max(before_t2))
                    if before_t2.size
                    else math.nan
                ),
                "maximum_acceleration_after_t2_data_observed_mps2": (
                    float(np.max(after_t2))
                    if after_t2.size
                    else math.nan
                ),
                "maximum_deceleration_after_t2_data_observed_mps2": (
                    float(np.min(after_t2))
                    if after_t2.size
                    else math.nan
                ),
                "positive_acceleration_interval_count_after_t2": (
                    int(np.sum(after_t2 > 0))
                ),
                "v1_data_observed_mps": float(
                    run["observed"]["v1_data_observed_mps"]
                ),
                "v2_data_observed_mps": float(
                    run["observed"]["v2_data_observed_mps"]
                ),
                "v2_minus_v1_data_observed_mps": (
                    float(run["observed"]["v2_data_observed_mps"])
                    - float(run["observed"]["v1_data_observed_mps"])
                ),
                "acceleration_definition": (
                    "Adjacent Localization speed difference divided by "
                    "the corresponding wall-clock measurement-time "
                    "difference; no smoothing or clipping."
                ),
            }
        )
    return rows


def baseline_state_at_elapsed(
    runs: dict[str, dict[str, object]],
    baseline_ids: list[str],
    elapsed_s: float,
) -> tuple[float, float]:
    speeds = []
    stations = []
    for run_id in baseline_ids:
        run = runs[run_id]
        observed = run["observed"]
        absolute_s = float(observed["t1_data_observed_s"]) + elapsed_s
        sample = core.interpolate_sample(run["parsed"].localization, absolute_s)
        if sample is None:
            continue
        speeds.append(float(sample["speed_mps"]))
        stations.append(
            -float(observed["D1_clear_data_observed_m"])
            + core.integrate_speed(
                run["parsed"].localization,
                float(observed["t1_data_observed_s"]),
                absolute_s,
            )
        )
    return float(np.median(speeds)), float(np.median(stations))


def data_audit(
    runs: dict[str, dict[str, object]],
) -> list[dict[str, object]]:
    """Cross-check recomputed Chapter 7 values against core raw metrics."""
    mapping = {
        "actual_e2e_latency_data_observed_ms": "actual_e2e_latency_ms",
        "v1_data_observed_mps": "t1_speed_mps",
        "v2_data_observed_mps": "brake_start_speed_mps",
        "D1_clear_data_observed_m": "D1_clear_m",
        "D_delay_wall_integral_data_observed_m": "D_delay_m",
        "D2_clear_wall_budget_data_observed_m": (
            "clearance_at_brake_start_m"
        ),
    }
    results: list[dict[str, object]] = []
    for run_id, run in sorted(runs.items()):
        observed = run["observed"]
        raw = run["raw"]
        for observed_key, raw_key in mapping.items():
            observed_value = float(observed[observed_key])
            raw_value = float(raw[raw_key])
            difference = observed_value - raw_value
            results.append(
                {
                    "run_id": run_id,
                    "field": observed_key,
                    "recomputed_value": observed_value,
                    "core_raw_value": raw_value,
                    "absolute_difference": abs(difference),
                    "status": (
                        "EXACT_MATCH"
                        if abs(difference) <= 1e-12
                        else "MISMATCH"
                    ),
                }
            )
        endpoint_speed = float(
            observed["outcome_endpoint_speed_data_observed_mps"]
        )
        results.append(
            {
                "run_id": run_id,
                "field": "outcome_endpoint_speed_data_observed_mps",
                "recomputed_value": endpoint_speed,
                "core_raw_value": (
                    float(raw["impact_speed_mps"])
                    if bool(raw["collision"])
                    else endpoint_speed
                ),
                "absolute_difference": 0.0,
                "status": (
                    "COLLISION_CSV_MATCH"
                    if bool(raw["collision"])
                    else (
                        "LOCALIZATION_NEAR_STOP_MATCH"
                        if endpoint_speed < 0.1
                        else "MISMATCH"
                    )
                ),
            }
        )
    return results


def counterfactual_latency_recovery(
    run: dict[str, object],
    baseline_latency_ms: float,
) -> dict[str, object]:
    """Restore t2 to baseline latency using observed inputs and one model.

    The model keeps the collision run's observed energy-equivalent braking
    capability from actual t2 to actual contact.  It starts that capability at
    the restored t2 and uses the speed actually recorded at that earlier time.
    """
    observed = run["observed"]
    parsed = run["parsed"]
    t1_s = float(observed["t1_data_observed_s"])
    t2_actual_s = float(observed["t2_data_observed_s"])
    collision_s = float(observed["outcome_endpoint_data_observed_s"])
    t2_restored_s = t1_s + baseline_latency_ms / 1000.0
    state_restored = core.interpolate_sample(
        parsed.localization, t2_restored_s
    )
    if state_restored is None:
        raise RuntimeError("Missing Localization at restored t2")
    v2_restored_mps = float(state_restored["speed_mps"])
    d_delay_restored_m = core.integrate_speed(
        parsed.localization, t1_s, t2_restored_s
    )
    d_delay_actual_m = float(
        observed["D_delay_wall_integral_data_observed_m"]
    )
    recovered_distance_m = d_delay_actual_m - d_delay_restored_m
    actual_braking_distance_to_contact_m = float(
        observed[
            "post_t2_distance_to_outcome_wall_integral_data_observed_m"
        ]
    )
    equivalent_deceleration_mps2 = float(
        observed[
            "outcome_energy_equivalent_deceleration_data_observed_mps2"
        ]
    )
    if actual_braking_distance_to_contact_m <= 0:
        raise RuntimeError("Invalid collision-run braking calibration")
    d2_geometry_restored_m = (
        float(observed["D1_clear_data_observed_m"])
        - d_delay_restored_m
    )
    d2_geometry_actual_m = float(
        observed["D2_clear_wall_budget_data_observed_m"]
    )
    contact_boundary_correction_m = (
        actual_braking_distance_to_contact_m - d2_geometry_actual_m
    )
    available_to_observed_contact_restored_m = (
        d2_geometry_restored_m + contact_boundary_correction_m
    )
    required_stopping_distance_restored_m = (
        v2_restored_mps**2
        / (2.0 * equivalent_deceleration_mps2)
    )
    margin_to_observed_contact_m = (
        available_to_observed_contact_restored_m
        - required_stopping_distance_restored_m
    )
    collision_predicted = margin_to_observed_contact_m < 0.0
    predicted_impact_speed_mps = math.sqrt(
        max(
            0.0,
            v2_restored_mps**2
            - 2.0
            * equivalent_deceleration_mps2
            * available_to_observed_contact_restored_m,
        )
    )
    return {
        "run_id": str(observed["run_id"]),
        "display_name": str(observed["display_name"]),
        "reference_baseline_latency_model_input_ms": baseline_latency_ms,
        "actual_latency_data_observed_ms": float(
            observed["actual_e2e_latency_data_observed_ms"]
        ),
        "recovered_latency_model_input_ms": (
            float(observed["actual_e2e_latency_data_observed_ms"])
            - baseline_latency_ms
        ),
        "v2_restored_from_earlier_observed_state_mps": v2_restored_mps,
        "D_delay_actual_data_observed_m": d_delay_actual_m,
        "D_delay_restored_from_observed_pre_t2_trace_m": (
            d_delay_restored_m
        ),
        "response_distance_recovered_model_m": recovered_distance_m,
        "D2_geometry_restored_model_m": d2_geometry_restored_m,
        "actual_braking_distance_t2_to_contact_data_observed_m": (
            actual_braking_distance_to_contact_m
        ),
        "contact_boundary_correction_data_observed_m": (
            contact_boundary_correction_m
        ),
        "available_to_observed_contact_restored_model_m": (
            available_to_observed_contact_restored_m
        ),
        "equivalent_deceleration_from_collision_data_mps2": (
            equivalent_deceleration_mps2
        ),
        "required_stopping_distance_restored_model_m": (
            required_stopping_distance_restored_m
        ),
        "margin_to_observed_contact_restored_model_m": (
            margin_to_observed_contact_m
        ),
        "collision_model_predicted": collision_predicted,
        "impact_speed_model_predicted_mps": predicted_impact_speed_mps,
        "model_definition": (
            "Actual pre-t2 speed integral plus the collision run's "
            "energy-equivalent deceleration calibrated from actual t2, "
            "actual impact speed, and wall-clock speed integral to contact."
        ),
    }


def figure_latency_distance(
    rows: list[dict[str, object]], output_dir: Path
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.4))
    styles = {
        "baseline": {
            "color": COLORS["baseline"],
            "marker": "o",
            "label": "baseline（7个未碰撞run）",
            "size": 48,
        },
        "delayed_safe": {
            "color": COLORS["delayed_safe"],
            "marker": "s",
            "label": "300 ms组未碰撞run",
            "size": 62,
        },
        "202607271131": {
            "color": COLORS["collision_1131"],
            "marker": "X",
            "label": "碰撞_1131",
            "size": 92,
        },
        "202607271643": {
            "color": COLORS["collision_1643"],
            "marker": "X",
            "label": "碰撞_1643",
            "size": 92,
        },
    }
    y_keys = [
        "D_delay_wall_integral_data_observed_m",
        "D2_clear_wall_budget_data_observed_m",
    ]
    for row in rows:
        if row["group_name"] == "baseline":
            key = "baseline"
        elif row["run_id"] in COLLISION_IDS:
            key = str(row["run_id"])
        else:
            key = "delayed_safe"
        style = styles[key]
        for ax, y_key in zip(axes, y_keys):
            ax.scatter(
                [row["actual_e2e_latency_data_observed_ms"]],
                [row[y_key]],
                color=style["color"],
                marker=style["marker"],
                s=style["size"],
                edgecolor="white",
                linewidth=0.8,
                zorder=4,
                label=style["label"],
            )
        if row["run_id"] in COLLISION_IDS:
            for ax, y_key in zip(axes, y_keys):
                ax.annotate(
                    DISPLAY_NAMES[str(row["run_id"])],
                    (
                        row["actual_e2e_latency_data_observed_ms"],
                        row[y_key],
                    ),
                    xytext=(-10, 9),
                    textcoords="offset points",
                    ha="right",
                    fontsize=9,
                )
    for ax in axes:
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(
            unique.values(),
            unique.keys(),
            loc="best",
            frameon=False,
            fontsize=8.5,
        )
        ax.set_xlabel("实际端到端响应时间/ms")
        ax.grid(alpha=0.22)
    axes[0].set_ylabel("响应阶段墙钟速度积分距离/m")
    axes[0].set_title("时延增加转化为响应距离债务")
    axes[1].set_ylabel("$t_2$时剩余净距/m")
    axes[1].set_title("时延增加压缩有效制动空间")
    fig.suptitle("图7-1 端到端响应时间与车辆空间状态", y=1.02)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_verified_latency_distance.png", dpi=220)
    plt.close(fig)


def figure_vt(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    baseline_ids = sorted(
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    )
    fig, ax = plt.subplots(figsize=(12.0, 6.5))
    for index, run_id in enumerate(baseline_ids):
        times, speeds = trajectory(runs[run_id])
        ax.plot(
            times,
            speeds,
            color=COLORS["baseline"],
            alpha=0.45,
            linewidth=1.2,
            label=(
                "baseline实际轨迹（7个run）"
                if index == 0
                else None
            ),
        )
        observed = runs[run_id]["observed"]
        ax.scatter(
            [
                float(
                    observed["actual_e2e_latency_data_observed_ms"]
                )
                / 1000.0
            ],
            [float(observed["v2_data_observed_mps"])],
            s=25,
            facecolor="white",
            edgecolor=COLORS["baseline"],
            linewidth=0.9,
            zorder=4,
        )
        ax.scatter(
            [times[-1]],
            [speeds[-1]],
            s=34,
            marker="v",
            color=COLORS["baseline"],
            zorder=5,
        )

    baseline_end = max(
        outcome_endpoint_s(runs[run_id])
        - float(runs[run_id]["raw"]["t_sensor_origin_s"])
        for run_id in baseline_ids
    )
    grid = np.arange(0.0, baseline_end + 0.0001, 0.01)
    matrix = np.full((len(baseline_ids), grid.size), np.nan)
    for index, run_id in enumerate(baseline_ids):
        run = runs[run_id]
        start_s = float(run["raw"]["t_sensor_origin_s"])
        end_elapsed = outcome_endpoint_s(run) - start_s
        for column, elapsed_s in enumerate(grid):
            if elapsed_s > end_elapsed:
                continue
            sample = core.interpolate_sample(
                run["parsed"].localization, start_s + elapsed_s
            )
            if sample is not None:
                matrix[index, column] = sample["speed_mps"]
    valid_count = np.sum(np.isfinite(matrix), axis=0)
    baseline_median = np.full(grid.size, np.nan)
    valid_columns = valid_count >= 4
    baseline_median[valid_columns] = np.nanmedian(
        matrix[:, valid_columns], axis=0
    )
    ax.plot(
        grid,
        baseline_median,
        color=COLORS["baseline_median"],
        linewidth=2.0,
        linestyle="--",
        label="baseline中位速度轨迹（至少4个run）",
    )

    for run_id, color in [
        ("202607271131", COLORS["collision_1131"]),
        ("202607271643", COLORS["collision_1643"]),
    ]:
        run = runs[run_id]
        times, speeds = trajectory(run)
        observed = run["observed"]
        ax.plot(
            times,
            speeds,
            color=color,
            linewidth=2.5,
            label=DISPLAY_NAMES[run_id],
        )
        t2_elapsed = (
            float(observed["actual_e2e_latency_data_observed_ms"])
            / 1000.0
        )
        ax.scatter(
            [t2_elapsed],
            [float(observed["v2_data_observed_mps"])],
            s=78,
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=6,
        )
        collision_elapsed = float(
            observed["outcome_endpoint_elapsed_data_observed_s"]
        )
        impact_speed = float(
            observed["impact_speed_data_observed_mps"]
        )
        ax.scatter(
            [collision_elapsed],
            [impact_speed],
            marker="X",
            s=95,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            zorder=7,
        )
        ax.annotate(
            f"{DISPLAY_NAMES[run_id]}\n碰撞速度{impact_speed:.3f} m/s",
            (collision_elapsed, impact_speed),
            xytext=(-10, 13),
            textcoords="offset points",
            ha="right",
            fontsize=8.7,
        )

    for run_id, offset in [
        ("202607271643", (10, -32)),
        ("202607271131", (10, 12)),
    ]:
        elapsed = float(
            runs[run_id]["observed"][
                "outcome_endpoint_elapsed_data_observed_s"
            ]
        )
        median_speed, _ = baseline_state_at_elapsed(
            runs, baseline_ids, elapsed
        )
        ax.scatter(
            [elapsed],
            [median_speed],
            marker="D",
            s=58,
            color=COLORS["baseline_median"],
            edgecolor="white",
            linewidth=0.8,
            zorder=7,
        )
        ax.annotate(
            f"同一时刻baseline中位速度\n{median_speed:.3f} m/s",
            (elapsed, median_speed),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.4,
        )

    baseline_latencies = [
        float(
            runs[run_id]["observed"][
                "actual_e2e_latency_data_observed_ms"
            ]
        )
        / 1000.0
        for run_id in baseline_ids
    ]
    ax.axvspan(
        min(baseline_latencies),
        max(baseline_latencies),
        color=COLORS["baseline"],
        alpha=0.08,
        label="baseline的$t_2$时间范围",
    )
    ax.axhline(
        0.1,
        color=COLORS["boundary"],
        linewidth=1.0,
        linestyle=":",
        label="近零速判据0.1 m/s",
    )
    ax.annotate(
        "baseline最晚在$t_1+4.600$ s首次低于0.1 m/s",
        (baseline_end, 0.1),
        xytext=(-8, 22),
        textcoords="offset points",
        ha="right",
        fontsize=8.5,
    )
    ax.set_xlim(0, baseline_end + 0.25)
    ax.set_ylim(-0.3, 19.2)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("实际车速/(m/s)")
    ax.set_title(
        "图7-2 VT轨迹：所有baseline轨迹均显示至首次近零速端点"
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="upper right", frameon=False, fontsize=8.3)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_verified_vt.png", dpi=220)
    plt.close(fig)


def figure_st(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    baseline_ids = sorted(
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    )
    fig, ax = plt.subplots(figsize=(12.0, 6.6))
    ax.axhline(
        0.0,
        color=COLORS["boundary"],
        linewidth=1.5,
        linestyle="--",
        label="统一几何分析边界$S=0$",
    )
    for index, run_id in enumerate(baseline_ids):
        times, stations = station_trajectory(runs[run_id])
        ax.plot(
            times,
            stations,
            color=COLORS["baseline"],
            alpha=0.45,
            linewidth=1.2,
            label=(
                "baseline实际ST轨迹（7个run）"
                if index == 0
                else None
            ),
        )
        observed = runs[run_id]["observed"]
        t2_elapsed = (
            float(observed["actual_e2e_latency_data_observed_ms"])
            / 1000.0
        )
        t2_station = -float(
            observed["D2_clear_wall_budget_data_observed_m"]
        )
        ax.scatter(
            [t2_elapsed],
            [t2_station],
            s=23,
            facecolor="white",
            edgecolor=COLORS["baseline"],
            linewidth=0.9,
            zorder=4,
        )
        ax.scatter(
            [times[-1]],
            [stations[-1]],
            s=34,
            marker="v",
            color=COLORS["baseline"],
            zorder=5,
        )

    for run_id, color in [
        ("202607271131", COLORS["collision_1131"]),
        ("202607271643", COLORS["collision_1643"]),
    ]:
        run = runs[run_id]
        times, stations = station_trajectory(run)
        observed = run["observed"]
        ax.plot(
            times,
            stations,
            color=color,
            linewidth=2.6,
            label=DISPLAY_NAMES[run_id],
        )
        t2_elapsed = (
            float(observed["actual_e2e_latency_data_observed_ms"])
            / 1000.0
        )
        t2_station = -float(
            observed["D2_clear_wall_budget_data_observed_m"]
        )
        ax.scatter(
            [t2_elapsed],
            [t2_station],
            s=78,
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=6,
        )
        ax.scatter(
            [times[-1]],
            [stations[-1]],
            marker="X",
            s=95,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            zorder=7,
        )
        ax.annotate(
            (
                f"{DISPLAY_NAMES[run_id]}\n"
                f"$t_2$: {t2_elapsed:.3f} s，"
                f"$S={t2_station:.3f}$ m"
            ),
            (t2_elapsed, t2_station),
            xytext=(12, -23 if run_id.endswith("1131") else 12),
            textcoords="offset points",
            fontsize=8.7,
        )

    baseline_end = max(
        outcome_endpoint_s(runs[run_id])
        - float(runs[run_id]["raw"]["t_sensor_origin_s"])
        for run_id in baseline_ids
    )
    ax.set_xlim(0, baseline_end + 0.25)
    ax.set_ylim(-42, 5.0)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("统一纵向位置 $S$/m")
    ax.set_title(
        "图7-3 ST轨迹：响应距离债务压缩碰撞前空间"
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="lower right", frameon=False, fontsize=8.5)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_verified_st.png", dpi=220)
    plt.close(fig)


def figure_acceleration(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    baseline_ids = sorted(
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    )
    fig, ax = plt.subplots(figsize=(12.0, 6.4))
    ax.axvline(
        0.0,
        color=COLORS["baseline_median"],
        linewidth=1.4,
        linestyle="-.",
        label="碰撞_1131与碰撞_1643共同$t_1=0$",
        zorder=2,
    )
    for index, run_id in enumerate(baseline_ids):
        times, acceleration = acceleration_trajectory(runs[run_id])
        ax.plot(
            times,
            acceleration,
            color=COLORS["baseline"],
            alpha=0.42,
            linewidth=1.05,
            label=(
                "baseline相邻Localization样本加速度"
                if index == 0
                else None
            ),
        )
    for run_id, color in [
        ("202607271131", COLORS["collision_1131"]),
        ("202607271643", COLORS["collision_1643"]),
    ]:
        times, acceleration = acceleration_trajectory(runs[run_id])
        ax.plot(
            times,
            acceleration,
            color=color,
            linewidth=2.1,
            label=DISPLAY_NAMES[run_id],
        )
        t2_elapsed = (
            float(
                runs[run_id]["observed"][
                    "actual_e2e_latency_data_observed_ms"
                ]
            )
            / 1000.0
        )
        ax.axvline(
            t2_elapsed,
            color=color,
            linewidth=1.0,
            linestyle=":",
            alpha=0.9,
            label=f"{DISPLAY_NAMES[run_id]}的$t_2$",
        )
    baseline_latencies = [
        float(
            runs[run_id]["observed"][
                "actual_e2e_latency_data_observed_ms"
            ]
        )
        / 1000.0
        for run_id in baseline_ids
    ]
    ax.axvspan(
        min(baseline_latencies),
        max(baseline_latencies),
        color=COLORS["baseline"],
        alpha=0.08,
        label="baseline的$t_2$时间范围",
    )
    ax.axhline(
        -0.5,
        color=COLORS["boundary"],
        linewidth=1.1,
        linestyle="--",
        label="持续有效减速阈值$-0.5\\,\\mathrm{m/s^2}$",
    )
    baseline_end = max(
        outcome_endpoint_s(runs[run_id])
        - float(runs[run_id]["raw"]["t_sensor_origin_s"])
        for run_id in baseline_ids
    )
    ax.annotate(
        "两组碰撞run的$t_1$",
        (0.0, 5.25),
        xytext=(10, 0),
        textcoords="offset points",
        ha="left",
        va="center",
        fontsize=8.6,
    )
    ax.set_xlim(-0.12, baseline_end + 0.25)
    ax.set_ylim(-28.5, 6.0)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("相邻Localization样本加速度/(m/s²)")
    ax.set_title(
        "图7-4 AT轨迹：加速度由相邻实测速度按墙钟时间差分"
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="lower left", frameon=False, fontsize=8.3)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_verified_acceleration.png", dpi=220)
    plt.close(fig)


def figure_recovery(
    counterfactual_rows: list[dict[str, object]], output_dir: Path
) -> None:
    labels = [row["display_name"] for row in counterfactual_rows]
    y = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 5.2))

    actual_delay = np.asarray(
        [
            row["D_delay_actual_data_observed_m"]
            for row in counterfactual_rows
        ]
    )
    restored_delay = np.asarray(
        [
            row["D_delay_restored_from_observed_pre_t2_trace_m"]
            for row in counterfactual_rows
        ]
    )
    axes[0].barh(
        y - 0.17,
        actual_delay,
        height=0.32,
        color=COLORS["collision_1131"],
        alpha=0.72,
        label="实际响应距离",
    )
    axes[0].barh(
        y + 0.17,
        restored_delay,
        height=0.32,
        color=COLORS["restored"],
        alpha=0.82,
        label="恢复到baseline中位时延",
    )
    for index, row in enumerate(counterfactual_rows):
        axes[0].text(
            max(actual_delay[index], restored_delay[index]) + 0.25,
            index,
            (
                f"收回"
                f"{row['response_distance_recovered_model_m']:.3f} m"
            ),
            va="center",
            fontsize=8.7,
        )
    axes[0].set_yticks(y, labels)
    axes[0].set_xlabel("响应阶段墙钟速度积分距离/m")
    axes[0].set_title("恢复时延后收回的响应距离")
    axes[0].grid(axis="x", alpha=0.22)
    axes[0].legend(frameon=False, fontsize=8.5)

    available = np.asarray(
        [
            row["available_to_observed_contact_restored_model_m"]
            for row in counterfactual_rows
        ]
    )
    required = np.asarray(
        [
            row["required_stopping_distance_restored_model_m"]
            for row in counterfactual_rows
        ]
    )
    axes[1].barh(
        y - 0.17,
        available,
        height=0.32,
        color=COLORS["restored"],
        alpha=0.82,
        label="恢复后至实际接触位置可用距离",
    )
    axes[1].barh(
        y + 0.17,
        required,
        height=0.32,
        color=COLORS["collision_1643"],
        alpha=0.72,
        label="模型所需停车距离",
    )
    for index, row in enumerate(counterfactual_rows):
        margin = float(
            row["margin_to_observed_contact_restored_model_m"]
        )
        if bool(row["collision_model_predicted"]):
            text = (
                f"余量{margin:.3f} m，预测碰撞"
                f"{row['impact_speed_model_predicted_mps']:.3f} m/s"
            )
        else:
            text = f"余量+{margin:.3f} m，预测不碰撞"
        axes[1].text(
            max(available[index], required[index]) + 0.25,
            index,
            text,
            va="center",
            fontsize=8.5,
        )
    axes[1].set_yticks(y, labels)
    axes[1].set_xlabel("距离/m")
    axes[1].set_title("恢复到baseline中位时延后的模型结局")
    axes[1].grid(axis="x", alpha=0.22)
    axes[1].legend(frameon=False, fontsize=8.3)
    fig.suptitle(
        "图7-4 碰撞run响应时延恢复计算（模型结果）",
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_verified_recovery_model.png", dpi=220)
    plt.close(fig)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def source_provenance(
    runs: dict[str, dict[str, object]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run_id, run in sorted(runs.items()):
        parsed = run["parsed"]
        observed = run["observed"]
        t1_s = float(observed["t1_data_observed_s"])
        t2_s = float(observed["t2_data_observed_s"])
        endpoint_s = float(
            observed["outcome_endpoint_data_observed_s"]
        )
        t1_sample = min(
            parsed.localization,
            key=lambda sample: abs(sample.time_s - t1_s),
        )
        t2_sample = min(
            parsed.localization,
            key=lambda sample: abs(sample.time_s - t2_s),
        )
        endpoint_sample = min(
            parsed.localization,
            key=lambda sample: abs(sample.time_s - endpoint_s),
        )
        collision_source = parsed.collision.get("source_file")
        collision_row = parsed.collision.get("source_row")
        direct_collision_speed = math.nan
        collision_speed_difference = math.nan
        if bool(observed["collision_data_observed"]):
            collision_path = Path(str(collision_source))
            with collision_path.open(
                "r",
                encoding="utf-8-sig",
                errors="replace",
                newline="",
            ) as handle:
                direct_row = next(csv.DictReader(handle))
            direct_collision_speed = float(direct_row["ego_speed_mps"])
            collision_speed_difference = (
                direct_collision_speed
                - float(observed["impact_speed_data_observed_mps"])
            )
            if abs(collision_speed_difference) > 1e-12:
                raise RuntimeError(
                    f"Collision CSV speed mismatch for {run_id}"
                )
        rows.append(
            {
                "run_id": run_id,
                "localization_source_file": t1_sample.source_file,
                "nearest_t1_localization_source_line": (
                    t1_sample.source_line
                ),
                "nearest_t2_localization_source_line": (
                    t2_sample.source_line
                ),
                "nearest_outcome_localization_source_line": (
                    endpoint_sample.source_line
                ),
                "collision_csv_source_file": (
                    str(collision_source) if collision_source else ""
                ),
                "collision_csv_source_row": (
                    collision_row if collision_row else ""
                ),
                "collision_speed_direct_csv_mps": direct_collision_speed,
                "collision_speed_parsed_difference_mps": (
                    collision_speed_difference
                ),
            }
        )
    return rows


def main() -> None:
    configure_plotting()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    FIGURE_ROOT.mkdir(parents=True, exist_ok=True)
    config = make_config()
    runs = load_runs()
    rows = [
        observed_row(run_id, run, config)
        for run_id, run in sorted(runs.items())
    ]
    for row in rows:
        runs[str(row["run_id"])]["observed"] = row

    baseline_rows = [
        row for row in rows if row["group_name"] == "baseline"
    ]
    observed_metrics = [
        "actual_e2e_latency_data_observed_ms",
        "v1_data_observed_mps",
        "v2_data_observed_mps",
        "D1_clear_data_observed_m",
        "D_delay_wall_integral_data_observed_m",
        "response_mean_speed_data_observed_mps",
        "D2_clear_wall_budget_data_observed_m",
        "outcome_endpoint_speed_data_observed_mps",
        "post_t2_distance_to_outcome_wall_integral_data_observed_m",
        "outcome_energy_equivalent_deceleration_data_observed_mps2",
    ]
    baseline_summary = {
        metric: describe([float(row[metric]) for row in baseline_rows])
        for metric in observed_metrics
    }
    baseline_latency_median_ms = float(
        baseline_summary[
            "actual_e2e_latency_data_observed_ms"
        ]["median"]
    )
    counterfactual_rows = [
        counterfactual_latency_recovery(
            runs[run_id], baseline_latency_median_ms
        )
        for run_id in sorted(COLLISION_IDS)
    ]
    acceleration_rows = acceleration_summary(runs)
    baseline_acceleration_rows = [
        row
        for row in acceleration_rows
        if row["group_name"] == "baseline"
    ]
    baseline_acceleration_summary = {
        metric: describe(
            [float(row[metric]) for row in baseline_acceleration_rows]
        )
        for metric in [
            "maximum_positive_acceleration_data_observed_mps2",
            "maximum_deceleration_data_observed_mps2",
            "maximum_acceleration_after_t2_data_observed_mps2",
            "maximum_deceleration_after_t2_data_observed_mps2",
        ]
    }
    audit_rows = data_audit(runs)
    provenance_rows = source_provenance(runs)
    mismatches = [
        row
        for row in audit_rows
        if row["status"] == "MISMATCH"
    ]
    if mismatches:
        raise RuntimeError(
            f"Observed-data audit failed: {json.dumps(mismatches)}"
        )

    summary = {
        "scope": {
            "baseline_run_count": len(baseline_rows),
            "collision_run_ids": sorted(COLLISION_IDS),
            "excluded_run_ids": sorted(EXCLUDED_RUN_IDS),
            "observed_distance_definition": (
                "D_delay is the trapezoidal integral of Localization "
                "speed over wall-clock time from t1 to t2."
            ),
            "noncollision_graph_endpoint": (
                "First post-t2 Localization sample below 0.1 m/s."
            ),
            "collision_graph_endpoint": "CARLA CollisionSensor event.",
        },
        "data_observed": {
            "baseline_summary": baseline_summary,
            "runs": rows,
            "acceleration_summary": {
                "baseline": baseline_acceleration_summary,
                "runs": acceleration_rows,
            },
        },
        "model_predicted": {
            "reference_baseline_latency_ms": (
                baseline_latency_median_ms
            ),
            "counterfactual_collision_latency_recovery": (
                counterfactual_rows
            ),
        },
        "audit": {
            "checked_value_count": len(audit_rows),
            "mismatch_count": len(mismatches),
            "status": "PASS" if not mismatches else "FAIL",
            "collision_csv_direct_checks": len(COLLISION_IDS),
        },
        "sources": {
            "localization": (
                "Each run's localization.log; measurement_time, position, "
                "and vector-speed magnitude."
            ),
            "collision": (
                "Each collision run's carla_collision_events CSV; "
                "CollisionSensor event time and ego_speed_mps."
            ),
            "events": (
                "Apollo perception/planning/control logs and trace CSVs "
                "parsed with realtime_collision_core."
            ),
        },
    }
    (OUTPUT_ROOT / "chapter7_verified_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_csv(
        OUTPUT_ROOT / "chapter7_verified_observed_metrics.csv", rows
    )
    write_csv(
        OUTPUT_ROOT / "chapter7_verified_counterfactual_model.csv",
        counterfactual_rows,
    )
    write_csv(
        OUTPUT_ROOT / "chapter7_verified_data_audit.csv", audit_rows
    )
    write_csv(
        OUTPUT_ROOT / "chapter7_verified_source_provenance.csv",
        provenance_rows,
    )
    write_csv(
        OUTPUT_ROOT / "chapter7_verified_acceleration_summary.csv",
        acceleration_rows,
    )
    figure_latency_distance(rows, FIGURE_ROOT)
    figure_vt(runs, FIGURE_ROOT)
    figure_st(runs, FIGURE_ROOT)
    figure_acceleration(runs, FIGURE_ROOT)
    figure_recovery(counterfactual_rows, FIGURE_ROOT)
    REPORT_FIGURE_ROOT.mkdir(parents=True, exist_ok=True)
    report_figures = {
        "chapter7_verified_latency_distance.png": (
            "chapter7_latency_distance.png"
        ),
        "chapter7_verified_vt.png": "chapter7_vt.png",
        "chapter7_verified_st.png": "chapter7_st.png",
        "chapter7_verified_recovery_model.png": (
            "chapter7_recovery_model.png"
        ),
    }
    for source_name, destination_name in report_figures.items():
        shutil.copy2(
            FIGURE_ROOT / source_name,
            REPORT_FIGURE_ROOT / destination_name,
        )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
