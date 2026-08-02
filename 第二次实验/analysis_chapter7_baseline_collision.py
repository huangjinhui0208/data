#!/usr/bin/env python3
"""Prepare observed-only Chapter 7 baseline-versus-collision evidence.

The script does not edit the report. It computes every run with the established
t1/t2 and wall-clock trapezoidal-integration definitions, then creates draft
figures for review.
"""

from __future__ import annotations

import csv
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
FIGURE_ROOT = OUTPUT_ROOT / "chapter7_draft_figures"
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


def endpoint_s(run: dict[str, object]) -> float:
    raw = run["raw"]
    if bool(raw["collision"]):
        return float(raw["t_collision_s"])
    return float(raw["t_minimum_speed_s"])


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


def observed_row(run_id: str, run: dict[str, object]) -> dict[str, object]:
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
        make_config()["geometry"]["combined_center_to_surface_offset_m"]
    )
    d1_clear_m = interpolated_clearance(stable, state_t1, offset_m)
    d_delay_m = core.integrate_speed(parsed.localization, t1_s, t2_s)
    d2_clear_m = d1_clear_m - d_delay_m
    return {
        "run_id": run_id,
        "display_name": DISPLAY_NAMES.get(run_id, run_id),
        "group_name": run["spec"].group_name,
        "collision": bool(raw["collision"]),
        "actual_e2e_latency_ms": (t2_s - t1_s) * 1000.0,
        "v1_mps": float(state_t1["speed_mps"]),
        "v2_mps": float(state_t2["speed_mps"]),
        "D1_clear_m": d1_clear_m,
        "D_delay_wall_integral_m": d_delay_m,
        "response_mean_speed_mps": d_delay_m / (t2_s - t1_s),
        "D2_clear_wall_budget_m": d2_clear_m,
        "impact_speed_mps": (
            float(raw["impact_speed_mps"])
            if bool(raw["collision"])
            else math.nan
        ),
        "t1_s": t1_s,
        "t2_s": t2_s,
        "endpoint_s": endpoint_s(run),
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
    replace_collision_endpoint_speed: bool,
) -> tuple[np.ndarray, np.ndarray]:
    parsed = run["parsed"]
    raw = run["raw"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = endpoint_s(run)
    start = core.interpolate_sample(parsed.localization, start_s)
    if start is None:
        raise RuntimeError("Missing Localization at t1")
    times = [0.0]
    speeds = [float(start["speed_mps"])]
    for sample in parsed.localization:
        if start_s < sample.time_s < end_s:
            times.append(sample.time_s - start_s)
            speeds.append(sample.speed_mps)
    end = core.interpolate_sample(parsed.localization, end_s)
    if end is None:
        raise RuntimeError("Missing Localization at endpoint")
    times.append(end_s - start_s)
    speeds.append(float(end["speed_mps"]))
    if bool(raw["collision"]) and replace_collision_endpoint_speed:
        speeds[-1] = float(raw["impact_speed_mps"])
    return np.asarray(times), np.asarray(speeds)


def station_trajectory(
    run: dict[str, object],
) -> tuple[np.ndarray, np.ndarray]:
    parsed = run["parsed"]
    raw = run["raw"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = endpoint_s(run)
    d1_m = float(run["observed"]["D1_clear_m"])
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
        absolute_s = float(observed["t1_s"]) + elapsed_s
        sample = core.interpolate_sample(run["parsed"].localization, absolute_s)
        if sample is None:
            continue
        speeds.append(float(sample["speed_mps"]))
        stations.append(
            -float(observed["D1_clear_m"])
            + core.integrate_speed(
                run["parsed"].localization,
                float(observed["t1_s"]),
                absolute_s,
            )
        )
    return float(np.median(speeds)), float(np.median(stations))


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

    for row in rows:
        if row["group_name"] == "baseline":
            key = "baseline"
        elif row["run_id"] in COLLISION_IDS:
            key = str(row["run_id"])
        else:
            key = "delayed_safe"
        style = styles[key]
        for ax, y_key in zip(
            axes,
            ["D_delay_wall_integral_m", "D2_clear_wall_budget_m"],
        ):
            ax.scatter(
                [row["actual_e2e_latency_ms"]],
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
            for ax, y_key in zip(
                axes,
                ["D_delay_wall_integral_m", "D2_clear_wall_budget_m"],
            ):
                ax.annotate(
                    DISPLAY_NAMES[str(row["run_id"])],
                    (
                        row["actual_e2e_latency_ms"],
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
    axes[0].set_ylabel("响应阶段实际行驶距离/m")
    axes[0].set_title("时延增加转化为距离债务")
    axes[1].set_ylabel("$t_2$时剩余净距/m")
    axes[1].set_title("时延增加压缩有效制动空间")
    fig.suptitle("图7-1 端到端响应时间与车辆空间状态", y=1.02)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_draft_latency_distance.png", dpi=220)
    plt.close(fig)


def figure_vt(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    baseline_ids = [
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    ]
    fig, ax = plt.subplots(figsize=(11.6, 6.2))
    for index, run_id in enumerate(sorted(baseline_ids)):
        times, speeds = trajectory(
            runs[run_id], replace_collision_endpoint_speed=False
        )
        ax.plot(
            times,
            speeds,
            color=COLORS["baseline"],
            alpha=0.42,
            linewidth=1.15,
            label="baseline实际轨迹（7个run）" if index == 0 else None,
        )
        raw = runs[run_id]["raw"]
        observed = runs[run_id]["observed"]
        ax.scatter(
            [
                float(observed["actual_e2e_latency_ms"]) / 1000.0
            ],
            [float(observed["v2_mps"])],
            s=22,
            facecolor="white",
            edgecolor=COLORS["baseline"],
            linewidth=0.9,
            zorder=4,
        )

    baseline_end = max(
        endpoint_s(runs[run_id])
        - float(runs[run_id]["raw"]["t_sensor_origin_s"])
        for run_id in baseline_ids
    )
    grid = np.arange(0.0, baseline_end + 0.0001, 0.01)
    matrix = np.full((len(baseline_ids), grid.size), np.nan)
    for index, run_id in enumerate(sorted(baseline_ids)):
        run = runs[run_id]
        start_s = float(run["raw"]["t_sensor_origin_s"])
        end_elapsed = endpoint_s(run) - start_s
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
        label="baseline中位速度轨迹",
    )

    for run_id, color in [
        ("202607271131", COLORS["collision_1131"]),
        ("202607271643", COLORS["collision_1643"]),
    ]:
        run = runs[run_id]
        times, speeds = trajectory(
            run, replace_collision_endpoint_speed=True
        )
        ax.plot(
            times,
            speeds,
            color=color,
            linewidth=2.5,
            label=DISPLAY_NAMES[run_id],
        )
        raw = run["raw"]
        observed = run["observed"]
        t2_elapsed = float(observed["actual_e2e_latency_ms"]) / 1000.0
        ax.scatter(
            [t2_elapsed],
            [float(observed["v2_mps"])],
            s=78,
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=6,
        )
        collision_elapsed = (
            float(raw["t_collision_s"]) - float(raw["t_sensor_origin_s"])
        )
        impact_speed = float(raw["impact_speed_mps"])
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
            (
                f"{DISPLAY_NAMES[run_id]}\n"
                f"碰撞速度{impact_speed:.3f} m/s"
            ),
            (collision_elapsed, impact_speed),
            xytext=(-10, 13),
            textcoords="offset points",
            ha="right",
            fontsize=8.7,
            color="#333333",
        )

    for run_id, offset in [
        ("202607271643", (10, -32)),
        ("202607271131", (10, 12)),
    ]:
        run = runs[run_id]
        raw = run["raw"]
        collision_elapsed = (
            float(raw["t_collision_s"]) - float(raw["t_sensor_origin_s"])
        )
        median_speed, _ = baseline_state_at_elapsed(
            runs, baseline_ids, collision_elapsed
        )
        ax.scatter(
            [collision_elapsed],
            [median_speed],
            marker="D",
            s=58,
            color=COLORS["baseline_median"],
            edgecolor="white",
            linewidth=0.8,
            zorder=7,
        )
        ax.annotate(
            (
                f"同一时刻baseline中位速度\n"
                f"{median_speed:.3f} m/s"
            ),
            (collision_elapsed, median_speed),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.4,
            color="#333333",
        )

    baseline_latencies = [
        float(runs[run_id]["observed"]["actual_e2e_latency_ms"])
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
    ax.set_xlim(0, 3.65)
    ax.set_ylim(0, 19.2)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("实际车速/(m/s)")
    ax.set_title("图7-2 VT轨迹：响应时间推迟使速度下降整体后移")
    ax.grid(alpha=0.22)
    ax.legend(loc="upper right", frameon=False, fontsize=8.6)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_draft_vt.png", dpi=220)
    plt.close(fig)


def figure_st(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    baseline_ids = [
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    ]
    fig, ax = plt.subplots(figsize=(11.6, 6.5))
    ax.axhline(
        0.0,
        color=COLORS["boundary"],
        linewidth=1.5,
        linestyle="--",
        label="统一障碍物分析边界 $S=0$",
    )
    for index, run_id in enumerate(sorted(baseline_ids)):
        times, stations = station_trajectory(runs[run_id])
        ax.plot(
            times,
            stations,
            color=COLORS["baseline"],
            alpha=0.45,
            linewidth=1.2,
            label="baseline实际ST轨迹（7个run）" if index == 0 else None,
        )
        raw = runs[run_id]["raw"]
        observed = runs[run_id]["observed"]
        t2_elapsed = float(observed["actual_e2e_latency_ms"]) / 1000.0
        t2_station = -float(observed["D2_clear_wall_budget_m"])
        ax.scatter(
            [t2_elapsed],
            [t2_station],
            s=23,
            facecolor="white",
            edgecolor=COLORS["baseline"],
            linewidth=0.9,
            zorder=4,
        )

    for run_id, color in [
        ("202607271131", COLORS["collision_1131"]),
        ("202607271643", COLORS["collision_1643"]),
    ]:
        run = runs[run_id]
        times, stations = station_trajectory(run)
        ax.plot(
            times,
            stations,
            color=color,
            linewidth=2.6,
            label=DISPLAY_NAMES[run_id],
        )
        raw = run["raw"]
        observed = run["observed"]
        t2_elapsed = float(observed["actual_e2e_latency_ms"]) / 1000.0
        t2_station = -float(observed["D2_clear_wall_budget_m"])
        ax.scatter(
            [t2_elapsed],
            [t2_station],
            s=78,
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=6,
        )
        endpoint_station = float(stations[-1])
        endpoint_elapsed = float(times[-1])
        ax.scatter(
            [endpoint_elapsed],
            [endpoint_station],
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
            color="#333333",
        )

    for run_id, offset in [
        ("202607271643", (10, -31)),
        ("202607271131", (10, 11)),
    ]:
        run = runs[run_id]
        raw = run["raw"]
        collision_elapsed = (
            float(raw["t_collision_s"]) - float(raw["t_sensor_origin_s"])
        )
        _, median_station = baseline_state_at_elapsed(
            runs, baseline_ids, collision_elapsed
        )
        ax.scatter(
            [collision_elapsed],
            [median_station],
            marker="D",
            s=58,
            color=COLORS["baseline_median"],
            edgecolor="white",
            linewidth=0.8,
            zorder=7,
        )
        ax.annotate(
            (
                f"同一时刻baseline中位位置\n"
                f"$S={median_station:.3f}$ m"
            ),
            (collision_elapsed, median_station),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.4,
            color="#333333",
        )

    ax.set_xlim(0, 3.65)
    ax.set_ylim(-42, 4.0)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("统一纵向位置 $S$/m")
    ax.set_title("图7-3 ST轨迹：响应距离债务压缩碰撞前空间")
    ax.grid(alpha=0.22)
    ax.legend(loc="lower right", frameon=False, fontsize=8.6)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter7_draft_st.png", dpi=220)
    plt.close(fig)


def main() -> None:
    configure_plotting()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    FIGURE_ROOT.mkdir(parents=True, exist_ok=True)
    runs = load_runs()
    rows = [
        observed_row(run_id, run)
        for run_id, run in sorted(runs.items())
    ]
    for row in rows:
        runs[str(row["run_id"])]["observed"] = row
    baseline_rows = [
        row for row in rows if row["group_name"] == "baseline"
    ]
    metrics = [
        "actual_e2e_latency_ms",
        "v1_mps",
        "v2_mps",
        "D1_clear_m",
        "D_delay_wall_integral_m",
        "response_mean_speed_mps",
        "D2_clear_wall_budget_m",
    ]
    baseline_summary = {
        metric: describe([float(row[metric]) for row in baseline_rows])
        for metric in metrics
    }
    collision_rows = {
        str(row["run_id"]): row
        for row in rows
        if row["run_id"] in COLLISION_IDS
    }
    collision_vs_baseline_median = {}
    for run_id, row in collision_rows.items():
        collision_vs_baseline_median[run_id] = {
            metric: float(row[metric])
            - baseline_summary[metric]["median"]
            for metric in metrics
        }
    summary = {
        "scope": {
            "baseline_run_count": len(baseline_rows),
            "collision_run_ids": sorted(COLLISION_IDS),
            "excluded_run_ids": sorted(EXCLUDED_RUN_IDS),
            "distance_definition": (
                "D_delay is the trapezoidal integral of Localization speed "
                "over wall-clock time from t1 to t2."
            ),
        },
        "baseline_summary": baseline_summary,
        "collision_rows": collision_rows,
        "collision_minus_baseline_median": collision_vs_baseline_median,
    }
    (OUTPUT_ROOT / "chapter7_baseline_collision_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    csv_path = OUTPUT_ROOT / "chapter7_baseline_collision_metrics.csv"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    figure_latency_distance(rows, FIGURE_ROOT)
    figure_vt(runs, FIGURE_ROOT)
    figure_st(runs, FIGURE_ROOT)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
