#!/usr/bin/env python3
"""Generate Chapter 6 figures from runs 202607271643 and 202607271211."""

from __future__ import annotations

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

import diagnose_202607271131_vs_202607271211 as diagnostic
import realtime_collision_core as core


RUN_IDS = ("202607271643", "202607271211")
COLORS = {
    "1643": "#D55E00",
    "1211": "#0072B2",
    "fusion": "#4C78A8",
    "prediction": "#72B7B2",
    "planning": "#59A14F",
    "control": "#EDC948",
    "physical": "#E15759",
    "delay_distance": "#F28E2B",
    "remaining_distance": "#4E79A7",
    "freshness": "#B279A2",
}


def configure_plotting() -> None:
    font_path = Path(r"C:\Windows\Fonts\msyh.ttc")
    if font_path.exists():
        fm.fontManager.addfont(str(font_path))
        family = fm.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams["font.family"] = family
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.titleweight": "normal",
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


def load_runs() -> dict[str, dict[str, object]]:
    config = diagnostic.make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    results: dict[str, dict[str, object]] = {}
    for run_spec in core.discover_runs(config):
        if run_spec.run_id not in RUN_IDS:
            continue
        parsed = core.parse_run(run_spec, config, timezone)
        raw, debug = core.raw_run_metrics(parsed, config)
        results[run_spec.run_id] = {
            "parsed": parsed,
            "raw": raw,
            "debug": debug,
        }
    missing = set(RUN_IDS) - set(results)
    if missing:
        raise RuntimeError(f"Missing runs: {sorted(missing)}")
    return results


def add_center_labels(
    ax: plt.Axes,
    bars: list[matplotlib.patches.Rectangle],
    values: list[float],
    fmt: str = "{:.3f}",
) -> None:
    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_y() + bar.get_height() / 2,
            fmt.format(value),
            ha="center",
            va="center",
            color="white",
            fontsize=9,
        )


def figure_explicit_latency(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    values: dict[str, dict[str, float]] = {}
    for run_id in RUN_IDS:
        raw = runs[run_id]["raw"]
        values[run_id] = {
            "sensor_control": float(raw["sensor_to_control_ms"]),
            "control_t2": float(raw["control_to_effective_brake_ms"]),
            "sensor_fusion": float(raw["sensor_to_perception_ms"]),
            "fusion_prediction": float(raw["perception_to_prediction_ms"]),
            "prediction_planning": float(
                raw["prediction_to_planning_stop_ms"]
            ),
            "planning_control": float(raw["planning_stop_to_control_ms"]),
            "total": float(raw["actual_e2e_latency_ms"]),
            "scb": float(raw["scb_actual_wall_delay_ms"]),
        }

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.3))
    y = np.arange(2)
    labels = ["碰撞_1643", "未碰撞_1211"]

    first = [values[run_id]["sensor_control"] for run_id in RUN_IDS]
    second = [values[run_id]["control_t2"] for run_id in RUN_IDS]
    first_bars = axes[0].barh(
        y,
        first,
        color=COLORS["fusion"],
        height=0.54,
        label="$t_1$至Control",
    )
    second_bars = axes[0].barh(
        y,
        second,
        left=first,
        color=COLORS["physical"],
        height=0.54,
        label="Control至$t_2$",
    )
    add_center_labels(axes[0], list(first_bars), first)
    add_center_labels(axes[0], list(second_bars), second)
    for index, run_id in enumerate(RUN_IDS):
        axes[0].text(
            values[run_id]["total"] + 10,
            index,
            f"总计 {values[run_id]['total']:.3f} ms",
            va="center",
            fontsize=9,
        )
        start = values[run_id]["sensor_control"]
        axes[0].plot(
            [start, start + values[run_id]["scb"]],
            [index - 0.35, index - 0.35],
            color="#333333",
            linewidth=2,
        )
        axes[0].text(
            start + values[run_id]["scb"] / 2,
            index - 0.43,
            f"SCB {values[run_id]['scb']:.3f} ms",
            ha="center",
            va="top",
            fontsize=8,
        )
    axes[0].set_yticks(y, labels)
    axes[0].invert_yaxis()
    axes[0].set_xlim(0, 1030)
    axes[0].set_xlabel("墙钟时间/ms")
    axes[0].set_title("完整显式响应")
    axes[0].grid(axis="x", alpha=0.25)
    axes[0].legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=2,
        frameon=False,
    )

    stage_names = [
        "$t_1$→Fusion",
        "Fusion→Prediction",
        "Prediction→Planning",
        "Planning→Control",
    ]
    stage_keys = [
        "sensor_fusion",
        "fusion_prediction",
        "prediction_planning",
        "planning_control",
    ]
    stage_colors = [
        COLORS["fusion"],
        COLORS["prediction"],
        COLORS["planning"],
        COLORS["control"],
    ]
    left = np.zeros(2)
    for name, key, color in zip(stage_names, stage_keys, stage_colors):
        segment = np.asarray([values[run_id][key] for run_id in RUN_IDS])
        axes[1].barh(
            y,
            segment,
            left=left,
            color=color,
            height=0.54,
            label=name,
        )
        left += segment
    for index, run_id in enumerate(RUN_IDS):
        axes[1].text(
            values[run_id]["sensor_control"] + 4,
            index,
            f"{values[run_id]['sensor_control']:.3f} ms",
            va="center",
            fontsize=9,
        )
    axes[1].set_yticks(y, labels)
    axes[1].invert_yaxis()
    axes[1].set_xlim(0, 395)
    axes[1].set_xlabel("墙钟时间/ms")
    axes[1].set_title("$t_1$至Control内部阶段")
    axes[1].grid(axis="x", alpha=0.25)
    axes[1].legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=2,
        frameon=False,
        fontsize=8,
    )

    fig.suptitle(
        "图6-1 碰撞_1643的显式响应在前段和后段均更长",
        y=1.02,
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(output_dir / "chapter6_explicit_latency.png", dpi=220)
    plt.close(fig)


def figure_distance_budget(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    d1: list[float] = []
    d_delay: list[float] = []
    d2: list[float] = []
    v2: list[float] = []
    t_budget_ms: list[float] = []
    for run_id in RUN_IDS:
        raw = runs[run_id]["raw"]
        d1_value = float(raw["D1_clear_m"])
        delay_value = float(raw["D_delay_m"])
        d2_value = d1_value - delay_value
        v2_value = float(raw["brake_start_speed_mps"])
        d1.append(d1_value)
        d_delay.append(delay_value)
        d2.append(d2_value)
        v2.append(v2_value)
        t_budget_ms.append(1000 * d2_value / v2_value)

    fig, ax = plt.subplots(figsize=(10.9, 5.1))
    y = np.arange(2)
    labels = ["碰撞_1643", "未碰撞_1211"]
    delay_bars = ax.barh(
        y,
        d_delay,
        color=COLORS["delay_distance"],
        height=0.55,
        label="响应阶段距离 $D_{delay}$",
    )
    remaining_bars = ax.barh(
        y,
        d2,
        left=d_delay,
        color=COLORS["remaining_distance"],
        height=0.55,
        label="$t_2$剩余净距 $D_2$",
    )
    add_center_labels(ax, list(delay_bars), d_delay)
    add_center_labels(ax, list(remaining_bars), d2)
    for index in range(2):
        ax.text(
            d1[index] + 0.35,
            index,
            (
                f"$D_1$={d1[index]:.3f} m\n"
                f"$v_2$={v2[index]:.3f} m/s，"
                f"$T_{{budget,2}}$={t_budget_ms[index]:.1f} ms"
            ),
            va="center",
            fontsize=9,
        )
    ax.annotate(
        "$t_2$剩余净距多7.002 m\n等效时间预算多472.6 ms",
        xy=(d_delay[1] + d2[1] - 2.5, 1),
        xytext=(29.0, 0.50),
        arrowprops={"arrowstyle": "->", "color": "#333333"},
        ha="center",
        va="center",
        fontsize=9,
    )
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlim(0, 49)
    ax.set_xlabel("纵向距离/m")
    ax.set_title(
        "图6-2 更长响应与更小初始净距共同压缩制动预算",
        pad=14,
    )
    ax.grid(axis="x", alpha=0.25)
    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.14),
        ncol=2,
        frameon=False,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(output_dir / "chapter6_distance_budget.png", dpi=220)
    plt.close(fig)


def localization_curve(
    parsed: core.ParsedRun,
    start_s: float,
    end_s: float,
    append_endpoint_speed: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    start = core.interpolate_sample(parsed.localization, start_s)
    if start is None:
        raise RuntimeError("Cannot interpolate t2 localization state")
    times = [0.0]
    speeds = [float(start["speed_mps"])]
    for row in parsed.localization:
        if start_s < row.time_s <= end_s:
            times.append(row.time_s - start_s)
            speeds.append(row.speed_mps)
    if append_endpoint_speed is not None:
        elapsed = end_s - start_s
        if elapsed > times[-1] + 1e-9:
            times.append(elapsed)
            speeds.append(append_endpoint_speed)
        else:
            times[-1] = elapsed
            speeds[-1] = append_endpoint_speed
    return np.asarray(times), np.asarray(speeds)


def figure_speed_curves(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    raw_collision = runs[RUN_IDS[0]]["raw"]
    parsed_collision = runs[RUN_IDS[0]]["parsed"]
    t2_collision = float(raw_collision["t_brake_effective_s"])
    collision_s = float(raw_collision["t_collision_s"])
    collision_elapsed = collision_s - t2_collision
    collision_speed = float(raw_collision["impact_speed_mps"])
    t_collision, v_collision = localization_curve(
        parsed_collision,
        t2_collision,
        collision_s,
        append_endpoint_speed=collision_speed,
    )

    raw_safe = runs[RUN_IDS[1]]["raw"]
    parsed_safe = runs[RUN_IDS[1]]["parsed"]
    t2_safe = float(raw_safe["t_brake_effective_s"])
    minimum_s = float(raw_safe["t_minimum_speed_s"])
    t_safe, v_safe = localization_curve(
        parsed_safe,
        t2_safe,
        minimum_s,
    )
    safe_at_collision_elapsed = core.interpolate_sample(
        parsed_safe.localization,
        t2_safe + collision_elapsed,
    )
    if safe_at_collision_elapsed is None:
        raise RuntimeError("Cannot interpolate matched 1211 speed")

    fig, ax = plt.subplots(figsize=(10.8, 5.6))
    ax.plot(
        t_collision,
        v_collision,
        color=COLORS["1643"],
        linewidth=2.2,
        marker="o",
        markersize=3.3,
        label="碰撞_1643",
    )
    ax.plot(
        t_safe,
        v_safe,
        color=COLORS["1211"],
        linewidth=2.2,
        marker="s",
        markersize=3.0,
        label="未碰撞_1211",
    )
    ax.axvline(
        collision_elapsed,
        color="#777777",
        linewidth=1.0,
        linestyle="--",
    )
    ax.scatter(
        [collision_elapsed],
        [collision_speed],
        color=COLORS["1643"],
        edgecolor="white",
        linewidth=1.0,
        s=75,
        zorder=5,
    )
    ax.annotate(
        f"碰撞_1643\n{collision_elapsed:.3f} s，{collision_speed:.3f} m/s",
        xy=(collision_elapsed, collision_speed),
        xytext=(1.86, 14.4),
        arrowprops={"arrowstyle": "->", "color": COLORS["1643"]},
        fontsize=9,
    )
    safe_matched_speed = float(safe_at_collision_elapsed["speed_mps"])
    ax.scatter(
        [collision_elapsed],
        [safe_matched_speed],
        color=COLORS["1211"],
        edgecolor="white",
        linewidth=1.0,
        s=70,
        zorder=5,
    )
    ax.annotate(
        f"同一时刻未碰撞_1211\n{safe_matched_speed:.3f} m/s，仍未碰撞",
        xy=(collision_elapsed, safe_matched_speed),
        xytext=(1.88, 7.3),
        arrowprops={"arrowstyle": "->", "color": COLORS["1211"]},
        fontsize=9,
    )
    near_stop_elapsed = float(raw_safe["t_near_stop_s"]) - t2_safe
    near_stop = core.interpolate_sample(
        parsed_safe.localization,
        float(raw_safe["t_near_stop_s"]),
    )
    near_stop_speed = float(near_stop["speed_mps"]) if near_stop else 0.0
    ax.scatter(
        [near_stop_elapsed],
        [near_stop_speed],
        color=COLORS["1211"],
        edgecolor="white",
        linewidth=1.0,
        s=75,
        zorder=5,
    )
    ax.annotate(
        f"未碰撞_1211近零速\n{near_stop_elapsed:.3f} s，{near_stop_speed:.3f} m/s",
        xy=(near_stop_elapsed, near_stop_speed),
        xytext=(2.02, 3.4),
        arrowprops={"arrowstyle": "->", "color": COLORS["1211"]},
        fontsize=9,
    )
    ax.axhline(12.0, color="#888888", linewidth=1.0, linestyle=":")
    ax.text(2.79, 12.15, "12 m/s", ha="right", va="bottom", fontsize=8)
    ax.set_xlim(0, 2.82)
    ax.set_ylim(0, 18.9)
    ax.set_xlabel("$t_2$后的墙钟时间/s")
    ax.set_ylabel("实际车速/(m/s)")
    ax.set_title("图6-3 $t_2$后实际速度变化与最终结局", pad=14)
    ax.grid(alpha=0.22)
    ax.legend(loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter6_speed_after_t2.png", dpi=220)
    plt.close(fig)


def normalized_st_curve(
    run: dict[str, object],
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    parsed = run["parsed"]
    raw = run["raw"]
    t1_s = float(raw["t_sensor_origin_s"])
    t2_s = float(raw["t_brake_effective_s"])
    endpoint_s = (
        float(raw["t_collision_s"])
        if bool(raw["collision"])
        else float(raw["t_minimum_speed_s"])
    )
    d1_m = float(raw["D1_clear_m"])
    times = [0.0]
    stations = [-d1_m]
    for row in parsed.localization:
        if t1_s < row.time_s < endpoint_s:
            times.append(row.time_s - t1_s)
            stations.append(
                -d1_m
                + core.integrate_speed(
                    parsed.localization,
                    t1_s,
                    row.time_s,
                )
            )
    endpoint_station_m = (
        -d1_m
        + core.integrate_speed(
            parsed.localization,
            t1_s,
            endpoint_s,
        )
    )
    times.append(endpoint_s - t1_s)
    stations.append(endpoint_station_m)
    key_points = {
        "t2_rel_s": t2_s - t1_s,
        "t2_station_m": -d1_m + float(raw["D_delay_m"]),
        "endpoint_rel_s": endpoint_s - t1_s,
        "endpoint_station_m": endpoint_station_m,
    }
    return np.asarray(times), np.asarray(stations), key_points


def figure_st_overlay(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    curves = {
        run_id: normalized_st_curve(runs[run_id])
        for run_id in RUN_IDS
    }
    fig, ax = plt.subplots(figsize=(10.9, 6.0))
    ax.axhspan(0, 4.5, color=COLORS["1643"], alpha=0.055)
    ax.axhline(
        0,
        color="#444444",
        linewidth=1.7,
        linestyle="--",
        label="统一障碍物参考边界  S=0",
    )

    for run_id, color, marker, label in [
        (
            RUN_IDS[0],
            COLORS["1643"],
            "o",
            "碰撞_1643",
        ),
        (
            RUN_IDS[1],
            COLORS["1211"],
            "s",
            "未碰撞_1211",
        ),
    ]:
        times, stations, points = curves[run_id]
        ax.plot(
            times,
            stations,
            color=color,
            linewidth=2.5,
            marker=marker,
            markersize=3.1,
            label=label,
        )
        ax.scatter(
            [points["t2_rel_s"]],
            [points["t2_station_m"]],
            s=85,
            marker=marker,
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=6,
        )
        t2_offset = (0.12, -4.6) if run_id == RUN_IDS[0] else (0.12, 1.2)
        ax.annotate(
            (
                f"$t_2$: {points['t2_rel_s']:.3f} s，"
                f"$S$={points['t2_station_m']:.3f} m"
            ),
            xy=(points["t2_rel_s"], points["t2_station_m"]),
            xytext=(
                points["t2_rel_s"] + t2_offset[0],
                points["t2_station_m"] + t2_offset[1],
            ),
            arrowprops={"arrowstyle": "->", "color": color},
            fontsize=9,
        )

    collision_points = curves[RUN_IDS[0]][2]
    safe_points = curves[RUN_IDS[1]][2]
    collision_speed = float(runs[RUN_IDS[0]]["raw"]["impact_speed_mps"])
    ax.scatter(
        [collision_points["endpoint_rel_s"]],
        [collision_points["endpoint_station_m"]],
        s=95,
        marker="X",
        color=COLORS["1643"],
        edgecolor="white",
        linewidth=1.0,
        zorder=7,
    )
    ax.annotate(
        (
            "碰撞_1643端点\n"
            f"{collision_points['endpoint_rel_s']:.3f} s，"
            f"$S$={collision_points['endpoint_station_m']:.3f} m\n"
            f"碰撞速度{collision_speed:.3f} m/s"
        ),
        xy=(
            collision_points["endpoint_rel_s"],
            collision_points["endpoint_station_m"],
        ),
        xytext=(2.05, -8.0),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1643"],
        },
        fontsize=9,
    )
    safe_min_speed = core.interpolate_sample(
        runs[RUN_IDS[1]]["parsed"].localization,
        float(runs[RUN_IDS[1]]["raw"]["t_minimum_speed_s"]),
    )
    safe_speed = (
        float(safe_min_speed["speed_mps"])
        if safe_min_speed is not None
        else 0.0
    )
    ax.scatter(
        [safe_points["endpoint_rel_s"]],
        [safe_points["endpoint_station_m"]],
        s=85,
        marker="P",
        color=COLORS["1211"],
        edgecolor="white",
        linewidth=1.0,
        zorder=7,
    )
    ax.annotate(
        (
            "未碰撞_1211停车端点\n"
            f"{safe_points['endpoint_rel_s']:.3f} s，"
            f"$S$={safe_points['endpoint_station_m']:.3f} m\n"
            f"速度{safe_speed:.3f} m/s"
        ),
        xy=(
            safe_points["endpoint_rel_s"],
            safe_points["endpoint_station_m"],
        ),
        xytext=(2.78, -9.8),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1211"],
        },
        fontsize=9,
    )
    ax.text(
        0.08,
        1.4,
        "$S>0$：越过统一参考边界",
        fontsize=9,
        color="#444444",
    )
    ax.set_xlim(-0.03, 3.55)
    ax.set_ylim(-42.5, 4.5)
    ax.set_xlabel("相对各自$t_1$的墙钟时间/s")
    ax.set_ylabel("统一空间中的纵向位置 $S$/m")
    ax.set_title(
        "图6-4 两组对齐至同一障碍物边界后的ST轨迹",
        pad=14,
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="lower right", frameon=False)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter6_st_overlay.png", dpi=220)
    plt.close(fig)


def target_metrics_to_matched_endpoint(
    run: dict[str, object],
    matched_elapsed_s: float,
) -> dict[str, float | np.ndarray]:
    parsed = run["parsed"]
    raw = run["raw"]
    t1_s = float(raw["t_sensor_origin_s"])
    t2_s = float(raw["t_brake_effective_s"])
    endpoint_s = t2_s + matched_elapsed_s
    all_rows = parsed.perception.get("target_rows", [])
    rows = [
        row
        for row in all_rows
        if t2_s - 0.6 <= row.header_time_s <= endpoint_s
    ]
    target_rows = [
        row
        for row in all_rows
        if t1_s <= row.header_time_s <= endpoint_s
    ]
    output_times = np.asarray(
        [row.header_time_s - t2_s for row in rows],
        dtype=float,
    )
    gaps_ms = np.diff(output_times) * 1000.0
    lifecycle_ms = np.asarray(
        [
            (row.header_time_s - row.obs_time_s) * 1000.0
            for row in target_rows
        ],
        dtype=float,
    )
    last = rows[-1]
    return {
        "output_times": output_times,
        "max_gap_ms": float(np.max(gaps_ms)),
        "lifecycle_median_ms": float(np.median(lifecycle_ms)),
        "lifecycle_p90_ms": float(np.percentile(lifecycle_ms, 90)),
        "lifecycle_max_ms": float(np.max(lifecycle_ms)),
        "last_output_rel_s": last.header_time_s - t2_s,
        "last_output_age_ms": (endpoint_s - last.header_time_s) * 1000.0,
        "last_source_age_ms": (endpoint_s - last.obs_time_s) * 1000.0,
    }


def figure_fusion_freshness(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    raw_collision = runs[RUN_IDS[0]]["raw"]
    matched_elapsed_s = float(raw_collision["time_braked_before_collision_s"])
    metrics = {
        run_id: target_metrics_to_matched_endpoint(
            runs[run_id],
            matched_elapsed_s,
        )
        for run_id in RUN_IDS
    }

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(12.0, 5.3),
        gridspec_kw={"width_ratios": [1.55, 1.0]},
    )
    y_positions = [1, 0]
    labels = ["碰撞_1643", "未碰撞_1211"]
    for y, run_id, color in zip(
        y_positions,
        RUN_IDS,
        [COLORS["1643"], COLORS["1211"]],
    ):
        output_times = metrics[run_id]["output_times"]
        axes[0].scatter(
            output_times,
            np.full_like(output_times, y, dtype=float),
            marker="|",
            s=330,
            linewidths=2.0,
            color=color,
        )
        tail_start = float(metrics[run_id]["last_output_rel_s"])
        axes[0].plot(
            [tail_start, matched_elapsed_s],
            [y + 0.17, y + 0.17],
            color=color,
            linewidth=3,
        )
        axes[0].text(
            (tail_start + matched_elapsed_s) / 2,
            y + 0.28,
            f"末次输出后{metrics[run_id]['last_output_age_ms']:.1f} ms",
            ha="center",
            va="bottom",
            fontsize=8.5,
        )
    axes[0].axvline(0, color="#555555", linestyle="--", linewidth=1.0)
    axes[0].axvline(
        matched_elapsed_s,
        color="#777777",
        linestyle=":",
        linewidth=1.2,
    )
    axes[0].text(0.02, -0.34, "$t_2$", ha="left", va="bottom", fontsize=9)
    axes[0].text(
        matched_elapsed_s - 0.02,
        -0.34,
        "碰撞_1643时刻",
        ha="right",
        va="bottom",
        fontsize=9,
    )
    axes[0].set_xlim(-0.62, 1.66)
    axes[0].set_ylim(-0.45, 1.58)
    axes[0].set_yticks(y_positions, labels)
    axes[0].set_xlabel("相对$t_2$的墙钟时间/s")
    axes[0].set_title("相同时间窗内的目标Fusion输出")
    axes[0].grid(axis="x", alpha=0.22)

    x = np.arange(2)
    width = 0.23
    max_gaps = [float(metrics[run_id]["max_gap_ms"]) for run_id in RUN_IDS]
    medians = [
        float(metrics[run_id]["lifecycle_median_ms"]) for run_id in RUN_IDS
    ]
    source_ages = [
        float(metrics[run_id]["last_source_age_ms"]) for run_id in RUN_IDS
    ]
    bars_gap = axes[1].bar(
        x - width,
        max_gaps,
        width,
        color=COLORS["fusion"],
        label="最大连续输出间隔",
    )
    bars_lifecycle = axes[1].bar(
        x,
        medians,
        width,
        color=COLORS["control"],
        label="目标生命周期中位数",
    )
    bars_source_age = axes[1].bar(
        x + width,
        source_ages,
        width,
        color=COLORS["freshness"],
        label="时间窗末源数据年龄",
    )
    for bars in (bars_gap, bars_lifecycle, bars_source_age):
        for bar in bars:
            axes[1].text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 10,
                f"{bar.get_height():.1f}",
                ha="center",
                va="bottom",
                fontsize=8.2,
            )
    axes[1].axhline(
        500,
        color="#666666",
        linestyle="--",
        linewidth=1.0,
        label="500 ms参考线",
    )
    axes[1].set_xticks(x, ["碰撞_1643", "未碰撞_1211"])
    axes[1].set_ylabel("墙钟时间/ms")
    axes[1].set_ylim(0, 680)
    axes[1].set_title("连续性与数据新鲜度")
    axes[1].grid(axis="y", alpha=0.22)
    axes[1].legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.13),
        frameon=False,
        fontsize=8,
    )

    fig.suptitle(
        "图6-5 碰撞_1643没有长间断，但碰撞前目标数据明显变旧",
        y=1.02,
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(output_dir / "chapter6_fusion_freshness.png", dpi=220)
    plt.close(fig)


def main() -> None:
    configure_plotting()
    output_dir = EXPERIMENT_ROOT / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    runs = load_runs()
    figure_explicit_latency(runs, output_dir)
    figure_distance_budget(runs, output_dir)
    figure_speed_curves(runs, output_dir)
    figure_st_overlay(runs, output_dir)
    figure_fusion_freshness(runs, output_dir)
    for path in sorted(output_dir.glob("chapter6_*.png")):
        print(path)


if __name__ == "__main__":
    main()
