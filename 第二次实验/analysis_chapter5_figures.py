#!/usr/bin/env python3
"""Generate Chapter 5 figures for 202607271131 and 202607271211."""

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


RUN_IDS = ("202607271131", "202607271211")
COLORS = {
    "1131": "#D55E00",
    "1211": "#0072B2",
    "fusion": "#4C78A8",
    "prediction": "#72B7B2",
    "planning": "#59A14F",
    "control": "#EDC948",
    "physical": "#E15759",
    "delay_distance": "#F28E2B",
    "remaining_distance": "#4E79A7",
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


def add_bar_labels(
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
    values = {}
    for run_id in RUN_IDS:
        raw = runs[run_id]["raw"]
        values[run_id] = {
            "sensor_control": float(raw["sensor_to_control_ms"]),
            "control_t2": float(raw["control_to_effective_brake_ms"]),
            "sensor_fusion": float(raw["sensor_to_perception_ms"]),
            "fusion_prediction": float(raw["perception_to_prediction_ms"]),
            "prediction_planning": float(raw["prediction_to_planning_stop_ms"]),
            "planning_control": float(raw["planning_stop_to_control_ms"]),
            "total": float(raw["actual_e2e_latency_ms"]),
            "scb": float(raw["scb_actual_wall_delay_ms"]),
        }

    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.2))
    y = np.arange(2)
    labels = ["碰撞_1131", "未碰撞_1211"]

    first = [values[run_id]["sensor_control"] for run_id in RUN_IDS]
    second = [values[run_id]["control_t2"] for run_id in RUN_IDS]
    bars_first = axes[0].barh(
        y,
        first,
        color=COLORS["fusion"],
        height=0.54,
        label="$t_1$至Control",
    )
    bars_second = axes[0].barh(
        y,
        second,
        left=first,
        color=COLORS["physical"],
        height=0.54,
        label="Control至$t_2$",
    )
    add_bar_labels(axes[0], list(bars_first), first)
    add_bar_labels(axes[0], list(bars_second), second)
    for idx, run_id in enumerate(RUN_IDS):
        axes[0].text(
            values[run_id]["total"] + 8,
            idx,
            f"总计 {values[run_id]['total']:.3f} ms",
            va="center",
            fontsize=9,
        )
        start = values[run_id]["sensor_control"]
        axes[0].plot(
            [start, start + values[run_id]["scb"]],
            [idx - 0.35, idx - 0.35],
            color="#333333",
            linewidth=2,
        )
        axes[0].text(
            start + values[run_id]["scb"] / 2,
            idx - 0.43,
            f"SCB {values[run_id]['scb']:.3f} ms",
            ha="center",
            va="top",
            fontsize=8,
        )
    axes[0].set_yticks(y, labels)
    axes[0].invert_yaxis()
    axes[0].set_xlim(0, 930)
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
    for idx, run_id in enumerate(RUN_IDS):
        axes[1].text(
            values[run_id]["sensor_control"] + 4,
            idx,
            f"{values[run_id]['sensor_control']:.3f} ms",
            va="center",
            fontsize=9,
        )
    axes[1].set_yticks(y, labels)
    axes[1].invert_yaxis()
    axes[1].set_xlim(0, 370)
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
        "图5-1 显式时延分解：碰撞run总响应更长",
        y=1.02,
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(output_dir / "chapter5_explicit_latency.png", dpi=220)
    plt.close(fig)


def figure_distance_budget(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    d1 = []
    d_delay = []
    d2 = []
    v2 = []
    t_budget_ms = []
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

    fig, ax = plt.subplots(figsize=(10.8, 5.0))
    y = np.arange(2)
    labels = ["碰撞_1131", "未碰撞_1211"]
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
    add_bar_labels(ax, list(delay_bars), d_delay)
    add_bar_labels(ax, list(remaining_bars), d2)
    for idx in range(2):
        ax.text(
            d1[idx] + 0.35,
            idx,
            (
                f"$D_1$={d1[idx]:.3f} m\n"
                f"$v_2$={v2[idx]:.3f} m/s，"
                f"$T_{{budget,2}}$={t_budget_ms[idx]:.1f} ms"
            ),
            va="center",
            fontsize=9,
        )
    d2_advantage = d2[1] - d2[0]
    time_advantage_ms = t_budget_ms[1] - t_budget_ms[0]
    ax.annotate(
        (
            f"$t_2$剩余净距多{d2_advantage:.3f} m\n"
            f"等效时间预算多{time_advantage_ms:.1f} ms"
        ),
        xy=(d_delay[1] + d2[1] - 1.0, 1),
        xytext=(29.5, 0.53),
        arrowprops={"arrowstyle": "->", "color": "#333333"},
        ha="center",
        va="center",
        fontsize=9,
    )
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlim(0, 50)
    ax.set_xlabel("纵向距离/m")
    ax.set_title("图5-2 相近显式时延对应不同的空间与时间预算", pad=14)
    ax.grid(axis="x", alpha=0.25)
    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.14),
        ncol=2,
        frameon=False,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(output_dir / "chapter5_distance_budget.png", dpi=220)
    plt.close(fig)


def localization_curve(
    parsed: core.ParsedRun,
    start_s: float,
    end_s: float,
    append_endpoint_speed: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    times = [0.0]
    start = core.interpolate_sample(parsed.localization, start_s)
    if start is None:
        raise RuntimeError("Cannot interpolate t2 localization state")
    speeds = [float(start["speed_mps"])]
    for row in parsed.localization:
        if start_s < row.time_s <= end_s:
            times.append(row.time_s - start_s)
            speeds.append(row.speed_mps)
    if append_endpoint_speed is not None:
        elapsed = end_s - start_s
        if not times or elapsed > times[-1] + 1e-9:
            times.append(elapsed)
            speeds.append(append_endpoint_speed)
        else:
            speeds[-1] = append_endpoint_speed
    return np.asarray(times), np.asarray(speeds)


def figure_speed_curves(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    raw_1131 = runs[RUN_IDS[0]]["raw"]
    parsed_1131 = runs[RUN_IDS[0]]["parsed"]
    t2_1131 = float(raw_1131["t_brake_effective_s"])
    collision_s = float(raw_1131["t_collision_s"])
    t_1131, v_1131 = localization_curve(
        parsed_1131,
        t2_1131,
        collision_s,
        append_endpoint_speed=float(raw_1131["impact_speed_mps"]),
    )

    raw_1211 = runs[RUN_IDS[1]]["raw"]
    parsed_1211 = runs[RUN_IDS[1]]["parsed"]
    t2_1211 = float(raw_1211["t_brake_effective_s"])
    minimum_s = float(raw_1211["t_minimum_speed_s"])
    t_1211, v_1211 = localization_curve(
        parsed_1211,
        t2_1211,
        minimum_s,
    )

    fig, ax = plt.subplots(figsize=(10.8, 5.6))
    ax.plot(
        t_1131,
        v_1131,
        color=COLORS["1131"],
        linewidth=2.2,
        marker="o",
        markersize=3.3,
        label="碰撞_1131",
    )
    ax.plot(
        t_1211,
        v_1211,
        color=COLORS["1211"],
        linewidth=2.2,
        marker="s",
        markersize=3.0,
        label="未碰撞_1211",
    )
    gap_start = 1.1923720836639404
    gap_end = 1.6998109817504883
    ax.axvspan(
        gap_start,
        gap_end,
        color=COLORS["1131"],
        alpha=0.10,
        label="碰撞_1131目标Fusion输出间断",
    )
    collision_elapsed = collision_s - t2_1131
    impact_speed = float(raw_1131["impact_speed_mps"])
    ax.scatter(
        [collision_elapsed],
        [impact_speed],
        color=COLORS["1131"],
        edgecolor="white",
        linewidth=1.0,
        s=75,
        zorder=5,
    )
    ax.annotate(
        f"碰撞\n{collision_elapsed:.3f} s，{impact_speed:.3f} m/s",
        xy=(collision_elapsed, impact_speed),
        xytext=(2.42, 10.8),
        arrowprops={"arrowstyle": "->", "color": COLORS["1131"]},
        fontsize=9,
    )
    near_stop_s = float(raw_1211["t_near_stop_s"])
    near_stop_elapsed = near_stop_s - t2_1211
    near_stop_state = core.interpolate_sample(parsed_1211.localization, near_stop_s)
    near_stop_speed = float(near_stop_state["speed_mps"])
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
        f"近零速\n{near_stop_elapsed:.3f} s，{near_stop_speed:.3f} m/s",
        xy=(near_stop_elapsed, near_stop_speed),
        xytext=(2.75, 3.0),
        arrowprops={"arrowstyle": "->", "color": COLORS["1211"]},
        fontsize=9,
    )
    ax.axhline(8.0, color="#777777", linewidth=1.0, linestyle="--")
    ax.text(2.95, 8.15, "8 m/s", ha="right", va="bottom", fontsize=8)
    ax.text(
        (gap_start + gap_end) / 2,
        17.7,
        "507.439 ms",
        ha="center",
        va="top",
        fontsize=9,
        color="#333333",
    )
    ax.set_xlim(0, 3.05)
    ax.set_ylim(0, 18.7)
    ax.set_xlabel("$t_2$后的墙钟时间/s")
    ax.set_ylabel("实际车速/(m/s)")
    ax.set_title("图5-3 $t_2$后实际速度变化与最终结局", pad=14)
    ax.grid(alpha=0.22)
    ax.legend(loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter5_speed_after_t2.png", dpi=220)
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
    ax.axhspan(0, 5.0, color=COLORS["1131"], alpha=0.055)
    ax.axhline(
        0,
        color="#444444",
        linewidth=1.7,
        linestyle="--",
        label="统一障碍物分析边界  S=0",
    )

    for run_id, color, marker, label in [
        (
            RUN_IDS[0],
            COLORS["1131"],
            "o",
            "碰撞_1131",
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

    collision_points = curves[RUN_IDS[0]][2]
    safe_points = curves[RUN_IDS[1]][2]
    ax.annotate(
        (
            f"碰撞_1131的$t_2$: {collision_points['t2_rel_s']:.3f} s\n"
            f"$S$={collision_points['t2_station_m']:.3f} m"
        ),
        xy=(
            collision_points["t2_rel_s"],
            collision_points["t2_station_m"],
        ),
        xytext=(1.05, -20.2),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1131"],
        },
        fontsize=9,
    )
    ax.annotate(
        (
            f"未碰撞_1211的$t_2$: {safe_points['t2_rel_s']:.3f} s\n"
            f"$S$={safe_points['t2_station_m']:.3f} m"
        ),
        xy=(
            safe_points["t2_rel_s"],
            safe_points["t2_station_m"],
        ),
        xytext=(0.23, -31.2),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1211"],
        },
        fontsize=9,
    )

    collision_speed = float(runs[RUN_IDS[0]]["raw"]["impact_speed_mps"])
    ax.scatter(
        [collision_points["endpoint_rel_s"]],
        [collision_points["endpoint_station_m"]],
        s=95,
        marker="X",
        color=COLORS["1131"],
        edgecolor="white",
        linewidth=1.0,
        zorder=7,
    )
    ax.annotate(
        (
            "碰撞_1131端点\n"
            f"{collision_points['endpoint_rel_s']:.3f} s，"
            f"$S$={collision_points['endpoint_station_m']:.3f} m\n"
            f"碰撞速度{collision_speed:.3f} m/s"
        ),
        xy=(
            collision_points["endpoint_rel_s"],
            collision_points["endpoint_station_m"],
        ),
        xytext=(1.92, -12.3),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1131"],
        },
        fontsize=9,
    )

    safe_raw = runs[RUN_IDS[1]]["raw"]
    safe_parsed = runs[RUN_IDS[1]]["parsed"]
    safe_t1_s = float(safe_raw["t_sensor_origin_s"])
    matched_s = safe_t1_s + collision_points["endpoint_rel_s"]
    matched_station_m = (
        -float(safe_raw["D1_clear_m"])
        + core.integrate_speed(
            safe_parsed.localization,
            safe_t1_s,
            matched_s,
        )
    )
    matched_state = core.interpolate_sample(
        safe_parsed.localization,
        matched_s,
    )
    matched_speed = (
        float(matched_state["speed_mps"])
        if matched_state is not None
        else float("nan")
    )
    ax.scatter(
        [collision_points["endpoint_rel_s"]],
        [matched_station_m],
        s=82,
        marker="s",
        facecolor="white",
        edgecolor=COLORS["1211"],
        linewidth=2.0,
        zorder=7,
    )
    ax.annotate(
        (
            "碰撞_1131的同一相对时刻\n"
            f"未碰撞_1211，$S$={matched_station_m:.3f} m\n"
            f"速度{matched_speed:.3f} m/s"
        ),
        xy=(collision_points["endpoint_rel_s"], matched_station_m),
        xytext=(3.15, -6.8),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1211"],
        },
        fontsize=9,
    )

    safe_minimum = core.interpolate_sample(
        safe_parsed.localization,
        float(safe_raw["t_minimum_speed_s"]),
    )
    safe_speed = (
        float(safe_minimum["speed_mps"])
        if safe_minimum is not None
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
            f"速度{safe_speed:.6f} m/s"
        ),
        xy=(
            safe_points["endpoint_rel_s"],
            safe_points["endpoint_station_m"],
        ),
        xytext=(3.73, -14.0),
        arrowprops={
            "arrowstyle": "->",
            "color": COLORS["1211"],
        },
        fontsize=9,
    )
    ax.text(
        0.10,
        1.5,
        "$S>0$：越过分析参考边界，不等同CARLA已碰撞",
        fontsize=9,
        color="#444444",
    )
    ax.set_xlim(-0.03, 4.30)
    ax.set_ylim(-42.5, 5.0)
    ax.set_xlabel("相对各自$t_1$的墙钟时间/s")
    ax.set_ylabel("统一空间中的纵向位置 $S$/m")
    ax.set_title(
        "图5-4 两组对齐至同一障碍物分析边界后的ST轨迹",
        pad=14,
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="lower right", frameon=False)
    fig.tight_layout()
    fig.savefig(output_dir / "chapter5_st_overlay.png", dpi=220)
    plt.close(fig)


def largest_target_gap(
    parsed: core.ParsedRun,
) -> tuple[float, float, float]:
    times = [
        row.header_time_s
        for row in parsed.perception.get("target_rows", [])
    ]
    gaps = [
        (times[index] - times[index - 1], times[index - 1], times[index])
        for index in range(1, len(times))
    ]
    return max(gaps)


def figure_fusion_continuity(
    runs: dict[str, dict[str, object]], output_dir: Path
) -> None:
    fig, axes = plt.subplots(
        1,
        2,
        figsize=(11.8, 5.2),
        gridspec_kw={"width_ratios": [1.55, 1.0]},
    )
    y_positions = [1, 0]
    labels = ["碰撞_1131", "未碰撞_1211"]
    gap_values = []
    lifecycle_values = []
    travel_values = []

    for y, run_id, color in zip(
        y_positions,
        RUN_IDS,
        [COLORS["1131"], COLORS["1211"]],
    ):
        parsed = runs[run_id]["parsed"]
        raw = runs[run_id]["raw"]
        t2 = float(raw["t_brake_effective_s"])
        output_times = np.asarray(
            [
                row.header_time_s - t2
                for row in parsed.perception.get("target_rows", [])
            ]
        )
        output_times = output_times[
            (output_times >= -0.6) & (output_times <= 2.35)
        ]
        axes[0].scatter(
            output_times,
            np.full_like(output_times, y, dtype=float),
            marker="|",
            s=330,
            linewidths=2.0,
            color=color,
            label=labels[0] if run_id == RUN_IDS[0] else labels[1],
        )
        gap_s, gap_start, gap_end = largest_target_gap(parsed)
        start_rel = gap_start - t2
        end_rel = gap_end - t2
        sample_distance = core.integrate_speed(
            parsed.localization, gap_start, gap_end
        )
        gap_values.append(gap_s * 1000)
        lifecycle_values.append(float(raw["fusion_lifecycle_max_ms"]))
        travel_values.append(sample_distance)
        axes[0].plot(
            [start_rel, end_rel],
            [y + 0.18, y + 0.18],
            color=color,
            linewidth=3,
        )
        axes[0].text(
            (start_rel + end_rel) / 2,
            y + 0.28,
            f"{gap_s*1000:.3f} ms / 行驶{sample_distance:.3f} m",
            ha="center",
            va="bottom",
            fontsize=8.5,
        )

    axes[0].axvline(0, color="#555555", linestyle="--", linewidth=1.0)
    axes[0].text(0.02, -0.34, "$t_2$", ha="left", va="bottom", fontsize=9)
    axes[0].set_xlim(-0.6, 2.37)
    axes[0].set_ylim(-0.45, 1.58)
    axes[0].set_yticks(y_positions, labels)
    axes[0].set_xlabel("相对$t_2$的墙钟时间/s")
    axes[0].set_title("目标Fusion输出时间线")
    axes[0].grid(axis="x", alpha=0.22)

    x = np.arange(2)
    width = 0.34
    gap_bars = axes[1].bar(
        x - width / 2,
        gap_values,
        width,
        color=[COLORS["1131"], COLORS["1211"]],
        label="目标输出最大间隔",
    )
    lifecycle_bars = axes[1].bar(
        x + width / 2,
        lifecycle_values,
        width,
        color=[COLORS["1131"], COLORS["1211"]],
        alpha=0.48,
        hatch="//",
        label="Fusion生命周期最大值",
    )
    for bars in (gap_bars, lifecycle_bars):
        for bar in bars:
            axes[1].text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 12,
                f"{bar.get_height():.1f}",
                ha="center",
                va="bottom",
                fontsize=8.5,
            )
    axes[1].axhline(
        500,
        color="#666666",
        linestyle="--",
        linewidth=1.0,
        label="500 ms",
    )
    axes[1].set_xticks(x, ["碰撞_1131", "未碰撞_1211"])
    axes[1].set_ylabel("墙钟时间/ms")
    axes[1].set_ylim(0, 800)
    axes[1].set_title("连续性尾部指标")
    axes[1].grid(axis="y", alpha=0.22)
    axes[1].legend(loc="upper right", frameon=False, fontsize=8)

    fig.suptitle(
        "图5-5 制动阶段目标输出连续性对比",
        y=1.02,
        fontsize=14,
    )
    fig.tight_layout()
    fig.savefig(output_dir / "chapter5_fusion_continuity.png", dpi=220)
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
    figure_fusion_continuity(runs, output_dir)
    for path in sorted(output_dir.glob("chapter5_*.png")):
        print(path)


if __name__ == "__main__":
    main()
