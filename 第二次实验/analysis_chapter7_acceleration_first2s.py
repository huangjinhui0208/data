"""Chapter 7 observed acceleration comparison over the first 2 s after t1.

All runs use the same wall-clock interval [t1, t1 + 2.000 s].  Speed is the
three-dimensional Localization speed magnitude already parsed by the verified
analysis core.  Acceleration is the unsmoothed adjacent-sample speed difference.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import shutil
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
RUNTIME_PACKAGES = (
    WORKSPACE_ROOT / "realtime_collision_analysis" / ".python_packages"
)
sys.path.insert(0, str(RUNTIME_PACKAGES))

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "analysis_chapter7_baseline_collision_verified.py"
OUTPUT_ROOT = ROOT / "analysis_results"
VERIFIED_FIGURE_ROOT = OUTPUT_ROOT / "chapter7_verified_figures"
REPORT_FIGURE_ROOT = ROOT / "figures"
WINDOW_S = 2.0
COLLISION_IDS = ("202607271131", "202607271643")
DISPLAY_NAMES = {
    "202607271131": "碰撞_1131",
    "202607271643": "碰撞_1643",
}


def load_verified_module():
    spec = importlib.util.spec_from_file_location("chapter7_verified", SOURCE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import {SOURCE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def window_trajectory(module, run: dict[str, object]) -> dict[str, object]:
    raw = run["raw"]
    parsed = run["parsed"]
    start_s = float(raw["t_sensor_origin_s"])
    end_s = start_s + WINDOW_S
    start = module.core.interpolate_sample(parsed.localization, start_s)
    end = module.core.interpolate_sample(parsed.localization, end_s)
    if start is None or end is None:
        raise RuntimeError(
            f"Missing Localization endpoint for {run['spec'].run_id}"
        )

    points: list[tuple[float, float]] = [
        (start_s, float(start["speed_mps"]))
    ]
    points.extend(
        (float(sample.time_s), float(sample.speed_mps))
        for sample in parsed.localization
        if start_s < sample.time_s < end_s
    )
    points.append((end_s, float(end["speed_mps"])))

    times = np.asarray([point[0] for point in points], dtype=float)
    speeds = np.asarray([point[1] for point in points], dtype=float)
    dt = np.diff(times)
    if np.any(dt <= 0):
        raise RuntimeError(
            f"Non-increasing Localization time in {run['spec'].run_id}"
        )
    acceleration = np.diff(speeds) / dt
    midpoint = (times[:-1] + times[1:]) / 2.0 - start_s
    index_max_abs = int(np.argmax(np.abs(acceleration)))
    t2_elapsed_s = float(raw["t_brake_effective_s"]) - start_s
    return {
        "time_s": midpoint,
        "acceleration_mps2": acceleration,
        "v_start_mps": float(speeds[0]),
        "v_end_mps": float(speeds[-1]),
        "delta_v_mps": float(speeds[-1] - speeds[0]),
        "mean_acceleration_mps2": float(
            (speeds[-1] - speeds[0]) / WINDOW_S
        ),
        "maximum_absolute_acceleration_mps2": float(
            abs(acceleration[index_max_abs])
        ),
        "signed_acceleration_at_maximum_absolute_mps2": float(
            acceleration[index_max_abs]
        ),
        "maximum_absolute_acceleration_time_after_t1_s": float(
            midpoint[index_max_abs]
        ),
        "t2_after_t1_s": t2_elapsed_s,
    }


def save_rows(rows: list[dict[str, object]]) -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "run_id",
        "display_name",
        "group_name",
        "window_start",
        "window_duration_s",
        "speed_definition",
        "acceleration_definition",
        "v_t1_data_observed_mps",
        "v_t1_plus_2s_data_observed_mps",
        "delta_v_first_2s_data_observed_mps",
        "mean_acceleration_first_2s_data_observed_mps2",
        "maximum_absolute_adjacent_sample_acceleration_first_2s_data_observed_mps2",
        "signed_acceleration_at_maximum_absolute_first_2s_data_observed_mps2",
        "maximum_absolute_acceleration_time_after_t1_data_observed_s",
        "t2_after_t1_data_observed_s",
    ]
    path = OUTPUT_ROOT / "chapter7_acceleration_first2s_observed.csv"
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_figure(module, trajectories: dict[str, dict[str, object]]) -> None:
    module.configure_plotting()
    baseline_ids = sorted(
        run_id
        for run_id, item in trajectories.items()
        if item["group_name"] == "baseline"
    )
    fig, ax = plt.subplots(figsize=(12.2, 6.2))

    for index, run_id in enumerate(baseline_ids):
        item = trajectories[run_id]
        ax.plot(
            item["time_s"],
            item["acceleration_mps2"],
            color="#8A8A8A",
            alpha=0.48,
            linewidth=1.25,
            label="baseline实际轨迹（7个run）" if index == 0 else None,
            zorder=2,
        )

    for run_id, color in [
        ("202607271131", "#E69F00"),
        ("202607271643", "#D55E00"),
    ]:
        item = trajectories[run_id]
        ax.plot(
            item["time_s"],
            item["acceleration_mps2"],
            color=color,
            linewidth=2.5,
            label=DISPLAY_NAMES[run_id],
            zorder=4,
        )
        ax.axvline(
            float(item["t2_after_t1_s"]),
            color=color,
            linewidth=1.15,
            linestyle=":",
            alpha=0.95,
            label=(
                f"{DISPLAY_NAMES[run_id]}的$t_2$"
                f"（{float(item['t2_after_t1_s']):.3f} s）"
            ),
            zorder=3,
        )

    baseline_t2 = [
        float(trajectories[run_id]["t2_after_t1_s"])
        for run_id in baseline_ids
    ]
    ax.axvspan(
        min(baseline_t2),
        max(baseline_t2),
        color="#8A8A8A",
        alpha=0.10,
        label=(
            "baseline的$t_2$范围"
            f"（{min(baseline_t2):.3f}–{max(baseline_t2):.3f} s）"
        ),
        zorder=1,
    )
    ax.axhline(0.0, color="#333333", linewidth=1.0, linestyle="--")
    ax.axvline(
        WINDOW_S,
        color="#333333",
        linewidth=1.2,
        linestyle="-.",
        label="统一2.000 s窗口终点",
        zorder=3,
    )
    all_acceleration = np.concatenate(
        [
            np.asarray(item["acceleration_mps2"], dtype=float)
            for item in trajectories.values()
        ]
    )
    lower = float(np.min(all_acceleration))
    upper = float(np.max(all_acceleration))
    padding = max(0.8, (upper - lower) * 0.08)
    ax.set_xlim(0.0, WINDOW_S + 0.015)
    ax.set_ylim(lower - padding, upper + padding)
    ax.set_xlabel("$t_1$后的墙钟时间/s")
    ax.set_ylabel("相邻Localization样本加速度/(m/s²)")
    ax.set_title(
        "图7-5 统一截取$t_1$后前2 s的实测加速度轨迹"
    )
    ax.grid(alpha=0.22)
    ax.legend(loc="lower left", frameon=False, fontsize=8.4, ncol=2)
    fig.tight_layout()

    VERIFIED_FIGURE_ROOT.mkdir(parents=True, exist_ok=True)
    REPORT_FIGURE_ROOT.mkdir(parents=True, exist_ok=True)
    verified_path = (
        VERIFIED_FIGURE_ROOT / "chapter7_verified_acceleration_first2s.png"
    )
    report_path = REPORT_FIGURE_ROOT / "chapter7_acceleration_first2s.png"
    fig.savefig(verified_path, dpi=220)
    plt.close(fig)
    shutil.copy2(verified_path, report_path)


def main() -> None:
    module = load_verified_module()
    runs = module.load_runs()
    selected_ids = sorted(
        run_id
        for run_id, run in runs.items()
        if run["spec"].group_name == "baseline"
    ) + list(COLLISION_IDS)

    rows: list[dict[str, object]] = []
    trajectories: dict[str, dict[str, object]] = {}
    for run_id in selected_ids:
        run = runs[run_id]
        result = window_trajectory(module, run)
        trajectories[run_id] = {
            "group_name": run["spec"].group_name,
            **result,
        }
        rows.append(
            {
                "run_id": run_id,
                "display_name": DISPLAY_NAMES.get(run_id, run_id),
                "group_name": run["spec"].group_name,
                "window_start": "t1",
                "window_duration_s": WINDOW_S,
                "speed_definition": "sqrt(vx^2+vy^2+vz^2)",
                "acceleration_definition": "adjacent_sample_delta_v/delta_t_wall",
                "v_t1_data_observed_mps": result["v_start_mps"],
                "v_t1_plus_2s_data_observed_mps": result["v_end_mps"],
                "delta_v_first_2s_data_observed_mps": result["delta_v_mps"],
                "mean_acceleration_first_2s_data_observed_mps2": result[
                    "mean_acceleration_mps2"
                ],
                "maximum_absolute_adjacent_sample_acceleration_first_2s_data_observed_mps2": result[
                    "maximum_absolute_acceleration_mps2"
                ],
                "signed_acceleration_at_maximum_absolute_first_2s_data_observed_mps2": result[
                    "signed_acceleration_at_maximum_absolute_mps2"
                ],
                "maximum_absolute_acceleration_time_after_t1_data_observed_s": result[
                    "maximum_absolute_acceleration_time_after_t1_s"
                ],
                "t2_after_t1_data_observed_s": result["t2_after_t1_s"],
            }
        )

    save_rows(rows)
    make_figure(module, trajectories)

    for row in rows:
        print(
            f"{row['display_name']}: "
            f"v1={float(row['v_t1_data_observed_mps']):.6f}, "
            f"v2s={float(row['v_t1_plus_2s_data_observed_mps']):.6f}, "
            f"mean_a={float(row['mean_acceleration_first_2s_data_observed_mps2']):.6f}, "
            f"max_abs_a={float(row['maximum_absolute_adjacent_sample_acceleration_first_2s_data_observed_mps2']):.6f}, "
            f"t2={float(row['t2_after_t1_data_observed_s']):.6f}"
        )


if __name__ == "__main__":
    main()
