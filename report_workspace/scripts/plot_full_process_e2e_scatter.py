#!/usr/bin/env python3
"""Plot full-process E2E from stable obstacle output to stop/collision.

The outcome timestamp is interval-censored by the local wall-clock sampling
interval.  The plotted estimate uses the interval midpoint and keeps the
half-interval as an error bar.  This weakens the visible 0.1 s quantization
without pretending that the continuous event time was directly observed.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


WORKSPACE = Path(__file__).resolve().parents[2]
OBSERVED = (
    WORKSPACE
    / "第二次实验"
    / "analysis_results"
    / "chapter7_verified_observed_metrics.csv"
)
RUN_LEVEL = WORKSPACE / "report_workspace" / "tables" / "run_level_metrics.csv"
OUTPUT_TABLE = (
    WORKSPACE / "report_workspace" / "tables" / "full_process_e2e_scatter_data.csv"
)
OUTPUT_PNG = (
    WORKSPACE
    / "report_workspace"
    / "figures"
    / "full_process_e2e_from_fusion_output_dequantized.png"
)
OUTPUT_SVG = OUTPUT_PNG.with_suffix(".svg")

LOCALIZATION_TIME_RE = re.compile(r"measurement_time=(?P<time>[-+0-9.eE]+)")
GROUP_ORDER = ["baseline", "delay_300ms"]
GROUP_LABELS = {"baseline": "baseline (0 ms)", "delay_300ms": "持续时延组 (300 ms)"}
GROUP_COLORS = {"baseline": "#4C78A8", "delay_300ms": "#F2A541"}


def configure_plotting() -> None:
    candidates = [
        Path("/System/Library/Fonts/PingFang.ttc"),
        Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf"),
    ]
    for font in candidates:
        if font.exists():
            fm.fontManager.addfont(str(font))
            plt.rcParams["font.family"] = fm.FontProperties(fname=str(font)).get_name()
            break
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.dpi": 220,
        }
    )


def local_wall_interval_s(path: Path, endpoint_s: float) -> float:
    """Return a robust wall-clock sampling interval near the outcome."""
    times: list[float] = []
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = LOCALIZATION_TIME_RE.search(line)
            if match:
                times.append(float(match.group("time")))
    values = np.asarray(sorted(times), dtype=float)
    if values.size < 2:
        return math.nan
    index = int(np.searchsorted(values, endpoint_s, side="right"))
    left = max(1, index - 5)
    right = min(values.size, index + 2)
    gaps = np.diff(values[left - 1 : right])
    gaps = gaps[(gaps > 0.02) & (gaps < 0.5)]
    return float(np.median(gaps)) if gaps.size else math.nan


def build_chart_data() -> pd.DataFrame:
    observed = pd.read_csv(OBSERVED, encoding="utf-8-sig")
    run_level = pd.read_csv(RUN_LEVEL, encoding="utf-8-sig")
    context = run_level[
        [
            "run_id",
            "nominal_injected_delay_ms",
            "sensor_to_fusion_ms",
            "source_localization_file",
            "included_main_analysis",
        ]
    ]
    frame = observed.merge(context, on="run_id", how="left", validate="one_to_one")
    frame = frame.loc[frame["included_main_analysis"].fillna(False)].copy()

    intervals: list[float] = []
    for row in frame.itertuples(index=False):
        interval = local_wall_interval_s(
            Path(row.source_localization_file), float(row.outcome_endpoint_data_observed_s)
        )
        intervals.append(interval)
    frame["outcome_wall_sampling_interval_s"] = intervals
    frame["phase_center_correction_s"] = (
        frame["outcome_wall_sampling_interval_s"] / 2.0
    )
    frame["phase_center_uncertainty_s"] = frame["phase_center_correction_s"]

    frame["stable_fusion_output_s"] = (
        frame["t1_data_observed_s"] + frame["sensor_to_fusion_ms"] / 1000.0
    )
    frame["source_to_outcome_raw_s"] = (
        frame["outcome_endpoint_data_observed_s"] - frame["t1_data_observed_s"]
    )
    frame["source_to_outcome_phase_center_s"] = (
        frame["source_to_outcome_raw_s"] - frame["phase_center_correction_s"]
    )
    frame["fusion_output_to_outcome_raw_s"] = (
        frame["outcome_endpoint_data_observed_s"] - frame["stable_fusion_output_s"]
    )
    frame["fusion_output_to_outcome_phase_center_s"] = (
        frame["fusion_output_to_outcome_raw_s"] - frame["phase_center_correction_s"]
    )
    frame["endpoint_label"] = np.where(
        frame["collision_data_observed"], "碰撞", "停车"
    )
    frame["group_label"] = frame["group_name"].map(GROUP_LABELS)
    frame["run_label"] = frame["run_id"].astype(str).str[-4:]

    fields = [
        "run_id",
        "run_label",
        "group_name",
        "group_label",
        "nominal_injected_delay_ms",
        "collision_data_observed",
        "endpoint_label",
        "outcome_endpoint_type_data_observed",
        "t1_data_observed_s",
        "stable_fusion_output_s",
        "t2_data_observed_s",
        "outcome_endpoint_data_observed_s",
        "sensor_to_fusion_ms",
        "actual_e2e_latency_data_observed_ms",
        "source_to_outcome_raw_s",
        "source_to_outcome_phase_center_s",
        "fusion_output_to_outcome_raw_s",
        "fusion_output_to_outcome_phase_center_s",
        "outcome_wall_sampling_interval_s",
        "phase_center_correction_s",
        "phase_center_uncertainty_s",
        "outcome_endpoint_speed_data_observed_mps",
        "impact_speed_data_observed_mps",
        "D1_clear_data_observed_m",
        "D_delay_wall_integral_data_observed_m",
        "D2_clear_wall_budget_data_observed_m",
        "source_localization_file",
    ]
    return frame[fields].sort_values(
        ["nominal_injected_delay_ms", "run_id"]
    )


def plot(frame: pd.DataFrame) -> None:
    fig, ax = plt.subplots(figsize=(10.5, 6.6))

    for group_index, group in enumerate(GROUP_ORDER):
        subset = frame.loc[frame["group_name"] == group].reset_index(drop=True)
        offsets = np.linspace(-0.18, 0.18, len(subset)) if len(subset) > 1 else np.array([0.0])
        x_values = group_index + offsets
        center = subset["fusion_output_to_outcome_phase_center_s"].to_numpy(float)
        raw = subset["fusion_output_to_outcome_raw_s"].to_numpy(float)
        uncertainty = subset["phase_center_uncertainty_s"].to_numpy(float)

        # Hollow points preserve the directly observed quantized endpoint.
        ax.scatter(
            x_values,
            raw,
            s=42,
            facecolors="none",
            edgecolors="#777777",
            linewidths=1.0,
            zorder=2,
        )
        ax.errorbar(
            x_values,
            center,
            yerr=uncertainty,
            fmt="none",
            ecolor=GROUP_COLORS[group],
            elinewidth=1.1,
            capsize=3,
            alpha=0.75,
            zorder=3,
        )

        for index, row in subset.iterrows():
            marker = "X" if bool(row["collision_data_observed"]) else "o"
            size = 105 if marker == "X" else 74
            ax.scatter(
                x_values[index],
                row["fusion_output_to_outcome_phase_center_s"],
                s=size,
                marker=marker,
                color=GROUP_COLORS[group],
                edgecolor="white",
                linewidth=0.8,
                zorder=4,
            )
            label_offsets = {
                "1031": (0, 10),
                "1048": (-1, 10),
                "1054": (0, 10),
                "1059": (0, 10),
                "1104": (-9, 10),
                "1108": (9, 10),
                "1113": (0, 10),
                "1131": (0, 12),
                "1202": (0, 10),
                "1211": (0, 10),
                "1643": (0, 12),
            }
            dx, dy = label_offsets.get(row["run_label"], (0, 10))
            point_label = row["run_label"]
            if marker == "X":
                point_label += "\n碰撞截断"
            ax.annotate(
                point_label,
                (x_values[index], row["fusion_output_to_outcome_phase_center_s"]),
                xytext=(dx, dy),
                textcoords="offset points",
                ha="center",
                va="bottom",
                fontsize=8.5,
                color="#30343B",
            )

    ax.set_xticks(range(len(GROUP_ORDER)), [GROUP_LABELS[group] for group in GROUP_ORDER])
    ax.set_xlim(-0.45, 1.55)
    ax.set_ylabel("稳定 Fusion 障碍物输出→停车/碰撞 E2E / s")
    ax.set_xlabel("实验条件（每个点为一个 run）")
    ax.grid(axis="y", color="#D9E2EC", linewidth=0.8, alpha=0.8)

    fig.suptitle(
        "稳定障碍物信息输出到停车/碰撞的全过程 E2E 散点图",
        x=0.08,
        y=0.98,
        ha="left",
        fontsize=15,
        fontweight="bold",
        color="#252A34",
    )
    ax.set_title(
        "n=11；彩色点=终点采样区间中心，误差条=±半个当地墙钟采样间隔，灰色空心点=原始观测终点",
        loc="left",
        fontsize=10,
        color="#555555",
        pad=14,
    )

    safe_handle = plt.Line2D(
        [], [], marker="o", linestyle="none", markerfacecolor="#777777",
        markeredgecolor="white", markersize=7, label="完整停车"
    )
    collision_handle = plt.Line2D(
        [], [], marker="X", linestyle="none", color="#777777",
        markeredgecolor="white", markersize=8, label="CARLA 碰撞事件"
    )
    raw_handle = plt.Line2D(
        [], [], marker="o", linestyle="none", markerfacecolor="none",
        markeredgecolor="#777777", markersize=6, label="原始量化观测"
    )
    ax.legend(
        handles=[safe_handle, collision_handle, raw_handle],
        loc="upper right",
        frameon=False,
        ncol=3,
        fontsize=8.5,
    )

    fig.text(
        0.08,
        0.018,
        "说明：不使用 CARLA 帧数或 sim time 替代墙钟时间。每个 run 根据结局附近 Localization 的实际墙钟采样间隔进行区间中心化；碰撞点是接触截断终点，不代表比停车更快完成；1206因结局冲突未绘制。",
        ha="left",
        fontsize=8.5,
        color="#666666",
    )
    fig.subplots_adjust(left=0.10, right=0.94, top=0.82, bottom=0.16)
    fig.savefig(OUTPUT_PNG)
    fig.savefig(OUTPUT_SVG)
    plt.close(fig)


def main() -> None:
    configure_plotting()
    frame = build_chart_data()
    OUTPUT_TABLE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(OUTPUT_TABLE, index=False, encoding="utf-8-sig")
    plot(frame)
    print(f"wrote {OUTPUT_TABLE}")
    print(f"wrote {OUTPUT_PNG}")
    print(f"wrote {OUTPUT_SVG}")


if __name__ == "__main__":
    main()
