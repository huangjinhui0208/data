#!/usr/bin/env python3
"""Reconstruct and plot the three overlapping post-t2 timing anomalies in run 1131.

Raw experiment files are read-only. Derived CSV and figures are written under the
existing v4.1 analysis directory.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


ANALYSIS_DIR = Path(__file__).resolve().parent.parent
WORKSPACE = ANALYSIS_DIR.parent.parent
RUN_DIR = WORKSPACE / "第二次实验" / "300ms" / "202607271131"
TABLE_DIR = ANALYSIS_DIR / "tables"
FIGURE_DIR = ANALYSIS_DIR / "figures"

ANCHOR_FILE = RUN_DIR / "trace" / "trace_anchor" / "perception.476004.csv"
FUSION_INPUT_FILE = (
    RUN_DIR
    / "trace"
    / "fusion_inputs"
    / "perception.multi_sensor_fusion.476004.csv"
)
EVENT_TIMELINE_FILE = TABLE_DIR / "event_timeline.csv"
TARGET_TIMELINE_FILE = TABLE_DIR / "target_freshness_timeline.csv"
OUTPUT_TABLE = TABLE_DIR / "post_t2_anomaly_timeline.csv"
OUTPUT_PNG = FIGURE_DIR / "post_t2_concurrent_anomaly_timeline.png"
OUTPUT_SVG = FIGURE_DIR / "post_t2_concurrent_anomaly_timeline.svg"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def first(rows: list[dict[str, str]], **conditions: str) -> dict[str, str]:
    for row in rows:
        if all(row.get(key) == value for key, value in conditions.items()):
            return row
    raise KeyError(f"No row matching {conditions}")


def event_endpoint(path: Path, trace_id: str, phase: str) -> dict[str, str]:
    matches = [
        row
        for row in read_csv(path)
        if row["trace_id"] == trace_id and row["phase"] == phase
    ]
    if not matches:
        raise KeyError(f"No {phase} event for trace {trace_id} in {path}")
    return min(matches, key=lambda row: int(row["mono_ns"]))


def wall_from_mono(anchor: dict[str, str], mono_ns: int) -> float:
    source_wall_s = float(anchor["sensor_time_sec"])
    preproc_wall_s = source_wall_s + float(anchor["ingress_ms"]) / 1000.0
    return preproc_wall_s + (mono_ns - int(anchor["preproc_enter_ns"])) / 1e9


def relative_ms_from_mono(
    anchor: dict[str, str], mono_ns: int, origin_wall_s: float
) -> float:
    """Keep sub-microsecond interval precision without subtracting two epoch floats."""
    source_to_origin_ms = (
        float(anchor["sensor_time_sec"]) - origin_wall_s
    ) * 1000.0
    return (
        source_to_origin_ms
        + float(anchor["ingress_ms"])
        + (mono_ns - int(anchor["preproc_enter_ns"])) / 1e6
    )


def build_rows() -> tuple[list[dict[str, object]], float, float]:
    anchors = {row["trace_id"]: row for row in read_csv(ANCHOR_FILE)}
    # All event mono_ns values are in the same Orin CLOCK_MONOTONIC domain.
    # Use one wall anchor for cross-process placement; per-frame ingress values
    # are rounded and would otherwise introduce a few microseconds of false skew.
    timeline_anchor_trace_id = "72131690813719851"
    timeline_anchor = anchors[timeline_anchor_trace_id]
    fusion_inputs = read_csv(FUSION_INPUT_FILE)
    target_rows = {
        row["trace_id"]: row for row in read_csv(TARGET_TIMELINE_FILE)
    }
    event_rows = read_csv(EVENT_TIMELINE_FILE)
    t_phys = float(first(event_rows, event_id="E07")["t_wall_s"])
    collision = float(first(event_rows, event_id="E09")["t_wall_s"])

    specs = [
        {
            "metric_name": "Lidar Detection",
            "module_trace_id": "72131690813719851",
            "source_trace_id": "72131690813719851",
            "fusion_trace_id": "17293896665878496500",
            "event_file": RUN_DIR
            / "trace"
            / "events"
            / "perception.lidar_detection.476004.csv",
            "start_phase": "proc_enter",
            "end_phase": "output_pub",
        },
        {
            "metric_name": "Planning RunOnce",
            "module_trace_id": "17293896665878496499",
            "source_trace_id": "72131690813719850",
            "fusion_trace_id": "17293896665878496499",
            "event_file": RUN_DIR
            / "trace"
            / "events"
            / "planning.476000.csv",
            "start_phase": "runonce_enter",
            "end_phase": "runonce_exit",
        },
        {
            "metric_name": "Ground Detection",
            "module_trace_id": "72131690813719852",
            "source_trace_id": "72131690813719852",
            "fusion_trace_id": "17293896665878496501",
            "event_file": RUN_DIR
            / "trace"
            / "events"
            / "perception.pointcloud_ground_detection.476004.csv",
            "start_phase": "proc_enter",
            "end_phase": "output_pub",
        },
    ]

    output_rows: list[dict[str, object]] = []
    for spec in specs:
        source_trace_id = str(spec["source_trace_id"])
        fusion_trace_id = str(spec["fusion_trace_id"])
        start = event_endpoint(
            Path(spec["event_file"]),
            str(spec["module_trace_id"]),
            str(spec["start_phase"]),
        )
        end = event_endpoint(
            Path(spec["event_file"]),
            str(spec["module_trace_id"]),
            str(spec["end_phase"]),
        )
        anchor = anchors[source_trace_id]
        target = target_rows[fusion_trace_id]
        start_mono_ns = int(start["mono_ns"])
        end_mono_ns = int(end["mono_ns"])
        start_wall_s = wall_from_mono(timeline_anchor, start_mono_ns)
        end_wall_s = wall_from_mono(timeline_anchor, end_mono_ns)
        start_relative_t_phys_ms = relative_ms_from_mono(
            timeline_anchor, start_mono_ns, t_phys
        )
        end_relative_t_phys_ms = relative_ms_from_mono(
            timeline_anchor, end_mono_ns, t_phys
        )

        mapped_parent = first(
            fusion_inputs,
            fusion_trace_id=fusion_trace_id,
            sensor="velodyne64",
        )["parent_trace_id"]
        if mapped_parent != source_trace_id:
            raise ValueError(
                f"Fusion mapping mismatch: {fusion_trace_id} -> {mapped_parent}, "
                f"expected {source_trace_id}"
            )

        output_rows.append(
            {
                "run_id": "202607271131",
                "metric_name": spec["metric_name"],
                "module_trace_id": spec["module_trace_id"],
                "source_trace_id": source_trace_id,
                "fusion_trace_id": fusion_trace_id,
                "source_wall_s": float(anchor["sensor_time_sec"]),
                "start_phase": spec["start_phase"],
                "end_phase": spec["end_phase"],
                "start_mono_ns": start_mono_ns,
                "end_mono_ns": end_mono_ns,
                "start_wall_s": start_wall_s,
                "end_wall_s": end_wall_s,
                "start_relative_t_phys_ms": start_relative_t_phys_ms,
                "end_relative_t_phys_ms": end_relative_t_phys_ms,
                "duration_ms": (end_mono_ns - start_mono_ns) / 1e6,
                "fusion_output_wall_s": float(target["fusion_output_wall_s"]),
                "fusion_output_relative_t_phys_ms": (
                    float(target["fusion_output_wall_s"]) - t_phys
                )
                * 1000.0,
                "fusion_lifecycle_ms": float(target["lifecycle_age_at_output_ms"]),
                "gap_from_previous_fusion_ms": float(
                    target["output_gap_from_previous_ms"]
                ),
                "pid": start["pid"],
                "tid": start["tid"],
                "clock_basis": "shared monotonic_ns mapped with one source wall anchor",
                "wall_anchor_trace_id": timeline_anchor_trace_id,
                "source_event_file": str(spec["event_file"]),
                "source_anchor_file": str(ANCHOR_FILE),
            }
        )
    return output_rows, t_phys, collision


def write_table(rows: list[dict[str, object]]) -> None:
    OUTPUT_TABLE.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_TABLE.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def add_blossom(fig: plt.Figure) -> None:
    """Small fixed research mark in the top-right header."""
    center_x, center_y = 0.967, 0.965
    for dx, dy, angle in [
        (0.0, 0.009, 0),
        (0.009, 0.0, 90),
        (0.0, -0.009, 0),
        (-0.009, 0.0, 90),
    ]:
        fig.add_artist(
            Ellipse(
                (center_x + dx, center_y + dy),
                0.012,
                0.022,
                angle=angle,
                transform=fig.transFigure,
                facecolor="#DDEEFF",
                edgecolor="#0169CC",
                linewidth=0.8,
                zorder=20,
            )
        )


def plot(rows: list[dict[str, object]], t_phys: float, collision: float) -> None:
    plt.rcParams.update(
        {
            "font.family": "Arial Unicode MS",
            "axes.unicode_minus": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#0D0D0D",
            "text.color": "#0D0D0D",
            "axes.labelcolor": "#0D0D0D",
            "xtick.color": "#5D5D5D",
            "ytick.color": "#0D0D0D",
        }
    )

    colors = {
        "Lidar Detection": "#0169CC",
        "Planning RunOnce": "#8046D9",
        "Ground Detection": "#E25507",
    }
    hatches = {
        "Lidar Detection": "//",
        "Planning RunOnce": "xx",
        "Ground Detection": "..",
    }
    row_by_metric = {str(row["metric_name"]): row for row in rows}
    common_start = max(float(row["start_relative_t_phys_ms"]) for row in rows)
    common_end = min(float(row["end_relative_t_phys_ms"]) for row in rows)
    common_overlap = common_end - common_start
    collision_rel_ms = (collision - t_phys) * 1000.0

    fig, (ax_exec, ax_life) = plt.subplots(
        2,
        1,
        figsize=(15.5, 8.8),
        dpi=180,
        sharex=True,
        gridspec_kw={"height_ratios": [1.15, 1.0], "hspace": 0.30},
    )
    fig.subplots_adjust(left=0.19, right=0.965, top=0.86, bottom=0.13)
    fig.suptitle(
        "1131 run：碰撞前三个时序异常的执行窗口",
        x=0.19,
        y=0.955,
        ha="left",
        fontsize=20,
        fontweight="bold",
    )
    fig.text(
        0.19,
        0.912,
        "墙钟位置由 monotonic trace 通过各 source trace anchor 对齐；横轴为相对首次物理起效 t_phys 的时间",
        ha="left",
        fontsize=11.5,
        color="#5D5D5D",
    )
    add_blossom(fig)

    order = ["Lidar Detection", "Planning RunOnce", "Ground Detection"]
    y_positions = {name: 2 - index for index, name in enumerate(order)}
    for name in order:
        row = row_by_metric[name]
        start_ms = float(row["start_relative_t_phys_ms"])
        duration_ms = float(row["duration_ms"])
        y = y_positions[name]
        ax_exec.barh(
            y,
            duration_ms,
            left=start_ms,
            height=0.56,
            color=colors[name],
            edgecolor="#0D0D0D",
            linewidth=0.9,
            hatch=hatches[name],
            alpha=0.84,
            zorder=3,
        )
        ax_exec.text(
            start_ms + duration_ms / 2,
            y,
            f"{duration_ms:.3f} ms",
            ha="center",
            va="center",
            color="white",
            fontsize=10.5,
            fontweight="bold",
            zorder=5,
        )
        fusion_x = float(row["fusion_output_relative_t_phys_ms"])
        ax_exec.scatter(
            fusion_x,
            y,
            marker="D",
            s=55,
            facecolor="white",
            edgecolor=colors[name],
            linewidth=1.8,
            zorder=6,
        )

    ax_exec.axvspan(
        common_start,
        common_end,
        color="#F4C95D",
        alpha=0.20,
        zorder=1,
    )
    ax_exec.annotate(
        f"三者共同重叠 {common_overlap:.3f} ms",
        xy=((common_start + common_end) / 2, 2.48),
        xytext=((common_start + common_end) / 2, 2.72),
        ha="center",
        va="bottom",
        fontsize=11,
        color="#5A4300",
        arrowprops={"arrowstyle": "-[,widthB=11.2", "color": "#8B6B00", "lw": 1.2},
    )
    ax_exec.set_title("A. 模块执行区间（不同 source/trace 实例）", loc="left", fontsize=13.5)
    ax_exec.set_yticks([2, 1, 0], order)
    ax_exec.set_ylim(-0.55, 2.92)
    ax_exec.text(
        0.995,
        0.02,
        "菱形：该 source 实例对应的 Fusion output",
        transform=ax_exec.transAxes,
        ha="right",
        va="bottom",
        fontsize=9.5,
        color="#5D5D5D",
    )

    freshness = read_csv(TARGET_TIMELINE_FILE)
    target_ids = [
        "17293896665878496499",
        "17293896665878496500",
        "17293896665878496501",
    ]
    target_by_id = {row["trace_id"]: row for row in freshness if row["trace_id"] in target_ids}
    life_y = {target_ids[0]: 2, target_ids[1]: 1, target_ids[2]: 0}
    life_labels = {
        target_ids[0]: "Fusion .96499 / parent .19850",
        target_ids[1]: "Fusion .96500 / parent .19851",
        target_ids[2]: "Fusion .96501 / parent .19852",
    }
    for trace_id in target_ids:
        target = target_by_id[trace_id]
        source_rel = (float(target["source_time_wall_s"]) - t_phys) * 1000.0
        output_rel = (float(target["fusion_output_wall_s"]) - t_phys) * 1000.0
        lifecycle = float(target["lifecycle_age_at_output_ms"])
        is_delayed = trace_id != target_ids[0]
        fill = "#99CEFF" if is_delayed else "#D8D8D8"
        edge = "#0169CC" if is_delayed else "#5D5D5D"
        y = life_y[trace_id]
        ax_life.barh(
            y,
            output_rel - source_rel,
            left=source_rel,
            height=0.48,
            color=fill,
            edgecolor=edge,
            linewidth=1.2,
            zorder=3,
        )
        ax_life.scatter(
            output_rel,
            y,
            marker="D",
            s=48,
            facecolor="white",
            edgecolor=edge,
            linewidth=1.6,
            zorder=5,
        )
        ax_life.text(
            source_rel + (output_rel - source_rel) / 2,
            y,
            f"lifecycle {lifecycle:.3f} ms",
            ha="center",
            va="center",
            fontsize=10,
            color="#0D0D0D",
            zorder=6,
        )

    prev_output = (float(target_by_id[target_ids[0]]["fusion_output_wall_s"]) - t_phys) * 1000.0
    delayed_output = (float(target_by_id[target_ids[1]]["fusion_output_wall_s"]) - t_phys) * 1000.0
    gap_ms = delayed_output - prev_output
    ax_life.annotate(
        "",
        xy=(delayed_output, 2.52),
        xytext=(prev_output, 2.52),
        arrowprops={"arrowstyle": "<->", "color": "#0169CC", "lw": 1.6},
    )
    ax_life.text(
        (prev_output + delayed_output) / 2,
        2.62,
        f"Fusion output gap {gap_ms:.3f} ms",
        ha="center",
        va="bottom",
        fontsize=10.5,
        color="#0169CC",
        fontweight="bold",
    )
    ax_life.set_title("B. 相邻目标实例的 source→Fusion lifecycle 与输出缺口", loc="left", fontsize=13.5)
    ax_life.set_yticks([2, 1, 0], [life_labels[trace_id] for trace_id in target_ids])
    ax_life.set_ylim(-0.55, 2.95)

    for axis in (ax_exec, ax_life):
        axis.axvline(0, color="#0D0D0D", linewidth=1.2, zorder=2)
        axis.axvline(
            collision_rel_ms,
            color="#0D0D0D",
            linewidth=1.3,
            linestyle=(0, (5, 4)),
            zorder=2,
        )
        axis.grid(axis="x", color="#D9D9D9", linewidth=0.7, alpha=0.8, zorder=0)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        axis.tick_params(axis="y", length=0, pad=8)
    ax_exec.text(5, 2.78, "t_phys", ha="left", va="top", fontsize=10, fontweight="bold")
    ax_exec.text(
        collision_rel_ms - 7,
        2.78,
        f"碰撞  +{collision_rel_ms:.3f} ms",
        ha="right",
        va="top",
        fontsize=10,
        fontweight="bold",
    )
    ax_life.set_xlim(-20, collision_rel_ms + 85)
    ax_life.set_xlabel("相对 t_phys 的墙钟时间（ms）", fontsize=11.5)
    ax_life.set_xticks(range(0, int(collision_rel_ms) + 1, 250))

    fig.text(
        0.19,
        0.048,
        "结论：三个尖峰来自三个相邻 trace，不是同一帧；但执行区间共同重叠约 467 ms。"
        " .96500 对应最大 Fusion gap，.96501 对应最大 lifecycle。",
        ha="left",
        fontsize=10.5,
        color="#0D0D0D",
    )
    fig.text(
        0.19,
        0.018,
        "数据源：run 202607271131 原始 trace/events、trace_anchor、fusion_inputs；Fusion 输出时刻来自 target_freshness_timeline.csv。",
        ha="left",
        fontsize=9.2,
        color="#5D5D5D",
    )

    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PNG, dpi=220, bbox_inches="tight", facecolor="white")
    fig.savefig(OUTPUT_SVG, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> None:
    rows, t_phys, collision = build_rows()
    write_table(rows)
    plot(rows, t_phys, collision)
    common_start = max(float(row["start_relative_t_phys_ms"]) for row in rows)
    common_end = min(float(row["end_relative_t_phys_ms"]) for row in rows)
    print(f"wrote {OUTPUT_TABLE}")
    print(f"wrote {OUTPUT_PNG}")
    print(f"wrote {OUTPUT_SVG}")
    print(f"triple-overlap-ms={common_end - common_start:.6f}")


if __name__ == "__main__":
    main()
