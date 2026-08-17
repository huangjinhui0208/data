#!/usr/bin/env python3
"""Quantify frame-wise cyber latency and distance exposure for run 202607271131.

The primary endpoint is the first trace-matched Control output, not a per-frame
physical-effect time.  The ego vehicle is already moving and Control messages are
reused, so a unique position-change effect cannot be attributed to every sensor
frame with the archived evidence.

Raw run files are read-only.  Main distance results use the AGENTS.md convention:
wall-clock trapezoidal integration of observed Localization speed.  Any trace that
does not finish before the collision is explicitly outcome-censored.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


WORKSPACE = Path("/Users/huangjinhui/Desktop/萨卡班/data")
RUN_DIR = WORKSPACE / "第二次实验/300ms/202607271131"
OUTPUT = WORKSPACE / "output/second_experiment_1131_tcps_pa_v4_2_all_instances"
TABLES = OUTPUT / "tables"
FIGURES = OUTPUT / "figures"
REPORT = OUTPUT / "report"
RUN_ID = "202607271131"

ANCHOR_FILE = RUN_DIR / "trace/trace_anchor/perception.476004.csv"
FUSION_INPUT_FILE = RUN_DIR / "trace/fusion_inputs/perception.multi_sensor_fusion.476004.csv"
LINEAGE_FILE = TABLES / "all_instance_lineage_timing.csv"
VELOCITY_FILE = TABLES / "velocity_trajectory_observed.csv"
EVENT_FILE = TABLES / "event_timeline.csv"

FRAME_TABLE = TABLES / "framewise_sensor_to_control_performance.csv"
STAGE_TABLE = TABLES / "framewise_stage_performance.csv"
SUMMARY_TABLE = TABLES / "framewise_performance_summary.csv"
FIGURE_PNG = FIGURES / "framewise_e2e_latency_distance.png"
FIGURE_SVG = FIGURES / "framewise_e2e_latency_distance.svg"
REPORT_FILE = REPORT / "framewise_latency_vehicle_performance_report.md"

STAGES = (
    ("source_to_fusion_output", "Source→Fusion", "#2C7FB8"),
    ("fusion_to_prediction_output", "Fusion→Prediction", "#45A9A5"),
    ("prediction_to_planning_output", "Prediction→Planning", "#7A5AA6"),
    ("planning_output_to_first_control_output", "Planning→Control", "#E07A3F"),
)
SELECTED_LATE_TRACES = {
    "17293896665878496499",
    "17293896665878496500",
    "17293896665878496501",
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = []
        for row in rows:
            for field in row:
                if field not in fields:
                    fields.append(field)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def upsert_rows(path: Path, new_rows: list[dict], key: str) -> None:
    existing = read_csv(path) if path.exists() else []
    new_keys = {str(row[key]) for row in new_rows}
    kept = [row for row in existing if str(row.get(key)) not in new_keys]
    write_csv(path, kept + new_rows)


def choose_font() -> str:
    available = {font.name for font in fm.fontManager.ttflist}
    for candidate in ("Arial Unicode MS", "PingFang SC", "Songti SC", "DejaVu Sans"):
        if candidate in available:
            return candidate
    return "DejaVu Sans"


def event_time(event_id: str) -> float:
    return float(next(row["t_wall_s"] for row in read_csv(EVENT_FILE) if row["event_id"] == event_id))


def interpolate(samples: list[tuple[float, float]], t: float) -> float:
    times = [row[0] for row in samples]
    position = bisect.bisect_right(times, t)
    if position == 0:
        return samples[0][1]
    if position == len(samples):
        return samples[-1][1]
    left_t, left_v = samples[position - 1]
    right_t, right_v = samples[position]
    fraction = (t - left_t) / (right_t - left_t)
    return left_v + fraction * (right_v - left_v)


def wall_speed_integral(samples: list[tuple[float, float]], start: float, end: float) -> float:
    if end <= start:
        return 0.0
    points = [(start, interpolate(samples, start))]
    points.extend((t, speed) for t, speed in samples if start < t < end)
    points.append((end, interpolate(samples, end)))
    return sum(
        (right[0] - left[0]) * (left[1] + right[1]) / 2.0
        for left, right in zip(points, points[1:])
    )


def fmt(value: float | str | None, digits: int = 6) -> str:
    if value is None or value == "":
        return ""
    return f"{float(value):.{digits}f}"


def build_tables() -> tuple[list[dict], list[dict], list[dict], dict[str, float]]:
    t_sample = event_time("E01")
    t_phys = event_time("E07")
    collision = event_time("E09")

    anchors = [
        row
        for row in read_csv(ANCHOR_FILE)
        if row["sensor_kind"] == "lidar"
        and t_sample <= float(row["sensor_time_sec"]) < collision
    ]
    anchors.sort(key=lambda row: float(row["sensor_time_sec"]))

    main_inputs = {
        row["parent_trace_id"]: row
        for row in read_csv(FUSION_INPUT_FILE)
        if row["sensor_kind"] == "lidar" and row["is_main_sensor"] == "1"
    }
    lineage_rows = read_csv(LINEAGE_FILE)
    lineage: dict[str, dict[str, dict[str, str]]] = {}
    for row in lineage_rows:
        lineage.setdefault(row["fusion_trace_id"], {})[row["metric_name"]] = row

    speed_samples = sorted(
        (float(row["t_wall_s"]), float(row["speed_mps"]))
        for row in read_csv(VELOCITY_FILE)
        if row["availability"] == "AVAILABLE"
    )

    # Preserve the last directly observed prefix endpoint for source frames that
    # never acquire a main-Fusion lineage.  This local monotonic timestamp is
    # mapped from the frame's own source/preprocess anchor; it is not imputed as
    # a Fusion or full-stack completion.
    prefix_outputs: dict[str, list[tuple[int, str]]] = {}
    for event_path in sorted((RUN_DIR / "trace/events").glob("perception.*.csv")):
        for event in read_csv(event_path):
            if event.get("phase") != "output_pub":
                continue
            prefix_outputs.setdefault(event["trace_id"], []).append(
                (int(event["mono_ns"]), event["module"])
            )

    # The reference is descriptive, based only on the 17 target-bearing frames
    # before the selected late anomaly traces. It is not a deadline.
    reference_values: list[float] = []
    for anchor in anchors:
        fusion_input = main_inputs.get(anchor["trace_id"])
        if not fusion_input:
            continue
        fusion_trace = fusion_input["fusion_trace_id"]
        flow = lineage.get(fusion_trace, {})
        e2e = flow.get("source_to_first_control_output")
        source_to_fusion = flow.get("source_to_fusion_output")
        if (
            e2e
            and e2e["availability"] == "AVAILABLE"
            and source_to_fusion
            and source_to_fusion["is_target_11_instance"] == "True"
            and fusion_trace not in SELECTED_LATE_TRACES
        ):
            reference_values.append(float(e2e["duration_ms"]))
    reference_p50_ms = statistics.median(reference_values)

    frame_rows: list[dict] = []
    stage_rows: list[dict] = []
    previous_source: float | None = None

    for frame_index, anchor in enumerate(anchors):
        parent = anchor["trace_id"]
        source = float(anchor["sensor_time_sec"])
        fusion_input = main_inputs.get(parent)
        fusion_trace = fusion_input["fusion_trace_id"] if fusion_input else ""
        flow = lineage.get(fusion_trace, {}) if fusion_trace else {}
        e2e = flow.get("source_to_first_control_output")
        source_to_fusion = flow.get("source_to_fusion_output")
        target_present = bool(
            source_to_fusion and source_to_fusion.get("is_target_11_instance") == "True"
        )
        prefix_candidates = prefix_outputs.get(parent, [])
        if prefix_candidates:
            prefix_mono_ns, prefix_module = max(prefix_candidates)
            prefix_wall = (
                source
                + float(anchor["ingress_ms"]) / 1000.0
                + (prefix_mono_ns - int(anchor["preproc_enter_ns"])) / 1e9
            )
        else:
            prefix_module = ""
            prefix_wall = None

        if not fusion_input:
            endpoint_status = "NO_FUSION_LINEAGE"
            outcome_compatibility = "MISSING_BEFORE_OUTCOME"
            first_control = None
        elif not e2e or e2e.get("availability") != "AVAILABLE":
            endpoint_status = "INCOMPLETE_DOWNSTREAM_LINEAGE"
            outcome_compatibility = "MISSING_BEFORE_OUTCOME"
            first_control = None
        else:
            first_control = float(e2e["end_wall_s"])
            endpoint_status = "COMPLETE_TO_FIRST_CONTROL"
            outcome_compatibility = (
                "PRE_COLLISION_COMPLETE"
                if first_control <= collision
                else "RIGHT_CENSORED_AT_COLLISION"
            )

        full_trace_latency_ms = (
            (first_control - source) * 1000.0 if first_control is not None else None
        )
        full_trace_distance_m = (
            wall_speed_integral(speed_samples, source, first_control)
            if first_control is not None
            else None
        )
        pre_collision_complete = outcome_compatibility == "PRE_COLLISION_COMPLETE"
        main_e2e_latency_ms = full_trace_latency_ms if pre_collision_complete else None
        main_e2e_distance_m = full_trace_distance_m if pre_collision_complete else None
        source_to_collision_ms = (
            (collision - source) * 1000.0
            if outcome_compatibility == "RIGHT_CENSORED_AT_COLLISION"
            else None
        )
        source_to_collision_distance_m = (
            wall_speed_integral(speed_samples, source, collision)
            if outcome_compatibility == "RIGHT_CENSORED_AT_COLLISION"
            else None
        )

        excess_ms = (
            max(0.0, main_e2e_latency_ms - reference_p50_ms)
            if main_e2e_latency_ms is not None
            else None
        )
        excess_distance_m = (
            wall_speed_integral(
                speed_samples,
                source + reference_p50_ms / 1000.0,
                first_control,
            )
            if pre_collision_complete
            and first_control is not None
            and full_trace_latency_ms is not None
            and full_trace_latency_ms > reference_p50_ms
            else 0.0 if pre_collision_complete else None
        )

        performance_state = (
            "SELECTED_LATE_ANOMALY"
            if fusion_trace in SELECTED_LATE_TRACES
            else "TARGET_PRESENT"
            if target_present
            else "TARGET_ABSENT"
        )

        stage_times: dict[str, float | None] = {}
        stage_distances: dict[str, float | None] = {}
        plotted_time_total = 0.0
        plotted_distance_total = 0.0
        for order, (metric, label, _) in enumerate(STAGES, 1):
            edge = flow.get(metric)
            if not edge or edge.get("availability") != "AVAILABLE":
                stage_time = None
                stage_distance = None
                clipped_start = None
                clipped_end = None
                stage_status = "UNAVAILABLE"
            else:
                edge_start = float(edge["start_wall_s"])
                edge_end = float(edge["end_wall_s"])
                if pre_collision_complete:
                    clipped_start = edge_start
                    clipped_end = edge_end
                    stage_status = "PRE_COLLISION_COMPLETE"
                elif outcome_compatibility == "RIGHT_CENSORED_AT_COLLISION":
                    clipped_start = edge_start
                    clipped_end = min(edge_end, collision)
                    if clipped_end <= clipped_start:
                        clipped_start = None
                        clipped_end = None
                        stage_status = "POST_OUTCOME_EXCLUDED"
                    elif edge_end > collision:
                        stage_status = "RIGHT_CENSORED_WITHIN_STAGE"
                    else:
                        stage_status = "COMPLETED_BEFORE_OUTCOME"
                else:
                    clipped_start = None
                    clipped_end = None
                    stage_status = "UNAVAILABLE"
                if clipped_start is None or clipped_end is None:
                    stage_time = None
                    stage_distance = None
                else:
                    stage_time = (clipped_end - clipped_start) * 1000.0
                    stage_distance = wall_speed_integral(
                        speed_samples, clipped_start, clipped_end
                    )
                    plotted_time_total += stage_time
                    plotted_distance_total += stage_distance

            stage_times[metric] = stage_time
            stage_distances[metric] = stage_distance
            stage_rows.append(
                {
                    "run_id": RUN_ID,
                    "frame_index": frame_index,
                    "parent_trace_id": parent,
                    "fusion_trace_id": fusion_trace,
                    "source_relative_t_sample_s": fmt(source - t_sample, 6),
                    "stage_order": order,
                    "stage_metric": metric,
                    "stage_label": label,
                    "stage_start_wall_s": fmt(clipped_start, 9),
                    "stage_end_wall_s": fmt(clipped_end, 9),
                    "T_stage_data_observed_ms": fmt(stage_time, 6),
                    "D_stage_wall_integral_data_observed_m": fmt(stage_distance, 6),
                    "stage_outcome_status": stage_status,
                    "target_11_present": target_present,
                    "performance_state": performance_state,
                    "lineage_grade": edge.get("lineage_grade", "") if edge else "",
                    "clock_basis": "aligned wall epoch; source-wall anchor",
                    "distance_method": "endpoint linear interpolation + wall-clock trapezoidal speed integral",
                }
            )

        frame_rows.append(
            {
                "run_id": RUN_ID,
                "frame_index": frame_index,
                "parent_trace_id": parent,
                "fusion_trace_id": fusion_trace,
                "source_wall_s": fmt(source, 9),
                "source_relative_t_sample_s": fmt(source - t_sample, 6),
                "source_gap_ms": fmt((source - previous_source) * 1000.0 if previous_source else None, 6),
                "ingress_ms": fmt(anchor.get("ingress_ms"), 6),
                "fusion_object_count": fusion_input.get("object_count", "") if fusion_input else "",
                "target_11_present": target_present,
                "performance_state": performance_state,
                "endpoint_status": endpoint_status,
                "outcome_compatibility": outcome_compatibility,
                "first_control_wall_s": fmt(first_control, 9),
                "v_source_data_observed_mps": fmt(interpolate(speed_samples, source), 6),
                "v_first_control_data_observed_mps": fmt(
                    interpolate(speed_samples, first_control) if first_control is not None else None,
                    6,
                ),
                "last_observed_prefix_endpoint": prefix_module if not fusion_input else "",
                "last_observed_prefix_wall_s": fmt(prefix_wall if not fusion_input else None, 9),
                "T_source_to_last_observed_prefix_ms": fmt(
                    (prefix_wall - source) * 1000.0
                    if not fusion_input and prefix_wall is not None
                    else None,
                    6,
                ),
                "D_source_to_last_observed_prefix_wall_integral_m": fmt(
                    wall_speed_integral(speed_samples, source, prefix_wall)
                    if not fusion_input and prefix_wall is not None
                    else None,
                    6,
                ),
                "T_e2e_data_observed_pre_collision_ms": fmt(main_e2e_latency_ms, 6),
                "D_e2e_wall_integral_data_observed_m": fmt(main_e2e_distance_m, 6),
                "T_source_to_collision_right_censored_ms": fmt(source_to_collision_ms, 6),
                "D_source_to_collision_right_censored_m": fmt(source_to_collision_distance_m, 6),
                "T_trace_complete_post_outcome_diagnostic_ms": fmt(
                    full_trace_latency_ms if not pre_collision_complete else None, 6
                ),
                "D_trace_complete_post_outcome_diagnostic_m": fmt(
                    full_trace_distance_m if not pre_collision_complete else None, 6
                ),
                "reference_pre_anomaly_target_p50_ms": fmt(reference_p50_ms, 6),
                "T_excess_vs_reference_research_ms": fmt(excess_ms, 6),
                "D_excess_after_reference_wall_integral_research_m": fmt(excess_distance_m, 6),
                "T_plotted_stage_sum_ms": fmt(plotted_time_total, 6),
                "D_plotted_stage_sum_m": fmt(plotted_distance_total, 6),
                **{
                    f"T_{metric}_data_observed_ms": fmt(stage_times[metric], 6)
                    for metric, _, _ in STAGES
                },
                **{
                    f"D_{metric}_wall_integral_data_observed_m": fmt(stage_distances[metric], 6)
                    for metric, _, _ in STAGES
                },
                "physical_effect_episode_status": "NOT_IDENTIFIABLE_NO_UNIQUE_ACTION_EPISODE",
                "primary_semantic_label": "OBSERVED_PIPELINE_OCCUPANCY_EXPOSURE_NOT_CAUSAL_LOSS",
                "reference_semantic_label": "RETROSPECTIVE_RESEARCH_REFERENCE_DIAGNOSTIC_NOT_DEADLINE",
                "lineage_grade": "A" if fusion_trace else "UNAVAILABLE",
                "clock_basis": "sensor source wall epoch; trace affine mapping to wall epoch",
                "distance_method": "Localization speed endpoint interpolation + wall-clock trapezoidal integration",
                "quality_flags": (
                    "SELECTED_LATE_ANOMALY;INTERVALS_OVERLAP_DO_NOT_SUM"
                    if fusion_trace in SELECTED_LATE_TRACES
                    else "INTERVALS_OVERLAP_DO_NOT_SUM"
                ),
            }
        )
        previous_source = source

    completed = [
        row for row in frame_rows if row["outcome_compatibility"] == "PRE_COLLISION_COMPLETE"
    ]
    target_completed = [row for row in completed if row["target_11_present"]]
    selected = [row for row in completed if row["performance_state"] == "SELECTED_LATE_ANOMALY"]
    pre_selected = [
        row
        for row in target_completed
        if row["performance_state"] != "SELECTED_LATE_ANOMALY"
    ]
    overlap_intervals = [
        (float(row["source_wall_s"]), float(row["first_control_wall_s"]))
        for row in target_completed
    ]

    def union_distance(intervals: list[tuple[float, float]]) -> float:
        if not intervals:
            return 0.0
        merged: list[list[float]] = []
        for start, end in sorted(intervals):
            if not merged or start > merged[-1][1]:
                merged.append([start, end])
            else:
                merged[-1][1] = max(merged[-1][1], end)
        return sum(wall_speed_integral(speed_samples, start, end) for start, end in merged)

    sum_target_distances = sum(
        float(row["D_e2e_wall_integral_data_observed_m"]) for row in target_completed
    )
    union_target_distance = union_distance(overlap_intervals)
    selected_mean_latency = statistics.mean(
        float(row["T_e2e_data_observed_pre_collision_ms"]) for row in selected
    )
    selected_mean_distance = statistics.mean(
        float(row["D_e2e_wall_integral_data_observed_m"]) for row in selected
    )

    summary_rows = [
        {
            "run_id": RUN_ID,
            "metric": "lidar_inputs_t_sample_to_collision",
            "value": len(frame_rows),
            "unit": "frames",
            "evidence_class": "DIRECT_OBSERVED",
            "interpretation": "all LiDAR source anchors from first stable target sample to collision",
        },
        {
            "run_id": RUN_ID,
            "metric": "pre_collision_complete_to_first_control",
            "value": len(completed),
            "unit": "frames",
            "evidence_class": "TRACE_LINEAGE",
            "interpretation": "Grade-A software chain compatible with pre-collision outcome",
        },
        {
            "run_id": RUN_ID,
            "metric": "no_fusion_lineage",
            "value": sum(row["endpoint_status"] == "NO_FUSION_LINEAGE" for row in frame_rows),
            "unit": "frames",
            "evidence_class": "MISSING_EXPLICIT",
            "interpretation": "sensor inputs retained as missing, not imputed",
        },
        {
            "run_id": RUN_ID,
            "metric": "outcome_right_censored",
            "value": sum(row["outcome_compatibility"] == "RIGHT_CENSORED_AT_COLLISION" for row in frame_rows),
            "unit": "frames",
            "evidence_class": "TRACE_LINEAGE",
            "interpretation": "full trace exists, but first Control is after collision",
        },
        {
            "run_id": RUN_ID,
            "metric": "pre_selected_target_reference_p50",
            "value": fmt(reference_p50_ms, 6),
            "unit": "ms",
            "evidence_class": "RETROSPECTIVE_RESEARCH_REFERENCE",
            "interpretation": "17 target-bearing frames before selected late traces; not a deadline",
        },
        {
            "run_id": RUN_ID,
            "metric": "selected_late_frames_mean_e2e_latency",
            "value": fmt(selected_mean_latency, 6),
            "unit": "ms",
            "evidence_class": "OBSERVED_DERIVED",
            "interpretation": "mean of three selected late target-bearing frames",
        },
        {
            "run_id": RUN_ID,
            "metric": "selected_late_frames_mean_distance_exposure",
            "value": fmt(selected_mean_distance, 6),
            "unit": "m",
            "evidence_class": "OBSERVED_DERIVED",
            "interpretation": "wall-speed integral; occupancy exposure, not causal loss",
        },
        {
            "run_id": RUN_ID,
            "metric": "target_frame_distance_sum_non_additive",
            "value": fmt(sum_target_distances, 6),
            "unit": "m",
            "evidence_class": "DIAGNOSTIC_ONLY",
            "interpretation": "overlapping per-frame intervals; forbidden as total loss",
        },
        {
            "run_id": RUN_ID,
            "metric": "target_frame_interval_union_distance",
            "value": fmt(union_target_distance, 6),
            "unit": "m",
            "evidence_class": "OBSERVED_DERIVED",
            "interpretation": "distance on de-duplicated union of the 20 target-frame intervals",
        },
        {
            "run_id": RUN_ID,
            "metric": "per_frame_physical_effect_episodes_identified",
            "value": 0,
            "unit": "episodes",
            "evidence_class": "NOT_IDENTIFIABLE",
            "interpretation": "no Control payload/apply/feedback binding per independent action episode",
        },
    ]

    metadata = {
        "t_sample": t_sample,
        "t_phys": t_phys,
        "collision": collision,
        "reference_p50_ms": reference_p50_ms,
        "pre_selected_mean_distance_m": statistics.mean(
            float(row["D_e2e_wall_integral_data_observed_m"]) for row in pre_selected
        ),
        "selected_mean_latency_ms": selected_mean_latency,
        "selected_mean_distance_m": selected_mean_distance,
        "sum_target_distance_m": sum_target_distances,
        "union_target_distance_m": union_target_distance,
    }
    return frame_rows, stage_rows, summary_rows, metadata


def draw_figure(frame_rows: list[dict], metadata: dict[str, float]) -> None:
    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "font.size": 10.5,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "figure.dpi": 150,
        }
    )
    fig = plt.figure(figsize=(17.5, 10.4), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=(2.15, 1.0), height_ratios=(1, 1))
    ax_time = fig.add_subplot(grid[0, 0])
    ax_distance = fig.add_subplot(grid[1, 0], sharex=ax_time)
    ax_scatter = fig.add_subplot(grid[:, 1])

    x = np.array([float(row["source_relative_t_sample_s"]) for row in frame_rows])
    width = 0.062
    time_bottom = np.zeros(len(frame_rows))
    distance_bottom = np.zeros(len(frame_rows))

    for metric, label, color in STAGES:
        time_values = np.array(
            [float(row.get(f"T_{metric}_data_observed_ms") or 0.0) for row in frame_rows]
        )
        distance_values = np.array(
            [
                float(row.get(f"D_{metric}_wall_integral_data_observed_m") or 0.0)
                for row in frame_rows
            ]
        )
        ax_time.bar(
            x,
            time_values,
            width=width,
            bottom=time_bottom,
            color=color,
            edgecolor="white",
            linewidth=0.35,
            label=label,
            zorder=3,
        )
        ax_distance.bar(
            x,
            distance_values,
            width=width,
            bottom=distance_bottom,
            color=color,
            edgecolor="white",
            linewidth=0.35,
            label=label,
            zorder=3,
        )
        time_bottom += time_values
        distance_bottom += distance_values

    # Censored traces are plotted only up to the collision and are hatched.
    for index, row in enumerate(frame_rows):
        if row["outcome_compatibility"] == "RIGHT_CENSORED_AT_COLLISION":
            ax_time.bar(
                x[index],
                time_bottom[index],
                width=width,
                facecolor="none",
                edgecolor="#3D3D3D",
                hatch="////",
                linewidth=1.0,
                zorder=5,
            )
            ax_distance.bar(
                x[index],
                distance_bottom[index],
                width=width,
                facecolor="none",
                edgecolor="#3D3D3D",
                hatch="////",
                linewidth=1.0,
                zorder=5,
            )
        elif row["endpoint_status"] == "NO_FUSION_LINEAGE":
            ax_time.scatter(
                x[index], 18, marker="X", s=90, color="#B22222", edgecolor="white", zorder=8
            )
            ax_distance.scatter(
                x[index], 0.22, marker="X", s=90, color="#B22222", edgecolor="white", zorder=8
            )

    ref = metadata["reference_p50_ms"]
    ax_time.axhline(ref, color="#1F2937", linestyle="--", linewidth=1.1, zorder=2)
    base_ticks = list(ax_time.get_yticks())
    ax_time.set_yticks(sorted(set([tick for tick in base_ticks if tick >= 0] + [ref])))
    ax_time.text(
        0.015,
        ref + 13,
        f"前17个目标帧 P50 = {ref:.3f} ms（research参考，非deadline）",
        color="#1F2937",
        fontsize=9.3,
    )

    t_phys_rel = metadata["t_phys"] - metadata["t_sample"]
    collision_rel = metadata["collision"] - metadata["t_sample"]
    for axis in (ax_time, ax_distance):
        axis.axvline(t_phys_rel, color="#626262", linestyle=":", linewidth=1.2, zorder=1)
        axis.axvline(collision_rel, color="#B22222", linestyle="--", linewidth=1.3, zorder=1)
        axis.grid(axis="y", color="#E7E7E7", linewidth=0.8, zorder=0)
        axis.ticklabel_format(style="plain", axis="both", useOffset=False)
        axis.spines[["top", "right"]].set_visible(False)

    ax_time.text(t_phys_rel + 0.018, 812, r"$t_{phys}$", color="#555555", fontsize=10)
    ax_time.text(collision_rel - 0.02, 812, "碰撞", color="#B22222", ha="right", fontsize=10)
    ax_time.set_ylim(0, 850)
    ax_distance.set_ylim(0, 10.2)
    ax_time.set_ylabel("至碰撞前可观测端点的阶段时间 (ms)")
    ax_distance.set_ylabel("同一阶段墙钟速度积分距离 (m)")
    ax_distance.set_xlabel(r"LiDAR source 相对 $t_{sample}$ 的时间 (s)")
    ax_time.set_title("A. 每个传感器输入的逐阶段时间", loc="left", fontweight="bold")
    ax_distance.set_title("B. 各阶段期间车辆实际行驶距离", loc="left", fontweight="bold")

    for row in frame_rows:
        if row["performance_state"] != "SELECTED_LATE_ANOMALY":
            continue
        rel = float(row["source_relative_t_sample_s"])
        latency = float(row["T_e2e_data_observed_pre_collision_ms"])
        distance = float(row["D_e2e_wall_integral_data_observed_m"])
        index = int(row["frame_index"])
        ax_time.annotate(
            f"F{index}: {latency:.1f} ms",
            (rel, latency),
            xytext=(4, 8),
            textcoords="offset points",
            fontsize=8.7,
            color="#8B1A1A",
            fontweight="bold",
        )
        ax_distance.annotate(
            f"{distance:.3f} m",
            (rel, distance),
            xytext=(4, 6),
            textcoords="offset points",
            fontsize=8.7,
            color="#8B1A1A",
            fontweight="bold",
        )

    completed = [
        row for row in frame_rows if row["outcome_compatibility"] == "PRE_COLLISION_COMPLETE"
    ]
    state_style = {
        "TARGET_PRESENT": ("#2C7FB8", "o", "目标 11 存在"),
        "SELECTED_LATE_ANOMALY": ("#B22222", "D", "三个晚期异常帧"),
        "TARGET_ABSENT": ("#777777", "o", "Fusion无目标 11"),
    }
    for state, (color, marker, label) in state_style.items():
        rows = [row for row in completed if row["performance_state"] == state]
        if not rows:
            continue
        ax_scatter.scatter(
            [float(row["T_e2e_data_observed_pre_collision_ms"]) for row in rows],
            [float(row["D_e2e_wall_integral_data_observed_m"]) for row in rows],
            s=72 if state == "SELECTED_LATE_ANOMALY" else 45,
            marker=marker,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            alpha=0.92,
            label=label,
            zorder=4,
        )
    for row in completed:
        if row["performance_state"] == "SELECTED_LATE_ANOMALY":
            ax_scatter.annotate(
                f"F{row['frame_index']}",
                (
                    float(row["T_e2e_data_observed_pre_collision_ms"]),
                    float(row["D_e2e_wall_integral_data_observed_m"]),
                ),
                xytext=(6, 5),
                textcoords="offset points",
                fontsize=9,
                color="#8B1A1A",
            )
    ax_scatter.axvline(ref, color="#1F2937", linestyle="--", linewidth=1.0)
    ax_scatter.set_xticks(sorted(set([0, 200, 400, 600, 800, round(ref, 3)])))
    ax_scatter.tick_params(axis="x", rotation=25)
    ax_scatter.set_xlim(280, 820)
    ax_scatter.set_ylim(4.0, 10.0)
    ax_scatter.set_xlabel("传感器 source→首个关联 Control (ms)")
    ax_scatter.set_ylabel("该区间墙钟速度积分距离 (m)")
    ax_scatter.set_title("C. 端到端时延与处理期间行驶距离", loc="left", fontweight="bold")
    ax_scatter.grid(color="#E7E7E7", linewidth=0.8)
    ax_scatter.spines[["top", "right"]].set_visible(False)
    ax_scatter.ticklabel_format(style="plain", axis="y", useOffset=False)
    ax_scatter.legend(loc="upper left", frameon=False)
    ax_scatter.text(
        0.04,
        0.04,
        "距离是处理区间的 observed exposure\n"
        "它随积分区间长度机械增长，不是独立因果检验",
        transform=ax_scatter.transAxes,
        fontsize=9.3,
        color="#444444",
        bbox={"boxstyle": "round,pad=0.5", "fc": "white", "ec": "#D2D2D2", "alpha": 0.92},
    )

    legend_handles = [Patch(facecolor=color, label=label) for _, label, color in STAGES]
    legend_handles.extend(
        [
            Patch(facecolor="white", edgecolor="#3D3D3D", hatch="////", label="碰撞右删失（仅画到碰撞）"),
            Line2D([0], [0], marker="X", color="none", markerfacecolor="#B22222", markeredgecolor="white", markersize=9, label="无 Fusion lineage"),
        ]
    )
    fig.legend(
        handles=legend_handles,
        loc="outside lower center",
        ncol=6,
        frameon=False,
        fontsize=9.3,
    )
    fig.suptitle(
        "1131 run：首次稳定观测障碍物后的逐帧时延—行驶距离暴露",
        fontsize=18,
        fontweight="bold",
        x=0.01,
        ha="left",
    )
    fig.text(
        0.01,
        0.955,
        "27个LiDAR输入；22个在碰撞前到首个Control，3个右删失，2个无Fusion lineage。"
        "距离统一为车速对墙钟时间梯形积分；逐帧区间重叠，不可求和当作总安全损失。",
        ha="left",
        va="top",
        fontsize=11,
        color="#333333",
    )

    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE_PNG, dpi=220, bbox_inches="tight", facecolor="white")
    fig.savefig(FIGURE_SVG, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def update_ledgers(frame_rows: list[dict], metadata: dict[str, float]) -> None:
    selected = [row for row in frame_rows if row["performance_state"] == "SELECTED_LATE_ANOMALY"]
    evidence_rows = [
        {
            "evidence_id": "EV.FRAMEWISE.CYBERDIST.1131",
            "run_id": RUN_ID,
            "layer": "L2/L3/L5",
            "metric": "framewise_source_to_first_control_latency_and_wall_distance",
            "value": (
                "|".join(
                    f"F{row['frame_index']}={float(row['T_e2e_data_observed_pre_collision_ms']):.3f}ms/"
                    f"{float(row['D_e2e_wall_integral_data_observed_m']):.3f}m"
                    for row in selected
                )
            ),
            "unit": "ms;m",
            "evidence_class": "TRACE_LINEAGE_PLUS_OBSERVED_DERIVED",
            "clock_domain": "sensor source wall epoch; trace-affine wall epoch; Localization wall epoch",
            "source_file": str(FRAME_TABLE),
            "source_locator": "frames 17-19; full 27-frame population retained",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131|C3.1131",
            "challenges_claim_ids": "C7.1131",
            "limitations": "distance is pipeline occupancy exposure, not attributable causal loss; overlapping frame intervals cannot be summed",
            "semantic_role": "FRAMEWISE_PERFORMANCE_EXPOSURE",
            "reference_type": "OBSERVED_WITH_RETROSPECTIVE_RESEARCH_REFERENCE",
            "distribution_scope": "single run; LiDAR inputs from t_sample to collision",
            "causal_lineage_grade": "A_TO_FIRST_CONTROL",
        },
        {
            "evidence_id": "EV.FRAMEWISE.CENSOR.1131",
            "run_id": RUN_ID,
            "layer": "P_OBSERVABILITY/L2/L3",
            "metric": "framewise_endpoint_availability",
            "value": "27 inputs;22 pre-collision complete;3 outcome-censored;2 no-Fusion lineage",
            "unit": "frames",
            "evidence_class": "TRACE_LINEAGE_WITH_EXPLICIT_MISSINGNESS",
            "clock_domain": "sensor source wall epoch and aligned trace wall epoch",
            "source_file": str(FRAME_TABLE),
            "source_locator": "all rows",
            "availability": "PARTIAL",
            "confidence": "HIGH",
            "supports_claim_ids": "P_OBSERVABILITY.1131",
            "challenges_claim_ids": "C7.1131",
            "limitations": "t_observable and per-frame physical effect are not identifiable",
            "semantic_role": "ENDPOINT_AVAILABILITY_AND_CENSORING",
            "reference_type": "OUTCOME_CENSORING",
            "distribution_scope": "single run; t_sample to collision",
            "causal_lineage_grade": "A_WHERE_AVAILABLE",
        },
    ]
    upsert_rows(TABLES / "evidence_ledger.csv", evidence_rows, "evidence_id")


def upsert_json_item(items: list[dict], item: dict, key: str = "id") -> None:
    for index, existing in enumerate(items):
        if existing.get(key) == item.get(key):
            items[index] = item
            return
    items.append(item)


def update_artifact(frame_rows: list[dict], metadata: dict[str, float]) -> None:
    """Extend the existing complete report artifact without removing prior sections."""
    path = REPORT / "artifact.json"
    if not path.exists():
        return
    artifact = json.loads(path.read_text(encoding="utf-8"))
    generated_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    artifact["manifest"]["generatedAt"] = generated_at
    artifact["snapshot"]["generatedAt"] = generated_at
    artifact["manifest"]["description"] = (
        "TCPS-PA v4.2：全run逐实例、Apollo链路映射、observed/model分离碰撞反事实，"
        "以及从首个稳定目标样本到碰撞的逐帧时延—行驶距离暴露。"
    )

    complete_dataset = []
    endpoint_dataset = []
    for row in frame_rows:
        endpoint_dataset.append(
            {
                "frame": int(row["frame_index"]),
                "source_rel_s": float(row["source_relative_t_sample_s"]),
                "parent_trace": row["parent_trace_id"],
                "fusion_trace": row["fusion_trace_id"] or "—",
                "target_11": "是" if row["target_11_present"] else "否",
                "endpoint_status": row["endpoint_status"],
                "outcome_status": row["outcome_compatibility"],
                "e2e_ms": (
                    round(float(row["T_e2e_data_observed_pre_collision_ms"]), 3)
                    if row["T_e2e_data_observed_pre_collision_ms"]
                    else None
                ),
                "distance_m": (
                    round(float(row["D_e2e_wall_integral_data_observed_m"]), 3)
                    if row["D_e2e_wall_integral_data_observed_m"]
                    else None
                ),
                "right_censored_ms": (
                    round(float(row["T_source_to_collision_right_censored_ms"]), 3)
                    if row["T_source_to_collision_right_censored_ms"]
                    else None
                ),
                "last_prefix": row["last_observed_prefix_endpoint"] or "—",
            }
        )
        if row["outcome_compatibility"] != "PRE_COLLISION_COMPLETE":
            continue
        complete_dataset.append(
            {
                "frame": int(row["frame_index"]),
                "source_rel_s": float(row["source_relative_t_sample_s"]),
                "fusion_trace": row["fusion_trace_id"],
                "target_11": "是" if row["target_11_present"] else "否",
                "performance_state": row["performance_state"],
                "e2e_ms": round(float(row["T_e2e_data_observed_pre_collision_ms"]), 3),
                "distance_m": round(float(row["D_e2e_wall_integral_data_observed_m"]), 3),
                "source_to_fusion_ms": round(
                    float(row["T_source_to_fusion_output_data_observed_ms"]), 3
                ),
                "fusion_to_prediction_ms": round(
                    float(row["T_fusion_to_prediction_output_data_observed_ms"]), 3
                ),
                "prediction_to_planning_ms": round(
                    float(row["T_prediction_to_planning_output_data_observed_ms"]), 3
                ),
                "planning_to_control_ms": round(
                    float(row["T_planning_output_to_first_control_output_data_observed_ms"]), 3
                ),
                "reference_p50_ms": round(metadata["reference_p50_ms"], 3),
            }
        )
    artifact["snapshot"]["datasets"]["framewise_performance_complete"] = complete_dataset
    artifact["snapshot"]["datasets"]["framewise_endpoint_availability"] = endpoint_dataset

    source_query = {
        "engine": "duckdb",
        "language": "sql",
        "sql": (
            "SELECT CAST(frame_index AS INTEGER) AS frame, "
            "CAST(source_relative_t_sample_s AS DOUBLE) AS source_rel_s, fusion_trace_id AS fusion_trace, "
            "performance_state, CAST(T_e2e_data_observed_pre_collision_ms AS DOUBLE) AS e2e_ms, "
            "CAST(D_e2e_wall_integral_data_observed_m AS DOUBLE) AS distance_m, "
            "CAST(T_source_to_fusion_output_data_observed_ms AS DOUBLE) AS source_to_fusion_ms, "
            "CAST(T_fusion_to_prediction_output_data_observed_ms AS DOUBLE) AS fusion_to_prediction_ms, "
            "CAST(T_prediction_to_planning_output_data_observed_ms AS DOUBLE) AS prediction_to_planning_ms, "
            "CAST(T_planning_output_to_first_control_output_data_observed_ms AS DOUBLE) AS planning_to_control_ms "
            f"FROM read_csv_auto('{FRAME_TABLE}', header=true, nullstr='') "
            "WHERE outcome_compatibility='PRE_COLLISION_COMPLETE' ORDER BY frame"
        ),
        "description": "从27个LiDAR source输入中选择碰撞前完成到首个关联Control的22帧，并列展示Grade-A软件时延和同区间墙钟速度积分距离。",
        "tables_used": [str(FRAME_TABLE), str(VELOCITY_FILE), str(LINEAGE_FILE)],
        "metric_definitions": [
            "e2e_ms：sensor source到同trace首个Control output的逐实例时延。",
            "distance_m：同一墙钟区间的Localization速度梯形积分；语义为pipeline occupancy exposure，非因果损失。",
            "碰撞后完成端点右删失；无Fusion lineage的输入保留缺失，不填补。",
        ],
    }
    source = {
        "id": "framewise_performance_analysis",
        "label": "1131逐帧时延—行驶距离暴露",
        "path": str(Path(__file__)),
        "query": source_query,
    }
    upsert_json_item(artifact["sources"], source)
    manifest_source = dict(source)
    upsert_json_item(artifact["manifest"]["sources"], manifest_source)

    chart = {
        "id": "framewise_latency_distance_scatter",
        "title": "逐帧端到端时延与处理期间行驶距离",
        "subtitle": (
            f"碰撞前完成的22帧；虚线参考={metadata['reference_p50_ms']:.3f} ms。"
            "距离是observed pipeline exposure，不是可相加的因果损失。"
        ),
        "type": "scatter",
        "dataset": "framewise_performance_complete",
        "sourceId": "framewise_performance_analysis",
        "intent": "relationship",
        "question": "当sensor→first Control时延增大时，该软件处理窗口内自车实际行驶了多远？",
        "rationale": "用散点保留每帧，避免均值掩盖三个尾部帧；颜色区分目标在场、晚期异常和目标缺失状态。",
        "encodings": {
            "x": {"field": "e2e_ms", "type": "quantitative", "label": "source→first Control", "unit": "ms"},
            "y": {"field": "distance_m", "type": "quantitative", "label": "墙钟速度积分距离", "unit": "m"},
            "color": {"field": "performance_state", "type": "nominal", "label": "帧状态"},
            "tooltip": [
                {"field": "frame", "type": "quantitative", "label": "帧"},
                {"field": "source_rel_s", "type": "quantitative", "label": "source相对t_sample", "unit": "s"},
                {"field": "fusion_trace", "type": "nominal", "label": "Fusion trace"},
                {"field": "source_to_fusion_ms", "type": "quantitative", "label": "Source→Fusion", "unit": "ms"},
                {"field": "prediction_to_planning_ms", "type": "quantitative", "label": "Prediction→Planning", "unit": "ms"},
            ],
        },
        "xAxisTitle": "sensor source→首个关联Control (ms)",
        "yAxisTitle": "处理期间行驶距离 (m)",
        "layout": "full",
        "maxRows": 22,
        "surface": {"viewMode": "both", "interactiveLegend": True},
    }
    upsert_json_item(artifact["manifest"]["charts"], chart)

    table = {
        "id": "framewise_endpoint_availability_table",
        "title": "从首个稳定目标样本到碰撞的27个LiDAR输入",
        "subtitle": "22帧碰撞前完成，3帧右删失，2帧无Fusion lineage；缺失值不填补。",
        "dataset": "framewise_endpoint_availability",
        "sourceId": "framewise_performance_analysis",
        "defaultSort": {"field": "frame", "direction": "asc"},
        "density": "dense",
        "columns": [
            {"field": "frame", "label": "帧", "type": "number"},
            {"field": "source_rel_s", "label": "source rel. s", "type": "number"},
            {"field": "target_11", "label": "目标11", "type": "text"},
            {"field": "e2e_ms", "label": "E2E ms", "type": "number"},
            {"field": "distance_m", "label": "距离 m", "type": "number"},
            {"field": "endpoint_status", "label": "端点状态", "type": "text"},
            {"field": "outcome_status", "label": "碰撞兼容性", "type": "text"},
            {"field": "right_censored_ms", "label": "到碰撞 ms", "type": "number"},
            {"field": "last_prefix", "label": "最后前缀端点", "type": "text"},
        ],
    }
    upsert_json_item(artifact["manifest"]["tables"], table)

    technical_summary = next(
        (block for block in artifact["manifest"]["blocks"] if block.get("id") == "technical_summary"),
        None,
    )
    addition = (
        "\n\n新增逐帧性能映射后，可以更直接地看到时延的车辆侧后果：三个晚期异常帧"
        "source→first Control为 **783.440/721.321/731.829 ms**，其软件处理窗口内自车实际行驶"
        "**9.519/8.520/8.234 m**。这证明尾部时延增加了新闭环结果可用前的空间暴露；"
        "但不能把重叠帧距离求和，也不能将它们直接命名为因果碰撞损失。"
    )
    if technical_summary is not None and "新增逐帧性能映射" not in technical_summary["body"]:
        technical_summary["body"] += addition

    new_blocks = [
        {
            "id": "framewise_performance_heading",
            "type": "markdown",
            "body": (
                "## 逐帧时延如何转化为车辆侧空间暴露\n\n"
                "每帧主端点是sensor source→同trace首个Control output；距离是同一墙钟区间的实测速度积分。"
                "它表示新帧处理期间已消耗的行驶空间，不是逐帧可归因物理响应。"
            ),
        },
        {"id": "framewise_performance_chart", "type": "chart", "chartId": "framewise_latency_distance_scatter"},
        {"id": "framewise_endpoint_table", "type": "table", "tableId": "framewise_endpoint_availability_table"},
        {
            "id": "framewise_performance_limits",
            "type": "markdown",
            "body": (
                "### 证据边界\n\n"
                "27个输入中，22帧在碰撞前完成到首个Control，3帧右删失，2帧无Fusion lineage。"
                "车辆本来就在运动，且Control以约10 ms复用轨迹，当前数据不能为每帧唯一绑定物理位置变化。"
            ),
        },
    ]
    blocks = artifact["manifest"]["blocks"]
    new_ids = {block["id"] for block in new_blocks}
    blocks[:] = [block for block in blocks if block.get("id") not in new_ids]
    insertion = next((index for index, block in enumerate(blocks) if block.get("id") == "limitations"), len(blocks))
    blocks[insertion:insertion] = new_blocks
    path.write_text(json.dumps(artifact, ensure_ascii=False, indent=2), encoding="utf-8")


def write_report(frame_rows: list[dict], metadata: dict[str, float]) -> None:
    selected = [row for row in frame_rows if row["performance_state"] == "SELECTED_LATE_ANOMALY"]
    complete = [row for row in frame_rows if row["outcome_compatibility"] == "PRE_COLLISION_COMPLETE"]
    no_fusion = [row for row in frame_rows if row["endpoint_status"] == "NO_FUSION_LINEAGE"]
    censored = [row for row in frame_rows if row["outcome_compatibility"] == "RIGHT_CENSORED_AT_COLLISION"]
    missing_prefix_text = "；".join(
        f"F{row['frame_index']} 停在 {row['last_observed_prefix_endpoint'].replace('perception.', '')} "
        f"（source后 {float(row['T_source_to_last_observed_prefix_ms']):.3f} ms，"
        f"期间行驶 {float(row['D_source_to_last_observed_prefix_wall_integral_m']):.3f} m）"
        for row in no_fusion
    )
    selected_table = "\n".join(
        "| {frame} | `{trace}` | {latency:.3f} | {distance:.3f} | {extra:.3f} | {dominant} |".format(
            frame=row["frame_index"],
            trace=row["fusion_trace_id"],
            latency=float(row["T_e2e_data_observed_pre_collision_ms"]),
            distance=float(row["D_e2e_wall_integral_data_observed_m"]),
            extra=float(row["D_excess_after_reference_wall_integral_research_m"]),
            dominant=(
                "Prediction→Planning"
                if row["fusion_trace_id"] == "17293896665878496499"
                else "Source→Fusion"
            ),
        )
        for row in selected
    )
    text = f"""# 1131 run 逐帧时延—自车行驶距离性能映射

## 结论

这次补充分析已经把“时延变大”映射成直接可观测的车辆性能量：从每个 LiDAR source 到该 trace 的首个 Control 输出，同时计算软件端到端时延和车速对墙钟时间的梯形积分距离。三个晚期异常帧在软件管线中分别驻留 **783.440 ms、721.321 ms、731.829 ms**，期间自车分别行驶 **9.519 m、8.520 m、8.234 m**。相比异常前 17 个目标帧的描述性 P50 **{metadata['reference_p50_ms']:.3f} ms**，同一 observed 速度轨迹上的参考条件化额外暴露为 **5.246 m、4.408 m、4.273 m**。

但必须限定语义：这些距离是“帧在自动驾驶软件管线内流动期间，自车实际前进的距离暴露”，不是已证明由该帧时延独立造成的可避免距离或碰撞损失。

![1131逐帧时延与行驶距离]({FIGURE_PNG})

## 分析人口与端点

- 分析起点是当前证据中“障碍物 11 首个稳定序列帧”的 source epoch，即 `t_sample={metadata['t_sample']:.6f}`。这不等于未标定的 `t_observable` 或物理需求起点 `t_demand`。
- 从 `t_sample` 到碰撞前共有 **{len(frame_rows)}** 个 LiDAR source 输入；其中 **{len(complete)}** 个在碰撞前完成到首个关联 Control，**{len(censored)}** 个被碰撞右删失，**{len(no_fusion)}** 个没有 Fusion lineage。
- 软件端点是通过 trace lineage 配对的 `sensor source→Fusion→Prediction→Planning→first Control output`，有端点时为 Grade A。
- 距离严格按 `D=∫v(t)dt_wall` 计算：Localization 速度在端点线性插值，然后按墙钟时间梯形积分。没有使用 CARLA 仿真帧数或仿真时间替代主距离。

## 三个异常帧的性能影响

| 帧 | Fusion trace | source→Control (ms) | 处理期间距离 (m) | P50后额外暴露 (m) | 主导阶段 |
|---:|---|---:|---:|---:|---|
{selected_table}

这张表揭示了两种不同的性能损害路径：帧 F17 主要卡在 `Prediction→Planning`，该段单独持续约 **480.043 ms**，期间自车行驶约 **5.521 m**；F18/F19 主要卡在 `source→Fusion`，该段分别为 **699.268/705.980 ms**，期间自车行驶约 **8.302/8.001 m**。这说明同样的端到端变慢，可以来自不同 Apollo 模块，并在物理上表现为新感知或新规划结果到达前的更长行驶暴露。

## 不能把“首次位置变化”当作逐帧物理端点

自车在所有这些帧到来前已经连续运动，因此“Control 后下一个位置样本变了”只能说明采样时序，不能证明是该帧引起的物理作用。本 run 中 Control 以约 10 ms 重复发布并复用 trace，同时缺少事件局部 Control payload、Bridge receive/apply 和执行器反馈，所以不能为 27 帧各自建立独立 `Control→physical` episode。

因此，本补充分析对“时延影响自动驾驶性能”的可证明结论是：**时延尖峰显著增加了新闭环结果发布前的行驶距离，并降低了碰撞前可用刷新机会**。但它不单独证明“这三帧的额外距离造成了碰撞”。

## 右删失、缺失和不可相加性

- 后续 3 帧的首个 Control 发布在碰撞后，主结果只画到碰撞并标注右删失；碰撞后的 trace 完成时间只作诊断字段，不补成碰撞前完整结果。
- 两个输入没有对应 Fusion lineage，在图中使用红色 `X` 保留，不做均值填补。已知的最后前缀端点是：{missing_prefix_text}。它们只能说明当前证据链在哪里停止，不足以断言是 drop、覆盖还是调度根因。
- 同时在管线中的帧大量重叠。20 个目标帧逐帧距离之和是 **{metadata['sum_target_distance_m']:.3f} m**，但它们的时间区间去重后仅覆盖 **{metadata['union_target_distance_m']:.3f} m**。前者重复计数，禁止当作“总损失距离”。

## 权威方法与 Apollo 10 实现核对

- Apollo 10 的 [`Header`](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/common_msgs/basic_msgs/header.proto) 明确区分消息发布时间与 `lidar_timestamp`，因此本分析使用 LiDAR source/trace anchor 作为帧起点，不用 Fusion 或 Planning 发布时间代替传感器采样时间。
- Apollo 10 源码显示 `lidar_timestamp` 沿 [Prediction](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/prediction/prediction_component.cc#L246-L285)、[Planning](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/planning/planning_component/planning_base.cc#L98-L108) 到 [Control](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/control/control_component/control_component.cc#L452-L495) 传播；这是逐帧 source→first Control lineage 的实现依据。Control 是 timer component，反复读取 latest trajectory，所以同一 trace 的后续发布是 reuse，不是新帧或独立物理 episode。
- [ECRTS 2023](https://doi.org/10.4230/LIPIcs.ECRTS.2023.10) 要求 effect/actuation 必须确实“based on”对应样本；这支持不把运动车辆的下一个位置样本冒充为逐帧物理效果。
- [Yi 2021](https://arxiv.org/abs/2106.04508) 将 sensor→actuator 时限与处理期间可行驶距离联系起来；[Koopman 2019](https://arxiv.org/abs/1911.01207) 则明确把 response delay 纳入安全距离。本报告因此将时延与墙钟积分距离并列，但没有在缺少合格dynamic contract和反事实replay时把它宣布为deadline debt或碰撞因果。

## 数据产物

- 逐帧主表：`{FRAME_TABLE}`
- 逐阶段长表：`{STAGE_TABLE}`
- 摘要表：`{SUMMARY_TABLE}`
- 图：`{FIGURE_PNG}` / `{FIGURE_SVG}`
- 复现脚本：`{Path(__file__)}`

## 下一证据门槛

若要把软件端到端暴露提升为逐动作的物理响应时间或可归因安全损失，需要在同一时钟基础上保存 Control payload/sequence、Bridge receive/apply、制动或加速度反馈，再按“命令语义发生独立变化”切分 action episode，而不是按每条 Control 发布消息切分。
"""
    REPORT.mkdir(parents=True, exist_ok=True)
    REPORT_FILE.write_text(text, encoding="utf-8")


def update_readme() -> None:
    readme = OUTPUT / "README.md"
    text = readme.read_text(encoding="utf-8") if readme.exists() else "# 1131 TCPS-PA v4.2\n"
    entry = "- [逐帧时延—自车行驶距离性能映射](report/framewise_latency_vehicle_performance_report.md)\n"
    if entry not in text:
        text = text.rstrip() + "\n" + entry
        readme.write_text(text, encoding="utf-8")


def main() -> None:
    frame_rows, stage_rows, summary_rows, metadata = build_tables()
    write_csv(FRAME_TABLE, frame_rows)
    write_csv(STAGE_TABLE, stage_rows)
    write_csv(SUMMARY_TABLE, summary_rows)
    draw_figure(frame_rows, metadata)
    update_ledgers(frame_rows, metadata)
    write_report(frame_rows, metadata)
    update_artifact(frame_rows, metadata)
    update_readme()
    validation = {
        "run_id": RUN_ID,
        "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "checks": {
            "frame_population_27": len(frame_rows) == 27,
            "pre_collision_complete_22": sum(row["outcome_compatibility"] == "PRE_COLLISION_COMPLETE" for row in frame_rows) == 22,
            "outcome_censored_3": sum(row["outcome_compatibility"] == "RIGHT_CENSORED_AT_COLLISION" for row in frame_rows) == 3,
            "no_fusion_2": sum(row["endpoint_status"] == "NO_FUSION_LINEAGE" for row in frame_rows) == 2,
            "target_frames_20": sum(bool(row["target_11_present"]) for row in frame_rows) == 20,
            "selected_frames_3": sum(row["performance_state"] == "SELECTED_LATE_ANOMALY" for row in frame_rows) == 3,
            "main_distance_only_pre_collision": all(
                bool(row["D_e2e_wall_integral_data_observed_m"])
                == (row["outcome_compatibility"] == "PRE_COLLISION_COMPLETE")
                for row in frame_rows
            ),
            "per_frame_physical_effect_not_imputed": all(
                row["physical_effect_episode_status"] == "NOT_IDENTIFIABLE_NO_UNIQUE_ACTION_EPISODE"
                for row in frame_rows
            ),
        },
        "reference_p50_ms": metadata["reference_p50_ms"],
    }
    validation["passed"] = all(validation["checks"].values())
    path = OUTPUT / "validation/framewise_performance_validation.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")
    if not validation["passed"]:
        raise RuntimeError(f"Validation failed: {validation['checks']}")
    print(json.dumps(validation, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
