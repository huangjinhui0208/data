#!/usr/bin/env python3
"""TCPS-PA v4.2 all-instance supplement for run 202607271131.

This script preserves the event-centered first causal chain while adding:
1) every trace-lineage software instance;
2) every Lidar Detection, Planning RunOnce, and Ground Detection instance;
3) research-only outlier segments;
4) Control reuse and physical-action episode identifiability audits.

Raw run files are read-only. All generated files stay in the v4.2 output tree.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


WORKSPACE = Path("/Users/huangjinhui/Desktop/萨卡班/data")
RUN_DIR = WORKSPACE / "第二次实验/300ms/202607271131"
OUTPUT = WORKSPACE / "output/second_experiment_1131_tcps_pa_v4_2_all_instances"
TABLES = OUTPUT / "tables"
FIGURES = OUTPUT / "figures"
REPORT = OUTPUT / "report"
VALIDATION = OUTPUT / "validation"
RUN_ID = "202607271131"

TRACE = RUN_DIR / "trace"
EVENTS = TRACE / "events"
CONTEXT = TRACE / "message_context"
ANCHOR_FILE = TRACE / "trace_anchor/perception.476004.csv"
FUSION_INPUT_FILE = TRACE / "fusion_inputs/perception.multi_sensor_fusion.476004.csv"

FIRST_CAUSAL_TRACE = "17293896665878496482"
FIRST_CAUSAL_PARENT = "72131690813719833"
SELECTED_EXTREMES = {
    "lidar_detection_processing": "72131690813719851",
    "planning_runonce": "17293896665878496499",
    "ground_detection_processing": "72131690813719852",
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = []
        seen: set[str] = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    fields.append(key)
                    seen.add(key)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def distribution(values: list[float]) -> dict[str, float | int]:
    median = statistics.median(values)
    mad = statistics.median(abs(value - median) for value in values)
    p25 = quantile(values, 0.25)
    p75 = quantile(values, 0.75)
    return {
        "n": len(values),
        "mean_ms": statistics.mean(values),
        "std_ms": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min_ms": min(values),
        "p50_ms": median,
        "p90_ms": quantile(values, 0.90),
        "p95_ms": quantile(values, 0.95),
        "p99_ms": quantile(values, 0.99),
        "max_ms": max(values),
        "mad_ms": mad,
        "iqr_ms": p75 - p25,
        "research_threshold_median_plus_6mad_ms": median + 6.0 * mad,
    }


def distribution_by_window(
    rows: list[dict], *, availability_required: bool = False
) -> list[dict]:
    grouped: dict[tuple[str, str], list[float]] = defaultdict(list)
    for row in rows:
        if availability_required and row.get("availability") != "AVAILABLE":
            continue
        value = row.get("duration_ms", "")
        if value == "":
            continue
        grouped[(str(row["metric_name"]), str(row["event_window"]))].append(
            float(value)
        )
    output: list[dict] = []
    for (metric_name, window), values in sorted(grouped.items()):
        output.append(
            {
                "run_id": RUN_ID,
                "metric_name": metric_name,
                "event_window": window,
                **distribution(values),
                "aggregation_scope": "within-run event window",
                "experimental_replication_note": "instances are temporal samples, not independent runs",
            }
        )
    return output


def choose_font() -> str:
    available = {font.name for font in fm.fontManager.ttflist}
    for candidate in ("Arial Unicode MS", "Songti SC", "DejaVu Sans"):
        if candidate in available:
            return candidate
    return "DejaVu Sans"


def add_blossom(fig: plt.Figure) -> None:
    center_x, center_y = 0.973, 0.971
    for dx, dy, angle in ((0, 0.008, 0), (0.008, 0, 90), (0, -0.008, 0), (-0.008, 0, 90)):
        fig.add_artist(
            Ellipse(
                (center_x + dx, center_y + dy),
                0.010,
                0.019,
                angle=angle,
                transform=fig.transFigure,
                facecolor="#DDEEFF",
                edgecolor="#0169CC",
                linewidth=0.7,
                zorder=20,
            )
        )


def iso(epoch_s: float) -> str:
    return datetime.fromtimestamp(epoch_s, ZoneInfo("Asia/Shanghai")).isoformat(
        timespec="microseconds"
    )


def first(rows: list[dict[str, str]], **conditions: str) -> dict[str, str]:
    for row in rows:
        if all(row.get(key) == value for key, value in conditions.items()):
            return row
    raise KeyError(conditions)


def event_window(start_s: float, end_s: float, t1: float, t2: float, collision: float) -> str:
    if end_s < t1:
        return "PRE_T_SAMPLE"
    if start_s < t1 <= end_s:
        return "CROSSES_T_SAMPLE"
    if end_s <= t2:
        return "T_SAMPLE_TO_T_PHYS"
    if start_s < t2 <= end_s:
        return "CROSSES_T_PHYS"
    if end_s <= collision:
        return "POST_T_PHYS_PRE_COLLISION"
    if start_s < collision <= end_s:
        return "CROSSES_COLLISION"
    return "POST_COLLISION"


def load_endpoint_map(path: Path, edge_or_phase: str, value: str) -> dict[str, int]:
    result: dict[str, int] = {}
    for row in read_csv(path):
        if row[edge_or_phase] == value:
            trace_id = row["trace_id"]
            mono_ns = int(row["mono_ns"])
            result[trace_id] = min(mono_ns, result.get(trace_id, mono_ns))
    return result


def build_clock_map(anchors: dict[str, dict[str, str]]) -> tuple[float, float, dict[str, float]]:
    points = [
        (
            int(row["preproc_enter_ns"]) / 1e9,
            float(row["sensor_time_sec"]) + float(row["ingress_ms"]) / 1000.0,
        )
        for row in anchors.values()
        if row["sensor_kind"] == "lidar"
    ]
    x = np.asarray([item[0] for item in points], dtype=float)
    y = np.asarray([item[1] for item in points], dtype=float)
    slope, intercept = np.polyfit(x, y, 1)
    residual_ms = np.abs((slope * x + intercept - y) * 1000.0)
    audit = {
        "anchor_count": len(points),
        "slope": float(slope),
        "drift_ppm": float((slope - 1.0) * 1e6),
        "median_abs_residual_ms": float(np.median(residual_ms)),
        "p95_abs_residual_ms": float(np.quantile(residual_ms, 0.95)),
        "max_abs_residual_ms": float(np.max(residual_ms)),
    }
    return float(slope), float(intercept), audit


def mono_to_wall(mono_ns: int, slope: float, intercept: float) -> float:
    return slope * (mono_ns / 1e9) + intercept


def build_lineage_instances(
    anchors: dict[str, dict[str, str]],
    slope: float,
    intercept: float,
    t1: float,
    t2: float,
    collision: float,
) -> tuple[list[dict], list[dict]]:
    main_parent: dict[str, str] = {}
    for row in read_csv(FUSION_INPUT_FILE):
        if row["is_main_sensor"] == "1" and row["sensor"] == "velodyne64":
            main_parent[row["fusion_trace_id"]] = row["parent_trace_id"]

    fusion_out = load_endpoint_map(
        CONTEXT / "perception.multi_sensor_fusion.476004.csv", "edge", "out"
    )
    prediction_out = load_endpoint_map(CONTEXT / "prediction.475996.csv", "edge", "out")
    planning_out = load_endpoint_map(CONTEXT / "planning.476000.csv", "edge", "out")
    control_rows = [
        row
        for row in read_csv(CONTEXT / "control.476002.csv")
        if row["edge"] == "out"
    ]
    control_by_trace: dict[str, list[int]] = defaultdict(list)
    for row in control_rows:
        control_by_trace[row["trace_id"]].append(int(row["mono_ns"]))

    target_trace_ids = {
        row["trace_id"] for row in read_csv(TABLES / "target_freshness_timeline.csv")
    }
    stage_defs = [
        "source_to_fusion_output",
        "fusion_to_prediction_output",
        "prediction_to_planning_output",
        "planning_output_to_first_control_output",
        "source_to_first_control_output",
    ]
    rows: list[dict] = []

    for fusion_trace_id in sorted(main_parent, key=int):
        parent_trace_id = main_parent[fusion_trace_id]
        anchor = anchors.get(parent_trace_id)
        fout = fusion_out.get(fusion_trace_id)
        pred = prediction_out.get(fusion_trace_id)
        plan = planning_out.get(fusion_trace_id)
        controls = sorted(control_by_trace.get(fusion_trace_id, []))
        first_control = None
        if plan is not None:
            first_control = next((value for value in controls if value >= plan), None)

        source_wall = float(anchor["sensor_time_sec"]) if anchor else None
        endpoint_map: dict[str, tuple[float | int | None, float | int | None, str]] = {
            "source_to_fusion_output": (source_wall, fout, "source_wall_to_mono"),
            "fusion_to_prediction_output": (fout, pred, "mono_to_mono"),
            "prediction_to_planning_output": (pred, plan, "mono_to_mono"),
            "planning_output_to_first_control_output": (plan, first_control, "mono_to_mono"),
            "source_to_first_control_output": (source_wall, first_control, "source_wall_to_mono"),
        }

        for metric_name in stage_defs:
            start, end, basis = endpoint_map[metric_name]
            availability = "AVAILABLE"
            missing_reason = ""
            start_wall = None
            end_wall = None
            duration_ms = None
            if basis == "source_wall_to_mono" and anchor is None:
                availability = "UNAVAILABLE"
                missing_reason = "MISSING_PARENT_TRACE_ANCHOR"
            elif start is None or end is None:
                availability = "UNAVAILABLE"
                missing_reason = "MISSING_STAGE_ENDPOINT"
            elif basis == "source_wall_to_mono":
                end_wall = mono_to_wall(int(end), slope, intercept)
                start_wall = float(start)
                if metric_name == "source_to_fusion_output":
                    duration_ms = float(anchor["ingress_ms"]) + (
                        int(end) - int(anchor["preproc_enter_ns"])
                    ) / 1e6
                    end_wall = start_wall + duration_ms / 1000.0
                else:
                    duration_ms = (end_wall - start_wall) * 1000.0
            else:
                start_wall = mono_to_wall(int(start), slope, intercept)
                end_wall = mono_to_wall(int(end), slope, intercept)
                duration_ms = (int(end) - int(start)) / 1e6

            window = "UNAVAILABLE"
            if start_wall is not None and end_wall is not None:
                window = event_window(start_wall, end_wall, t1, t2, collision)
            rows.append(
                {
                    "run_id": RUN_ID,
                    "fusion_trace_id": fusion_trace_id,
                    "parent_trace_id": parent_trace_id,
                    "metric_name": metric_name,
                    "source_wall_s": source_wall if source_wall is not None else "",
                    "start_wall_s": start_wall if start_wall is not None else "",
                    "end_wall_s": end_wall if end_wall is not None else "",
                    "start_relative_t_sample_s": (start_wall - t1) if start_wall is not None else "",
                    "end_relative_t_sample_s": (end_wall - t1) if end_wall is not None else "",
                    "duration_ms": duration_ms if duration_ms is not None else "",
                    "control_output_count_for_trace": len(controls),
                    "event_window": window,
                    "is_first_causal_instance": fusion_trace_id == FIRST_CAUSAL_TRACE,
                    "is_target_11_instance": fusion_trace_id in target_trace_ids,
                    "availability": availability,
                    "missing_reason": missing_reason,
                    "lineage_grade": "A" if availability == "AVAILABLE" else "UNKNOWN",
                    "clock_basis": "trace mono_ns with affine source-wall anchor",
                }
            )

    summary_rows: list[dict] = []
    for metric_name in stage_defs:
        metric_rows = [
            row
            for row in rows
            if row["metric_name"] == metric_name and row["availability"] == "AVAILABLE"
        ]
        values = [float(row["duration_ms"]) for row in metric_rows]
        stats = distribution(values)
        first_row = next(
            (
                row
                for row in metric_rows
                if row["fusion_trace_id"] == FIRST_CAUSAL_TRACE
            ),
            None,
        )
        first_value = float(first_row["duration_ms"]) if first_row else math.nan
        threshold = float(stats["research_threshold_median_plus_6mad_ms"])
        summary_rows.append(
            {
                "run_id": RUN_ID,
                "metric_name": metric_name,
                "expected_instance_count": len(main_parent),
                "available_instance_count": len(values),
                "missing_instance_count": len(main_parent) - len(values),
                **stats,
                "research_outlier_count": sum(value > threshold for value in values),
                "research_outlier_rate": sum(value > threshold for value in values)
                / len(values),
                "first_causal_value_ms": first_value,
                "first_causal_percentile_rank": sum(value <= first_value for value in values)
                / len(values)
                if values and math.isfinite(first_value)
                else "",
                "aggregation_scope": "within-run repeated trace instances; not experimental replicates",
                "threshold_provenance": "RESEARCH median+6*MAD; not a contract",
            }
        )
    return rows, summary_rows


def paired_component_rows(
    metric_name: str,
    event_file: Path,
    start_phase: str,
    end_phase: str,
    slope: float,
    intercept: float,
    t1: float,
    t2: float,
    collision: float,
) -> list[dict]:
    by_trace: dict[str, dict[str, list[dict[str, str]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for row in read_csv(event_file):
        by_trace[row["trace_id"]][row["phase"]].append(row)

    rows: list[dict] = []
    for trace_id, phases in by_trace.items():
        if start_phase not in phases or end_phase not in phases:
            continue
        start = min(phases[start_phase], key=lambda row: int(row["mono_ns"]))
        end = min(phases[end_phase], key=lambda row: int(row["mono_ns"]))
        start_ns = int(start["mono_ns"])
        end_ns = int(end["mono_ns"])
        if end_ns < start_ns:
            continue
        start_wall = mono_to_wall(start_ns, slope, intercept)
        end_wall = mono_to_wall(end_ns, slope, intercept)
        rows.append(
            {
                "run_id": RUN_ID,
                "metric_name": metric_name,
                "trace_id": trace_id,
                "start_phase": start_phase,
                "end_phase": end_phase,
                "start_mono_ns": start_ns,
                "end_mono_ns": end_ns,
                "start_wall_s": start_wall,
                "end_wall_s": end_wall,
                "start_wall_iso": iso(start_wall),
                "start_relative_t_sample_s": start_wall - t1,
                "end_relative_t_sample_s": end_wall - t1,
                "duration_ms": (end_ns - start_ns) / 1e6,
                "pid": start["pid"],
                "tid": start["tid"],
                "event_window": event_window(start_wall, end_wall, t1, t2, collision),
                "source_event_file": str(event_file),
                "clock_basis": "shared Orin monotonic_ns; affine wall anchor",
            }
        )
    return rows


def build_component_instances(
    slope: float,
    intercept: float,
    t1: float,
    t2: float,
    collision: float,
) -> tuple[list[dict], list[dict]]:
    specs = [
        (
            "lidar_detection_processing",
            EVENTS / "perception.lidar_detection.476004.csv",
            "proc_enter",
            "output_pub",
        ),
        (
            "planning_runonce",
            EVENTS / "planning.476000.csv",
            "runonce_enter",
            "runonce_exit",
        ),
        (
            "ground_detection_processing",
            EVENTS / "perception.pointcloud_ground_detection.476004.csv",
            "proc_enter",
            "output_pub",
        ),
    ]
    rows: list[dict] = []
    summaries: list[dict] = []
    for metric_name, file_path, start_phase, end_phase in specs:
        metric_rows = paired_component_rows(
            metric_name,
            file_path,
            start_phase,
            end_phase,
            slope,
            intercept,
            t1,
            t2,
            collision,
        )
        values = [float(row["duration_ms"]) for row in metric_rows]
        stats = distribution(values)
        threshold = float(stats["research_threshold_median_plus_6mad_ms"])
        p99 = float(stats["p99_ms"])
        for row in metric_rows:
            duration = float(row["duration_ms"])
            robust = duration > threshold
            extreme = duration >= p99
            row["research_threshold_ms"] = threshold
            row["p99_reference_ms"] = p99
            row["research_outlier"] = robust
            row["p99_tail"] = extreme
            row["anomaly_class"] = (
                "ROBUST_AND_P99"
                if robust and extreme
                else "ROBUST_ONLY"
                if robust
                else "P99_ONLY"
                if extreme
                else "WITHIN_REFERENCE"
            )
            row["threshold_provenance"] = "RESEARCH median+6*MAD and empirical P99"
            row["qualification"] = "RESEARCH_ONLY_NOT_A_CONTRACT"
        selected = next(
            row for row in metric_rows if row["trace_id"] == SELECTED_EXTREMES[metric_name]
        )
        summaries.append(
            {
                "run_id": RUN_ID,
                "metric_name": metric_name,
                **stats,
                "research_outlier_count": sum(bool(row["research_outlier"]) for row in metric_rows),
                "research_outlier_rate": sum(bool(row["research_outlier"]) for row in metric_rows)
                / len(metric_rows),
                "p99_tail_count": sum(bool(row["p99_tail"]) for row in metric_rows),
                "selected_max_trace_id": selected["trace_id"],
                "selected_max_ms": selected["duration_ms"],
                "aggregation_scope": "all paired instances in run 1131",
                "experimental_replication_note": "frames are temporal samples, not independent runs",
                "threshold_provenance": "RESEARCH_ONLY_NOT_A_CONTRACT",
            }
        )
        rows.extend(metric_rows)
    rows.sort(key=lambda row: (float(row["start_wall_s"]), row["metric_name"]))
    return rows, summaries


def merge_segments(rows: list[dict], merge_gap_s: float = 0.100) -> list[dict]:
    intervals = sorted(
        [
            {
                "start": float(row["start_relative_t_sample_s"]),
                "end": float(row["end_relative_t_sample_s"]),
                "metric": row["metric_name"],
                "trace_id": row["trace_id"],
                "duration_ms": float(row["duration_ms"]),
                "event_window": row["event_window"],
            }
            for row in rows
            if bool(row["research_outlier"])
        ],
        key=lambda item: item["start"],
    )

    def merge(items: list[dict], scope: str) -> list[dict]:
        merged: list[dict] = []
        for item in items:
            if not merged or item["start"] - merged[-1]["end"] > merge_gap_s:
                merged.append(
                    {
                        "scope": scope,
                        "start": item["start"],
                        "end": item["end"],
                        "members": [item],
                    }
                )
            else:
                merged[-1]["end"] = max(merged[-1]["end"], item["end"])
                merged[-1]["members"].append(item)
        return merged

    component_groups: dict[str, list[dict]] = defaultdict(list)
    for item in intervals:
        component_groups[item["metric"]].append(item)
    segments: list[dict] = []
    raw_segments: list[dict] = []
    for metric, items in component_groups.items():
        raw_segments.extend(merge(items, f"COMPONENT:{metric}"))
    raw_segments.extend(merge(intervals, "CROSS_COMPONENT"))
    raw_segments.sort(key=lambda item: (item["start"], item["scope"]))

    for index, segment in enumerate(raw_segments, start=1):
        metrics = sorted({item["metric"] for item in segment["members"]})
        trace_ids = [item["trace_id"] for item in segment["members"]]
        classification = (
            "MULTI_COMPONENT_CONCURRENT"
            if len(metrics) >= 2
            else "SINGLE_COMPONENT_BURST"
            if len(segment["members"]) >= 2
            else "SINGLE_INSTANCE"
        )
        segments.append(
            {
                "run_id": RUN_ID,
                "segment_id": f"SEG{index:02d}",
                "scope": segment["scope"],
                "start_relative_t_sample_s": segment["start"],
                "end_relative_t_sample_s": segment["end"],
                "span_ms": (segment["end"] - segment["start"]) * 1000.0,
                "member_instance_count": len(segment["members"]),
                "component_count": len(metrics),
                "metrics": "|".join(metrics),
                "trace_ids": "|".join(trace_ids),
                "max_member_duration_ms": max(
                    item["duration_ms"] for item in segment["members"]
                ),
                "classification": classification,
                "merge_gap_ms": merge_gap_s * 1000.0,
                "threshold_provenance": "instances > within-run median+6*MAD; RESEARCH",
                "qualification": "RESEARCH_ONLY_NOT_A_CONTRACT",
            }
        )
    return segments


def build_control_audit(
    slope: float,
    intercept: float,
    t1: float,
    t2: float,
) -> tuple[list[dict], list[dict]]:
    control_out = [
        row
        for row in read_csv(CONTEXT / "control.476002.csv")
        if row["edge"] == "out"
    ]
    by_trace: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in control_out:
        by_trace[row["trace_id"]].append(row)
    reuse_rows: list[dict] = []
    for trace_id, group in by_trace.items():
        monos = sorted(int(row["mono_ns"]) for row in group)
        reuse_rows.append(
            {
                "run_id": RUN_ID,
                "trace_id": trace_id,
                "control_output_count": len(monos),
                "first_output_mono_ns": monos[0],
                "last_output_mono_ns": monos[-1],
                "first_output_wall_s": mono_to_wall(monos[0], slope, intercept),
                "last_output_wall_s": mono_to_wall(monos[-1], slope, intercept),
                "reuse_span_ms": (monos[-1] - monos[0]) / 1e6,
                "primary_parent_trace_id": group[0]["primary_parent_trace_id"],
                "data_ts_ns": group[0]["data_ts_ns"],
                "is_first_causal_trace": trace_id == FIRST_CAUSAL_TRACE,
                "is_selected_post_t2_trace": trace_id
                in {
                    "17293896665878496499",
                    "17293896665878496500",
                    "17293896665878496501",
                },
            }
        )
    reuse_rows.sort(key=lambda row: int(row["trace_id"]))

    event_rows = read_csv(TABLES / "event_timeline.csv")
    control_wall = float(first(event_rows, event_id="E06")["t_wall_s"])
    observed = read_csv(TABLES / "run_level_observed.csv")[0]
    lower_tr_ms = float(observed["T_R_sample_lower_bracket_ms"])
    upper_tr_ms = float(observed["T_R_sample_upper_bracket_ms"])
    source_to_control_ms = (control_wall - t1) * 1000.0
    strict_episode_count = 0
    analysis_episode_count = 1

    audit_rows = [
        {
            "run_id": RUN_ID,
            "audit_item": "CONTROL_MESSAGE_POPULATION",
            "total_control_output_messages": len(control_out),
            "unique_control_trace_ids": len(by_trace),
            "max_outputs_per_trace": max(len(group) for group in by_trace.values()),
            "median_outputs_per_trace": statistics.median(
                len(group) for group in by_trace.values()
            ),
            "strict_physical_action_episode_count": strict_episode_count,
            "analysis_defined_episode_count": analysis_episode_count,
            "availability": "AVAILABLE_FOR_SOFTWARE_REUSE_ONLY",
            "lineage_grade": "A_TO_CONTROL_ONLY",
            "notes": "Repeated Control outputs are not independent physical-response episodes.",
        },
        {
            "run_id": RUN_ID,
            "audit_item": "EP_INITIAL_TARGET_BRAKE",
            "episode_origin": "first stable target-11 causal trace and Planning STOP",
            "fusion_trace_id": FIRST_CAUSAL_TRACE,
            "parent_trace_id": FIRST_CAUSAL_PARENT,
            "control_output_wall_s": control_wall,
            "bridge_apply_wall_s": "",
            "physical_effect_wall_s": t2,
            "control_to_physical_reported_upper_ms": (t2 - control_wall) * 1000.0,
            "control_to_physical_lower_bracket_ms": lower_tr_ms - source_to_control_ms,
            "control_to_physical_upper_bracket_ms": upper_tr_ms - source_to_control_ms,
            "availability": "PARTIAL_EVENT_LEVEL_ASSOCIATION",
            "lineage_grade": "C_CONTROL_TO_PHYSICAL",
            "notes": "Event-local command payload and Bridge apply row are absent; no per-episode latency distribution is identifiable.",
        },
        {
            "run_id": RUN_ID,
            "audit_item": "CONTROL_TO_PHYSICAL_EPISODE_DISTRIBUTION",
            "availability": "UNAVAILABLE",
            "lineage_grade": "UNKNOWN",
            "missing_reason": "log_all_delayed_commands=false; no record; no event-local payload/apply-to-feedback mapping",
            "notes": "Do not calculate t_phys minus every Control publication; that would be many-to-one pseudoreplication.",
        },
    ]
    return reuse_rows, audit_rows


def plot_component_scatter(
    rows: list[dict],
    summaries: list[dict],
    segments: list[dict],
    t1: float,
    t2: float,
    collision: float,
) -> None:
    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "text.color": "#0D0D0D",
            "axes.labelcolor": "#0D0D0D",
            "xtick.color": "#5D5D5D",
            "ytick.color": "#5D5D5D",
        }
    )
    order = [
        "lidar_detection_processing",
        "planning_runonce",
        "ground_detection_processing",
    ]
    labels = {
        "lidar_detection_processing": "Lidar Detection",
        "planning_runonce": "Planning RunOnce",
        "ground_detection_processing": "Ground Detection",
    }
    colors = {
        "lidar_detection_processing": "#0169CC",
        "planning_runonce": "#8046D9",
        "ground_detection_processing": "#E25507",
    }
    summary_map = {row["metric_name"]: row for row in summaries}
    min_x = min(float(row["start_relative_t_sample_s"]) for row in rows)
    max_x = max(float(row["start_relative_t_sample_s"]) for row in rows)
    t2_rel = t2 - t1
    collision_rel = collision - t1
    zoom_limits = (-1.8, collision_rel + 0.35)

    fig, axes = plt.subplots(
        3,
        2,
        figsize=(16.8, 10.5),
        dpi=180,
        sharey="row",
        gridspec_kw={"width_ratios": [1.35, 1.0], "hspace": 0.18, "wspace": 0.10},
    )
    fig.subplots_adjust(left=0.12, right=0.97, top=0.875, bottom=0.11)
    fig.suptitle(
        "1131 run：三个模块全部实例的执行时间散点图",
        x=0.095,
        y=0.965,
        ha="left",
        fontsize=20,
        fontweight="bold",
    )
    fig.text(
        0.095,
        0.925,
        "横轴为实例开始时间相对 t_sample；纵轴为单实例执行时间（线性刻度，单位 ms）。彩色虚线及纵轴彩色数值为 median+6×MAD research筛查阈值。",
        ha="left",
        fontsize=11.2,
        color="#5D5D5D",
    )
    add_blossom(fig)

    multi_segments = [
        segment
        for segment in segments
        if segment["scope"] == "CROSS_COMPONENT"
        and int(segment["component_count"]) >= 2
    ]
    linear_y_max = math.ceil(
        max(float(row["duration_ms"]) for row in rows) / 100.0
    ) * 100.0
    linear_y_ticks = np.arange(0.0, linear_y_max + 0.1, 100.0)

    for row_index, metric in enumerate(order):
        metric_rows = [row for row in rows if row["metric_name"] == metric]
        x = np.asarray([float(row["start_relative_t_sample_s"]) for row in metric_rows])
        y = np.asarray([float(row["duration_ms"]) for row in metric_rows])
        robust = np.asarray([bool(row["research_outlier"]) for row in metric_rows])
        p99_only = np.asarray(
            [bool(row["p99_tail"]) and not bool(row["research_outlier"]) for row in metric_rows]
        )
        summary = summary_map[metric]
        threshold = float(summary["research_threshold_median_plus_6mad_ms"])
        median = float(summary["p50_ms"])
        component_segments = [
            segment
            for segment in segments
            if segment["scope"] == f"COMPONENT:{metric}"
        ]

        for col_index, axis in enumerate(axes[row_index]):
            for segment in component_segments:
                axis.axvspan(
                    float(segment["start_relative_t_sample_s"]),
                    float(segment["end_relative_t_sample_s"]),
                    color=colors[metric],
                    alpha=0.10,
                    zorder=0,
                )
            for segment in multi_segments:
                axis.axvspan(
                    float(segment["start_relative_t_sample_s"]),
                    float(segment["end_relative_t_sample_s"]),
                    color="#F4C95D",
                    alpha=0.20,
                    zorder=0,
                )
            axis.scatter(
                x[~robust & ~p99_only],
                y[~robust & ~p99_only],
                s=16,
                facecolor="white",
                edgecolor="#8F8F8F",
                linewidth=0.65,
                alpha=0.72,
                zorder=2,
            )
            if np.any(p99_only):
                axis.scatter(
                    x[p99_only],
                    y[p99_only],
                    s=42,
                    marker="D",
                    facecolor="white",
                    edgecolor=colors[metric],
                    linewidth=1.3,
                    zorder=4,
                )
            axis.scatter(
                x[robust],
                y[robust],
                s=48,
                marker="o",
                facecolor=colors[metric],
                edgecolor="#0D0D0D",
                linewidth=0.7,
                zorder=5,
            )
            axis.axhline(
                threshold,
                color=colors[metric],
                linewidth=1.2,
                linestyle=(0, (6, 4)),
                zorder=1,
            )
            axis.axhline(
                median,
                color="#5D5D5D",
                linewidth=0.9,
                linestyle=(0, (2, 3)),
                zorder=1,
            )
            axis.set_ylim(0.0, linear_y_max)
            axis.set_yticks(linear_y_ticks)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
            axis.grid(axis="y", color="#D9D9D9", linewidth=0.65, alpha=0.75)
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            axis.set_xlim((min_x - 0.5, max_x + 0.5) if col_index == 0 else zoom_limits)
            if col_index == 1:
                axis.axvspan(collision_rel, zoom_limits[1], color="#E8E8E8", alpha=0.5)
                for event_x, event_label, style in (
                    (0.0, "t_sample", "solid"),
                    (t2_rel, "t_phys", "dashed"),
                    (collision_rel, "collision", "dashdot"),
                ):
                    axis.axvline(event_x, color="#0D0D0D", linewidth=1.0, linestyle=style)
                if row_index == 0:
                    event_label_y = linear_y_max - 12.0
                    axis.text(0.0, event_label_y, "t_sample", ha="left", va="top", fontsize=9)
                    axis.text(t2_rel, event_label_y, "t_phys", ha="left", va="top", fontsize=9)
                    axis.text(collision_rel, event_label_y, "碰撞", ha="right", va="top", fontsize=9)
            if row_index == 0:
                axis.set_title(
                    "全run" if col_index == 0 else "关键窗口放大",
                    loc="left",
                    fontsize=12.5,
                    fontweight="bold",
                )
            if col_index == 0:
                axis.set_ylabel(f"{labels[metric]}\n执行时间（ms）", fontsize=10.5)
                axis.plot(
                    [-0.008, 0.0],
                    [threshold, threshold],
                    transform=axis.get_yaxis_transform(),
                    color=colors[metric],
                    linewidth=1.6,
                    clip_on=False,
                    zorder=6,
                )
                axis.annotate(
                    f"{threshold:.3f}",
                    xy=(0.0, threshold),
                    xycoords=axis.get_yaxis_transform(),
                    xytext=(-7, 3),
                    textcoords="offset points",
                    ha="right",
                    va="bottom",
                    fontsize=8.5,
                    color=colors[metric],
                    fontweight="bold",
                    bbox={"boxstyle": "square,pad=0.12", "fc": "white", "ec": "none", "alpha": 0.94},
                    zorder=7,
                )
                axis.text(
                    0.012,
                    0.94,
                    f"n={int(summary['n'])}  P50={median:.3f} ms  阈值={threshold:.3f} ms",
                    transform=axis.transAxes,
                    ha="left",
                    va="top",
                    fontsize=9.2,
                    color="#5D5D5D",
                )
        selected = next(
            row
            for row in metric_rows
            if row["trace_id"] == SELECTED_EXTREMES[metric]
        )
        axes[row_index, 1].annotate(
            f"{float(selected['duration_ms']):.3f} ms",
            xy=(
                float(selected["start_relative_t_sample_s"]),
                float(selected["duration_ms"]),
            ),
            xytext=(8, 9),
            textcoords="offset points",
            fontsize=9.5,
            color=colors[metric],
            fontweight="bold",
            arrowprops={"arrowstyle": "-", "color": colors[metric], "lw": 0.9},
        )

    axes[-1, 0].set_xlabel("实例开始时间相对 t_sample（s）", fontsize=11)
    axes[-1, 1].set_xlabel("实例开始时间相对 t_sample（s）", fontsize=11)
    fig.text(
        0.095,
        0.048,
        "纵轴彩色数值：该模块的 median+6×MAD research筛查阈值；实心圆：超过该阈值的实例；"
        "空心菱形：仅进入经验P99尾部；灰色空心点：其余实例。"
        " 黄色阴影为多模块并发异常段，不能单凭时间重叠认定共同根因。",
        ha="left",
        fontsize=9.6,
        color="#5D5D5D",
    )
    fig.text(
        0.095,
        0.020,
        "数据源：run 202607271131 原始 trace/events；时间位置由共享 Orin monotonic_ns 经全anchor仿射映射到墙钟。",
        ha="left",
        fontsize=9.0,
        color="#8F8F8F",
    )
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURES / "component_timing_scatter_all_instances.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIGURES / "component_timing_scatter_all_instances.svg", bbox_inches="tight")
    plt.close(fig)


def append_evidence_and_claim_links(
    lineage_rows: list[dict],
    component_rows: list[dict],
    segments: list[dict],
    control_audit: list[dict],
) -> None:
    evidence_path = TABLES / "evidence_ledger.csv"
    evidence = read_csv(evidence_path)
    existing_ids = {row["evidence_id"] for row in evidence}
    additions = [
        {
            "evidence_id": "EV.ALLINST.LINEAGE.1131",
            "run_id": RUN_ID,
            "layer": "L2/L3",
            "metric": "all_trace_lineage_instances",
            "value": sum(row["availability"] == "AVAILABLE" for row in lineage_rows),
            "unit": "stage-instances",
            "evidence_class": "TRACE_LINEAGE",
            "clock_domain": "source epoch + Orin monotonic_ns",
            "source_file": str(TABLES / "all_instance_lineage_timing.csv"),
            "source_locator": "all 353 main-sensor Fusion traces; five lineage metrics with explicit missing rows",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131|C3.1131",
            "challenges_claim_ids": "",
            "limitations": "within-run instances are temporal samples, not independent experimental replicates",
            "semantic_role": "CAUSE_EFFECT_LINEAGE",
            "reference_type": "WITHIN_RUN_DISTRIBUTION",
            "distribution_scope": "all eligible trace instances",
            "causal_lineage_grade": "A_TO_CONTROL",
        },
        {
            "evidence_id": "EV.ALLINST.COMPONENT.1131",
            "run_id": RUN_ID,
            "layer": "L2",
            "metric": "all_component_execution_instances",
            "value": len(component_rows),
            "unit": "instances",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "Orin monotonic_ns",
            "source_file": str(TABLES / "component_timing_all_instances.csv"),
            "source_locator": "Lidar Detection + Planning RunOnce + Ground Detection paired intervals",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131",
            "challenges_claim_ids": "",
            "limitations": "research thresholds are not architectural contracts",
            "semantic_role": "MODULE_EXECUTION_DISTRIBUTION",
            "reference_type": "WITHIN_RUN_DISTRIBUTION",
            "distribution_scope": "all paired component instances",
            "causal_lineage_grade": "",
        },
        {
            "evidence_id": "EV.ANOMSEG.1131",
            "run_id": RUN_ID,
            "layer": "L2",
            "metric": "research_anomaly_segments",
            "value": sum(row["scope"] == "CROSS_COMPONENT" for row in segments),
            "unit": "segments",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_domain": "Orin monotonic_ns",
            "source_file": str(TABLES / "research_anomaly_segments.csv"),
            "source_locator": "median+6*MAD instances merged at <=100 ms",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131",
            "challenges_claim_ids": "",
            "limitations": "segmentation and threshold are research provenance; overlap is not root-cause proof",
            "semantic_role": "TEMPORAL_ANOMALY_SEGMENT",
            "reference_type": "WITHIN_RUN_ROBUST_REFERENCE",
            "distribution_scope": "run 1131",
            "causal_lineage_grade": "",
        },
        {
            "evidence_id": "EV.CONTROL.EPISODE.1131",
            "run_id": RUN_ID,
            "layer": "L3",
            "metric": "strict_control_to_physical_episode_distribution",
            "value": 0,
            "unit": "strictly-paired episodes",
            "evidence_class": "MISSING",
            "clock_domain": "Control monotonic to Localization wall",
            "source_file": str(TABLES / "control_physical_episode_audit.csv"),
            "source_locator": "3636 outputs / 354 traces; one analysis-defined Grade C event",
            "availability": "UNAVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "",
            "challenges_claim_ids": "C3.1131",
            "limitations": "event-local Bridge apply/payload and record feedback are absent",
            "semantic_role": "PHYSICAL_EPISODE_IDENTIFIABILITY",
            "reference_type": "",
            "distribution_scope": "full run",
            "causal_lineage_grade": "C_FOR_ONE_EVENT;UNKNOWN_FOR_DISTRIBUTION",
        },
    ]
    addition_by_id = {row["evidence_id"]: row for row in additions}
    evidence = [addition_by_id.get(row["evidence_id"], row) for row in evidence]
    evidence.extend(row for row in additions if row["evidence_id"] not in existing_ids)
    write_csv(evidence_path, evidence, list(evidence[0].keys()))

    claim_path = TABLES / "claim_ledger.csv"
    claims = read_csv(claim_path)
    link_map = {
        "C2.1131": ["EV.ALLINST.LINEAGE.1131", "EV.ALLINST.COMPONENT.1131", "EV.ANOMSEG.1131"],
        "C3.1131": ["EV.ALLINST.LINEAGE.1131"],
    }
    for claim in claims:
        if claim["claim_id"] not in link_map:
            continue
        current = [value for value in claim["supporting_evidence_ids"].split("|") if value]
        for evidence_id in link_map[claim["claim_id"]]:
            if evidence_id not in current:
                current.append(evidence_id)
        claim["supporting_evidence_ids"] = "|".join(current)
        if claim["claim_id"] == "C3.1131":
            support = [
                value
                for value in claim["supporting_evidence_ids"].split("|")
                if value and value != "EV.CONTROL.EPISODE.1131"
            ]
            challenge = [
                value for value in claim["challenging_evidence_ids"].split("|") if value
            ]
            if "EV.CONTROL.EPISODE.1131" not in challenge:
                challenge.append("EV.CONTROL.EPISODE.1131")
            claim["supporting_evidence_ids"] = "|".join(support)
            claim["challenging_evidence_ids"] = "|".join(challenge)
    write_csv(claim_path, claims, list(claims[0].keys()))

    completeness_path = TABLES / "method_completeness_matrix.csv"
    completeness = read_csv(completeness_path)
    existing_requirements = {row["requirement"] for row in completeness}
    extra = [
        {
            "requirement": "all eligible trace instances persisted",
            "status": "PASS",
            "evidence_or_gap": "all_instance_lineage_timing.csv retains available and missing stage instances",
        },
        {
            "requirement": "Control-to-physical action-episode distribution",
            "status": "NOT_ESTABLISHED",
            "evidence_or_gap": "3636 Control outputs collapse to 354 traces; payload/apply/record mapping missing",
        },
    ]
    completeness.extend(row for row in extra if row["requirement"] not in existing_requirements)
    write_csv(completeness_path, completeness, list(completeness[0].keys()))

    chart_path = TABLES / "chart_map.csv"
    chart_map = read_csv(chart_path)
    if not any(row.get("figure") == "component_timing_scatter_all_instances.png" for row in chart_map):
        chart_map.append(
            {
                "figure": "component_timing_scatter_all_instances.png",
                "question": "When did all component execution instances occur and where are robust anomaly segments?",
                "takeaway": "A pre-t_sample Lidar burst and a post-t_phys multi-component concurrent anomaly segment are visible.",
                "chart_family": "faceted scatter with interval shading",
                "fields": "start_relative_t_sample_s,duration_ms,metric_name,research_outlier,segment",
                "palette_policy": "relaxed three-category plus neutral references",
                "source_table": "component_timing_all_instances.csv|research_anomaly_segments.csv",
            }
        )
        write_csv(chart_path, chart_map)


def write_report(
    lineage_summary: list[dict],
    component_summary: list[dict],
    segments: list[dict],
    control_audit: list[dict],
    clock_audit: dict[str, float],
    t1: float,
    t2: float,
    collision: float,
) -> None:
    cross_clock_row = read_csv(TABLES / "clock_alignment_audit.csv")[0]
    cross_clock_p95_ms = float(cross_clock_row["dispersion_or_sync_distance_ms"])
    lineage = {row["metric_name"]: row for row in lineage_summary}
    components = {row["metric_name"]: row for row in component_summary}
    source_fusion = lineage["source_to_fusion_output"]
    multi = next(
        row
        for row in segments
        if row["scope"] == "CROSS_COMPONENT" and int(row["component_count"]) >= 2
    )
    early_lidar = next(
        row
        for row in segments
        if row["scope"] == "COMPONENT:lidar_detection_processing"
        and float(row["start_relative_t_sample_s"]) < 0
        and float(row["end_relative_t_sample_s"]) >= 0
    )
    population = first(control_audit, audit_item="CONTROL_MESSAGE_POPULATION")
    episode = first(control_audit, audit_item="EP_INITIAL_TARGET_BRAKE")

    component_lines = []
    for metric in (
        "lidar_detection_processing",
        "planning_runonce",
        "ground_detection_processing",
    ):
        row = components[metric]
        component_lines.append(
            f"| {metric} | {int(row['n'])} | {float(row['mean_ms']):.3f} | "
            f"{float(row['p50_ms']):.3f} | {float(row['p95_ms']):.3f} | "
            f"{float(row['p99_ms']):.3f} | {float(row['max_ms']):.3f} | "
            f"{int(row['research_outlier_count'])} |"
        )

    lineage_lines = []
    for metric in (
        "source_to_fusion_output",
        "fusion_to_prediction_output",
        "prediction_to_planning_output",
        "planning_output_to_first_control_output",
        "source_to_first_control_output",
    ):
        row = lineage[metric]
        first_value = row["first_causal_value_ms"]
        first_text = f"{float(first_value):.3f}" if first_value != "" else "不可用"
        rank = row["first_causal_percentile_rank"]
        rank_text = f"{float(rank)*100:.1f}%" if rank != "" else "不可用"
        lineage_lines.append(
            f"| {metric} | {int(row['available_instance_count'])}/{int(row['expected_instance_count'])} | "
            f"{float(row['p50_ms']):.3f} | {float(row['p95_ms']):.3f} | "
            f"{float(row['p99_ms']):.3f} | {float(row['max_ms']):.3f} | {first_text} | {rank_text} |"
        )

    segment_lines = []
    for row in segments:
        if row["scope"] != "CROSS_COMPONENT":
            continue
        segment_lines.append(
            f"| {row['segment_id']} | {float(row['start_relative_t_sample_s']):.3f} | "
            f"{float(row['end_relative_t_sample_s']):.3f} | {row['metrics']} | "
            f"{row['member_instance_count']} | {row['classification']} |"
        )

    report = f"""# 1131 run 全实例实时性重新分析（TCPS-PA v4.2）

## 技术结论

重新按“首次因果链 + 全run逐实例 + 独立物理动作episode”分析后，1131的结论比原报告更完整：

1. 首次 `source→Fusion` 的报告值仍为 **292.885 ms**（日志墙钟）；同一实例的trace重算值为 **{float(source_fusion['first_causal_value_ms']):.3f} ms**，在全部{int(source_fusion['available_instance_count'])}个可用实例中位于 **{float(source_fusion['first_causal_percentile_rank'])*100:.1f}%分位**。它不是平均值，也不是最大值，但相对本run中位数 **{float(source_fusion['p50_ms']):.3f} ms** 已进入research异常区。
2. 全run存在一个跨越`t_sample`的Lidar异常突发段：相对`t_sample` **[{float(early_lidar['start_relative_t_sample_s']):.3f}, {float(early_lidar['end_relative_t_sample_s']):.3f}] s**。这说明首次因果链慢并非完全孤立点，而处在一段感知执行时间抬升的尾部。
3. `t_phys`之后出现跨三模块并发异常段：相对`t_sample` **[{float(multi['start_relative_t_sample_s']):.3f}, {float(multi['end_relative_t_sample_s']):.3f}] s**，覆盖Lidar Detection 507.315 ms、Planning RunOnce 473.557 ms和Ground Detection 481.354 ms。三者共同执行重叠约467.161 ms；它们来自相邻trace，不是同一帧。
4. Control共有 **{int(population['total_control_output_messages'])}** 条输出，但只有 **{int(population['unique_control_trace_ids'])}** 个唯一trace，单trace最多重复 **{int(population['max_outputs_per_trace'])}** 次。缺少事件级Bridge apply和payload，不能构造逐物理动作episode的延迟分布；仅首次制动episode可保留Grade C的 **{float(episode['control_to_physical_reported_upper_ms']):.3f} ms** 上界采样值。
5. 全实例统计强化了L2/L3的定位，但不产生新的独立动态deadline，也不把C4/C5从`NOT_TESTABLE/MODEL_SUPPORTED_ONLY`升级为直接成立。

首次`source→Fusion`虽然超过`median+6×MAD`，但该筛查线在全体352个实例中标记了38个（10.8%）。这更像运行阶段/负载相关的非平稳分布，而不是38个彼此独立的罕见故障；因此报告采用事件窗口分层，并不把该research筛查直接解释为contract violation。

## 明确lineage边的全部实例统计

| 软件链指标 | 可用/期望 | P50 ms | P95 ms | P99 ms | MAX ms | 首次因果值 ms | 首次值分位 |
|---|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(lineage_lines)}

其中`source→Fusion`和`source→first Control`各有一个启动期Fusion实例缺少父LiDAR anchor，明确保存为不可用；三个纯monotonic阶段仍保留353/353个实例，没有因为source anchor缺失而错误丢弃。

## 全实例分布

| 指标 | n | mean ms | P50 ms | P95 ms | P99 ms | MAX ms | median+6MAD异常数 |
|---|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(component_lines)}

平均值仅作为完整统计的一部分；实时性判断主要同时查看P95/P99/MAX、MAD/IQR、异常连续段和事件窗口。run内帧是时间样本，不是独立实验重复。

## 异常时间段

| 段 | 起点相对t_sample s | 终点相对t_sample s | 涉及模块 | 实例数 | 分类 |
|---|---:|---:|---|---:|---|
{chr(10).join(segment_lines)}

异常判据为本run分布的`median+6×MAD`，provenance为`RESEARCH`。它可以定位异常，但不是architectural/calibrated contract，不能据此直接宣称deadline miss。

![三个模块全实例散点图](../figures/component_timing_scatter_all_instances.png)

## 明确trace lineage的全部实例

`all_instance_lineage_timing.csv`以353个主LiDAR Fusion trace为母体，逐实例保存：

- `source→Fusion output`；
- `Fusion→Prediction output`；
- `Prediction→Planning output`；
- `Planning output→first Control output`；
- `source→first Control output`；
- 不可用端点和missing reason。

首次值继续服务于首次障碍响应和物理预算；全体分布用于闭环timing integrity诊断。二者并列，不相互覆盖。

## Control独立动作episode审计

当前数据不能知道每条Control输出的制动/转向payload，也没有逐命令Bridge apply记录；Apollo record同样未录制。因此：

- 软件更新episode候选可以按唯一trace统计；
- 真正的物理动作episode不能从3636条Control发布中可靠分割；
- 将每条Control消息与唯一`t_phys`相减会形成多对一伪重复，禁止作为物理响应时间分布；
- 首次目标制动只支持Control输出到`t_phys`的事件级Grade C关联，采样区间约为 **[{float(episode['control_to_physical_lower_bracket_ms']):.3f}, {float(episode['control_to_physical_upper_bracket_ms']):.3f}] ms**。

## 时钟、稳健性与限制

- 实例执行时间直接使用同一Orin `monotonic_ns`相减；不依赖墙钟拟合。
- 散点内部位置使用{int(clock_audit['anchor_count'])}个LiDAR source/monotonic anchor拟合，内部P95残差为 **{clock_audit['p95_abs_residual_ms']:.3f} ms**；这不是跨主机物理事件误差。Apollo/CARLA/Bridge事件比较继续采用原`P_CLOCK`审计的 **{cross_clock_p95_ms:.3f} ms** P95界限。两者均不改变约0.5 s尖峰和事件先后排序。
- `t_sample={iso(t1)}`，`t_phys={iso(t2)}`，碰撞=`{iso(collision)}`。
- 多模块时间重叠支持共同资源/调度干扰候选，但没有CPU/GPU调度、队列深度和利用率证据，不能证明唯一根因。
- 动态物理deadline仍受原v4.2报告中的独立性、锁时和模型验证限制。

## 后续需要的证据

1. 开启Apollo record，保存Control payload、Planning reuse、Chassis反馈和模块timeline。
2. `log_all_delayed_commands=true`，保存每个Bridge receive/release/apply事件。
3. 记录CPU/GPU利用率、线程调度、GPU kernel和队列深度，以区分共享资源竞争与模块内部执行尾部。
4. 对匹配初始状态进行多run复现；run内帧分布不能替代实验重复。

## 复现

```bash
python3 {OUTPUT / 'scripts/analyze_1131_single_run_v4_2.py'}
python3 {OUTPUT / 'scripts/analyze_1131_all_instances_v4_2.py'}
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/recompute_l5_metrics.py --analysis-dir {OUTPUT}
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/validate_analysis_outputs.py --analysis-dir {OUTPUT}
```

原始run目录保持只读；所有新增结果写入独立v4.2目录。
"""
    REPORT.mkdir(parents=True, exist_ok=True)
    (REPORT / "all_instance_reanalysis_report.md").write_text(report, encoding="utf-8")


def validate_outputs(
    lineage_rows: list[dict],
    lineage_summary: list[dict],
    component_rows: list[dict],
    component_summary: list[dict],
    segments: list[dict],
    control_reuse: list[dict],
    control_audit: list[dict],
) -> None:
    checks: list[dict] = []

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"check": name, "passed": bool(passed), "detail": detail})

    counts = {row["metric_name"]: int(row["n"]) for row in component_summary}
    check("lidar_instance_count", counts["lidar_detection_processing"] == 352, str(counts))
    check("planning_instance_count", counts["planning_runonce"] == 353, str(counts))
    check("ground_instance_count", counts["ground_detection_processing"] == 361, str(counts))
    selected_values = {
        metric: max(
            float(row["duration_ms"])
            for row in component_rows
            if row["metric_name"] == metric
        )
        for metric in SELECTED_EXTREMES
    }
    check(
        "selected_extreme_values",
        abs(selected_values["lidar_detection_processing"] - 507.314848) < 1e-9
        and abs(selected_values["planning_runonce"] - 473.55728) < 1e-9
        and abs(selected_values["ground_detection_processing"] - 481.353664) < 1e-9,
        json.dumps(selected_values, ensure_ascii=False),
    )
    multi = [
        row
        for row in segments
        if row["scope"] == "CROSS_COMPONENT" and int(row["component_count"]) >= 2
    ]
    check("one_multi_component_segment", len(multi) == 1, f"count={len(multi)}")
    check(
        "control_population",
        sum(int(row["control_output_count"]) for row in control_reuse) == 3636
        and len(control_reuse) == 354,
        f"outputs={sum(int(row['control_output_count']) for row in control_reuse)}, traces={len(control_reuse)}",
    )
    source_summary = next(
        row for row in lineage_summary if row["metric_name"] == "source_to_fusion_output"
    )
    check(
        "source_fusion_all_instances",
        int(source_summary["expected_instance_count"]) == 353
        and int(source_summary["available_instance_count"]) == 352,
        f"expected={source_summary['expected_instance_count']}, available={source_summary['available_instance_count']}",
    )
    episode_dist = first(control_audit, audit_item="CONTROL_TO_PHYSICAL_EPISODE_DISTRIBUTION")
    check(
        "no_fabricated_physical_episode_distribution",
        episode_dist["availability"] == "UNAVAILABLE",
        episode_dist.get("missing_reason", ""),
    )
    required_files = [
        TABLES / "all_instance_lineage_timing.csv",
        TABLES / "all_instance_lineage_distribution.csv",
        TABLES / "component_timing_all_instances.csv",
        TABLES / "component_timing_distribution.csv",
        TABLES / "research_anomaly_segments.csv",
        TABLES / "control_trace_reuse.csv",
        TABLES / "control_physical_episode_audit.csv",
        FIGURES / "component_timing_scatter_all_instances.png",
        REPORT / "all_instance_reanalysis_report.md",
    ]
    check(
        "required_artifacts_exist",
        all(path.exists() and path.stat().st_size > 0 for path in required_files),
        "|".join(str(path) for path in required_files),
    )
    status = "PASS" if all(item["passed"] for item in checks) else "FAIL"
    result = {
        "run_id": RUN_ID,
        "status": status,
        "checks": checks,
        "important_limitations": [
            "Research anomaly thresholds are not component contracts.",
            "Within-run instances are not independent experimental replicates.",
            "Strict Control-to-physical episode distribution is unavailable.",
        ],
    }
    VALIDATION.mkdir(parents=True, exist_ok=True)
    (VALIDATION / "all_instance_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    lines = [
        "# 全实例分析验证",
        "",
        f"总体状态：**{status}**",
        "",
    ]
    lines.extend(
        f"- {'PASS' if item['passed'] else 'FAIL'} — {item['check']}: {item['detail']}"
        for item in checks
    )
    lines.extend(
        [
            "",
            "## 必须保留的限制",
            "",
            "- research阈值不是component contract；",
            "- run内实例不是独立实验重复；",
            "- Control→physical严格episode分布不可用。",
        ]
    )
    (VALIDATION / "all_instance_validation.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def main() -> None:
    TABLES.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)
    REPORT.mkdir(parents=True, exist_ok=True)
    VALIDATION.mkdir(parents=True, exist_ok=True)

    event_rows = read_csv(TABLES / "event_timeline.csv")
    t1 = float(first(event_rows, event_id="E01")["t_wall_s"])
    t2 = float(first(event_rows, event_id="E07")["t_wall_s"])
    collision = float(first(event_rows, event_id="E09")["t_wall_s"])
    anchors = {row["trace_id"]: row for row in read_csv(ANCHOR_FILE)}
    slope, intercept, clock_audit = build_clock_map(anchors)

    lineage_rows, lineage_summary = build_lineage_instances(
        anchors, slope, intercept, t1, t2, collision
    )
    component_rows, component_summary = build_component_instances(
        slope, intercept, t1, t2, collision
    )
    segments = merge_segments(component_rows)
    control_reuse, control_audit = build_control_audit(slope, intercept, t1, t2)

    write_csv(TABLES / "all_instance_lineage_timing.csv", lineage_rows)
    write_csv(TABLES / "all_instance_lineage_distribution.csv", lineage_summary)
    write_csv(
        TABLES / "all_instance_lineage_distribution_by_window.csv",
        distribution_by_window(lineage_rows, availability_required=True),
    )
    write_csv(TABLES / "component_timing_all_instances.csv", component_rows)
    write_csv(TABLES / "component_timing_distribution.csv", component_summary)
    write_csv(
        TABLES / "component_timing_distribution_by_window.csv",
        distribution_by_window(component_rows),
    )
    write_csv(TABLES / "research_anomaly_segments.csv", segments)
    write_csv(TABLES / "control_trace_reuse.csv", control_reuse)
    write_csv(TABLES / "control_physical_episode_audit.csv", control_audit)
    write_csv(TABLES / "all_instance_clock_mapping_audit.csv", [clock_audit])

    plot_component_scatter(component_rows, component_summary, segments, t1, t2, collision)
    append_evidence_and_claim_links(lineage_rows, component_rows, segments, control_audit)
    write_report(
        lineage_summary,
        component_summary,
        segments,
        control_audit,
        clock_audit,
        t1,
        t2,
        collision,
    )
    validate_outputs(
        lineage_rows,
        lineage_summary,
        component_rows,
        component_summary,
        segments,
        control_reuse,
        control_audit,
    )
    print(
        json.dumps(
            {
                "lineage_rows": len(lineage_rows),
                "component_rows": len(component_rows),
                "segments": len(segments),
                "control_traces": len(control_reuse),
                "report": str(REPORT / "all_instance_reanalysis_report.md"),
                "figure": str(FIGURES / "component_timing_scatter_all_instances.png"),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
