#!/usr/bin/env python3
"""Build Prediction deadline tables, spike-frame extracts, and static charts."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Iterable


csv.field_size_limit(1024 * 1024 * 1024)

VARIABLE_NODES = {
    "cyber_taskmanager_notify_task",
    "cruise_mlp_inference",
    "model_cruise_mlp",
    "jointly_prediction",
    "jointly_prediction_container",
    "jointly_prediction_evaluator",
    "jointly_prediction_predictor",
    "jointly_prediction_trim",
    "model_jointly_prediction",
}

GANTT_CALLBACK_NODES = (
    "container",
    "semantic_map_run",
    "thread_pool_run",
    "predictor",
    "writer",
)
GANTT_ASYNC_NODE = "semantic_base_async_draw"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--fixed-deadline-ms", type=float, default=100.0)
    parser.add_argument("--extreme-fraction", type=float, default=0.01)
    return parser.parse_args()


def one(path: Path, pattern: str) -> Path:
    matches = sorted(path.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(f"expected one {pattern} under {path}, found {len(matches)}")
    return matches[0]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def fmt(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}" if math.isfinite(value) else ""
    return value


def write_csv(path: Path, rows: list[dict[str, Any]], fields: Iterable[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(fields or (rows[0].keys() if rows else []))
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: fmt(row.get(field)) for field in fieldnames})


def quantile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def aggregate_variable_nodes(
    events_path: Path,
    frame_by_trace: dict[int, dict[str, Any]],
) -> list[dict[str, Any]]:
    pending: dict[tuple[int, str, int], deque[tuple[int, int]]] = defaultdict(deque)
    aggregates: dict[tuple[int, str], dict[str, Any]] = {}
    with events_path.open(newline="", encoding="utf-8-sig") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("trace_id") or not raw.get("phase") or not raw.get("mono_ns"):
                continue
            trace_id = int(raw["trace_id"])
            if trace_id not in frame_by_trace:
                continue
            phase = raw["phase"]
            tid = int(raw["tid"])
            mono_ns = int(raw["mono_ns"])
            if phase.endswith("_enter"):
                node = phase[: -len("_enter")]
                if node in VARIABLE_NODES:
                    pending[(trace_id, node, tid)].append((mono_ns, int(raw["event_id"])))
                continue
            if not phase.endswith("_exit"):
                continue
            node = phase[: -len("_exit")]
            key = (trace_id, node, tid)
            if node not in VARIABLE_NODES or not pending[key]:
                continue
            enter_ns, enter_event_id = pending[key].popleft()
            duration_ms = (mono_ns - enter_ns) / 1e6
            aggregate_key = (trace_id, node)
            aggregate = aggregates.setdefault(
                aggregate_key,
                {
                    "trace_id": trace_id,
                    "node": node,
                    "call_count": 0,
                    "total_duration_ms": 0.0,
                    "max_call_duration_ms": 0.0,
                    "first_enter_mono_ns": enter_ns,
                    "last_exit_mono_ns": mono_ns,
                    "tids": set(),
                    "first_enter_event_id": enter_event_id,
                    "last_exit_event_id": int(raw["event_id"]),
                },
            )
            aggregate["call_count"] += 1
            aggregate["total_duration_ms"] += duration_ms
            aggregate["max_call_duration_ms"] = max(
                aggregate["max_call_duration_ms"], duration_ms
            )
            aggregate["first_enter_mono_ns"] = min(aggregate["first_enter_mono_ns"], enter_ns)
            aggregate["last_exit_mono_ns"] = max(aggregate["last_exit_mono_ns"], mono_ns)
            aggregate["tids"].add(tid)
            aggregate["first_enter_event_id"] = min(
                aggregate["first_enter_event_id"], enter_event_id
            )
            aggregate["last_exit_event_id"] = max(
                aggregate["last_exit_event_id"], int(raw["event_id"])
            )

    rows: list[dict[str, Any]] = []
    for (trace_id, _), aggregate in aggregates.items():
        frame = frame_by_trace[trace_id]
        rows.append(
            {
                "frame_index": frame["frame_index"],
                "trace_id": trace_id,
                "input_perception_seq": frame["input_perception_seq"],
                "prediction_output_seq": frame["prediction_output_seq"],
                "proc_enter_tid": frame["proc_enter_tid"],
                "node": aggregate["node"],
                "call_count": aggregate["call_count"],
                "total_duration_ms": aggregate["total_duration_ms"],
                "max_call_duration_ms": aggregate["max_call_duration_ms"],
                "first_enter_mono_ns": aggregate["first_enter_mono_ns"],
                "last_exit_mono_ns": aggregate["last_exit_mono_ns"],
                "first_enter_event_id": aggregate["first_enter_event_id"],
                "last_exit_event_id": aggregate["last_exit_event_id"],
                "tids": ";".join(str(value) for value in sorted(aggregate["tids"])),
            }
        )
    return sorted(rows, key=lambda row: (row["frame_index"], row["node"]))


def variable_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["node"])].append(row)
    result: list[dict[str, Any]] = []
    for node, node_rows in sorted(grouped.items()):
        totals = [float(row["total_duration_ms"]) for row in node_rows]
        counts = [int(row["call_count"]) for row in node_rows]
        maximum = max(node_rows, key=lambda row: float(row["total_duration_ms"]))
        result.append(
            {
                "node": node,
                "frames_present": len(node_rows),
                "total_calls": sum(counts),
                "median_calls_per_present_frame": statistics.median(counts),
                "max_calls_per_frame": max(counts),
                "median_total_duration_ms": statistics.median(totals),
                "p95_total_duration_ms": quantile(totals, 0.95),
                "p99_total_duration_ms": quantile(totals, 0.99),
                "max_total_duration_ms": max(totals),
                "max_frame_index": maximum["frame_index"],
                "max_input_perception_seq": maximum["input_perception_seq"],
            }
        )
    return result


def attach_log_tids(
    detail: list[dict[str, Any]], log_path: Path
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    pattern = re.compile(r"\[PRED_TID\]\s+input_perception_seq=(\d+)\s+tid=(\d+)")
    by_seq: dict[int, tuple[int, int]] = {}
    duplicate_sequences = 0
    with log_path.open(encoding="utf-8", errors="replace") as handle:
        for line_number, line in enumerate(handle, start=1):
            match = pattern.search(line)
            if not match:
                continue
            sequence = int(match.group(1))
            tid = int(match.group(2))
            if sequence in by_seq and by_seq[sequence][0] != tid:
                duplicate_sequences += 1
            by_seq[sequence] = (tid, line_number)

    audit: list[dict[str, Any]] = []
    for frame in detail:
        sequence = frame["input_perception_seq"]
        log_entry = by_seq.get(sequence) if sequence is not None else None
        log_tid = log_entry[0] if log_entry else None
        log_line = log_entry[1] if log_entry else None
        match = log_tid == frame["proc_enter_tid"] if log_tid is not None else None
        frame["prediction_log_tid"] = log_tid
        frame["prediction_log_line"] = log_line
        frame["log_trace_tid_match"] = match
        audit.append(
            {
                "frame_index": frame["frame_index"],
                "trace_id": frame["trace_id"],
                "input_perception_seq": sequence,
                "trace_proc_enter_tid": frame["proc_enter_tid"],
                "prediction_log_tid": log_tid,
                "prediction_log_line": log_line,
                "tid_match": match,
                "status": "match" if match else ("mismatch" if match is False else "unlinked"),
            }
        )
    summary = {
        "log_tid_records": len(by_seq),
        "eligible_frames": len(detail),
        "linked_frames": sum(row["prediction_log_tid"] is not None for row in detail),
        "matching_frames": sum(row["log_trace_tid_match"] is True for row in detail),
        "mismatching_frames": sum(row["log_trace_tid_match"] is False for row in detail),
        "unlinked_frames": sum(row["log_trace_tid_match"] is None for row in detail),
        "duplicate_log_sequences_with_conflicting_tid": duplicate_sequences,
    }
    return audit, summary


def build_deadlines(
    raw_frames: list[dict[str, str]], fixed_deadline_ms: float
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    frames: list[dict[str, Any]] = []
    for raw in raw_frames:
        frames.append(
            {
                "frame_index": int(raw["frame_index"]),
                "trace_id": int(raw["trace_id"]),
                "input_perception_seq": int(raw["input_perception_seq"]) if raw["input_perception_seq"] else None,
                "prediction_output_seq": int(raw["prediction_output_seq"]) if raw["prediction_output_seq"] else None,
                "proc_enter_tid": int(raw["proc_enter_tid"]),
                "proc_enter_mono_ns": int(raw["proc_enter_mono_ns"]),
                "output_pub_mono_ns": int(raw["output_pub_mono_ns"]),
                "proc_exit_mono_ns": int(raw["proc_exit_mono_ns"]),
                "last_event_mono_ns": int(raw["last_event_mono_ns"]),
                "proc_ms": float(raw["proc_ms"]),
                "publish_latency_ms": float(raw["publish_latency_ms"]),
                "frame_complete_ms": float(raw["frame_complete_ms"]),
                "input_interarrival_ms": float(raw["input_interarrival_ms"]) if raw["input_interarrival_ms"] else None,
                "prediction_obstacle_count": int(raw["prediction_obstacle_count"]) if raw["prediction_obstacle_count"] else None,
                "prediction_trajectory_count": int(raw["prediction_trajectory_count"]) if raw["prediction_trajectory_count"] else None,
            }
        )
    frames.sort(key=lambda row: row["frame_index"])

    detail: list[dict[str, Any]] = []
    fixed_ns = int(round(fixed_deadline_ms * 1e6))
    for index, frame in enumerate(frames):
        next_enter_ns = frames[index + 1]["proc_enter_mono_ns"] if index + 1 < len(frames) else None
        fixed_deadline_ns = frame["proc_enter_mono_ns"] + fixed_ns
        fixed_miss = frame["proc_exit_mono_ns"] > fixed_deadline_ns
        full_fixed_miss = frame["last_event_mono_ns"] > fixed_deadline_ns
        observed_miss = (
            frame["proc_exit_mono_ns"] > next_enter_ns if next_enter_ns is not None else None
        )
        observed_full_miss = (
            frame["last_event_mono_ns"] > next_enter_ns if next_enter_ns is not None else None
        )
        detail.append(
            {
                **frame,
                "fixed_deadline_ms": fixed_deadline_ms,
                "fixed_deadline_mono_ns": fixed_deadline_ns,
                "fixed_callback_slack_ms": fixed_deadline_ms - frame["proc_ms"],
                "fixed_callback_overrun_ms": max(0.0, frame["proc_ms"] - fixed_deadline_ms),
                "fixed_callback_deadline_miss": fixed_miss,
                "fixed_full_frame_slack_ms": fixed_deadline_ms - frame["frame_complete_ms"],
                "fixed_full_frame_overrun_ms": max(
                    0.0, frame["frame_complete_ms"] - fixed_deadline_ms
                ),
                "fixed_full_frame_deadline_miss": full_fixed_miss,
                "next_proc_enter_mono_ns": next_enter_ns,
                "observed_next_input_deadline_ms": (
                    (next_enter_ns - frame["proc_enter_mono_ns"]) / 1e6
                    if next_enter_ns is not None
                    else None
                ),
                "observed_next_input_callback_slack_ms": (
                    (next_enter_ns - frame["proc_exit_mono_ns"]) / 1e6
                    if next_enter_ns is not None
                    else None
                ),
                "observed_next_input_callback_miss": observed_miss,
                "observed_next_input_full_frame_slack_ms": (
                    (next_enter_ns - frame["last_event_mono_ns"]) / 1e6
                    if next_enter_ns is not None
                    else None
                ),
                "observed_next_input_full_frame_miss": observed_full_miss,
            }
        )

    proc_values = [row["proc_ms"] for row in detail]
    fixed_miss_rows = [row for row in detail if row["fixed_callback_deadline_miss"]]
    fixed_full_miss_rows = [row for row in detail if row["fixed_full_frame_deadline_miss"]]
    observed_rows = [row for row in detail if row["observed_next_input_callback_miss"] is not None]
    observed_full_rows = [row for row in detail if row["observed_next_input_full_frame_miss"] is not None]
    observed_periods = [row["observed_next_input_deadline_ms"] for row in observed_rows]

    def rate(count: int, denominator: int) -> float:
        return count / denominator if denominator else math.nan

    summary = [
        {
            "deadline_name": f"fixed_callback_{fixed_deadline_ms:g}ms",
            "primary_result": True,
            "completion_event": "proc_exit",
            "deadline_definition": f"proc_enter + {fixed_deadline_ms:g} ms",
            "eligible_frames": len(detail),
            "miss_count": len(fixed_miss_rows),
            "miss_rate": rate(len(fixed_miss_rows), len(detail)),
            "miss_rate_pct": 100.0 * rate(len(fixed_miss_rows), len(detail)),
            "deadline_ms_median": fixed_deadline_ms,
            "deadline_ms_p99": fixed_deadline_ms,
            "slack_ms_min": min(row["fixed_callback_slack_ms"] for row in detail),
            "slack_ms_median": statistics.median(
                row["fixed_callback_slack_ms"] for row in detail
            ),
            "max_overrun_ms": max(row["fixed_callback_overrun_ms"] for row in detail),
        },
        {
            "deadline_name": f"fixed_full_trace_{fixed_deadline_ms:g}ms",
            "primary_result": False,
            "completion_event": "last traced event (includes async tail)",
            "deadline_definition": f"proc_enter + {fixed_deadline_ms:g} ms",
            "eligible_frames": len(detail),
            "miss_count": len(fixed_full_miss_rows),
            "miss_rate": rate(len(fixed_full_miss_rows), len(detail)),
            "miss_rate_pct": 100.0 * rate(len(fixed_full_miss_rows), len(detail)),
            "deadline_ms_median": fixed_deadline_ms,
            "deadline_ms_p99": fixed_deadline_ms,
            "slack_ms_min": min(row["fixed_full_frame_slack_ms"] for row in detail),
            "slack_ms_median": statistics.median(
                row["fixed_full_frame_slack_ms"] for row in detail
            ),
            "max_overrun_ms": max(row["fixed_full_frame_overrun_ms"] for row in detail),
        },
        {
            "deadline_name": "observed_next_input_callback",
            "primary_result": False,
            "completion_event": "proc_exit",
            "deadline_definition": "next observed proc_enter (diagnostic, not an independent requirement)",
            "eligible_frames": len(observed_rows),
            "miss_count": sum(bool(row["observed_next_input_callback_miss"]) for row in observed_rows),
            "miss_rate": rate(
                sum(bool(row["observed_next_input_callback_miss"]) for row in observed_rows),
                len(observed_rows),
            ),
            "miss_rate_pct": 100.0
            * rate(
                sum(bool(row["observed_next_input_callback_miss"]) for row in observed_rows),
                len(observed_rows),
            ),
            "deadline_ms_median": statistics.median(observed_periods),
            "deadline_ms_p99": quantile(observed_periods, 0.99),
            "slack_ms_min": min(row["observed_next_input_callback_slack_ms"] for row in observed_rows),
            "slack_ms_median": statistics.median(
                row["observed_next_input_callback_slack_ms"] for row in observed_rows
            ),
            "max_overrun_ms": max(
                0.0,
                -min(row["observed_next_input_callback_slack_ms"] for row in observed_rows),
            ),
        },
        {
            "deadline_name": "observed_next_input_full_trace",
            "primary_result": False,
            "completion_event": "last traced event (includes async tail)",
            "deadline_definition": "next observed proc_enter (diagnostic, not an independent requirement)",
            "eligible_frames": len(observed_full_rows),
            "miss_count": sum(bool(row["observed_next_input_full_frame_miss"]) for row in observed_full_rows),
            "miss_rate": rate(
                sum(bool(row["observed_next_input_full_frame_miss"]) for row in observed_full_rows),
                len(observed_full_rows),
            ),
            "miss_rate_pct": 100.0
            * rate(
                sum(bool(row["observed_next_input_full_frame_miss"]) for row in observed_full_rows),
                len(observed_full_rows),
            ),
            "deadline_ms_median": statistics.median(observed_periods),
            "deadline_ms_p99": quantile(observed_periods, 0.99),
            "slack_ms_min": min(row["observed_next_input_full_frame_slack_ms"] for row in observed_full_rows),
            "slack_ms_median": statistics.median(
                row["observed_next_input_full_frame_slack_ms"] for row in observed_full_rows
            ),
            "max_overrun_ms": max(
                0.0,
                -min(row["observed_next_input_full_frame_slack_ms"] for row in observed_full_rows),
            ),
        },
    ]

    miss_intervals: list[dict[str, Any]] = []
    current: list[dict[str, Any]] = []
    for row in detail:
        if row["fixed_callback_deadline_miss"]:
            if current and row["frame_index"] != current[-1]["frame_index"] + 1:
                miss_intervals.append(summarize_interval(current))
                current = []
            current.append(row)
        elif current:
            miss_intervals.append(summarize_interval(current))
            current = []
    if current:
        miss_intervals.append(summarize_interval(current))

    return detail, summary, miss_intervals


def summarize_interval(rows: list[dict[str, Any]]) -> dict[str, Any]:
    peak = max(rows, key=lambda row: row["proc_ms"])
    return {
        "start_frame_index": rows[0]["frame_index"],
        "end_frame_index": rows[-1]["frame_index"],
        "start_input_perception_seq": rows[0]["input_perception_seq"],
        "end_input_perception_seq": rows[-1]["input_perception_seq"],
        "miss_frames": len(rows),
        "mean_proc_ms": statistics.fmean(row["proc_ms"] for row in rows),
        "max_proc_ms": peak["proc_ms"],
        "max_overrun_ms": peak["fixed_callback_overrun_ms"],
        "peak_frame_index": peak["frame_index"],
        "peak_input_perception_seq": peak["input_perception_seq"],
        "worker_tids": ";".join(str(value) for value in sorted({row["proc_enter_tid"] for row in rows})),
    }


def select_extremes(
    detail: list[dict[str, Any]], fraction: float
) -> tuple[list[dict[str, Any]], float]:
    if not (0 < fraction <= 1):
        raise ValueError("extreme fraction must be in (0, 1]")
    threshold = quantile([row["proc_ms"] for row in detail], 1.0 - fraction)
    selected = [row.copy() for row in detail if row["proc_ms"] > threshold]
    selected.sort(key=lambda row: row["proc_ms"], reverse=True)
    for rank, row in enumerate(selected, start=1):
        row["proc_extreme_rank"] = rank
        row["proc_p99_threshold_ms"] = threshold
        row["extreme_definition"] = f"proc_ms > empirical P{100 * (1 - fraction):g}"
    return selected, threshold


def build_gantt_data(
    detail: list[dict[str, Any]],
    extremes: list[dict[str, Any]],
    long_rows: list[dict[str, str]],
) -> list[dict[str, Any]]:
    extreme_traces = {int(row["trace_id"]) for row in extremes}
    detail_by_trace = {int(row["trace_id"]): row for row in detail}
    rows: list[dict[str, Any]] = []
    for raw in long_rows:
        trace_id = int(raw["trace_id"])
        node = raw["node"]
        if trace_id not in extreme_traces or node not in set(GANTT_CALLBACK_NODES) | {GANTT_ASYNC_NODE}:
            continue
        frame = detail_by_trace[trace_id]
        enter_ns = int(raw["enter_mono_ns"])
        exit_ns = int(raw["exit_mono_ns"])
        rows.append(
            {
                "frame_index": frame["frame_index"],
                "input_perception_seq": frame["input_perception_seq"],
                "prediction_output_seq": frame["prediction_output_seq"],
                "proc_enter_tid": frame["proc_enter_tid"],
                "trace_id": trace_id,
                "lane": "async" if node == GANTT_ASYNC_NODE else "callback",
                "node": node,
                "enter_tid": int(raw["enter_tid"]),
                "exit_tid": int(raw["exit_tid"]),
                "start_from_proc_enter_ms": (enter_ns - frame["proc_enter_mono_ns"]) / 1e6,
                "end_from_proc_enter_ms": (exit_ns - frame["proc_enter_mono_ns"]) / 1e6,
                "duration_ms": (exit_ns - enter_ns) / 1e6,
            }
        )
    return sorted(rows, key=lambda row: (row["frame_index"], row["lane"], row["start_from_proc_enter_ms"]))


def build_stage_breakdown(
    extremes: list[dict[str, Any]], raw_frames: list[dict[str, str]]
) -> list[dict[str, Any]]:
    by_index = {int(row["frame_index"]): row for row in raw_frames}
    rows: list[dict[str, Any]] = []
    for extreme in extremes:
        raw = by_index[extreme["frame_index"]]
        thread_pool_ms = float(raw["thread_pool_run_ms"])
        rows.append(
            {
                "proc_extreme_rank": extreme["proc_extreme_rank"],
                "frame_index": extreme["frame_index"],
                "input_perception_seq": extreme["input_perception_seq"],
                "proc_enter_tid": extreme["proc_enter_tid"],
                "proc_ms": extreme["proc_ms"],
                "container_ms": float(raw["container_ms"]),
                "semantic_map_run_ms": float(raw["semantic_map_run_ms"]),
                "thread_pool_run_ms": thread_pool_ms,
                "thread_pool_share_of_proc_pct": 100.0 * thread_pool_ms / extreme["proc_ms"],
                "predictor_ms": float(raw["predictor_ms"]),
                "writer_ms": float(raw["writer_ms"]),
                "semantic_base_async_draw_ms": float(raw["semantic_base_async_draw_ms"]),
            }
        )
    return rows


def build_execution_regimes(
    detail: list[dict[str, Any]], variable_rows: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    model_frames = {
        int(row["frame_index"])
        for row in variable_rows
        if row["node"] == "model_cruise_mlp"
    }
    result: list[dict[str, Any]] = []
    for name, present in (
        ("model_cruise_mlp_present", True),
        ("model_cruise_mlp_absent", False),
    ):
        rows = [
            row
            for row in detail
            if (row["frame_index"] in model_frames) is present
        ]
        values = [row["proc_ms"] for row in rows]
        misses = sum(bool(row["fixed_callback_deadline_miss"]) for row in rows)
        result.append(
            {
                "regime": name,
                "frames": len(rows),
                "fixed_deadline_misses": misses,
                "fixed_deadline_miss_rate": misses / len(rows),
                "fixed_deadline_miss_rate_pct": 100.0 * misses / len(rows),
                "proc_median_ms": statistics.median(values),
                "proc_p95_ms": quantile(values, 0.95),
                "proc_p99_ms": quantile(values, 0.99),
                "proc_max_ms": max(values),
            }
        )
    return result


def render_charts(
    analysis_dir: Path,
    detail: list[dict[str, Any]],
    summary: list[dict[str, Any]],
    extremes: list[dict[str, Any]],
    gantt_rows: list[dict[str, Any]],
    fixed_deadline_ms: float,
) -> None:
    from PIL import Image, ImageDraw, ImageFont

    def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
        candidates = [
            Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
            Path("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc"),
        ]
        for candidate in candidates:
            if candidate.exists():
                return ImageFont.truetype(str(candidate), size=size)
        return ImageFont.load_default()

    def dashed_vertical(
        draw: ImageDraw.ImageDraw,
        x: int,
        y0: int,
        y1: int,
        fill: str,
        width: int = 3,
        dash: int = 12,
    ) -> None:
        y = y0
        while y < y1:
            draw.line((x, y, x, min(y + dash, y1)), fill=fill, width=width)
            y += 2 * dash

    figure_dir = analysis_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    frames = [row["frame_index"] for row in detail]
    proc = [row["proc_ms"] for row in detail]
    misses = [row for row in detail if row["fixed_callback_deadline_miss"]]
    width, height = 2400, 1050
    left, right, top, bottom = 150, 80, 150, 145
    plot_left, plot_right = left, width - right
    plot_top, plot_bottom = top, height - bottom
    y_max = max(300.0, math.ceil(max(proc) / 50.0) * 50.0)
    x_min, x_max = min(frames), max(frames)
    x_scale = (plot_right - plot_left) / max(1, x_max - x_min)
    y_scale = (plot_bottom - plot_top) / y_max

    def px(frame_index: int) -> int:
        return int(round(plot_left + (frame_index - x_min) * x_scale))

    def py(value: float) -> int:
        return int(round(plot_bottom - value * y_scale))

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    title_font, subtitle_font = font(44, True), font(25)
    axis_font, small_font = font(22), font(19)
    primary = summary[0]
    draw.text(
        (left, 35),
        f"Prediction callback latency — {primary['miss_count']} / {primary['eligible_frames']} misses ({primary['miss_rate_pct']:.2f}%)",
        fill="#172B4D",
        font=title_font,
    )
    draw.text(
        (left, 92),
        "Monotonic proc_enter to proc_exit; orange markers are empirical top-1% spikes",
        fill="#52606D",
        font=subtitle_font,
    )
    for tick in range(0, int(y_max) + 1, 50):
        y = py(float(tick))
        draw.line((plot_left, y, plot_right, y), fill="#D9E2EC", width=2)
        draw.text((55, y - 13), f"{tick} ms", fill="#52606D", font=small_font)
    for tick in range(0, 1001, 100):
        if x_min <= tick <= x_max:
            x = px(tick)
            draw.line((x, plot_bottom, x, plot_bottom + 8), fill="#52606D", width=2)
            draw.text((x - 18, plot_bottom + 18), str(tick), fill="#52606D", font=small_font)
    deadline_y = py(fixed_deadline_ms)
    x = plot_left
    while x < plot_right:
        draw.line((x, deadline_y, min(x + 14, plot_right), deadline_y), fill="#A31621", width=4)
        x += 28
    points = [(px(row["frame_index"]), py(row["proc_ms"])) for row in detail]
    draw.line(points, fill="#52606D", width=2)
    for row in misses:
        x, y = px(row["frame_index"]), py(row["proc_ms"])
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill="#D64545")
    for row in extremes:
        x, y = px(row["frame_index"]), py(row["proc_ms"])
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill="#FFB000", outline="#202020", width=2)
    draw.line((plot_left, plot_top, plot_left, plot_bottom), fill="#52606D", width=3)
    draw.line((plot_left, plot_bottom, plot_right, plot_bottom), fill="#52606D", width=3)
    draw.text((width // 2 - 150, height - 68), "Complete callback frame index", fill="#334E68", font=axis_font)
    draw.rectangle((left, height - 122, left + 24, height - 98), fill="#D64545")
    draw.text((left + 34, height - 125), "deadline miss", fill="#334E68", font=small_font)
    draw.ellipse((left + 220, height - 123, left + 244, height - 99), fill="#FFB000", outline="#202020", width=2)
    draw.text((left + 254, height - 125), "top-1% spike", fill="#334E68", font=small_font)
    image.save(figure_dir / "prediction_deadline_timeline.png")

    colors = {
        "container": "#4C78A8",
        "semantic_map_run": "#F58518",
        "thread_pool_run": "#E45756",
        "predictor": "#72B7B2",
        "writer": "#54A24B",
        GANTT_ASYNC_NODE: "#B279A2",
    }
    selected = sorted(extremes, key=lambda row: row["proc_ms"], reverse=True)
    width = 2800
    row_height = 104
    height = 300 + row_height * len(selected)
    left, right, top, bottom = 600, 230, 165, 150
    plot_left, plot_right = left, width - right
    plot_top, plot_bottom = top, height - bottom
    x_max_ms = max(300.0, math.ceil(max(row["proc_ms"] for row in selected) / 50.0) * 50.0)
    x_scale = (plot_right - plot_left) / x_max_ms

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    draw.text(
        (left, 35),
        "Prediction top-1% callback spikes — stage Gantt",
        fill="#172B4D",
        font=font(46, True),
    )
    draw.text(
        (left, 95),
        "Outline = proc span; slim lower bar = parallel semantic_base_async_draw",
        fill="#52606D",
        font=font(25),
    )
    for tick in range(0, int(x_max_ms) + 1, 50):
        x = int(round(plot_left + tick * x_scale))
        draw.line((x, plot_top, x, plot_bottom), fill="#E4E7EB", width=2)
        draw.text((x - 24, plot_bottom + 18), f"{tick}", fill="#52606D", font=font(19))
    deadline_x = int(round(plot_left + fixed_deadline_ms * x_scale))
    dashed_vertical(draw, deadline_x, plot_top, plot_bottom, "#A31621", width=4)
    draw.text((deadline_x + 8, plot_top - 35), f"{fixed_deadline_ms:g} ms deadline", fill="#A31621", font=font(20, True))

    y_by_trace: dict[int, int] = {}
    for index, frame in enumerate(selected):
        center_y = plot_top + 48 + index * row_height
        y_by_trace[int(frame["trace_id"])] = center_y
        label = f"#{frame['proc_extreme_rank']}  F{frame['frame_index']} | in {frame['input_perception_seq']} | TID {frame['proc_enter_tid']}"
        draw.text((38, center_y - 20), label, fill="#243B53", font=font(23, True))
        proc_end = int(round(plot_left + frame["proc_ms"] * x_scale))
        draw.rectangle(
            (plot_left, center_y - 28, proc_end, center_y + 28),
            outline="#263238",
            width=3,
        )
        draw.text(
            (proc_end + 12, center_y - 15),
            f"{frame['proc_ms']:.1f} ms",
            fill="#263238",
            font=font(20),
        )

    for row in gantt_rows:
        center_y = y_by_trace[int(row["trace_id"])]
        x0 = int(round(plot_left + row["start_from_proc_enter_ms"] * x_scale))
        x1 = max(x0 + 2, int(round(plot_left + row["end_from_proc_enter_ms"] * x_scale)))
        if row["lane"] == "async":
            rect = (x0, center_y + 10, x1, center_y + 24)
        else:
            rect = (x0, center_y - 23, x1, center_y + 4)
        draw.rectangle(rect, fill=colors[row["node"]], outline="white", width=1)

    legend_x, legend_y = left, height - 86
    for node, color in colors.items():
        draw.rectangle((legend_x, legend_y, legend_x + 30, legend_y + 22), fill=color)
        draw.text((legend_x + 40, legend_y - 3), node, fill="#334E68", font=font(18))
        legend_x += 300 if node != GANTT_ASYNC_NODE else 390
        if legend_x > width - 420:
            legend_x = left
            legend_y += 34
    draw.text((width // 2 - 130, height - 38), "Time from proc_enter (ms)", fill="#334E68", font=font(22))
    image.save(figure_dir / "prediction_extreme_frames_gantt.png")


def write_summary(
    path: Path,
    run_id: str,
    detail: list[dict[str, Any]],
    deadline_summary: list[dict[str, Any]],
    intervals: list[dict[str, Any]],
    extremes: list[dict[str, Any]],
    p99_threshold: float,
    validation: dict[str, Any],
    execution_regimes: list[dict[str, Any]],
    stage_breakdown: list[dict[str, Any]],
) -> None:
    proc = [row["proc_ms"] for row in detail]
    primary = deadline_summary[0]
    observed = deadline_summary[2]
    model_present = next(row for row in execution_regimes if row["regime"].endswith("present"))
    model_absent = next(row for row in execution_regimes if row["regime"].endswith("absent"))
    thread_pool_shares = [row["thread_pool_share_of_proc_pct"] for row in stage_breakdown]
    lines = [
        f"# Prediction 打点逐帧与 deadline 分析（{run_id}）",
        "",
        "## 结论",
        "",
        f"- 完整 callback：{len(detail)} 帧；固定 deadline 采用 `proc_enter + {primary['deadline_ms_median']:.0f} ms`，完成点为 `proc_exit`。",
        f"- 固定 deadline miss：{primary['miss_count']} 帧，miss rate = {primary['miss_count']} / {primary['eligible_frames']} = {primary['miss_rate_pct']:.4f}%。",
        f"- Prediction callback 时延：median {statistics.median(proc):.3f} ms，P95 {quantile(proc, 0.95):.3f} ms，P99 {quantile(proc, 0.99):.3f} ms，max {max(proc):.3f} ms。",
        f"- 按 `proc_ms > P99 ({p99_threshold:.3f} ms)` 定义尖峰异常，共 {len(extremes)} 帧。",
        f"- 以“下一次实际 proc_enter”为观测截止点时：{observed['miss_count']} / {observed['eligible_frames']} miss；该口径受 callback 串行派发影响，只作重叠诊断，不替代 100 ms 固定要求。",
        f"- 固定 deadline miss 形成 {len(intervals)} 个连续区间。",
        f"- 关联性诊断：`model_cruise_mlp` 出现的 {model_present['frames']} 帧中有 {model_present['fixed_deadline_misses']} 帧 miss（{model_present['fixed_deadline_miss_rate_pct']:.2f}%）；未出现的 {model_absent['frames']} 帧为 {model_absent['fixed_deadline_misses']} miss。该结果说明 miss 集中于模型执行帧，但本身不证明因果。",
        f"- top-1% 尖峰中，`thread_pool_run` 占 proc span 的中位数为 {statistics.median(thread_pool_shares):.2f}%（范围 {min(thread_pool_shares):.2f}%–{max(thread_pool_shares):.2f}%），是甘特图中的主导区段。",
        "",
        "## 尖峰异常帧",
        "",
        "| rank | frame | input seq | output seq | TID | proc ms | overrun ms |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in extremes:
        lines.append(
            f"| {row['proc_extreme_rank']} | {row['frame_index']} | {row['input_perception_seq']} | "
            f"{row['prediction_output_seq']} | {row['proc_enter_tid']} | {row['proc_ms']:.3f} | "
            f"{row['fixed_callback_overrun_ms']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## 口径与证据边界",
            "",
            "- `proc_ms = proc_exit - proc_enter`，使用 Apollo trace 的 monotonic wall-clock；节点 span 可嵌套或并行，不能相加得到 proc_ms。",
            "- 100 ms 是本次分析的显式固定阈值；若系统规格另有配置值，可用脚本参数重算。",
            "- `frame_complete_ms` 包含 proc_exit 后的异步事件，单独列为诊断，不覆盖 callback 主结果。",
            "- 同帧重复的 notify-task / MLP 节点按事件顺序逐实例配对，并另存逐帧调用次数、总耗时和最大单次耗时。",
            f"- 质量状态：`{validation.get('status')}`。原始 {validation['counts'].get('raw_trace_frames')} 个 trace callback 中，末尾 {validation['counts'].get('skipped_incomplete_trace_frames')} 个不完整 callback 不进入分母；另有 {validation['quality_checks'].get('malformed_event_rows')} 个不完整 CSV 尾行。",
            f"- record 负载字段成功关联 {validation['counts'].get('record_load_linked_frames')} / {len(detail)} 帧；未关联帧的 obstacle/trajectory 字段保持空值，不影响 trace 时延统计。",
            f"- Prediction 日志 TID 与 trace `proc_enter_tid`：{validation['tid_log_trace_audit']['matching_frames']} / {validation['tid_log_trace_audit']['linked_frames']} 个可关联帧一致，mismatch {validation['tid_log_trace_audit']['mismatching_frames']} 个；不可关联 {validation['tid_log_trace_audit']['unlinked_frames']} 帧。",
            "",
            "## 主要文件",
            "",
            "- `data/prediction_deadline_detail.csv`：逐帧固定/观测 deadline、slack、miss、TID。",
            "- `data/prediction_deadline_summary.csv`：各口径分母、miss count 与 miss rate。",
            "- `data/prediction_deadline_miss_frames.csv`：全部固定 deadline miss 帧。",
            "- `data/prediction_extreme_frames.csv`：proc top-1% 尖峰帧。",
            "- `data/prediction_variable_node_frame_aggregates.csv`：重复/可选节点逐帧聚合。",
            "- `data/prediction_execution_regimes.csv`：模型执行帧/非模型帧的时延与 miss 对照。",
            "- `data/prediction_extreme_stage_breakdown.csv`：尖峰帧关键阶段耗时与占比。",
            "- `data/prediction_tid_log_trace_audit.csv`：日志 TID 与 trace TID 逐帧核对。",
            "- `figures/prediction_extreme_frames_gantt.png`：尖峰帧关键阶段甘特图。",
            "- `figures/prediction_deadline_timeline.png`：全程 deadline 与 miss 分布。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_manifest(analysis_dir: Path) -> None:
    manifest_path = analysis_dir / "output_manifest.csv"
    rows = []
    for path in sorted(analysis_dir.rglob("*")):
        if not path.is_file() or path == manifest_path or "__pycache__" in path.parts:
            continue
        rows.append(
            {
                "file": str(path.relative_to(analysis_dir)),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    write_csv(manifest_path, rows)


def main() -> None:
    args = parse_args()
    data_dir = args.analysis_dir / "data"
    frame_path = data_dir / "prediction_frame_node_timings.csv"
    long_path = data_dir / "prediction_node_timings_long.csv"
    validation_path = data_dir / "validation_summary.json"
    events_path = one(args.run_dir / "trace" / "events", "prediction.*.csv")
    prediction_log_path = one(args.run_dir / "log", "prediction.log.INFO.*")

    raw_frames = read_csv(frame_path)
    detail, deadline_summary, intervals = build_deadlines(
        raw_frames, args.fixed_deadline_ms
    )
    tid_audit_rows, tid_audit_summary = attach_log_tids(detail, prediction_log_path)
    extremes, threshold = select_extremes(detail, args.extreme_fraction)
    frame_by_trace = {int(row["trace_id"]): row for row in detail}
    variable_rows = aggregate_variable_nodes(events_path, frame_by_trace)
    variable_summary_rows = variable_summary(variable_rows)
    gantt_rows = build_gantt_data(detail, extremes, read_csv(long_path))
    stage_breakdown = build_stage_breakdown(extremes, raw_frames)
    execution_regimes = build_execution_regimes(detail, variable_rows)

    write_csv(data_dir / "prediction_deadline_detail.csv", detail)
    write_csv(data_dir / "prediction_deadline_summary.csv", deadline_summary)
    write_csv(
        data_dir / "prediction_deadline_miss_frames.csv",
        [row for row in detail if row["fixed_callback_deadline_miss"]],
    )
    write_csv(data_dir / "prediction_deadline_miss_intervals.csv", intervals)
    write_csv(data_dir / "prediction_extreme_frames.csv", extremes)
    write_csv(data_dir / "prediction_variable_node_frame_aggregates.csv", variable_rows)
    write_csv(data_dir / "prediction_variable_node_summary.csv", variable_summary_rows)
    write_csv(data_dir / "prediction_extreme_frames_gantt_data.csv", gantt_rows)
    write_csv(data_dir / "prediction_tid_log_trace_audit.csv", tid_audit_rows)
    write_csv(data_dir / "prediction_extreme_stage_breakdown.csv", stage_breakdown)
    write_csv(data_dir / "prediction_execution_regimes.csv", execution_regimes)

    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    validation["deadline_results"] = {
        "primary_definition": f"proc_exit <= proc_enter + {args.fixed_deadline_ms:g} ms",
        "eligible_frames": deadline_summary[0]["eligible_frames"],
        "miss_count": deadline_summary[0]["miss_count"],
        "miss_rate": deadline_summary[0]["miss_rate"],
        "miss_rate_pct": deadline_summary[0]["miss_rate_pct"],
        "extreme_definition": f"proc_ms > empirical P{100 * (1 - args.extreme_fraction):g}",
        "extreme_threshold_ms": threshold,
        "extreme_frame_count": len(extremes),
        "observed_next_input_is_diagnostic_only": True,
    }
    validation["tid_log_trace_audit"] = {
        "source": str(prediction_log_path),
        **tid_audit_summary,
    }
    validation["diagnostic_findings"] = {
        "model_cruise_mlp_present": execution_regimes[0],
        "model_cruise_mlp_absent": execution_regimes[1],
        "top_1pct_thread_pool_share_of_proc_pct_median": statistics.median(
            row["thread_pool_share_of_proc_pct"] for row in stage_breakdown
        ),
        "top_1pct_thread_pool_share_of_proc_pct_min": min(
            row["thread_pool_share_of_proc_pct"] for row in stage_breakdown
        ),
        "top_1pct_thread_pool_share_of_proc_pct_max": max(
            row["thread_pool_share_of_proc_pct"] for row in stage_breakdown
        ),
        "causality_claimed": False,
    }
    validation_path.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    render_charts(
        args.analysis_dir,
        detail,
        deadline_summary,
        extremes,
        gantt_rows,
        args.fixed_deadline_ms,
    )
    write_summary(
        args.analysis_dir / "分析说明.md",
        args.run_dir.name,
        detail,
        deadline_summary,
        intervals,
        extremes,
        threshold,
        validation,
        execution_regimes,
        stage_breakdown,
    )
    write_manifest(args.analysis_dir)
    print(
        json.dumps(
            {
                "complete_frames": len(detail),
                "fixed_deadline_misses": deadline_summary[0]["miss_count"],
                "fixed_deadline_miss_rate_pct": deadline_summary[0]["miss_rate_pct"],
                "extreme_frames": len(extremes),
                "p99_threshold_ms": threshold,
                "variable_node_frame_rows": len(variable_rows),
                "tid_log_trace_audit": tid_audit_summary,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
