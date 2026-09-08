#!/usr/bin/env python3
"""Analyze e2e_trace_v3 Prediction node spans frame by frame."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sqlite3
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


SPECIAL_PAIRS = {"writer": ("writer_enter", "writer_done")}
ROLLUP_NODES = {
    "proc",
    "perception",
    "evaluator",
    "semantic_map_run",
    "semantic_base_image",
    "semantic_base_async_schedule",
    "semantic_base_async_submit",
    "semantic_base_async_draw",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate per-frame Prediction node timings and robust anomaly flags."
    )
    parser.add_argument("run_id")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--events", type=Path)
    parser.add_argument("--context", type=Path)
    parser.add_argument("--fusion-context", type=Path)
    parser.add_argument("--prediction-raw", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fixed-deadline-ms", type=float, default=100.0)
    parser.add_argument("--robust-z", type=float, default=6.0)
    parser.add_argument("--min-effect-ms", type=float, default=0.05)
    parser.add_argument("--localized-min-excess-ms", type=float, default=0.2)
    return parser.parse_args()


def one(path: Path, pattern: str) -> Path:
    matches = sorted(path.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(f"expected one {pattern} under {path}, found {len(matches)}")
    return matches[0]


def to_int(value: Any) -> int | None:
    if value in (None, ""):
        return None
    return int(value)


def to_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def quantile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def fmt(value: Any, digits: int = 6) -> Any:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}"
    if value is None:
        return ""
    return value


def write_csv(path: Path, rows: list[dict[str, Any]], fields: Iterable[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(fields or (rows[0].keys() if rows else []))
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key)) for key in fieldnames})


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def write_sqlite_snapshot(
    path: Path,
    frame_rows: list[dict[str, Any]],
    node_summary_rows: list[dict[str, Any]],
    anomaly_rows: list[dict[str, Any]],
    cadence_rows: list[dict[str, Any]],
) -> None:
    if path.exists():
        path.unlink()
    connection = sqlite3.connect(path)
    try:
        connection.executescript(
            """
            CREATE TABLE frame_timings (
                frame_index INTEGER PRIMARY KEY,
                trace_id TEXT NOT NULL,
                input_seq INTEGER NOT NULL,
                output_seq INTEGER NOT NULL,
                prediction_header_time_s REAL,
                obstacle_count INTEGER,
                proc_ms REAL NOT NULL,
                frame_complete_ms REAL NOT NULL,
                async_tail_ms REAL NOT NULL,
                input_interarrival_ms REAL,
                dynamic_deadline_slack_ms REAL,
                fixed_deadline_miss INTEGER NOT NULL,
                dynamic_deadline_miss INTEGER,
                async_overlaps_next_frame INTEGER,
                proc_top_1pct INTEGER NOT NULL,
                frame_complete_top_1pct INTEGER NOT NULL,
                localized_node_spike INTEGER NOT NULL,
                notify_tasks_ms REAL NOT NULL,
                async_draw_ms REAL NOT NULL,
                draw_lanes_ms REAL NOT NULL
            );
            CREATE TABLE node_summary (
                node TEXT PRIMARY KEY,
                median_ms REAL NOT NULL,
                p95_ms REAL NOT NULL,
                p99_ms REAL NOT NULL,
                max_ms REAL NOT NULL,
                max_frame INTEGER NOT NULL,
                max_output_seq INTEGER NOT NULL
            );
            CREATE TABLE anomalous_frames (
                severity TEXT NOT NULL,
                frame_index INTEGER NOT NULL,
                input_seq INTEGER NOT NULL,
                output_seq INTEGER NOT NULL,
                proc_ms REAL NOT NULL,
                frame_complete_ms REAL NOT NULL,
                dominant_leaf_node TEXT,
                localized_spike_nodes TEXT,
                reason TEXT NOT NULL
            );
            CREATE TABLE cadence_gaps (
                before_frame_index INTEGER NOT NULL,
                before_input_seq INTEGER NOT NULL,
                after_input_seq INTEGER NOT NULL,
                input_sequence_gap_count INTEGER NOT NULL,
                input_interarrival_ms REAL NOT NULL,
                input_interarrival_top_1pct INTEGER NOT NULL,
                dynamic_deadline_slack_ms REAL NOT NULL
            );
            """
        )
        connection.executemany(
            "INSERT INTO frame_timings VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            [
                (
                    row["frame_index"],
                    str(row["trace_id"]),
                    row["input_perception_seq"],
                    row["prediction_output_seq"],
                    row["prediction_header_time_s"],
                    row["prediction_obstacle_count"],
                    row["proc_ms"],
                    row["frame_complete_ms"],
                    row["async_tail_ms"],
                    row["input_interarrival_ms"],
                    row["dynamic_deadline_slack_ms"],
                    int(bool(row["fixed_deadline_miss"])),
                    None if row["dynamic_deadline_miss"] is None else int(bool(row["dynamic_deadline_miss"])),
                    None if row["async_overlaps_next_frame"] is None else int(bool(row["async_overlaps_next_frame"])),
                    int(bool(row["proc_top_1pct"])),
                    int(bool(row["frame_complete_top_1pct"])),
                    int(bool(row["localized_node_spike"])),
                    row["cyber_taskmanager_notify_tasks_ms"],
                    row["semantic_base_async_draw_ms"],
                    row["semantic_draw_lanes_ms"],
                )
                for row in frame_rows
            ],
        )
        connection.executemany(
            "INSERT INTO node_summary VALUES (?,?,?,?,?,?,?)",
            [
                (
                    row["node"],
                    row["median_ms"],
                    row["p95_ms"],
                    row["p99_ms"],
                    row["max_ms"],
                    row["max_frame_index"],
                    row["max_prediction_output_seq"],
                )
                for row in node_summary_rows
            ],
        )
        connection.executemany(
            "INSERT INTO anomalous_frames VALUES (?,?,?,?,?,?,?,?,?)",
            [
                (
                    row["severity"],
                    row["frame_index"],
                    row["input_perception_seq"],
                    row["prediction_output_seq"],
                    row["proc_ms"],
                    row["frame_complete_ms"],
                    row["dominant_leaf_node"],
                    row["localized_spike_nodes"],
                    row["reason"],
                )
                for row in anomaly_rows
            ],
        )
        connection.executemany(
            "INSERT INTO cadence_gaps VALUES (?,?,?,?,?,?,?)",
            [
                (
                    row["before_frame_index"],
                    row["before_input_perception_seq"],
                    row["after_input_perception_seq"],
                    row["input_sequence_gap_count"] or 0,
                    row["input_interarrival_ms"],
                    int(bool(row["input_interarrival_top_1pct"])),
                    row["dynamic_deadline_slack_ms"],
                )
                for row in cadence_rows
            ],
        )
        connection.commit()
    finally:
        connection.close()


def read_events(path: Path) -> tuple[dict[int, dict[str, dict[str, Any]]], dict[str, int], int]:
    by_trace: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    phase_counts: Counter[str] = Counter()
    event_ids: set[int] = set()
    duplicate_event_ids = 0
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            trace_id = int(row["trace_id"])
            phase = row["phase"]
            event_id = int(row["event_id"])
            if event_id in event_ids:
                duplicate_event_ids += 1
            event_ids.add(event_id)
            if phase in by_trace[trace_id]:
                raise RuntimeError(f"duplicate phase {phase} for trace {trace_id}")
            row["event_id"] = event_id
            row["mono_ns"] = int(row["mono_ns"])
            row["pid"] = int(row["pid"])
            row["tid"] = int(row["tid"])
            by_trace[trace_id][phase] = row
            phase_counts[phase] += 1
    return by_trace, dict(phase_counts), duplicate_event_ids


def read_context(path: Path) -> tuple[dict[int, dict[str, dict[str, Any]]], int]:
    by_trace: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    duplicate_edges = 0
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            trace_id = int(row["trace_id"])
            edge = row["edge"]
            if edge in by_trace[trace_id]:
                duplicate_edges += 1
            by_trace[trace_id][edge] = row
    return by_trace, duplicate_edges


def read_prediction_load(path: Path | None) -> dict[int, dict[str, Any]]:
    if path is None or not path.exists():
        return {}
    result: dict[int, dict[str, Any]] = {}
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            sequence = to_int(row.get("header_sequence_num"))
            if sequence is None:
                continue
            raw = json.loads(row["raw_json"])
            obstacles = raw.get("prediction_obstacle", []) or []
            trajectory_count = sum(len(item.get("trajectory", []) or []) for item in obstacles)
            trajectory_points = sum(
                len(trajectory.get("trajectory_point", []) or [])
                for item in obstacles
                for trajectory in (item.get("trajectory", []) or [])
            )
            result[sequence] = {
                "prediction_header_time_s": to_float(row.get("header_timestamp_sec")),
                "record_time_s": to_float(row.get("record_time_sec")),
                "prediction_obstacle_count": len(obstacles),
                "prediction_trajectory_count": trajectory_count,
                "prediction_trajectory_point_count": trajectory_points,
            }
    return result


def read_fusion_outputs(path: Path | None) -> set[int]:
    if path is None or not path.exists():
        return set()
    output_sequences: set[int] = set()
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            if row.get("edge") == "out" and row.get("output_seq") not in (None, ""):
                output_sequences.add(int(row["output_seq"]))
    return output_sequences


def discover_pairs(first_trace: dict[str, dict[str, Any]]) -> list[tuple[str, str, str]]:
    phases = set(first_trace)
    pairs: list[tuple[str, str, str]] = []
    for phase, row in sorted(first_trace.items(), key=lambda item: item[1]["event_id"]):
        if not phase.endswith("_enter"):
            continue
        node = phase[: -len("_enter")]
        exit_phase = f"{node}_exit"
        if exit_phase in phases:
            pairs.append((node, phase, exit_phase))
        elif node in SPECIAL_PAIRS and SPECIAL_PAIRS[node][1] in phases:
            pairs.append((node, phase, SPECIAL_PAIRS[node][1]))
    return pairs


def metric_stats(values: list[float], robust_z: float, min_effect_ms: float) -> dict[str, float]:
    median = statistics.median(values)
    mad = statistics.median(abs(value - median) for value in values)
    robust_sigma = 1.4826 * mad
    effect_floor = max(min_effect_ms, abs(median) * 0.25)
    score_denominator = max(robust_sigma, effect_floor / robust_z)
    return {
        "count": float(len(values)),
        "min_ms": min(values),
        "mean_ms": statistics.fmean(values),
        "median_ms": median,
        "p90_ms": quantile(values, 0.90),
        "p95_ms": quantile(values, 0.95),
        "p99_ms": quantile(values, 0.99),
        "max_ms": max(values),
        "mad_ms": mad,
        "robust_sigma_ms": robust_sigma,
        "robust_upper_ms": median + robust_z * robust_sigma,
        "effect_floor_ms": effect_floor,
        "score_denominator_ms": score_denominator,
    }


def is_outlier(value: float, stats: dict[str, float]) -> bool:
    return (
        value > stats["robust_upper_ms"]
        and value - stats["median_ms"] >= stats["effect_floor_ms"]
    )


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir or Path.cwd() / args.run_id
    events_path = args.events or one(run_dir / "trace" / "events", "prediction.*.csv")
    context_path = args.context or one(run_dir / "trace" / "message_context", "prediction.*.csv")
    fusion_context = args.fusion_context
    if fusion_context is None:
        candidates = sorted(
            (run_dir / "trace" / "message_context").glob("perception.multi_sensor_fusion.*.csv")
        )
        fusion_context = candidates[0] if len(candidates) == 1 else None
    prediction_raw = args.prediction_raw
    if prediction_raw is None:
        candidate = run_dir / "record" / "04_prediction_perception" / "prediction_raw.jsonl"
        prediction_raw = candidate if candidate.exists() else None
    output_dir = args.output_dir or (
        run_dir / "打点逐帧数据统计" / "prediction数据统计" / "节点时间分析"
    )
    data_dir = output_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    events, phase_counts, duplicate_event_ids = read_events(events_path)
    contexts, duplicate_context_edges = read_context(context_path)
    prediction_load = read_prediction_load(prediction_raw)
    fusion_outputs = read_fusion_outputs(fusion_context)
    if not events:
        raise RuntimeError("Prediction event trace is empty")

    ordered_trace_ids = sorted(events, key=lambda trace_id: events[trace_id]["proc_enter"]["mono_ns"])
    node_pairs = discover_pairs(events[ordered_trace_ids[0]])
    expected_phases = set(events[ordered_trace_ids[0]])
    missing_phases: list[dict[str, Any]] = []
    negative_spans: list[dict[str, Any]] = []
    context_missing_in = 0
    context_missing_out = 0
    frame_rows: list[dict[str, Any]] = []
    node_rows: list[dict[str, Any]] = []

    for frame_index, trace_id in enumerate(ordered_trace_ids, start=1):
        trace = events[trace_id]
        missing = sorted(expected_phases - set(trace))
        extra = sorted(set(trace) - expected_phases)
        if missing or extra:
            missing_phases.append({"trace_id": trace_id, "missing": missing, "extra": extra})
        context = contexts.get(trace_id, {})
        input_context = context.get("in")
        output_context = context.get("out")
        context_missing_in += int(input_context is None)
        context_missing_out += int(output_context is None)
        input_seq = to_int(input_context.get("input_seq")) if input_context else None
        output_seq = to_int(output_context.get("output_seq")) if output_context else None
        proc_enter = trace["proc_enter"]["mono_ns"]
        proc_exit = trace["proc_exit"]["mono_ns"]
        output_pub = trace["output_pub"]["mono_ns"]
        last_event = max(row["mono_ns"] for row in trace.values())
        frame = {
            "frame_index": frame_index,
            "trace_id": trace_id,
            "input_perception_seq": input_seq,
            "prediction_output_seq": output_seq,
            "input_data_ts_ns": to_int(input_context.get("data_ts_ns")) if input_context else None,
            "proc_enter_mono_ns": proc_enter,
            "output_pub_mono_ns": output_pub,
            "proc_exit_mono_ns": proc_exit,
            "last_event_mono_ns": last_event,
            "publish_latency_ms": (output_pub - proc_enter) / 1e6,
            "frame_complete_ms": (last_event - proc_enter) / 1e6,
            "async_tail_ms": max(0.0, (last_event - proc_exit) / 1e6),
            "input_interarrival_ms": None,
            "next_input_perception_seq": None,
            "input_sequence_delta": None,
            "input_sequence_gap_count": None,
            "output_sequence_delta": None,
            "dynamic_deadline_slack_ms": None,
            "dynamic_deadline_miss": None,
            "async_overlaps_next_frame": None,
        }
        if output_seq is not None and output_seq in prediction_load:
            frame.update(prediction_load[output_seq])
        else:
            frame.update(
                {
                    "prediction_header_time_s": None,
                    "record_time_s": None,
                    "prediction_obstacle_count": None,
                    "prediction_trajectory_count": None,
                    "prediction_trajectory_point_count": None,
                }
            )
        for node, enter_phase, exit_phase in node_pairs:
            if enter_phase not in trace or exit_phase not in trace:
                continue
            enter = trace[enter_phase]
            exit_row = trace[exit_phase]
            duration_ms = (exit_row["mono_ns"] - enter["mono_ns"]) / 1e6
            if duration_ms < 0:
                negative_spans.append(
                    {"trace_id": trace_id, "node": node, "duration_ms": duration_ms}
                )
            frame[f"{node}_ms"] = duration_ms
            node_rows.append(
                {
                    "frame_index": frame_index,
                    "trace_id": trace_id,
                    "input_perception_seq": input_seq,
                    "prediction_output_seq": output_seq,
                    "node": node,
                    "enter_phase": enter_phase,
                    "exit_phase": exit_phase,
                    "enter_event_id": enter["event_id"],
                    "exit_event_id": exit_row["event_id"],
                    "enter_mono_ns": enter["mono_ns"],
                    "exit_mono_ns": exit_row["mono_ns"],
                    "enter_tid": enter["tid"],
                    "exit_tid": exit_row["tid"],
                    "duration_ms": duration_ms,
                }
            )
        frame_rows.append(frame)

    for index, frame in enumerate(frame_rows[:-1]):
        next_enter = frame_rows[index + 1]["proc_enter_mono_ns"]
        interarrival = (next_enter - frame["proc_enter_mono_ns"]) / 1e6
        current_input_seq = to_int(frame["input_perception_seq"])
        next_input_seq = to_int(frame_rows[index + 1]["input_perception_seq"])
        current_output_seq = to_int(frame["prediction_output_seq"])
        next_output_seq = to_int(frame_rows[index + 1]["prediction_output_seq"])
        frame["input_interarrival_ms"] = interarrival
        frame["next_input_perception_seq"] = next_input_seq
        frame["input_sequence_delta"] = (
            next_input_seq - current_input_seq
            if current_input_seq is not None and next_input_seq is not None
            else None
        )
        frame["input_sequence_gap_count"] = (
            max(0, int(frame["input_sequence_delta"]) - 1)
            if frame["input_sequence_delta"] is not None
            else None
        )
        frame["output_sequence_delta"] = (
            next_output_seq - current_output_seq
            if current_output_seq is not None and next_output_seq is not None
            else None
        )
        frame["dynamic_deadline_slack_ms"] = (next_enter - frame["proc_exit_mono_ns"]) / 1e6
        frame["dynamic_deadline_miss"] = frame["proc_exit_mono_ns"] > next_enter
        frame["async_overlaps_next_frame"] = frame["last_event_mono_ns"] > next_enter

    metric_values: dict[str, list[float]] = {
        node: [float(row[f"{node}_ms"]) for row in frame_rows] for node, _, _ in node_pairs
    }
    metric_values.update(
        {
            "publish_latency": [float(row["publish_latency_ms"]) for row in frame_rows],
            "frame_complete": [float(row["frame_complete_ms"]) for row in frame_rows],
            "async_tail": [float(row["async_tail_ms"]) for row in frame_rows],
            "input_interarrival": [
                float(row["input_interarrival_ms"])
                for row in frame_rows
                if row["input_interarrival_ms"] is not None
            ],
        }
    )
    stats_by_metric = {
        metric: metric_stats(values, args.robust_z, args.min_effect_ms)
        for metric, values in metric_values.items()
    }

    node_row_lookup: dict[tuple[int, str], dict[str, Any]] = {}
    for row in node_rows:
        node = row["node"]
        stats = stats_by_metric[node]
        value = float(row["duration_ms"])
        score = (value - stats["median_ms"]) / stats["score_denominator_ms"]
        row["median_ms"] = stats["median_ms"]
        row["p99_ms"] = stats["p99_ms"]
        row["robust_upper_ms"] = stats["robust_upper_ms"]
        row["excess_over_median_ms"] = value - stats["median_ms"]
        row["robust_score"] = score
        row["is_robust_outlier"] = is_outlier(value, stats)
        row["is_top_1pct"] = value >= stats["p99_ms"]
        node_row_lookup[(int(row["frame_index"]), str(node))] = row

    node_summary_rows: list[dict[str, Any]] = []
    for node, _, _ in node_pairs:
        stats = stats_by_metric[node]
        rows = [row for row in node_rows if row["node"] == node]
        maximum = max(rows, key=lambda row: float(row["duration_ms"]))
        node_summary_rows.append(
            {
                "node": node,
                **{key: value for key, value in stats.items() if key != "score_denominator_ms"},
                "robust_outlier_count": sum(bool(row["is_robust_outlier"]) for row in rows),
                "top_1pct_count": sum(bool(row["is_top_1pct"]) for row in rows),
                "max_frame_index": maximum["frame_index"],
                "max_trace_id": maximum["trace_id"],
                "max_input_perception_seq": maximum["input_perception_seq"],
                "max_prediction_output_seq": maximum["prediction_output_seq"],
            }
        )

    frame_span_summary_rows: list[dict[str, Any]] = []
    for metric in ("publish_latency", "frame_complete", "async_tail", "input_interarrival"):
        stats = stats_by_metric[metric]
        frame_span_summary_rows.append(
            {
                "metric": metric,
                **{key: value for key, value in stats.items() if key != "score_denominator_ms"},
                "robust_outlier_count": sum(
                    is_outlier(value, stats) for value in metric_values[metric]
                ),
                "above_p99_count": sum(value > stats["p99_ms"] for value in metric_values[metric]),
            }
        )

    anomaly_rows: list[dict[str, Any]] = []
    cadence_rows: list[dict[str, Any]] = []
    for frame in frame_rows:
        frame_index = int(frame["frame_index"])
        robust_outliers = [
            node_row_lookup[(frame_index, node)]
            for node, _, _ in node_pairs
            if node_row_lookup[(frame_index, node)]["is_robust_outlier"]
        ]
        tail_nodes = [
            node_row_lookup[(frame_index, node)]
            for node, _, _ in node_pairs
            if node_row_lookup[(frame_index, node)]["is_top_1pct"]
        ]
        localized_spikes = [
            row
            for row in tail_nodes
            if row["node"] not in ROLLUP_NODES
            and float(row["excess_over_median_ms"])
            >= max(
                args.localized_min_excess_ms,
                stats_by_metric[str(row["node"])] ["median_ms"] * 0.5,
            )
        ]
        derived_outliers: list[str] = []
        derived_scores: list[float] = []
        for metric, field in (
            ("publish_latency", "publish_latency_ms"),
            ("frame_complete", "frame_complete_ms"),
            ("async_tail", "async_tail_ms"),
        ):
            value = float(frame[field])
            stats = stats_by_metric[metric]
            score = (value - stats["median_ms"]) / stats["score_denominator_ms"]
            frame[f"{metric}_robust_score"] = score
            frame[f"{metric}_robust_outlier"] = is_outlier(value, stats)
            if frame[f"{metric}_robust_outlier"]:
                derived_outliers.append(metric)
                derived_scores.append(score)
        proc_ms = float(frame["proc_ms"])
        fixed_deadline_miss = proc_ms > args.fixed_deadline_ms
        frame["fixed_deadline_miss"] = fixed_deadline_miss
        deadline_miss = bool(frame["dynamic_deadline_miss"]) or fixed_deadline_miss
        proc_top_1pct = proc_ms > stats_by_metric["proc"]["p99_ms"]
        complete_top_1pct = (
            float(frame["frame_complete_ms"]) > stats_by_metric["frame_complete"]["p99_ms"]
        )
        async_tail_top_1pct = (
            float(frame["async_tail_ms"]) > stats_by_metric["async_tail"]["p99_ms"]
        )
        frame["proc_top_1pct"] = proc_top_1pct
        frame["frame_complete_top_1pct"] = complete_top_1pct
        frame["async_tail_top_1pct"] = async_tail_top_1pct
        frame["localized_node_spike"] = bool(localized_spikes)
        input_interarrival_top_1pct = (
            frame["input_interarrival_ms"] is not None
            and float(frame["input_interarrival_ms"])
            > stats_by_metric["input_interarrival"]["p99_ms"]
        )
        frame["input_interarrival_top_1pct"] = input_interarrival_top_1pct
        if (frame["input_sequence_gap_count"] or 0) > 0 or input_interarrival_top_1pct:
            cadence_rows.append(
                {
                    "before_frame_index": frame_index,
                    "before_trace_id": frame["trace_id"],
                    "before_input_perception_seq": frame["input_perception_seq"],
                    "after_input_perception_seq": frame["next_input_perception_seq"],
                    "input_sequence_gap_count": frame["input_sequence_gap_count"],
                    "input_interarrival_ms": frame["input_interarrival_ms"],
                    "input_interarrival_top_1pct": input_interarrival_top_1pct,
                    "before_proc_ms": proc_ms,
                    "dynamic_deadline_slack_ms": frame["dynamic_deadline_slack_ms"],
                    "scope": "upstream input cadence; not Prediction internal node time",
                }
            )
        aggregate_tail = proc_top_1pct or complete_top_1pct
        abnormal = deadline_miss or aggregate_tail or bool(localized_spikes)
        frame["abnormal_frame"] = abnormal
        dominant_leaf = max(
            localized_spikes,
            key=lambda row: float(row["excess_over_median_ms"]),
            default=None,
        )
        dominant_all = max(
            tail_nodes,
            key=lambda row: float(row["excess_over_median_ms"]),
            default=None,
        )
        reasons: list[str] = []
        if fixed_deadline_miss:
            reasons.append(f"proc>{args.fixed_deadline_ms:g}ms")
        if frame["dynamic_deadline_miss"]:
            reasons.append("proc overlaps next frame")
        if proc_top_1pct:
            reasons.append("proc above P99")
        if complete_top_1pct:
            reasons.append("frame completion above P99")
        if localized_spikes:
            reasons.append("localized leaf-node spike")
        max_score = max(
            [float(row["robust_score"]) for row in tail_nodes] + derived_scores,
            default=0.0,
        )
        if not abnormal:
            continue
        severity = "critical" if deadline_miss else ("high" if aggregate_tail else "medium")
        anomaly_rows.append(
            {
                "severity": severity,
                "frame_index": frame_index,
                "trace_id": frame["trace_id"],
                "input_perception_seq": frame["input_perception_seq"],
                "prediction_output_seq": frame["prediction_output_seq"],
                "prediction_header_time_s": frame["prediction_header_time_s"],
                "proc_ms": proc_ms,
                "publish_latency_ms": frame["publish_latency_ms"],
                "frame_complete_ms": frame["frame_complete_ms"],
                "async_tail_ms": frame["async_tail_ms"],
                "input_interarrival_ms": frame["input_interarrival_ms"],
                "dynamic_deadline_slack_ms": frame["dynamic_deadline_slack_ms"],
                "fixed_deadline_miss": fixed_deadline_miss,
                "dynamic_deadline_miss": frame["dynamic_deadline_miss"],
                "async_overlaps_next_frame": frame["async_overlaps_next_frame"],
                "prediction_obstacle_count": frame["prediction_obstacle_count"],
                "prediction_trajectory_count": frame["prediction_trajectory_count"],
                "proc_top_1pct": proc_top_1pct,
                "frame_complete_top_1pct": complete_top_1pct,
                "async_tail_top_1pct": async_tail_top_1pct,
                "node_tail_count": len(tail_nodes),
                "node_tail_exceedances": ";".join(str(row["node"]) for row in tail_nodes),
                "localized_spike_count": len(localized_spikes),
                "localized_spike_nodes": ";".join(str(row["node"]) for row in localized_spikes),
                "robust_diagnostic_nodes": ";".join(str(row["node"]) for row in robust_outliers),
                "derived_robust_diagnostics": ";".join(derived_outliers),
                "dominant_leaf_node": dominant_leaf["node"] if dominant_leaf else "",
                "dominant_leaf_excess_ms": dominant_leaf["excess_over_median_ms"] if dominant_leaf else None,
                "dominant_rollup_node": dominant_all["node"] if dominant_all else "",
                "dominant_rollup_excess_ms": dominant_all["excess_over_median_ms"] if dominant_all else None,
                "max_robust_score": max_score,
                "reason": "; ".join(reasons),
            }
        )
    severity_order = {"critical": 0, "high": 1, "medium": 2}
    anomaly_rows.sort(key=lambda row: (severity_order[row["severity"]], -float(row["max_robust_score"])))
    for rank, row in enumerate(anomaly_rows, start=1):
        row["anomaly_rank"] = rank

    frame_fields = [
        "frame_index",
        "trace_id",
        "input_perception_seq",
        "prediction_output_seq",
        "input_data_ts_ns",
        "prediction_header_time_s",
        "record_time_s",
        "prediction_obstacle_count",
        "prediction_trajectory_count",
        "prediction_trajectory_point_count",
        "proc_enter_mono_ns",
        "output_pub_mono_ns",
        "proc_exit_mono_ns",
        "last_event_mono_ns",
        "publish_latency_ms",
        "frame_complete_ms",
        "async_tail_ms",
        "input_interarrival_ms",
        "next_input_perception_seq",
        "input_sequence_delta",
        "input_sequence_gap_count",
        "output_sequence_delta",
        "dynamic_deadline_slack_ms",
        "fixed_deadline_miss",
        "dynamic_deadline_miss",
        "async_overlaps_next_frame",
        "publish_latency_robust_score",
        "publish_latency_robust_outlier",
        "frame_complete_robust_score",
        "frame_complete_robust_outlier",
        "async_tail_robust_score",
        "async_tail_robust_outlier",
        "proc_top_1pct",
        "frame_complete_top_1pct",
        "async_tail_top_1pct",
        "localized_node_spike",
        "input_interarrival_top_1pct",
        "abnormal_frame",
    ] + [f"{node}_ms" for node, _, _ in node_pairs]

    frame_path = data_dir / "prediction_frame_node_timings.csv"
    long_path = data_dir / "prediction_node_timings_long.csv"
    summary_path = data_dir / "prediction_node_summary.csv"
    span_summary_path = data_dir / "prediction_frame_span_summary.csv"
    anomaly_path = data_dir / "prediction_anomalous_frames.csv"
    cadence_path = data_dir / "prediction_input_cadence_gaps.csv"
    sqlite_path = data_dir / "prediction_analysis.sqlite"
    validation_path = data_dir / "validation_summary.json"
    manifest_path = output_dir / "output_manifest.csv"
    write_csv(frame_path, frame_rows, frame_fields)
    write_csv(long_path, node_rows)
    write_csv(summary_path, node_summary_rows)
    write_csv(span_summary_path, frame_span_summary_rows)
    write_csv(anomaly_path, anomaly_rows)
    write_csv(cadence_path, cadence_rows)
    write_sqlite_snapshot(sqlite_path, frame_rows, node_summary_rows, anomaly_rows, cadence_rows)

    input_seqs = [to_int(row["input_perception_seq"]) for row in frame_rows]
    output_seqs = [to_int(row["prediction_output_seq"]) for row in frame_rows]
    prediction_input_set = {value for value in input_seqs if value is not None}
    prediction_input_min = min(prediction_input_set)
    fusion_in_range = {value for value in fusion_outputs if value >= prediction_input_min}
    phase_per_trace = [len(trace) for trace in events.values()]
    validation = {
        "run_id": args.run_id,
        "status": "pass" if not any(
            [
                duplicate_event_ids,
                duplicate_context_edges,
                missing_phases,
                negative_spans,
                context_missing_in,
                context_missing_out,
            ]
        ) else "fail",
        "sources": {
            "events": str(events_path),
            "context": str(context_path),
            "prediction_raw": str(prediction_raw) if prediction_raw else None,
            "fusion_context": str(fusion_context) if fusion_context else None,
        },
        "grain": "one Prediction callback trace per frame; one enter/exit wall span per node",
        "counts": {
            "trace_frames": len(frame_rows),
            "event_rows": sum(phase_counts.values()),
            "context_rows": sum(len(edges) for edges in contexts.values()),
            "node_pairs": len(node_pairs),
            "node_timing_rows": len(node_rows),
            "anomalous_frames": len(anomaly_rows),
            "fixed_deadline_misses": sum(bool(row["fixed_deadline_miss"]) for row in frame_rows),
            "dynamic_deadline_misses": sum(bool(row["dynamic_deadline_miss"]) for row in frame_rows),
            "async_overlaps_next_frame": sum(bool(row["async_overlaps_next_frame"]) for row in frame_rows),
            "record_load_linked_frames": sum(row["prediction_obstacle_count"] is not None for row in frame_rows),
            "input_sequence_gap_intervals": sum(
                (row["input_sequence_gap_count"] or 0) > 0 for row in frame_rows
            ),
            "input_sequence_gap_count": sum(
                int(row["input_sequence_gap_count"] or 0) for row in frame_rows
            ),
            "input_interarrival_above_p99": sum(
                bool(row["input_interarrival_top_1pct"]) for row in frame_rows
            ),
        },
        "quality_checks": {
            "duplicate_event_ids": duplicate_event_ids,
            "duplicate_context_edges": duplicate_context_edges,
            "missing_or_extra_phase_frames": missing_phases,
            "negative_spans": negative_spans,
            "missing_context_in": context_missing_in,
            "missing_context_out": context_missing_out,
            "phases_per_trace_min": min(phase_per_trace),
            "phases_per_trace_max": max(phase_per_trace),
            "input_sequence_strictly_increasing": all(
                left is not None and right is not None and right > left
                for left, right in zip(input_seqs, input_seqs[1:])
            ),
            "output_sequence_strictly_increasing": all(
                left is not None and right is not None and right > left
                for left, right in zip(output_seqs, output_seqs[1:])
            ),
            "output_sequence_contiguous": all(
                left is not None and right is not None and right - left == 1
                for left, right in zip(output_seqs, output_seqs[1:])
            ),
            "fusion_outputs_in_prediction_range_not_consumed": sorted(
                fusion_in_range - prediction_input_set
            ),
            "prediction_inputs_without_fusion_output": (
                sorted(prediction_input_set - fusion_outputs) if fusion_outputs else None
            ),
        },
        "anomaly_definition": {
            "node_rule": (
                f"duration > median + {args.robust_z:g} * 1.4826 * MAD and "
                f"duration - median >= max({args.min_effect_ms:g} ms, 25% of median)"
            ),
            "fixed_deadline_ms": args.fixed_deadline_ms,
            "dynamic_deadline": "proc_exit must precede the next frame proc_enter",
            "frame_rule": (
                "deadline miss; proc or frame_complete above its P99; or a leaf node above P99 "
                f"with excess >= max({args.localized_min_excess_ms:g} ms, 50% of node median)"
            ),
            "robust_mad_role": (
                "diagnostic only because several node distributions are multimodal; MAD alone does not mark a frame"
            ),
        },
        "evidence_boundaries": [
            "Node durations are monotonic wall-clock spans between matching trace markers.",
            "Nested and parallel spans overlap, so node durations must not be summed into proc time.",
            "frame_complete_ms includes asynchronous worker events that can end after proc_exit.",
            "The final frame has no dynamic next-frame deadline observation.",
            "Prediction record linkage covers only the record window; missing load fields are not imputed.",
            "Input cadence gaps are reported separately and are not classified as Prediction internal node anomalies.",
        ],
    }
    validation_path.write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")

    output_files = [
        frame_path,
        long_path,
        summary_path,
        span_summary_path,
        anomaly_path,
        cadence_path,
        sqlite_path,
        validation_path,
    ]
    manifest_rows = [
        {
            "file": str(path.relative_to(output_dir)),
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path in output_files
    ]
    write_csv(manifest_path, manifest_rows)
    print(json.dumps(validation["counts"], ensure_ascii=False, indent=2))
    print(f"wrote {len(output_files)} data artifacts to {data_dir}")


if __name__ == "__main__":
    main()
