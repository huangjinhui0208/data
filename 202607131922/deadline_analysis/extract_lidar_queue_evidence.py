#!/usr/bin/env python3
"""Extract reproducible Lidar Detection queue and throughput evidence.

The source material contains explicit ``[LIDAR_DET_QUEUE]`` records, CenterPoint
stage timings, Cyber channel overflow warnings, and trace events.  This script
joins those sources without requiring the uncollected record file.
"""

from __future__ import annotations

import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "log" / "perception.log.INFO.20260713-192006.389396"
TRACE = ROOT / "trace"
OUT = Path(__file__).resolve().parent

BRIDGE_STEP_MS = 100.0
CRITICAL_MSG_TIME = 1783942139.947392
POST_DROP_FIRST_MSG_TIME = 1783942107.449978

QUEUE_RE = re.compile(
    r"^([IWE])0713 (\d\d:\d\d:\d\d\.\d+) (\d+).*"
    r"\[LIDAR_DET_QUEUE\] msg_time=([\d.]+) queue_wait_ms=([\d.]+) "
    r"inject_sleep_ms=([\d.]+) proc_ms=([\d.]+) total_ms=([\d.]+)"
)
BREAKDOWN_RE = re.compile(
    r"down sample: ([\d.]+)\s+fuse: ([\d.]+)\s+shuffle: ([\d.]+)\s+"
    r"cloud_to_array: ([\d.]+)\s+inference: ([\d.]+)\s+"
    r"postprocess: ([\d.]+)\s+nms: ([\d.]+)"
)
OVERFLOW_RE = re.compile(
    r"^W0713 (\d\d:\d\d:\d\d\.\d+) (\d+).*"
    r"channel\[/perception/lidar/pointcloud_ground_detection\] read buffer overflow, "
    r"drop_message\[(\d+)\] pre_index\[(\d+)\] current_index\[(\d+)\]"
)


def percentile(values: list[float], ratio: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * ratio
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_log() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    queues: list[dict[str, Any]] = []
    overflows: list[dict[str, Any]] = []
    latest_breakdown: tuple[float, ...] | None = None

    with LOG.open(encoding="utf-8", errors="ignore") as handle:
        for line_number, line in enumerate(handle, 1):
            breakdown = BREAKDOWN_RE.search(line)
            if breakdown:
                latest_breakdown = tuple(map(float, breakdown.groups()))
                continue

            queue = QUEUE_RE.search(line)
            if queue:
                stage = latest_breakdown or (float("nan"),) * 7
                queues.append(
                    {
                        "source_line": line_number,
                        "wall_time": queue.group(2),
                        "tid": int(queue.group(3)),
                        "msg_time": float(queue.group(4)),
                        "queue_wait_ms": float(queue.group(5)),
                        "inject_sleep_ms": float(queue.group(6)),
                        "proc_ms": float(queue.group(7)),
                        "total_ms": float(queue.group(8)),
                        "down_sample_ms": stage[0],
                        "fuse_ms": stage[1],
                        "shuffle_ms": stage[2],
                        "cloud_to_array_ms": stage[3],
                        "inference_ms": stage[4],
                        "postprocess_ms": stage[5],
                        "nms_ms": stage[6],
                        "proc_over_bridge_step_ms": float(queue.group(7))
                        - BRIDGE_STEP_MS,
                    }
                )
                latest_breakdown = None
                continue

            overflow = OVERFLOW_RE.search(line)
            if overflow:
                overflows.append(
                    {
                        "source_line": line_number,
                        "wall_time": overflow.group(1),
                        "tid": int(overflow.group(2)),
                        "drop_message_count": int(overflow.group(3)),
                        "pre_index": int(overflow.group(4)),
                        "current_index": int(overflow.group(5)),
                    }
                )

    return queues, overflows


def summarize(name: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    proc = [float(row["proc_ms"]) for row in rows]
    waits = [float(row["queue_wait_ms"]) for row in rows]
    inference = [
        float(row["inference_ms"])
        for row in rows
        if math.isfinite(float(row["inference_ms"]))
    ]
    arrivals = [
        (float(b["msg_time"]) - float(a["msg_time"])) * 1000
        for a, b in zip(rows, rows[1:])
    ]
    return {
        "window": name,
        "frame_count": len(rows),
        "msg_time_first": rows[0]["msg_time"],
        "msg_time_last": rows[-1]["msg_time"],
        "mean_input_interval_ms": statistics.mean(arrivals) if arrivals else "",
        "proc_min_ms": min(proc),
        "proc_p50_ms": percentile(proc, 0.50),
        "proc_p95_ms": percentile(proc, 0.95),
        "proc_mean_ms": statistics.mean(proc),
        "proc_max_ms": max(proc),
        "proc_gt_100ms_count": sum(value > BRIDGE_STEP_MS for value in proc),
        "inference_mean_ms": statistics.mean(inference),
        "inference_p95_ms": percentile(inference, 0.95),
        "estimated_service_rate_hz": 1000.0 / statistics.mean(proc),
        "queue_wait_first_ms": waits[0],
        "queue_wait_last_ms": waits[-1],
        "queue_wait_growth_ms": waits[-1] - waits[0],
        "queue_wait_max_ms": max(waits),
        "inject_sleep_nonzero_count": sum(
            float(row["inject_sleep_ms"]) > 0 for row in rows
        ),
    }


def trace_serialization_summary() -> list[dict[str, Any]]:
    context_path = (
        TRACE
        / "message_context"
        / "perception.lidar_detection.389396.csv"
    )
    events_path = TRACE / "events" / "perception.lidar_detection.389396.csv"

    data_time: dict[int, float] = {}
    with context_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["edge"] == "in":
                data_time[int(row["trace_id"])] = int(row["data_ts_ns"]) / 1e9

    events: dict[int, dict[str, tuple[int, int]]] = defaultdict(dict)
    with events_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            events[int(row["trace_id"])][row["phase"]] = (
                int(row["mono_ns"]),
                int(row["tid"]),
            )

    calls: list[tuple[float, int, int, int]] = []
    for trace_id, phases in events.items():
        if (
            trace_id in data_time
            and "proc_enter" in phases
            and "output_pub" in phases
            and POST_DROP_FIRST_MSG_TIME
            <= data_time[trace_id]
            <= CRITICAL_MSG_TIME + 1e-6
        ):
            calls.append(
                (
                    data_time[trace_id],
                    phases["proc_enter"][0],
                    phases["output_pub"][0],
                    phases["proc_enter"][1],
                )
            )
    calls.sort(key=lambda item: item[1])

    gaps = [(b[1] - a[2]) / 1e6 for a, b in zip(calls, calls[1:])]
    start_intervals = [
        (b[1] - a[1]) / 1e6 for a, b in zip(calls, calls[1:])
    ]
    return [
        {
            "window": "post_overflow_reset_to_critical_frame",
            "complete_call_count": len(calls),
            "worker_tid_count": len({call[3] for call in calls}),
            "overlapping_adjacent_call_count": sum(
                b[1] < a[2] for a, b in zip(calls, calls[1:])
            ),
            "output_to_next_enter_gap_min_ms": min(gaps),
            "output_to_next_enter_gap_p50_ms": percentile(gaps, 0.50),
            "output_to_next_enter_gap_mean_ms": statistics.mean(gaps),
            "output_to_next_enter_gap_max_ms": max(gaps),
            "start_interval_p50_ms": percentile(start_intervals, 0.50),
            "start_interval_mean_ms": statistics.mean(start_intervals),
        }
    ]


def main() -> None:
    queues, overflows = load_log()
    critical_index = min(
        range(len(queues)),
        key=lambda index: abs(queues[index]["msg_time"] - CRITICAL_MSG_TIME),
    )
    post_drop = [
        row
        for row in queues
        if POST_DROP_FIRST_MSG_TIME
        <= row["msg_time"]
        <= CRITICAL_MSG_TIME + 1e-6
    ]
    last_50 = queues[critical_index - 50 : critical_index + 1]
    critical = [queues[critical_index]]

    summaries = [
        summarize("all_logged_queue_events", queues),
        summarize("post_overflow_reset_to_critical_frame", post_drop),
        summarize("critical_and_previous_50_frames", last_50),
        summarize("critical_frame", critical),
    ]

    write_csv(OUT / "lidar_detection_queue_events.csv", queues)
    write_csv(OUT / "lidar_detection_queue_summary.csv", summaries)
    write_csv(OUT / "lidar_detection_overflow_events.csv", overflows)
    write_csv(
        OUT / "lidar_detection_execution_summary.csv",
        trace_serialization_summary(),
    )

    print(f"queue_events={len(queues)}")
    print(f"overflow_events={len(overflows)}")
    print(f"critical_source_line={critical[0]['source_line']}")
    print(f"outputs={OUT}")


if __name__ == "__main__":
    main()
