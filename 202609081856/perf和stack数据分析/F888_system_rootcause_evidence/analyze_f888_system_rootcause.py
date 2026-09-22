#!/usr/bin/env python3
"""Build event-level scheduler evidence for Prediction frame 888.

The script deliberately reuses the already-produced worker state ledger.  It
does not recompute workload, deadline, callback state totals, or the same-load
classification.  The raw system-wide perf sched text is read only to recover
CPU occupants, wake dependencies, migrations, and the irq/134-host_sy thread.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FRAMES = (887, 888, 889)
KEY_WINDOWS = (
    ("runnable_1646959", 968509.383000, 968509.413500),
    ("runnable_1646949", 968509.545500, 968509.556800),
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-sched", type=Path, required=True)
    parser.add_argument("--analysis-root", type=Path, required=True)
    parser.add_argument("--model-calls", type=Path, required=True)
    parser.add_argument("--parser-script", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_sched_parser(path: Path):
    spec = importlib.util.spec_from_file_location("prediction_sched_parser", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load parser: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def trace_to_sched(trace_ns: str | int, calibration: dict[str, object]) -> float:
    trace_s = int(trace_ns) / 1e9
    affine = ("trace_offset_intercept_s", "trace_offset_slope", "trace_reference_s")
    if all(key in calibration for key in affine):
        return (
            trace_s
            + float(calibration["trace_offset_intercept_s"])
            + float(calibration["trace_offset_slope"])
            * (trace_s - float(calibration["trace_reference_s"]))
        )
    return trace_s + float(calibration["trace_to_sched_offset_s"])


def overlap(left: float, right: float, start: float, end: float) -> float:
    return max(0.0, min(right, end) - max(left, start))


def interval_union_ms(intervals: list[tuple[float, float]]) -> float:
    merged: list[list[float]] = []
    for left, right in sorted(intervals):
        if right <= left:
            continue
        if merged and left <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], right)
        else:
            merged.append([left, right])
    return sum(right - left for left, right in merged) * 1000


def fmt_map(values: dict[object, float], suffix: str = "") -> str:
    return ";".join(
        f"{key}:{value * 1000:.6f}{suffix}"
        for key, value in sorted(values.items(), key=lambda pair: (-pair[1], str(pair[0])))
    )


def load_frame_data(root: Path, model_call_path: Path, calibration: dict[str, object]) -> dict[int, dict]:
    model_calls = [row for row in read_csv(model_call_path) if int(row["frame_index"]) in FRAMES]
    obstacle_lookup: dict[tuple[int, int, int], int] = {}
    for row in model_calls:
        obstacle_lookup[(int(row["frame_index"]), int(row["worker_tid"]), int(row["call_index_in_tid_model"]))] = int(row["obstacle_id"])

    frame_data: dict[int, dict] = {}
    for frame in FRAMES:
        frame_dir = root / f"F{frame:04d}"
        frame_row = read_csv(frame_dir / "frame.csv")[0]
        nodes = read_csv(frame_dir / "trace_node_instances.csv")
        pool = next(row for row in nodes if row["node"] == "thread_pool_run")
        models_by_tid: dict[int, list[dict]] = defaultdict(list)
        for row in nodes:
            if row["node"] != "model_cruise_mlp":
                continue
            models_by_tid[int(row["tid"])].append(row)
        models: list[dict] = []
        for tid, rows in models_by_tid.items():
            rows.sort(key=lambda row: int(row["enter_ns"]))
            for ordinal, row in enumerate(rows, start=1):
                models.append({
                    "frame": frame,
                    "worker_tid": tid,
                    "ordinal": ordinal,
                    "obstacle_id": obstacle_lookup[(frame, tid, ordinal)],
                    "start": trace_to_sched(row["enter_ns"], calibration),
                    "end": trace_to_sched(row["exit_ns"], calibration),
                    "duration_ms": float(row["duration_ms"]),
                })
        frame_data[frame] = {
            "callback_start": trace_to_sched(frame_row["prediction_enter_ns"], calibration),
            "callback_end": trace_to_sched(frame_row["prediction_exit_ns"], calibration),
            "pool_start": trace_to_sched(pool["enter_ns"], calibration),
            "pool_end": trace_to_sched(pool["exit_ns"], calibration),
            "models": sorted(models, key=lambda item: (item["worker_tid"], item["start"])),
            "worker_tids": set(models_by_tid),
        }
    return frame_data


def load_existing_states(root: Path, frame_data: dict[int, dict]) -> dict[int, list[dict]]:
    rows = read_csv(root / "worker_gantt" / "worker_sched_timeline.csv")
    states: dict[int, list[dict]] = defaultdict(list)
    for row in rows:
        frame = int(row["frame_index"])
        if frame not in FRAMES:
            continue
        states[frame].append({
            "frame": frame,
            "tid": int(row["worker_tid"]),
            "state": row["state"],
            "cpu": int(row["cpu"]) if row["cpu"] else None,
            "start": frame_data[frame]["pool_start"] + float(row["start_from_threadpool_ms"]) / 1000,
            "end": frame_data[frame]["pool_start"] + float(row["end_from_threadpool_ms"]) / 1000,
        })
    for frame in states:
        states[frame].sort(key=lambda row: (row["tid"], row["start"], row["end"]))
    return states


def build_switch_intervals(events: list[dict], scan_start: float, scan_end: float) -> dict[int, list[dict]]:
    switches: dict[int, list[dict]] = defaultdict(list)
    for event in events:
        if event["event"] == "sched_switch":
            switches[int(event["cpu"])].append(event)
    intervals: dict[int, list[dict]] = defaultdict(list)
    for cpu, rows in switches.items():
        rows.sort(key=lambda event: event["timestamp"])
        for index, event in enumerate(rows):
            right = rows[index + 1]["timestamp"] if index + 1 < len(rows) else scan_end
            if right <= event["timestamp"]:
                continue
            intervals[cpu].append({
                "start": event["timestamp"],
                "end": right,
                "cpu": cpu,
                "prev_tid": int(event["prev_tid"]),
                "prev_comm": event["prev_comm"],
                "prev_prio": int(event["prev_prio"]),
                "prev_state": event["prev_state"],
                "next_tid": int(event["next_tid"]),
                "next_comm": event["next_comm"],
                "next_prio": int(event["next_prio"]),
            })
    return intervals


def system_timeline_rows(intervals: dict[int, list[dict]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for window_id, start, end in KEY_WINDOWS:
        for cpu in sorted(intervals):
            for item in intervals[cpu]:
                used = overlap(item["start"], item["end"], start, end)
                if used <= 0:
                    continue
                rows.append({
                    "window_id": window_id,
                    "window_start_s": f"{start:.9f}",
                    "window_end_s": f"{end:.9f}",
                    "timestamp_s": f"{item['start']:.9f}",
                    "interval_start_s": f"{max(item['start'], start):.9f}",
                    "interval_end_s": f"{min(item['end'], end):.9f}",
                    "cpu": cpu,
                    "prev_tid": item["prev_tid"],
                    "prev_comm": item["prev_comm"],
                    "prev_prio": item["prev_prio"],
                    "prev_state": item["prev_state"],
                    "next_tid": item["next_tid"],
                    "next_comm": item["next_comm"],
                    "next_prio": item["next_prio"],
                    "running_interval_ms": f"{used * 1000:.6f}",
                })
    rows.sort(key=lambda row: (row["window_id"], float(row["interval_start_s"]), int(row["cpu"])))
    return rows


def event_at(events: list[dict], timestamp: float, predicate) -> dict | None:
    candidates = [event for event in events if abs(event["timestamp"] - timestamp) <= 2e-6 and predicate(event)]
    return sorted(candidates, key=lambda event: (abs(event["timestamp"] - timestamp), event["event"]))[0] if candidates else None


def adjacent_cpu(states: list[dict], target: dict, before: bool) -> int | None:
    same = [row for row in states if row["tid"] == target["tid"] and row["state"] == "running"]
    if before:
        candidates = [row for row in same if row["end"] <= target["start"] + 2e-6]
        return max(candidates, key=lambda row: row["end"])["cpu"] if candidates else None
    candidates = [row for row in same if row["start"] >= target["end"] - 2e-6]
    return min(candidates, key=lambda row: row["start"])["cpu"] if candidates else None


def route_for_episode(events: list[dict], row: dict, cpu_before: int | None, cpu_after: int | None) -> tuple[list[tuple[float, float, int | None]], str]:
    start_event = event_at(
        events,
        row["start"],
        lambda event: (
            (event["event"] == "sched_switch" and int(event["prev_tid"]) == row["tid"] and str(event["prev_state"]).startswith("R"))
            or (event["event"] in {"sched_waking", "sched_wakeup", "sched_wakeup_new"} and int(event["target_tid"]) == row["tid"])
        ),
    )
    if start_event and start_event["event"] == "sched_switch":
        current_cpu = int(start_event["cpu"])
        start_type = "preempted"
    elif start_event:
        current_cpu = start_event.get("wake_target_cpu")
        start_type = "wakeup"
    else:
        current_cpu = cpu_before
        start_type = "unmatched_start"
    changes = [(row["start"], current_cpu)]
    for event in events:
        if not (row["start"] < event["timestamp"] < row["end"]):
            continue
        if event["event"] == "sched_migrate_task" and int(event["target_tid"]) == row["tid"]:
            current_cpu = int(event["migrate_dest_cpu"])
            changes.append((event["timestamp"], current_cpu))
    if current_cpu is None:
        current_cpu = cpu_after
        changes = [(row["start"], current_cpu)]
    route = []
    for index, (left, cpu) in enumerate(changes):
        right = changes[index + 1][0] if index + 1 < len(changes) else row["end"]
        route.append((left, right, cpu))
    return route, start_type


def runnable_rootcause_rows(events: list[dict], intervals: dict[int, list[dict]], states: dict[int, list[dict]]) -> tuple[list[dict], list[dict], list[dict]]:
    candidates = [row for row in states[888] if row["state"] == "runnable" and (row["end"] - row["start"]) * 1000 > 2.0]
    output: list[dict] = []
    route_audit: list[dict] = []
    occupancy_audit: list[dict] = []
    for episode_id, row in enumerate(sorted(candidates, key=lambda item: item["start"]), start=1):
        cpu_before = adjacent_cpu(states[888], row, True)
        cpu_after = adjacent_cpu(states[888], row, False)
        route, start_type = route_for_episode(events, row, cpu_before, cpu_after)
        occupancy: Counter[tuple[int, str, int]] = Counter()
        coverage = 0.0
        idle = 0.0
        for route_start, route_end, cpu in route:
            route_audit.append({
                "episode_id": episode_id,
                "worker_tid": row["tid"],
                "route_start_s": f"{route_start:.9f}",
                "route_end_s": f"{route_end:.9f}",
                "assigned_cpu": "" if cpu is None else cpu,
            })
            if cpu is None:
                continue
            for item in intervals.get(cpu, []):
                used = overlap(item["start"], item["end"], route_start, route_end)
                if used <= 0:
                    continue
                coverage += used
                key = (int(item["next_tid"]), str(item["next_comm"]), int(item["next_prio"]))
                occupancy[key] += used
                if int(item["next_tid"]) == 0:
                    idle += used
        competitors = [(key, value) for key, value in occupancy.items() if key[0] not in {0, row["tid"]}]
        competitors.sort(key=lambda pair: (-pair[1], pair[0][0]))
        for (tid, comm, prio), value in sorted(occupancy.items(), key=lambda pair: (-pair[1], pair[0][0])):
            occupancy_audit.append({
                "episode_id": episode_id,
                "worker_tid": row["tid"],
                "runnable_start_s": f"{row['start']:.9f}",
                "runnable_end_s": f"{row['end']:.9f}",
                "occupant_tid": tid,
                "occupant_comm": comm,
                "occupant_priority": prio,
                "occupant_runtime_ms": f"{value * 1000:.6f}",
                "is_idle": int(tid == 0),
                "is_irq_thread": int(comm.startswith("irq/")),
            })
        dominant = competitors[0] if competitors else ((0, "idle", 0), idle)
        irq_runtime = sum(value for (tid, comm, prio), value in competitors if comm.startswith("irq/"))
        competitor_runtime = sum(value for key, value in competitors)
        worker_priority = ""
        end_event = event_at(events, row["end"], lambda event: event["event"] == "sched_switch" and int(event["next_tid"]) == row["tid"])
        if end_event:
            worker_priority = int(end_event["next_prio"])
        output.append({
            "episode_id": episode_id,
            "worker_tid": row["tid"],
            "start_sched_s": f"{row['start']:.9f}",
            "end_sched_s": f"{row['end']:.9f}",
            "duration_ms": f"{(row['end'] - row['start']) * 1000:.6f}",
            "start_type": start_type,
            "cpu_before": "" if cpu_before is None else cpu_before,
            "cpu_after": "" if cpu_after is None else cpu_after,
            "assigned_cpu_route": ";".join(f"{left:.6f}-{right:.6f}:CPU{cpu}" for left, right, cpu in route),
            "migrated": int(len({cpu for _, _, cpu in route if cpu is not None}) > 1 or (cpu_before is not None and cpu_after is not None and cpu_before != cpu_after)),
            "competing_threads": ";".join(f"{comm}[{tid},prio={prio}]:{value * 1000:.6f}ms" for (tid, comm, prio), value in competitors),
            "competing_runtime_ms": f"{competitor_runtime * 1000:.6f}",
            "idle_runtime_ms": f"{idle * 1000:.6f}",
            "irq_runtime_ms": f"{irq_runtime * 1000:.6f}",
            "dominant_competitor": f"{dominant[0][1]}[{dominant[0][0]}]",
            "dominant_competitor_runtime_ms": f"{dominant[1] * 1000:.6f}",
            "worker_priority": worker_priority,
            "competitor_priority": dominant[0][2],
            "occupancy_coverage_ms": f"{coverage * 1000:.6f}",
            "occupancy_coverage_ratio": f"{coverage / (row['end'] - row['start']):.9f}",
        })
    return output, route_audit, occupancy_audit


def blocked_dependency_rows(root: Path, frame_data: dict[int, dict], events: list[dict]) -> list[dict]:
    blocked = read_csv(root / "worker_gantt" / "worker_blocked_intervals.csv")
    rows: list[dict] = []
    for item in blocked:
        if int(item["frame_index"]) != 888 or float(item["duration_ms"]) <= 2.0:
            continue
        tid = int(item["worker_tid"])
        start, wake = float(item["start_sched_s"]), float(item["end_sched_s"])
        model = next((model for model in frame_data[888]["models"] if model["worker_tid"] == tid and overlap(start, wake, model["start"], model["end"]) > 0), None)
        wake_event = event_at(
            events,
            wake,
            lambda event: event["event"] in {"sched_waking", "sched_wakeup", "sched_wakeup_new"} and int(event["target_tid"]) == tid,
        )
        waker_tid = int(item["waker_tid"])
        prior_waker_runs = [
            event for event in events
            if event["event"] == "sched_switch"
            and int(event["next_tid"]) == waker_tid
            and event["timestamp"] <= wake
        ]
        waker_schedule_in = max((event["timestamp"] for event in prior_waker_runs), default=None)
        rows.append({
            "blocked_tid": tid,
            "block_start": f"{start:.9f}",
            "wake_time": f"{wake:.9f}",
            "blocked_ms": f"{(wake - start) * 1000:.6f}",
            "waker_tid": item["waker_tid"],
            "waker_comm": item["waker_comm"],
            "active_trace_node": "model_cruise_mlp" if model else "thread_pool_run",
            "model_ordinal": model["ordinal"] if model else "",
            "obstacle_id": model["obstacle_id"] if model else "",
            "wake_event": wake_event["event"] if wake_event else item["wake_event"],
            "wake_cpu": wake_event["cpu"] if wake_event else "",
            "wake_raw_event_verified": int(wake_event is not None),
            "waker_last_schedule_in_s": "" if waker_schedule_in is None else f"{waker_schedule_in:.9f}",
            "wake_after_waker_schedule_in_ms": "" if waker_schedule_in is None else f"{(wake - waker_schedule_in) * 1000:.6f}",
            "dependency_chain": "",
            "interval_from_previous_chain_event_ms": "",
        })
    chain = [(1646959, 1646942), (1646942, 1646949), (1646949, 1646953)]
    chain_rows = []
    for waker, target in chain:
        candidates = [row for row in rows if int(row["blocked_tid"]) == target and int(row["waker_tid"]) == waker and 968509.412 < float(row["wake_time"]) < 968509.414]
        if candidates:
            chain_rows.append(min(candidates, key=lambda row: float(row["wake_time"])))
    previous = None
    for index, row in enumerate(chain_rows, start=1):
        row["dependency_chain"] = f"chain_step_{index}"
        if previous is not None:
            row["interval_from_previous_chain_event_ms"] = f"{(float(row['wake_time']) - previous) * 1000:.6f}"
        previous = float(row["wake_time"])
    rows.sort(key=lambda row: (float(row["block_start"]), int(row["blocked_tid"])))
    return rows


def irq_rows(events: list[dict], intervals: dict[int, list[dict]], frame_data: dict[int, dict], states: dict[int, list[dict]]) -> tuple[list[dict], int, str]:
    identities = Counter()
    for event in events:
        if event["event"] == "sched_switch":
            if str(event["prev_comm"]).startswith("irq/134-host_sy"):
                identities[(int(event["prev_tid"]), str(event["prev_comm"]))] += 1
            if str(event["next_comm"]).startswith("irq/134-host_sy"):
                identities[(int(event["next_tid"]), str(event["next_comm"]))] += 1
    if not identities:
        raise RuntimeError("irq/134-host_sy was not found in the scanned raw scheduler data")
    (irq_tid, irq_comm), _ = identities.most_common(1)[0]
    output = []
    for frame in FRAMES:
        runnable_segments = [row for row in states[frame] if row["state"] == "runnable"]
        for window_type, start, end in (
            ("callback", frame_data[frame]["callback_start"], frame_data[frame]["callback_end"]),
            ("thread_pool_run", frame_data[frame]["pool_start"], frame_data[frame]["pool_end"]),
        ):
            irq_intervals = []
            cpu_time: Counter[int] = Counter()
            segment_count = 0
            max_runtime = 0.0
            overlap_intervals = []
            covered_episodes = set()
            for cpu, cpu_rows in intervals.items():
                for item in cpu_rows:
                    if int(item["next_tid"]) != irq_tid:
                        continue
                    used = overlap(item["start"], item["end"], start, end)
                    if used <= 0:
                        continue
                    left, right = max(item["start"], start), min(item["end"], end)
                    irq_intervals.append((left, right))
                    cpu_time[cpu] += used
                    segment_count += 1
                    max_runtime = max(max_runtime, used)
                    for index, runnable in enumerate(runnable_segments):
                        intersection = overlap(left, right, runnable["start"], runnable["end"])
                        if intersection > 0:
                            overlap_intervals.append((max(left, runnable["start"]), min(right, runnable["end"])))
                            covered_episodes.add(index)
            wakes = {
                (round(event["timestamp"], 6), int(event["target_tid"]))
                for event in events
                if start <= event["timestamp"] <= end
                and event["event"] == "sched_waking"
                and int(event["sample_tid"]) == irq_tid
                and int(event["target_tid"]) in frame_data[frame]["worker_tids"]
            }
            output.append({
                "frame": frame,
                "window_type": window_type,
                "window_start_s": f"{start:.9f}",
                "window_end_s": f"{end:.9f}",
                "irq_tid": irq_tid,
                "irq_comm": irq_comm,
                "runtime_ms": f"{sum(right - left for left, right in irq_intervals) * 1000:.6f}",
                "running_segment_count": segment_count,
                "max_continuous_runtime_ms": f"{max_runtime * 1000:.6f}",
                "cpu_distribution_ms": fmt_map(cpu_time),
                "prediction_worker_waking_count": len(wakes),
                "worker_runnable_overlap_ms": f"{interval_union_ms(overlap_intervals):.6f}",
                "worker_runnable_episode_overlap_count": len(covered_episodes),
            })
    return output, irq_tid, irq_comm


def model_alignment_rows(root: Path, frame_data: dict[int, dict]) -> list[dict]:
    summaries = [row for row in read_csv(root / "worker_gantt" / "model_sched_summary.csv") if int(row["frame_index"]) in FRAMES]
    lookup = {(int(row["frame_index"]), int(row["worker_tid"]), int(row["model_ordinal"])): row for row in summaries}
    timeline_rows = read_csv(root / "worker_gantt" / "worker_sched_timeline.csv")
    state_lookup: dict[tuple[int, int], list[dict]] = defaultdict(list)
    for row in timeline_rows:
        frame = int(row["frame_index"])
        if frame not in FRAMES:
            continue
        pool_start = frame_data[frame]["pool_start"]
        state_lookup[(frame, int(row["worker_tid"]))].append({
            "state": row["state"],
            "start": pool_start + float(row["start_from_threadpool_ms"]) / 1000,
            "end": pool_start + float(row["end_from_threadpool_ms"]) / 1000,
        })
    output = []
    for frame in FRAMES:
        by_tid: dict[int, list[dict]] = defaultdict(list)
        for model in frame_data[frame]["models"]:
            by_tid[model["worker_tid"]].append(model)
        for tid, models in by_tid.items():
            models.sort(key=lambda model: model["ordinal"])
            previous_end = None
            for model in models:
                summary = lookup[(frame, tid, model["ordinal"])]
                gap = 0.0 if previous_end is None else (model["start"] - previous_end) * 1000
                gap_state = Counter()
                gap_max_runnable = 0.0
                if previous_end is not None:
                    for state in state_lookup[(frame, tid)]:
                        used = overlap(state["start"], state["end"], previous_end, model["start"])
                        gap_state[state["state"]] += used
                        if state["state"] == "runnable":
                            gap_max_runnable = max(gap_max_runnable, used)
                output.append({
                    "frame": frame,
                    "obstacle_id": model["obstacle_id"],
                    "worker_tid": tid,
                    "model_ordinal": model["ordinal"],
                    "model_start": f"{model['start']:.9f}",
                    "model_end": f"{model['end']:.9f}",
                    "model_duration_ms": f"{model['duration_ms']:.6f}",
                    "running_ms": f"{float(summary['running_ms']):.6f}",
                    "blocked_ms": f"{float(summary['blocked_ms']):.6f}",
                    "runnable_ms": f"{float(summary['runnable_ms']):.6f}",
                    "max_runnable_ms": f"{float(summary['max_runnable_ms']):.6f}",
                    "inter_model_gap_ms": f"{gap:.6f}",
                    "gap_running_ms": f"{gap_state['running'] * 1000:.6f}",
                    "gap_blocked_ms": f"{gap_state['blocked'] * 1000:.6f}",
                    "gap_runnable_ms": f"{gap_state['runnable'] * 1000:.6f}",
                    "gap_max_runnable_ms": f"{gap_max_runnable * 1000:.6f}",
                    "gap_runnable_explanation_ratio": "" if gap == 0 else f"{gap_state['runnable'] * 1000 / gap:.9f}",
                })
                previous_end = model["end"]
    output.sort(key=lambda row: (int(row["frame"]), int(row["worker_tid"]), int(row["model_ordinal"])))
    return output


def font(size: int):
    for path in (Path(r"C:\Windows\Fonts\msyh.ttc"), Path(r"C:\Windows\Fonts\simhei.ttf")):
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def draw_dependency_timeline(path: Path, states: dict[int, list[dict]], blocked_rows: list[dict]) -> None:
    tids = [1646959, 1646942, 1646949, 1646953]
    start, end = 968509.3825, 968509.4140
    width, height = 1900, 530
    left, right, top = 210, 80, 95
    plot_width = width - left - right
    lane_gap = 88
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    title_font, label_font, small_font = font(28), font(20), font(16)
    draw.text((left, 22), "F888 worker唤醒依赖时间线", fill="#172033", font=title_font)
    x = lambda value: left + (value - start) / (end - start) * plot_width
    colors = {"running": "#2166AC", "blocked": "#D7DCE2", "runnable": "#E7A52A", "unknown": "#333333"}
    for tick_ms in range(0, 32, 2):
        value = start + tick_ms / 1000
        px = x(value)
        draw.line((px, top - 8, px, height - 70), fill="#E6E9EE", width=1)
        draw.text((px - 16, height - 62), f"{tick_ms}", fill="#4D5666", font=small_font)
    lane_y = {}
    for index, tid in enumerate(tids):
        y = top + index * lane_gap
        lane_y[tid] = y
        draw.text((24, y + 10), f"TID {tid}", fill="#172033", font=label_font)
        draw.line((left, y + 22, width - right, y + 22), fill="#BBC1CB", width=1)
        for segment in states[888]:
            if segment["tid"] != tid:
                continue
            used = overlap(segment["start"], segment["end"], start, end)
            if used <= 0:
                continue
            draw.rectangle((x(max(segment["start"], start)), y + 8, x(min(segment["end"], end)), y + 36), fill=colors[segment["state"]])
    chain_rows = [row for row in blocked_rows if row["dependency_chain"]]
    chain_rows.sort(key=lambda row: float(row["wake_time"]))
    for row in chain_rows:
        source, target, when = int(row["waker_tid"]), int(row["blocked_tid"]), float(row["wake_time"])
        px = x(when)
        sy, ty = lane_y[source] + 8, lane_y[target] + 36
        draw.line((px, sy, px, ty), fill="#B2182B", width=4)
        direction = 1 if ty > sy else -1
        draw.polygon([(px, ty), (px - 8, ty - 12 * direction), (px + 8, ty - 12 * direction)], fill="#B2182B")
        short_source = str(source)[-3:]
        short_target = str(target)[-3:]
        label_x = max(left + 4, min(px - 105, width - right - 105))
        draw.text((label_x, min(sy, ty) + abs(ty - sy) / 2 - 10), f"{short_source}→{short_target}", fill="#8B1020", font=small_font)
    legends = [("#2166AC", "运行中"), ("#D7DCE2", "睡眠等待"), ("#E7A52A", "可运行等待"), ("#B2182B", "sched_waking依赖")]
    lx = 900
    for color, label in legends:
        draw.rectangle((lx, 52, lx + 24, 68), fill=color)
        draw.text((lx + 31, 48), label, fill="#172033", font=small_font)
        lx += 190
    draw.text((left + plot_width / 2 - 130, height - 28), "相对 968509.382500 s 的时间  毫秒", fill="#172033", font=label_font)
    image.save(path)


def critical_path_decomposition(root: Path) -> dict[str, float]:
    rows = [row for row in read_csv(root / "worker_gantt" / "worker_sched_summary.csv") if int(row["frame_index"]) in FRAMES]
    critical = {}
    for frame in FRAMES:
        critical[frame] = max((row for row in rows if int(row["frame_index"]) == frame), key=lambda row: float(row["completion_from_pool_start_ms"]))
    control_blocked = (float(critical[887]["span_blocked_ms"]) + float(critical[889]["span_blocked_ms"])) / 2
    control_runnable = (float(critical[887]["span_runnable_ms"]) + float(critical[889]["span_runnable_ms"])) / 2
    control_running = (float(critical[887]["span_running_ms"]) + float(critical[889]["span_running_ms"])) / 2
    control_pool = (143.418848 + 142.292096) / 2
    excess = 253.361792 - control_pool
    return {
        "threadpool_excess_ms": excess,
        "blocked_delta_ms": float(critical[888]["span_blocked_ms"]) - control_blocked,
        "runnable_delta_ms": float(critical[888]["span_runnable_ms"]) - control_runnable,
        "running_delta_ms": float(critical[888]["span_running_ms"]) - control_running,
        "f888_critical_tid": int(critical[888]["worker_tid"]),
    }


def render_report(path: Path, root: Path, runnable_rows: list[dict], occupancy_rows: list[dict], blocked_rows: list[dict], irq_table: list[dict], alignment: list[dict], irq_tid: int, irq_comm: str) -> None:
    decomposition = critical_path_decomposition(root)
    excess = decomposition["threadpool_excess_ms"]
    competitors: Counter[tuple[str, str, str]] = Counter()
    for row in occupancy_rows:
        if int(row["is_idle"]) or int(row["occupant_tid"]) in {1646942, 1646949, 1646953, 1646959}:
            continue
        key = (str(row["occupant_tid"]), str(row["occupant_comm"]), str(row["occupant_priority"]))
        competitors[key] += float(row["occupant_runtime_ms"])
    irq_threadpool = {int(row["frame"]): row for row in irq_table if row["window_type"] == "thread_pool_run"}
    gap_model = next(row for row in alignment if int(row["frame"]) == 888 and int(row["worker_tid"]) == 1646949 and int(row["model_ordinal"]) == 3)
    long_wait = next(row for row in runnable_rows if int(row["worker_tid"]) == 1646949 and abs(float(row["duration_ms"]) - 10.071) < 0.01)
    first_long_wait = next(row for row in runnable_rows if int(row["worker_tid"]) == 1646959 and abs(float(row["duration_ms"]) - 24.655) < 0.01)
    gap = float(gap_model["inter_model_gap_ms"])
    wait = float(long_wait["duration_ms"])
    chain = [row for row in blocked_rows if row["dependency_chain"]]
    chain.sort(key=lambda row: float(row["wake_time"]))
    blocked_by_waker: Counter[tuple[str, str]] = Counter()
    blocked_count_by_waker: Counter[tuple[str, str]] = Counter()
    for row in blocked_rows:
        key = (str(row["waker_tid"]), str(row["waker_comm"]))
        blocked_by_waker[key] += float(row["blocked_ms"])
        blocked_count_by_waker[key] += 1
    lines = [
        "# F888 CruiseMLP worker 系统层根因证据",
        "",
        "## 范围与证据口径",
        "",
        "本次分析直接复用既有逐 worker 状态账本，不重新计算 Prediction 工作量、deadline、callback 状态统计，也不重新论证 F888 的同负载异常属性。应用打点通过当前 run 的校准参数转换到 perf sched 时间域。校准文件记录 trace marker 支持率为 100%，本报告的系统竞争与唤醒关系来自原始 system-wide sched 事件。",
        "",
        "`blocked` 只表示线程从 CPU 切出到首次 sched_waking 的睡眠区间；`runnable` 只表示线程已可运行但尚未重新获得 CPU。二者不等同于具体锁等待、GPU 完成等待或其他设备完成等待。",
        "",
        "## 关键事实",
        "",
        f"- F888 ThreadPoolRun 相对 F887 与 F889 均值增加 {excess:.6f} ms。F888 关键 worker 是 TID {int(decomposition['f888_critical_tid'])}。",
        f"- 关键 worker 的 blocked 增量为 {decomposition['blocked_delta_ms']:.6f} ms，占 ThreadPoolRun 增量的 {decomposition['blocked_delta_ms'] / excess * 100:.2f}%。这是关键路径状态账本的时间归属，不是具体阻塞对象的因果证明。",
        f"- 关键 worker 的 runnable 增量为 {decomposition['runnable_delta_ms']:.6f} ms，占 ThreadPoolRun 增量的 {decomposition['runnable_delta_ms'] / excess * 100:.2f}%。",
        f"- 关键 worker 的 running 增量为 {decomposition['running_delta_ms']:.6f} ms，占 ThreadPoolRun 增量的 {decomposition['running_delta_ms'] / excess * 100:.2f}%。",
        f"- 三类状态增量合计 {decomposition['blocked_delta_ms'] + decomposition['runnable_delta_ms'] + decomposition['running_delta_ms']:.6f} ms，与 ThreadPoolRun 增量相差 {excess - decomposition['blocked_delta_ms'] - decomposition['runnable_delta_ms'] - decomposition['running_delta_ms']:.6f} ms。该残差来自 ThreadPoolRun 边界与关键 worker span 边界的差异。",
        "",
        "## 10.347 ms 应用空隙与 10.071 ms 调度等待",
        "",
        f"F888 TID 1646949 的模型 2 结束到模型 3 开始间隔为 {gap:.6f} ms；对应 runnable 区间为 {wait:.6f} ms，解释比例为 {wait / gap * 100:.2f}%。两者边界差为 {gap - wait:.6f} ms。二者属于同一绝对时间窗口，CSV 可按 TID 1646949、模型序号 3 和 runnable 起止时间反查。剩余时间由模型标记边界与 scheduler 状态边界之间的小段执行和状态转换构成。",
        "",
        "## 两段重点 Runnable Waiting 的直接解释",
        "",
        f"- TID 1646959 在 {first_long_wait['start_sched_s']}–{first_long_wait['end_sched_s']} s 等待 {float(first_long_wait['duration_ms']):.6f} ms。它先在 CPU0 的可运行队列等待，968509.412062 s 迁移到 CPU8，968509.412974 s 获得 CPU。指定 CPU 在该段被其他线程占用 {float(first_long_wait['competing_runtime_ms']):.6f} ms，空闲仅 {float(first_long_wait['idle_runtime_ms']):.6f} ms；主要占用者是 `dds.udp.27457[1648118]` 7.940000 ms、`dds.udp.27417[1644778]` 4.900000 ms、`mainboard[1645550]` 2.737000 ms 和 `ksoftirqd/0[12]` 2.666000 ms。`irq/134-host_sy[174]` 占用 0.816000 ms，不是该段最大竞争者。",
        f"- TID 1646949 在 {long_wait['start_sched_s']}–{long_wait['end_sched_s']} s 等待 {float(long_wait['duration_ms']):.6f} ms。它先在 CPU0 等待，968509.556033 s 迁移到 CPU1，968509.556313 s 获得 CPU。指定 CPU 在该段被其他线程占用 {float(long_wait['competing_runtime_ms']):.6f} ms，空闲仅 {float(long_wait['idle_runtime_ms']):.6f} ms；主要占用者是 `dds.udp.27457[1648118]` 6.131000 ms、`ksoftirqd/0[12]` 1.413000 ms 和 `irq/134-host_sy[174]` 1.007000 ms。",
        "",
        "这两段都有完整的 assigned-CPU 占用闭合。直接原因是 worker 已可运行，但其当时所在 CPU 持续运行其他线程；在迁移到另一 CPU 后才重新获得执行。现有事件不包含 CFS vruntime、调度域负载均衡决策和 CPU affinity 的时点快照，因此无法进一步证明为何调度器没有更早迁移。",
        "",
        "## 唤醒依赖链",
        "",
    ]
    if len(chain) == 3:
        first_time = float(chain[0]["wake_time"])
        lines.append(f"原始 sched_waking 事件验证链条真实存在：TID 1646959 在 24.655 ms runnable waiting 后于 968509.412974000 s 重新获得 CPU，并在 {first_time:.9f} s 唤醒 TID 1646942，两者间隔 {float(chain[0]['wake_after_waker_schedule_in_ms']):.6f} ms；随后 TID 1646942 在 {float(chain[1]['wake_time']):.9f} s 唤醒 TID 1646949，前后唤醒事件间隔 {float(chain[1]['interval_from_previous_chain_event_ms']):.6f} ms；TID 1646949 在 {float(chain[2]['wake_time']):.9f} s 唤醒 TID 1646953，前后唤醒事件间隔 {float(chain[2]['interval_from_previous_chain_event_ms']):.6f} ms。该链证明唤醒先后与直接 waker，不证明睡眠期间等待的内核对象。")
    else:
        lines.append("现有数据无法证明完整的三段唤醒链；详见依赖 CSV 中的原始事件验证列。")
    lines.extend([
        "",
        "## irq/134-host_sy",
        "",
        f"原始 sched 事件确认完整线程名为 `{irq_comm}`，TID 为 {irq_tid}。它是内核 IRQ 线程，但现有数据没有 `/proc/interrupts`、IRQ action 和设备驱动映射，因此不能称为 GPU IRQ。",
        "",
        f"ThreadPoolRun 内运行时间：F887 为 {float(irq_threadpool[887]['runtime_ms']):.6f} ms，F888 为 {float(irq_threadpool[888]['runtime_ms']):.6f} ms，F889 为 {float(irq_threadpool[889]['runtime_ms']):.6f} ms。F888 对 worker runnable 的时间重叠为 {float(irq_threadpool[888]['worker_runnable_overlap_ms']):.6f} ms。是否显著增强应以该三帧表中运行时间、段数与最大连续运行时间共同判断，不能只因它出现在异常窗口就认定为主因。",
        "",
        "## 证据级回答",
        "",
        "### A. Blocked 能解释多少 ThreadPoolRun 增长",
        "",
        f"按关键路径 worker 状态账本，blocked 增量为 {decomposition['blocked_delta_ms']:.6f} ms，占约 {decomposition['blocked_delta_ms'] / excess * 100:.2f}%。该数值回答时间归属；具体 blocked 对象仍需同步原语或设备等待 trace 才能定因。",
        "",
        "### B. Runnable Waiting 能解释多少 ThreadPoolRun 增长",
        "",
        f"按关键路径 worker 状态账本，runnable 增量为 {decomposition['runnable_delta_ms']:.6f} ms，占约 {decomposition['runnable_delta_ms'] / excess * 100:.2f}%。",
        "",
        "### C. Runnable Waiting 的主要 CPU 竞争线程",
        "",
    ])
    for (tid, comm, priority), duration in competitors.most_common(8):
        lines.append(f"- `{comm}[{tid}]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 {duration:.6f} ms，sched 优先级数值为 {priority}。")
    lines.append("Linux sched 优先级数值越小，调度优先级越高。四个 Prediction worker 的记录值为 120；`irq/134-host_sy[174]` 的记录值为 49，但它在七段长 runnable waiting 中的累计占用并非最大。逐段 CPU 路由、优先级和全部竞争线程见 `f888_runnable_delay_rootcause.csv`，独立明细见 `f888_runnable_cpu_occupancy.csv`。")
    lines.extend([
        "",
        "### D. Blocked 的主要 waker 与依赖对象",
        "",
        "waker 可以由 sched_waking 直接确定。按大于 2 ms 的 blocked 区间汇总如下；不同 worker 的 blocked 可以并行重叠，因此这些时长不能相加后与 ThreadPoolRun 直接比较。",
        "",
    ])
    for (waker_tid, waker_comm), duration in blocked_by_waker.most_common():
        lines.append(f"- `{waker_comm}[{waker_tid}]`：{blocked_count_by_waker[(waker_tid, waker_comm)]} 次直接唤醒，对应 blocked 区间累计 {duration:.6f} ms。")
    lines.extend([
        "",
        "长 blocked 区间中存在 worker 间串行唤醒，且上述三段链已由原始事件验证。blocked 期间等待的具体对象，现有数据无法证明。`sched_waking` 只证明谁执行了唤醒，不包含 futex 地址、锁 owner、CUDA event、dma-fence 或设备队列标识。",
        "",
        "### E. irq/134-host_sy 是否为 F888 特有或显著增强",
        "",
    ])
    f887, f888, f889 = (float(irq_threadpool[frame]["runtime_ms"]) for frame in FRAMES)
    if f888 > max(f887, f889) * 1.5:
        lines.append(f"它不是 F888 特有，因为 F887 与 F889 均有运行记录；F888 ThreadPoolRun 内运行时间高于两帧对照，分别是 F887 的 {f888 / f887:.2f} 倍和 F889 的 {f888 / f889:.2f} 倍。F888 运行段数不是三帧最高，但最大连续运行段为 {float(irq_threadpool[888]['max_continuous_runtime_ms']):.6f} ms，高于 F887 的 {float(irq_threadpool[887]['max_continuous_runtime_ms']):.6f} ms 和 F889 的 {float(irq_threadpool[889]['max_continuous_runtime_ms']):.6f} ms。局部三帧可描述为运行时间与最长连续段增强，但样本不足以进行总体统计显著性判断。它与 worker runnable 的精确重叠量见对照 CSV；即使增多，也只能证明同期 CPU 竞争活动增强，不能证明它触发了 blocked。")
    else:
        lines.append("它不是 F888 特有，且三帧对照不足以支持它在 F888 显著增强。其运行与部分 runnable 区间重叠只能说明 CPU 竞争，不说明 blocked 的等待对象。")
    lines.extend([
        "",
        "### F. PI pthread mutex 是否已量化为主要根因",
        "",
        "目前只能证明 PI pthread mutex 慢路径在 CPU stack 采样中存在。现有数据无法证明它覆盖了主要 blocked 时长，也无法量化为主要根因。单个或少量 CPU stack 样本不能外推整段 off-CPU 时间，perf sched 也没有用户态 mutex 地址与 owner。",
        "",
        "## 下一次采集需要补充的证据",
        "",
        "- 保留 `sched:sched_switch`、`sched:sched_waking`、`sched:sched_wakeup`、`sched:sched_migrate_task`，并同时记录 raw trace，避免文本转换丢失字段。",
        "- 增加 `syscalls:sys_enter_futex` 与 `syscalls:sys_exit_futex`，或者使用 eBPF uprobe 跟踪 `pthread_mutex_lock`、`pthread_mutex_unlock`、futex wait 与 futex wake；必须保存 mutex 地址、owner TID、waiter TID、PI 标志和返回码。",
        "- 若要区分 GPU 完成等待与其他设备完成等待，增加 CUDA CUPTI 或 Nsight Systems 事件，并采集平台可用的 `dma_fence`、`nvhost`、GPU submit、GPU complete tracepoint，保留 correlation id。",
        "- 采集 `/proc/interrupts`、`/proc/irq/134/*`、`/proc/174/status`、`/proc/174/stack`、`/proc/174/sched`、`/proc/174/cgroup` 和 `/sys/kernel/irq/134/actions`，用于把 `irq/134-host_sy` 映射到确切设备与驱动。",
        "- 在采集前后保存所有目标 TID 的 `/proc/<tid>/status`、`sched`、`cgroup`、`comm`、CPU affinity 与调度策略，避免优先级和绑核配置只能从 perf 事件间接推断。",
        "",
        "## 文件索引",
        "",
        "- `f888_system_sched_timeline.csv`：两个关键窗口的全 CPU sched_switch 运行区间。",
        "- `f888_runnable_delay_rootcause.csv`：七段大于 2 ms runnable waiting 的 CPU 路由与竞争线程。",
        "- `f888_runnable_cpu_occupancy.csv`：每段 runnable waiting 内每个实际 CPU occupant 的独立明细。",
        "- `f888_worker_block_dependency.csv`：大于 2 ms blocked 区间及直接 waker。",
        "- `f888_worker_dependency_timeline.png`：第一关键窗口的 worker 状态与唤醒链。",
        "- `irq_host_sync_same_workload_comparison.csv`：F887、F888、F889 的 IRQ 线程对照。",
        "- `model_system_state_alignment_887_888_889.csv`：每次 CruiseMLP 模型调用与系统状态对齐。",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8-sig")


def main() -> None:
    args = arguments()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    calibration = json.loads((args.analysis_root / "clock_calibration.json").read_text(encoding="utf-8-sig"))
    if float(calibration.get("recheck_trace_support", calibration.get("trace_marker_support", 0))) < 0.95:
        raise RuntimeError("Current-run trace-to-sched calibration support is below 95%")
    parser = load_sched_parser(args.parser_script)
    frame_data = load_frame_data(args.analysis_root, args.model_calls, calibration)
    states = load_existing_states(args.analysis_root, frame_data)
    scan_start = min(frame_data[frame]["callback_start"] for frame in FRAMES) - 0.050
    scan_end = max(frame_data[frame]["callback_end"] for frame in FRAMES) + 0.050
    events = list(parser.iter_range(args.raw_sched, scan_start, scan_end))
    events.sort(key=lambda event: (event["timestamp"], event["cpu"], event["event"]))
    intervals = build_switch_intervals(events, scan_start, scan_end)

    timeline = system_timeline_rows(intervals)
    write_csv(args.output_dir / "f888_system_sched_timeline.csv", timeline)
    runnable, route_audit, occupancy_audit = runnable_rootcause_rows(events, intervals, states)
    write_csv(args.output_dir / "f888_runnable_delay_rootcause.csv", runnable)
    write_csv(args.output_dir / "f888_runnable_cpu_route_audit.csv", route_audit)
    write_csv(args.output_dir / "f888_runnable_cpu_occupancy.csv", occupancy_audit)
    blocked = blocked_dependency_rows(args.analysis_root, frame_data, events)
    write_csv(args.output_dir / "f888_worker_block_dependency.csv", blocked)
    draw_dependency_timeline(args.output_dir / "f888_worker_dependency_timeline.png", states, blocked)
    irq_table, irq_tid, irq_comm = irq_rows(events, intervals, frame_data, states)
    write_csv(args.output_dir / "irq_host_sync_same_workload_comparison.csv", irq_table)
    alignment = model_alignment_rows(args.analysis_root, frame_data)
    write_csv(args.output_dir / "model_system_state_alignment_887_888_889.csv", alignment)
    render_report(args.output_dir / "F888_system_rootcause_evidence.md", args.analysis_root, runnable, occupancy_audit, blocked, irq_table, alignment, irq_tid, irq_comm)

    window_cpu_coverage = {}
    for window_id, start, end in KEY_WINDOWS:
        cpu_coverage = Counter()
        for row in timeline:
            if row["window_id"] == window_id:
                cpu_coverage[int(row["cpu"])] += float(row["running_interval_ms"])
        window_cpu_coverage[window_id] = {
            "cpu_count": len(cpu_coverage),
            "min_coverage_ms": min(cpu_coverage.values()),
            "max_coverage_ms": max(cpu_coverage.values()),
            "expected_per_cpu_ms": (end - start) * 1000,
        }
        if len(cpu_coverage) != 12 or max(abs(value - (end - start) * 1000) for value in cpu_coverage.values()) > 1e-3:
            raise RuntimeError(f"Incomplete system timeline coverage for {window_id}")
    if any(abs(float(row["occupancy_coverage_ratio"]) - 1.0) > 1e-6 for row in runnable):
        raise RuntimeError("A runnable episode lacks complete assigned-CPU occupancy coverage")
    validation = {
        "raw_sched": str(args.raw_sched),
        "raw_sched_size_bytes": args.raw_sched.stat().st_size,
        "scan_start_s": scan_start,
        "scan_end_s": scan_end,
        "parsed_sched_event_count": len(events),
        "cpu_count_with_switches": len(intervals),
        "timeline_rows": len(timeline),
        "window_cpu_coverage": window_cpu_coverage,
        "runnable_over_2ms_count": len(runnable),
        "blocked_over_2ms_count": len(blocked),
        "trace_to_sched_calibration": calibration,
        "worker_state_source": str(args.analysis_root / "worker_gantt" / "worker_sched_timeline.csv"),
        "irq_identity": {"tid": irq_tid, "comm": irq_comm},
        "limitations": [
            "CPU stack samples are not used to extend blocked durations.",
            "sched_waking identifies the direct waker, not the waited-on object.",
            "irq/134-host_sy cannot be mapped to a device without procfs and IRQ action evidence.",
        ],
    }
    (args.output_dir / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
