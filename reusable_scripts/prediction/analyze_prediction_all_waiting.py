"""Build full-run Prediction callback, NotifyTask, and waiting-episode evidence.

The script consumes a verified current-run clock calibration and a scheduler
context containing every Prediction callback TID.  It optionally enriches the
episodes from the system-wide Notify RCA perf-script report in one sequential
pass; the large report is never copied or loaded into memory.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import importlib.util
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PERF = load_module("prediction_waiting_perf", ROOT / "perf" / "generate_p4_perf_analysis.py")
HEADER = re.compile(
    r"^\s*(.*?)\s+(-?\d+)(?:/(-?\d+))?\s+\[(\d+)\]\s+"
    r"(?:\S+\s+)?(\d+\.\d+):\s+(?:\d+\s+)?"
    r"((?:sched|syscalls|raw_syscalls):\w+):\s*(.*)$"
)
SWITCH = re.compile(
    r"prev_comm=(.*?) prev_pid=(-?\d+) prev_prio=(-?\d+) prev_state=(\S+)\s+==>\s+"
    r"next_comm=(.*?) next_pid=(-?\d+) next_prio=(-?\d+)"
)
RAW_SYSCALL_ID = re.compile(r"(?:\bNR\s+|\bid\s*[=:]\s*)(-?\d+)")


def header_time(raw: bytes) -> float | None:
    match = HEADER.match(raw.decode("utf-8", errors="replace"))
    return float(match.group(5)) if match else None


def event_at_or_after_offset(handle, offset: int, scan_limit: int = 16 * 1024 * 1024):
    handle.seek(offset)
    if offset:
        handle.readline()
    origin = handle.tell()
    while handle.tell() - origin <= scan_limit:
        position = handle.tell()
        line = handle.readline()
        if not line:
            return None
        timestamp = header_time(line)
        if timestamp is not None:
            return position, timestamp
    return None


def lower_bound_event_offset(path: Path, target: float) -> int:
    size = path.stat().st_size
    if size <= 1024 * 1024:
        return 0
    low, high = 0, size
    with path.open("rb") as handle:
        while high - low > 1024 * 1024:
            middle = (low + high) // 2
            probe = event_at_or_after_offset(handle, middle)
            if probe is None:
                high = middle
                continue
            position, timestamp = probe
            if timestamp < target:
                low = max(low + 1, position + 1)
            else:
                high = min(high, position)
    return high


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values: list[float], probability: float) -> float:
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * probability
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return values[low]
    return values[low] * (high - position) + values[high] * (position - low)


def trace_to_sched_s(value_ns: str | int, calibration: dict) -> float:
    source = int(value_ns) / 1e9
    return source + float(calibration["trace_offset_intercept_s"]) + float(
        calibration["trace_offset_slope"]
    ) * (source - float(calibration["trace_reference_s"]))


def load_sched(path: Path, tids: set[int], start: float, end: float):
    retained = [PERF.parse_sched_line(line) for line in path.read_text(encoding="utf-8").splitlines()]
    if not retained or not all(retained):
        raise ValueError("Scheduler context is empty or contains unparseable events")
    if any(left["time"] > right["time"] for left, right in zip(retained, retained[1:])):
        raise ValueError("Scheduler context is not ordered")
    original = PERF.iter_sched_range
    PERF.iter_sched_range = lambda _path, left, right: (
        event for event in retained if left <= event["time"] <= right
    )
    try:
        sched = PERF.scan_scheduler(path, tids, start, end)
    finally:
        PERF.iter_sched_range = original
    return sched, retained


def stage_episodes(stage: dict, runs: list[dict], wake_events: list[dict], migrations: list[dict]) -> list[dict]:
    start = float(stage["aligned_sched_start_s"])
    end = float(stage["aligned_sched_end_s"])
    result = []
    sequence = 0
    wake_times = [event["time"] for event in wake_events]
    for left_run, right_run in zip(runs, runs[1:]):
        gap_start, gap_end = left_run["end"], right_run["start"]
        if gap_end <= start or gap_start >= end or gap_end <= gap_start:
            continue
        state = left_run["prev_state"]
        position = bisect.bisect_left(wake_times, gap_start)
        wake = wake_events[position] if position < len(wake_events) and wake_times[position] <= gap_end else None
        pieces = []
        if state.startswith("R") or state == "W":
            pieces.append(("runnable", gap_start, gap_end, "switchout_R"))
        elif state in {"S", "D", "I"}:
            if wake is None:
                pieces.append(("blocked", gap_start, gap_end, "no_wakeup_observed"))
            else:
                if wake["time"] > gap_start:
                    pieces.append(("blocked", gap_start, wake["time"], wake["event"]))
                if gap_end > wake["time"]:
                    pieces.append(("runnable", wake["time"], gap_end, wake["event"]))
        else:
            pieces.append(("unknown", gap_start, gap_end, "unclassified_prev_state"))
        for kind, left, right, basis in pieces:
            left, right = max(left, start), min(right, end)
            if right <= left:
                continue
            sequence += 1
            moved = [event for event in migrations if left <= event["time"] <= right]
            result.append({
                "episode_index": sequence,
                "episode_type": kind,
                "start_s": left,
                "end_s": right,
                "duration_ms": (right - left) * 1000,
                "offcpu_start_s": gap_start,
                "offcpu_end_s": gap_end,
                "sched_out_s": gap_start,
                "sched_in_s": gap_end,
                "prev_state": state,
                "out_cpu": left_run["cpu"],
                "in_cpu": right_run["cpu"],
                "ready_basis": basis,
                "wakeup_time_s": wake["time"] if wake else "",
                "wakeup_event": wake["event"] if wake else "",
                "waker_tid": wake.get("sample_tid", "") if wake else "",
                "waker_comm": wake.get("comm", "") if wake else "",
                "wakeup_target_cpu": wake.get("target_cpu", "") if wake else "",
                "migration_count": len(moved),
                "migration_path": ";".join(
                    f'{event.get("orig_cpu", "")}->{event.get("dest_cpu", "")}@{event["time"]:.9f}'
                    for event in moved
                ),
            })
    return result


def parse_hex_word(text: str) -> int | None:
    try:
        return int(text, 16)
    except ValueError:
        return None


FUTEX_ARGS = re.compile(r"NR\s+98\s+\(([^,]+),\s*([^,\)]+)")


def futex_call(body: str) -> tuple[str, str, bool]:
    match = FUTEX_ARGS.search(body)
    if not match:
        return "", "", False
    address, operation_text = match.groups()
    operation = parse_hex_word(operation_text.strip())
    command = (operation & 0x7F) if operation is not None else None
    return address.strip(), operation_text.strip(), command in {0, 9}


def primary_cause(row: dict) -> tuple[str, str]:
    if row["episode_type"] == "runnable":
        if row.get("sched_yield_associated"):
            return "sched_yield_associated_runnable", "medium"
        if row.get("ready_basis") in {"sched_waking", "sched_wakeup", "sched_wakeup_new"}:
            return "post_wakeup_runnable", "medium"
        if row.get("competitor_covered_ms", 0) > 0:
            return "cpu_competition_runnable", "medium"
        return "unknown_runnable", "unknown"
    if row["episode_type"] == "blocked":
        if row.get("futex_wait_associated"):
            return "futex_associated_blocked", "medium"
        if row.get("wakeup_event") and row.get("switchout_stack"):
            return "non_futex_blocked", "medium"
        return "unknown_blocked", "unknown"
    return "unknown", "unknown"


def stack_signature(stack: str) -> str:
    symbols = []
    for line in stack.splitlines():
        match = re.search(r"\s([^\s]+)\+0x[0-9a-fA-F]+", line)
        if match and (not symbols or symbols[-1] != match.group(1)):
            symbols.append(match.group(1))
    return " -> ".join(symbols)


def interval_index(episodes: list[dict]) -> dict[int, tuple[list[float], list[dict]]]:
    by_cpu = defaultdict(list)
    for row in episodes:
        if row["episode_type"] != "runnable":
            continue
        cpu = row["out_cpu"] if row["ready_basis"] == "switchout_R" else row.get("wakeup_target_cpu", "")
        try:
            cpu = int(cpu)
        except (TypeError, ValueError):
            continue
        cursor = row["start_s"]
        migrations = []
        for token in str(row.get("migration_path", "")).split(";"):
            match = re.match(r"(\d+)->(\d+)@(\d+\.\d+)", token)
            if match:
                migrations.append((float(match.group(3)), int(match.group(1)), int(match.group(2))))
        for when, original, destination in migrations:
            if when > cursor:
                by_cpu[cpu].append({"start": cursor, "end": min(when, row["end_s"]), "episode": row})
            cpu = destination if original == cpu else destination
            cursor = max(cursor, when)
        if cursor < row["end_s"]:
            by_cpu[cpu].append({"start": cursor, "end": row["end_s"], "episode": row})
    return {
        cpu: ([segment["start"] for segment in sorted(rows, key=lambda x: x["start"])],
              sorted(rows, key=lambda x: x["start"]))
        for cpu, rows in by_cpu.items()
    }


def overlaps_for_cpu(index, cpu: int, left: float, right: float):
    if cpu not in index:
        return []
    starts, rows = index[cpu]
    position = max(0, bisect.bisect_left(starts, left) - 1)
    output = []
    while position < len(rows) and rows[position]["start"] < right:
        row = rows[position]
        a, b = max(left, row["start"]), min(right, row["end"])
        if b > a:
            output.append((row["episode"], a, b))
        position += 1
    return output


def enrich_from_rca(path: Path, episodes: list[dict], scan_start: float, scan_end: float) -> dict:
    target_tids = {int(row["tid"]) for row in episodes}
    by_tid = defaultdict(list)
    for row in episodes:
        by_tid[int(row["tid"])].append(row)
        row.update(
            switchout_stack="", waker_stack="", futex_uaddr="", futex_op="",
            futex_wait_associated=False, sched_yield_associated=False,
            competitor_covered_ms=0.0, competitor_event_gap_ms=0.0,
            top_competitor_tid="", top_competitor_comm="", top_competitor_prio="",
            top_competitor_ms=0.0, competitor_count=0,
        )
    for rows in by_tid.values():
        rows.sort(key=lambda row: row["start_s"])
    runnable_index = interval_index(episodes)
    cpu_active = {}
    resident_totals = defaultdict(Counter)
    target_events = defaultdict(list)
    parsed_headers = 0
    first_time = last_time = None
    current_kept = None

    def relevant_rows(tid: int, time_s: float, pad: float = 0.001):
        return [row for row in by_tid.get(tid, []) if row["offcpu_start_s"] - pad <= time_s <= row["offcpu_end_s"] + pad]

    def finish_current():
        nonlocal current_kept
        if current_kept is not None:
            target_events[current_kept["tid"]].append(current_kept)
            current_kept = None

    offset = lower_bound_event_offset(path, scan_start - 0.5)
    with path.open("rb") as handle:
        handle.seek(max(0, offset - 64 * 1024 * 1024))
        if handle.tell():
            handle.readline()
        for raw in handle:
            text = raw.decode("utf-8", errors="replace")
            match = HEADER.match(text)
            if not match:
                if current_kept is not None and text.strip():
                    current_kept["stack"] += text
                continue
            finish_current()
            comm, pid, tid_text, cpu_text, timestamp, event, body = match.groups()
            time_s = float(timestamp)
            if time_s < scan_start - 0.05:
                continue
            if time_s > scan_end + 0.05:
                break
            parsed_headers += 1
            first_time = time_s if first_time is None else first_time
            last_time = time_s
            sample_tid, cpu = int(tid_text or pid), int(cpu_text)
            if event == "sched:sched_switch":
                switch = SWITCH.search(body)
                if not switch:
                    continue
                prev_comm, prev_pid, prev_prio, prev_state, next_comm, next_pid, next_prio = switch.groups()
                previous = cpu_active.get(cpu)
                if previous and time_s > previous["start"]:
                    for episode, left, right in overlaps_for_cpu(runnable_index, cpu, previous["start"], time_s):
                        duration = (right - left) * 1000
                        if int(previous["pid"]) == int(episode["tid"]):
                            continue
                        key = (previous["pid"], previous["comm"], previous["prio"])
                        resident_totals[id(episode)][key] += duration
                        episode["competitor_covered_ms"] += duration
                cpu_active[cpu] = {"pid": int(next_pid), "comm": next_comm, "prio": int(next_prio), "start": time_s}
                if int(prev_pid) in target_tids and relevant_rows(int(prev_pid), time_s):
                    current_kept = {"tid": int(prev_pid), "time": time_s, "event": event, "body": body, "stack": ""}
            elif event in {"sched:sched_waking", "sched:sched_wakeup", "sched:sched_migrate_task"}:
                values = dict(re.findall(r"(\w+)=([^\s]+)", body))
                target = int(values.get("pid", -1))
                if target in target_tids and relevant_rows(target, time_s):
                    current_kept = {"tid": target, "time": time_s, "event": event, "body": body,
                                    "sample_tid": sample_tid, "comm": comm, "stack": ""}
            elif event.startswith("raw_syscalls:") and sample_tid in target_tids and relevant_rows(sample_tid, time_s):
                syscall = RAW_SYSCALL_ID.search(body)
                if syscall:
                    current_kept = {"tid": sample_tid, "time": time_s, "event": event,
                                    "syscall_id": int(syscall.group(1)), "body": body, "stack": ""}
        finish_current()

    for tid, rows in by_tid.items():
        events = sorted(target_events.get(tid, []), key=lambda event: event["time"])
        for row in rows:
            out_time, in_time = row["offcpu_start_s"], row["offcpu_end_s"]
            switches = [event for event in events if event["event"] == "sched:sched_switch" and abs(event["time"] - out_time) <= 0.000002]
            if switches:
                row["switchout_stack"] = switches[0]["stack"].strip()
            wakes = [event for event in events if event["event"] in {"sched:sched_waking", "sched:sched_wakeup"}
                     and out_time <= event["time"] <= in_time]
            if wakes:
                row["waker_tid"] = wakes[0].get("sample_tid", "")
                row["waker_comm"] = wakes[0].get("comm", "")
                row["waker_stack"] = wakes[0]["stack"].strip()
            enters = [event for event in events if event["event"] == "raw_syscalls:sys_enter"
                      and out_time - 0.001 <= event["time"] <= out_time + 0.000002]
            for event in reversed(enters):
                if event.get("syscall_id") == 124:
                    row["sched_yield_associated"] = True
                    break
                if event.get("syscall_id") == 98:
                    address, operation, is_wait = futex_call(event["body"])
                    row["futex_uaddr"], row["futex_op"] = address, operation
                    row["futex_wait_associated"] = is_wait
                    break
            totals = resident_totals[id(row)]
            if totals:
                (comp_tid, comp_comm, comp_prio), value = totals.most_common(1)[0]
                row["top_competitor_tid"] = comp_tid
                row["top_competitor_comm"] = comp_comm
                row["top_competitor_prio"] = comp_prio
                row["top_competitor_ms"] = value
                row["competitor_count"] = len(totals)
            if row["episode_type"] == "runnable":
                row["competitor_event_gap_ms"] = max(0.0, float(row["duration_ms"]) - row["competitor_covered_ms"])
            row["rca_coverage_ok"] = bool(first_time is not None and last_time is not None
                                           and first_time <= out_time and in_time <= last_time)
            if row["rca_coverage_ok"]:
                row["cause_class"], row["confidence"] = primary_cause(row)
            else:
                row["cause_class"], row["confidence"] = "unknown_rca_coverage", "unknown"
    return {"parsed_event_headers": parsed_headers, "first_event_s": first_time, "last_event_s": last_time,
            "target_evidence_events": sum(map(len, target_events.values()))}


def summary_rows(episodes: list[dict], level: str, eligible_frames: int, eligible_instances: int) -> list[dict]:
    eligible = [row for row in episodes if row["level"] == level
                and row["episode_type"] in {"runnable", "blocked"} and row.get("rca_coverage_ok")]
    total_waiting = sum(float(row["duration_ms"]) for row in eligible)
    output = []
    for cause in sorted({row.get("cause_class", "unknown") for row in eligible}):
        rows = [row for row in eligible if row.get("cause_class", "unknown") == cause]
        values = [float(row["duration_ms"]) for row in rows]
        frames = {row["frame_index"] for row in rows}
        output.append({
            "level": level, "cause": cause, "frames_affected": len(frames),
            "eligible_frames": eligible_frames,
            "eligible_instances": eligible_instances,
            "cause_frame_rate": len(frames) / eligible_frames if eligible_frames else "",
            "episodes": len(rows), "total_delay_ms": sum(values), "median_ms": percentile(values, 0.5),
            "p95_ms": percentile(values, 0.95), "max_ms": max(values),
            "waiting_time_share": sum(values) / total_waiting if total_waiting else "",
        })
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--instances", type=Path, required=True)
    parser.add_argument("--callback-states", type=Path, required=True)
    parser.add_argument("--notify-states", type=Path, required=True)
    parser.add_argument("--sched-context", type=Path, required=True)
    parser.add_argument("--calibration-json", type=Path, required=True)
    parser.add_argument("--rca-events", type=Path)
    parser.add_argument("--rca-status", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    calibration = json.loads(args.calibration_json.read_text(encoding="utf-8"))
    if calibration.get("status") != "verified" or float(calibration.get("trace_heldout_support", 0)) < 0.95:
        raise ValueError("Current-run trace-to-sched calibration is not verified")
    if float(calibration.get("stack_heldout_support", 0)) < 0.95:
        raise ValueError("Current-run CPU-stack calibration is not verified")
    frames = read_csv(args.frames)
    instances = read_csv(args.instances)
    callbacks = read_csv(args.callback_states)
    notify_states = read_csv(args.notify_states)
    if len(callbacks) != len(frames):
        raise ValueError("Callback state count does not match complete frame count")
    if len(notify_states) != len(frames) * 16:
        raise ValueError("Expected exactly 16 NotifyTask instances per frame")
    tids = {int(row["callback_tid"]) for row in frames}
    sched, retained = load_sched(args.sched_context, tids, float(calibration["scan_start_s"]), float(calibration["scan_end_s"]))
    by_tid_wakes = defaultdict(list)
    by_tid_migrations = defaultdict(list)
    for event in retained:
        if event["event"] in {"sched_waking", "sched_wakeup", "sched_wakeup_new"} and event.get("pid") in tids:
            by_tid_wakes[event["pid"]].append(event)
        elif event["event"] == "sched_migrate_task" and event.get("pid") in tids:
            by_tid_migrations[event["pid"]].append(event)

    frame_by_id = {row["frame_index"]: row for row in frames}
    callback_by_id = {row["frame_index"]: row for row in callbacks}
    notify_by_frame = defaultdict(list)
    for row in notify_states:
        notify_by_frame[row["frame_index"]].append(row)
    for rows in notify_by_frame.values():
        rows.sort(key=lambda row: int(row["enter_ns"]))

    node_by_frame = defaultdict(list)
    for row in instances:
        node_by_frame[row["frame_index"]].append(row)

    episodes = []
    all_frames = []
    all_notify = []
    for frame_id in sorted(frame_by_id, key=int):
        frame = frame_by_id[frame_id]
        callback = callback_by_id[frame_id]
        tid = int(frame["callback_tid"])
        common = {"frame_index": int(frame_id), "trace_id": frame["trace_id"], "tid": tid}
        callback_eps = stage_episodes(callback, sched["target_runs"][tid], by_tid_wakes[tid], by_tid_migrations[tid])
        for row in callback_eps:
            row.update(common, level="callback", notify_index="", stage_instance_index=callback["stage_instance_index"],
                       trace_enter_ns=callback["enter_ns"], trace_exit_ns=callback["exit_ns"])
        episodes.extend(callback_eps)

        notify_rows = notify_by_frame[frame_id]
        for notify_index, stage in enumerate(notify_rows, 1):
            stage_eps = stage_episodes(stage, sched["target_runs"][tid], by_tid_wakes[tid], by_tid_migrations[tid])
            for row in stage_eps:
                row.update(common, level="notify_task", notify_index=notify_index,
                           stage_instance_index=stage["stage_instance_index"], trace_enter_ns=stage["enter_ns"],
                           trace_exit_ns=stage["exit_ns"])
            episodes.extend(stage_eps)
            all_notify.append({
                **common, "notify_index": notify_index, "stage_instance_index": stage["stage_instance_index"],
                "enter_ns": stage["enter_ns"], "exit_ns": stage["exit_ns"],
                "aligned_sched_start_s": stage["aligned_sched_start_s"],
                "aligned_sched_end_s": stage["aligned_sched_end_s"],
                "duration_ms": stage["duration_ms"], "running_ms": stage["running_ms"],
                "runnable_ms": stage["runnable_ms"], "blocked_ms": stage["blocked_ms"],
                "waiting_ms": float(stage["runnable_ms"]) + float(stage["blocked_ms"]),
                "waiting_share": (float(stage["runnable_ms"]) + float(stage["blocked_ms"])) / float(stage["duration_ms"]),
                "unknown_ms": stage["unknown_ms"], "completion_status": stage["completion_status"],
                "runnable_episode_count": sum(row["episode_type"] == "runnable" for row in stage_eps),
                "blocked_episode_count": sum(row["episode_type"] == "blocked" for row in stage_eps),
                "max_runnable_episode_ms": max((row["duration_ms"] for row in stage_eps if row["episode_type"] == "runnable"), default=0),
                "max_blocked_episode_ms": max((row["duration_ms"] for row in stage_eps if row["episode_type"] == "blocked"), default=0),
            })
        async_rows = [row for row in node_by_frame[frame_id] if row["node"] == "semantic_base_async_draw"]
        callback_start, callback_end = int(frame["prediction_enter_ns"]), int(frame["prediction_exit_ns"])
        async_overlap_ns = sum(max(0, min(callback_end, int(row["exit_ns"])) - max(callback_start, int(row["enter_ns"]))) for row in async_rows)
        longest = max(notify_rows, key=lambda row: float(row["duration_ms"]))
        parent = next(row for row in node_by_frame[frame_id] if row["node"] == "cyber_taskmanager_notify_tasks")
        all_frames.append({
            "frame_index": int(frame_id), "trace_id": frame["trace_id"], "callback_tid": tid,
            "prediction_enter_ns": frame["prediction_enter_ns"], "prediction_exit_ns": frame["prediction_exit_ns"],
            "aligned_sched_start_s": callback["aligned_sched_start_s"],
            "aligned_sched_end_s": callback["aligned_sched_end_s"], "execution_ms": frame["execution_ms"],
            "running_ms": callback["running_ms"], "runnable_ms": callback["runnable_ms"],
            "blocked_ms": callback["blocked_ms"],
            "waiting_ms": float(callback["runnable_ms"]) + float(callback["blocked_ms"]),
            "waiting_share": (float(callback["runnable_ms"]) + float(callback["blocked_ms"])) / float(frame["execution_ms"]),
            "unknown_ms": callback["unknown_ms"],
            "runnable_episode_count": sum(row["episode_type"] == "runnable" for row in callback_eps),
            "blocked_episode_count": sum(row["episode_type"] == "blocked" for row in callback_eps),
            "max_runnable_episode_ms": max((row["duration_ms"] for row in callback_eps if row["episode_type"] == "runnable"), default=0),
            "max_blocked_episode_ms": max((row["duration_ms"] for row in callback_eps if row["episode_type"] == "blocked"), default=0),
            "notify_tasks_duration_ms": parent["duration_ms"], "notifytask_count": len(notify_rows),
            "notifytask_running_ms": sum(float(row["running_ms"]) for row in notify_rows),
            "notifytask_runnable_ms": sum(float(row["runnable_ms"]) for row in notify_rows),
            "notifytask_blocked_ms": sum(float(row["blocked_ms"]) for row in notify_rows),
            "longest_notifytask_duration_ms": longest["duration_ms"],
            "longest_notifytask_running_ms": longest["running_ms"],
            "longest_notifytask_runnable_ms": longest["runnable_ms"],
            "longest_notifytask_blocked_ms": longest["blocked_ms"],
            "async_schedule_calls": frame.get("semantic_base_async_schedule_calls", ""),
            "async_submit_calls": frame.get("semantic_base_async_submit_calls", ""),
            "notify_tasks_calls": frame.get("cyber_taskmanager_notify_tasks_calls", ""),
            "notify_task_calls": frame.get("cyber_taskmanager_notify_task_calls", ""),
            "async_draw_calls": frame.get("semantic_base_async_draw_calls", ""),
            "async_completion_status": ";".join(sorted({row.get("completion_status", "") for row in async_rows})),
            "draw_base_map_thread_overlaps_callback": async_overlap_ns > 0,
            "async_draw_callback_overlap_ms": async_overlap_ns / 1e6,
            "coverage_ok": float(callback["unknown_ms"]) <= 0.001,
        })

    rca_audit = None
    if args.rca_events:
        rca_audit = enrich_from_rca(args.rca_events, episodes, float(calibration["scan_start_s"]), float(calibration["scan_end_s"]))
    else:
        for row in episodes:
            row.update(
                cause_class="not_enriched", confidence="unknown", rca_coverage_ok=False,
                futex_wait_associated=False,
                futex_uaddr="", futex_op="", sched_yield_associated=False, waker_tid="",
                waker_comm="", switchout_stack="", waker_stack="", top_competitor_tid="",
                top_competitor_comm="", top_competitor_prio="", top_competitor_ms=0.0,
                competitor_count=0, competitor_covered_ms=0.0, competitor_event_gap_ms=0.0,
            )

    callback_waiting = [row for row in episodes if row["level"] == "callback"]
    by_frame_waiting = defaultdict(list)
    for row in callback_waiting:
        by_frame_waiting[row["frame_index"]].append(row)
    for frame in all_frames:
        rows = by_frame_waiting[frame["frame_index"]]
        frame["futex_blocked_ms"] = sum(row["duration_ms"] for row in rows if row.get("cause_class") == "futex_associated_blocked")
        frame["yield_runnable_ms"] = sum(row["duration_ms"] for row in rows if row.get("cause_class") == "sched_yield_associated_runnable")
        frame["cpu_competition_runnable_ms"] = sum(row["duration_ms"] for row in rows if row.get("cause_class") == "cpu_competition_runnable")
        frame["post_wakeup_runnable_ms"] = sum(row["duration_ms"] for row in rows if row.get("cause_class") == "post_wakeup_runnable")
        frame["unknown_waiting_ms"] = sum(row["duration_ms"] for row in rows if str(row.get("cause_class", "")).startswith("unknown"))

    covered_frames = {
        row["frame_index"] for row in all_frames
        if rca_audit and rca_audit["first_event_s"] <= float(row["aligned_sched_start_s"])
        and float(row["aligned_sched_end_s"]) <= rca_audit["last_event_s"]
    }
    covered_notify_instances = sum(
        rca_audit and rca_audit["first_event_s"] <= float(row["aligned_sched_start_s"])
        and float(row["aligned_sched_end_s"]) <= rca_audit["last_event_s"]
        for row in all_notify
    )
    for frame in all_frames:
        frame["rca_cause_coverage_ok"] = frame["frame_index"] in covered_frames
    write_csv(args.output_dir / "all_frames_system_states.csv", all_frames)
    write_csv(args.output_dir / "all_notifytask_system_states.csv", all_notify)
    write_csv(args.output_dir / "all_waiting_episodes.csv", episodes)
    causes = summary_rows(episodes, "callback", len(covered_frames), len(covered_frames)) + summary_rows(
        episodes, "notify_task", len(covered_frames), covered_notify_instances)
    write_csv(args.output_dir / "waiting_cause_summary.csv", causes)

    notify_summary = []
    for index in range(1, 17):
        rows = [row for row in all_notify if row["notify_index"] == index]
        waits = [float(row["waiting_ms"]) for row in rows]
        notify_summary.append({
            "notify_index": index, "calls": len(rows), "waiting_gt_1ms": sum(value > 1 for value in waits),
            "waiting_gt_5ms": sum(value > 5 for value in waits), "total_waiting_ms": sum(waits),
            "total_blocked_ms": sum(float(row["blocked_ms"]) for row in rows),
            "total_runnable_ms": sum(float(row["runnable_ms"]) for row in rows),
            "waiting_median_ms": percentile(waits, 0.5), "waiting_p95_ms": percentile(waits, 0.95),
            "waiting_max_ms": max(waits),
        })
    write_csv(args.output_dir / "notify_index_summary.csv", notify_summary)

    futex = defaultdict(lambda: {"episodes": 0, "frames": set(), "blocked_ms": 0.0})
    wakers = defaultdict(lambda: {"episodes": 0, "frames": set(), "blocked_ms": 0.0})
    competitors = defaultdict(lambda: {"episodes": 0, "frames": set(), "overlap_ms": 0.0})
    for row in callback_waiting:
        if row["episode_type"] == "blocked" and row.get("futex_wait_associated") and row.get("futex_uaddr"):
            item = futex[row["futex_uaddr"]]; item["episodes"] += 1; item["frames"].add(row["frame_index"]); item["blocked_ms"] += row["duration_ms"]
        if row["episode_type"] == "blocked" and row.get("waker_tid") != "":
            key = (row.get("waker_tid"), row.get("waker_comm", "")); item = wakers[key]
            item["episodes"] += 1; item["frames"].add(row["frame_index"]); item["blocked_ms"] += row["duration_ms"]
        if row["episode_type"] == "runnable" and row.get("top_competitor_tid") != "":
            key = (row["top_competitor_tid"], row["top_competitor_comm"], row["top_competitor_prio"]); item = competitors[key]
            item["episodes"] += 1; item["frames"].add(row["frame_index"]); item["overlap_ms"] += row["top_competitor_ms"]
    write_csv(args.output_dir / "futex_address_summary.csv", [
        {"futex_uaddr": key, "episodes": value["episodes"], "frames_affected": len(value["frames"]), "blocked_ms": value["blocked_ms"]}
        for key, value in sorted(futex.items(), key=lambda item: -item[1]["blocked_ms"])
    ], ["futex_uaddr", "episodes", "frames_affected", "blocked_ms"])
    write_csv(args.output_dir / "waker_summary.csv", [
        {"waker_tid": key[0], "waker_comm": key[1], "episodes": value["episodes"], "frames_affected": len(value["frames"]), "blocked_ms": value["blocked_ms"]}
        for key, value in sorted(wakers.items(), key=lambda item: -item[1]["blocked_ms"])
    ], ["waker_tid", "waker_comm", "episodes", "frames_affected", "blocked_ms"])
    write_csv(args.output_dir / "runnable_competitor_summary.csv", [
        {"competitor_tid": key[0], "competitor_comm": key[1], "competitor_prio": key[2], "episodes": value["episodes"], "frames_affected": len(value["frames"]), "overlap_ms": value["overlap_ms"]}
        for key, value in sorted(competitors.items(), key=lambda item: -item[1]["overlap_ms"])
    ], ["competitor_tid", "competitor_comm", "competitor_prio", "episodes", "frames_affected", "overlap_ms"])

    for name, stack_field in (("blocking_stack_summary.csv", "switchout_stack"),
                              ("waker_stack_summary.csv", "waker_stack")):
        groups = defaultdict(lambda: {"episodes": 0, "frames": set(), "delay_ms": 0.0})
        for row in callback_waiting:
            if row.get("rca_coverage_ok") and row["episode_type"] == "blocked" and row.get(stack_field):
                signature = stack_signature(row[stack_field])
                item = groups[signature]
                item["episodes"] += 1
                item["frames"].add(row["frame_index"])
                item["delay_ms"] += row["duration_ms"]
        write_csv(args.output_dir / name, [
            {"stack_signature": key, "episodes": value["episodes"],
             "frames_affected": len(value["frames"]), "blocked_ms": value["delay_ms"]}
            for key, value in sorted(groups.items(), key=lambda item: -item[1]["delay_ms"])
        ], ["stack_signature", "episodes", "frames_affected", "blocked_ms"])

    closure = [abs(float(row["execution_ms"]) - sum(float(row[key]) for key in ("running_ms", "runnable_ms", "blocked_ms", "unknown_ms"))) for row in all_frames]
    notify_closure = [abs(float(row["duration_ms"]) - sum(float(row[key]) for key in ("running_ms", "runnable_ms", "blocked_ms", "unknown_ms"))) for row in all_notify]
    rca_status = json.loads(args.rca_status.read_text(encoding="utf-8")) if args.rca_status else {}
    validation = {
        "status": "complete_with_source_quality_warning" if rca_status.get("quality_review_required") else "complete",
        "frames": len(all_frames), "notifytask_instances": len(all_notify), "waiting_episodes": len(episodes),
        "callback_max_closure_error_ms": max(closure), "notifytask_max_closure_error_ms": max(notify_closure),
        "trace_heldout_support": calibration["trace_heldout_support"],
        "trace_heldout_residual_abs_p95_ms": calibration["trace_heldout_residual_abs_p95_ms"],
        "stack_heldout_support": calibration["stack_heldout_support"],
        "calibration_sha256": sha256(args.calibration_json), "sched_context_sha256": sha256(args.sched_context),
        "rca_audit": rca_audit, "rca_lost_chunks": next((warning.get("text") for warning in rca_status.get("quality_warnings", []) if "lost" in warning.get("text", "")), ""),
        "rca_cause_covered_frames": len(covered_frames),
        "rca_cause_uncovered_frames": len(all_frames) - len(covered_frames),
        "rca_cause_covered_notifytask_instances": covered_notify_instances,
        "identity_limit": "Current-run before snapshots only; no after snapshot was captured.",
        "cause_limit": "Global RCA loss and symbol failures cap positive cause classifications at medium confidence; incomplete chains remain unknown.",
    }
    (args.output_dir / "validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    print(json.dumps(validation, indent=2))


if __name__ == "__main__":
    main()
