#!/usr/bin/env python3
"""Generate auditable per-frame P4 scheduler and CPU-stack extracts.

This script is intentionally read-only with respect to the two perf data files
and their text exports.  It writes only to the sibling ``analysis`` directory.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import importlib.util
import json
import math
import re
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import asdict
from decimal import Decimal
from pathlib import Path
from typing import Iterable, Optional


DATA_ROOT = Path(r"D:\data")
RUN_DIR = DATA_ROOT / "202608271537"
PERF_DIR = RUN_DIR / "perf_run_cpu_stack_001"
ANALYSIS_DIR = PERF_DIR / "analysis"
PERCEPTION_LOG = RUN_DIR / "log" / "perception.log.INFO.20260827-153547.1016337"
CENTERPOINT_CSV = (
    DATA_ROOT
    / "output"
    / "202608271537_perception_deadline_1_1_1_2"
    / "data"
    / "centerpoint_internal_timing_per_source_frame.csv"
)
REUSABLE_SCHED = DATA_ROOT / "reusable_scripts" / "scheduler" / "analyze_perf_sched_infer_frames.py"
CALIBRATION_JSON = (
    DATA_ROOT
    / "output"
    / "202608271537_F611_F649_perf_diagnosis"
    / "sched_clock_calibration.json"
)

FRAME_FIRST = 601
FRAME_LAST = 659
ANOMALY_FIRST = 611
ANOMALY_LAST = 649
SCHED_MINUS_MONOTONIC_NS = -297_346_430
UNKNOWN_RATIO_HIGH_THRESHOLD = 0.50

STACK_HEADER_RE = re.compile(
    r"^(?P<comm>.*?)\s+(?P<tid>\d+)\s+\[(?P<cpu>\d+)\]\s+"
    r"(?P<time>\d+\.\d+):\s+\d+\s+cpu-clock:"
)
STACK_FRAME_RE = re.compile(
    r"^\s*(?:(?P<address>[0-9a-f]+)\s+)?(?P<symbol>.*?)\s+\((?P<dso>[^()]*)\)\s*$"
)
SCHED_PREFIX_RE = re.compile(
    r"^\s*(?P<sample_comm>.*?)\s+(?P<sample_tid>\d+)\s+\[(?P<cpu>\d+)\]\s+"
    r"(?P<time>\d+\.\d+):\s+sched:(?P<event>sched_[a-z_]+):\s+(?P<body>.*)$"
)
SCHED_TIME_BYTES_RE = re.compile(rb"\[(?:\d+)\]\s+(\d+\.\d+):\s+sched:")
SWITCH_RE = re.compile(
    r"prev_comm=(?P<prev_comm>.*?) prev_pid=(?P<prev_pid>\d+) prev_prio=.*? "
    r"prev_state=(?P<prev_state>\S+) ==> next_comm=(?P<next_comm>.*?) "
    r"next_pid=(?P<next_pid>\d+)"
)
WAKE_RE = re.compile(
    r"comm=(?P<comm>.*?) pid=(?P<pid>\d+) prio=.*?(?: target_cpu=(?P<target_cpu>\d+))?$"
)
MIGRATE_RE = re.compile(
    r"comm=(?P<comm>.*?) pid=(?P<pid>\d+) prio=.*? "
    r"orig_cpu=(?P<orig>\d+) dest_cpu=(?P<dest>\d+)"
)


def load_reusable_module():
    spec = importlib.util.spec_from_file_location("reusable_sched", REUSABLE_SCHED)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load reusable scheduler parser: {REUSABLE_SCHED}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: Iterable[dict], fieldnames: list[str]) -> None:
    rows = list(rows)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: Optional[float], digits: int = 6):
    if value is None:
        return ""
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def overlap(left: float, right: float, start: float, end: float) -> float:
    return max(0.0, min(right, end) - max(left, start))


def interval_overlap(intervals_a: list[tuple[float, float]], intervals_b: list[tuple[float, float]]) -> float:
    total = 0.0
    i = 0
    j = 0
    a = sorted(intervals_a)
    b = sorted(intervals_b)
    while i < len(a) and j < len(b):
        total += overlap(a[i][0], a[i][1], b[j][0], b[j][1])
        if a[i][1] <= b[j][1]:
            i += 1
        else:
            j += 1
    return total


def raw_stat(path: Path) -> dict:
    value = path.stat()
    return {
        "path": str(path),
        "size_bytes": value.st_size,
        "mtime_ns": value.st_mtime_ns,
    }


def parse_status_snapshot(path: Path) -> dict[int, dict]:
    result: dict[int, dict] = {}
    current_tid: Optional[int] = None
    current: dict = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        marker = re.match(r"^TID=(\d+)\s*$", line)
        if marker:
            if current_tid is not None:
                result[current_tid] = current
            current_tid = int(marker.group(1))
            current = {}
            continue
        field = re.match(r"^(Tgid|Pid|NSpid):\s*(.*)$", line)
        if field and current_tid is not None:
            current[field.group(1)] = field.group(2).strip()
    if current_tid is not None:
        result[current_tid] = current
    return result


def parse_ps_lwps(path: Path) -> set[int]:
    result: set[int] = set()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(r"^\s*\d+\s+(\d+)\s+", line)
        if match:
            result.add(int(match.group(1)))
    return result


def validate_tid_mapping(tids: set[int]) -> dict[int, dict]:
    before_status = parse_status_snapshot(PERF_DIR / "perception_proc_status_before.txt")
    after_status = parse_status_snapshot(PERF_DIR / "perception_proc_status_after.txt")
    before_ps = parse_ps_lwps(PERF_DIR / "perception_ps_threads_before.txt")
    after_ps = parse_ps_lwps(PERF_DIR / "perception_ps_threads_after.txt")
    result = {}
    for tid in sorted(tids):
        before = before_status.get(tid, {})
        after = after_status.get(tid, {})
        before_ok = before.get("Pid") == str(tid) and before.get("NSpid") == str(tid)
        after_ok = after.get("Pid") == str(tid) and after.get("NSpid") == str(tid)
        ps_ok = tid in before_ps and tid in after_ps
        valid = before_ok and after_ok and ps_ok
        result[tid] = {
            "valid": valid,
            "logged_tid_namespace": (
                "container_tid=host_tid (single-level NSpid identity)" if valid else "unverified"
            ),
            "host_tid": tid if valid else None,
            "evidence": (
                "validated: before/after /proc task status Pid=NSpid={} and Host ps LWP={}"
                .format(tid, tid)
                if valid
                else "unavailable: required before/after Pid=NSpid and Host ps LWP evidence incomplete"
            ),
            "before_status": before,
            "after_status": after,
            "host_ps_before": tid in before_ps,
            "host_ps_after": tid in after_ps,
        }
    return result


def parse_stack_samples(path: Path) -> list[dict]:
    samples: list[dict] = []
    current: Optional[dict] = None

    def finish() -> None:
        nonlocal current
        if current is not None:
            current["raw"] = "".join(current.pop("raw_lines"))
            samples.append(current)
        current = None

    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for line in handle:
            header = STACK_HEADER_RE.match(line)
            if header:
                finish()
                time_text = header.group("time")
                current = {
                    "comm": header.group("comm").strip(),
                    "tid": int(header.group("tid")),
                    "cpu": int(header.group("cpu")),
                    "time_text": time_text,
                    "time_ns": int(Decimal(time_text) * Decimal(1_000_000_000)),
                    "frames": [],
                    "raw_lines": [line],
                }
                continue
            if current is None:
                continue
            current["raw_lines"].append(line)
            if not line.strip():
                continue
            frame = STACK_FRAME_RE.match(line)
            if frame:
                current["frames"].append(
                    {
                        "address": frame.group("address") or "",
                        "symbol": frame.group("symbol").strip(),
                        "dso": frame.group("dso").strip(),
                    }
                )
    finish()
    return samples


def normalized_leaf(sample: dict) -> tuple[str, str]:
    if not sample["frames"]:
        return "[no_callchain]", ""
    leaf = sample["frames"][0]
    symbol = re.sub(r"\+0x[0-9a-fA-F]+$", "", leaf["symbol"])
    return symbol or "[unknown]", leaf["dso"]


def folded_callchain(sample: dict) -> str:
    if not sample["frames"]:
        return "[no_callchain]"
    return "; ".join(
        f"{frame['symbol']} ({frame['dso']})" for frame in sample["frames"]
    )


def parse_sched_line(line: str) -> Optional[dict]:
    prefix = SCHED_PREFIX_RE.match(line.rstrip("\r\n"))
    if not prefix:
        return None
    event = prefix.groupdict()
    event["time"] = float(event["time"])
    event["cpu"] = int(event["cpu"])
    event["sample_tid"] = int(event["sample_tid"])
    event["raw_event"] = line.rstrip("\r\n")
    name = event["event"]
    if name == "sched_switch":
        detail = SWITCH_RE.search(event["body"])
        if not detail:
            return None
        event.update(detail.groupdict())
        event["prev_pid"] = int(event["prev_pid"])
        event["next_pid"] = int(event["next_pid"])
    elif name in {"sched_waking", "sched_wakeup", "sched_wakeup_new"}:
        detail = WAKE_RE.search(event["body"])
        if not detail:
            return None
        event.update(detail.groupdict())
        event["pid"] = int(event["pid"])
        event["target_cpu"] = int(event["target_cpu"]) if event.get("target_cpu") else None
    elif name == "sched_migrate_task":
        detail = MIGRATE_RE.search(event["body"])
        if not detail:
            return None
        event.update(detail.groupdict())
        event["pid"] = int(event["pid"])
        event["orig"] = int(event["orig"])
        event["dest"] = int(event["dest"])
    else:
        return None
    return event


def locate_sched_offset(path: Path, target_s: float) -> int:
    """Binary-search an approximately time-sorted perf script for target_s."""
    size = path.stat().st_size
    low = 0
    high = size
    with path.open("rb") as handle:
        for _ in range(42):
            if high - low < 8192:
                break
            mid = (low + high) // 2
            handle.seek(mid)
            if mid:
                handle.readline()
            pos = handle.tell()
            found = None
            for _ in range(256):
                line = handle.readline()
                if not line:
                    break
                match = SCHED_TIME_BYTES_RE.search(line)
                if match:
                    found = (pos, float(match.group(1)))
                    break
                pos = handle.tell()
            if found is None:
                high = mid
            elif found[1] < target_s:
                low = handle.tell()
            else:
                high = found[0]
    return max(0, low - 4 * 1024 * 1024)


def iter_sched_range(path: Path, start_s: float, end_s: float):
    seek = locate_sched_offset(path, start_s - 0.05)
    crossed_end = False
    with path.open("rb") as handle:
        handle.seek(seek)
        if seek:
            handle.readline()
        for raw in handle:
            line = raw.decode("utf-8", errors="replace")
            event = parse_sched_line(line)
            if event is None:
                continue
            if event["time"] < start_s:
                continue
            if event["time"] > end_s:
                crossed_end = True
                break
            yield event
    if not crossed_end:
        raise RuntimeError(f"scheduler range did not reach requested end {end_s:.9f}")


def scan_scheduler(path: Path, target_tids: set[int], start_s: float, end_s: float) -> dict:
    target_runs: dict[int, list[dict]] = defaultdict(list)
    target_active: dict[int, tuple[float, int]] = {}
    target_wakes: dict[int, list[float]] = defaultdict(list)
    target_switchouts: dict[int, list[float]] = defaultdict(list)
    target_migrations: dict[int, list[float]] = defaultdict(list)
    kswapd_active: dict[int, tuple[float, int]] = {}
    kswapd_runs: list[dict] = []
    kswapd_tids: set[int] = set()
    cpu_current: dict[int, tuple[int, str, float]] = {}
    pending_direct: dict[int, tuple[int, int, str, float]] = {}
    direct_intervals: list[dict] = []
    competitor_links: dict[int, set[int]] = defaultdict(set)
    first_event = None
    last_event = None
    event_count = 0

    def add_direct(target_tid: int, comp_tid: int, comp_comm: str, cpu: int, left: float, right: float):
        if comp_tid == 0 or comp_tid == target_tid or right <= left:
            return
        competitor_links[comp_tid].add(target_tid)
        direct_intervals.append(
            {
                "target_tid": target_tid,
                "competitor_tid": comp_tid,
                "competitor_comm": comp_comm,
                "cpu": cpu,
                "start": left,
                "end": right,
            }
        )

    for event in iter_sched_range(path, start_s, end_s):
        event_count += 1
        first_event = event["time"] if first_event is None else first_event
        last_event = event["time"]
        name = event["event"]
        if name == "sched_switch":
            t = event["time"]
            cpu = event["cpu"]
            prev_tid = event["prev_pid"]
            next_tid = event["next_pid"]
            prev_comm = event["prev_comm"]
            next_comm = event["next_comm"]

            current = cpu_current.get(cpu)
            current_start = current[2] if current and current[0] == prev_tid else start_s
            pending = pending_direct.pop(cpu, None)
            if pending and pending[1] == prev_tid:
                add_direct(pending[0], prev_tid, prev_comm, cpu, pending[3], t)
            if next_tid in target_tids:
                add_direct(next_tid, prev_tid, prev_comm, cpu, current_start, t)

            if prev_tid in target_tids:
                active = target_active.pop(prev_tid, None)
                run_start = active[0] if active is not None else current_start
                target_runs[prev_tid].append(
                    {
                        "start": run_start,
                        "end": t,
                        "cpu": cpu,
                        "prev_state": event["prev_state"],
                        "next_tid": next_tid,
                        "next_comm": next_comm,
                    }
                )
                target_switchouts[prev_tid].append(t)
                if event["prev_state"].startswith("R") and next_tid != 0:
                    pending_direct[cpu] = (prev_tid, next_tid, next_comm, t)
                    competitor_links[next_tid].add(prev_tid)
            if next_tid in target_tids:
                target_active[next_tid] = (t, cpu)

            if prev_comm == "kswapd0":
                kswapd_tids.add(prev_tid)
                active = kswapd_active.pop(prev_tid, None)
                run_start = active[0] if active is not None else current_start
                kswapd_runs.append({"start": run_start, "end": t, "cpu": cpu, "tid": prev_tid})
            if next_comm == "kswapd0":
                kswapd_tids.add(next_tid)
                kswapd_active[next_tid] = (t, cpu)

            cpu_current[cpu] = (next_tid, next_comm, t)

        elif name in {"sched_waking", "sched_wakeup", "sched_wakeup_new"}:
            if event["pid"] in target_tids:
                target_wakes[event["pid"]].append(event["time"])
        elif name == "sched_migrate_task" and event["pid"] in target_tids:
            target_migrations[event["pid"]].append(event["time"])

    for tid, (left, cpu) in target_active.items():
        target_runs[tid].append(
            {
                "start": left,
                "end": end_s,
                "cpu": cpu,
                "prev_state": "UNKNOWN_END",
                "next_tid": -1,
                "next_comm": "",
            }
        )
    for tid, (left, cpu) in kswapd_active.items():
        kswapd_runs.append({"start": left, "end": end_s, "cpu": cpu, "tid": tid})
    for cpu, pending in pending_direct.items():
        add_direct(pending[0], pending[1], pending[2], cpu, pending[3], end_s)

    unique_direct = {}
    for item in direct_intervals:
        key = (
            item["target_tid"], item["competitor_tid"], item["cpu"],
            round(item["start"], 9), round(item["end"], 9),
        )
        unique_direct[key] = item
    return {
        "target_runs": {tid: sorted(value, key=lambda x: x["start"]) for tid, value in target_runs.items()},
        "target_wakes": {tid: sorted(value) for tid, value in target_wakes.items()},
        "target_switchouts": dict(target_switchouts),
        "target_migrations": dict(target_migrations),
        "kswapd_runs": sorted(kswapd_runs, key=lambda x: x["start"]),
        "kswapd_tids": kswapd_tids,
        "direct_intervals": list(unique_direct.values()),
        "competitor_links": competitor_links,
        "first_event_s": first_event,
        "last_event_s": last_event,
        "event_count": event_count,
    }


def classify_target_window(tid: int, start: float, end: float, sched: dict) -> dict:
    runs = sched["target_runs"].get(tid, [])
    wakes = sched["target_wakes"].get(tid, [])
    running_intervals = []
    cpu_set = set()
    for run in runs:
        left = max(run["start"], start)
        right = min(run["end"], end)
        if right > left:
            running_intervals.append((left, right))
            cpu_set.add(run["cpu"])

    blocked_intervals = []
    runnable_intervals = []
    unknown_intervals = []
    for left_run, right_run in zip(runs, runs[1:]):
        gap_start = left_run["end"]
        gap_end = right_run["start"]
        if gap_end <= gap_start or overlap(gap_start, gap_end, start, end) <= 0:
            continue
        state = left_run["prev_state"]
        position = bisect.bisect_left(wakes, gap_start)
        wake = wakes[position] if position < len(wakes) and wakes[position] <= gap_end else None
        if state.startswith("R") or state == "W":
            runnable_intervals.append((max(gap_start, start), min(gap_end, end)))
        elif state in {"S", "D", "I"}:
            if wake is None:
                blocked_intervals.append((max(gap_start, start), min(gap_end, end)))
            else:
                if wake > gap_start:
                    blocked_intervals.append((max(gap_start, start), min(wake, end)))
                if gap_end > wake:
                    runnable_intervals.append((max(wake, start), min(gap_end, end)))
        else:
            unknown_intervals.append((max(gap_start, start), min(gap_end, end)))

    running = sum(right - left for left, right in running_intervals)
    blocked = sum(max(0.0, right - left) for left, right in blocked_intervals)
    runnable = sum(max(0.0, right - left) for left, right in runnable_intervals)
    unknown = sum(max(0.0, right - left) for left, right in unknown_intervals)
    duration = end - start
    unknown += duration - running - blocked - runnable - unknown
    max_delay = max((right - left for left, right in runnable_intervals), default=0.0)
    switches = sum(start <= t <= end for t in sched["target_switchouts"].get(tid, []))
    migrations = sum(start <= t <= end for t in sched["target_migrations"].get(tid, []))

    competitor_cpu: dict[tuple[int, str], float] = defaultdict(float)
    for item in sched["direct_intervals"]:
        if item["target_tid"] != tid:
            continue
        direct = [(max(item["start"], start), min(item["end"], end))]
        value = interval_overlap(direct, runnable_intervals)
        if value > 0:
            competitor_cpu[(item["competitor_tid"], item["competitor_comm"])] += value
    if competitor_cpu:
        top_key, top_seconds = sorted(
            competitor_cpu.items(), key=lambda x: (-x[1], x[0][0], x[0][1])
        )[0]
        top_tid, top_comm = top_key
    else:
        top_tid, top_comm, top_seconds = None, "", 0.0

    frame_interval = [(start, end)]
    kswapd_intervals = [
        (max(run["start"], start), min(run["end"], end))
        for run in sched["kswapd_runs"]
        if overlap(run["start"], run["end"], start, end) > 0
    ]
    kswapd_cpu = interval_overlap(kswapd_intervals, frame_interval)
    kswapd_running = interval_overlap(kswapd_intervals, running_intervals)
    kswapd_runnable = interval_overlap(kswapd_intervals, runnable_intervals)
    return {
        "running_s": running,
        "blocked_s": blocked,
        "runnable_s": runnable,
        "unknown_s": unknown,
        "max_sched_delay_s": max_delay,
        "context_switches": switches,
        "migration_count": migrations,
        "cpu_list": ";".join(map(str, sorted(cpu_set))),
        "running_intervals": running_intervals,
        "runnable_intervals": runnable_intervals,
        "kswapd_cpu_s": kswapd_cpu,
        "kswapd_overlap_running_s": kswapd_running,
        "kswapd_overlap_runnable_s": kswapd_runnable,
        "top_competitor_tid": top_tid,
        "top_competitor_comm": top_comm,
        "top_competitor_cpu_s": top_seconds,
    }


def related_targets(tid: int, anomaly_tids: set[int], competitor_links: dict[int, set[int]]) -> str:
    if tid in anomaly_tids:
        return str(tid)
    targets = sorted(competitor_links.get(tid, set()) & anomaly_tids)
    return "|".join(map(str, targets))


def extract_context_events(
    path: Path,
    start_s: float,
    end_s: float,
    anomaly_tids: set[int],
    anomaly_cpus: set[int],
    kswapd_tids: set[int],
    competition_windows: list[dict],
) -> list[dict]:
    competition_by_tid: dict[int, list[dict]] = defaultdict(list)
    competitor_links: dict[int, set[int]] = defaultdict(set)
    for item in competition_windows:
        competition_by_tid[item["competitor_tid"]].append(item)
        competitor_links[item["competitor_tid"]].add(item["target_tid"])
    core_tids = anomaly_tids | kswapd_tids

    def matching_windows(tid: int, timestamp: float, cpus: set[int]) -> list[dict]:
        # 0.1 ms includes the wake/migration event immediately adjacent to the
        # direct CPU run without expanding to the thread's unrelated activity.
        halo = 0.0001
        return [
            item for item in competition_by_tid.get(tid, [])
            if item["start"] - halo <= timestamp <= item["end"] + halo
            and (not cpus or item["cpu"] in cpus)
        ]

    rows = []
    for event in iter_sched_range(path, start_s, end_s):
        name = event["event"]
        keep = False
        reason = ""
        prev_tid = prev_comm = prev_state = next_tid = next_comm = target_tid = waker_tid = ""
        orig_cpu = dest_cpu = ""
        if name == "sched_switch":
            involved = {event["prev_pid"], event["next_pid"]}
            matches = []
            for tid in involved:
                matches.extend(matching_windows(tid, event["time"], {event["cpu"]}))
            core_keep = bool(involved & core_tids)
            keep = core_keep or bool(matches)
            if keep:
                reason = (
                    "switch_involving_anomaly_target_or_kswapd0"
                    if core_keep else "switch_within_direct_competitor_run_window"
                )
                prev_tid = event["prev_pid"]
                prev_comm = event["prev_comm"]
                prev_state = event["prev_state"]
                next_tid = event["next_pid"]
                next_comm = event["next_comm"]
                target_values = set()
                for tid in involved:
                    related = related_targets(tid, anomaly_tids, competitor_links)
                    if related:
                        target_values.update(map(int, related.split("|")))
                target_values.update(item["target_tid"] for item in matches)
                target_tid = "|".join(map(str, sorted(target_values)))
        elif name in {"sched_waking", "sched_wakeup", "sched_wakeup_new"}:
            event_cpus = {event["cpu"]}
            if event.get("target_cpu") is not None:
                event_cpus.add(event["target_cpu"])
            matches = matching_windows(event["pid"], event["time"], event_cpus)
            core_keep = event["pid"] in core_tids
            keep = core_keep or bool(matches)
            if keep:
                reason = (
                    "wake_of_anomaly_target_or_kswapd0"
                    if core_keep else "wake_adjacent_to_direct_competitor_run_window"
                )
                next_tid = event["pid"]
                next_comm = event["comm"]
                waker_tid = event["sample_tid"]
                target_tid = related_targets(event["pid"], anomaly_tids, competitor_links)
                if matches:
                    target_tid = "|".join(map(str, sorted({item["target_tid"] for item in matches})))
        elif name == "sched_migrate_task":
            matches = matching_windows(
                event["pid"], event["time"], {event["orig"], event["dest"]}
            )
            core_keep = event["pid"] in core_tids
            keep = core_keep or bool(matches)
            if keep:
                reason = (
                    "migration_of_anomaly_target_or_kswapd0"
                    if core_keep else "migration_adjacent_to_direct_competitor_run_window"
                )
                prev_tid = event["pid"]
                prev_comm = event["comm"]
                orig_cpu = event["orig"]
                dest_cpu = event["dest"]
                target_tid = related_targets(event["pid"], anomaly_tids, competitor_links)
                if matches:
                    target_tid = "|".join(map(str, sorted({item["target_tid"] for item in matches})))
        if not keep:
            continue
        rows.append(
            {
                "timestamp": event["time"],
                "cpu": event["cpu"],
                "event": name,
                "prev_tid": prev_tid,
                "prev_comm": prev_comm,
                "prev_state": prev_state,
                "next_tid": next_tid,
                "next_comm": next_comm,
                "target_tid": target_tid,
                "waker_tid": waker_tid,
                "orig_cpu": orig_cpu,
                "dest_cpu": dest_cpu,
                "selection_reason": reason,
                "raw_event": event["raw_event"],
            }
        )
    return rows


def frame_role(frame: int) -> str:
    return "anomaly" if ANOMALY_FIRST <= frame <= ANOMALY_LAST else "candidate_control"


def main() -> int:
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    raw_paths = [
        PERF_DIR / "perf.data",
        PERF_DIR / "perf_cpu_stack.data",
        PERF_DIR / "perf_sched_script.txt",
        PERF_DIR / "perf_cpu_stack_script.txt",
    ]
    raw_before = {str(path): raw_stat(path) for path in raw_paths}

    reusable = load_reusable_module()
    requested_frames = list(range(FRAME_FIRST, FRAME_LAST + 1))
    windows, window_warnings = reusable.build_windows(
        CENTERPOINT_CSV,
        PERCEPTION_LOG,
        requested_frames,
        SCHED_MINUS_MONOTONIC_NS,
        2.0,
    )
    by_frame = {window.source_frame_index: window for window in windows}
    all_logged_tids = {window.target_tid for window in windows}
    tid_map = validate_tid_mapping(all_logged_tids)

    identity_rows = []
    for frame in requested_frames:
        window = by_frame.get(frame)
        if window is None:
            identity_rows.append(
                {
                    "frame": frame,
                    "infer_enter_monotonic_ns": "",
                    "infer_exit_monotonic_ns": "",
                    "infer_ms": "",
                    "logged_tid": "",
                    "logged_tid_namespace": "unavailable",
                    "host_tid": "",
                    "tid_mapping_evidence": "unavailable: no unique CP_INFER ENTER/EXIT pair mapped to this source frame",
                }
            )
            continue
        mapping = tid_map[window.target_tid]
        identity_rows.append(
            {
                "frame": frame,
                "infer_enter_monotonic_ns": window.enter_apollo_mono_ns,
                "infer_exit_monotonic_ns": window.exit_apollo_mono_ns,
                "infer_ms": fmt(window.cp_duration_ms, 6),
                "logged_tid": window.target_tid,
                "logged_tid_namespace": mapping["logged_tid_namespace"],
                "host_tid": mapping["host_tid"] or "",
                "tid_mapping_evidence": mapping["evidence"],
            }
        )
    identity_fields = [
        "frame", "infer_enter_monotonic_ns", "infer_exit_monotonic_ns", "infer_ms",
        "logged_tid", "logged_tid_namespace", "host_tid", "tid_mapping_evidence",
    ]
    write_csv(ANALYSIS_DIR / "p4_frame_identity.csv", identity_rows, identity_fields)

    stack_samples = parse_stack_samples(PERF_DIR / "perf_cpu_stack_script.txt")
    samples_by_tid: dict[int, list[dict]] = defaultdict(list)
    for sample in stack_samples:
        samples_by_tid[sample["tid"]].append(sample)
    for value in samples_by_tid.values():
        value.sort(key=lambda x: x["time_ns"])

    stack_frame_rows = []
    symbol_rows = []
    callchain_rows = []
    selected_samples_by_frame: dict[int, list[dict]] = {}
    for frame in requested_frames:
        window = by_frame.get(frame)
        mapping = tid_map.get(window.target_tid) if window else None
        host_tid = mapping["host_tid"] if mapping and mapping["valid"] else None
        if window is None or host_tid is None:
            stack_frame_rows.append(
                {
                    "frame": frame,
                    "host_tid": "",
                    "infer_ms": fmt(window.cp_duration_ms, 6) if window else "",
                    "stack_sample_count": "",
                    "resolved_sample_count": "",
                    "unknown_sample_count": "",
                    "unknown_ratio": "",
                    "top1_symbol": "", "top1_dso": "", "top1_samples": "", "top1_ratio": "",
                    "top2_symbol": "", "top2_dso": "", "top2_samples": "", "top2_ratio": "",
                    "top3_symbol": "", "top3_dso": "", "top3_samples": "", "top3_ratio": "",
                    "frame_role": frame_role(frame),
                    "data_status": "TID_MAPPING_UNAVAILABLE" if window else "NO_MAPPED_INFER_INSTANCE",
                    "quality_flag": "explicit_unavailable_no_guess",
                }
            )
            selected_samples_by_frame[frame] = []
            continue
        candidates = samples_by_tid.get(host_tid, [])
        times = [sample["time_ns"] for sample in candidates]
        left = bisect.bisect_left(times, window.enter_apollo_mono_ns)
        right = bisect.bisect_right(times, window.exit_apollo_mono_ns)
        selected = candidates[left:right]
        selected_samples_by_frame[frame] = selected
        leaves = Counter(normalized_leaf(sample) for sample in selected)
        unknown = sum(symbol in {"[unknown]", "[no_callchain]"} for symbol, _dso in map(normalized_leaf, selected))
        resolved = len(selected) - unknown
        unknown_ratio = unknown / len(selected) if selected else None
        top = sorted(leaves.items(), key=lambda x: (-x[1], x[0][0], x[0][1]))[:3]
        row = {
            "frame": frame,
            "host_tid": host_tid,
            "infer_ms": fmt(window.cp_duration_ms, 6),
            "stack_sample_count": len(selected),
            "resolved_sample_count": resolved,
            "unknown_sample_count": unknown,
            "unknown_ratio": fmt(unknown_ratio, 9),
            "frame_role": frame_role(frame),
            "data_status": "OK" if selected else "ZERO_STACK_SAMPLES",
            "quality_flag": (
                "zero_stack_samples"
                if not selected
                else "high_unknown_ratio" if unknown_ratio is not None and unknown_ratio >= UNKNOWN_RATIO_HIGH_THRESHOLD
                else "none"
            ),
        }
        for rank in range(1, 4):
            if rank <= len(top):
                (symbol, dso), count = top[rank - 1]
                row[f"top{rank}_symbol"] = symbol
                row[f"top{rank}_dso"] = dso
                row[f"top{rank}_samples"] = count
                row[f"top{rank}_ratio"] = fmt(count / len(selected), 9)
            else:
                row[f"top{rank}_symbol"] = ""
                row[f"top{rank}_dso"] = ""
                row[f"top{rank}_samples"] = ""
                row[f"top{rank}_ratio"] = ""
        stack_frame_rows.append(row)
        for (symbol, dso), count in sorted(leaves.items(), key=lambda x: (-x[1], x[0][0], x[0][1])):
            symbol_rows.append(
                {
                    "frame": frame,
                    "host_tid": host_tid,
                    "symbol": symbol,
                    "dso": dso,
                    "sample_count": count,
                    "sample_ratio": fmt(count / len(selected), 9),
                }
            )
        chains = Counter(folded_callchain(sample) for sample in selected)
        for chain, count in sorted(chains.items(), key=lambda x: (-x[1], x[0]))[:10]:
            callchain_rows.append(
                {
                    "frame": frame,
                    "callchain": chain,
                    "sample_count": count,
                    "sample_ratio": fmt(count / len(selected), 9),
                }
            )

    stack_fields = [
        "frame", "host_tid", "infer_ms", "stack_sample_count", "resolved_sample_count",
        "unknown_sample_count", "unknown_ratio", "top1_symbol", "top1_dso", "top1_samples",
        "top1_ratio", "top2_symbol", "top2_dso", "top2_samples", "top2_ratio",
        "top3_symbol", "top3_dso", "top3_samples", "top3_ratio",
        "frame_role", "data_status", "quality_flag",
    ]
    write_csv(ANALYSIS_DIR / "p4_cpu_stack_frame_summary.csv", stack_frame_rows, stack_fields)
    write_csv(
        ANALYSIS_DIR / "p4_cpu_stack_symbols_long.csv",
        symbol_rows,
        ["frame", "host_tid", "symbol", "dso", "sample_count", "sample_ratio"],
    )
    write_csv(
        ANALYSIS_DIR / "p4_cpu_stack_callchains.csv",
        callchain_rows,
        ["frame", "callchain", "sample_count", "sample_ratio"],
    )

    raw_anomaly = ANALYSIS_DIR / "cpu_stack_samples_anomaly.txt"
    with raw_anomaly.open("w", encoding="utf-8", newline="") as handle:
        for frame in range(ANOMALY_FIRST, ANOMALY_LAST + 1):
            window = by_frame.get(frame)
            mapping = tid_map.get(window.target_tid) if window else None
            host_tid = mapping["host_tid"] if mapping and mapping["valid"] else None
            enter = window.enter_apollo_mono_ns if window else "UNAVAILABLE"
            exit_value = window.exit_apollo_mono_ns if window else "UNAVAILABLE"
            handle.write(
                f"===== FRAME F{frame} / HOST_TID {host_tid if host_tid is not None else 'UNAVAILABLE'} "
                f"/ ENTER {enter} / EXIT {exit_value} =====\n"
            )
            samples = selected_samples_by_frame.get(frame, [])
            if window is None or host_tid is None:
                handle.write("NO_MAPPED_INFER_INSTANCE_OR_UNVERIFIED_HOST_TID\n\n")
            elif not samples:
                handle.write("NO_STACK_SAMPLES_IN_EXACT_INFER_WINDOW\n\n")
            else:
                for sample in samples:
                    handle.write(sample["raw"])
                    if not sample["raw"].endswith("\n"):
                        handle.write("\n")
                    handle.write("\n")

    valid_windows = [
        window for window in windows if tid_map[window.target_tid]["valid"]
    ]
    sched_scan_start = min(window.enter_perf_s for window in valid_windows) - 2.0
    sched_scan_end = max(window.exit_perf_s for window in valid_windows) + 1.0
    sched = scan_scheduler(
        PERF_DIR / "perf_sched_script.txt",
        {tid_map[window.target_tid]["host_tid"] for window in valid_windows},
        sched_scan_start,
        sched_scan_end,
    )

    sched_rows = []
    sched_metrics_by_frame = {}
    for frame in requested_frames:
        window = by_frame.get(frame)
        mapping = tid_map.get(window.target_tid) if window else None
        host_tid = mapping["host_tid"] if mapping and mapping["valid"] else None
        if window is None or host_tid is None:
            sched_rows.append(
                {
                    "frame": frame, "host_tid": "", "infer_ms": fmt(window.cp_duration_ms, 6) if window else "",
                    "running_ms": "", "running_ratio": "", "sleep_block_ms": "", "sleep_block_ratio": "",
                    "runnable_wait_ms": "", "runnable_wait_ratio": "", "max_sched_delay_ms": "",
                    "context_switches": "", "migration_count": "", "cpu_list": "", "kswapd0_cpu_ms": "",
                    "kswapd0_overlap_running_ms": "", "kswapd0_overlap_runnable_ms": "",
                    "top_competitor_tid": "", "top_competitor_comm": "", "top_competitor_cpu_ms": "",
                    "frame_role": frame_role(frame),
                    "data_status": "TID_MAPPING_UNAVAILABLE" if window else "NO_MAPPED_INFER_INSTANCE",
                    "unclassified_sched_ms": "",
                }
            )
            continue
        metrics = classify_target_window(host_tid, window.enter_perf_s, window.exit_perf_s, sched)
        sched_metrics_by_frame[frame] = metrics
        duration_s = window.cp_duration_ms / 1000.0
        sched_rows.append(
            {
                "frame": frame,
                "host_tid": host_tid,
                "infer_ms": fmt(window.cp_duration_ms, 6),
                "running_ms": fmt(metrics["running_s"] * 1000),
                "running_ratio": fmt(metrics["running_s"] / duration_s, 9),
                "sleep_block_ms": fmt(metrics["blocked_s"] * 1000),
                "sleep_block_ratio": fmt(metrics["blocked_s"] / duration_s, 9),
                "runnable_wait_ms": fmt(metrics["runnable_s"] * 1000),
                "runnable_wait_ratio": fmt(metrics["runnable_s"] / duration_s, 9),
                "max_sched_delay_ms": fmt(metrics["max_sched_delay_s"] * 1000),
                "context_switches": metrics["context_switches"],
                "migration_count": metrics["migration_count"],
                "cpu_list": metrics["cpu_list"],
                "kswapd0_cpu_ms": fmt(metrics["kswapd_cpu_s"] * 1000),
                "kswapd0_overlap_running_ms": fmt(metrics["kswapd_overlap_running_s"] * 1000),
                "kswapd0_overlap_runnable_ms": fmt(metrics["kswapd_overlap_runnable_s"] * 1000),
                "top_competitor_tid": metrics["top_competitor_tid"] or "",
                "top_competitor_comm": metrics["top_competitor_comm"],
                "top_competitor_cpu_ms": fmt(metrics["top_competitor_cpu_s"] * 1000),
                "frame_role": frame_role(frame),
                "data_status": "OK",
                "unclassified_sched_ms": fmt(metrics["unknown_s"] * 1000),
            }
        )
    sched_fields = [
        "frame", "host_tid", "infer_ms", "running_ms", "running_ratio", "sleep_block_ms",
        "sleep_block_ratio", "runnable_wait_ms", "runnable_wait_ratio", "max_sched_delay_ms",
        "context_switches", "migration_count", "cpu_list", "kswapd0_cpu_ms",
        "kswapd0_overlap_running_ms", "kswapd0_overlap_runnable_ms", "top_competitor_tid",
        "top_competitor_comm", "top_competitor_cpu_ms", "frame_role", "data_status",
        "unclassified_sched_ms",
    ]
    write_csv(ANALYSIS_DIR / "p4_sched_frame_summary.csv", sched_rows, sched_fields)

    anomaly_windows = [
        by_frame[frame] for frame in range(ANOMALY_FIRST, ANOMALY_LAST + 1)
        if frame in by_frame and tid_map[by_frame[frame].target_tid]["valid"]
    ]
    anomaly_tids = {tid_map[window.target_tid]["host_tid"] for window in anomaly_windows}
    context_start = by_frame[ANOMALY_FIRST].enter_perf_s - 1.0
    context_end = by_frame[ANOMALY_LAST].exit_perf_s + 1.0
    anomaly_cpus = set()
    for frame in range(ANOMALY_FIRST, ANOMALY_LAST + 1):
        metrics = sched_metrics_by_frame.get(frame)
        if metrics and metrics["cpu_list"]:
            anomaly_cpus.update(map(int, metrics["cpu_list"].split(";")))
    # "Direct competitor" is deliberately restricted to a task whose actual
    # same-CPU run interval intersects an anomaly target's classified runnable
    # wait.  Immediate neighbors seen while the target was sleeping are not
    # propagated into the context set.
    context_competitor_links: dict[int, set[int]] = defaultdict(set)
    context_competition_windows: list[dict] = []
    for frame in range(ANOMALY_FIRST, ANOMALY_LAST + 1):
        window = by_frame.get(frame)
        metrics = sched_metrics_by_frame.get(frame)
        if window is None or metrics is None:
            continue
        target_tid = tid_map[window.target_tid]["host_tid"]
        for item in sched["direct_intervals"]:
            if item["target_tid"] != target_tid:
                continue
            for runnable_start, runnable_end in metrics["runnable_intervals"]:
                left = max(item["start"], runnable_start, context_start)
                right = min(item["end"], runnable_end, context_end)
                if right > left:
                    context_competitor_links[item["competitor_tid"]].add(target_tid)
                    context_competition_windows.append(
                        {
                            "target_tid": target_tid,
                            "competitor_tid": item["competitor_tid"],
                            "competitor_comm": item["competitor_comm"],
                            "cpu": item["cpu"],
                            "start": left,
                            "end": right,
                        }
                    )
    context_rows = extract_context_events(
        PERF_DIR / "perf_sched_script.txt",
        context_start,
        context_end,
        anomaly_tids,
        anomaly_cpus,
        sched["kswapd_tids"],
        context_competition_windows,
    )
    context_fields = [
        "timestamp", "cpu", "event", "prev_tid", "prev_comm", "prev_state",
        "next_tid", "next_comm", "target_tid", "waker_tid", "orig_cpu", "dest_cpu",
        "selection_reason", "raw_event",
    ]
    write_csv(ANALYSIS_DIR / "p4_sched_events_context.csv", context_rows, context_fields)

    metadata_files = [
        "perf_cpu_stack_record_command.txt",
        "capture_start_clock.txt",
        "capture_end_clock.txt",
        "cpu_stack_report_status.txt",
        "perf_cpu_stack_record.stderr.txt",
    ]
    for name in metadata_files:
        shutil.copy2(PERF_DIR / name, ANALYSIS_DIR / name)

    start_clock = dict(
        line.split("=", 1)
        for line in (PERF_DIR / "capture_start_clock.txt").read_text(encoding="utf-8").splitlines()
        if "=" in line
    )
    end_clock = dict(
        line.split("=", 1)
        for line in (PERF_DIR / "capture_end_clock.txt").read_text(encoding="utf-8").splitlines()
        if "=" in line
    )
    stack_command = (PERF_DIR / "perf_cpu_stack_record_command.txt").read_text(encoding="utf-8").strip()
    calibration = json.loads(CALIBRATION_JSON.read_text(encoding="utf-8"))
    mapped_frames = sorted(by_frame)
    unmapped_frames = sorted(set(requested_frames) - set(mapped_frames))
    all_mapping_valid = all(tid_map[tid]["valid"] for tid in all_logged_tids)
    stack_first_ns = min(sample["time_ns"] for sample in stack_samples)
    stack_last_ns = max(sample["time_ns"] for sample in stack_samples)
    min_enter_ns = min(window.enter_apollo_mono_ns for window in valid_windows)
    max_exit_ns = max(window.exit_apollo_mono_ns for window in valid_windows)
    capture_start_ns = int(start_clock["monotonic_ns"])
    capture_end_ns = int(end_clock["monotonic_ns"])
    stack_domain_valid = (
        "--clockid CLOCK_MONOTONIC" in stack_command
        and capture_start_ns <= min_enter_ns <= max_exit_ns <= capture_end_ns
        and stack_first_ns <= min_enter_ns <= max_exit_ns <= stack_last_ns
        and calibration["best_sched_minus_stack_offset_ns"] == SCHED_MINUS_MONOTONIC_NS
        and calibration["support_pct"] >= 95.0
    )
    zero_stack_frames = [
        row["frame"] for row in stack_frame_rows if row.get("data_status") == "ZERO_STACK_SAMPLES"
    ]
    high_unknown_frames = [
        row["frame"] for row in stack_frame_rows if row.get("quality_flag") == "high_unknown_ratio"
    ]
    arithmetic_errors = []
    for row in sched_rows:
        if row["data_status"] != "OK":
            continue
        total = sum(float(row[key]) for key in ["running_ms", "sleep_block_ms", "runnable_wait_ms", "unclassified_sched_ms"])
        if abs(total - float(row["infer_ms"])) > 0.02:
            arithmetic_errors.append({"frame": row["frame"], "difference_ms": total - float(row["infer_ms"])})

    raw_after = {str(path): raw_stat(path) for path in raw_paths}
    raw_unchanged = raw_before == raw_after
    validation = {
        "analysis_scope": {
            "frames": [FRAME_FIRST, FRAME_LAST],
            "anomaly_frames": [ANOMALY_FIRST, ANOMALY_LAST],
            "candidate_control_frames": [[FRAME_FIRST, ANOMALY_FIRST - 1], [ANOMALY_LAST + 1, FRAME_LAST]],
            "candidate_control_note": "candidate control only; not asserted normal",
        },
        "source_files": {
            "perception_log": str(PERCEPTION_LOG),
            "centerpoint_csv": str(CENTERPOINT_CSV),
            "perf_sched_script": str(PERF_DIR / "perf_sched_script.txt"),
            "perf_cpu_stack_script": str(PERF_DIR / "perf_cpu_stack_script.txt"),
        },
        "frame_identity": {
            "requested_frame_count": len(requested_frames),
            "mapped_frame_count": len(mapped_frames),
            "mapped_frames": mapped_frames,
            "unmapped_frames": unmapped_frames,
            "all_observed_tid_mappings_valid": all_mapping_valid,
            "tid_mapping_details": tid_map,
            "window_warnings": window_warnings,
        },
        "clock_validation": {
            "cpu_stack_and_infer_monotonic_alignment_status": "validated" if stack_domain_valid else "not_validated",
            "cpu_stack_command_uses_clock_monotonic": "--clockid CLOCK_MONOTONIC" in stack_command,
            "capture_monotonic_ns": [capture_start_ns, capture_end_ns],
            "requested_mapped_infer_ns": [min_enter_ns, max_exit_ns],
            "cpu_stack_script_sample_ns": [stack_first_ns, stack_last_ns],
            "sched_minus_infer_monotonic_offset_ns": SCHED_MINUS_MONOTONIC_NS,
            "sched_offset_calibration": calibration,
        },
        "stack_quality": {
            "parsed_total_samples": len(stack_samples),
            "zero_stack_sample_frames": zero_stack_frames,
            "unknown_ratio_high_threshold": UNKNOWN_RATIO_HIGH_THRESHOLD,
            "high_unknown_ratio_frames": high_unknown_frames,
            "unknown_definition": "leaf instruction symbol is [unknown] or no_callchain",
            "symbol_statistic_definition": "one normalized leaf instruction symbol+DSO per CPU-clock sample",
            "sample_ratio_caveat": "sampling composition only; not an exact wall-time ratio",
        },
        "scheduler_quality": {
            "parsed_internal_range_s": [sched_scan_start, sched_scan_end],
            "parsed_event_coverage_s": [sched["first_event_s"], sched["last_event_s"]],
            "context_output_range_s": [context_start, context_end],
            "context_event_count": len(context_rows),
            "anomaly_actual_host_tids": sorted(anomaly_tids),
            "anomaly_target_cpus": sorted(anomaly_cpus),
            "kswapd0_tids_seen": sorted(sched["kswapd_tids"]),
            "direct_competitor_tids": sorted(context_competitor_links),
            "direct_competition_window_count": len(context_competition_windows),
            "context_switch_definition": "target TID sched_switch records where it is prev_tid",
            "top_competitor_definition": "actual CPU-resident time of the immediate same-CPU predecessor/successor task, clipped to target runnable-wait intervals",
            "running_caveat": "CPU-resident only; not asserted to be effective computation",
            "arithmetic_tolerance_ms": 0.02,
            "arithmetic_errors": arithmetic_errors,
        },
        "immutability_check": {
            "raw_files_unchanged_during_generation": raw_unchanged,
            "before": raw_before,
            "after": raw_after,
        },
    }
    (ANALYSIS_DIR / "analysis_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if not all_mapping_valid:
        raise RuntimeError("one or more observed logged TIDs lack validated Host TID mapping")
    if not stack_domain_valid:
        raise RuntimeError("CPU stack and Infer monotonic time domain was not validated")
    if arithmetic_errors:
        raise RuntimeError(f"scheduler accounting arithmetic errors: {arithmetic_errors}")
    if not raw_unchanged:
        raise RuntimeError("raw perf source metadata changed during generation")

    manifest_rows = []
    for path in sorted(ANALYSIS_DIR.iterdir(), key=lambda p: p.name):
        if path.is_file() and path.name != "analysis_manifest.csv":
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            manifest_rows.append(
                {"file": path.name, "size_bytes": path.stat().st_size, "sha256": digest}
            )
    write_csv(
        ANALYSIS_DIR / "analysis_manifest.csv",
        manifest_rows,
        ["file", "size_bytes", "sha256"],
    )
    print(json.dumps({
        "analysis_dir": str(ANALYSIS_DIR),
        "requested_frames": len(requested_frames),
        "mapped_frames": len(mapped_frames),
        "unmapped_frames": unmapped_frames,
        "stack_samples": len(stack_samples),
        "context_events": len(context_rows),
        "raw_unchanged": raw_unchanged,
        "stack_time_domain_validated": stack_domain_valid,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
