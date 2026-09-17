#!/usr/bin/env python3
"""Build a same-observed-path normal versus P99 Prediction system baseline."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


ANALYSIS_ROOT = Path(__file__).resolve().parent
RUN_ROOT = ANALYSIS_ROOT.parents[2]
SYSTEM_ROOT = RUN_ROOT / "perf和stack数据分析" / "系统层实时性分析_20260915" / "prediction"
OUTPUT_DIR = SYSTEM_ROOT / "same_path_p99_system_baseline"

JOBS = ANALYSIS_ROOT / "jobs.csv"
CONTROLS = ANALYSIS_ROOT / "normal_controls.csv"
NODES = ANALYSIS_ROOT / "timings" / "prediction_node_instances.csv"
FRAME_STATES = SYSTEM_ROOT / "sched_and_stack" / "frame_summary.csv"
STAGE_STATES = SYSTEM_ROOT / "stage_thread_states.csv"

PATH_FIELDS = (
    "semantic_base_async_schedule_calls",
    "semantic_base_async_submit_calls",
    "cyber_taskmanager_notify_tasks_calls",
    "semantic_base_async_draw_calls",
    "async_completion_status",
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def number(value: str) -> float:
    return float(value)


def fmt(value: float) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.9f}".rstrip("0").rstrip(".")


def quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(values: list[float]) -> dict[str, object]:
    return {
        "n": len(values),
        "mean": fmt(sum(values) / len(values)),
        "median": fmt(quantile(values, 0.50)),
        "p25": fmt(quantile(values, 0.25)),
        "p75": fmt(quantile(values, 0.75)),
        "p95": fmt(quantile(values, 0.95)),
        "min": fmt(min(values)),
        "max": fmt(max(values)),
    }


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    jobs = {int(row["frame_index"]): row for row in read_csv(JOBS)}
    controls = read_csv(CONTROLS)
    frame_states = {int(row["frame_index"]): row for row in read_csv(FRAME_STATES)}

    node_rows: dict[tuple[int, str], list[dict[str, str]]] = defaultdict(list)
    for row in read_csv(NODES):
        node_rows[(int(row["frame_index"]), row["node"])].append(row)

    stage_rows: dict[tuple[int, str], list[dict[str, str]]] = defaultdict(list)
    for row in read_csv(STAGE_STATES):
        stage_rows[(int(row["frame_index"]), row["node"])].append(row)

    p99_frames = sorted(
        frame for frame, row in jobs.items() if row["execution_ms_p99_tail"].lower() == "true"
    )
    normal_frames = sorted(
        {
            int(row["control_frame"])
            for row in controls
            if int(row["case_frame"]) in p99_frames
        }
    )
    selected = [(frame, "normal") for frame in normal_frames] + [
        (frame, "p99_abnormal") for frame in p99_frames
    ]

    missing_states = [frame for frame, _ in selected if frame not in frame_states]
    if missing_states:
        raise RuntimeError(f"Selected frames missing callback state data: {missing_states}")

    signatures = {
        tuple(jobs[frame][field] for field in PATH_FIELDS) for frame, _ in selected
    }
    if len(signatures) != 1:
        raise RuntimeError(f"Selected frames do not share one observed path signature: {signatures}")

    selection_rows: list[dict[str, object]] = []
    callback_rows: list[dict[str, object]] = []
    notify_rows: list[dict[str, object]] = []
    closure_errors: list[dict[str, object]] = []
    concentration_by_role: dict[str, list[float]] = defaultdict(list)

    for frame, role in selected:
        job = jobs[frame]
        draws = node_rows[(frame, "semantic_base_async_draw")]
        notify_parents = stage_rows[(frame, "cyber_taskmanager_notify_tasks")]
        notify_children = stage_rows[(frame, "cyber_taskmanager_notify_task")]
        if len(draws) != 1 or len(notify_parents) != 1 or len(notify_children) != 16:
            raise RuntimeError(
                f"Frame {frame} has unexpected async or NotifyTask cardinality: "
                f"draw={len(draws)}, parents={len(notify_parents)}, children={len(notify_children)}"
            )

        draw = draws[0]
        proc_enter_ns = int(job["prediction_enter_ns"])
        proc_exit_ns = int(job["prediction_exit_ns"])
        draw_enter_ns = int(draw["enter_ns"])
        draw_exit_ns = int(draw["exit_ns"])
        overlap_ms = max(
            0.0,
            (min(proc_exit_ns, draw_exit_ns) - max(proc_enter_ns, draw_enter_ns)) / 1_000_000.0,
        )
        selection_rows.append(
            {
                "frame_index": frame,
                "role": role,
                "execution_ms": job["execution_ms"],
                "async_schedule_calls": job["semantic_base_async_schedule_calls"],
                "async_submit_calls": job["semantic_base_async_submit_calls"],
                "notify_tasks_calls": job["cyber_taskmanager_notify_tasks_calls"],
                "async_draw_calls": job["semantic_base_async_draw_calls"],
                "proc_enter_ns": proc_enter_ns,
                "proc_exit_ns": proc_exit_ns,
                "async_draw_enter_ns": draw_enter_ns,
                "async_draw_exit_ns": draw_exit_ns,
                "async_overlap_ms": fmt(overlap_ms),
                "async_completion_status": job["async_completion_status"],
            }
        )

        state = frame_states[frame]
        execution_ms = number(state["execution_ms"])
        running_ms = number(state["callback_running_ms"])
        runnable_ms = number(state["callback_runnable_ms"])
        blocked_ms = number(state["callback_blocked_ms"])
        unknown_ms = number(state["unknown_ms"])
        waiting_ms = runnable_ms + blocked_ms
        callback_rows.append(
            {
                "frame_index": frame,
                "role": role,
                "execution_ms": fmt(execution_ms),
                "running_ms": fmt(running_ms),
                "runnable_ms": fmt(runnable_ms),
                "blocked_ms": fmt(blocked_ms),
                "waiting_ms": fmt(waiting_ms),
                "waiting_share": fmt(waiting_ms / execution_ms),
                "unknown_ms": fmt(unknown_ms),
            }
        )
        closure_error = abs(execution_ms - running_ms - runnable_ms - blocked_ms - unknown_ms)
        if closure_error > 0.001:
            closure_errors.append({"frame_index": frame, "callback_error_ms": closure_error})

        parent = notify_parents[0]
        longest = max(notify_children, key=lambda row: number(row["duration_ms"]))
        longest_duration = number(longest["duration_ms"])
        longest_running = number(longest["running_ms"])
        longest_runnable = number(longest["runnable_ms"])
        longest_blocked = number(longest["blocked_ms"])
        longest_unknown = number(longest["unknown_ms"])
        longest_waiting = longest_runnable + longest_blocked
        if waiting_ms > 0:
            concentration_by_role[role].append(longest_waiting / waiting_ms)
        notify_rows.append(
            {
                "frame_index": frame,
                "role": role,
                "notify_tasks_duration_ms": fmt(number(parent["duration_ms"])),
                "longest_notifytask_duration_ms": fmt(longest_duration),
                "longest_notifytask_running_ms": fmt(longest_running),
                "longest_notifytask_runnable_ms": fmt(longest_runnable),
                "longest_notifytask_blocked_ms": fmt(longest_blocked),
                "longest_notifytask_blocked_share": fmt(longest_blocked / longest_duration),
            }
        )
        notify_closure_error = abs(
            longest_duration
            - longest_running
            - longest_runnable
            - longest_blocked
            - longest_unknown
        )
        if notify_closure_error > 0.001:
            closure_errors.append({"frame_index": frame, "notify_error_ms": notify_closure_error})

    selection_fields = [
        "frame_index", "role", "execution_ms", "async_schedule_calls", "async_submit_calls",
        "notify_tasks_calls", "async_draw_calls", "proc_enter_ns", "proc_exit_ns",
        "async_draw_enter_ns", "async_draw_exit_ns", "async_overlap_ms",
        "async_completion_status",
    ]
    callback_fields = [
        "frame_index", "role", "execution_ms", "running_ms", "runnable_ms", "blocked_ms",
        "waiting_ms", "waiting_share", "unknown_ms",
    ]
    notify_fields = [
        "frame_index", "role", "notify_tasks_duration_ms", "longest_notifytask_duration_ms",
        "longest_notifytask_running_ms", "longest_notifytask_runnable_ms",
        "longest_notifytask_blocked_ms", "longest_notifytask_blocked_share",
    ]

    write_csv(OUTPUT_DIR / "same_path_frame_selection.csv", selection_fields, selection_rows)
    write_csv(OUTPUT_DIR / "same_path_callback_system_states.csv", callback_fields, callback_rows)
    write_csv(OUTPUT_DIR / "same_path_notifytask_system_states.csv", notify_fields, notify_rows)

    summary_source: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for row in callback_rows:
        cohort = "same_path_normal" if row["role"] == "normal" else "p99_abnormal"
        for metric in callback_fields[2:]:
            summary_source[cohort][metric].append(float(row[metric]))
    for row in notify_rows:
        cohort = "same_path_normal" if row["role"] == "normal" else "p99_abnormal"
        for metric in notify_fields[2:]:
            summary_source[cohort][metric].append(float(row[metric]))

    summary_rows: list[dict[str, object]] = []
    for cohort in ("same_path_normal", "p99_abnormal"):
        for metric, values in summary_source[cohort].items():
            summary_rows.append({"role": cohort, "metric": metric, **summarize(values)})
    write_csv(
        OUTPUT_DIR / "same_path_cohort_summary.csv",
        ["role", "metric", "n", "mean", "median", "p25", "p75", "p95", "min", "max"],
        summary_rows,
    )

    path_signature = list(next(iter(signatures)))
    callback_by_role = {
        role: {row["metric"]: row for row in summary_rows if row["role"] == role}
        for role in ("same_path_normal", "p99_abnormal")
    }
    validation = {
        "run_id": "202609111649",
        "selection": {
            "normal_count": len(normal_frames),
            "p99_abnormal_count": len(p99_frames),
            "normal_frames": normal_frames,
            "p99_abnormal_frames": p99_frames,
            "normal_source": "unique normal_controls.csv controls attached to strict execution P99 frames",
        },
        "same_observed_path": {
            "verified": len(signatures) == 1,
            "fields": list(PATH_FIELDS),
            "signature": path_signature,
            "async_draw_instance_count_per_frame": 1,
            "notify_task_child_count_per_frame": 16,
        },
        "state_accounting": {
            "closure_tolerance_ms": 0.001,
            "verified": not closure_errors,
            "errors": closure_errors,
            "waiting_definition": "runnable_ms + blocked_ms",
            "waiting_share_definition": "waiting_ms / execution_ms",
        },
        "cohort_medians": {
            role: {
                metric: float(callback_by_role[role][metric]["median"])
                for metric in ("execution_ms", "running_ms", "runnable_ms", "blocked_ms", "waiting_ms", "waiting_share")
            }
            for role in ("same_path_normal", "p99_abnormal")
        },
        "longest_notifytask_waiting_over_callback_waiting": {
            role: summarize(values) for role, values in concentration_by_role.items()
        },
        "interpretation_limit": (
            "The tables locate scheduler-observed waiting in the longest NotifyTask interval. "
            "They do not identify a lock, GPU operation, device, or other concrete wait object."
        ),
    }
    (OUTPUT_DIR / "same_path_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
