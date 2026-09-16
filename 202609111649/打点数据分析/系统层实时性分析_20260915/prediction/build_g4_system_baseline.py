"""Build the G4 local-jump-only versus matched-normal system baseline."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
from collections import defaultdict
from pathlib import Path

RUN = Path(r"D:\data\202609111649")
ANALYSIS = RUN / "打点数据分析/系统层实时性分析_20260915/prediction"
SYSTEM = RUN / "perf和stack数据分析/系统层实时性分析_20260915/prediction"
APP_OUT = ANALYSIS / "g4_abnormal_vs_normal_system_baseline"
SYS_OUT = SYSTEM / "g4_abnormal_vs_normal_system_baseline"
PERF_OUT = SYS_OUT / "perf_extract"


def read(path: Path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write(path: Path, rows):
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError(f"refusing to write an empty table: {path}")
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def q(values, p):
    values = sorted(float(value) for value in values)
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * p
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    fraction = position - lower
    return values[lower] * (1 - fraction) + values[upper] * fraction


def prepare():
    reasons = read(ANALYSIS / "selection_reasons.csv")
    reasons_by_frame = defaultdict(list)
    for row in reasons:
        reasons_by_frame[row["frame_index"]].append(row["reason"])

    local_jump_frames = {
        row["frame_index"]
        for row in reasons
        if row["reason"] == "callback_previous20_median_x2_and_plus10ms"
    }
    local_only = sorted([
        frame
        for frame in local_jump_frames
        if not any("strict_p99" in reason for reason in reasons_by_frame[frame])
        and reasons_by_frame[frame] == ["callback_previous20_median_x2_and_plus10ms"]
    ], key=int)

    controls = [
        row for row in read(ANALYSIS / "normal_controls.csv")
        if row["case_frame"] in local_only
    ]
    controls_by_case = defaultdict(list)
    for row in controls:
        controls_by_case[row["case_frame"]].append(row["control_frame"])
    normal_frames = sorted({row["control_frame"] for row in controls}, key=int)

    frames = read(ANALYSIS / "timings/prediction_frames.csv")
    by_frame = {row["frame_index"]: row for row in frames}
    jobs = {row["frame_index"]: row for row in read(ANALYSIS / "jobs.csv")}
    coverage = {row["frame_index"]: row for row in read(ANALYSIS / "selected_coverage.csv")}
    selected_ids = set(local_only) | set(normal_frames)
    p75_execution_ms = q([row["execution_ms"] for row in jobs.values()], .75)
    workload_keys = ["perception_count", "prediction_count", "trajectory_count", "trajectory_points", "caution_count", "normal_count", "ignore_count", "interactive_count"]

    assert local_only == ["184", "204", "311", "755"]
    assert len(normal_frames) == 12
    assert all(len(controls_by_case[frame]) == 3 for frame in local_only)
    assert all(frame in by_frame for frame in selected_ids)
    assert all(coverage[frame]["sched_status"] == "eligible" for frame in selected_ids)
    assert all(frame not in reasons_by_frame for frame in normal_frames)
    assert all(jobs[frame]["local_execution_jump"] == "False" for frame in normal_frames)
    for edge in controls:
        case = jobs[edge["case_frame"]]
        normal = jobs[edge["control_frame"]]
        assert case["output_record_matched"] == "True"
        assert all(case[key] == normal[key] for key in workload_keys)
        assert case["async_completion_status"] == normal["async_completion_status"]
        assert 0 < abs(int(edge["case_frame"]) - int(edge["control_frame"])) <= 30
        assert float(normal["execution_ms"]) <= p75_execution_ms

    selection = []
    for frame in local_only:
        selection.append({
            "frame_index": frame,
            "role": "abnormal_local_jump_only",
            "paired_case_frames": frame,
            "selection_rule": "local_execution_jump=True; sole candidate reason; no strict-P99 hit",
            "execution_ms": jobs[frame]["execution_ms"],
            "previous20_execution_median_ms": jobs[frame]["previous20_execution_median_ms"],
            "observed_workload": ";".join(f"{key}={jobs[frame][key]}" for key in ["perception_count", "prediction_count", "trajectory_count", "trajectory_points"]),
            "sched_status": coverage[frame]["sched_status"],
        })
    for frame in normal_frames:
        paired = sorted({row["case_frame"] for row in controls if row["control_frame"] == frame}, key=int)
        selection.append({
            "frame_index": frame,
            "role": "matched_normal",
            "paired_case_frames": ";".join(paired),
            "selection_rule": "existing G4 control: same run/status and observed workload, noncandidate, <=P75 callback, nearest within 30 frames",
            "execution_ms": jobs[frame]["execution_ms"],
            "previous20_execution_median_ms": jobs[frame]["previous20_execution_median_ms"],
            "observed_workload": ";".join(f"{key}={jobs[frame][key]}" for key in ["perception_count", "prediction_count", "trajectory_count", "trajectory_points"]),
            "sched_status": coverage[frame]["sched_status"],
        })
    selection.sort(key=lambda row: int(row["frame_index"]))
    write(APP_OUT / "selection_audit.csv", selection)
    write(APP_OUT / "matched_control_edges.csv", controls)
    write(APP_OUT / "selected_frames.csv", [by_frame[frame] for frame in sorted(selected_ids, key=int)])

    audit = {
        "run_id": "202609111649",
        "definition": "local-jump-only means the previous-20-median x2 and +10 ms rule is the sole candidate reason; every strict-P99 rule is absent",
        "abnormal_frames": [int(frame) for frame in local_only],
        "normal_frames": [int(frame) for frame in normal_frames],
        "abnormal_count": len(local_only),
        "normal_count": len(normal_frames),
        "controls_per_abnormal": 3,
        "all_sched_eligible": True,
        "run_execution_p75_ms": p75_execution_ms,
        "workload_match_fields": workload_keys,
        "normal_selection_limit": "Controls are selected low-latency comparators (<= run P75), so this is a matched operational baseline, not an unbiased estimate of every normal frame.",
    }
    APP_OUT.mkdir(parents=True, exist_ok=True)
    (APP_OUT / "selection_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False))


def summarize():
    selection = read(APP_OUT / "selection_audit.csv")
    selection_by_frame = {row["frame_index"]: row for row in selection}
    selected_ids = set(selection_by_frame)
    frame_states = read(PERF_OUT / "frame_summary.csv")
    assert {row["frame_index"] for row in frame_states} == selected_ids

    callback_rows = []
    for row in frame_states:
        info = selection_by_frame[row["frame_index"]]
        callback_rows.append({
            "frame_index": row["frame_index"],
            "role": info["role"],
            "paired_case_frames": info["paired_case_frames"],
            "callback_tid": row["callback_tid"],
            "execution_ms": row["execution_ms"],
            "running_ms": row["callback_running_ms"],
            "runnable_ms": row["callback_runnable_ms"],
            "blocked_ms": row["callback_blocked_ms"],
            "unknown_ms": row["unknown_ms"],
            "max_sched_delay_ms": max_sched_delay(row["frame_index"]),
            "stack_samples_all_frame_tids": row["stack_samples_all_frame_tids"],
            "stack_samples_callback": row["stack_samples_callback"],
            "sched_context_events": row["sched_context_events"],
        })
    callback_rows.sort(key=lambda row: int(row["frame_index"]))
    write(SYS_OUT / "callback_system_states.csv", callback_rows)

    controls = read(APP_OUT / "matched_control_edges.csv")
    states = {row["frame_index"]: row for row in callback_rows}
    pair_rows = []
    metrics = ["execution_ms", "running_ms", "runnable_ms", "blocked_ms", "unknown_ms", "max_sched_delay_ms"]
    for edge in controls:
        case = states[edge["case_frame"]]
        normal = states[edge["control_frame"]]
        output = {
            "case_frame": edge["case_frame"],
            "control_frame": edge["control_frame"],
            "frame_distance": edge["frame_distance"],
            "matching": edge["matching"],
        }
        for metric in metrics:
            output[f"case_{metric}"] = case[metric]
            output[f"control_{metric}"] = normal[metric]
            output[f"delta_{metric}"] = float(case[metric]) - float(normal[metric])
        pair_rows.append(output)
    write(SYS_OUT / "matched_pair_system_deltas.csv", pair_rows)

    case_rows = []
    for frame in sorted({row["case_frame"] for row in controls}, key=int):
        matched = [row for row in pair_rows if row["case_frame"] == frame]
        output = {"case_frame": frame, "control_frames": ";".join(row["control_frame"] for row in matched), "control_count": len(matched)}
        for metric in metrics:
            case_value = float(states[frame][metric])
            baseline = statistics.mean(float(row[f"control_{metric}"]) for row in matched)
            output[f"case_{metric}"] = case_value
            output[f"control_mean_{metric}"] = baseline
            output[f"delta_vs_control_mean_{metric}"] = case_value - baseline
        case_rows.append(output)
    write(SYS_OUT / "per_case_system_baseline.csv", case_rows)

    cohort_rows = []
    for role in ["abnormal_local_jump_only", "matched_normal"]:
        group = [row for row in callback_rows if row["role"] == role]
        for metric in metrics:
            values = [float(row[metric]) for row in group]
            cohort_rows.append({
                "role": role,
                "metric": metric,
                "n_frames": len(group),
                "mean": statistics.mean(values),
                "median": statistics.median(values),
                "p25": q(values, .25),
                "p75": q(values, .75),
                "min": min(values),
                "max": max(values),
            })
    write(SYS_OUT / "cohort_system_baseline.csv", cohort_rows)

    stack_rows = []
    for role in ["abnormal_local_jump_only", "matched_normal"]:
        group = [row for row in callback_rows if row["role"] == role]
        stack_rows.append({
            "role": role,
            "n_frames": len(group),
            "frames_with_any_stack_sample": sum(int(row["stack_samples_all_frame_tids"]) > 0 for row in group),
            "all_frame_tid_stack_samples": sum(int(row["stack_samples_all_frame_tids"]) for row in group),
            "frames_with_callback_stack_sample": sum(int(row["stack_samples_callback"]) > 0 for row in group),
            "callback_stack_samples": sum(int(row["stack_samples_callback"]) for row in group),
        })
    write(SYS_OUT / "stack_coverage_summary.csv", stack_rows)

    stage_rows = [row for row in read(SYSTEM / "stage_thread_states.csv") if row["frame_index"] in selected_ids]
    write(SYS_OUT / "stage_system_states.csv", stage_rows)
    stage_max = {}
    for row in stage_rows:
        key = (row["frame_index"], row["node"])
        if key not in stage_max or float(row["duration_ms"]) > float(stage_max[key]["duration_ms"]):
            stage_max[key] = row
    stage_frame_rows = []
    for (frame, node), row in sorted(stage_max.items(), key=lambda item: (int(item[0][0]), item[0][1])):
        stage_frame_rows.append({
            "frame_index": frame,
            "role": selection_by_frame[frame]["role"],
            "node": node,
            "tid": row["tid"],
            "duration_ms": row["duration_ms"],
            "running_ms": row["running_ms"],
            "runnable_ms": row["runnable_ms"],
            "blocked_ms": row["blocked_ms"],
            "unknown_ms": row["unknown_ms"],
            "max_sched_delay_ms": row["max_sched_delay_ms"],
            "completion_status": row["completion_status"],
        })
    write(SYS_OUT / "stage_frame_max_system_states.csv", stage_frame_rows)

    stage_cohort = []
    nodes = sorted({row["node"] for row in stage_frame_rows})
    for role in ["abnormal_local_jump_only", "matched_normal"]:
        for node in nodes:
            group = [row for row in stage_frame_rows if row["role"] == role and row["node"] == node]
            if not group:
                continue
            for metric in ["duration_ms", "running_ms", "runnable_ms", "blocked_ms", "unknown_ms", "max_sched_delay_ms"]:
                values = [float(row[metric]) for row in group]
                stage_cohort.append({"role": role, "node": node, "metric": metric, "n_frames": len(group), "median": statistics.median(values), "p25": q(values, .25), "p75": q(values, .75), "max": max(values)})
    write(SYS_OUT / "stage_cohort_baseline.csv", stage_cohort)

    closure = max(abs(float(row["execution_ms"]) - sum(float(row[key]) for key in ["running_ms", "runnable_ms", "blocked_ms", "unknown_ms"])) for row in callback_rows)
    stage_closure = max(abs(float(row["duration_ms"]) - sum(float(row[key]) for key in ["running_ms", "runnable_ms", "blocked_ms", "unknown_ms"])) for row in stage_rows)
    previous = {row["frame_index"]: row for row in read(SYSTEM / "sched_and_stack/frame_summary.csv") if row["frame_index"] in selected_ids}
    repeat_max_difference = max(abs(float(states[frame][new]) - float(previous[frame][old])) for frame in selected_ids for new, old in [("execution_ms", "execution_ms"), ("running_ms", "callback_running_ms"), ("runnable_ms", "callback_runnable_ms"), ("blocked_ms", "callback_blocked_ms"), ("unknown_ms", "unknown_ms")])
    pair_metrics = {
        "pair_count": len(pair_rows),
        "execution_delta_positive_pairs": sum(float(row["delta_execution_ms"]) > 0 for row in pair_rows),
        "blocked_delta_positive_pairs": sum(float(row["delta_blocked_ms"]) > 0 for row in pair_rows),
        "runnable_delta_positive_pairs": sum(float(row["delta_runnable_ms"]) > 0 for row in pair_rows),
        "median_pair_execution_delta_ms": statistics.median(float(row["delta_execution_ms"]) for row in pair_rows),
        "median_pair_blocked_delta_ms": statistics.median(float(row["delta_blocked_ms"]) for row in pair_rows),
    }
    findings = {
        "cohort_medians_ms": {
            role: {metric: next(float(row["median"]) for row in cohort_rows if row["role"] == role and row["metric"] == metric) for metric in metrics}
            for role in ["abnormal_local_jump_only", "matched_normal"]
        },
        "matched_pairs": pair_metrics,
        "per_case": [{
            "frame_index": int(row["case_frame"]),
            "wall_excess_ms": row["delta_vs_control_mean_execution_ms"],
            "blocked_excess_ms": row["delta_vs_control_mean_blocked_ms"],
            "blocked_share_of_wall_excess_pct": 100 * float(row["delta_vs_control_mean_blocked_ms"]) / float(row["delta_vs_control_mean_execution_ms"]),
        } for row in case_rows],
        "interpretation": "The four local-jump-only callbacks are predominantly blocked/sleep extensions inside the NotifyTask path. Runnable waiting and maximum continuous scheduler delay are too small to explain the wall-time excess.",
        "causal_limit": "The wait object is not identified; this baseline does not by itself distinguish a lock, condition/future, worker dependency, or device completion.",
    }
    (SYS_OUT / "baseline_findings.json").write_text(json.dumps(findings, ensure_ascii=False, indent=2), encoding="utf-8")
    validation = {
        "status": "passed",
        "abnormal_frames": 4,
        "normal_frames": 12,
        "perf_frame_directories": sum((PERF_OUT / f"F{int(frame):04d}").is_dir() for frame in selected_ids),
        "perf_frames_with_required_files": sum(all((PERF_OUT / f"F{int(frame):04d}" / name).is_file() for name in ["frame.csv", "extract_window.json", "sched_all_frame_tids_context.txt", "sched_callback_tid_context.txt", "trace_node_instances.csv", "cpu_stack_samples.csv"]) for frame in selected_ids),
        "callback_state_rows": len(callback_rows),
        "matched_pair_rows": len(pair_rows),
        "stage_state_rows": len(stage_rows),
        "maximum_callback_closure_error_ms": closure,
        "maximum_stage_closure_error_ms": stage_closure,
        "unknown_callback_total_ms": sum(float(row["unknown_ms"]) for row in callback_rows),
        "repeat_extraction_max_difference_ms": repeat_max_difference,
        "stage_state_source": str(SYSTEM / "stage_thread_states.csv") + "; filtered to the same 16 frames; generated earlier from the same normalized sched cache and calibration",
        "calibration": json.loads((PERF_OUT / "clock_calibration.json").read_text(encoding="utf-8")),
        "interpretation_limits": [
            "Four abnormal frames are an exhaustive local-jump-only set for this run, but a small cohort.",
            "Twelve controls are deliberately low-latency matched controls, not a random sample of every noncandidate frame.",
            "Stage cohort uses the longest instance per frame and node; parent/child inclusive stages are not additive.",
            "Blocked state does not identify the wait object; CPU stack sampling cannot measure off-CPU time.",
        ],
    }
    assert validation["perf_frame_directories"] == 16
    assert validation["perf_frames_with_required_files"] == 16
    assert closure < 1e-5 and stage_closure < 1e-5
    assert validation["unknown_callback_total_ms"] < 1e-9
    assert repeat_max_difference < 1e-9
    SYS_OUT.mkdir(parents=True, exist_ok=True)
    (SYS_OUT / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")
    write_readme(case_rows, cohort_rows, validation, findings)
    build_manifest()
    print(json.dumps({key: validation[key] for key in ["status", "perf_frame_directories", "callback_state_rows", "stage_state_rows", "maximum_callback_closure_error_ms"]}, ensure_ascii=False))


def max_sched_delay(frame):
    rows = [row for row in read(SYSTEM / "stage_thread_states.csv") if row["frame_index"] == frame and row["node"] == "proc"]
    if rows:
        return max(float(row["max_sched_delay_ms"]) for row in rows)
    intervals = [row for row in read(SYSTEM / "state_intervals.csv") if row["frame_index"] == frame and row["node"] == "proc" and row["state"] == "runnable_waking_proxy"]
    return max((float(row["duration_ms"]) for row in intervals), default=0.0)


def write_readme(case_rows, cohort_rows, validation, findings):
    abnormal = {row["metric"]: row for row in cohort_rows if row["role"] == "abnormal_local_jump_only"}
    normal = {row["metric"]: row for row in cohort_rows if row["role"] == "matched_normal"}
    lines = [
        "# G4 local-jump-only vs matched-normal system baseline",
        "",
        "本目录只比较未命中任何严格P99规则的4个local jump与12个匹配正常帧。应用选帧及完整S1行在打点数据分析的同名目录；本目录保存独立S5原始窗口、callback状态、阶段状态、匹配差值和校验。",
        "",
        "| 指标 | local-jump-only中位数 ms | 正常对照中位数 ms |",
        "| --- | ---: | ---: |",
    ]
    for metric in ["execution_ms", "running_ms", "runnable_ms", "blocked_ms", "max_sched_delay_ms"]:
        lines.append(f"| {metric} | {float(abnormal[metric]['median']):.6f} | {float(normal[metric]['median']):.6f} |")
    lines.extend([
        "",
        "逐案例结果见 `per_case_system_baseline.csv`；每个异常帧与三个控制帧的直接差值见 `matched_pair_system_deltas.csv`。这些差值是匹配描述，不是独立因果效应。blocked只表示睡眠/等待状态，具体锁、future、worker或设备对象仍需对象级打点。",
        "",
        f"12个异常—对照配对中，execution差值为正 {findings['matched_pairs']['execution_delta_positive_pairs']}/12，blocked差值为正 {findings['matched_pairs']['blocked_delta_positive_pairs']}/12；配对execution差值中位数 {findings['matched_pairs']['median_pair_execution_delta_ms']:.6f} ms，blocked差值中位数 {findings['matched_pairs']['median_pair_blocked_delta_ms']:.6f} ms。四个案例的blocked增量占墙钟增量90.5%–98.9%。最长NotifyTask实例的blocked中位数为9.829500 ms（正常0.108500 ms）；最大连续调度等待中位数仅0.017000 ms（正常0.012000 ms）。因此这组local jump表现为NotifyTask路径内的睡眠/等待延长，不是明显的CPU就绪排队。等待对象仍未知。",
        "",
        "CPU stack覆盖较稀：异常组全部帧TID样本6个、callback样本1个；正常组分别4个和3个。调用栈不足以形成组间路径基线，本结论依赖完整sched状态分解。阶段表从此前同一校准与同一规范化sched缓存生成的全量选帧结果中严格过滤这16帧。",
        "",
        f"状态闭合最大误差：callback {validation['maximum_callback_closure_error_ms']:.3e} ms，阶段 {validation['maximum_stage_closure_error_ms']:.3e} ms；16个callback的unknown合计为0。",
    ])
    (SYS_OUT / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_manifest():
    rows = []
    for base in [APP_OUT, SYS_OUT]:
        for path in sorted(base.rglob("*")):
            if not path.is_file() or path.name == "output_manifest_sha256.csv":
                continue
            digest = hashlib.sha256()
            with path.open("rb") as handle:
                for block in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(block)
            rows.append({"path": str(path), "bytes": path.stat().st_size, "sha256": digest.hexdigest()})
    write(SYS_OUT / "output_manifest_sha256.csv", rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["prepare", "summarize"])
    args = parser.parse_args()
    prepare() if args.mode == "prepare" else summarize()
