from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import shutil
from collections import defaultdict
from datetime import datetime
from pathlib import Path


WORKSPACE = Path(r"D:\data")
SOURCE_RUN = WORKSPACE / "202608271537"
APP_OUTPUT = WORKSPACE / "output" / "202608271537_perception_deadline_1_1_1_2"
PERF_OUTPUT = WORKSPACE / "output" / "202608271537_F611_F649_perf_diagnosis"
PERF_RUN = SOURCE_RUN / "perf_run_cpu_stack_001"
CPU_ANALYSIS = PERF_RUN / "analysis"
HANDOFF = WORKSPACE / "202609031348" / "交接文件材料"

CORE_FRAMES = [611, 613, 616, 619, 623]


def rel(path: Path) -> str:
    return path.relative_to(HANDOFF).as_posix()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        fieldnames = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def sha256_copy(source: Path, destination: Path) -> tuple[str, int]:
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite existing handoff file: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256()
    size = 0
    with source.open("rb") as source_handle, destination.open("xb") as destination_handle:
        while True:
            chunk = source_handle.read(8 * 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            destination_handle.write(chunk)
            size += len(chunk)
    shutil.copystat(source, destination)
    if size != source.stat().st_size or destination.stat().st_size != size:
        raise OSError(f"Size mismatch after copying {source}")
    return digest.hexdigest(), size


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(8 * 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def add_file(specs: list[tuple[Path, Path, str, str, str, str, str]], source: Path, destination: Path,
             category: str, source_tool: str, frame_range: str, tid: str, description: str) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    specs.append((source, destination, category, source_tool, frame_range, tid, description))


def add_tree(specs: list[tuple[Path, Path, str, str, str, str, str]], source_root: Path,
             destination_root: Path, category: str, source_tool: str, frame_range: str,
             tid: str, description: str) -> None:
    for source in sorted(source_root.rglob("*")):
        if not source.is_file() or "__pycache__" in source.parts or source.suffix == ".pyc":
            continue
        destination = destination_root / source.relative_to(source_root)
        add_file(specs, source, destination, category, source_tool, frame_range, tid, description)


def phase_for(frame: int) -> str:
    if 601 <= frame <= 610:
        return "normal"
    if 611 <= frame <= 623:
        return "core_abnormal"
    if 626 <= frame <= 649:
        return "recovery"
    return ""


def phase_dir(frame: int) -> Path | None:
    phase = phase_for(frame)
    if phase == "normal":
        return HANDOFF / "extracted" / "F601_F610_normal"
    if phase == "core_abnormal":
        return HANDOFF / "extracted" / "F611_F623_core"
    if phase == "recovery":
        return HANDOFF / "extracted" / "F626_F649_recovery"
    return None


def float_text(value: str, decimals: int = 3) -> str:
    if value in (None, ""):
        return "NA"
    return f"{float(value):.{decimals}f}"


def pct_text(ratio: str) -> str:
    if ratio in (None, ""):
        return "NA"
    return f"{float(ratio) * 100:.1f}%"


def human_bytes(size: int) -> str:
    units = ["B", "KiB", "MiB", "GiB", "TiB"]
    value = float(size)
    for unit in units:
        if value < 1024 or unit == units[-1]:
            return f"{value:.2f} {unit}"
        value /= 1024
    raise AssertionError


def sched_table(rows: list[dict[str, str]]) -> str:
    header = (
        "| Frame | Host TID | Infer ms | CPU Running ms | Running % | sleep/block ms | "
        "runnable wait ms | max scheduler delay ms | CPU | kswapd0 CPU ms |\n"
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |\n"
    )
    body = []
    for row in rows:
        body.append(
            "| F{frame} | {tid} | {infer} | {running} | {ratio} | {blocked} | {runnable} | "
            "{delay} | {cpus} | {kswapd} |".format(
                frame=row["frame"],
                tid=row["host_tid"],
                infer=float_text(row["infer_ms"]),
                running=float_text(row["running_ms"]),
                ratio=pct_text(row["running_ratio"]),
                blocked=float_text(row["sleep_block_ms"]),
                runnable=float_text(row["runnable_wait_ms"]),
                delay=float_text(row["max_sched_delay_ms"]),
                cpus=row["cpu_list"],
                kswapd=float_text(row["kswapd0_cpu_ms"]),
            )
        )
    return header + "\n".join(body)


def main() -> None:
    # The destination was user-selected for delivery only. All evidence comes from run 202608271537.
    for required in [SOURCE_RUN, APP_OUTPUT, PERF_OUTPUT, PERF_RUN, CPU_ANALYSIS]:
        if not required.exists():
            raise FileNotFoundError(required)

    specs: list[tuple[Path, Path, str, str, str, str, str]] = []

    # Capture-wide perf sched files. These are copied byte-for-byte without filtering.
    for name in [
        "perf.data",
        "perf_record_command.txt",
        "perf_record.stderr.txt",
        "perf_record.stdout.txt",
        "perf_sched_script.txt",
        "perf_sched_script.stderr.txt",
        "perf_sched_timehist.txt",
        "perf_sched_timehist.stderr.txt",
        "perf_sched_map.txt",
        "perf_sched_map.stderr.txt",
        "perf_sched_latency.txt",
        "perf_sched_latency.stderr.txt",
        "report_status.txt",
    ]:
        add_file(
            specs, PERF_RUN / name, HANDOFF / "raw" / "perf_sched" / name,
            "perf_sched", "perf_sched", "capture-wide", "",
            "Capture-wide perf sched raw data or the collector-generated companion output; copied unchanged.",
        )

    # Capture-wide CPU stack files.
    for name in [
        "perf_cpu_stack.data",
        "perf_cpu_stack_record_command.txt",
        "perf_cpu_stack_record.stderr.txt",
        "perf_cpu_stack_record.stdout.txt",
        "perf_cpu_stack_script.txt",
        "perf_cpu_stack_script.stderr.txt",
        "perf_cpu_stack_report.txt",
        "perf_cpu_stack_report.stderr.txt",
        "cpu_stack_report_status.txt",
    ]:
        add_file(
            specs, PERF_RUN / name, HANDOFF / "raw" / "cpu_stack" / name,
            "cpu_stack", "perf_record", "capture-wide", "1016337",
            "Capture-wide Perception CPU stack raw data or perf-generated companion output; copied unchanged.",
        )

    # Collector clock, process, TID, policy, affinity, and environment evidence.
    for name in [
        "capture_start_clock.txt",
        "capture_end_clock.txt",
        "collector_script_sha256.txt",
        "collector_script_used.py",
        "environment.txt",
        "perception_process.txt",
        "perception_affinity_before.txt",
        "perception_affinity_after.txt",
        "perception_chrt_before.txt",
        "perception_chrt_after.txt",
        "perception_proc_status_before.txt",
        "perception_proc_status_after.txt",
        "perception_ps_threads_before.txt",
        "perception_ps_threads_before.stderr.txt",
        "perception_ps_threads_after.txt",
        "perception_ps_threads_after.stderr.txt",
    ]:
        add_file(
            specs, PERF_RUN / name, HANDOFF / "raw" / "tid_mapping" / name,
            "tid_mapping", "other", "capture-wide", "Perception process and worker TIDs",
            "Collector metadata used to validate process/TID identity and the capture environment; copied unchanged.",
        )

    add_file(
        specs, SOURCE_RUN / "collect_time.txt", HANDOFF / "raw" / "frame_timing" / "collect_time.txt",
        "frame_timing", "application_instrumentation", "run-wide", "",
        "Run collection time metadata; copied unchanged.",
    )
    add_file(
        specs,
        SOURCE_RUN / "log" / "perception.log.INFO.20260827-153547.1016337",
        HANDOFF / "raw" / "frame_timing" / "perception.log.INFO.20260827-153547.1016337",
        "frame_timing", "application_instrumentation", "run-wide", "1016337",
        "Original Perception log containing CenterPoint/Infer timing markers; copied unchanged.",
    )

    perception_stages = [
        "perception.pointcloud_preprocess.1016337.csv",
        "perception.pointcloud_map_based_roi.1016337.csv",
        "perception.pointcloud_ground_detection.1016337.csv",
        "perception.lidar_detection.1016337.csv",
        "perception.lidar_detection_filter.1016337.csv",
        "perception.lidar_tracking.1016337.csv",
        "perception.multi_sensor_fusion.1016337.csv",
    ]
    for name in perception_stages:
        add_file(
            specs, SOURCE_RUN / "trace" / "events" / name,
            HANDOFF / "raw" / "frame_timing" / "trace_events" / name,
            "frame_timing", "application_instrumentation", "run-wide", "1016337",
            "Original Perception node event trace for P1-P7 frame timing; copied unchanged.",
        )
        add_file(
            specs, SOURCE_RUN / "trace" / "message_context" / name,
            HANDOFF / "raw" / "frame_timing" / "message_context" / name,
            "frame_timing", "application_instrumentation", "run-wide", "1016337",
            "Original Perception node message-context trace; copied unchanged.",
        )
    add_file(
        specs,
        SOURCE_RUN / "trace" / "fusion_inputs" / "perception.multi_sensor_fusion.1016337.csv",
        HANDOFF / "raw" / "frame_timing" / "fusion_inputs" / "perception.multi_sensor_fusion.1016337.csv",
        "frame_timing", "application_instrumentation", "run-wide", "1016337",
        "Original MultiSensorFusion input trace; copied unchanged.",
    )
    add_file(
        specs,
        SOURCE_RUN / "trace" / "trace_anchor" / "perception.1016337.csv",
        HANDOFF / "raw" / "frame_timing" / "trace_anchor" / "perception.1016337.csv",
        "frame_timing", "application_instrumentation", "run-wide", "1016337",
        "Original Perception trace anchor; copied unchanged.",
    )

    add_tree(
        specs, APP_OUTPUT, HANDOFF / "raw" / "original_reports" / "application_analysis",
        "original_report", "application_instrumentation", "F1-F688", "",
        "Existing application-layer analysis artifact; copied unchanged.",
    )
    add_tree(
        specs, PERF_OUTPUT, HANDOFF / "raw" / "original_reports" / "perf_diagnosis",
        "original_report", "perf_sched", "F601-F649", "varies by frame",
        "Existing F611-F649 perf diagnosis artifact; copied unchanged.",
    )
    add_tree(
        specs, CPU_ANALYSIS, HANDOFF / "raw" / "original_reports" / "cpu_stack_analysis",
        "original_report", "cpu_stack_extractor", "F601-F659", "varies by frame",
        "Existing P4 sched/CPU-stack audit artifact; copied unchanged.",
    )

    reusable = [
        ("perception", "analyze_perception_realtime.py"),
        ("perception", "plot_perception_critical_path_gantt.py"),
        ("scheduler", "extract_perf_sched_frame_windows.py"),
        ("scheduler", "analyze_perf_sched_infer_frames.py"),
        ("perf", "generate_p4_perf_analysis.py"),
        ("perf", "classify_p4_cpu_stack_samples.py"),
    ]
    for folder, name in reusable:
        source = WORKSPACE / "reusable_scripts" / folder / name
        add_file(
            specs, source, HANDOFF / "raw" / "original_reports" / "reusable_scripts" / folder / name,
            "analysis_script", "other", "parameterized", "",
            "Current reusable analysis/extraction script from the workspace index; copied unchanged.",
        )

    inventory: list[dict[str, object]] = []
    for index, (source, destination, category, source_tool, frame_range, tid, description) in enumerate(specs, 1):
        print(f"[{index}/{len(specs)}] copying {source}", flush=True)
        sha256, size = sha256_copy(source, destination)
        inventory.append(
            {
                "category": category,
                "filename": source.name,
                "absolute_original_path": str(source),
                "handoff_relative_path": rel(destination),
                "file_size": size,
                "mtime": datetime.fromtimestamp(source.stat().st_mtime).astimezone().isoformat(timespec="seconds"),
                "sha256": sha256,
                "source_tool": source_tool,
                "time_range": "run/capture 202608271537",
                "frame_range": frame_range,
                "tid": tid,
                "description": description,
            }
        )

    write_csv(
        HANDOFF / "index" / "raw_data_inventory.csv",
        inventory,
        [
            "category", "filename", "absolute_original_path", "handoff_relative_path", "file_size",
            "mtime", "sha256", "source_tool", "time_range", "frame_range", "tid", "description",
        ],
    )
    write_csv(
        HANDOFF / "index" / "source_evidence_catalog.csv",
        [
            {
                "source_path": row["absolute_original_path"],
                "category": row["category"],
                "purpose": row["description"],
                "copied_to": row["handoff_relative_path"],
                "sha256": row["sha256"],
            }
            for row in inventory
        ],
        ["source_path", "category", "purpose", "copied_to", "sha256"],
    )

    destination_hash_mismatches = []
    for row in inventory:
        destination = HANDOFF / Path(str(row["handoff_relative_path"]))
        if sha256_file(destination) != row["sha256"]:
            destination_hash_mismatches.append(str(row["handoff_relative_path"]))
    if destination_hash_mismatches:
        raise AssertionError(f"Destination SHA-256 mismatch: {destination_hash_mismatches}")

    app_rows = read_csv(APP_OUTPUT / "data" / "p4_framewise_change_diagnostics.csv")
    app_by_frame = {int(row["source_frame_index"]): row for row in app_rows}
    window_rows = read_csv(PERF_OUTPUT / "sched_aligned" / "infer_frame_windows.csv")
    windows_by_frame = {int(row["source_frame_index"]): row for row in window_rows}
    aligned_sched_rows = read_csv(PERF_OUTPUT / "sched_aligned" / "infer_sched_frame_summary.csv")
    aligned_sched_by_frame = {int(row["source_frame_index"]): row for row in aligned_sched_rows}
    sched_rows = read_csv(CPU_ANALYSIS / "p4_sched_frame_summary.csv")
    sched_by_frame = {int(row["frame"]): row for row in sched_rows}
    identity_rows = read_csv(CPU_ANALYSIS / "p4_frame_identity.csv")
    identity_by_frame = {int(row["frame"]): row for row in identity_rows}
    stack_summary_rows = read_csv(CPU_ANALYSIS / "p4_cpu_stack_frame_summary.csv")
    stack_summary_by_frame = {int(row["frame"]): row for row in stack_summary_rows}

    # Preserve raw event lines already selected by the existing audit script; no deduplication or reordering.
    sched_context_rows = read_csv(CPU_ANALYSIS / "p4_sched_events_context.csv")
    events = [(float(row["timestamp"]), row["raw_event"]) for row in sched_context_rows]
    perf_file_by_frame: dict[int, str] = {}
    perf_event_count_by_frame: dict[int, int] = {}
    for frame, window in sorted(windows_by_frame.items()):
        if not 601 <= frame <= 649:
            continue
        destination_dir = phase_dir(frame)
        if destination_dir is None:
            continue
        start = float(window["enter_perf_s"])
        end = float(window["exit_perf_s"])
        selected_events = [raw_event for timestamp, raw_event in events if start <= timestamp <= end]
        destination = destination_dir / f"F{frame}_perf_sched_raw.txt"
        write_text(destination, "\n".join(selected_events) + ("\n" if selected_events else ""))
        perf_file_by_frame[frame] = rel(destination)
        perf_event_count_by_frame[frame] = len(selected_events)

    # Split the existing full-sample extract at its frame markers. Lines within each section are unchanged.
    combined_stack = (CPU_ANALYSIS / "cpu_stack_samples_anomaly.txt").read_text(encoding="utf-8")
    markers = list(re.finditer(r"(?m)^===== FRAME F(\d+) .*?=====\r?$", combined_stack))
    stack_sections: dict[int, str] = {}
    for marker_index, marker in enumerate(markers):
        start = marker.start()
        end = markers[marker_index + 1].start() if marker_index + 1 < len(markers) else len(combined_stack)
        stack_sections[int(marker.group(1))] = combined_stack[start:end]

    stack_file_by_frame: dict[int, str] = {}
    sample_header_re = re.compile(r"(?m)^[^\s].*\s+\d+\s+\[\d+\]\s+\d+\.\d+:\s+")
    extracted_sample_counts: dict[int, int] = {}
    for frame in CORE_FRAMES:
        if frame not in stack_sections:
            raise ValueError(f"Missing stack section for F{frame}")
        destination = HANDOFF / "extracted" / "F611_F623_core" / f"F{frame}_cpu_stack_raw.txt"
        write_text(destination, stack_sections[frame])
        stack_file_by_frame[frame] = rel(destination)
        extracted_sample_counts[frame] = len(sample_header_re.findall(stack_sections[frame]))

    phase_ranges = [
        ("F601_F610_normal", 601, 610),
        ("F611_F623_core", 611, 623),
        ("F626_F649_recovery", 626, 649),
    ]
    for folder, start, end in phase_ranges:
        output_dir = HANDOFF / "extracted" / folder
        selected_app = [row for row in app_rows if start <= int(row["source_frame_index"]) <= end]
        selected_sched = [row for row in aligned_sched_rows if start <= int(row["source_frame_index"]) <= end]
        selected_windows = [row for row in window_rows if start <= int(row["source_frame_index"]) <= end]
        write_csv(output_dir / "p4_application_timing.csv", selected_app)
        write_csv(output_dir / "infer_sched_frame_summary.csv", selected_sched)
        write_csv(output_dir / "infer_frame_windows.csv", selected_windows)

    core_stack_summaries = [stack_summary_by_frame[frame] for frame in CORE_FRAMES]
    write_csv(HANDOFF / "extracted" / "F611_F623_core" / "p4_cpu_stack_frame_summary.csv", core_stack_summaries)

    path_tokens = {
        "paddle_net_infer": "PaddleNet::Infer",
        "auto_growth_allocator": "AutoGrowthBestFitAllocator::AllocateImpl",
        "cuda_allocator": "CUDAAllocator::AllocateImpl",
        "recorded_gpu_malloc": "RecordedGpuMallocHelper::Malloc",
        "cuda_malloc": "cudaMalloc",
        "nvmap": "nvmap_",
        "linux_page_allocator": "get_page_from_freelist",
        "clear_page": "clear_page",
        "nvgpu_mapping": "nvgpu_vm_map",
    }
    presence_rows: list[dict[str, object]] = []
    for frame in CORE_FRAMES:
        section = stack_sections[frame]
        row: dict[str, object] = {"frame": frame}
        for column, token in path_tokens.items():
            row[column] = "present" if token in section else "not_located"
        row["raw_stack_file"] = stack_file_by_frame[frame]
        presence_rows.append(row)
    write_csv(
        HANDOFF / "extracted" / "F611_F623_core" / "stack_path_presence.csv",
        presence_rows,
        ["frame", *path_tokens, "raw_stack_file"],
    )

    index_rows: list[dict[str, object]] = []
    missing_infer_frames: list[int] = []
    for frame in range(601, 650):
        app = app_by_frame[frame]
        window = windows_by_frame.get(frame)
        identity = identity_by_frame.get(frame, {})
        phase = phase_for(frame)
        notes = [app.get("evidence_boundary", "")]
        if not window:
            missing_infer_frames.append(frame)
            notes.append("No unique CP_INFER ENTER/EXIT pair mapped; Infer/perf/TID fields left blank.")
        if frame in (624, 625):
            notes.append("Outside the requested core_abnormal and recovery phase boundaries; phase intentionally blank.")
        if frame in perf_file_by_frame:
            notes.append("Per-frame perf file preserves selected raw_event lines from the existing audit context; capture-wide raw script is under raw/perf_sched.")
        index_rows.append(
            {
                "frame": frame,
                "segment": "S4_reference" if frame <= 610 else "S4",
                "phase": phase,
                "perception_input_time": app.get("source_time_unix_s", ""),
                "p4_input_ready_time": app.get("input_ready_proxy_mono_ns", ""),
                "p4_proc_enter": app.get("proc_enter_mono_ns", ""),
                "infer_start": window.get("enter_apollo_mono_ns", "") if window else "",
                "infer_end": window.get("exit_apollo_mono_ns", "") if window else "",
                "p4_output_time": app.get("output_pub_mono_ns", ""),
                "p4_execution_ms": app.get("execution_observed_ms", ""),
                "p4_waiting_ms": app.get("waiting_proxy_ms", ""),
                "p4_deadline_budget_ms": app.get("deadline_budget_proxy_ms", ""),
                "p4_slack_ms": app.get("slack_proxy_ms", ""),
                "p4_state": app.get("proxy_result", ""),
                "host_tid": window.get("target_tid", identity.get("host_tid", "")) if window else "",
                "perf_window_start": window.get("enter_perf_s", "") if window else "",
                "perf_window_end": window.get("exit_perf_s", "") if window else "",
                "perf_sched_raw_file": perf_file_by_frame.get(frame, ""),
                "cpu_stack_raw_file": stack_file_by_frame.get(frame, ""),
                "notes": " ".join(note for note in notes if note),
            }
        )
    index_fields = [
        "frame", "segment", "phase", "perception_input_time", "p4_input_ready_time", "p4_proc_enter",
        "infer_start", "infer_end", "p4_output_time", "p4_execution_ms", "p4_waiting_ms",
        "p4_deadline_budget_ms", "p4_slack_ms", "p4_state", "host_tid", "perf_window_start",
        "perf_window_end", "perf_sched_raw_file", "cpu_stack_raw_file", "notes",
    ]
    write_csv(HANDOFF / "index" / "frame_system_trace_index.csv", index_rows, index_fields)

    module = read_csv(APP_OUTPUT / "data" / "module_deadline_summary.csv")[0]
    nodes = read_csv(APP_OUTPUT / "data" / "perception_node_proxy_deadline_summary.csv")
    sections_metrics = json.loads((APP_OUTPUT / "data" / "deadline_sections_1_1_1_2_metrics.json").read_text(encoding="utf-8"))
    segments = read_csv(APP_OUTPUT / "data" / "p4_major_abnormal_segments.csv")

    normal_sched = [sched_by_frame[frame] for frame in range(601, 611)]
    core_sched = [sched_by_frame[frame] for frame in CORE_FRAMES]
    recovery_sched = [sched_by_frame[frame] for frame in sorted(sched_by_frame) if 626 <= frame <= 649 and sched_by_frame[frame]["data_status"] == "OK"]

    total_raw_bytes = sum(int(row["file_size"]) for row in inventory)
    large_raw = [row for row in inventory if row["category"] in {"perf_sched", "cpu_stack"} and int(row["file_size"]) >= 1_000_000]
    large_raw_md = "\n".join(
        f"- `{row['filename']}` — {human_bytes(int(row['file_size']))}; SHA256 `{row['sha256']}`; "
        f"original: `{row['absolute_original_path']}`; handoff: `{row['handoff_relative_path']}`"
        for row in large_raw
    )

    node_lines = [
        "| 节点 | 可判实例 | deadline miss | miss rate | 已有判断 |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for node in nodes:
        symbol = node["node_symbol"]
        if symbol in {"P1", "P2", "P3"}:
            judgement = "未形成大量自身 deadline miss"
        elif symbol == "P4":
            judgement = "主要异常节点；123 execution-driven / 53 waiting-driven / 48 no-service"
        else:
            judgement = "自身无新增 miss；仅统计真正从 P4 到达的实例"
        node_lines.append(
            f"| {symbol} {node['node_name']} | {node['evaluated_instances']} | {node['deadline_miss_count']} | "
            f"{float(node['deadline_miss_rate']) * 100:.2f}% | {judgement} |"
        )
    node_table = "\n".join(node_lines)

    segment_table = "\n".join(
        [
            "| 段 | 帧范围 | span | 既有起点说明 |",
            "| --- | --- | ---: | --- |",
        ]
        + [
            f"| {row['segment_id']} | {row['frame_range']} | {row['span_frames']} | {row['start_mechanism']} |"
            for row in segments
        ]
    )

    problem_discovery = f"""# 1. 实时性问题发现过程

本文件只复述既有分析步骤、指标和证据边界，不新增根因判断。数据源 run 为 `202608271537`；交付目录所在的 `202609031348` 仅是本次材料的目标位置。

## Step 1：模块级 deadline

既有报告将 Perception freshness deadline 定义为：本帧 P1 `proc_enter` 到 P7 `output_pub` 必须不晚于下一帧 P1 `proc_enter`。约 100 ms 表示 P1 输入帧间隔，不是 sensor 产生到进入 Perception 的传输时延。

| 指标 | 既有结果 |
| --- | ---: |
| 可判帧 | {module['evaluated_instances']} |
| deadline miss | {module['deadline_miss_count']} / {module['evaluated_instances']} |
| miss rate | {float(module['deadline_miss_rate']) * 100:.2f}% |
| 输入周期 P50 | {float(module['budget_p50_ms']):.3f} ms |
| response time P50 | {float(module['response_p50_ms']):.3f} ms |
| response time P95 | {float(module['response_p95_ms']):.3f} ms |
| response time MAX | {float(module['response_max_ms']):.3f} ms |

来源：[`module_deadline_summary.csv`](raw/original_reports/application_analysis/data/module_deadline_summary.csv)、[`sections_1_1_1_2.md`](raw/original_reports/application_analysis/report/sections_1_1_1_2.md)。

## Step 2：节点级 deadline

{node_table}

P2–P7 的 input-ready 使用上一节点 `output_pub` 代理；严格 Reader receive/enqueue/drop 标记不可用。P5–P7 的 639 个实例只覆盖真正从 P4 到达的实例。

来源：[`perception_node_proxy_deadline_summary.csv`](raw/original_reports/application_analysis/data/perception_node_proxy_deadline_summary.csv)、[`perception_node_proxy_deadline_audit.csv`](raw/original_reports/application_analysis/data/perception_node_proxy_deadline_audit.csv)。

## Step 3：异常段划分

既有结果从 P4 deadline miss 逐帧数据中得到 76 个连续 miss 段，并提取 4 个主要异常段：

{segment_table}

来源：[`p4_contiguous_miss_runs.csv`](raw/original_reports/application_analysis/data/p4_contiguous_miss_runs.csv)、[`p4_major_abnormal_segments.csv`](raw/original_reports/application_analysis/data/p4_major_abnormal_segments.csv)。

## Step 4：为什么进一步使用 perf

应用层打点已经记录 P4 execution 长尾、deadline miss、waiting、no-service/frame skipping，以及异常连续维持多个 source frame；这些数据不能区分 execution 增长期间线程是在等待 CPU、sleep/block，还是获得 CPU 后的 CPU Running 时间增加。因此既有工作针对异常段继续检查 perf sched。本交接不进一步判断 OS 根因。

## Step 5：perf sched 得到了什么

既有 S4 分解将 F601–F610 作为异常前参考，将 F611/F613/F616/F619/F623 作为严重 CPU Running inflation 的有完整 Infer 实例重点帧，并记录 F626 起的恢复参考。完整逐帧表见 [`02_system_observation.md`](02_system_observation.md) 和各阶段 `infer_sched_frame_summary.csv`。

现有 perf 数据观察到：严重异常窗口中 CPU Running 明显增加，而 runnable waiting / scheduler delay 相对较小。该句是现有观测的整理，不扩展为 OS 根因判断。

## Step 6：为什么继续采集 CPU Stack

perf sched 给出线程状态时长，但不能直接说明 CPU Running 时运行在哪些函数。因此既有工作在 F611、F613、F616、F619、F623 的严格 Infer 时间窗口内保留 CPU stack samples。每个逐帧文件保留 sample timestamp、comm、原始数值 ID、CPU、`cpu-clock` event 和完整 user + kernel call chain，不去重、不按符号筛选。
"""
    write_text(HANDOFF / "01_problem_discovery.md", problem_discovery)

    presence_by_frame = {int(row["frame"]): row for row in presence_rows}
    presence_table_lines = [
        "| Frame | samples | PaddleNet::Infer | AutoGrowth | CUDAAllocator | cudaMalloc | nvmap | page allocator | clear_page | nvgpu mapping | 原始文件 |",
        "| --- | ---: | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for frame in CORE_FRAMES:
        presence = presence_by_frame[frame]
        summary = stack_summary_by_frame[frame]
        marks = lambda key: "已定位" if presence[key] == "present" else "未定位"
        presence_table_lines.append(
            f"| F{frame} | {summary['stack_sample_count']} | {marks('paddle_net_infer')} | {marks('auto_growth_allocator')} | {marks('cuda_allocator')} | "
            f"{marks('cuda_malloc')} | {marks('nvmap')} | {marks('linux_page_allocator')} | {marks('clear_page')} | "
            f"{marks('nvgpu_mapping')} | [`F{frame}_cpu_stack_raw.txt`](extracted/F611_F623_core/F{frame}_cpu_stack_raw.txt) |"
        )
    presence_table = "\n".join(presence_table_lines)

    system_observation = f"""# 2. 系统层已有观测与 CPU Stack 数据整理

本文件只整理既有 perf/CPU stack 结果。`Running` 表示 CPU-resident；不在本交接中把相关路径改写为根因或因果关系。

## 2.1 异常前参考：F601–F610

既有 CPU-stack 审计文件将这些帧标为 `candidate_control`；本交接按用户给定范围作为异常前参考，不额外证明其统计学“正常性”。

{sched_table(normal_sched)}

## 2.2 核心严重 CPU Running inflation：F611–F623 中的可映射重点帧

F612/F614/F615/F617/F618/F620/F621/F622 在应用层为 no-service，未映射唯一 Infer ENTER/EXIT；因此不使用相邻帧补值。

{sched_table(core_sched)}

现有 perf 数据观察到严重异常窗口中 CPU Running 明显增加，而 runnable waiting / scheduler delay 相对较小。完整原始 capture 在 [`raw/perf_sched`](raw/perf_sched/)，逐帧入口见 [`frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。

## 2.3 执行状态恢复参考：F626 及之后

下表保留当前已有可映射 Infer 实例；F640 无唯一 Infer 映射，因此不补值。

{sched_table(recovery_sched)}

## 2.4 CPU Stack 数据整理

以下“已定位”只表示函数名确实出现在该帧现有原始 stack section 中，不表示精确耗时占比或因果关系。sample percentage 是采样点比例，不能自动转换为精确毫秒耗时。

{presence_table}

现有 stack 中可定位的路径包括：

- `AutoGrowthBestFitAllocator::AllocateImpl` → `CUDAAllocator::AllocateImpl` → `RecordedGpuMallocHelper::Malloc` → `cudaMalloc` 在五个逐帧原始 section 中均可定位。`PaddleNet::Infer` 虽在已有报告路径描述中出现，但未在当前 F611、F623 原始 section 中重新定位到；F613、F616、F619 中可定位。
- `cudaMalloc` 下游样本中出现 `nvmap`、Linux page allocator（包括 `get_page_from_freelist` / `clear_page`）以及 `nvgpu`/GMMU mapping 相关函数。

逐帧文件保留所有原始 samples，包括普通 Paddle/cuDNN 路径；未去重，未只筛选 `cudaMalloc`。capture-wide stack 原文为 [`perf_cpu_stack_script.txt`](raw/cpu_stack/perf_cpu_stack_script.txt)。
"""
    write_text(HANDOFF / "02_system_observation.md", system_observation)

    calibration = json.loads((PERF_OUTPUT / "sched_clock_calibration.json").read_text(encoding="utf-8"))
    data_mapping = f"""# 3. Frame → Time → TID → Raw Data 映射说明

主索引：[`frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。索引覆盖 F601–F649，共 49 个 source frame；空值表示当前证据中不存在该字段，没有用相邻帧、均值或模型值补齐。

## 字段与时钟

- `perception_input_time`：应用层 `source_time_unix_s`，Unix wall-clock 秒。
- `p4_input_ready_time`：`e2e_trace_v3` 中的 `input_ready_proxy_mono_ns`；P4 使用上游 P3 `output_pub` 作为 input-ready 代理，不是严格 Reader 排队时间。
- `p4_proc_enter` / `p4_output_time`：`e2e_trace_v3` 事件的 `mono_ns`。
- `infer_start` / `infer_end`：Perception log 中唯一配对的 `CP_INFER_ENTER/EXIT mono_ns`；现有 validation 已验证该时间域与 `perf_cpu_stack` 的 `CLOCK_MONOTONIC` samples 对齐。
- `perf_window_start` / `perf_window_end`：与 Infer 窗口对齐后的 perf sched 秒。
- 时钟校准方法：`{calibration['method']}`。
- 校准值：`sched_minus_stack = {calibration['best_sched_minus_stack_offset_ns']} ns`；支持样本 {calibration['supported_samples']} / {calibration['sample_count']}（{calibration['support_pct']:.2f}%）。
- `host_tid`：由原始 `/proc` task status、Host `ps` before/after 快照和现有 identity audit 给出的该帧实际 worker TID；不是固定的 LidarDetection 线程名。

`e2e_trace_v3 mono_ns` 与 Perception log 的 `CP_INFER mono_ns` 在当前文件中不是同一数值基准；例如 F611 的 P4 trace 时间与 CP Infer 时间不能直接相减。现有 Frame→Infer 关联来自 `centerpoint_internal_timing_per_source_frame.csv` 的 source-frame/sensor-time 映射和相邻日志行中的唯一 CP pair，而不是把这两个 `mono_ns` 字段直接对齐。当前交接未发现现成的 trace→CP 显式时钟 offset，因此保留两个原始时间字段并明确分域，不自行补算 offset。

## phase 边界

- F601–F610：`normal`（异常前参考；原审计角色为 `candidate_control`）。
- F611–F623：`core_abnormal`。
- F624–F625：位于 core 与 recovery 边界之间，按字段留空，避免混入任一概念。
- F626–F649：`recovery`。

## per-frame perf 文件边界

每个 `Fxxx_perf_sched_raw.txt` 由现有 `p4_sched_events_context.csv` 中、落在该帧严格 Infer perf 窗口内的 `raw_event` 行按原顺序导出；未去重、未改写行内容。该文件是目标 TID/kswapd0 等既有筛选上下文，不等同于窗口内全部 system-wide 事件。完整 system-wide 原文保留在 [`perf_sched_script.txt`](raw/perf_sched/perf_sched_script.txt)，原始二进制为 [`perf.data`](raw/perf_sched/perf.data)。
"""
    write_text(HANDOFF / "03_data_mapping.md", data_mapping)

    five_frame_checks: dict[str, dict[str, object]] = {}
    for frame in CORE_FRAMES:
        stack_expected = int(stack_summary_by_frame[frame]["stack_sample_count"])
        five_frame_checks[f"F{frame}"] = {
            "application_timing": frame in app_by_frame and bool(app_by_frame[frame]["proc_enter_mono_ns"]),
            "infer_window": frame in windows_by_frame,
            "host_tid": windows_by_frame.get(frame, {}).get("target_tid") == "1016506",
            "perf_sched_summary": frame in sched_by_frame and sched_by_frame[frame]["data_status"] == "OK",
            "perf_sched_raw_extract": frame in perf_file_by_frame and perf_event_count_by_frame.get(frame, 0) > 0,
            "cpu_stack_raw_samples": frame in stack_file_by_frame and extracted_sample_counts[frame] > 0,
            "stack_sample_count_expected": stack_expected,
            "stack_sample_count_extracted": extracted_sample_counts[frame],
            "stack_sample_count_match": stack_expected == extracted_sample_counts[frame],
        }
    all_five_complete = all(all(value for key, value in check.items() if key not in {"stack_sample_count_expected", "stack_sample_count_extracted"}) for check in five_frame_checks.values())

    missing_data = f"""# MISSING_DATA

## 五个重点帧完整性结论

F611、F613、F616、F619、F623 均具备：应用层 P4 timing、唯一 Infer window、Host TID、perf sched summary、逐帧 perf raw-event extract 和未去重的 CPU stack raw samples。详见 [`integrity_audit.json`](index/integrity_audit.json)。

## 仍存在的证据缺口

- P2–P7 的严格 Reader receive/enqueue/drop 时间不可用；`p4_input_ready_time` 是上一节点 `output_pub` 代理。
- 当前资料没有提供 `e2e_trace_v3 mono_ns` 到 Perception log `CP_INFER mono_ns` 的显式校准 offset；两者按 source frame/sensor timestamp 建立关联，不能直接相减。perf sched 与 CP Infer/CPU stack 的 offset 已单独校准并保留。
- F612、F614、F615、F617、F618、F620、F621、F622、F624、F625、F640 没有唯一 `CP_INFER_ENTER/EXIT` 映射；对应 Infer、Host TID、perf window 和逐帧 stack 字段为空。应用层将这些帧标为 no-service；其中 F624–F625 不属于给定的 core phase。
- CPU stack 原始 sample header 保留 `comm`、数值 ID、CPU、timestamp 和 event；现有格式及审计将该数值 ID 用作 Host TID，但 sample 行没有同时展开独立的 PID 与 TID 两列。
- F624–F625 不属于给定的 `core_abnormal=F611–F623` 或 `recovery=F626–F649`，索引中的 `phase` 有意留空。

这些缺口未使用相邻帧、平均值或模型值补齐。
"""
    write_text(HANDOFF / "MISSING_DATA.md", missing_data)

    core_links = "\n".join(
        f"- F{frame}: [perf raw-event extract](extracted/F611_F623_core/F{frame}_perf_sched_raw.txt) · "
        f"[CPU stack raw samples](extracted/F611_F623_core/F{frame}_cpu_stack_raw.txt)"
        for frame in CORE_FRAMES
    )
    readme = f"""# Apollo Perception P4 实时性异常 OS 层交接材料

## OS 层建议首先查看的数据

- 主入口：[`SYSTEM_REALTIME_ISSUE_HANDOFF.md`](SYSTEM_REALTIME_ISSUE_HANDOFF.md)
- 重点异常段：S4 F611–F649
- 异常前参考：F601–F610
- CPU Running 明显异常窗口：F611–F623；不要与完整应用层异常段 F611–F649 混为一谈
- 重点异常帧：F611、F613、F616、F619、F623
- 恢复参考：F626 及之后
- 核心五帧实际 Host TID：1016506（已由现有 identity audit 与原始 before/after TID 快照核对）
- CPU stack 已观察到的路径：Paddle allocator → `cudaMalloc` → `nvmap` → Linux page allocator / `clear_page`，以及 `nvgpu`/GMMU mapping；这里只作为已观察路径导航，不作为根因结论

{core_links}

逐帧总索引：[`index/frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。缺失项与证据边界：[`MISSING_DATA.md`](MISSING_DATA.md)。

## 数据源与目录边界

本交接的数据源是 `D:\\data\\202608271537` 及其两个既有输出目录；`D:\\data\\202609031348\\交接文件材料` 只是用户指定的本次交付位置。未使用同级 `202609031348` run 的 timing/perf/stack 数据。

目录说明：

- `raw/perf_sched/`：capture-wide perf sched 二进制与原始文本，逐字节复制。
- `raw/cpu_stack/`：capture-wide CPU stack 二进制、perf script/report 文本，逐字节复制。
- `raw/frame_timing/`：Perception log 与 P1–P7 原始 trace。
- `raw/tid_mapping/`：采集时钟、进程、`/proc`、Host `ps`、调度策略与 affinity 快照。
- `raw/original_reports/`：既有应用层、perf、CPU stack 分析结果与相关脚本的未修改副本。
- `extracted/`：按 normal/core/recovery 划分的 Case 窗口与逐帧导航文件。
- `index/`：Frame→Time→TID→Raw Data 索引、原始文件清单、完整性审核和本构建脚本。

## 建议阅读顺序

1. [`SYSTEM_REALTIME_ISSUE_HANDOFF.md`](SYSTEM_REALTIME_ISSUE_HANDOFF.md)
2. [`01_problem_discovery.md`](01_problem_discovery.md)
3. [`02_system_observation.md`](02_system_observation.md)
4. [`index/frame_system_trace_index.csv`](index/frame_system_trace_index.csv)
5. 对应帧的 perf/stack raw extract，再回到 capture-wide `raw/` 原始文件复核

## 原始文件完整性

本交接共复制 {len(inventory)} 个 raw/ 文件，合计 {human_bytes(total_raw_bytes)}。每个文件的原绝对路径、交付相对路径、大小、mtime 和 SHA-256 见 [`raw_data_inventory.csv`](index/raw_data_inventory.csv)。复制过程不修改、格式化、删除或重排 raw/ 文件内容。

主要大文件：

{large_raw_md}

## 关键证据边界

- P4 input-ready 是上游输出代理，不是严格 Reader queue 时间。
- `e2e_trace_v3 mono_ns` 与 Perception log `CP_INFER mono_ns` 是分开的数值基准；本交接按 source frame/sensor timestamp 关联，不直接相减，也不静默套用 perf 的 offset。
- `Running` 仅表示 CPU-resident，不能单独解释为有效计算或硬件等待。
- per-frame perf extract 是现有审计筛选后的 raw-event 行；完整 system-wide 数据须查看 capture-wide `perf_sched_script.txt` / `perf.data`。
- CPU stack sample percentage 是样本比例，不是精确 wall time 或 CPU ms。
- 本材料不重新判定根因，不把 association/correlation/observed path 改写为 causal conclusion。
"""
    write_text(HANDOFF / "README.md", readme)

    handoff_doc = f"""# Apollo Perception P4 实时性异常 OS 层交接

## 1. Case 范围

本交接关注 run `202608271537` 中 Apollo Perception P4 LidarDetection 的应用层实时性异常，以及已经完成的 perf sched / CPU stack 采集与对齐。交付目标是让 OS 同事能从 source frame 追到时间、实际 Host TID、capture-wide perf 原始数据和完整 stack samples；不重新寻找或判定根因。

S4 的应用层完整异常段为 F611–F649；当前具有明显 CPU Running inflation 且完成重点 CPU stack 整理的核心窗口为 F611–F623，两者不是同一概念。

## 2. 实时性问题发现步骤

Module deadline → P1–P7 node deadline → P4 LidarDetection → 连续异常段 → perf sched → CPU stack。详细定义、分母、指标和证据路径见 [`01_problem_discovery.md`](01_problem_discovery.md)。

## 3. 应用层已经观测到的问题

- Perception module：{module['deadline_miss_count']} / {module['evaluated_instances']} deadline miss（{float(module['deadline_miss_rate']) * 100:.2f}%）；输入周期 P50 {float(module['budget_p50_ms']):.3f} ms；response P50/P95/MAX = {float(module['response_p50_ms']):.3f}/{float(module['response_p95_ms']):.3f}/{float(module['response_max_ms']):.3f} ms。
- P4：{sections_metrics['p4']['deadline_miss']} / {sections_metrics['p4']['evaluated']} miss（{sections_metrics['p4']['miss_rate'] * 100:.2f}%）；execution-driven {sections_metrics['p4']['execution_driven']}，waiting-driven {sections_metrics['p4']['waiting_driven']}，no-service {sections_metrics['p4']['no_service']}。
- 完整连续 miss 段 76 个；四个主要段为 S1 F17–F38、S2 F61–F95、S3 F308–F331、S4 F611–F649。
- P1–P3 未形成大量自身 miss；P5–P7 统计只覆盖实际从 P4 到达的实例。

## 4. 系统层已经观测到的问题

S4 的既有 perf 结果记录：F601–F610 参考帧 Infer 约 81–87 ms、CPU Running 约 29–34 ms；F611/F613/F616/F619/F623 的 Infer 为 276–327 ms、CPU Running 为 250–293 ms（约 89.7%–91.0%）。这些重点帧的 runnable waiting / max scheduler delay 见 [`02_system_observation.md`](02_system_observation.md) 逐帧表。

现有 perf 数据观察到严重异常窗口中 CPU Running 明显增加，而 runnable waiting / scheduler delay 相对较小。该观测不在本交接中继续扩展为 scheduler 或其他 OS 根因结论。

## 5. CPU Stack 已采集内容

F611、F613、F616、F619、F623 均保留严格 Infer 窗口内全部 stack samples，包含普通 Paddle/cuDNN 和已有 allocator/`cudaMalloc`/kernel path 样本，不去重、不只保留“有意义”样本。逐帧链接位于本文件第 9 节；已观察函数路径与证据边界见 [`02_system_observation.md`](02_system_observation.md)。

## 6. 重点异常窗口

- F601–F610：异常前参考（原审计角色 `candidate_control`）。
- F611–F623：严重 CPU Running inflation 核心窗口；有唯一 Infer 实例的重点帧为 F611/F613/F616/F619/F623。
- F624–F625：core 与 recovery 边界之间的 no-service 帧，不强行归入任一 phase。
- F626–F649：执行状态恢复参考；F640 无唯一 Infer 映射。

## 7. Frame / Timestamp / TID 对照

打开 [`frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。时钟、单位、校准和空值规则见 [`03_data_mapping.md`](03_data_mapping.md)。其中 `e2e_trace_v3 mono_ns` 与 Perception log `CP_INFER mono_ns` 分域保留，不直接相减；perf sched 只使用已经单独校准到 CP Infer/CPU stack 时间域的窗口。

## 8. perf 原始数据

- 原始二进制：[`perf.data`](raw/perf_sched/perf.data)
- 原始 sched script：[`perf_sched_script.txt`](raw/perf_sched/perf_sched_script.txt)
- 原始 timehist：[`perf_sched_timehist.txt`](raw/perf_sched/perf_sched_timehist.txt)
- 原始 map：[`perf_sched_map.txt`](raw/perf_sched/perf_sched_map.txt)
- collector 命令、stderr/stdout、latency 和状态文件：[`raw/perf_sched`](raw/perf_sched/)
- TID 映射证据：[`raw/tid_mapping`](raw/tid_mapping/)

## 9. CPU Stack 原始数据

- capture-wide 原始二进制：[`perf_cpu_stack.data`](raw/cpu_stack/perf_cpu_stack.data)
- capture-wide perf script：[`perf_cpu_stack_script.txt`](raw/cpu_stack/perf_cpu_stack_script.txt)
- 五帧逐帧完整 samples：

{core_links}

## 10. OS 层后续分析入口

以下问题尚未在本交接材料中继续分析，可由 OS 层基于原始数据进一步判断。

- 从 F611 的 `frame_system_trace_index.csv` 行核对应用层/Infer/perf 三个时间域与 Host TID，再打开 F611 perf 与 stack 文件。
- 依次对照 F613、F616、F619、F623，必要时回到 capture-wide `perf_sched_script.txt`、`perf_cpu_stack_script.txt` 或两个 `.data` 文件重新生成视图。
- 使用 `raw/tid_mapping/` 中的 before/after `/proc` 与 Host `ps` 快照复核线程身份。
- 对尚未回答的问题保持为待分析项，不从本交接中的函数路径自动推导因果关系。
"""
    write_text(HANDOFF / "SYSTEM_REALTIME_ISSUE_HANDOFF.md", handoff_doc)

    audit = {
        "source_run": "202608271537",
        "delivery_directory": str(HANDOFF),
        "raw_file_count": len(inventory),
        "raw_total_bytes": total_raw_bytes,
        "raw_total_human": human_bytes(total_raw_bytes),
        "raw_copy_method": "byte-for-byte stream copy; SHA-256 computed from source bytes while copying; destination size checked",
        "raw_copy_size_checks_passed": True,
        "destination_sha256_verification": {
            "hashed_files": len(inventory),
            "sha256_mismatches": len(destination_hash_mismatches),
            "status": "passed" if not destination_hash_mismatches else "failed",
        },
        "clock_domain_boundary": {
            "e2e_trace_mono_ns_vs_cp_infer_mono_ns": "separate numeric bases; do not subtract directly; no explicit trace-to-CP offset present in current artifacts",
            "frame_association": "source frame / sensor timestamp mapping plus unique adjacent CP pair",
            "sched_to_cp_infer_offset_ns": calibration["best_sched_minus_stack_offset_ns"],
        },
        "five_core_frames": five_frame_checks,
        "all_five_core_frames_complete": all_five_complete,
        "frames_without_unique_infer_mapping_F601_F649": missing_infer_frames,
        "per_frame_perf_extract_event_counts": {f"F{frame}": count for frame, count in sorted(perf_event_count_by_frame.items())},
        "phase_boundaries": {
            "normal": "F601-F610",
            "core_abnormal": "F611-F623",
            "unassigned_boundary": "F624-F625",
            "recovery": "F626-F649",
        },
        "evidence_policy": "No adjacent-frame imputation, deduplication, sample filtering, or new root-cause conclusion.",
    }
    write_text(HANDOFF / "index" / "integrity_audit.json", json.dumps(audit, ensure_ascii=False, indent=2) + "\n")

    if not all_five_complete:
        raise AssertionError("At least one core frame failed the completeness audit")
    if any(extracted_sample_counts[frame] != int(stack_summary_by_frame[frame]["stack_sample_count"]) for frame in CORE_FRAMES):
        raise AssertionError("Extracted CPU stack sample count mismatch")
    print(json.dumps({"raw_file_count": len(inventory), "raw_total_bytes": total_raw_bytes, "all_five_complete": all_five_complete}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
