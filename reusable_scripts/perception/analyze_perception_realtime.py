#!/usr/bin/env python3
"""Per-run PDF-style realtime audit for the seven second-experiment baselines.

This is the trace/log-only adaptation of the 202608241701 analysis.  Baseline
runs do not contain the record exports required for exact per-frame
sensor-to-Bridge joins, so that metric remains explicitly unavailable.  The
P1-P7 upstream-output proxy, full Perception freshness deadline, CenterPoint
breakdown, and buffer-overflow evidence retain the same definitions as the
prior notebook.  The requested scope is Perception-only.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import pandas as pd


WORKSPACE = Path(r"D:\data")
BASELINE = WORKSPACE / "第二次实验" / "baseline"
CONTROL_PERIOD_MS = 10.0
LOCAL_TZ = ZoneInfo("Asia/Shanghai")

NODE_DEFS = [
    (1, "P1", "PointCloudPreprocess", "pointcloud_preprocess"),
    (2, "P2", "MapBasedROI", "pointcloud_map_based_roi"),
    (3, "P3", "GroundDetection", "pointcloud_ground_detection"),
    (4, "P4", "LidarDetection", "lidar_detection"),
    (5, "P5", "LidarDetectionFilter", "lidar_detection_filter"),
    (6, "P6", "LidarTracking", "lidar_tracking"),
    (7, "P7", "MultiSensorFusion", "multi_sensor_fusion"),
]

FRAME_RE = re.compile(
    r"\[FUSION_OBS_FRAME\].*?seq=(\d+).*?trace_id=(\d+).*?"
    r"header_time=([\d.eE+-]+).*?lidar_timestamp=(\d+).*?obstacle_count=(\d+)"
)
POINTS_RE = re.compile(r"num points before fusing:\s*(\d+)")
BREAKDOWN_RE = re.compile(
    r"down sample:\s*([\d.]+)\s+fuse:\s*([\d.]+)\s+shuffle:\s*([\d.]+)\s+"
    r"cloud_to_array:\s*([\d.]+)\s+inference:\s*([\d.]+)\s+"
    r"postprocess:\s*([\d.]+)\s+nms:\s*([\d.]+)"
)
DETECTION_END_RE = re.compile(r"FRAME_STATISTICS:LidarDetection:End:msg_time\[([\d.]+)\]")
OVERFLOW_RE = re.compile(
    r"^W(\d{4})\s+(\d\d:\d\d:\d\d\.\d+)\s+\d+.*"
    r"channel\[([^\]]+)\] read buffer overflow, drop_message\[(\d+)\]"
)


def one(root: Path, pattern: str) -> Path:
    matches = sorted(root.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one {pattern!r} under {root}, found {len(matches)}")
    return matches[0]


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def percentile(values: pd.Series, q: float) -> float | None:
    numeric = pd.to_numeric(values, errors="coerce").dropna()
    return float(numeric.quantile(q / 100.0)) if len(numeric) else None


def iso_local(epoch_s: float) -> str:
    return datetime.fromtimestamp(epoch_s, LOCAL_TZ).isoformat(timespec="microseconds")


def read_events(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, dtype=str)
    frame["mono_ns"] = pd.to_numeric(frame["mono_ns"], errors="coerce").astype("Int64")
    return frame


def event_ns(events: dict[str, pd.DataFrame], stage: str, trace_id: str, phase: str) -> int | None:
    frame = events[stage]
    rows = frame[frame["trace_id"].eq(str(trace_id)) & frame["phase"].eq(phase)]["mono_ns"].dropna()
    return int(rows.min()) if len(rows) else None


def delta_ms(later: int | None, earlier: int | None) -> float | None:
    if later is None or earlier is None:
        return None
    return (later - earlier) / 1e6


def parse_fusion_window(perception_log: Path) -> tuple[pd.DataFrame, dict[str, object]]:
    rows: list[dict[str, object]] = []
    with perception_log.open(encoding="utf-8", errors="ignore") as handle:
        for line_number, line in enumerate(handle, 1):
            match = FRAME_RE.search(line)
            if not match:
                continue
            seq, trace_id, header, lidar_ns, count = match.groups()
            rows.append(
                {
                    "sequence": int(seq),
                    "fusion_trace_id": trace_id,
                    "header_time_s": float(header),
                    "sensor_time_s": int(lidar_ns) / 1e9,
                    "obstacle_count": int(count),
                    "log_source_line": line_number,
                }
            )
    frame = pd.DataFrame(rows).sort_values("sensor_time_s").reset_index(drop=True)
    positive = frame[frame["obstacle_count"].gt(0)].copy()
    if positive.empty:
        return frame, {
            "selection_status": "fallback_all_fusion_frames_no_positive_obstacle_episode",
            "onset_s": float(frame["sensor_time_s"].min()),
            "end_s": float(frame["sensor_time_s"].max()),
            "positive_frame_count": 0,
        }
    positive["episode"] = positive["sensor_time_s"].diff().gt(1.0).cumsum()
    groups = list(positive.groupby("episode", sort=False))
    _, selected = max(groups, key=lambda item: (len(item[1]), item[1]["sensor_time_s"].max()))
    return frame, {
        "selection_status": "largest_positive_obstacle_episode",
        "onset_s": float(selected["sensor_time_s"].min()),
        "end_s": float(selected["sensor_time_s"].max()),
        "positive_frame_count": int(len(selected)),
        "positive_episode_count": int(len(groups)),
    }


def build_sources(
    run_dir: Path,
    perception_log: Path,
    window_mode: str = "largest-positive-episode",
) -> tuple[pd.DataFrame, dict[str, object]]:
    anchors_path = one(run_dir, "trace/trace_anchor/perception.*.csv")
    inputs_path = one(run_dir, "trace/fusion_inputs/perception.multi_sensor_fusion.*.csv")
    anchors = pd.read_csv(anchors_path, dtype=str)
    anchors = anchors[anchors["sensor_kind"].eq("lidar")].copy()
    anchors["data_ts_ns"] = pd.to_numeric(anchors["data_ts_ns"], errors="coerce").astype("Int64")
    anchors["sensor_time_sec"] = pd.to_numeric(anchors["sensor_time_sec"], errors="coerce")
    anchors["preproc_enter_ns"] = pd.to_numeric(anchors["preproc_enter_ns"], errors="coerce").astype("Int64")
    anchors["ingress_ms"] = pd.to_numeric(anchors["ingress_ms"], errors="coerce")

    fusion_frames, window = parse_fusion_window(perception_log)
    if window_mode == "all":
        if fusion_frames.empty:
            raise RuntimeError("perception log has no FUSION_OBS_FRAME records")
        window = {
            **window,
            "selection_status": "all_fusion_frames",
            "onset_s": float(fusion_frames["sensor_time_s"].min()),
            "end_s": float(fusion_frames["sensor_time_s"].max()),
            "fusion_frame_count": int(len(fusion_frames)),
        }
    onset_s = float(window["onset_s"])
    end_s = float(window["end_s"])
    selected = anchors[
        anchors["sensor_time_sec"].between(onset_s - 0.002, end_s + 0.002, inclusive="both")
    ].copy()
    selected = selected.sort_values("sensor_time_sec").drop_duplicates("trace_id").reset_index(drop=True)
    if selected.empty:
        raise RuntimeError("selected obstacle episode has no lidar trace anchors")

    inputs = pd.read_csv(inputs_path, dtype=str)
    inputs = inputs[
        inputs["sensor_kind"].eq("lidar") & inputs["is_main_sensor"].isin(["1", "true", "True"])
    ].drop_duplicates("parent_trace_id")
    selected = selected.merge(
        inputs[["fusion_trace_id", "parent_trace_id", "object_count"]],
        left_on="trace_id",
        right_on="parent_trace_id",
        how="left",
        validate="one_to_one",
    )
    selected["source_frame_index"] = np.arange(1, len(selected) + 1)
    selected["relative_source_time_s"] = selected["sensor_time_sec"] - float(selected["sensor_time_sec"].min())
    selected["source_time_local"] = selected["sensor_time_sec"].map(iso_local)
    selected["object_count"] = pd.to_numeric(selected["object_count"], errors="coerce")
    return selected, {
        **window,
        "anchor_file": str(anchors_path),
        "fusion_inputs_file": str(inputs_path),
        "selected_source_frames": int(len(selected)),
        "selected_onset_local": iso_local(float(selected["sensor_time_sec"].min())),
        "selected_end_local": iso_local(float(selected["sensor_time_sec"].max())),
    }


def build_node_audit(
    run_id: str,
    sources: pd.DataFrame,
    events: dict[str, pd.DataFrame],
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    timing_rows: list[dict[str, object]] = []
    for row in sources.itertuples(index=False):
        frame = int(row.source_frame_index)
        lidar = str(row.trace_id)
        fusion = str(row.fusion_trace_id) if pd.notna(row.fusion_trace_id) else ""
        for node_index, symbol, name, stage in NODE_DEFS:
            start = event_ns(events, stage, lidar, "proc_enter")
            finish = (
                event_ns(events, stage, fusion, "output_pub")
                if symbol == "P7" and fusion
                else event_ns(events, stage, lidar, "output_pub")
            )
            timing_rows.append(
                {
                    "run_id": run_id,
                    "source_frame_index": frame,
                    "source_time_unix_s": float(row.sensor_time_sec),
                    "source_time_local": row.source_time_local,
                    "relative_source_time_s": float(row.relative_source_time_s),
                    "lidar_trace_id": lidar,
                    "fusion_trace_id": fusion,
                    "node_index": node_index,
                    "node_symbol": symbol,
                    "node_name": name,
                    "stage": stage,
                    "proc_enter_mono_ns": start,
                    "output_pub_mono_ns": finish,
                    "execution_observed_ms": delta_ms(finish, start),
                    "complete": bool(start is not None and finish is not None and finish >= start),
                }
            )
    timings = pd.DataFrame(timing_rows).sort_values(["node_index", "source_frame_index"])
    lookup = {
        (str(row.node_symbol), int(row.source_frame_index)): row
        for row in timings.itertuples(index=False)
    }
    output: list[dict[str, object]] = []
    for node_index, symbol, name, stage in NODE_DEFS:
        rows: list[dict[str, object]] = []
        for frame in sources["source_frame_index"].astype(int):
            current = lookup[(symbol, int(frame))]
            start = int(current.proc_enter_mono_ns) if pd.notna(current.proc_enter_mono_ns) else None
            finish = int(current.output_pub_mono_ns) if pd.notna(current.output_pub_mono_ns) else None
            if node_index == 1:
                ready = start
                basis = "callback_start_proxy"
            else:
                upstream = lookup[(f"P{node_index - 1}", int(frame))]
                ready = int(upstream.output_pub_mono_ns) if pd.notna(upstream.output_pub_mono_ns) else None
                basis = "upstream_output_pub_proxy"
            if ready is None:
                lifecycle = "upstream_not_reached"
            elif start is None:
                lifecycle = "input_ready_but_no_service_observed"
            elif finish is None:
                lifecycle = "service_started_output_missing"
            else:
                lifecycle = "completed"
            rows.append(
                {
                    "run_id": run_id,
                    "source_frame_index": int(frame),
                    "source_time_unix_s": current.source_time_unix_s,
                    "source_time_local": current.source_time_local,
                    "relative_source_time_s": current.relative_source_time_s,
                    "lidar_trace_id": current.lidar_trace_id,
                    "fusion_trace_id": current.fusion_trace_id,
                    "node_index": node_index,
                    "node_symbol": symbol,
                    "node_name": name,
                    "stage": stage,
                    "input_ready_basis": basis,
                    "strict_reader_arrival_available": False,
                    "input_ready_proxy_mono_ns": ready,
                    "proc_enter_mono_ns": start,
                    "output_pub_mono_ns": finish,
                    "waiting_proxy_ms": delta_ms(start, ready),
                    "execution_observed_ms": delta_ms(finish, start),
                    "response_proxy_ms": delta_ms(finish, ready),
                    "lifecycle_status": lifecycle,
                }
            )
        arrivals = [row for row in rows if row["input_ready_proxy_mono_ns"] is not None]
        arrivals.sort(key=lambda item: int(item["input_ready_proxy_mono_ns"]))
        next_ready = {int(a["source_frame_index"]): b for a, b in zip(arrivals, arrivals[1:])}
        for row in rows:
            following = next_ready.get(int(row["source_frame_index"]))
            ready = row["input_ready_proxy_mono_ns"]
            finish = row["output_pub_mono_ns"]
            execution = row["execution_observed_ms"]
            budget = (
                delta_ms(int(following["input_ready_proxy_mono_ns"]), int(ready))
                if following is not None and ready is not None
                else None
            )
            slack = (
                delta_ms(int(following["input_ready_proxy_mono_ns"]), int(finish))
                if following is not None and finish is not None
                else None
            )
            evaluable = following is not None and ready is not None
            if ready is None:
                miss, result = None, "upstream_not_reached"
            elif following is None:
                miss, result = None, "right_censored_no_next_arrival"
            elif row["lifecycle_status"] != "completed":
                miss, result = True, "no_service_observed"
            elif slack is not None and slack < 0:
                miss = True
                result = (
                    "execution_driven_miss"
                    if execution is not None and budget is not None and execution > budget
                    else "waiting_driven_miss"
                )
            else:
                miss, result = False, "pass"
            row.update(
                {
                    "next_arrival_source_frame_index": (
                        int(following["source_frame_index"]) if following is not None else None
                    ),
                    "next_input_ready_proxy_mono_ns": (
                        int(following["input_ready_proxy_mono_ns"]) if following is not None else None
                    ),
                    "deadline_budget_proxy_ms": budget,
                    "slack_proxy_ms": slack,
                    "deadline_evaluable_proxy": evaluable,
                    "deadline_miss_proxy": miss,
                    "proxy_result": result,
                    "evidence_boundary": (
                        "Reader receive/enqueue/drop markers unavailable; input-ready is a pipeline proxy"
                    ),
                }
            )
            output.append(row)
    audit = pd.DataFrame(output).sort_values(["node_index", "source_frame_index"])
    summaries: list[dict[str, object]] = []
    for node_index, symbol, name, _ in NODE_DEFS:
        node = audit[audit["node_symbol"].eq(symbol)]
        evaluable = node[node["deadline_evaluable_proxy"].eq(True)]
        counts = node["proxy_result"].value_counts()
        summaries.append(
            {
                "run_id": run_id,
                "node_index": node_index,
                "node_symbol": symbol,
                "node_name": name,
                "source_frames": int(len(node)),
                "evaluated_instances": int(len(evaluable)),
                "deadline_miss_count": int(evaluable["deadline_miss_proxy"].eq(True).sum()),
                "deadline_miss_rate": float(evaluable["deadline_miss_proxy"].eq(True).mean()) if len(evaluable) else None,
                "pass_count": int(counts.get("pass", 0)),
                "execution_driven_miss_count": int(counts.get("execution_driven_miss", 0)),
                "waiting_driven_miss_count": int(counts.get("waiting_driven_miss", 0)),
                "no_service_observed_count": int(counts.get("no_service_observed", 0)),
                "upstream_not_reached_count": int(counts.get("upstream_not_reached", 0)),
                "right_censored_count": int(counts.get("right_censored_no_next_arrival", 0)),
                "waiting_proxy_p50_ms": percentile(node["waiting_proxy_ms"], 50),
                "waiting_proxy_max_ms": pd.to_numeric(node["waiting_proxy_ms"], errors="coerce").max(),
                "execution_p50_ms": percentile(node["execution_observed_ms"], 50),
                "execution_p95_ms": percentile(node["execution_observed_ms"], 95),
                "execution_max_ms": pd.to_numeric(node["execution_observed_ms"], errors="coerce").max(),
                "slack_min_ms": pd.to_numeric(evaluable["slack_proxy_ms"], errors="coerce").min(),
            }
        )
    return timings, audit, pd.DataFrame(summaries)


def add_p4_change_metrics(p4: pd.DataFrame) -> tuple[pd.DataFrame, dict[str, object]]:
    """Add reproducible adjacent-frame P4 execution-change diagnostics.

    A sudden increase must be both materially large (robust delta threshold,
    never below 30 ms) and at least 1.5x the immediately preceding source
    frame. Missing/no-service frames remain unavailable rather than being
    bridged across.
    """
    result = p4.sort_values("source_frame_index").copy()
    execution = pd.to_numeric(result["execution_observed_ms"], errors="coerce")
    previous_execution = execution.shift(1)
    delta = execution - previous_execution
    ratio = execution / previous_execution.where(previous_execution.gt(0))
    valid_delta = delta.dropna()
    median_delta = float(valid_delta.median()) if len(valid_delta) else 0.0
    mad_delta = (
        float((valid_delta - median_delta).abs().median()) if len(valid_delta) else 0.0
    )
    robust_delta_threshold_ms = max(30.0, median_delta + 6.0 * 1.4826 * mad_delta)

    result["previous_source_frame_index"] = result["source_frame_index"].shift(1).astype("Int64")
    result["previous_execution_observed_ms"] = previous_execution
    result["execution_delta_from_previous_frame_ms"] = delta
    result["execution_ratio_to_previous_frame"] = ratio
    result["waiting_delta_from_previous_frame_ms"] = (
        pd.to_numeric(result["waiting_proxy_ms"], errors="coerce").diff()
    )
    result["response_delta_from_previous_frame_ms"] = (
        pd.to_numeric(result["response_proxy_ms"], errors="coerce").diff()
    )
    result["execution_sudden_increase_flag"] = (
        delta.ge(robust_delta_threshold_ms) & ratio.ge(1.5)
    )
    flagged = result[result["execution_sudden_increase_flag"]]
    max_delta = delta.max()
    max_ratio = ratio.max()
    diagnostics = {
        "observed": bool(len(flagged)),
        "rule": (
            "adjacent source frames both have P4 execution; execution delta >= "
            "max(30 ms, median adjacent delta + 6*1.4826*MAD) and ratio >= 1.5"
        ),
        "robust_delta_threshold_ms": robust_delta_threshold_ms,
        "flagged_frame_count": int(len(flagged)),
        "flagged_source_frames": flagged["source_frame_index"].astype(int).tolist(),
        "max_positive_adjacent_delta_ms": float(max_delta) if pd.notna(max_delta) else None,
        "max_adjacent_ratio": float(max_ratio) if pd.notna(max_ratio) else None,
    }
    return result, diagnostics


def event_deadlines(
    run_id: str,
    module: str,
    instances: list[dict[str, object]],
) -> list[dict[str, object]]:
    ordered = sorted(
        (item for item in instances if item["input_mono_ns"] is not None),
        key=lambda item: int(item["input_mono_ns"]),
    )
    rows: list[dict[str, object]] = []
    for index, current in enumerate(ordered):
        input_ns = int(current["input_mono_ns"])
        output_ns = int(current["output_mono_ns"]) if current["output_mono_ns"] is not None else None
        next_ns = int(ordered[index + 1]["input_mono_ns"]) if index + 1 < len(ordered) else None
        evaluable = next_ns is not None
        miss = None if not evaluable else output_ns is None or output_ns > next_ns
        rows.append(
            {
                "run_id": run_id,
                "module": module,
                "trigger_type": "event",
                "instance_id": current["instance_id"],
                "source_frame_index": current.get("source_frame_index"),
                "input_mono_ns": input_ns,
                "output_mono_ns": output_ns,
                "next_input_mono_ns": next_ns,
                "deadline_budget_ms": delta_ms(next_ns, input_ns),
                "response_time_ms": delta_ms(output_ns, input_ns),
                "slack_ms": delta_ms(next_ns, output_ns),
                "deadline_evaluable": evaluable,
                "deadline_miss": miss,
                "deadline_result_reason": (
                    "last_arrival_has_no_next_arrival"
                    if not evaluable
                    else "no_output_before_next_input"
                    if output_ns is None
                    else "output_after_next_input"
                    if miss
                    else "output_before_or_at_next_input"
                ),
            }
        )
    return rows


def build_module_deadlines(
    run_id: str,
    sources: pd.DataFrame,
    events: dict[str, pd.DataFrame],
) -> tuple[pd.DataFrame, pd.DataFrame]:
    detail: list[dict[str, object]] = []
    perception_instances: list[dict[str, object]] = []
    downstream: dict[str, list[dict[str, object]]] = {"prediction": [], "planning": []}
    fusion_traces: set[str] = set()
    for row in sources.itertuples(index=False):
        lidar = str(row.trace_id)
        fusion = str(row.fusion_trace_id) if pd.notna(row.fusion_trace_id) else ""
        frame = int(row.source_frame_index)
        perception_instances.append(
            {
                "instance_id": lidar,
                "source_frame_index": frame,
                "input_mono_ns": event_ns(events, "pointcloud_preprocess", lidar, "proc_enter"),
                "output_mono_ns": event_ns(events, "multi_sensor_fusion", fusion, "output_pub") if fusion else None,
            }
        )
        if fusion:
            fusion_traces.add(fusion)
            for module in ("prediction", "planning"):
                downstream[module].append(
                    {
                        "instance_id": fusion,
                        "source_frame_index": frame,
                        "input_mono_ns": event_ns(events, module, fusion, "proc_enter"),
                        "output_mono_ns": event_ns(events, module, fusion, "writer_done"),
                    }
                )
    detail.extend(event_deadlines(run_id, "perception", perception_instances))
    for module in ("prediction", "planning"):
        detail.extend(event_deadlines(run_id, module, downstream[module]))

    control = events["control"]
    for proc_id, group in control.groupby("proc_id", sort=False):
        traces = group.loc[group["trace_id"].ne("0"), "trace_id"].dropna().unique()
        if len(traces) != 1 or str(traces[0]) not in fusion_traces:
            continue
        phases = group.groupby("phase")["mono_ns"].min()
        start = int(phases["proc_enter"]) if "proc_enter" in phases and pd.notna(phases["proc_enter"]) else None
        finish = int(phases["cmd_write_done"]) if "cmd_write_done" in phases and pd.notna(phases["cmd_write_done"]) else None
        response = delta_ms(finish, start)
        miss = finish is None or (response is not None and response > CONTROL_PERIOD_MS)
        detail.append(
            {
                "run_id": run_id,
                "module": "control",
                "trigger_type": "periodic_100Hz",
                "instance_id": f"{traces[0]}:{proc_id}",
                "source_frame_index": None,
                "input_mono_ns": start,
                "output_mono_ns": finish,
                "next_input_mono_ns": None,
                "deadline_budget_ms": CONTROL_PERIOD_MS,
                "response_time_ms": response,
                "slack_ms": CONTROL_PERIOD_MS - response if response is not None else None,
                "deadline_evaluable": start is not None,
                "deadline_miss": miss,
                "deadline_result_reason": (
                    "output_missing" if finish is None else "execution_exceeds_10ms" if miss else "execution_within_10ms"
                ),
            }
        )
    detail_frame = pd.DataFrame(detail)
    summaries: list[dict[str, object]] = []
    for module in ("perception", "prediction", "planning", "control"):
        group = detail_frame[detail_frame["module"].eq(module)]
        evaluated = group[group["deadline_evaluable"].eq(True)]
        misses = int(evaluated["deadline_miss"].eq(True).sum())
        summaries.append(
            {
                "run_id": run_id,
                "module": module,
                "evaluated_instances": int(len(evaluated)),
                "deadline_miss_count": misses,
                "deadline_miss_rate": misses / len(evaluated) if len(evaluated) else None,
                "budget_p50_ms": percentile(evaluated["deadline_budget_ms"], 50),
                "response_p50_ms": percentile(evaluated["response_time_ms"], 50),
                "response_p95_ms": percentile(evaluated["response_time_ms"], 95),
                "response_max_ms": pd.to_numeric(evaluated["response_time_ms"], errors="coerce").max(),
                "slack_min_ms": pd.to_numeric(evaluated["slack_ms"], errors="coerce").min(),
            }
        )
    return detail_frame, pd.DataFrame(summaries)


def build_perception_pipeline_deadlines(
    run_id: str,
    sources: pd.DataFrame,
    events: dict[str, pd.DataFrame],
) -> tuple[pd.DataFrame, pd.DataFrame]:
    instances: list[dict[str, object]] = []
    for row in sources.itertuples(index=False):
        lidar = str(row.trace_id)
        fusion = str(row.fusion_trace_id) if pd.notna(row.fusion_trace_id) else ""
        instances.append(
            {
                "instance_id": lidar,
                "source_frame_index": int(row.source_frame_index),
                "input_mono_ns": event_ns(events, "pointcloud_preprocess", lidar, "proc_enter"),
                "output_mono_ns": event_ns(events, "multi_sensor_fusion", fusion, "output_pub") if fusion else None,
            }
        )
    detail = pd.DataFrame(event_deadlines(run_id, "perception", instances))
    evaluated = detail[detail["deadline_evaluable"].eq(True)]
    misses = int(evaluated["deadline_miss"].eq(True).sum())
    summary = pd.DataFrame(
        [
            {
                "run_id": run_id,
                "module": "perception",
                "evaluated_instances": int(len(evaluated)),
                "deadline_miss_count": misses,
                "deadline_miss_rate": misses / len(evaluated) if len(evaluated) else None,
                "budget_p50_ms": percentile(evaluated["deadline_budget_ms"], 50),
                "response_p50_ms": percentile(evaluated["response_time_ms"], 50),
                "response_p95_ms": percentile(evaluated["response_time_ms"], 95),
                "response_max_ms": pd.to_numeric(evaluated["response_time_ms"], errors="coerce").max(),
                "slack_min_ms": pd.to_numeric(evaluated["slack_ms"], errors="coerce").min(),
            }
        ]
    )
    return detail, summary


def parse_centerpoint(
    run_id: str,
    log: Path,
    sources: pd.DataFrame,
    p4: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    source_times = sources.set_index("source_frame_index")["sensor_time_sec"]
    frames = source_times.index.to_numpy(dtype=int)
    times = source_times.to_numpy(dtype=float)
    start_s, end_s = float(times.min()), float(times.max())
    p4_exec = p4.set_index("source_frame_index")["execution_observed_ms"]
    mapped: dict[int, dict[str, object]] = {}
    overflow_rows: list[dict[str, object]] = []
    latest_points: int | None = None
    latest_breakdown: tuple[float, ...] | None = None
    latest_breakdown_points: int | None = None
    with log.open(encoding="utf-8", errors="ignore") as handle:
        for line_number, line in enumerate(handle, 1):
            points = POINTS_RE.search(line)
            if points:
                latest_points = int(points.group(1))
            breakdown = BREAKDOWN_RE.search(line)
            if breakdown:
                latest_breakdown = tuple(float(value) for value in breakdown.groups())
                latest_breakdown_points = latest_points
            end = DETECTION_END_RE.search(line)
            if end and latest_breakdown is not None:
                msg_time = float(end.group(1))
                nearest = int(np.argmin(np.abs(times - msg_time)))
                residual_ms = abs(float(times[nearest]) - msg_time) * 1000.0
                if residual_ms <= 30.0:
                    frame = int(frames[nearest])
                    downsample, fuse, shuffle, cloud_to_array, inference, postprocess, nms = latest_breakdown
                    execution = pd.to_numeric(pd.Series([p4_exec.get(frame)]), errors="coerce").iloc[0]
                    listed = sum(latest_breakdown)
                    mapped[frame] = {
                        "run_id": run_id,
                        "source_frame_index": frame,
                        "sensor_time_unix_s": float(times[nearest]),
                        "log_msg_time_unix_s": msg_time,
                        "mapping_abs_residual_ms": residual_ms,
                        "log_source_line": line_number,
                        "points_before_fusing": latest_breakdown_points,
                        "downsample_ms": downsample,
                        "fuse_ms": fuse,
                        "shuffle_ms": shuffle,
                        "cloud_to_array_ms": cloud_to_array,
                        "inference_ms": inference,
                        "postprocess_ms": postprocess,
                        "nms_ms": nms,
                        "listed_stage_sum_ms": listed,
                        "lidar_detection_execution_trace_ms": float(execution) if pd.notna(execution) else None,
                        "inference_share_of_trace_execution": (
                            float(inference / execution) if pd.notna(execution) and execution > 0 else None
                        ),
                        "data_status": "complete",
                        "missing_reason": "",
                    }
                latest_breakdown = None
                latest_breakdown_points = None
            overflow = OVERFLOW_RE.search(line)
            if overflow:
                md, clock, channel, dropped = overflow.groups()
                wall = datetime.strptime(f"{run_id[:4]}{md} {clock}", "%Y%m%d %H:%M:%S.%f").replace(tzinfo=LOCAL_TZ)
                wall_s = wall.timestamp()
                if start_s - 0.25 <= wall_s <= end_s + 0.25:
                    overflow_rows.append(
                        {
                            "run_id": run_id,
                            "log_source_line": line_number,
                            "wall_time_local": wall.isoformat(timespec="microseconds"),
                            "wall_time_unix_s": wall_s,
                            "channel": channel,
                            "drop_message_count": int(dropped),
                        }
                    )
    rows: list[dict[str, object]] = []
    for frame, sensor_time in source_times.items():
        if int(frame) in mapped:
            rows.append(mapped[int(frame)])
        else:
            rows.append(
                {
                    "run_id": run_id,
                    "source_frame_index": int(frame),
                    "sensor_time_unix_s": float(sensor_time),
                    "data_status": "unavailable",
                    "missing_reason": "no uniquely mapped CenterPoint breakdown in selected window",
                }
            )
    return pd.DataFrame(rows).sort_values("source_frame_index"), pd.DataFrame(overflow_rows)


def classify(
    p4: pd.DataFrame,
    pipeline: pd.DataFrame,
) -> dict[str, object]:
    p4_eval = p4[p4["deadline_evaluable_proxy"].eq(True)].copy()
    pipeline_eval = pipeline[pipeline["deadline_evaluable"].eq(True)].copy()
    counts = p4["proxy_result"].value_counts()
    execution = int(counts.get("execution_driven_miss", 0))
    waiting = int(counts.get("waiting_driven_miss", 0))
    no_service = int(counts.get("no_service_observed", 0))
    p4_misses = int(p4_eval["deadline_miss_proxy"].eq(True).sum())
    pipeline_misses = int(pipeline_eval["deadline_miss"].eq(True).sum())
    # The PDF's acute overload chain is stronger than a few timing misses: it
    # contains execution overrun, residual waiting, and frames that reached the
    # P4 input-ready boundary but received no service.  Require all three so a
    # stable freshness violation without frame loss is not over-labelled.
    overload = execution > 0 and waiting > 0 and no_service > 0

    miss_frames = p4.loc[p4["deadline_miss_proxy"].eq(True), "source_frame_index"]
    recovery = p4.iloc[0:0]
    if len(miss_frames):
        recovery = p4[p4["source_frame_index"].gt(int(miss_frames.max()))]
        first_nonpass = recovery.index[~recovery["proxy_result"].eq("pass")]
        if len(first_nonpass):
            recovery = recovery.loc[: first_nonpass[0] - 1]
    recovery_frames = set(recovery["source_frame_index"].astype(int))
    recovery_pipeline = pipeline_eval[pipeline_eval["source_frame_index"].isin(recovery_frames)]
    recovery_rate = (
        float(recovery_pipeline["deadline_miss"].eq(True).mean()) if len(recovery_pipeline) else None
    )

    tail_frames = set(p4.tail(min(15, len(p4)))["source_frame_index"].astype(int))
    tail_p4 = p4[p4["source_frame_index"].isin(tail_frames)]
    tail_pipeline = pipeline_eval[pipeline_eval["source_frame_index"].isin(tail_frames)]
    tail_p4_pass_rate = float(tail_p4["proxy_result"].eq("pass").mean()) if len(tail_p4) else None
    tail_pipeline_miss_rate = (
        float(tail_pipeline["deadline_miss"].eq(True).mean()) if len(tail_pipeline) else None
    )
    stable_freshness = bool(
        tail_p4_pass_rate is not None
        and tail_p4_pass_rate >= 0.9
        and tail_pipeline_miss_rate is not None
        and tail_pipeline_miss_rate > 0.5
    )
    full_chain = bool(
        overload
        and len(recovery_pipeline) >= 5
        and recovery_rate is not None
        and recovery_rate > 0.5
    )
    if full_chain:
        verdict = "FULL_PDF_FAILURE_CHAIN_REPRODUCED"
    elif overload:
        verdict = "P4_OVERLOAD_CHAIN_REPRODUCED_RECOVERY_NOT_ESTABLISHED"
    elif stable_freshness:
        verdict = "SAME_STABLE_PERCEPTION_FRESHNESS_VIOLATION_WITHOUT_P4_OVERLOAD_CASCADE"
    elif pipeline_misses > 0:
        verdict = "PERCEPTION_FRESHNESS_VIOLATION_DIFFERENT_NODE_PATTERN"
    else:
        verdict = "PDF_LIKE_REALTIME_PROBLEM_NOT_OBSERVED"
    return {
        "verdict": verdict,
        "pdf_full_failure_chain_reproduced": full_chain,
        "p4_overload_cascade_reproduced": overload,
        "stable_perception_freshness_violation_reproduced": stable_freshness,
        "p4_miss_count": p4_misses,
        "p4_evaluated": int(len(p4_eval)),
        "p4_miss_rate": p4_misses / len(p4_eval) if len(p4_eval) else None,
        "p4_execution_driven": execution,
        "p4_waiting_driven": waiting,
        "p4_no_service": no_service,
        "pipeline_miss_count": pipeline_misses,
        "pipeline_evaluated": int(len(pipeline_eval)),
        "pipeline_miss_rate": pipeline_misses / len(pipeline_eval) if len(pipeline_eval) else None,
        "recovery_tail_frame_count": int(len(recovery_pipeline)),
        "recovery_tail_pipeline_miss_rate": recovery_rate,
        "last15_p4_pass_rate": tail_p4_pass_rate,
        "last15_pipeline_miss_rate": tail_pipeline_miss_rate,
    }


def validate(
    run_dir: Path,
    sources: pd.DataFrame,
    node: pd.DataFrame,
    node_summary: pd.DataFrame,
    deadline: pd.DataFrame,
) -> dict[str, object]:
    checks: dict[str, dict[str, object]] = {}
    file_count = sum(1 for path in run_dir.rglob("*") if path.is_file())
    checks["input_file_count"] = {
        "passed": file_count >= 10,
        "evidence": {"observed": file_count, "minimum_required_by_perception_scope": 10},
    }
    duplicate_sources = int(sources["trace_id"].duplicated().sum())
    checks["source_trace_ids_unique"] = {"passed": duplicate_sources == 0, "evidence": {"duplicates": duplicate_sources}}
    duplicate_node = int(node.duplicated(["source_frame_index", "node_symbol"]).sum())
    expected_node = len(sources) * 7
    checks["node_row_grain"] = {
        "passed": len(node) == expected_node and duplicate_node == 0,
        "evidence": {"rows": len(node), "expected": expected_node, "duplicates": duplicate_node},
    }
    complete = node[node["lifecycle_status"].eq("completed")].copy()
    arithmetic = (
        pd.to_numeric(complete["waiting_proxy_ms"], errors="coerce")
        + pd.to_numeric(complete["execution_observed_ms"], errors="coerce")
        - pd.to_numeric(complete["response_proxy_ms"], errors="coerce")
    ).abs()
    max_error = float(arithmetic.max()) if len(arithmetic) else 0.0
    checks["node_response_arithmetic"] = {"passed": max_error < 1e-9, "evidence": {"max_abs_error_ms": max_error}}
    summary_misses = int(node_summary["deadline_miss_count"].sum())
    detail_misses = int(node["deadline_miss_proxy"].eq(True).sum())
    checks["node_summary_matches_detail"] = {
        "passed": summary_misses == detail_misses,
        "evidence": {"summary": summary_misses, "detail": detail_misses},
    }
    event_rows = deadline[deadline["trigger_type"].eq("event") & deadline["deadline_evaluable"].eq(True)]
    recomputed = event_rows.apply(
        lambda row: row["output_mono_ns"] is None
        or pd.isna(row["output_mono_ns"])
        or int(row["output_mono_ns"]) > int(row["next_input_mono_ns"]),
        axis=1,
    )
    mismatches = int((recomputed.to_numpy() != event_rows["deadline_miss"].astype(bool).to_numpy()).sum())
    checks["module_deadline_flags_recomputed"] = {"passed": mismatches == 0, "evidence": {"mismatches": mismatches}}
    passed = sum(bool(item["passed"]) for item in checks.values())
    return {
        "status": "pass" if passed == len(checks) else "fail",
        "check_count": len(checks),
        "passed_count": passed,
        "checks": checks,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_id", nargs="?")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "output directory; defaults to "
            "<run-dir>/打点逐帧数据统计/perception数据统计"
        ),
    )
    parser.add_argument(
        "--window",
        choices=("largest-positive-episode", "all"),
        default="largest-positive-episode",
    )
    args = parser.parse_args()
    if args.run_dir is None and args.run_id is None:
        parser.error("run_id or --run-dir is required")
    run_dir = args.run_dir.resolve() if args.run_dir is not None else BASELINE / args.run_id
    run_id = args.run_id or run_dir.name
    if not run_dir.is_dir():
        raise SystemExit(f"run directory not found: {run_dir}")
    out = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else run_dir / "打点逐帧数据统计" / "perception数据统计"
    )
    data = out / "data"
    report = out / "report"
    validation_dir = out / "validation"
    for directory in (data, report, validation_dir):
        directory.mkdir(parents=True, exist_ok=True)

    perception_log = one(run_dir, "log/perception.log.INFO.*")
    stage_paths = {
        "pointcloud_preprocess": one(run_dir, "trace/events/perception.pointcloud_preprocess.*.csv"),
        "pointcloud_map_based_roi": one(run_dir, "trace/events/perception.pointcloud_map_based_roi.*.csv"),
        "pointcloud_ground_detection": one(run_dir, "trace/events/perception.pointcloud_ground_detection.*.csv"),
        "lidar_detection": one(run_dir, "trace/events/perception.lidar_detection.*.csv"),
        "lidar_detection_filter": one(run_dir, "trace/events/perception.lidar_detection_filter.*.csv"),
        "lidar_tracking": one(run_dir, "trace/events/perception.lidar_tracking.*.csv"),
        "multi_sensor_fusion": one(run_dir, "trace/events/perception.multi_sensor_fusion.*.csv"),
    }
    events = {name: read_events(path) for name, path in stage_paths.items()}
    sources, window = build_sources(run_dir, perception_log, args.window)
    timings, node, node_summary = build_node_audit(run_id, sources, events)
    deadline, deadline_summary = build_perception_pipeline_deadlines(run_id, sources, events)
    p4 = node[node["node_symbol"].eq("P4")].copy()
    p4_framewise, p4_change = add_p4_change_metrics(p4)
    pipeline = deadline[deadline["module"].eq("perception")].copy()
    centerpoint, overflow = parse_centerpoint(run_id, perception_log, sources, p4)
    classification = classify(p4, pipeline)
    validation = validate(run_dir, sources, node, node_summary, deadline)

    sources.to_csv(data / "selected_source_frames.csv", index=False, encoding="utf-8-sig")
    timings.to_csv(data / "perception_node_frame_timings.csv", index=False, encoding="utf-8-sig")
    node.to_csv(data / "perception_node_proxy_deadline_audit.csv", index=False, encoding="utf-8-sig")
    node_summary.to_csv(data / "perception_node_proxy_deadline_summary.csv", index=False, encoding="utf-8-sig")
    p4_framewise.to_csv(data / "p4_framewise_change_diagnostics.csv", index=False, encoding="utf-8-sig")
    deadline.to_csv(data / "module_deadline_detail.csv", index=False, encoding="utf-8-sig")
    deadline_summary.to_csv(data / "module_deadline_summary.csv", index=False, encoding="utf-8-sig")
    centerpoint.to_csv(data / "centerpoint_internal_timing_per_source_frame.csv", index=False, encoding="utf-8-sig")
    overflow.to_csv(data / "perception_buffer_overflow_events.csv", index=False, encoding="utf-8-sig")

    p4_summary = node_summary[node_summary["node_symbol"].eq("P4")].iloc[0]
    pipeline_summary = deadline_summary[deadline_summary["module"].eq("perception")].iloc[0]
    complete_center = centerpoint[centerpoint["data_status"].eq("complete")]
    ground_overflow = (
        overflow[overflow["channel"].eq("/perception/lidar/pointcloud_ground_detection")]
        if "channel" in overflow.columns
        else overflow.iloc[0:0]
    )
    summary = {
        "run_id": run_id,
        "analysis_scope": "perception_only",
        "run_directory": str(run_dir),
        "output_directory": str(out),
        "window": window,
        "classification": classification,
        "p4": {
            "execution_p50_ms": p4_summary["execution_p50_ms"],
            "execution_p95_ms": p4_summary["execution_p95_ms"],
            "execution_max_ms": p4_summary["execution_max_ms"],
            "waiting_p50_ms": p4_summary["waiting_proxy_p50_ms"],
            "waiting_max_ms": p4_summary["waiting_proxy_max_ms"],
            "slack_min_ms": p4_summary["slack_min_ms"],
            "sudden_increase": p4_change,
        },
        "perception_pipeline": {
            "response_p50_ms": pipeline_summary["response_p50_ms"],
            "response_p95_ms": pipeline_summary["response_p95_ms"],
            "response_max_ms": pipeline_summary["response_max_ms"],
            "slack_min_ms": pipeline_summary["slack_min_ms"],
        },
        "centerpoint": {
            "complete_frames": int(len(complete_center)),
            "inference_p50_ms": percentile(complete_center["inference_ms"], 50),
            "inference_p95_ms": percentile(complete_center["inference_ms"], 95),
            "inference_max_ms": pd.to_numeric(complete_center["inference_ms"], errors="coerce").max(),
            "mapping_residual_max_ms": pd.to_numeric(complete_center["mapping_abs_residual_ms"], errors="coerce").max(),
        },
        "buffer_overflow": {
            "all_channel_warning_count": int(len(overflow)),
            "all_channel_drop_message_sum": int(pd.to_numeric(overflow.get("drop_message_count"), errors="coerce").sum()) if len(overflow) else 0,
            "ground_detection_warning_count": int(len(ground_overflow)),
            "ground_detection_drop_message_sum": int(pd.to_numeric(ground_overflow.get("drop_message_count"), errors="coerce").sum()) if len(ground_overflow) else 0,
        },
        "out_of_scope": ["prediction", "planning", "control", "bridge", "vehicle_dynamics"],
        "validation": {"status": validation["status"], "passed": validation["passed_count"], "total": validation["check_count"]},
    }
    (data / "run_realtime_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, default=lambda value: None if pd.isna(value) else value),
        encoding="utf-8",
    )
    (validation_dir / "validation_summary.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    report_text = f"""# {run_id} Perception 实时性逐帧检查

## 结论

判定：`{classification['verdict']}`。

- 选定窗口：{window['selected_onset_local']} 至 {window['selected_end_local']}，{window['selected_source_frames']} 个 lidar 源帧；窗口模式 `{window['selection_status']}`。
- P4 proxy deadline：{classification['p4_miss_count']}/{classification['p4_evaluated']} miss，execution/waiting/no-service = {classification['p4_execution_driven']}/{classification['p4_waiting_driven']}/{classification['p4_no_service']}。
- P4 execution 相邻帧突增：{'有' if p4_change['observed'] else '无'}，标记 {p4_change['flagged_frame_count']} 帧（{p4_change['flagged_source_frames']}）；本 run 的稳健增量阈值为 {p4_change['robust_delta_threshold_ms']:.3f} ms，且要求相邻帧倍率不低于 1.5。
- 完整 Perception freshness：{classification['pipeline_miss_count']}/{classification['pipeline_evaluated']} miss。
- P4 execution P50/P95/MAX：{p4_summary['execution_p50_ms']:.3f}/{p4_summary['execution_p95_ms']:.3f}/{p4_summary['execution_max_ms']:.3f} ms；waiting MAX {p4_summary['waiting_proxy_max_ms']:.3f} ms。
- CenterPoint inference P50/P95/MAX：{summary['centerpoint']['inference_p50_ms']:.3f}/{summary['centerpoint']['inference_p95_ms']:.3f}/{summary['centerpoint']['inference_max_ms']:.3f} ms。
- GroundDetection buffer overflow：{summary['buffer_overflow']['ground_detection_warning_count']} 条警告，drop_message 合计 {summary['buffer_overflow']['ground_detection_drop_message_sum']}；全通道统计仅作旁证，不混入 P4 判定。

## 口径与证据边界

P2–P7 使用上一节点 `output_pub` 作为 input-ready proxy，P1 使用自身 callback start；strict Reader arrival/enqueue 不可用。完整 Perception deadline 定义为本帧 P1 `proc_enter` 到 P7 `output_pub` 必须不晚于下一帧 P1 `proc_enter`。本次按用户要求仅检查 Perception，Prediction、Planning、Control、bridge 与车辆动力学均不在范围内。

独立结构与算术检查：{validation['passed_count']}/{validation['check_count']} 通过。
"""
    (report / "run_summary.md").write_text(report_text, encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, default=lambda value: None if pd.isna(value) else value))


if __name__ == "__main__":
    main()
