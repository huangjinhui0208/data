#!/usr/bin/env python3
"""Extract the evidence needed for speed-dependent hidden-deadline analysis.

The script intentionally uses only the Python standard library so that the
analysis can be rerun without installing packages.  It aligns Apollo wall-clock
logs, monotonic trace events, localization, CARLA collision evidence, and the
user-provided 0.1 s bridge step / 50 m radar-range assumptions.
"""

from __future__ import annotations

import bisect
import collections
import csv
import datetime as dt
import glob
import math
import os
import re
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "log"
TRACE = ROOT / "trace"
OUT = Path(__file__).resolve().parent

RUN_DATE = dt.date(2026, 7, 13)
BRIDGE_STEP_S = 0.1
RADAR_RANGE_M = 50.0
PLANNING_DECEL_MPS2 = 4.0
MEASURED_EFFECTIVE_DECEL_MPS2 = 5.371740881648919
PLANNING_SAFETY_MARGIN_M = 6.0
COLLISION_TS = dt.datetime.fromisoformat("2026-07-13T19:29:02.347041+08:00").timestamp()

LOCALIZATION_RE = re.compile(
    r"measurement_time=([\d.]+) ego_x=([-\d.e+]+) ego_y=([-\d.e+]+).*?"
    r"heading=([-\d.e+]+) ego_vx=([-\d.e+]+) ego_vy=([-\d.e+]+) ego_vz=([-\d.e+]+)"
)
LOG_TS_RE = re.compile(r"^.[0-9]{4} (\d\d):(\d\d):(\d\d\.\d+)")
KV_RE = re.compile(r"(\w+)=([^\s]+)")


def stamp(clock: str) -> float:
    value = dt.datetime.combine(RUN_DATE, dt.time.fromisoformat(clock))
    return value.timestamp()


def iso_local(epoch_s: float) -> str:
    return dt.datetime.fromtimestamp(epoch_s).isoformat(timespec="microseconds")


def read_csv(path: Path) -> Iterable[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        yield from csv.DictReader(handle)


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_localization() -> list[dict[str, float]]:
    path = next(LOG.glob("localization.log.INFO.*"))
    rows: list[dict[str, float]] = []
    with path.open(encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = LOCALIZATION_RE.search(line)
            if not match:
                continue
            t, x, y, heading, vx, vy, vz = map(float, match.groups())
            rows.append(
                {
                    "epoch_s": t,
                    "x_m": x,
                    "y_m": y,
                    "heading_rad": heading,
                    "vx_mps": vx,
                    "vy_mps": vy,
                    "vz_mps": vz,
                    "speed_mps": math.hypot(vx, vy),
                }
            )
    return rows


def nearest_state(states: list[dict[str, float]], t: float) -> dict[str, float]:
    times = [row["epoch_s"] for row in states]
    index = bisect.bisect_left(times, t)
    choices = [max(0, index - 1), min(len(states) - 1, index)]
    return min((states[i] for i in choices), key=lambda row: abs(row["epoch_s"] - t))


def load_events(module: str) -> dict[int, dict[str, list[int]]]:
    path = Path(glob.glob(str(TRACE / "events" / f"{module}.*.csv"))[0])
    events: dict[int, dict[str, list[int]]] = collections.defaultdict(
        lambda: collections.defaultdict(list)
    )
    for row in read_csv(path):
        events[int(row["trace_id"])][row["phase"]].append(int(row["mono_ns"]))
    return events


def first_after(values: list[int], threshold: int) -> int | None:
    values = sorted(values)
    index = bisect.bisect_left(values, threshold)
    return values[index] if index < len(values) else None


def build_e2e_trace() -> list[dict[str, Any]]:
    modules = {
        name: load_events(name)
        for name in (
            "perception.multi_sensor_fusion",
            "prediction",
            "planning",
            "control",
        )
    }

    anchors: dict[int, tuple[int, float, int]] = {}
    anchor_path = next((TRACE / "trace_anchor").glob("*.csv"))
    for row in read_csv(anchor_path):
        if row["sensor_kind"] == "lidar":
            anchors[int(row["trace_id"])] = (
                int(row["preproc_enter_ns"]),
                float(row["ingress_ms"]),
                int(row["data_ts_ns"]),
            )

    fusion_inputs: dict[int, tuple[int, int, int]] = {}
    fusion_path = next((TRACE / "fusion_inputs").glob("*.csv"))
    for row in read_csv(fusion_path):
        if row["sensor_kind"] == "lidar":
            fusion_inputs[int(row["fusion_trace_id"])] = (
                int(row["parent_trace_id"]),
                int(row["frame_time_ns"]),
                int(row["object_count"]),
            )

    result: list[dict[str, Any]] = []
    for fusion_trace, (lidar_trace, data_ts_ns, object_count) in fusion_inputs.items():
        if lidar_trace not in anchors:
            continue
        fusion_events = modules["perception.multi_sensor_fusion"].get(fusion_trace, {})
        prediction_events = modules["prediction"].get(fusion_trace, {})
        planning_events = modules["planning"].get(fusion_trace, {})
        if not fusion_events.get("output_pub"):
            continue
        if not prediction_events.get("proc_enter") or not prediction_events.get("output_pub"):
            continue
        if not planning_events.get("proc_enter") or not planning_events.get("output_pub"):
            continue

        preproc_enter, ingress_ms, _ = anchors[lidar_trace]
        sensor_mono = preproc_enter - ingress_ms * 1e6
        fusion_out = min(fusion_events["output_pub"])
        prediction_in = min(prediction_events["proc_enter"])
        prediction_out = min(prediction_events["output_pub"])
        planning_in = min(planning_events["proc_enter"])
        planning_out = min(planning_events["output_pub"])
        control_out = first_after(
            modules["control"].get(fusion_trace, {}).get("output_pub", []), planning_out
        )

        def elapsed_ms(value: int | None) -> float | None:
            return None if value is None else (value - sensor_mono) / 1e6

        result.append(
            {
                "sensor_time": iso_local(data_ts_ns / 1e9),
                "sensor_epoch_s": data_ts_ns / 1e9,
                "fusion_trace_id": fusion_trace,
                "lidar_trace_id": lidar_trace,
                "fused_object_count": object_count,
                "ingress_ms": ingress_ms,
                "sensor_to_fusion_ms": elapsed_ms(fusion_out),
                "prediction_compute_ms": (prediction_out - prediction_in) / 1e6,
                "fusion_to_prediction_out_ms": (prediction_out - fusion_out) / 1e6,
                "prediction_to_planning_in_ms": (planning_in - prediction_out) / 1e6,
                "planning_compute_ms": (planning_out - planning_in) / 1e6,
                "planning_to_first_control_ms": (
                    None if control_out is None else (control_out - planning_out) / 1e6
                ),
                "sensor_to_first_control_ms": elapsed_ms(control_out),
                "fusion_publish_time": iso_local(
                    data_ts_ns / 1e9 + (fusion_out - sensor_mono) / 1e9
                ),
                "control_publish_time": (
                    ""
                    if control_out is None
                    else iso_local(data_ts_ns / 1e9 + (control_out - sensor_mono) / 1e9)
                ),
            }
        )
    result.sort(key=lambda row: row["sensor_epoch_s"])
    return result


def percentile(values: list[float], ratio: float) -> float:
    values = sorted(values)
    index = (len(values) - 1) * ratio
    lo = int(index)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] * (hi - index) + values[hi] * (index - lo)


def latency_stats(rows: list[dict[str, Any]], start: float, end: float) -> dict[str, float]:
    selected = [
        float(row["sensor_to_first_control_ms"])
        for row in rows
        if start <= row["sensor_epoch_s"] < end
        and row["sensor_to_first_control_ms"] is not None
    ]
    fusion = [
        float(row["sensor_to_fusion_ms"])
        for row in rows
        if start <= row["sensor_epoch_s"] < end
    ]
    return {
        "frame_count": len(selected),
        "e2e_p50_ms": percentile(selected, 0.50),
        "e2e_p95_ms": percentile(selected, 0.95),
        "e2e_max_ms": max(selected),
        "sensor_to_fusion_p50_ms": percentile(fusion, 0.50),
        "sensor_to_fusion_p95_ms": percentile(fusion, 0.95),
    }


def trace_nearest(rows: list[dict[str, Any]], clock: str) -> dict[str, Any]:
    target = stamp(clock)
    return min(rows, key=lambda row: abs(row["sensor_epoch_s"] - target))


def first_stop_after_peak(
    states: list[dict[str, float]], start: float, end: float
) -> tuple[dict[str, float], dict[str, float], float, float]:
    selected = [row for row in states if start <= row["epoch_s"] <= end]
    peak = max(selected, key=lambda row: row["speed_mps"])
    peak_index = selected.index(peak)
    stop = next(row for row in selected[peak_index:] if row["speed_mps"] <= 0.2)
    stop_index = selected.index(stop)
    distance = sum(
        math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
        for a, b in zip(selected[peak_index:stop_index], selected[peak_index + 1 : stop_index + 1])
    )
    effective_decel = peak["speed_mps"] ** 2 / (2 * distance)
    return peak, stop, distance, effective_decel


def deadline(distance_m: float, speed_mps: float, decel_mps2: float) -> dict[str, float]:
    brake = speed_mps**2 / (2 * decel_mps2)
    latest_stop = brake + PLANNING_SAFETY_MARGIN_M
    budget = (distance_m - latest_stop) / speed_mps
    return {
        "braking_distance_m": brake,
        "d2_m": latest_stop,
        "deadline_s": budget,
        "deadline_bridge_steps": budget / BRIDGE_STEP_S,
    }


def extract_planning_evidence() -> list[dict[str, Any]]:
    windows = (
        ("E1", stamp("19:23:02"), stamp("19:23:24")),
        ("E2", stamp("19:23:24"), stamp("19:23:45")),
        ("E3", stamp("19:24:35"), stamp("19:24:45")),
        ("E4", stamp("19:28:55"), COLLISION_TS),
    )
    evidence: list[dict[str, Any]] = []
    for path in sorted(LOG.glob("planning.log.INFO.*")):
        with path.open(encoding="utf-8", errors="ignore") as handle:
            for line in handle:
                interesting = (
                    ("[PLANNING_DECISION]" in line and "decision_type=STOP" in line)
                    or ("[PLANNING_ST_BOUNDARY]" in line and "boundary_type=STOP" in line)
                    or ("[PLANNING_OUTPUT]" in line and "main_stop_reason=stop by" in line)
                )
                if not interesting:
                    continue
                match = LOG_TS_RE.search(line)
                if not match:
                    continue
                hh, mm, ss = match.groups()
                t = stamp(f"{hh}:{mm}:{ss}")
                scenario = next((name for name, a, b in windows if a <= t < b), None)
                if scenario is None:
                    continue
                kv = dict(KV_RE.findall(line))
                object_id = kv.get("id", "")
                if object_id == "DEST" or object_id.startswith("PATH_END"):
                    continue
                if "[PLANNING_DECISION]" in line:
                    kind = "stop_decision"
                elif "[PLANNING_ST_BOUNDARY]" in line:
                    kind = "stop_st_boundary"
                else:
                    kind = "stop_trajectory_output"
                evidence.append(
                    {
                        "scenario": scenario,
                        "time": iso_local(t),
                        "kind": kind,
                        "object_id": object_id,
                        "tag_or_source": kv.get("tag", kv.get("source", "")),
                        "stop_x": kv.get("stop_x", ""),
                        "stop_y": kv.get("stop_y", ""),
                        "distance_s": kv.get("distance_s", ""),
                        "reason_code": kv.get("reason_code", ""),
                        "main_stop_reason": (
                            line.split("main_stop_reason=", 1)[1].strip()
                            if "main_stop_reason=" in line
                            else ""
                        ),
                        "source_log": path.name,
                    }
                )
    evidence.sort(key=lambda row: row["time"])
    return evidence


def build_critical_stage_timeline(
    traces: list[dict[str, Any]], cases: list[tuple[str, str]]
) -> list[dict[str, Any]]:
    """Create an inspectable per-stage timeline for selected sensor frames."""
    stage_modules = (
        "perception.pointcloud_preprocess",
        "perception.pointcloud_map_based_roi",
        "perception.pointcloud_ground_detection",
        "perception.lidar_detection",
        "perception.lidar_detection_filter",
        "perception.lidar_tracking",
        "perception.multi_sensor_fusion",
        "prediction",
        "planning",
        "control",
    )
    modules = {name: load_events(name) for name in stage_modules}
    result: list[dict[str, Any]] = []
    for label, clock in cases:
        trace = trace_nearest(traces, clock)
        fusion_trace = int(trace["fusion_trace_id"])
        lidar_trace = int(trace["lidar_trace_id"])
        sensor_time = float(trace["sensor_epoch_s"])
        sensor_mono = None
        preprocess = modules["perception.pointcloud_preprocess"].get(lidar_trace, {})
        if preprocess.get("proc_enter"):
            sensor_mono = min(preprocess["proc_enter"]) - float(trace["ingress_ms"]) * 1e6
        if sensor_mono is None:
            continue

        planning_out = min(modules["planning"][fusion_trace]["output_pub"])
        control_out = first_after(
            modules["control"].get(fusion_trace, {}).get("output_pub", []), planning_out
        )

        for module in stage_modules:
            if module.startswith("perception.") and module != "perception.multi_sensor_fusion":
                trace_ids = (lidar_trace,)
            elif module == "perception.multi_sensor_fusion":
                trace_ids = (lidar_trace, fusion_trace)
            else:
                trace_ids = (fusion_trace,)
            events: list[tuple[int, str, int]] = []
            for trace_id in trace_ids:
                for phase, values in modules[module].get(trace_id, {}).items():
                    for value in values:
                        events.append((value, phase, trace_id))
            events.sort()

            # Control runs at 100 Hz on the same 10 Hz trace. Retain the first
            # effective publication path only; later repetitions add noise.
            if module == "control" and control_out is not None:
                cutoff = control_out
                events = [item for item in events if item[0] <= cutoff]

            for mono_ns, phase, trace_id in events:
                elapsed_ms = (mono_ns - sensor_mono) / 1e6
                result.append(
                    {
                        "case": label,
                        "sensor_time": iso_local(sensor_time),
                        "fusion_trace_id": fusion_trace,
                        "lidar_trace_id": lidar_trace,
                        "module": module,
                        "phase": phase,
                        "event_trace_id": trace_id,
                        "elapsed_from_sensor_ms": elapsed_ms,
                        "estimated_wall_time": iso_local(sensor_time + elapsed_ms / 1000),
                    }
                )
    result.sort(key=lambda row: (row["case"], row["elapsed_from_sensor_ms"]))
    return result


def selected_critical_path(
    label: str,
    sensor_clock: str,
    obstacle_y: float,
    states: list[dict[str, float]],
    traces: list[dict[str, Any]],
    interpretation: str,
) -> dict[str, Any]:
    trace = trace_nearest(traces, sensor_clock)
    state = nearest_state(states, trace["sensor_epoch_s"])
    distance = abs(state["y_m"] - obstacle_y)
    values = deadline(distance, state["speed_mps"], PLANNING_DECEL_MPS2)
    latency_s = trace["sensor_to_first_control_ms"] / 1000
    return {
        "case": label,
        "interpretation": interpretation,
        "sensor_time": trace["sensor_time"],
        "ego_y_m": state["y_m"],
        "obstacle_y_m": obstacle_y,
        "d1_distance_m": distance,
        "speed_mps": state["speed_mps"],
        "speed_kmh": state["speed_mps"] * 3.6,
        "assumed_decel_mps2": PLANNING_DECEL_MPS2,
        "safety_margin_m": PLANNING_SAFETY_MARGIN_M,
        **values,
        "sensor_to_fusion_ms": trace["sensor_to_fusion_ms"],
        "sensor_to_control_ms": trace["sensor_to_first_control_ms"],
        "observed_latency_steps": latency_s / BRIDGE_STEP_S,
        "slack_s": values["deadline_s"] - latency_s,
        "slack_steps": (values["deadline_s"] - latency_s) / BRIDGE_STEP_S,
        "fusion_publish_time": trace["fusion_publish_time"],
        "control_publish_time": trace["control_publish_time"],
        "published_before_collision": trace["sensor_epoch_s"] + latency_s < COLLISION_TS,
    }


def build_scenario_summary(
    states: list[dict[str, float]], traces: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    specifications = (
        {
            "scenario": "E1_初始前方约50m障碍",
            "start": stamp("19:23:03.247"),
            "end": stamp("19:23:24.547"),
            "obstacle_y": 111.0,
            "planner_reaction": "19:23:03.411705 blocking_obstacle STOP；障碍在起步前已被跟踪",
            "outcome": "刹停",
        },
        {
            "scenario": "E2_障碍再后移约50m",
            "start": stamp("19:23:24.547"),
            "end": stamp("19:23:34.0"),
            "obstacle_y": 51.74,
            "planner_reaction": "19:23:31.377融合发布id=2698；19:23:31.425规划输出stop by 2698",
            "outcome": "刹停",
        },
        {
            "scenario": "E3_障碍再后移约100m",
            "start": stamp("19:24:30.547"),
            "end": stamp("19:24:42.0"),
            "obstacle_y": -48.3,
            "planner_reaction": "19:24:39.159融合发布id=3869；19:24:39.199首次硬停轨迹",
            "outcome": "55.6km/h后刹停",
        },
    )
    result: list[dict[str, Any]] = []
    for item in specifications:
        peak, stop, braking_distance, effective_decel = first_stop_after_peak(
            states, item["start"], item["end"]
        )
        stats = latency_stats(traces, item["start"], item["end"])
        result.append(
            {
                "scenario": item["scenario"],
                "window_start": iso_local(item["start"]),
                "window_end": iso_local(item["end"]),
                "obstacle_y_est_m": item["obstacle_y"],
                "peak_time": iso_local(peak["epoch_s"]),
                "peak_speed_mps": peak["speed_mps"],
                "peak_speed_kmh": peak["speed_mps"] * 3.6,
                "first_stop_time": iso_local(stop["epoch_s"]),
                "stop_y_m": stop["y_m"],
                "center_gap_at_stop_m": abs(stop["y_m"] - item["obstacle_y"]),
                "peak_to_stop_path_m": braking_distance,
                "peak_to_stop_effective_decel_mps2": effective_decel,
                "planner_reaction": item["planner_reaction"],
                "outcome": item["outcome"],
                "latency_stats_scope": "该场景驾驶窗口",
                **stats,
            }
        )

    lap = [row for row in states if stamp("19:25:28.447") <= row["epoch_s"] <= COLLISION_TS]
    max_lap = max(lap, key=lambda row: row["speed_mps"])
    collision_state = nearest_state(states, stamp("19:29:02.248021"))
    stats = latency_stats(traces, stamp("19:28:50"), COLLISION_TS)
    result.append(
        {
            "scenario": "E4_障碍移回出生点并绕场碰撞",
            "window_start": iso_local(stamp("19:25:28.447")),
            "window_end": iso_local(COLLISION_TS),
            "obstacle_y_est_m": 160.0,
            "peak_time": iso_local(max_lap["epoch_s"]),
            "peak_speed_mps": max_lap["speed_mps"],
            "peak_speed_kmh": max_lap["speed_mps"] * 3.6,
            "first_stop_time": "",
            "stop_y_m": "",
            "center_gap_at_stop_m": "",
            "peak_to_stop_path_m": "",
            "peak_to_stop_effective_decel_mps2": "",
            "planner_reaction": "碰撞前没有目标障碍的STOP decision/ST boundary/stop trajectory",
            "outcome": (
                f"19:29:02.347碰撞；碰撞前一帧速度{collision_state['speed_mps'] * 3.6:.2f}km/h"
            ),
            "latency_stats_scope": "最终接近窗口19:28:50至碰撞",
            **stats,
        }
    )
    return result


def build_speed_table() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for speed_kmh in (30, 40, 50, 55.6, 60, 66.47, 70):
        speed = speed_kmh / 3.6
        for model, decel in (
            ("规划保守值", PLANNING_DECEL_MPS2),
            ("E3实测等效值_偏乐观", MEASURED_EFFECTIVE_DECEL_MPS2),
        ):
            values = deadline(RADAR_RANGE_M, speed, decel)
            rows.append(
                {
                    "speed_kmh": speed_kmh,
                    "speed_mps": speed,
                    "decel_model": model,
                    "decel_mps2": decel,
                    "radar_d1_m": RADAR_RANGE_M,
                    "safety_margin_m": PLANNING_SAFETY_MARGIN_M,
                    **values,
                    "one_full_bridge_step_available": values["deadline_s"] >= BRIDGE_STEP_S,
                }
            )
    return rows


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def build_report(
    states: list[dict[str, float]],
    traces: list[dict[str, Any]],
    scenarios: list[dict[str, Any]],
    critical: list[dict[str, Any]],
) -> str:
    all_stats = latency_stats(traces, traces[0]["sensor_epoch_s"], traces[-1]["sensor_epoch_s"] + 1)
    e4 = critical[2]
    e3 = critical[1]
    collision_state = nearest_state(states, stamp("19:29:02.248021"))
    collision_fast = deadline(RADAR_RANGE_M, collision_state["speed_mps"], PLANNING_DECEL_MPS2)
    return f"""# 202607131922 车速隐形 Deadline 分析

## 结论

最终碰撞的主因证据指向**感知主链路排队导致障碍物在碰撞前没有成为规划可用目标**。Bridge 直接消费 Control 输出，端到端时延终点按首个 Control 发布计算。

- 最终直道进入名义 50 m 雷达范围的代表帧为 `{e4['sensor_time']}`：车辆约 `{fmt(e4['speed_kmh'], 2)} km/h`，到出生点障碍中心约 `{fmt(e4['d1_distance_m'], 2)} m`。
- 按规划日志中的最大硬停减速度 `4 m/s²` 和 `PathDecider distance_s=-6` 对应的 6 m 裕量，隐形 deadline 约 `{fmt(e4['deadline_s'])} s`（`{fmt(e4['deadline_bridge_steps'], 2)}` 个 0.1 s bridge 步长）。
- 该帧到首个 Control 发布实际用了 `{fmt(e4['sensor_to_control_ms'] / 1000)} s`，超期 `{fmt(-e4['slack_s'])} s`（`{fmt(-e4['slack_steps'], 1)}` 个 bridge 步长），发布时已经在碰撞后。
- 碰撞记录为 `19:29:02.347041+08:00`，碰撞前一帧速度 `{fmt(collision_state['speed_mps'] * 3.6, 2)} km/h`。若直接用该速度和 50 m 计算，保守 deadline 只剩 `{fmt(collision_fast['deadline_s'])} s`，不足一个 0.1 s 步长。
- 成功的 55.6 km/h 刹停段 E3：首个可触发停车的帧到 Control 为 `{fmt(e3['sensor_to_control_ms'] / 1000)} s`，当时 deadline `{fmt(e3['deadline_s'])} s`，仅余 `{fmt(e3['slack_s'])} s`（约 `{fmt(e3['slack_steps'], 1)}` 个步长），已经很接近边界。

## 四段场景还原

|段|时间与结果|速度/位置证据|规划证据|
|---|---|---|---|
|E1|19:23:03 起步，19:23:17 首次静止|峰值 {scenarios[0]['peak_speed_kmh']:.1f} km/h，停在 y={scenarios[0]['stop_y_m']:.2f}|起步前目标已在跟踪；19:23:03.411 出现 blocking-obstacle STOP|
|E2|障碍约移到 y=51.7，19:23:32 刹停|峰值 {scenarios[1]['peak_speed_kmh']:.1f} km/h，停在 y={scenarios[1]['stop_y_m']:.2f}|id=2698；19:23:31.425 输出 `stop by 2698`|
|E3|障碍约移到 y=-48.3，19:24:41 刹停|峰值 {scenarios[2]['peak_speed_kmh']:.1f} km/h；峰值到停下 {scenarios[2]['peak_to_stop_path_m']:.2f} m；等效减速度 {scenarios[2]['peak_to_stop_effective_decel_mps2']:.2f} m/s²|id=3869；19:24:39.199 首次硬停轨迹，配置上限 4 m/s²|
|E4|19:25:28 开始绕场；19:29:02 碰撞|整圈最高 {scenarios[3]['peak_speed_kmh']:.1f} km/h；碰撞前 {collision_state['speed_mps'] * 3.6:.2f} km/h|碰撞前没有目标障碍的 STOP decision、STOP ST boundary 或 `stop by <id>`|

定位轨迹没有大于 10 m/0.1 s 的跳变；数据支持的是**障碍物继续后移约 100 m**，并不支持“车辆瞬移后置 100 m”。从 E2 障碍约 y=51.7 到 E3 障碍约 y=-48.3，正好约 100 m。

## 关键链路对比

|案例|D1/速度|理论 deadline|实测 sensor→control|余量|
|---|---:|---:|---:|---:|
|E2 成功|{critical[0]['d1_distance_m']:.2f} m / {critical[0]['speed_kmh']:.1f} km/h|{critical[0]['deadline_s']:.3f} s|{critical[0]['sensor_to_control_ms']/1000:.3f} s|+{critical[0]['slack_s']:.3f} s|
|E3 成功|{critical[1]['d1_distance_m']:.2f} m / {critical[1]['speed_kmh']:.1f} km/h|{critical[1]['deadline_s']:.3f} s|{critical[1]['sensor_to_control_ms']/1000:.3f} s|+{critical[1]['slack_s']:.3f} s|
|E4 碰撞|{critical[2]['d1_distance_m']:.2f} m / {critical[2]['speed_kmh']:.1f} km/h|{critical[2]['deadline_s']:.3f} s|{critical[2]['sensor_to_control_ms']/1000:.3f} s|{critical[2]['slack_s']:.3f} s|

E4 的 50 m 代表帧中，`sensor→fusion` 已占 `{e4['sensor_to_fusion_ms']/1000:.3f} s`；fusion 之后到首个 Control 只有 `{(e4['sensor_to_control_ms']-e4['sensor_to_fusion_ms']):.1f} ms`。关键车道目标对应的稍后帧（19:28:59.947）直到 19:29:04.631 才发布融合结果，也在碰撞后。

对 19:28:59.947 关键帧逐阶段检查：传感器到点云预处理入口 64.8 ms；预处理、ROI、地面检测在约 86.7 ms 完成；之后等待约 4.464 s，`lidar_detection` 才进入；检测计算约 121.6 ms；fusion 后到首个 Control 约 40.0 ms。因此主要瓶颈是 **ground-detection 输出到 lidar-detection 入口之间的队列/调度等待**。

## 雷达 50 m 的使用限制

`fusion_inputs` 明确将 `radar_front` 标为 `is_main_sensor=0`，而 `velodyne64` 为 `is_main_sensor=1`；perception 日志反复出现 `Fusion receive from radar_front. Skip because it is not the main sensor.`。这意味着雷达帧本身不能立即触发融合输出，必须等待激光雷达主帧到达。最终接近时激光雷达链路已积压 4–5 s，所以即使雷达在 50 m 内及时看到目标，也不能据现有架构形成及时的规划输入。

当前日志不能精确测出“原始雷达连续 N 帧稳定识别”的 D1：radar message-context 的 `object_count` 为 0，且没有目标级 radar 坐标/ID。因此本报告把 50 m 用作名义上界，把“首个能导致规划停车的融合帧”作为运行时 D1 代理。最终场景碰撞前不存在这样的 planner-visible D1；不能把名义 50 m 直接当成已实现的稳定感知距离。

## 时延统计与日志适用性

- 全时段 `sensor→control`：p50 `{all_stats['e2e_p50_ms']:.1f} ms`，p95 `{all_stats['e2e_p95_ms']:.1f} ms`，最大 `{all_stats['e2e_max_ms']:.1f} ms`。
- 最终接近窗口 p50 `{scenarios[3]['e2e_p50_ms']:.1f} ms`，p95 `{scenarios[3]['e2e_p95_ms']:.1f} ms`；远大于约 1 s 的 deadline。
- Control 约 100 Hz；Bridge 和传感器主帧约 10 Hz。以 0.1 s 步长分析时，应至少预留 1 个完整步长作为离散采样裕量。
- 本场景 Bridge 直接消费 Control 指令，Control 首次发布时刻作为软件链路终点。CARLA applied-control 未采集，执行器施加时延仍需后续补充。
- `control.log` 没有 brake/throttle/applied-control 数值；现有分析用定位速度下降和规划硬停轨迹反推制动。下次应同步记录 `/apollo/control` 命令和 CARLA 实际施加的 throttle/brake。
- 采集在碰撞后仍继续到 19:31:52；本次场景因果分析在 19:29:02.347 截止，之后数据不纳入最终碰撞结论。

## 建议的下一轮采集字段

1. 原始 radar 目标级：时间戳、ID、位置、range/range-rate、confidence、连续跟踪帧数。
2. 每个队列的 enqueue/dequeue 时间与队列深度，尤其 ground-detection→lidar-detection。
3. `/apollo/control` 的 brake、throttle、acceleration、命令时间戳，以及 bridge/CARLA applied-control 时间戳。
4. 障碍物 ground-truth actor 位姿历史，不只保留碰撞前 18 s。
5. 把 D1 定义为“同一目标连续 3 帧满足位置/置信度阈值”，D2 以沿参考线的障碍物前缘到自车前缘距离计算，避免中心点距离口径混用。

## 输出文件

- `scenario_summary.csv`：四段场景与速度、停车、时延统计。
- `critical_path_comparison.csv`：E2/E3 成功与 E4 碰撞的 deadline/实测时延对比。
- `deadline_speed_table.csv`：不同车速、两种减速度口径下的 deadline。
- `e2e_latency_by_frame.csv`：每个主激光帧的端到端分阶段时延。
- `vehicle_state_10hz.csv`：定位轨迹、速度。
- `planning_relevant_events.csv`：与停车有关的规划证据。
- `critical_stage_timeline.csv`：E2/E3/E4 代表帧的逐阶段事件时间轴。
"""


def main() -> None:
    states = load_localization()
    traces = build_e2e_trace()
    planning = extract_planning_evidence()

    vehicle_rows: list[dict[str, Any]] = []
    previous: dict[str, float] | None = None
    for state in states:
        row: dict[str, Any] = {
            "time": iso_local(state["epoch_s"]),
            **state,
            "speed_kmh": state["speed_mps"] * 3.6,
        }
        if previous is None:
            row["longitudinal_speed_delta_mps"] = ""
            row["sample_accel_mps2"] = ""
        else:
            delta_t = state["epoch_s"] - previous["epoch_s"]
            delta_v = state["speed_mps"] - previous["speed_mps"]
            row["longitudinal_speed_delta_mps"] = delta_v
            row["sample_accel_mps2"] = delta_v / delta_t if delta_t > 0 else ""
        vehicle_rows.append(row)
        previous = state

    critical = [
        selected_critical_path(
            "E2_成功_首个规划可用目标帧",
            "19:23:30.047683",
            51.74313,
            states,
            traces,
            "id=2698首帧，融合延迟约1.33s",
        ),
        selected_critical_path(
            "E3_成功_首个停车目标帧",
            "19:24:38.547463",
            -47.66152,
            states,
            traces,
            "id=3869首帧，随后输出4m/s²硬停轨迹",
        ),
        selected_critical_path(
            "E4_碰撞_名义50m入区帧",
            "19:28:59.647504",
            160.0,
            states,
            traces,
            "名义雷达入区；规划在碰撞前没有目标停车决策",
        ),
        selected_critical_path(
            "E4_碰撞_首个较大目标关联帧",
            "19:28:59.947392",
            160.0,
            states,
            traces,
            "融合日志随后出现出生点附近车辆尺寸目标，但发布已在碰撞后",
        ),
    ]
    scenarios = build_scenario_summary(states, traces)
    speed_table = build_speed_table()
    critical_stage_timeline = build_critical_stage_timeline(
        traces,
        [
            ("E2_成功", "19:23:30.047683"),
            ("E3_成功", "19:24:38.547463"),
            ("E4_50m入区", "19:28:59.647504"),
            ("E4_较大目标关联帧", "19:28:59.947392"),
        ],
    )

    write_csv(OUT / "vehicle_state_10hz.csv", vehicle_rows)
    write_csv(OUT / "e2e_latency_by_frame.csv", traces)
    write_csv(OUT / "planning_relevant_events.csv", planning)
    write_csv(OUT / "critical_path_comparison.csv", critical)
    write_csv(OUT / "scenario_summary.csv", scenarios)
    write_csv(OUT / "deadline_speed_table.csv", speed_table)
    write_csv(OUT / "critical_stage_timeline.csv", critical_stage_timeline)
    (OUT / "report.md").write_text(
        build_report(states, traces, scenarios, critical), encoding="utf-8-sig"
    )

    print(f"Wrote analysis package to {OUT}")
    print(f"Localization rows: {len(states)}")
    print(f"Trace rows: {len(traces)}")
    print(f"Planning evidence rows: {len(planning)}")


if __name__ == "__main__":
    main()
