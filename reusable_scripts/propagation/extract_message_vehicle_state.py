#!/usr/bin/env python3
"""Build cross-module message propagation and vehicle-state CSV files.

The script consumes the structured CSV/JSON exports under ``<run>/record`` and
the e2e trace message contexts under ``<run>/trace``.  It does not read or
modify the original Apollo record file.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Optional


TABLE_SPECS = {
    "perception_propagation.csv": (
        "perception数据统计",
        [
            "source_frame", "lidar_timestamp", "perception_seq",
            "perception_pub_time", "output_interval_ms",
            "perception_data_age_ms", "obstacle_ids", "target_id",
            "target_x", "target_y", "target_vx", "target_vy",
            "tracking_time",
        ],
    ),
    "prediction_propagation.csv": (
        "prediction数据统计",
        [
            "prediction_seq", "prediction_input_time",
            "prediction_output_time", "input_perception_seq",
            "lidar_timestamp", "input_interval_ms", "input_data_age_ms",
            "output_data_age_ms", "target_id", "history_length",
            "trajectory_points",
        ],
    ),
    "planning_propagation.csv": (
        "planning数据统计",
        [
            "planning_seq", "planning_start", "planning_pub_time",
            "input_prediction_seq", "prediction_timestamp",
            "lidar_timestamp", "planning_data_age_ms",
            "planning_interval_ms", "trajectory_point_count", "first_v",
            "first_a", "decision_type",
        ],
    ),
    "control_propagation.csv": (
        "control数据统计",
        [
            "control_seq", "control_time", "planning_seq_used",
            "planning_pub_time", "planning_lidar_timestamp",
            "planning_data_age_at_control_ms", "planning_reuse_count",
            "throttle", "brake", "steering",
        ],
    ),
    "vehicle_state.csv": (
        "vehicle_state数据统计",
        [
            "timestamp", "x", "y", "speed", "acceleration", "heading",
            "throttle", "brake", "steering",
        ],
    ),
    "target_obstacle_state.csv": (
        "target_obstacle数据统计",
        [
            "timestamp", "target_id", "target_x", "target_y",
            "target_speed", "ego_x", "ego_y", "ego_speed",
            "relative_distance", "relative_speed", "TTC",
        ],
    ),
}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="从record结构化导出与e2e trace生成消息传播和车辆状态逐帧表"
    )
    parser.add_argument("run_id", help="run目录名，例如202609031348")
    parser.add_argument("--run-dir", type=Path, help="run目录；默认D:/data/<run_id>")
    parser.add_argument("--record-dir", type=Path, help="record导出目录；默认<run-dir>/record")
    parser.add_argument(
        "--output-root",
        type=Path,
        help="模块化输出根目录；默认<run-dir>/打点逐帧数据统计",
    )
    parser.add_argument(
        "--source-frames",
        type=Path,
        help="Perception source-frame映射；默认读取标准perception统计目录",
    )
    parser.add_argument(
        "--target-id",
        type=int,
        help="固定目标障碍物ID；省略时逐消息选择离车辆最近的障碍物",
    )
    return parser.parse_args(argv)


def one(directory: Path, pattern: str) -> Path:
    matches = sorted(directory.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {pattern!r} under {directory}, found {len(matches)}"
        )
    return matches[0]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def read_raw_messages(path: Path) -> list[dict[str, Any]]:
    rows = read_csv(path)
    for row in rows:
        row["raw_message"] = json.loads(row["raw_json"])
    return rows


def to_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def to_int(value: Any) -> Optional[int]:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def int_or_blank(value: Any) -> Any:
    number = to_int(value)
    return "" if number is None else number


def lidar_seconds(value: Any) -> Optional[float]:
    number = to_int(value)
    return number / 1e9 if number and number > 0 else None


def fmt(value: Any, digits: int = 9) -> str:
    number = to_float(value)
    return "" if number is None else f"{number:.{digits}f}"


def fmt_ms(value: Any) -> str:
    return fmt(value, 6)


def nested(mapping: dict[str, Any], *keys: str) -> Any:
    value: Any = mapping
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_context(path: Path) -> tuple[dict[int, dict[str, str]], dict[tuple[str, str], dict[str, str]]]:
    outputs: dict[int, dict[str, str]] = {}
    inputs: dict[tuple[str, str], dict[str, str]] = {}
    for row in read_csv(path):
        key = (row["trace_id"], row["proc_id"])
        if row["edge"] == "in":
            inputs[key] = row
        elif row["edge"] == "out":
            sequence = to_int(row["output_seq"])
            if sequence is not None:
                outputs[sequence] = row
    return outputs, inputs


def input_time_from_trace(
    output_header_time: Optional[float],
    output_context: Optional[dict[str, str]],
    input_contexts: dict[tuple[str, str], dict[str, str]],
) -> Optional[float]:
    if output_header_time is None or output_context is None:
        return None
    input_context = input_contexts.get(
        (output_context["trace_id"], output_context["proc_id"])
    )
    if input_context is None:
        return None
    output_mono = to_int(output_context["mono_ns"])
    input_mono = to_int(input_context["mono_ns"])
    if output_mono is None or input_mono is None or output_mono < input_mono:
        return None
    return output_header_time - (output_mono - input_mono) / 1e9


def trace_input_sequence(
    output_context: Optional[dict[str, str]],
    input_contexts: dict[tuple[str, str], dict[str, str]],
) -> Optional[int]:
    if output_context is None:
        return None
    input_context = input_contexts.get(
        (output_context["trace_id"], output_context["proc_id"])
    )
    return to_int(input_context.get("input_seq")) if input_context else None


def interpolate_state(states: list[dict[str, float]], timestamp: float) -> Optional[dict[str, float]]:
    if not states or timestamp < states[0]["timestamp"] or timestamp > states[-1]["timestamp"]:
        return None
    times = [row["timestamp"] for row in states]
    right = bisect.bisect_left(times, timestamp)
    if right < len(states) and times[right] == timestamp:
        return states[right]
    if right == 0 or right == len(states):
        return None
    left = right - 1
    span = times[right] - times[left]
    if span <= 0:
        return None
    ratio = (timestamp - times[left]) / span
    output = {"timestamp": timestamp}
    for field in ["x", "y", "speed", "vx", "vy", "heading"]:
        output[field] = states[left][field] + ratio * (states[right][field] - states[left][field])
    return output


def choose_target(
    obstacles: list[dict[str, Any]],
    ego: Optional[dict[str, float]],
    fixed_target_id: Optional[int],
) -> Optional[dict[str, Any]]:
    if fixed_target_id is not None:
        return next((item for item in obstacles if to_int(item.get("id")) == fixed_target_id), None)
    if not obstacles:
        return None
    if ego is None:
        return min(obstacles, key=lambda item: to_int(item.get("id")) or 0)

    def distance(item: dict[str, Any]) -> float:
        x = to_float(nested(item, "position", "x"))
        y = to_float(nested(item, "position", "y"))
        if x is None or y is None:
            return math.inf
        return math.hypot(x - ego["x"], y - ego["y"])

    return min(obstacles, key=distance)


def build_vehicle_states(record_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, float]], dict[str, Any]]:
    base = record_dir / "01_localization_chassis"
    poses = read_csv(base / "localization_pose.csv")
    velocity_by_record = {
        row["record_time_ns"]: row
        for row in read_csv(base / "localization_velocity_acc.csv")
    }
    chassis = sorted(
        read_csv(base / "chassis.csv"),
        key=lambda row: to_float(row["header_timestamp_sec"]) or -math.inf,
    )
    chassis_times = [to_float(row["header_timestamp_sec"]) or -math.inf for row in chassis]
    output: list[dict[str, Any]] = []
    interpolation: list[dict[str, float]] = []
    chassis_gaps_ms: list[float] = []

    for pose in sorted(poses, key=lambda row: to_float(row["measurement_time"]) or -math.inf):
        velocity = velocity_by_record.get(pose["record_time_ns"])
        timestamp = to_float(pose["measurement_time"])
        x = to_float(pose["x"])
        y = to_float(pose["y"])
        heading = to_float(pose["heading"])
        if velocity is None or None in (timestamp, x, y, heading):
            continue
        speed = to_float(velocity["speed"])
        vx = to_float(velocity["vx"])
        vy = to_float(velocity["vy"])
        ax = to_float(velocity["ax"])
        ay = to_float(velocity["ay"])
        if None in (speed, vx, vy, ax, ay):
            continue
        nearest = bisect.bisect_left(chassis_times, timestamp)
        candidates = [index for index in (nearest - 1, nearest) if 0 <= index < len(chassis)]
        chassis_row = min(
            (chassis[index] for index in candidates),
            key=lambda row: abs((to_float(row["header_timestamp_sec"]) or -math.inf) - timestamp),
        ) if candidates else None
        gap_ms = (
            abs((to_float(chassis_row["header_timestamp_sec"]) or timestamp) - timestamp) * 1000
            if chassis_row else None
        )
        if gap_ms is not None:
            chassis_gaps_ms.append(gap_ms)
        acceleration = ax * math.cos(heading) + ay * math.sin(heading)
        output.append(
            {
                "timestamp": fmt(timestamp),
                "x": fmt(x),
                "y": fmt(y),
                "speed": fmt(speed),
                "acceleration": fmt(acceleration),
                "heading": fmt(heading),
                "throttle": fmt(chassis_row.get("throttle_percentage")) if chassis_row else "",
                "brake": fmt(chassis_row.get("brake_percentage")) if chassis_row else "",
                "steering": fmt(chassis_row.get("steering_percentage")) if chassis_row else "",
            }
        )
        interpolation.append(
            {
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "speed": speed,
                "vx": vx,
                "vy": vy,
                "heading": heading,
            }
        )
    return output, interpolation, {
        "localization_input_rows": len(poses),
        "vehicle_output_rows": len(output),
        "chassis_input_rows": len(chassis),
        "vehicle_rows_with_chassis": sum(bool(row["throttle"]) for row in output),
        "max_nearest_chassis_gap_ms": max(chassis_gaps_ms) if chassis_gaps_ms else None,
    }


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = (args.run_dir or Path("D:/data") / args.run_id).resolve()
    record_dir = (args.record_dir or run_dir / "record").resolve()
    output_root = (args.output_root or run_dir / "打点逐帧数据统计").resolve()
    source_frames_path = args.source_frames or (
        run_dir / "打点逐帧数据统计" / "perception数据统计" / "data" / "selected_source_frames.csv"
    )

    vehicle_rows, ego_states, vehicle_quality = build_vehicle_states(record_dir)

    source_frame_by_lidar: dict[int, int] = {}
    if source_frames_path.is_file():
        for row in read_csv(source_frames_path):
            lidar = to_int(row.get("data_ts_ns"))
            frame = to_int(row.get("source_frame_index"))
            if lidar is not None and frame is not None:
                source_frame_by_lidar[lidar] = frame

    context_dir = run_dir / "trace" / "message_context"
    prediction_outputs, prediction_inputs = load_context(one(context_dir, "prediction.*.csv"))
    planning_outputs, planning_inputs = load_context(one(context_dir, "planning.*.csv"))
    control_outputs, control_inputs = load_context(one(context_dir, "control.*.csv"))

    perception_messages = read_raw_messages(
        record_dir / "04_prediction_perception" / "perception_raw.jsonl"
    )
    perception_messages.sort(key=lambda row: to_float(row["header_timestamp_sec"]) or -math.inf)
    perception_rows: list[dict[str, Any]] = []
    target_state_rows: list[dict[str, Any]] = []
    previous_pub: Optional[float] = None
    target_ids: Counter[int] = Counter()
    for message in perception_messages:
        raw = message["raw_message"]
        pub = to_float(message["header_timestamp_sec"])
        lidar_ns = to_int(message["header_lidar_timestamp"])
        lidar_s = lidar_seconds(lidar_ns)
        obstacles = raw.get("perception_obstacle", []) or []
        ego_at_lidar = interpolate_state(ego_states, lidar_s) if lidar_s is not None else None
        target = choose_target(obstacles, ego_at_lidar, args.target_id)
        target_id = to_int(target.get("id")) if target else None
        if target_id is not None:
            target_ids[target_id] += 1
        perception_rows.append(
            {
                "source_frame": source_frame_by_lidar.get(lidar_ns, ""),
                "lidar_timestamp": int_or_blank(lidar_ns),
                "perception_seq": int_or_blank(message["header_sequence_num"]),
                "perception_pub_time": fmt(pub),
                "output_interval_ms": fmt_ms((pub - previous_pub) * 1000) if pub is not None and previous_pub is not None else "",
                "perception_data_age_ms": fmt_ms((pub - lidar_s) * 1000) if pub is not None and lidar_s is not None else "",
                "obstacle_ids": json.dumps(
                    [to_int(item.get("id")) for item in obstacles if to_int(item.get("id")) is not None],
                    separators=(",", ":"),
                ),
                "target_id": int_or_blank(target_id),
                "target_x": fmt(nested(target, "position", "x")) if target else "",
                "target_y": fmt(nested(target, "position", "y")) if target else "",
                "target_vx": fmt(nested(target, "velocity", "x")) if target else "",
                "target_vy": fmt(nested(target, "velocity", "y")) if target else "",
                "tracking_time": fmt(target.get("tracking_time")) if target else "",
            }
        )
        previous_pub = pub

        if target is None:
            continue
        obstacle_time = to_float(target.get("timestamp")) or lidar_s
        ego = interpolate_state(ego_states, obstacle_time) if obstacle_time is not None else None
        tx = to_float(nested(target, "position", "x"))
        ty = to_float(nested(target, "position", "y"))
        tvx = to_float(nested(target, "velocity", "x")) or 0.0
        tvy = to_float(nested(target, "velocity", "y")) or 0.0
        distance = closing = ttc = None
        if ego is not None and tx is not None and ty is not None:
            dx, dy = tx - ego["x"], ty - ego["y"]
            distance = math.hypot(dx, dy)
            if distance > 0:
                closing = ((ego["vx"] - tvx) * dx + (ego["vy"] - tvy) * dy) / distance
                if closing > 0:
                    ttc = distance / closing
        target_state_rows.append(
            {
                "timestamp": fmt(obstacle_time),
                "target_id": int_or_blank(target_id),
                "target_x": fmt(tx),
                "target_y": fmt(ty),
                "target_speed": fmt(math.hypot(tvx, tvy)),
                "ego_x": fmt(ego["x"]) if ego else "",
                "ego_y": fmt(ego["y"]) if ego else "",
                "ego_speed": fmt(ego["speed"]) if ego else "",
                "relative_distance": fmt(distance),
                "relative_speed": fmt(closing),
                "TTC": fmt(ttc),
            }
        )

    prediction_messages = read_raw_messages(
        record_dir / "04_prediction_perception" / "prediction_raw.jsonl"
    )
    prediction_messages.sort(key=lambda row: to_float(row["header_timestamp_sec"]) or -math.inf)
    prediction_rows: list[dict[str, Any]] = []
    prediction_time_by_seq: dict[int, float] = {}
    previous_input: Optional[float] = None
    prediction_trace_matches = 0
    prediction_history_available = 0
    for message in prediction_messages:
        raw = message["raw_message"]
        sequence = to_int(message["header_sequence_num"])
        output_time = to_float(message["header_timestamp_sec"])
        lidar_ns = to_int(message["header_lidar_timestamp"])
        lidar_s = lidar_seconds(lidar_ns)
        output_context = prediction_outputs.get(sequence) if sequence is not None else None
        input_time = input_time_from_trace(output_time, output_context, prediction_inputs)
        input_sequence = trace_input_sequence(output_context, prediction_inputs)
        if output_context is not None and input_time is not None:
            prediction_trace_matches += 1
        if sequence is not None and output_time is not None:
            prediction_time_by_seq[sequence] = output_time
        obstacles = raw.get("prediction_obstacle", []) or []
        perception_obstacles = [item.get("perception_obstacle", {}) for item in obstacles]
        ego = interpolate_state(ego_states, lidar_s) if lidar_s is not None else None
        target_perception = choose_target(perception_obstacles, ego, args.target_id)
        target = next(
            (item for item in obstacles if item.get("perception_obstacle") is target_perception),
            None,
        )
        history_length: Any = ""
        if target:
            history = target.get("history")
            if isinstance(history, list):
                history_length = len(history)
                prediction_history_available += 1
        trajectory_points = 0
        if target:
            for trajectory in target.get("trajectory", []) or []:
                trajectory_points += len(trajectory.get("trajectory_point", []) or [])
        prediction_rows.append(
            {
                "prediction_seq": int_or_blank(sequence),
                "prediction_input_time": fmt(input_time),
                "prediction_output_time": fmt(output_time),
                "input_perception_seq": int_or_blank(input_sequence),
                "lidar_timestamp": int_or_blank(lidar_ns),
                "input_interval_ms": fmt_ms((input_time - previous_input) * 1000) if input_time is not None and previous_input is not None else "",
                "input_data_age_ms": fmt_ms((input_time - lidar_s) * 1000) if input_time is not None and lidar_s is not None else "",
                "output_data_age_ms": fmt_ms((output_time - lidar_s) * 1000) if output_time is not None and lidar_s is not None else "",
                "target_id": to_int(target_perception.get("id")) if target_perception else "",
                "history_length": history_length,
                "trajectory_points": trajectory_points if target else "",
            }
        )
        previous_input = input_time

    planning_headers = read_csv(record_dir / "02_planning" / "planning_header.csv")
    planning_summary = {
        to_int(row["planning_seq"]): row
        for row in read_csv(record_dir / "02_planning" / "planning_trajectory_summary.csv")
    }
    planning_decisions: dict[int, str] = {}
    for row in read_csv(record_dir / "02_planning" / "planning_decision.csv"):
        sequence = to_int(row["planning_seq"])
        if sequence is None:
            continue
        try:
            decision = json.loads(row["decision_json"]) if row["decision_json"] else {}
            planning_decisions[sequence] = ";".join(sorted(decision))
        except json.JSONDecodeError:
            planning_decisions[sequence] = ""
    planning_headers.sort(key=lambda row: to_float(row["header_timestamp_sec"]) or -math.inf)
    planning_rows: list[dict[str, Any]] = []
    previous_pub = None
    planning_trace_matches = 0
    planning_prediction_timestamp_matches = 0
    for header in planning_headers:
        sequence = to_int(header["planning_seq"])
        pub = to_float(header["header_timestamp_sec"])
        lidar_ns = to_int(header["header_lidar_timestamp"])
        lidar_s = lidar_seconds(lidar_ns)
        output_context = planning_outputs.get(sequence) if sequence is not None else None
        start = input_time_from_trace(pub, output_context, planning_inputs)
        input_prediction_seq = trace_input_sequence(output_context, planning_inputs)
        prediction_timestamp = prediction_time_by_seq.get(input_prediction_seq)
        if output_context is not None and start is not None:
            planning_trace_matches += 1
        if prediction_timestamp is not None:
            planning_prediction_timestamp_matches += 1
        summary = planning_summary.get(sequence, {})
        planning_rows.append(
            {
                "planning_seq": int_or_blank(sequence),
                "planning_start": fmt(start),
                "planning_pub_time": fmt(pub),
                "input_prediction_seq": int_or_blank(input_prediction_seq),
                "prediction_timestamp": fmt(prediction_timestamp),
                "lidar_timestamp": int_or_blank(lidar_ns),
                "planning_data_age_ms": fmt_ms((pub - lidar_s) * 1000) if pub is not None and lidar_s is not None else "",
                "planning_interval_ms": fmt_ms((pub - previous_pub) * 1000) if pub is not None and previous_pub is not None else "",
                "trajectory_point_count": to_int(header["trajectory_point_size"]) or 0,
                "first_v": fmt(summary.get("first_traj_v")),
                "first_a": fmt(summary.get("first_traj_a")),
                "decision_type": planning_decisions.get(sequence, ""),
            }
        )
        previous_pub = pub

    control_headers = {
        to_int(row["control_seq"]): row
        for row in read_csv(record_dir / "03_control" / "control_header.csv")
    }
    control_reuse = {
        to_int(row["control_seq"]): row
        for row in read_csv(record_dir / "03_control" / "control_planning_reuse.csv")
    }
    control_commands = read_csv(record_dir / "03_control" / "control_command.csv")
    control_commands.sort(key=lambda row: to_float(row["header_timestamp_sec"]) or -math.inf)
    control_rows: list[dict[str, Any]] = []
    control_trace_matches = 0
    for command in control_commands:
        sequence = to_int(command["control_seq"])
        header = control_headers.get(sequence, {})
        control_time = to_float(command["header_timestamp_sec"])
        planning_lidar_ns = to_int(header.get("input_trajectory_header_lidar_timestamp"))
        planning_lidar_s = lidar_seconds(planning_lidar_ns)
        output_context = control_outputs.get(sequence) if sequence is not None else None
        if output_context is not None and trace_input_sequence(output_context, control_inputs) is not None:
            control_trace_matches += 1
        reuse = control_reuse.get(sequence, {})
        control_rows.append(
            {
                "control_seq": int_or_blank(sequence),
                "control_time": fmt(control_time),
                "planning_seq_used": int_or_blank(header.get("input_trajectory_header_sequence_num")),
                "planning_pub_time": fmt(header.get("input_trajectory_header_timestamp_sec")),
                "planning_lidar_timestamp": int_or_blank(planning_lidar_ns),
                "planning_data_age_at_control_ms": fmt_ms((control_time - planning_lidar_s) * 1000) if control_time is not None and planning_lidar_s is not None else "",
                "planning_reuse_count": int_or_blank(reuse.get("reuse_count")),
                "throttle": fmt(command.get("throttle")),
                "brake": fmt(command.get("brake")),
                "steering": fmt(command.get("steering_target")),
            }
        )

    rows_by_file = {
        "perception_propagation.csv": perception_rows,
        "prediction_propagation.csv": prediction_rows,
        "planning_propagation.csv": planning_rows,
        "control_propagation.csv": control_rows,
        "vehicle_state.csv": vehicle_rows,
        "target_obstacle_state.csv": target_state_rows,
    }
    output_paths: dict[str, Path] = {}
    for filename, rows in rows_by_file.items():
        module_dir, fields = TABLE_SPECS[filename]
        path = output_root / module_dir / "data" / filename
        write_csv(path, rows, fields)
        output_paths[filename] = path

    planning_lidar_count = sum(bool(row["lidar_timestamp"]) for row in planning_rows)
    quality = {
        "run_id": args.run_id,
        "run_directory": str(run_dir),
        "record_directory": str(record_dir),
        "output_root": str(output_root),
        "status": "pass_with_documented_limitations",
        "time_and_unit_definitions": {
            "module_publish_times": "Apollo message header.timestamp_sec, Unix seconds",
            "prediction_input_time": "output header time minus trace monotonic out-in duration",
            "planning_start": "output header time minus trace monotonic out-in duration",
            "lidar_timestamp": "original Apollo header.lidar_timestamp, nanoseconds",
            "data_age_ms": "module header timestamp minus lidar_timestamp/1e9",
            "vehicle_timestamp": "localization measurement_time, Unix seconds",
            "vehicle_acceleration": "localization world acceleration projected onto ego heading, m/s^2",
            "vehicle_actuation": "nearest chassis feedback percentages",
            "relative_distance": "ego-to-target center distance in XY, meters",
            "relative_speed": "positive line-of-sight closing speed, m/s",
            "TTC": "center distance / positive closing speed; blank when not closing",
            "planning_reuse_count": "total control uses of the referenced planning sequence within the record window",
        },
        "row_counts": {name: len(rows) for name, rows in rows_by_file.items()},
        "linkage": {
            "perception_source_frame_matches": sum(row["source_frame"] != "" for row in perception_rows),
            "perception_message_count": len(perception_rows),
            "prediction_trace_matches": prediction_trace_matches,
            "prediction_message_count": len(prediction_rows),
            "planning_trace_matches": planning_trace_matches,
            "planning_message_count": len(planning_rows),
            "planning_prediction_timestamp_matches": planning_prediction_timestamp_matches,
            "control_trace_matches": control_trace_matches,
            "control_message_count": len(control_rows),
        },
        "critical_field_checks": {
            "planning_header_lidar_timestamp_nonzero_count": planning_lidar_count,
            "planning_header_lidar_timestamp_total_count": len(planning_rows),
            "planning_header_lidar_timestamp_full_coverage": planning_lidar_count == len(planning_rows),
            "prediction_history_serialized_count": prediction_history_available,
            "prediction_history_note": "PredictionObstacles output does not serialize internal history in this run; history_length is blank",
            "prediction_target_trajectory_points_nonzero_count": sum(
                bool(row["trajectory_points"] and int(row["trajectory_points"]) > 0)
                for row in prediction_rows
            ),
            "target_selection": (
                f"fixed obstacle id {args.target_id}"
                if args.target_id is not None
                else "nearest obstacle to interpolated ego state per message"
            ),
            "selected_target_id_counts": dict(sorted(target_ids.items())),
            "target_rows_with_ego_state": sum(bool(row["ego_x"]) for row in target_state_rows),
        },
        "vehicle_state_quality": vehicle_quality,
        "source_files": {
            "source_frames": str(source_frames_path) if source_frames_path.is_file() else None,
            "perception_raw": str(record_dir / "04_prediction_perception" / "perception_raw.jsonl"),
            "prediction_raw": str(record_dir / "04_prediction_perception" / "prediction_raw.jsonl"),
            "planning_header": str(record_dir / "02_planning" / "planning_header.csv"),
            "control_header": str(record_dir / "03_control" / "control_header.csv"),
            "localization_pose": str(record_dir / "01_localization_chassis" / "localization_pose.csv"),
        },
    }
    if not quality["critical_field_checks"]["planning_header_lidar_timestamp_full_coverage"]:
        raise RuntimeError("planning.header.lidar_timestamp is not fully populated")
    if prediction_trace_matches != len(prediction_rows):
        raise RuntimeError("prediction record-to-trace linkage is incomplete")
    if planning_trace_matches != len(planning_rows):
        raise RuntimeError("planning record-to-trace linkage is incomplete")
    if control_trace_matches != len(control_rows):
        raise RuntimeError("control record-to-trace linkage is incomplete")

    audit_dir = output_root / "消息时序与车辆状态数据统计"
    audit_dir.mkdir(parents=True, exist_ok=True)
    quality_path = audit_dir / "data_quality_summary.json"
    quality_path.write_text(json.dumps(quality, ensure_ascii=False, indent=2), encoding="utf-8")

    manifest_rows = []
    for path in [*output_paths.values(), quality_path]:
        manifest_rows.append(
            {
                "file": path.relative_to(output_root).as_posix(),
                "size_bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        )
    manifest_path = audit_dir / "output_manifest.csv"
    write_csv(manifest_path, manifest_rows, ["file", "size_bytes", "sha256"])

    print(json.dumps({
        "output_root": str(output_root),
        "files": {name: str(path) for name, path in output_paths.items()},
        "quality_summary": str(quality_path),
        "manifest": str(manifest_path),
        "row_counts": quality["row_counts"],
        "planning_lidar_timestamp_full_coverage": True,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
