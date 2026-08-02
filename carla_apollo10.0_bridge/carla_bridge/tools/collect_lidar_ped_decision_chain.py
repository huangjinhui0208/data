#!/usr/bin/env python3

"""Collect pedestrian perception-to-planning decision-chain evidence.

This script is read-only with respect to Apollo/CARLA configuration. It runs
the existing raw lidar walker bbox diagnostic while one Cyber node captures
the public Apollo topics needed to localize where a pedestrian disappears:

  raw walker hit -> perception -> prediction -> planning -> control/chassis

The typed messages are written in protobuf text format so the existing
lightweight summary remains useful without record replay or extra dependencies.
"""

import argparse
import csv
import datetime
import math
import os
import re
import shutil
import subprocess
import sys
import threading
from collections import Counter
from pathlib import Path

from google.protobuf import text_format
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacles,
)
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.prediction_msgs.prediction_obstacle_pb2 import (
    PredictionObstacles,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
DIAGNOSE_SCRIPT = (
    REPO_ROOT
    / "modules/carla_apollo10.0_bridge/carla_bridge/tools/"
    / "diagnose_lidar_walker_hits.py")

RAW_LIDAR_TOPIC = "/apollo/sensor/velodyne64/compensator/PointCloud2"
PERCEPTION_TOPIC = "/apollo/perception/obstacles"
PREDICTION_TOPIC = "/apollo/prediction"
PLANNING_TOPIC = "/apollo/planning"
CONTROL_TOPIC = "/apollo/control"
CHASSIS_TOPIC = "/apollo/canbus/chassis"
LOCALIZATION_TOPIC = "/apollo/localization/pose"

ECHO_TOPICS = [
    ("perception_obstacles", PERCEPTION_TOPIC),
    ("prediction", PREDICTION_TOPIC),
    ("planning", PLANNING_TOPIC),
    ("control", CONTROL_TOPIC),
    ("chassis", CHASSIS_TOPIC),
    ("localization", LOCALIZATION_TOPIC),
]

ECHO_TOPIC_SPECS = [
    ("perception_obstacles", PERCEPTION_TOPIC, PerceptionObstacles),
    ("prediction", PREDICTION_TOPIC, PredictionObstacles),
    ("planning", PLANNING_TOPIC, ADCTrajectory),
    ("control", CONTROL_TOPIC, ControlCommand),
    ("chassis", CHASSIS_TOPIC, Chassis),
    ("localization", LOCALIZATION_TOPIC, LocalizationEstimate),
]

CHANNEL_INFO_TOPICS = [
    ("raw_lidar", RAW_LIDAR_TOPIC),
    ("perception_obstacles", PERCEPTION_TOPIC),
    ("prediction", PREDICTION_TOPIC),
    ("planning", PLANNING_TOPIC),
    ("control", CONTROL_TOPIC),
    ("chassis", CHASSIS_TOPIC),
    ("localization", LOCALIZATION_TOPIC),
]

STOP_REASONS = [
    "STOP_REASON_PEDESTRIAN",
    "STOP_REASON_CROSSWALK",
    "STOP_REASON_OBSTACLE",
    "STOP_REASON_HEAD_VEHICLE",
    "STOP_REASON_EMERGENCY",
]


def timestamp_suffix():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def default_output_dir():
    return "/tmp/apollo_lidar_ped_decision_chain_{}".format(timestamp_suffix())


def ensure_output_dir(path):
    output_dir = Path(path).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def import_cyber():
    neo_paths = [
        "/opt/apollo/neo/lib/cyber/python/internal",
        "/opt/apollo/neo/lib/cyber/python/cyber/python",
        "/opt/apollo/neo/lib/cyber/python",
    ]
    for path in neo_paths:
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        from cyber_py3 import cyber
    except ImportError:
        from cyber.python.cyber_py3 import cyber
    return cyber


def write_text(path, text):
    path.write_text(text, encoding="utf-8")


class TypedTopicCapture:
    """Capture multiple typed Cyber topics through one uniquely named node."""

    def __init__(self, output_dir, cyber_module, topic_specs=ECHO_TOPIC_SPECS):
        self.output_dir = output_dir
        self.cyber = cyber_module
        self.topic_specs = topic_specs
        self.lock = threading.Lock()
        self.counts = {name: 0 for name, _topic, _message_type in topic_specs}
        self.files = {}
        self.readers = []
        self.node = None
        self.closed = False

    def _callback(self, name):
        def write_message(message):
            serialized = text_format.MessageToString(message)
            with self.lock:
                output_file = self.files.get(name)
                if output_file is None or output_file.closed:
                    return
                output_file.write(serialized)
                if not serialized.endswith("\n"):
                    output_file.write("\n")
                output_file.write("\n")
                output_file.flush()
                self.counts[name] += 1
        return write_message

    def start(self):
        self.cyber.init()
        self.node = self.cyber.Node("carla_ped_decision_chain_{}".format(os.getpid()))
        for name, topic, message_type in self.topic_specs:
            output_file = (self.output_dir / "echo_{}.txt".format(name)).open(
                "w", encoding="utf-8")
            output_file.write("$ typed cyber reader {}\n\n".format(topic))
            output_file.flush()
            self.files[name] = output_file
            reader = self.node.create_reader(topic, message_type, self._callback(name))
            self.readers.append(reader)

    def close(self):
        if self.closed:
            return dict(self.counts)
        self.closed = True
        shutdown = getattr(self.cyber, "shutdown", None)
        if callable(shutdown):
            shutdown()
        with self.lock:
            for output_file in self.files.values():
                if not output_file.closed:
                    output_file.close()
        return dict(self.counts)


def run_command_to_file(command, output_path, timeout=None, cwd=REPO_ROOT):
    header = "$ {}\n\n".format(" ".join(command))
    try:
        result = subprocess.run(
            command,
            cwd=str(cwd),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False)
        write_text(
            output_path,
            header + result.stdout + "\nexit_code: {}\n".format(result.returncode))
        return result.returncode
    except FileNotFoundError as exc:
        write_text(output_path, header + "ERROR: {}\n".format(exc))
        return 127
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        write_text(
            output_path,
            header + output + "\nTIMEOUT after {:.1f}s\n".format(timeout))
        return 124


def start_command_to_file(command, output_path, cwd=REPO_ROOT):
    fout = output_path.open("w", encoding="utf-8")
    fout.write("$ {}\n\n".format(" ".join(command)))
    fout.flush()
    try:
        process = subprocess.Popen(
            command,
            cwd=str(cwd),
            text=True,
            stdout=fout,
            stderr=subprocess.STDOUT)
    except FileNotFoundError as exc:
        fout.write("ERROR: {}\n".format(exc))
        fout.close()
        return None, None
    return process, fout


def wait_process(process, output_file, timeout):
    if process is None:
        return 127
    try:
        return_code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return_code = process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            return_code = process.wait()
        output_file.write("\nterminated after timeout {:.1f}s\n".format(timeout))
    output_file.write("\nexit_code: {}\n".format(return_code))
    output_file.close()
    return return_code


def capture_channel_info(output_dir):
    for name, topic in CHANNEL_INFO_TOPICS:
        run_command_to_file(
            ["cyber_channel", "info", topic],
            output_dir / "channel_info_{}.txt".format(name),
            timeout=10.0)


def capture_actor_watch(args, output_dir):
    command = [
        sys.executable,
        str(DIAGNOSE_SCRIPT),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--duration",
        str(args.actor_watch_duration),
        "--sample-period",
        str(args.actor_watch_sample_period),
        "--watch-actors",
    ]
    return run_command_to_file(
        command,
        output_dir / "carla_actor_watch.txt",
        timeout=args.actor_watch_duration + 20.0)


def collect_parallel(args, output_dir):
    raw_csv = output_dir / "lidar_walker_hits_decision.csv"
    raw_stdout = output_dir / "lidar_walker_hits_stdout.txt"

    raw_command = [
        sys.executable,
        str(DIAGNOSE_SCRIPT),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--duration",
        str(args.duration),
        "--walker-filter",
        args.walker_filter,
        "--max-distance",
        str(args.max_distance),
        "--bbox-margin",
        str(args.bbox_margin),
        "--sample-period",
        str(args.sample_period),
        "--actor-wait-timeout",
        str(args.actor_wait_timeout),
        "--output-csv",
        str(raw_csv),
    ]

    typed_capture = None
    capture_counts = {name: 0 for name, _topic in ECHO_TOPICS}
    capture_error = None
    try:
        typed_capture = TypedTopicCapture(output_dir, import_cyber())
        typed_capture.start()
    except Exception as exc:  # pylint: disable=broad-except
        capture_error = "{}: {}".format(type(exc).__name__, exc)
        if typed_capture is not None:
            capture_counts = typed_capture.close()
        for name, topic in ECHO_TOPICS:
            output_path = output_dir / "echo_{}.txt".format(name)
            if not output_path.exists():
                write_text(
                    output_path,
                    "$ typed cyber reader {}\n\nERROR: {}\n".format(
                        topic, capture_error))

    raw_process, raw_file = start_command_to_file(raw_command, raw_stdout)

    return_codes = {}
    return_codes["raw_walker_hits"] = wait_process(
        raw_process, raw_file, args.duration + 30.0)

    if typed_capture is not None and not typed_capture.closed:
        capture_counts = typed_capture.close()
    for name, _topic in ECHO_TOPICS:
        return_codes[name] = 0 if capture_counts[name] > 0 else 2
    if capture_error:
        return_codes["typed_capture_start"] = 1
    return return_codes, capture_counts


def read_csv_rows(path):
    if not path.exists():
        return []
    rows = []
    with path.open(newline="", encoding="utf-8") as fin:
        reader = csv.DictReader(fin)
        for row in reader:
            try:
                row["measurement_time"] = float(row.get("measurement_time") or 0.0)
                row["header_time"] = float(row.get("header_time") or 0.0)
                row["distance_xy"] = float(row["distance_xy"])
                row["points_in_bbox"] = int(row["points_in_bbox"])
                row["total_points"] = int(row.get("total_points") or 0)
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(row)
    return rows


def read_file(path):
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def split_echo_messages(text):
    messages = []
    current = []
    for line in text.splitlines():
        if line.startswith("$ ") or line.startswith("I") or line.startswith("exit_code:"):
            continue
        if line.startswith("header {") and current:
            messages.append("\n".join(current).strip())
            current = []
        if line.strip():
            current.append(line)
    if current:
        messages.append("\n".join(current).strip())
    return [message for message in messages if "header {" in message]


def first_float(pattern, text, default=float("nan")):
    match = re.search(pattern, text)
    if not match:
        return default
    try:
        return float(match.group(1))
    except ValueError:
        return default


def first_int(pattern, text, default=None):
    match = re.search(pattern, text)
    if not match:
        return default
    try:
        return int(match.group(1))
    except ValueError:
        return default


def first_enum(pattern, text, default=""):
    match = re.search(pattern, text)
    return match.group(1) if match else default


def message_time(text):
    return first_float(r"timestamp_sec:\s*([-+0-9.eE]+)", text)


def extract_named_blocks(text, name):
    lines = text.splitlines()
    blocks = []
    i = 0
    start_pattern = "{} {{".format(name)
    while i < len(lines):
        if lines[i].strip() != start_pattern:
            i += 1
            continue
        depth = 0
        block = []
        while i < len(lines):
            line = lines[i]
            block.append(line)
            depth += line.count("{")
            depth -= line.count("}")
            i += 1
            if depth == 0:
                break
        blocks.append("\n".join(block))
    return blocks


def extract_top_level_blocks(text, name):
    lines = text.splitlines()
    blocks = []
    i = 0
    depth = 0
    start_pattern = "{} {{".format(name)
    while i < len(lines):
        stripped = lines[i].strip()
        if depth == 0 and stripped == start_pattern:
            block = []
            while i < len(lines):
                line = lines[i]
                block.append(line)
                depth += line.count("{")
                depth -= line.count("}")
                i += 1
                if depth == 0:
                    break
            blocks.append("\n".join(block))
            continue
        depth += lines[i].count("{")
        depth -= lines[i].count("}")
        i += 1
    return blocks


def parse_position(text):
    match = re.search(r"position\s*\{(.*?)\n\s*\}", text, re.S)
    if not match:
        return (float("nan"), float("nan"), float("nan"))
    block = match.group(1)
    return (
        first_float(r"\bx:\s*([-+0-9.eE]+)", block),
        first_float(r"\by:\s*([-+0-9.eE]+)", block),
        first_float(r"\bz:\s*([-+0-9.eE]+)", block),
    )


def parse_velocity(text):
    match = re.search(r"velocity\s*\{(.*?)\n\s*\}", text, re.S)
    if not match:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    block = match.group(1)
    vx = first_float(r"\bx:\s*([-+0-9.eE]+)", block)
    vy = first_float(r"\by:\s*([-+0-9.eE]+)", block)
    vz = first_float(r"\bz:\s*([-+0-9.eE]+)", block)
    speed = math.hypot(vx, vy) if math.isfinite(vx) and math.isfinite(vy) else float("nan")
    return (vx, vy, vz, speed)


def parse_perception_obstacle(block):
    x, y, z = parse_position(block)
    vx, vy, vz, speed = parse_velocity(block)
    return {
        "id": first_int(r"\bid:\s*(-?\d+)", block),
        "type": first_enum(r"\btype:\s*([A-Z_0-9]+)", block),
        "sub_type": first_enum(r"\bsub_type:\s*([A-Z_0-9]+)", block),
        "length": first_float(r"\blength:\s*([-+0-9.eE]+)", block),
        "width": first_float(r"\bwidth:\s*([-+0-9.eE]+)", block),
        "height": first_float(r"\bheight:\s*([-+0-9.eE]+)", block),
        "confidence": first_float(r"\bconfidence:\s*([-+0-9.eE]+)", block),
        "timestamp": first_float(r"\btimestamp:\s*([-+0-9.eE]+)", block),
        "x": x,
        "y": y,
        "z": z,
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "speed": speed,
    }


def parse_perception_echo(path):
    frames = []
    for message in split_echo_messages(read_file(path)):
        obstacles = [
            parse_perception_obstacle(block)
            for block in extract_named_blocks(message, "perception_obstacle")
        ]
        pedestrians = [item for item in obstacles if item["type"] == "PEDESTRIAN"]
        stable = [
            item for item in pedestrians
            if finite_ge(item["length"], 0.1)
            and finite_ge(item["width"], 0.1)
            and finite_ge(item["height"], 0.5)
        ]
        frames.append({
            "time": message_time(message),
            "pedestrians": pedestrians,
            "stable_pedestrians": stable,
            "vehicles": [item for item in obstacles if item["type"] == "VEHICLE"],
            "obstacle_count": len(obstacles),
        })
    return frames


def parse_prediction_echo(path):
    frames = []
    for message in split_echo_messages(read_file(path)):
        prediction_blocks = extract_named_blocks(message, "prediction_obstacle")
        pedestrians = []
        for block in prediction_blocks:
            obstacle_blocks = extract_named_blocks(block, "perception_obstacle")
            parsed = parse_perception_obstacle(obstacle_blocks[0] if obstacle_blocks else block)
            parsed["trajectory_count"] = len(extract_named_blocks(block, "trajectory"))
            parsed["priority"] = first_enum(r"\bpriority:\s*([A-Z_0-9]+)", block)
            parsed["interactive_tag"] = first_enum(
                r"\binteractive_tag:\s*([A-Z_0-9]+)", block)
            if parsed["type"] == "PEDESTRIAN":
                pedestrians.append(parsed)
        frames.append({
            "time": message_time(message),
            "pedestrians": pedestrians,
            "prediction_count": len(prediction_blocks),
        })
    return frames


def parse_trajectory_speeds(message):
    speeds = []
    for block in extract_top_level_blocks(message, "trajectory_point"):
        value = first_float(r"\bv:\s*([-+0-9.eE]+)", block)
        if math.isfinite(value):
            speeds.append(value)
    return speeds


def parse_planning_echo(path):
    frames = []
    for message in split_echo_messages(read_file(path)):
        object_decision_count = message.count("object_decision {")
        stop_count = len(re.findall(r"\bstop\s*\{", message))
        yield_count = len(re.findall(r"\byield\s*\{", message))
        ignore_count = len(re.findall(r"\bignore\s*\{", message))
        follow_count = len(re.findall(r"\bfollow\s*\{", message))
        reasons = [reason for reason in STOP_REASONS if reason in message]
        speeds = parse_trajectory_speeds(message)
        frames.append({
            "time": message_time(message),
            "object_decision_count": object_decision_count,
            "stop_count": stop_count,
            "yield_count": yield_count,
            "ignore_count": ignore_count,
            "follow_count": follow_count,
            "stop_reasons": reasons,
            "trajectory_first_v": speeds[0] if speeds else float("nan"),
            "trajectory_min_v": min(speeds) if speeds else float("nan"),
            "trajectory_points": len(speeds),
            "estop": "estop {" in message,
        })
    return frames


def parse_control_echo(path):
    frames = []
    for message in split_echo_messages(read_file(path)):
        frames.append({
            "time": message_time(message),
            "throttle": first_float(r"\bthrottle:\s*([-+0-9.eE]+)", message),
            "brake": first_float(r"\bbrake:\s*([-+0-9.eE]+)", message),
            "speed": first_float(r"\bspeed:\s*([-+0-9.eE]+)", message),
            "acceleration": first_float(r"\bacceleration:\s*([-+0-9.eE]+)", message),
        })
    return frames


def parse_chassis_echo(path):
    frames = []
    for message in split_echo_messages(read_file(path)):
        frames.append({
            "time": message_time(message),
            "speed_mps": first_float(r"\bspeed_mps:\s*([-+0-9.eE]+)", message),
            "throttle_percentage": first_float(
                r"\bthrottle_percentage:\s*([-+0-9.eE]+)", message),
            "brake_percentage": first_float(
                r"\bbrake_percentage:\s*([-+0-9.eE]+)", message),
            "driving_mode": first_enum(r"\bdriving_mode:\s*([A-Z_0-9]+)", message),
        })
    return frames


def finite_ge(value, threshold):
    return math.isfinite(value) and value >= threshold


def finite_values(items, key):
    values = [item[key] for item in items if math.isfinite(item.get(key, float("nan")))]
    return values


def nearest_frames(frames, start, end):
    return [
        frame for frame in frames
        if math.isfinite(frame["time"]) and start <= frame["time"] <= end
    ]


def build_focus_windows(raw_rows, args):
    event_times = sorted({
        row["measurement_time"] or row["header_time"]
        for row in raw_rows
        if row["distance_xy"] <= args.focus_max_distance
        and row["points_in_bbox"] >= args.focus_min_points
        and (row["measurement_time"] or row["header_time"]) > 0.0
    })
    windows = []
    for event_time in event_times:
        start = event_time - args.focus_window_sec
        end = event_time + args.focus_window_sec
        if windows and start <= windows[-1]["end"]:
            windows[-1]["end"] = max(windows[-1]["end"], end)
            windows[-1]["event_count"] += 1
        else:
            windows.append({"start": start, "end": end, "event_count": 1})
    return windows


def count_frames_with(frames, predicate):
    return sum(1 for frame in frames if predicate(frame))


def summarize_window(index, window, raw_rows, perception, prediction, planning, control, chassis):
    start = window["start"]
    end = window["end"]
    raw = [
        row for row in raw_rows
        if start <= (row["measurement_time"] or row["header_time"]) <= end
    ]
    raw_focus = [
        row for row in raw
        if row["points_in_bbox"] >= 3 and row["distance_xy"] <= 30.0
    ]
    perception_frames = nearest_frames(perception, start, end)
    prediction_frames = nearest_frames(prediction, start, end)
    planning_frames = nearest_frames(planning, start, end)
    control_frames = nearest_frames(control, start, end)
    chassis_frames = nearest_frames(chassis, start, end)

    ped_frames = count_frames_with(perception_frames, lambda frame: frame["pedestrians"])
    stable_ped_frames = count_frames_with(
        perception_frames, lambda frame: frame["stable_pedestrians"])
    pred_ped_frames = count_frames_with(prediction_frames, lambda frame: frame["pedestrians"])
    planning_stop_frames = count_frames_with(
        planning_frames,
        lambda frame: frame["stop_count"] > 0 or frame["yield_count"] > 0)

    control_brakes = finite_values(control_frames, "brake")
    chassis_brakes = finite_values(chassis_frames, "brake_percentage")
    chassis_speeds = finite_values(chassis_frames, "speed_mps")
    planning_min_speeds = finite_values(planning_frames, "trajectory_min_v")
    confidence_values = [
        ped["confidence"]
        for frame in perception_frames
        for ped in frame["pedestrians"]
        if math.isfinite(ped["confidence"])
    ]
    lengths = [
        ped["length"]
        for frame in perception_frames
        for ped in frame["pedestrians"]
        if math.isfinite(ped["length"])
    ]
    widths = [
        ped["width"]
        for frame in perception_frames
        for ped in frame["pedestrians"]
        if math.isfinite(ped["width"])
    ]

    if raw_focus and stable_ped_frames == 0:
        diagnosis = "perception_unstable"
    elif stable_ped_frames > 0 and pred_ped_frames == 0:
        diagnosis = "prediction_missing"
    elif pred_ped_frames > 0 and planning_stop_frames == 0:
        diagnosis = "planning_no_stop_decision"
    elif planning_stop_frames > 0 and max(control_brakes or [0.0]) <= 0.0 and max(chassis_brakes or [0.0]) <= 0.0:
        diagnosis = "control_no_brake"
    elif stable_ped_frames > 0:
        diagnosis = "dreamview_display_only_or_intermittent"
    else:
        diagnosis = "insufficient_window_evidence"

    reasons = Counter()
    driving_modes = Counter()
    for frame in planning_frames:
        reasons.update(frame["stop_reasons"])
    for frame in chassis_frames:
        if frame["driving_mode"]:
            driving_modes.update([frame["driving_mode"]])

    min_distance = min((row["distance_xy"] for row in raw), default=float("nan"))
    max_points = max((row["points_in_bbox"] for row in raw), default=0)
    return [
        "Window {} [{:.3f}, {:.3f}] events={} diagnosis={}".format(
            index, start, end, window["event_count"], diagnosis),
        "  raw rows={} focus_rows={} min_distance={:.2f}m max_points_in_bbox={}".format(
            len(raw), len(raw_focus), min_distance, max_points),
        "  perception frames={} ped_frames={} stable_ped_frames={} avg_confidence={:.3f} max_length={:.3f} max_width={:.3f}".format(
            len(perception_frames),
            ped_frames,
            stable_ped_frames,
            avg(confidence_values),
            max(lengths or [float("nan")]),
            max(widths or [float("nan")])),
        "  prediction frames={} ped_frames={}".format(
            len(prediction_frames), pred_ped_frames),
        "  planning frames={} stop_or_yield_frames={} stop_reasons={} min_trajectory_v={:.3f}".format(
            len(planning_frames),
            planning_stop_frames,
            dict(reasons),
            min(planning_min_speeds or [float("nan")])),
        "  control frames={} max_brake={:.3f} min_target_speed={:.3f}".format(
            len(control_frames),
            max(control_brakes or [float("nan")]),
            min(finite_values(control_frames, "speed") or [float("nan")])),
        "  chassis frames={} max_brake_percentage={:.3f} min_speed_mps={:.3f} driving_modes={}".format(
            len(chassis_frames),
            max(chassis_brakes or [float("nan")]),
            min(chassis_speeds or [float("nan")]),
            dict(driving_modes)),
    ]


def avg(values):
    if not values:
        return float("nan")
    return sum(values) / len(values)


def summarize_all(output_dir, return_codes, capture_counts, args):
    raw_rows = read_csv_rows(output_dir / "lidar_walker_hits_decision.csv")
    perception = parse_perception_echo(output_dir / "echo_perception_obstacles.txt")
    prediction = parse_prediction_echo(output_dir / "echo_prediction.txt")
    planning = parse_planning_echo(output_dir / "echo_planning.txt")
    control = parse_control_echo(output_dir / "echo_control.txt")
    chassis = parse_chassis_echo(output_dir / "echo_chassis.txt")

    raw_focus = [
        row for row in raw_rows
        if row["distance_xy"] <= args.focus_max_distance
        and row["points_in_bbox"] >= args.focus_min_points
    ]
    windows = build_focus_windows(raw_rows, args)

    perception_ped_frames = count_frames_with(perception, lambda frame: frame["pedestrians"])
    perception_stable_frames = count_frames_with(
        perception, lambda frame: frame["stable_pedestrians"])
    prediction_ped_frames = count_frames_with(prediction, lambda frame: frame["pedestrians"])
    planning_stop_frames = count_frames_with(
        planning,
        lambda frame: frame["stop_count"] > 0 or frame["yield_count"] > 0)

    lines = [
        "Apollo-CARLA lidar pedestrian decision-chain summary",
        "generated_at: {}".format(datetime.datetime.now().isoformat()),
        "output_dir: {}".format(output_dir),
        "",
        "Parameters",
        "  duration: {}".format(args.duration),
        "  host: {}".format(args.host),
        "  port: {}".format(args.port),
        "  walker_filter: {}".format(args.walker_filter),
        "  max_distance: {}".format(args.max_distance),
        "  bbox_margin: {}".format(args.bbox_margin),
        "  focus_max_distance: {}".format(args.focus_max_distance),
        "  focus_min_points: {}".format(args.focus_min_points),
        "  focus_window_sec: {}".format(args.focus_window_sec),
        "",
        "Command status",
    ]
    for name in sorted(return_codes):
        lines.append("  {} exit_code: {}".format(name, return_codes[name]))

    lines.extend([
        "",
        "Typed topic capture counts",
    ])
    for name, _topic in ECHO_TOPICS:
        count = capture_counts.get(name, 0)
        status = "ok" if count > 0 else "FAILED: zero messages"
        lines.append("  {}: {} ({})".format(name, count, status))

    lines.extend([
        "",
        "Overall chain counts",
        "  raw rows: {}".format(len(raw_rows)),
        "  raw focus rows: {}".format(len(raw_focus)),
        "  perception frames: {}".format(len(perception)),
        "  perception ped frames: {}".format(perception_ped_frames),
        "  perception stable ped frames: {}".format(perception_stable_frames),
        "  prediction frames: {}".format(len(prediction)),
        "  prediction ped frames: {}".format(prediction_ped_frames),
        "  planning frames: {}".format(len(planning)),
        "  planning stop/yield frames: {}".format(planning_stop_frames),
        "  control frames: {}".format(len(control)),
        "  chassis frames: {}".format(len(chassis)),
        "  focus windows: {}".format(len(windows)),
        "",
        "Window diagnosis",
    ])
    if not windows:
        lines.append("  no focus windows: no raw rows met distance/point thresholds")
    for index, window in enumerate(windows, 1):
        lines.extend(summarize_window(
            index,
            window,
            raw_rows,
            perception,
            prediction,
            planning,
            control,
            chassis))

    lines.extend([
        "",
        "Interpretation guide",
        "  perception_unstable: raw walker points exist, but final perception has no stable pedestrian box.",
        "  prediction_missing: stable perception pedestrian exists, but prediction has no pedestrian.",
        "  planning_no_stop_decision: prediction has pedestrian, but planning has no stop/yield evidence.",
        "  control_no_brake: planning has stop/yield evidence, but control/chassis braking is absent.",
        "  dreamview_display_only_or_intermittent: public topics contain pedestrian evidence; inspect Dreamview display/filtering and temporal stability.",
        "",
        "Generated files",
    ])
    for path in sorted(output_dir.iterdir()):
        if path.is_file():
            lines.append("  {}".format(path.name))
    write_text(output_dir / "decision_chain_summary.txt", "\n".join(lines) + "\n")


def copy_if_requested(output_dir, archive_path):
    if not archive_path:
        return None
    archive_path = Path(archive_path).expanduser().resolve()
    base_name = str(archive_path)
    if base_name.endswith(".zip"):
        base_name = base_name[:-4]
    return shutil.make_archive(base_name, "zip", output_dir)


def write_skipped_live_outputs(output_dir):
    for name, _topic in CHANNEL_INFO_TOPICS:
        write_text(
            output_dir / "channel_info_{}.txt".format(name),
            "SKIPPED: --skip-live-capture was set.\n")
    for name, _topic in ECHO_TOPICS:
        write_text(
            output_dir / "echo_{}.txt".format(name),
            "SKIPPED: --skip-live-capture was set.\n")
    write_text(output_dir / "carla_actor_watch.txt", "SKIPPED: --skip-live-capture was set.\n")
    write_text(output_dir / "lidar_walker_hits_stdout.txt", "SKIPPED: --skip-live-capture was set.\n")
    write_text(output_dir / "lidar_walker_hits_decision.csv", "")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--duration", default=120.0, type=float)
    parser.add_argument("--max-distance", default=50.0, type=float)
    parser.add_argument("--bbox-margin", default=0.3, type=float)
    parser.add_argument("--sample-period", default=0.5, type=float)
    parser.add_argument("--walker-filter", default="walker.pedestrian.*")
    parser.add_argument("--actor-wait-timeout", default=30.0, type=float)
    parser.add_argument("--actor-watch-duration", default=10.0, type=float)
    parser.add_argument("--actor-watch-sample-period", default=1.0, type=float)
    parser.add_argument("--focus-window-sec", default=6.0, type=float)
    parser.add_argument("--focus-max-distance", default=30.0, type=float)
    parser.add_argument("--focus-min-points", default=3, type=int)
    parser.add_argument("--output-dir", default=default_output_dir())
    parser.add_argument(
        "--archive",
        default=None,
        help="Optional zip archive path for the collected output directory.")
    parser.add_argument(
        "--skip-live-capture",
        action="store_true",
        help="Only create empty live-capture files and summary; useful for smoke tests.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    output_dir = ensure_output_dir(args.output_dir)

    print("collecting decision-chain outputs into {}".format(output_dir))
    if args.skip_live_capture:
        write_skipped_live_outputs(output_dir)
        return_codes = {"skip_live_capture": 0}
        capture_counts = {}
    else:
        capture_channel_info(output_dir)
        return_codes = {"actor_watch": capture_actor_watch(args, output_dir)}
        live_return_codes, capture_counts = collect_parallel(args, output_dir)
        return_codes.update(live_return_codes)

    summarize_all(output_dir, return_codes, capture_counts, args)
    archive = copy_if_requested(output_dir, args.archive)

    print("summary written to {}".format(output_dir / "decision_chain_summary.txt"))
    if archive:
        print("archive written to {}".format(archive))
    return return_codes.get("raw_walker_hits", 0)


if __name__ == "__main__":
    sys.exit(main())
