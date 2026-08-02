#!/usr/bin/env python3

"""Correlate raw CARLA walker lidar hits with Apollo pedestrian box quality.

This offline, read-only analyzer uses an existing decision-chain output
directory. It intentionally performs time-window correlation only: the raw CSV
does not contain walker world positions, so this script does not claim
actor-to-obstacle matching.
"""

import argparse
import csv
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path


DEFAULT_INPUT_DIR = (
    "modules/carla_apollo10.0_bridge/carla_bridge/testdata/"
    "apollo_lidar_ped_decision_chain_fixed"
)


def finite(value):
    return isinstance(value, float) and math.isfinite(value)


def parse_float(value, default=float("nan")):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed


def parse_int(value, default=0):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def first_float(pattern, text, default=float("nan")):
    match = re.search(pattern, text)
    if not match:
        return default
    return parse_float(match.group(1), default)


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


def avg(values):
    values = [value for value in values if finite(value)]
    if not values:
        return float("nan")
    return sum(values) / len(values)


def fmt(value, precision=3):
    if not finite(value):
        return "nan"
    return ("{:.%df}" % precision).format(value)


def read_raw_rows(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as fin:
        reader = csv.DictReader(fin)
        for row in reader:
            measurement_time = parse_float(row.get("measurement_time"))
            header_time = parse_float(row.get("header_time"))
            timestamp = measurement_time if measurement_time > 0.0 else header_time
            if not finite(timestamp) or timestamp <= 0.0:
                continue
            rows.append({
                "time": timestamp,
                "measurement_time": measurement_time,
                "header_time": header_time,
                "actor_id": row.get("actor_id", ""),
                "actor_type": row.get("actor_type", ""),
                "role_name": row.get("role_name", ""),
                "distance_xy": parse_float(row.get("distance_xy")),
                "points_in_bbox": parse_int(row.get("points_in_bbox")),
                "total_points": parse_int(row.get("total_points")),
            })
    return rows


def split_echo_messages(text):
    messages = []
    current = []
    for line in text.splitlines():
        if line.startswith("$ ") or line.startswith("I"):
            continue
        if not line.strip():
            if current:
                messages.append("\n".join(current))
                current = []
            continue
        current.append(line)
    if current:
        messages.append("\n".join(current))
    return messages


def extract_top_level_blocks(text, name):
    lines = text.splitlines()
    blocks = []
    index = 0
    target = "{} {{".format(name)
    while index < len(lines):
        if lines[index].strip() != target:
            index += 1
            continue
        block = []
        depth = 0
        while index < len(lines):
            line = lines[index]
            block.append(line)
            depth += line.count("{")
            depth -= line.count("}")
            index += 1
            if depth == 0:
                break
        blocks.append("\n".join(block))
    return blocks


def parse_vector_block(name, text):
    match = re.search(r"\b{}\s*\{{(.*?)\n\s*\}}".format(name), text, re.S)
    if not match:
        return (float("nan"), float("nan"), float("nan"))
    block = match.group(1)
    return (
        first_float(r"\bx:\s*([-+0-9.eE]+)", block),
        first_float(r"\by:\s*([-+0-9.eE]+)", block),
        first_float(r"\bz:\s*([-+0-9.eE]+)", block),
    )


def parse_obstacle(block, tiny_threshold):
    x, y, z = parse_vector_block("position", block)
    vx, vy, vz = parse_vector_block("velocity", block)
    length = first_float(r"\blength:\s*([-+0-9.eE]+)", block)
    width = first_float(r"\bwidth:\s*([-+0-9.eE]+)", block)
    height = first_float(r"\bheight:\s*([-+0-9.eE]+)", block)
    confidence = first_float(r"\bconfidence:\s*([-+0-9.eE]+)", block)
    speed = math.hypot(vx, vy) if finite(vx) and finite(vy) else float("nan")
    tiny = (
        (finite(length) and length < tiny_threshold)
        or (finite(width) and width < tiny_threshold)
    )
    valid = (
        finite(length) and length >= tiny_threshold
        and finite(width) and width >= tiny_threshold
        and finite(height) and height >= 0.5
    )
    return {
        "id": first_int(r"\bid:\s*(-?\d+)", block),
        "type": first_enum(r"\btype:\s*([A-Z_0-9]+)", block),
        "sub_type": first_enum(r"\bsub_type:\s*([A-Z_0-9]+)", block),
        "confidence": confidence,
        "length": length,
        "width": width,
        "height": height,
        "x": x,
        "y": y,
        "z": z,
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "speed": speed,
        "tiny_box": tiny,
        "valid_box": valid,
    }


def message_time(message, obstacles):
    header_time = first_float(r"\btimestamp_sec:\s*([-+0-9.eE]+)", message)
    if finite(header_time):
        return header_time
    obstacle_times = [
        first_float(r"\btimestamp:\s*([-+0-9.eE]+)", block)
        for block in extract_top_level_blocks(message, "perception_obstacle")
    ]
    obstacle_times = [value for value in obstacle_times if finite(value)]
    if obstacle_times:
        return max(obstacle_times)
    if obstacles:
        return first_float(r"\btimestamp:\s*([-+0-9.eE]+)", message)
    return float("nan")


def read_perception_frames(path, tiny_threshold):
    frames = []
    text = path.read_text(encoding="utf-8", errors="replace")
    for message in split_echo_messages(text):
        obstacles = [
            parse_obstacle(block, tiny_threshold)
            for block in extract_top_level_blocks(message, "perception_obstacle")
        ]
        pedestrians = [obstacle for obstacle in obstacles if obstacle["type"] == "PEDESTRIAN"]
        timestamp = message_time(message, obstacles)
        if not finite(timestamp):
            continue
        frames.append({
            "time": timestamp,
            "pedestrians": pedestrians,
            "obstacle_count": len(obstacles),
        })
    frames.sort(key=lambda frame: frame["time"])
    return frames


def group_raw_by_frame(raw_rows):
    grouped = defaultdict(list)
    for row in raw_rows:
        grouped[row["time"]].append(row)
    return [(timestamp, grouped[timestamp]) for timestamp in sorted(grouped)]


def frames_in_window(frames, start, end):
    return [frame for frame in frames if start <= frame["time"] <= end]


def best_pedestrian(pedestrians):
    if not pedestrians:
        return None
    return max(
        pedestrians,
        key=lambda item: (
            item["confidence"] if finite(item["confidence"]) else -1.0,
            item["length"] if finite(item["length"]) else -1.0,
            item["width"] if finite(item["width"]) else -1.0,
        ),
    )


def classify_window(raw_max_points, high_raw_points, pedestrians, best, low_confidence):
    if raw_max_points <= high_raw_points:
        return "raw_not_high"
    if not pedestrians:
        return "raw_high_no_ped"
    if all(pedestrian["tiny_box"] for pedestrian in pedestrians):
        return "raw_high_tiny_only"
    if best is not None and finite(best["confidence"]) and best["confidence"] < low_confidence:
        return "raw_high_low_conf"
    if any(pedestrian["valid_box"] for pedestrian in pedestrians):
        return "raw_high_valid_box"
    return "raw_high_box_invalid"


def summarize_frame(raw_time, raw_rows, perception_frames, args):
    start = raw_time - args.window_sec
    end = raw_time + args.window_sec
    matched_frames = frames_in_window(perception_frames, start, end)
    pedestrians = [
        pedestrian
        for frame in matched_frames
        for pedestrian in frame["pedestrians"]
    ]
    best = best_pedestrian(pedestrians)
    points = [row["points_in_bbox"] for row in raw_rows]
    distances = [row["distance_xy"] for row in raw_rows if finite(row["distance_xy"])]
    raw_max_points = max(points) if points else 0
    raw_high_actor_count = sum(
        1 for row in raw_rows
        if row["points_in_bbox"] >= args.high_raw_points
    )
    valid_count = sum(1 for pedestrian in pedestrians if pedestrian["valid_box"])
    tiny_count = sum(1 for pedestrian in pedestrians if pedestrian["tiny_box"])
    classification = classify_window(
        raw_max_points,
        args.high_raw_points,
        pedestrians,
        best,
        args.low_confidence_threshold,
    )
    return {
        "raw_time": raw_time,
        "window_start": start,
        "window_end": end,
        "raw_actor_count": len(raw_rows),
        "raw_high_actor_count": raw_high_actor_count,
        "raw_max_points": raw_max_points,
        "raw_avg_points": avg([float(value) for value in points]),
        "raw_min_distance": min(distances) if distances else float("nan"),
        "raw_nearest_actor_id": min(
            raw_rows,
            key=lambda row: row["distance_xy"] if finite(row["distance_xy"]) else float("inf"),
        )["actor_id"] if raw_rows else "",
        "perception_frame_count": len(matched_frames),
        "pedestrian_count": len(pedestrians),
        "valid_pedestrian_count": valid_count,
        "tiny_pedestrian_count": tiny_count,
        "best_ped_id": best["id"] if best else "",
        "best_confidence": best["confidence"] if best else float("nan"),
        "best_length": best["length"] if best else float("nan"),
        "best_width": best["width"] if best else float("nan"),
        "best_height": best["height"] if best else float("nan"),
        "best_x": best["x"] if best else float("nan"),
        "best_y": best["y"] if best else float("nan"),
        "best_z": best["z"] if best else float("nan"),
        "classification": classification,
    }


def bucket_for_points(points):
    buckets = [
        (0, 0, "0"),
        (1, 2, "1-2"),
        (3, 5, "3-5"),
        (6, 10, "6-10"),
        (11, 20, "11-20"),
        (21, 50, "21-50"),
        (51, 10**9, "51+"),
    ]
    for low, high, label in buckets:
        if low <= points <= high:
            return label
    return "unknown"


def write_csv(path, summaries):
    fieldnames = [
        "raw_time",
        "window_start",
        "window_end",
        "raw_actor_count",
        "raw_high_actor_count",
        "raw_max_points",
        "raw_avg_points",
        "raw_min_distance",
        "raw_nearest_actor_id",
        "perception_frame_count",
        "pedestrian_count",
        "valid_pedestrian_count",
        "tiny_pedestrian_count",
        "best_ped_id",
        "best_confidence",
        "best_length",
        "best_width",
        "best_height",
        "best_x",
        "best_y",
        "best_z",
        "classification",
    ]
    with path.open("w", newline="", encoding="utf-8") as fout:
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(summary)


def format_example(summary):
    return (
        "  time={raw_time:.6f} class={classification} raw_max_points={raw_max_points} "
        "raw_min_distance={raw_min_distance}m ped_count={pedestrian_count} "
        "valid={valid_pedestrian_count} tiny={tiny_pedestrian_count} "
        "best_id={best_ped_id} conf={best_confidence} "
        "box=({best_length},{best_width},{best_height})"
    ).format(
        raw_time=summary["raw_time"],
        classification=summary["classification"],
        raw_max_points=summary["raw_max_points"],
        raw_min_distance=fmt(summary["raw_min_distance"], 2),
        pedestrian_count=summary["pedestrian_count"],
        valid_pedestrian_count=summary["valid_pedestrian_count"],
        tiny_pedestrian_count=summary["tiny_pedestrian_count"],
        best_ped_id=summary["best_ped_id"],
        best_confidence=fmt(summary["best_confidence"], 3),
        best_length=fmt(summary["best_length"], 3),
        best_width=fmt(summary["best_width"], 3),
        best_height=fmt(summary["best_height"], 3),
    )


def ratio(numerator, denominator):
    if denominator <= 0:
        return "nan"
    return "{:.1%}".format(float(numerator) / float(denominator))


def write_report(path, input_dir, raw_rows, perception_frames, summaries, args):
    classifications = Counter(summary["classification"] for summary in summaries)
    high = [
        summary for summary in summaries
        if summary["raw_max_points"] > args.high_raw_points
    ]
    high_problem = [
        summary for summary in high
        if summary["classification"] in (
            "raw_high_no_ped",
            "raw_high_tiny_only",
            "raw_high_low_conf",
            "raw_high_box_invalid",
        )
    ]
    high_tiny_or_no_ped = [
        summary for summary in high
        if summary["classification"] in ("raw_high_no_ped", "raw_high_tiny_only")
    ]
    bucket_stats = defaultdict(lambda: Counter())
    for summary in summaries:
        label = bucket_for_points(summary["raw_max_points"])
        bucket_stats[label]["windows"] += 1
        bucket_stats[label][summary["classification"]] += 1
        if summary["pedestrian_count"] > 0:
            bucket_stats[label]["with_ped"] += 1
        if summary["valid_pedestrian_count"] > 0:
            bucket_stats[label]["with_valid_ped"] += 1

    all_pedestrians = [
        pedestrian
        for frame in perception_frames
        for pedestrian in frame["pedestrians"]
    ]
    lines = [
        "Apollo-CARLA raw lidar vs pedestrian box quality report",
        "input_dir: {}".format(input_dir),
        "",
        "Correlation mode",
        "  v1 is time-window correlation only, not actor-to-obstacle matching.",
        "  raw windows are centered on each raw lidar measurement_time.",
        "  matched perception frames are in [raw_time - window_sec, raw_time + window_sec].",
        "",
        "Parameters",
        "  window_sec: {}".format(args.window_sec),
        "  high_raw_points: {}".format(args.high_raw_points),
        "  tiny_box_threshold: {}".format(args.tiny_box_threshold),
        "  low_confidence_threshold: {}".format(args.low_confidence_threshold),
        "",
        "Input counts",
        "  raw rows: {}".format(len(raw_rows)),
        "  raw frames: {}".format(len(summaries)),
        "  perception frames: {}".format(len(perception_frames)),
        "  perception pedestrians: {}".format(len(all_pedestrians)),
        "",
        "Window classification counts",
    ]
    for name in sorted(classifications):
        lines.append("  {}: {}".format(name, classifications[name]))

    lines.extend([
        "",
        "High raw point windows",
        "  high windows (raw_max_points > {}): {}".format(args.high_raw_points, len(high)),
        "  high problematic windows: {} ({})".format(
            len(high_problem), ratio(len(high_problem), len(high))),
        "  high no-ped or tiny-only windows: {} ({})".format(
            len(high_tiny_or_no_ped), ratio(len(high_tiny_or_no_ped), len(high))),
        "  high valid-box windows: {} ({})".format(
            classifications["raw_high_valid_box"],
            ratio(classifications["raw_high_valid_box"], len(high))),
        "",
        "Raw point bucket summary",
    ])
    for label in ("0", "1-2", "3-5", "6-10", "11-20", "21-50", "51+"):
        stats = bucket_stats.get(label)
        if not stats:
            continue
        windows = stats["windows"]
        lines.append(
            "  {:>5}: windows={} with_ped={} ({}) with_valid_ped={} ({}) "
            "tiny_only={} no_ped={} low_conf={}".format(
                label,
                windows,
                stats["with_ped"],
                ratio(stats["with_ped"], windows),
                stats["with_valid_ped"],
                ratio(stats["with_valid_ped"], windows),
                stats["raw_high_tiny_only"],
                stats["raw_high_no_ped"],
                stats["raw_high_low_conf"],
            )
        )

    confidences = [ped["confidence"] for ped in all_pedestrians if finite(ped["confidence"])]
    lengths = [ped["length"] for ped in all_pedestrians if finite(ped["length"])]
    widths = [ped["width"] for ped in all_pedestrians if finite(ped["width"])]
    heights = [ped["height"] for ped in all_pedestrians if finite(ped["height"])]
    tiny_count = sum(1 for ped in all_pedestrians if ped["tiny_box"])
    valid_count = sum(1 for ped in all_pedestrians if ped["valid_box"])
    lines.extend([
        "",
        "Overall perception pedestrian box quality",
        "  confidence avg/min/max: {}/{}/{}".format(
            fmt(avg(confidences)), fmt(min(confidences) if confidences else float("nan")),
            fmt(max(confidences) if confidences else float("nan"))),
        "  length avg/min/max: {}/{}/{}".format(
            fmt(avg(lengths)), fmt(min(lengths) if lengths else float("nan")),
            fmt(max(lengths) if lengths else float("nan"))),
        "  width avg/min/max: {}/{}/{}".format(
            fmt(avg(widths)), fmt(min(widths) if widths else float("nan")),
            fmt(max(widths) if widths else float("nan"))),
        "  height avg/min/max: {}/{}/{}".format(
            fmt(avg(heights)), fmt(min(heights) if heights else float("nan")),
            fmt(max(heights) if heights else float("nan"))),
        "  tiny pedestrians: {} ({})".format(
            tiny_count, ratio(tiny_count, len(all_pedestrians))),
        "  valid pedestrians: {} ({})".format(
            valid_count, ratio(valid_count, len(all_pedestrians))),
        "",
        "Interpretation",
        "  If high raw windows still classify as raw_high_tiny_only or raw_high_low_conf,",
        "  raw hit scarcity is not sufficient to explain the unstable pedestrian boxes.",
        "  That points toward CenterPoint box regression / CARLA lidar domain mismatch",
        "  or postprocess/tracking behavior that preserves low-quality pedestrian boxes.",
        "",
        "sl_boundary / st_boundary note",
        "  sl_boundary is an obstacle's spatial projection on the planning reference line:",
        "  s is longitudinal distance along the line, l is lateral offset from it.",
        "  st_boundary is the time-space occupancy used by speed planning:",
        "  t is future time, s is occupied longitudinal range along the reference line.",
        "  Degenerate perception length/width can shrink the SL boundary; an empty or",
        "  non-conflicting ST boundary lets SpeedDecider ignore the obstacle before",
        "  pedestrian stop logic can produce a useful stop decision.",
        "",
        "Typical high-raw problematic windows",
    ])
    examples = sorted(
        high_problem,
        key=lambda item: (
            item["raw_min_distance"] if finite(item["raw_min_distance"]) else 999.0,
            -item["raw_max_points"],
        ),
    )[:12]
    if not examples:
        lines.append("  none")
    for summary in examples:
        lines.append(format_example(summary))

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", default=DEFAULT_INPUT_DIR)
    parser.add_argument("--window-sec", default=0.5, type=float)
    parser.add_argument("--high-raw-points", default=20, type=int)
    parser.add_argument("--tiny-box-threshold", default=0.1, type=float)
    parser.add_argument("--low-confidence-threshold", default=0.25, type=float)
    parser.add_argument(
        "--output-csv",
        default="/tmp/lidar_ped_box_quality_windows.csv")
    parser.add_argument(
        "--output-report",
        default="/tmp/lidar_ped_box_quality_report.txt")
    return parser


def main():
    args = build_arg_parser().parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    raw_csv = input_dir / "lidar_walker_hits_decision.csv"
    perception_echo = input_dir / "echo_perception_obstacles.txt"
    if not raw_csv.exists():
        print("ERROR: raw CSV not found: {}".format(raw_csv), file=sys.stderr)
        return 1
    if not perception_echo.exists():
        print("ERROR: perception echo not found: {}".format(perception_echo), file=sys.stderr)
        return 1
    if args.window_sec <= 0.0:
        print("ERROR: --window-sec must be positive", file=sys.stderr)
        return 1

    raw_rows = read_raw_rows(raw_csv)
    perception_frames = read_perception_frames(
        perception_echo, args.tiny_box_threshold)
    raw_frames = group_raw_by_frame(raw_rows)
    summaries = [
        summarize_frame(raw_time, rows, perception_frames, args)
        for raw_time, rows in raw_frames
    ]

    output_csv = Path(args.output_csv).expanduser().resolve()
    output_report = Path(args.output_report).expanduser().resolve()
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    output_report.parent.mkdir(parents=True, exist_ok=True)
    write_csv(output_csv, summaries)
    write_report(output_report, input_dir, raw_rows, perception_frames, summaries, args)

    print("windows written to {}".format(output_csv))
    print("report written to {}".format(output_report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
