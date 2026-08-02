#!/usr/bin/env python3

"""Collect baseline evidence for CARLA walker lidar perception debugging.

This script is read-only with respect to Apollo and bridge configuration. It
creates an output directory, captures current config snippets, runs existing
diagnostic commands, records final perception output, greps perception logs,
and writes a compact summary.
"""

import argparse
import csv
import datetime
import re
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
DIAGNOSE_SCRIPT = (
    REPO_ROOT
    / "modules/carla_apollo10.0_bridge/carla_bridge/tools/"
    / "diagnose_lidar_walker_hits.py")

RAW_LIDAR_TOPIC = "/apollo/sensor/velodyne64/compensator/PointCloud2"
OBSTACLES_TOPIC = "/apollo/perception/obstacles"
LIDAR_DETECTION_TOPIC = "/perception/lidar/detection"
LIDAR_DETECTION_FILTER_TOPIC = "/perception/lidar/detection_filter"

LOG_PATTERNS = [
    "CenterPointDetection BeforeNMS",
    "CenterPointDetection AfterNMS",
    "Roi boundary filter",
    "ObjectFilterBank",
]

OBSTACLE_PEDESTRIAN_PATTERNS = [
    "PEDESTRIAN",
    "type: 3",
    "ST_PEDESTRIAN",
    "sub_type: 10",
]


def timestamp_suffix():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def default_output_dir():
    return "/tmp/apollo_lidar_ped_baseline_{}".format(timestamp_suffix())


def ensure_output_dir(path):
    output_dir = Path(path).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def write_text(path, text):
    path.write_text(text, encoding="utf-8")


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


def capture_config_snippets(output_dir):
    snippets = [
        (
            "config_center_point_param.txt",
            REPO_ROOT / "modules/perception/lidar_detection/data/"
            / "center_point_param.pb.txt",
            20,
            80,
        ),
        (
            "config_filter_bank.txt",
            REPO_ROOT / "modules/perception/lidar_detection_filter/data/"
            / "filter_bank.pb.txt",
            1,
            40,
        ),
        (
            "config_roi_boundary_filter.txt",
            REPO_ROOT / "modules/perception/lidar_detection_filter/data/"
            / "roi_boundary_filter.pb.txt",
            1,
            40,
        ),
        (
            "config_objects_lidar.txt",
            REPO_ROOT / "modules/carla_apollo10.0_bridge/carla_bridge/"
            / "config/objects.json",
            48,
            58,
        ),
    ]
    for filename, source, start, end in snippets:
        output_path = output_dir / filename
        if not source.exists():
            write_text(output_path, "ERROR: missing {}\n".format(source))
            continue
        lines = source.read_text(encoding="utf-8").splitlines()
        selected = lines[start - 1:end]
        body = [
            "# Source: {}".format(source.relative_to(REPO_ROOT)),
            "# Lines: {}-{}".format(start, end),
            "",
        ]
        body.extend(selected)
        body.append("")
        write_text(output_path, "\n".join(body))


def capture_channel_info(output_dir):
    channels = [
        ("channel_info_raw_lidar.txt", RAW_LIDAR_TOPIC),
        ("channel_info_obstacles.txt", OBSTACLES_TOPIC),
        ("channel_info_lidar_detection.txt", LIDAR_DETECTION_TOPIC),
        ("channel_info_lidar_detection_filter.txt", LIDAR_DETECTION_FILTER_TOPIC),
    ]
    for filename, channel in channels:
        run_command_to_file(
            ["cyber_channel", "info", channel],
            output_dir / filename,
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
    run_command_to_file(
        command,
        output_dir / "carla_actor_watch.txt",
        timeout=args.actor_watch_duration + 20.0)


def collect_parallel(args, output_dir):
    raw_csv = output_dir / "lidar_walker_hits_baseline.csv"
    raw_stdout = output_dir / "lidar_walker_hits_stdout.txt"
    obstacles_output = output_dir / "perception_obstacles_baseline.txt"

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
    echo_command = ["cyber_channel", "echo", OBSTACLES_TOPIC]

    raw_process, raw_file = start_command_to_file(raw_command, raw_stdout)
    echo_process, echo_file = start_command_to_file(echo_command, obstacles_output)

    raw_code = wait_process(raw_process, raw_file, args.duration + 30.0)
    # Keep final perception capture aligned to raw capture even if raw exits early.
    if echo_process is not None and echo_process.poll() is None:
        echo_process.terminate()
    echo_code = wait_process(echo_process, echo_file, 10.0)
    return raw_code, echo_code


def grep_logs(log_dir, output_dir):
    output_path = output_dir / "perception_log_key_lines.txt"
    log_path = Path(log_dir).expanduser()
    header = "# log_dir: {}\n# patterns: {}\n\n".format(
        log_path,
        ", ".join(LOG_PATTERNS))
    if not log_path.exists():
        write_text(output_path, header + "ERROR: log dir does not exist\n")
        return

    pattern = re.compile("|".join(re.escape(item) for item in LOG_PATTERNS))
    matched = 0
    with output_path.open("w", encoding="utf-8") as fout:
        fout.write(header)
        for path in sorted(log_path.rglob("*")):
            if not path.is_file():
                continue
            try:
                with path.open("r", encoding="utf-8", errors="replace") as fin:
                    for line_no, line in enumerate(fin, 1):
                        if pattern.search(line):
                            fout.write("{}:{}:{}".format(path, line_no, line))
                            matched += 1
            except OSError as exc:
                fout.write("{}: ERROR: {}\n".format(path, exc))
        fout.write("\nmatched_lines: {}\n".format(matched))


def read_csv_rows(path):
    if not path.exists():
        return []
    rows = []
    with path.open(newline="", encoding="utf-8") as fin:
        reader = csv.DictReader(fin)
        for row in reader:
            try:
                row["distance_xy"] = float(row["distance_xy"])
                row["points_in_bbox"] = int(row["points_in_bbox"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(row)
    return rows


def summarize_raw_hits(rows):
    lines = []
    total = len(rows)
    hit_rows = sum(1 for row in rows if row["points_in_bbox"] > 0)
    ge3_rows = sum(1 for row in rows if row["points_in_bbox"] >= 3)
    max_points = max((row["points_in_bbox"] for row in rows), default=0)
    lines.append("Raw walker bbox point stats")
    lines.append("  rows: {}".format(total))
    lines.append("  points_in_bbox > 0: {} ({:.2f})".format(
        hit_rows,
        hit_rows / total if total else 0.0))
    lines.append("  points_in_bbox >= 3: {} ({:.2f})".format(
        ge3_rows,
        ge3_rows / total if total else 0.0))
    lines.append("  max_points_in_bbox: {}".format(max_points))
    lines.append("")
    lines.append("Distance buckets")
    for low in range(0, 50, 5):
        bucket = [
            row for row in rows
            if low <= row["distance_xy"] < low + 5
        ]
        if not bucket:
            lines.append("  {:02d}-{:02d}m n=0".format(low, low + 5))
            continue
        count = len(bucket)
        ge1 = sum(row["points_in_bbox"] >= 1 for row in bucket)
        ge3 = sum(row["points_in_bbox"] >= 3 for row in bucket)
        avg = sum(row["points_in_bbox"] for row in bucket) / count
        max_bucket = max(row["points_in_bbox"] for row in bucket)
        lines.append(
            "  {:02d}-{:02d}m n={:3d} >=1={:3d}({:.2f}) "
            ">=3={:3d}({:.2f}) avg={:.1f} max={}".format(
                low,
                low + 5,
                count,
                ge1,
                ge1 / count,
                ge3,
                ge3 / count,
                avg,
                max_bucket))
    return lines


def summarize_obstacles(output_dir):
    path = output_dir / "perception_obstacles_baseline.txt"
    text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    lines = ["", "Final /apollo/perception/obstacles pedestrian markers"]
    for pattern in OBSTACLE_PEDESTRIAN_PATTERNS:
        present = pattern in text
        lines.append("  {}: {}".format(pattern, "YES" if present else "NO"))
    return lines


def summarize_logs(output_dir):
    path = output_dir / "perception_log_key_lines.txt"
    text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    lines = ["", "Perception log markers"]
    for pattern in LOG_PATTERNS:
        present = pattern in text
        count = text.count(pattern)
        lines.append("  {}: {} lines".format(pattern, count if present else 0))
    return lines


def summarize_channel_info(output_dir):
    mapping = [
        ("raw_lidar", output_dir / "channel_info_raw_lidar.txt"),
        ("obstacles", output_dir / "channel_info_obstacles.txt"),
        ("lidar_detection", output_dir / "channel_info_lidar_detection.txt"),
        (
            "lidar_detection_filter",
            output_dir / "channel_info_lidar_detection_filter.txt",
        ),
    ]
    lines = ["", "Cyber channel info files"]
    for name, path in mapping:
        if not path.exists():
            lines.append("  {}: missing".format(name))
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if text.startswith("SKIPPED:"):
            lines.append("  {}: {} (skipped)".format(name, path.name))
            continue
        error = "ERROR:" in text or "exit_code: 127" in text
        lines.append("  {}: {} ({})".format(
            name,
            path.name,
            "check manually" if error else "captured"))
    return lines


def write_summary(output_dir, raw_code, echo_code, args):
    rows = read_csv_rows(output_dir / "lidar_walker_hits_baseline.csv")
    lines = [
        "Apollo-CARLA lidar pedestrian baseline summary",
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
        "  log_dir: {}".format(args.log_dir),
        "",
        "Command status",
        "  raw walker diagnostic exit_code: {}".format(raw_code),
        "  perception obstacles echo exit_code: {}".format(echo_code),
        "",
    ]
    lines.extend(summarize_raw_hits(rows))
    lines.extend(summarize_obstacles(output_dir))
    lines.extend(summarize_logs(output_dir))
    lines.extend(summarize_channel_info(output_dir))
    lines.extend([
        "",
        "Interpretation guide",
        "  raw >=3 points but no pedestrian markers: check CenterPoint thresholds/downsample.",
        "  CenterPoint log objects drop after ROI/ObjectFilterBank: check detection_filter.",
        "  final obstacles has pedestrian markers but Dreamview does not: check visualization.",
        "  raw points stay zero: check CARLA lidar hit, occlusion, or bridge before perception.",
        "",
        "Generated files",
    ])
    for path in sorted(output_dir.iterdir()):
        if path.is_file():
            lines.append("  {}".format(path.name))
    write_text(output_dir / "summary.txt", "\n".join(lines) + "\n")


def copy_if_requested(output_dir, archive_path):
    if not archive_path:
        return None
    archive_path = Path(archive_path).expanduser().resolve()
    base_name = str(archive_path)
    if base_name.endswith(".zip"):
        base_name = base_name[:-4]
    archive = shutil.make_archive(base_name, "zip", output_dir)
    return archive


def write_skipped_live_outputs(output_dir):
    skipped_files = [
        "channel_info_raw_lidar.txt",
        "channel_info_obstacles.txt",
        "channel_info_lidar_detection.txt",
        "channel_info_lidar_detection_filter.txt",
        "carla_actor_watch.txt",
        "lidar_walker_hits_stdout.txt",
        "perception_obstacles_baseline.txt",
        "perception_log_key_lines.txt",
    ]
    for filename in skipped_files:
        write_text(
            output_dir / filename,
            "SKIPPED: --skip-live-capture was set.\n")
    write_text(output_dir / "lidar_walker_hits_baseline.csv", "")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--duration", default=60.0, type=float)
    parser.add_argument("--max-distance", default=50.0, type=float)
    parser.add_argument("--bbox-margin", default=0.3, type=float)
    parser.add_argument("--sample-period", default=0.5, type=float)
    parser.add_argument("--walker-filter", default="walker.pedestrian.*")
    parser.add_argument("--actor-wait-timeout", default=30.0, type=float)
    parser.add_argument("--actor-watch-duration", default=10.0, type=float)
    parser.add_argument("--actor-watch-sample-period", default=1.0, type=float)
    parser.add_argument("--log-dir", default="/apollo_workspace/data/log")
    parser.add_argument("--output-dir", default=default_output_dir())
    parser.add_argument(
        "--archive",
        default=None,
        help="Optional zip archive path for the collected output directory.")
    parser.add_argument(
        "--skip-live-capture",
        action="store_true",
        help="Only write config snippets and a summary; skip CARLA/Cyber/log capture.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    output_dir = ensure_output_dir(args.output_dir)

    print("collecting baseline outputs into {}".format(output_dir))
    capture_config_snippets(output_dir)
    if args.skip_live_capture:
        write_skipped_live_outputs(output_dir)
        raw_code = 0
        echo_code = 0
    else:
        capture_channel_info(output_dir)
        capture_actor_watch(args, output_dir)
        raw_code, echo_code = collect_parallel(args, output_dir)
        grep_logs(args.log_dir, output_dir)
    write_summary(output_dir, raw_code, echo_code, args)
    archive = copy_if_requested(output_dir, args.archive)

    print("summary written to {}".format(output_dir / "summary.txt"))
    if archive:
        print("archive written to {}".format(archive))
    return 0 if raw_code == 0 else raw_code


if __name__ == "__main__":
    sys.exit(main())
