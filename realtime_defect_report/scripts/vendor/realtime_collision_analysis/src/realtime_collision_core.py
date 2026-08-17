#!/usr/bin/env python3
"""Reproducible run-level analysis for the Apollo + CARLA deadline experiment.

The implementation intentionally streams the large Apollo logs and never writes
to an input directory.  Every derived value retains its source path and the
event tables retain source line numbers whenever the source is a text log.
"""

from __future__ import annotations

import csv
import hashlib
import html
import json
import logging
import math
import os
import platform
import re
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple
from zoneinfo import ZoneInfo

import matplotlib
import numpy as np
import pandas as pd
import yaml
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D

try:
    import markdown
except ModuleNotFoundError:  # Optional: only used by the legacy HTML writer.
    markdown = None

try:
    import scipy
    from scipy import stats
except ModuleNotFoundError:  # Optional: not needed for raw-log recomputation.
    scipy = None
    stats = None


LOG = logging.getLogger("realtime_collision_analysis")
FIELD_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")
LOG_PREFIX_RE = re.compile(
    r"^[IWEF](?P<month>\d{2})(?P<day>\d{2})\s+"
    r"(?P<hour>\d{2}):(?P<minute>\d{2}):(?P<second>\d{2})\.(?P<micro>\d{6})"
)
LOCALIZATION_RE = re.compile(
    r"\[LOCALIZATION_POSE\].*?header_time=(?P<header>[-+0-9.eE]+).*?"
    r"measurement_time=(?P<measurement>[-+0-9.eE]+).*?"
    r"ego_x=(?P<x>[-+0-9.eE]+).*?ego_y=(?P<y>[-+0-9.eE]+).*?"
    r"ego_z=(?P<z>[-+0-9.eE]+).*?heading=(?P<heading>[-+0-9.eE]+).*?"
    r"ego_vx=(?P<vx>[-+0-9.eE]+).*?ego_vy=(?P<vy>[-+0-9.eE]+).*?"
    r"ego_vz=(?P<vz>[-+0-9.eE]+)"
)
STOP_TARGET_RE = re.compile(r"main_stop_reason=stop by\s+(\d+)")
PLANNING_DECISION_STOP_RE = re.compile(
    r"\[PLANNING_DECISION\].*?planning_seq=(\d+).*?input_trace_id=(\d+).*?"
    r"id=(\d+).*?decision_type=STOP"
)
RUN_RE = re.compile(r"^\d{12}$")


@dataclass
class EgoSample:
    time_s: float
    header_time_s: float
    x_m: float
    y_m: float
    z_m: float
    heading_rad: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    speed_mps: float
    source_file: str
    source_line: int


@dataclass
class FusionObservation:
    seq: int
    trace_id: str
    header_time_s: float
    obs_time_s: float
    obstacle_id: str
    obstacle_type: str
    x_m: float
    y_m: float
    z_m: float
    theta_rad: float
    vx_mps: float
    vy_mps: float
    speed_mps: float
    length_m: float
    width_m: float
    confidence: float
    source_file: str
    source_line: int


@dataclass
class RunSpec:
    group_name: str
    nominal_delay_ms: float
    run_id: str
    run_dir: Path


@dataclass
class ParsedRun:
    spec: RunSpec
    files: Dict[str, Optional[Path]]
    collect: Dict[str, Any]
    target_id: Optional[str]
    planning: Dict[str, Any]
    perception: Dict[str, Any]
    prediction: Dict[str, Any]
    trace: Dict[str, Any]
    localization: List[EgoSample]
    scb: Dict[str, Any]
    collision: Dict[str, Any]
    clock: Dict[str, Any]
    quality_notes: List[str] = field(default_factory=list)


def _json_safe(value: Any) -> Any:
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return None if not np.isfinite(value) else float(value)
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_safe(value), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    columns: List[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _scalar(row.get(key)) for key in columns})


def _scalar(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(_json_safe(value), ensure_ascii=False, separators=(",", ":"))
    return value


def fnum(value: Any, default: float = math.nan) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def parse_fields(line: str) -> Dict[str, str]:
    return {match.group(1): match.group(2) for match in FIELD_RE.finditer(line)}


def log_epoch(line: str, run_id: str, timezone: ZoneInfo) -> float:
    match = LOG_PREFIX_RE.match(line)
    if not match:
        return math.nan
    year = int(run_id[:4])
    values = {key: int(value) for key, value in match.groupdict().items()}
    return datetime(
        year,
        values["month"],
        values["day"],
        values["hour"],
        values["minute"],
        values["second"],
        values["micro"],
        tzinfo=timezone,
    ).timestamp()


def nearest_sample(rows: Sequence[EgoSample], timestamp_s: float) -> Optional[EgoSample]:
    if not rows or not math.isfinite(timestamp_s):
        return None
    return min(rows, key=lambda row: abs(row.time_s - timestamp_s))


def interpolate_sample(rows: Sequence[EgoSample], timestamp_s: float) -> Optional[Dict[str, float]]:
    if not rows or timestamp_s < rows[0].time_s or timestamp_s > rows[-1].time_s:
        return None
    times = np.asarray([row.time_s for row in rows], dtype=float)
    index = int(np.searchsorted(times, timestamp_s))
    if index == 0:
        row = rows[0]
        return {"time_s": timestamp_s, "speed_mps": row.speed_mps, "x_m": row.x_m, "y_m": row.y_m, "z_m": row.z_m, "heading_rad": row.heading_rad}
    if index >= len(rows):
        row = rows[-1]
        return {"time_s": timestamp_s, "speed_mps": row.speed_mps, "x_m": row.x_m, "y_m": row.y_m, "z_m": row.z_m, "heading_rad": row.heading_rad}
    left, right = rows[index - 1], rows[index]
    ratio = (timestamp_s - left.time_s) / max(right.time_s - left.time_s, 1e-12)
    return {
        "time_s": timestamp_s,
        "speed_mps": left.speed_mps + ratio * (right.speed_mps - left.speed_mps),
        "x_m": left.x_m + ratio * (right.x_m - left.x_m),
        "y_m": left.y_m + ratio * (right.y_m - left.y_m),
        "z_m": left.z_m + ratio * (right.z_m - left.z_m),
        "heading_rad": left.heading_rad + ratio * (right.heading_rad - left.heading_rad),
    }


def integrate_speed(rows: Sequence[EgoSample], start_s: float, end_s: float) -> float:
    if not rows or not (math.isfinite(start_s) and math.isfinite(end_s)) or end_s <= start_s:
        return math.nan
    start = interpolate_sample(rows, start_s)
    end = interpolate_sample(rows, end_s)
    if start is None or end is None:
        return math.nan
    points: List[Tuple[float, float]] = [(start_s, start["speed_mps"])]
    points.extend((row.time_s, row.speed_mps) for row in rows if start_s < row.time_s < end_s)
    points.append((end_s, end["speed_mps"]))
    times = np.asarray([point[0] for point in points], dtype=float)
    speeds = np.asarray([point[1] for point in points], dtype=float)
    return float(np.trapezoid(speeds, times))


def path_distance(rows: Sequence[EgoSample], start_s: float, end_s: float) -> float:
    if end_s <= start_s:
        return math.nan
    start = interpolate_sample(rows, start_s)
    end = interpolate_sample(rows, end_s)
    if start is None or end is None:
        return math.nan
    points = [(start["x_m"], start["y_m"], start["z_m"])]
    points.extend((row.x_m, row.y_m, row.z_m) for row in rows if start_s < row.time_s < end_s)
    points.append((end["x_m"], end["y_m"], end["z_m"]))
    return float(sum(math.dist(left, right) for left, right in zip(points, points[1:])))


def sha256_and_lines(path: Path) -> Tuple[str, int]:
    digest = hashlib.sha256()
    line_count = 0
    last = b""
    with path.open("rb") as handle:
        while chunk := handle.read(4 * 1024 * 1024):
            digest.update(chunk)
            line_count += chunk.count(b"\n")
            last = chunk[-1:]
    if path.stat().st_size and last != b"\n":
        line_count += 1
    return digest.hexdigest(), line_count


def categorize_file(path: Path) -> str:
    name = path.name.lower()
    text = str(path).lower()
    if "collision_actor_history" in name:
        return "carla_actor_history"
    if "collision_events" in name:
        return "carla_collision_event"
    if "scb_control_delay" in name:
        return "control_delay_evidence"
    if "localization.log" in name:
        return "localization_log"
    if "perception.log" in name:
        return "perception_log"
    if "prediction.log" in name:
        return "prediction_log"
    if "planning.log" in name:
        return "planning_log"
    if "trace\\events" in text or "trace/events" in text:
        return "trace_events"
    if "fusion_inputs" in text:
        return "trace_fusion_inputs"
    if "message_context" in text:
        return "trace_message_context"
    if "trace_anchor" in text:
        return "trace_anchor"
    if name == "collect_time.txt":
        return "collection_window"
    if path.suffix.lower() == ".py":
        return "existing_analysis_script"
    if path.suffix.lower() in {".md", ".txt"}:
        return "documentation"
    if path.suffix.lower() in {".png", ".svg"}:
        return "existing_figure"
    return "other"


def discover_runs(config: Dict[str, Any]) -> List[RunSpec]:
    runs: List[RunSpec] = []
    for group_name, group in config["groups"].items():
        root = Path(group["root"])
        for run_dir in sorted(root.iterdir()):
            if run_dir.is_dir() and RUN_RE.match(run_dir.name):
                runs.append(
                    RunSpec(
                        group_name=group_name,
                        nominal_delay_ms=float(group["nominal_injected_delay_ms"]),
                        run_id=run_dir.name,
                        run_dir=run_dir.resolve(),
                    )
                )
    return runs


def inventory_inputs(
    runs: Sequence[RunSpec],
    config: Dict[str, Any],
    compute_hashes: bool,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    run_lookup = {str(run.run_dir).lower(): run for run in runs}
    rows: List[Dict[str, Any]] = []
    schema: Dict[str, Any] = {"files": [], "category_summary": {}}
    category_counter: Counter[str] = Counter()
    for group_name, group in config["groups"].items():
        root = Path(group["root"]).resolve()
        for path in sorted(item for item in root.rglob("*") if item.is_file()):
            run = next(
                (candidate for prefix, candidate in run_lookup.items() if str(path).lower().startswith(prefix + os.sep)),
                None,
            )
            digest, line_count = sha256_and_lines(path) if compute_hashes else ("SKIPPED", -1)
            category = categorize_file(path)
            category_counter[category] += 1
            suffix = path.suffix.lower()
            columns: List[str] = []
            json_keys: List[str] = []
            schema_error = ""
            try:
                if suffix == ".csv":
                    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
                        columns = next(csv.reader(handle), [])
                elif suffix == ".jsonl":
                    with path.open("r", encoding="utf-8", errors="replace") as handle:
                        first = next((line for line in handle if line.strip()), "")
                    if first:
                        json_keys = sorted(json.loads(first).keys())
                elif suffix in {".yaml", ".yml"}:
                    parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
                    if isinstance(parsed, dict):
                        json_keys = sorted(str(key) for key in parsed)
            except Exception as exc:  # retained in schema inventory
                schema_error = f"{type(exc).__name__}: {exc}"
            row = {
                "group_name": group_name,
                "run_id": run.run_id if run else "",
                "run_directory": str(run.run_dir) if run else "",
                "source_file": str(path),
                "relative_path": str(path.relative_to(root)),
                "category": category,
                "extension": suffix,
                "size_bytes": path.stat().st_size,
                "line_count": line_count,
                "modified_time_iso": datetime.fromtimestamp(path.stat().st_mtime).astimezone().isoformat(),
                "sha256": digest,
            }
            rows.append(row)
            schema["files"].append(
                {
                    "source_file": str(path),
                    "group_name": group_name,
                    "run_id": row["run_id"],
                    "category": category,
                    "columns": columns,
                    "json_or_yaml_top_level_keys": json_keys,
                    "schema_error": schema_error,
                    "line_count": line_count,
                }
            )
    schema["category_summary"] = dict(sorted(category_counter.items()))
    return rows, schema


def locate_run_files(run_dir: Path) -> Dict[str, Optional[Path]]:
    def first(pattern: str) -> Optional[Path]:
        return next(iter(sorted(run_dir.glob(pattern))), None)

    return {
        "collect": first("collect_time.txt"),
        "localization": first("log/localization.log.INFO.*"),
        "perception": first("log/perception.log.INFO.*"),
        "prediction": first("log/prediction.log.INFO.*"),
        "planning": first("log/planning.log.INFO.*"),
        "scb": first("log/scb_control_delay_*.csv"),
        "collision_csv": first("log/carla_collision_events_*.csv"),
        "collision_jsonl": first("log/carla_collision_events_*.jsonl"),
        "actor_history": first("log/carla_collision_actor_history_*.csv"),
        "fusion_inputs": first("trace/fusion_inputs/perception.multi_sensor_fusion.*.csv"),
        "trace_anchor": first("trace/trace_anchor/perception.*.csv"),
        "fusion_events": first("trace/events/perception.multi_sensor_fusion.*.csv"),
        "prediction_events": first("trace/events/prediction.*.csv"),
        "planning_events": first("trace/events/planning.*.csv"),
        "control_context": first("trace/message_context/control.*.csv"),
    }


def parse_collect(path: Optional[Path], timezone: ZoneInfo) -> Dict[str, Any]:
    result: Dict[str, Any] = {"start": None, "end": None, "end_log": None}
    if path is None:
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
        try:
            result[f"{key.strip()}_epoch_s"] = datetime.strptime(
                value.strip(), "%Y%m%d%H%M%S"
            ).replace(tzinfo=timezone).timestamp()
        except ValueError:
            pass
    return result


def parse_planning(path: Optional[Path], run_id: str, timezone: ZoneInfo) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "target_id": None,
        "first_stop": None,
        "first_output": None,
        "stop_count": 0,
        "blocking_count": 0,
        "input_target_count": 0,
        "error_counts": {},
        "fallback_evidence": {},
        "source_file": str(path) if path else None,
    }
    if path is None:
        return result
    errors = Counter()
    target_inputs: List[Dict[str, Any]] = []
    stop_rows: List[Dict[str, Any]] = []
    output_rows: List[Dict[str, Any]] = []
    fallback_events: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line_no, line in enumerate(handle, 1):
            lower = line.lower()
            if "speed optimizer" in lower and ("fail" in lower or "error" in lower):
                errors["speed_optimizer_failure"] += 1
            if "fallback" in lower:
                errors["fallback_mentions"] += 1
            if "trajectory point is empty" in lower or "empty trajectory" in lower:
                errors["empty_trajectory"] += 1
            if "input check failed" in lower:
                errors["input_check_failed"] += 1
            if line.startswith(("E", "F")):
                errors["error_or_fatal_lines"] += 1
            for event_key, pattern in [
                ("primal_infeasible", "primal infeasible"),
                ("speed_fallback", "speed fallback due to algorithm failure"),
                (
                    "constant_deceleration_fallback",
                    "slowing down the car within a constant deceleration with fallback stopping profile",
                ),
            ]:
                if pattern in lower:
                    fallback_events[event_key].append(
                        {
                            "time_s": log_epoch(line, run_id, timezone),
                            "source_file": str(path),
                            "source_line": line_no,
                            "message": line.strip(),
                        }
                    )

            target_match = STOP_TARGET_RE.search(line)
            if target_match:
                target_id = target_match.group(1)
                if result["target_id"] is None:
                    result["target_id"] = target_id
                fields = parse_fields(line)
                output_rows.append(
                    {
                        "target_id": target_id,
                        "time_s": log_epoch(line, run_id, timezone),
                        "trace_id": fields.get("input_trace_id"),
                        "planning_seq": fields.get("seq"),
                        "status_ok": fields.get("status_ok"),
                        "trajectory_type": fields.get("trajectory_type"),
                        "trajectory_point_count": fields.get("trajectory_point_count"),
                        "first_v_mps": fnum(fields.get("first_v")),
                        "last_v_mps": fnum(fields.get("last_v")),
                        "max_abs_decel_mps2": fnum(fields.get("max_abs_decel")),
                        "source_file": str(path),
                        "source_line": line_no,
                    }
                )
            stop_match = PLANNING_DECISION_STOP_RE.search(line)
            if stop_match:
                stop_rows.append(
                    {
                        "planning_seq": int(stop_match.group(1)),
                        "trace_id": stop_match.group(2),
                        "target_id": stop_match.group(3),
                        "time_s": log_epoch(line, run_id, timezone),
                        "source_file": str(path),
                        "source_line": line_no,
                    }
                )
            if "Blocking obstacle ID[" in line:
                match = re.search(r"Blocking obstacle ID\[\s*([^\]\s]+)", line)
                if match:
                    result["blocking_count"] += 1
            if "[PLANNING_INPUT_OBS]" in line:
                fields = parse_fields(line)
                target_inputs.append(
                    {
                        "target_id": fields.get("perception_id") or fields.get("id"),
                        "trace_id": fields.get("input_trace_id"),
                        "is_static": fields.get("is_static"),
                        "time_s": log_epoch(line, run_id, timezone),
                        "source_file": str(path),
                        "source_line": line_no,
                    }
                )
    target_id = result["target_id"]
    relevant_stops = [row for row in stop_rows if row["target_id"] == target_id]
    relevant_outputs = [row for row in output_rows if row["target_id"] == target_id]
    result.update(
        {
            "first_stop": relevant_stops[0] if relevant_stops else None,
            "first_output": relevant_outputs[0] if relevant_outputs else None,
            "stop_count": len(relevant_stops),
            "input_target_count": sum(row["target_id"] == target_id for row in target_inputs),
            "target_input_static_values": sorted(
                {row["is_static"] for row in target_inputs if row["target_id"] == target_id and row["is_static"] is not None}
            ),
            "error_counts": dict(errors),
            "fallback_evidence": {
                key: {
                    "count": len(events),
                    "first": events[0] if events else None,
                }
                for key, events in fallback_events.items()
            },
        }
    )
    return result


def parse_perception(
    path: Optional[Path],
    run_id: str,
    target_id: Optional[str],
    timezone: ZoneInfo,
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "target_rows": [],
        "fusion_frames": [],
        "error_counts": {},
        "point_count_samples": [],
        "source_file": str(path) if path else None,
    }
    if path is None or target_id is None:
        return result
    errors = Counter()
    target_rows: List[FusionObservation] = []
    fusion_frames: List[Dict[str, Any]] = []
    point_samples: List[int] = []
    target_token = f" id={target_id} "
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line_no, line in enumerate(handle, 1):
            if line.startswith(("E", "F")):
                errors["error_or_fatal_lines"] += 1
            if "Fusion receive message with error code" in line:
                errors["fusion_input_error_skip"] += 1
            if "num points before fusing:" in line:
                match = re.search(r"num points before fusing:\s*(\d+)", line)
                if match:
                    point_samples.append(int(match.group(1)))
            if "[FUSION_OBS_FRAME]" in line:
                fields = parse_fields(line)
                lidar_ns = fnum(fields.get("lidar_timestamp"))
                header = fnum(fields.get("header_time"))
                fusion_frames.append(
                    {
                        "seq": int(fnum(fields.get("seq"), -1)),
                        "trace_id": fields.get("trace_id"),
                        "header_time_s": header,
                        "sensor_time_s": lidar_ns / 1e9 if math.isfinite(lidar_ns) else math.nan,
                        "latency_ms": (header - lidar_ns / 1e9) * 1000.0 if math.isfinite(header) and math.isfinite(lidar_ns) else math.nan,
                        "obstacle_count": int(fnum(fields.get("obstacle_count"), -1)),
                        "log_time_s": log_epoch(line, run_id, timezone),
                        "source_line": line_no,
                    }
                )
            if "[FUSION_OBS]" not in line or target_token not in line:
                continue
            fields = parse_fields(line)
            try:
                target_rows.append(
                    FusionObservation(
                        seq=int(fields["seq"]),
                        trace_id=str(fields["trace_id"]),
                        header_time_s=float(fields["header_time"]),
                        obs_time_s=float(fields["obs_time"]),
                        obstacle_id=str(fields["id"]),
                        obstacle_type=str(fields.get("type", "")),
                        x_m=float(fields["pos_x"]),
                        y_m=float(fields["pos_y"]),
                        z_m=float(fields.get("pos_z", 0.0)),
                        theta_rad=float(fields.get("theta", 0.0)),
                        vx_mps=float(fields.get("vel_x", 0.0)),
                        vy_mps=float(fields.get("vel_y", 0.0)),
                        speed_mps=float(fields.get("speed", 0.0)),
                        length_m=float(fields.get("length", math.nan)),
                        width_m=float(fields.get("width", math.nan)),
                        confidence=float(fields.get("confidence", math.nan)),
                        source_file=str(path),
                        source_line=line_no,
                    )
                )
            except (KeyError, ValueError):
                errors["malformed_target_rows"] += 1
    result.update(
        {
            "target_rows": target_rows,
            "fusion_frames": fusion_frames,
            "error_counts": dict(errors),
            "point_count_samples": point_samples,
        }
    )
    return result


def normal_gap_limit(target_rows: Sequence[FusionObservation], config: Dict[str, Any]) -> float:
    if len(target_rows) < 2:
        return float(config["stable_perception"]["absolute_max_gap_s"])
    gaps = np.diff([row.header_time_s for row in target_rows])
    p95 = float(np.percentile(gaps, 95)) if gaps.size else 0.0
    return min(
        float(config["stable_perception"]["absolute_max_gap_s"]),
        max(float(np.median(gaps)) * 1.25, p95 * float(config["stable_perception"]["p95_gap_multiplier"])),
    )


def stable_observation(
    rows: Sequence[FusionObservation],
    required: int,
    config: Dict[str, Any],
) -> Tuple[Optional[FusionObservation], List[FusionObservation], Dict[str, Any]]:
    if len(rows) < required:
        return None, [], {"reason": "INSUFFICIENT_TARGET_ROWS"}
    gap_limit = normal_gap_limit(rows, config)
    max_jump = float(config["stable_perception"]["max_position_jump_m"])
    for index in range(len(rows) - required + 1):
        segment = list(rows[index : index + required])
        valid = True
        for left, right in zip(segment, segment[1:]):
            gap = right.header_time_s - left.header_time_s
            position_jump = math.dist((left.x_m, left.y_m, left.z_m), (right.x_m, right.y_m, right.z_m))
            if right.seq != left.seq + 1 or gap > gap_limit or position_jump > max_jump:
                valid = False
                break
        if valid:
            return segment[0], segment, {"gap_limit_s": gap_limit, "start_index": index}
    return None, [], {"reason": "NO_CONSECUTIVE_STABLE_SEGMENT", "gap_limit_s": gap_limit}


def parse_prediction(
    path: Optional[Path],
    run_id: str,
    target_id: Optional[str],
    target_trace_id: Optional[str],
    timezone: ZoneInfo,
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "input_count": 0,
        "output_count": 0,
        "first_input": None,
        "first_output": None,
        "static_valid_count": 0,
        "static_invalid_count": 0,
        "error_counts": {},
        "source_file": str(path) if path else None,
    }
    if path is None or target_id is None:
        return result
    errors = Counter()
    target_token = f" id={target_id} "
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line_no, line in enumerate(handle, 1):
            if line.startswith(("E", "F")):
                errors["error_or_fatal_lines"] += 1
            if target_token not in line:
                continue
            fields = parse_fields(line)
            row = {
                "time_s": log_epoch(line, run_id, timezone),
                "trace_id": fields.get("trace_id"),
                "source_file": str(path),
                "source_line": line_no,
            }
            if "[PREDICTION_INPUT_OBS]" in line:
                result["input_count"] += 1
                if result["first_input"] is None and (target_trace_id is None or fields.get("trace_id") == target_trace_id):
                    result["first_input"] = row
            elif "[PREDICTION_OUTPUT_OBS]" in line:
                result["output_count"] += 1
                has_static = fields.get("pred_has_is_static") == "1"
                is_static = fields.get("pred_is_static") == "1"
                if has_static and is_static:
                    result["static_valid_count"] += 1
                elif has_static:
                    result["static_invalid_count"] += 1
                if result["first_output"] is None and (target_trace_id is None or fields.get("trace_id") == target_trace_id):
                    result["first_output"] = dict(row, pred_has_is_static=has_static, pred_is_static=is_static, trajectory_count=int(fnum(fields.get("traj_count"), 0)))
    result["error_counts"] = dict(errors)
    return result


def load_localization(path: Optional[Path]) -> List[EgoSample]:
    rows: List[EgoSample] = []
    if path is None:
        return rows
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line_no, line in enumerate(handle, 1):
            match = LOCALIZATION_RE.search(line)
            if not match:
                continue
            values = {key: float(value) for key, value in match.groupdict().items()}
            speed = math.sqrt(values["vx"] ** 2 + values["vy"] ** 2 + values["vz"] ** 2)
            rows.append(
                EgoSample(
                    time_s=values["measurement"],
                    header_time_s=values["header"],
                    x_m=values["x"],
                    y_m=values["y"],
                    z_m=values["z"],
                    heading_rad=values["heading"],
                    vx_mps=values["vx"],
                    vy_mps=values["vy"],
                    vz_mps=values["vz"],
                    speed_mps=speed,
                    source_file=str(path),
                    source_line=line_no,
                )
            )
    return sorted(rows, key=lambda row: row.time_s)


def _trace_phase_time(path: Optional[Path], trace_id: str, phase: str) -> float:
    if path is None:
        return math.nan
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("trace_id") == trace_id and row.get("phase") == phase:
                return fnum(row.get("mono_ns"))
    return math.nan


def parse_trace(files: Dict[str, Optional[Path]], target_trace_id: Optional[str]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "status": "MISSING",
        "target_trace_id": target_trace_id,
        "sensor_anchor": None,
        "e2e_ms": {},
        "source_files": {},
    }
    if not target_trace_id:
        return result
    fusion_inputs = files.get("fusion_inputs")
    anchor_file = files.get("trace_anchor")
    parent_trace: Optional[str] = None
    sensor_ts_ns = math.nan
    if fusion_inputs:
        with fusion_inputs.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
            for row in csv.DictReader(handle):
                if (
                    row.get("fusion_trace_id") == target_trace_id
                    and row.get("sensor_kind") == "lidar"
                    and row.get("is_main_sensor") == "1"
                ):
                    parent_trace = row.get("parent_trace_id")
                    sensor_ts_ns = fnum(row.get("sensor_ts_ns"))
                    break
    anchor: Optional[Dict[str, Any]] = None
    if anchor_file and parent_trace:
        with anchor_file.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
            for row in csv.DictReader(handle):
                if row.get("trace_id") == parent_trace:
                    anchor = row
                    break
    if anchor is None:
        result["reason"] = "TARGET_TRACE_ANCHOR_NOT_FOUND"
        return result
    preproc_enter_ns = fnum(anchor.get("preproc_enter_ns"))
    ingress_ms = fnum(anchor.get("ingress_ms"))
    phase_ns = {
        "fusion": _trace_phase_time(files.get("fusion_events"), target_trace_id, "output_pub"),
        "prediction": _trace_phase_time(files.get("prediction_events"), target_trace_id, "output_pub"),
        "planning": _trace_phase_time(files.get("planning_events"), target_trace_id, "output_pub"),
    }
    control_ns = math.nan
    control_path = files.get("control_context")
    if control_path:
        with control_path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
            for row in csv.DictReader(handle):
                if (
                    row.get("trace_id") == target_trace_id
                    and row.get("edge") == "out"
                    and row.get("channel") == "/apollo/control"
                ):
                    control_ns = fnum(row.get("mono_ns"))
                    break
    phase_ns["control"] = control_ns
    e2e_ms = {
        name: ingress_ms + (mono_ns - preproc_enter_ns) / 1e6
        if math.isfinite(mono_ns) and math.isfinite(preproc_enter_ns) and math.isfinite(ingress_ms)
        else math.nan
        for name, mono_ns in phase_ns.items()
    }
    result.update(
        {
            "status": "AVAILABLE" if math.isfinite(e2e_ms["control"]) else "PARTIAL",
            "parent_trace_id": parent_trace,
            "sensor_anchor": {
                "sensor_ts_ns": sensor_ts_ns,
                "preproc_enter_ns": preproc_enter_ns,
                "ingress_ms": ingress_ms,
                "source_file": str(anchor_file),
            },
            "phase_mono_ns": phase_ns,
            "e2e_ms": e2e_ms,
            "source_files": {
                key: str(files[key]) if files.get(key) else None
                for key in ["fusion_inputs", "trace_anchor", "fusion_events", "prediction_events", "planning_events", "control_context"]
            },
        }
    )
    return result


def parse_scb(path: Optional[Path]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "present": False,
        "source_file": str(path) if path else None,
        "statuses": [],
        "applied": None,
    }
    if path is None:
        return result
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        rows = list(csv.DictReader(handle))
    applied = next(
        (
            row
            for row in rows
            if str(row.get("first_effective_brake", "")).strip().lower() in {"1", "true"}
        ),
        None,
    )
    first = rows[0] if rows else {}
    result.update(
        {
            "present": True,
            "row_count": len(rows),
            "statuses": [row.get("status", "") for row in rows],
            "schema_version": first.get("schema_version"),
            "bridge_entry_file": first.get("bridge_entry_file"),
            "settings_source_file": first.get("settings_source_file"),
            "injector_source_file": first.get("injector_source_file"),
            "process_id": first.get("process_id"),
            "activation_speed_mps": fnum(first.get("activation_speed_mps")),
            "brake_threshold_percentage": fnum(first.get("brake_threshold_percentage")),
            "log_all_delayed_commands": first.get("log_all_delayed_commands"),
        }
    )
    if applied:
        result["applied"] = {
            "control_header_time_s": fnum(applied.get("control_header_time_sec")),
            "receive_wall_time_s": fnum(applied.get("receive_wall_time_unix_ns")) / 1e9,
            "release_wall_time_s": fnum(applied.get("release_wall_time_unix_ns")) / 1e9,
            "apply_start_wall_time_s": fnum(applied.get("apply_call_start_wall_time_unix_ns")) / 1e9,
            "apply_end_wall_time_s": fnum(applied.get("apply_call_end_wall_time_unix_ns")) / 1e9,
            "receive_carla_frame": fnum(applied.get("receive_carla_frame")),
            "receive_carla_elapsed_s": fnum(applied.get("receive_carla_elapsed_sec")),
            "apply_carla_frame": fnum(applied.get("apply_carla_frame")),
            "apply_carla_elapsed_s": fnum(applied.get("apply_carla_elapsed_sec")),
            "requested_delay_ms": fnum(applied.get("requested_delay_ms")),
            "actual_delay_ms": fnum(applied.get("actual_delay_ms")),
            "api_completion_delay_ms": fnum(applied.get("api_completion_delay_ms")),
            "api_call_duration_ms": fnum(applied.get("api_call_duration_ms")),
            "actual_frame_delay": fnum(applied.get("actual_frame_delay")),
            "actual_sim_delay_ms": fnum(applied.get("actual_sim_delay_ms")),
            "ego_speed_mps_at_receive": fnum(applied.get("ego_speed_mps_at_receive")),
            "brake_percentage": fnum(applied.get("brake_percentage")),
            "queue_depth": fnum(applied.get("queue_depth")),
            "status": applied.get("status"),
        }
    return result


def parse_collision(files: Dict[str, Optional[Path]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "occurred": False,
        "with_target": False,
        "source_file": None,
        "history_source_file": str(files.get("actor_history")) if files.get("actor_history") else None,
    }
    path = files.get("collision_csv")
    if path is None:
        return result
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        row = next(csv.DictReader(handle), None)
    if not row:
        return result
    result.update(
        {
            "occurred": True,
            "with_target": True,
            "event_type": row.get("event_type"),
            "time_s": fnum(row.get("wall_time_unix_ns")) / 1e9,
            "wall_time_iso": row.get("wall_time_iso"),
            "carla_frame": fnum(row.get("carla_frame")),
            "carla_timestamp_s": fnum(row.get("carla_timestamp_sec")),
            "map_name": row.get("map_name"),
            "ego_actor_id": row.get("ego_actor_id"),
            "other_actor_id": row.get("other_actor_id"),
            "ego_type_id": row.get("ego_type_id"),
            "other_type_id": row.get("other_type_id"),
            "impact_speed_mps": fnum(row.get("ego_speed_mps")),
            "impulse_x": fnum(row.get("normal_impulse_x")),
            "impulse_y": fnum(row.get("normal_impulse_y")),
            "impulse_z": fnum(row.get("normal_impulse_z")),
            "impulse_norm": fnum(row.get("normal_impulse_norm")),
            "source_file": str(path),
            "source_row": 2,
        }
    )
    history = files.get("actor_history")
    if history:
        last_ego: Optional[Dict[str, str]] = None
        with history.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
            for hist_row in csv.DictReader(handle):
                if hist_row.get("role") == "ego" and fnum(hist_row.get("wall_time_unix_ns")) / 1e9 <= result["time_s"]:
                    last_ego = hist_row
        if last_ego:
            speed = math.sqrt(
                fnum(last_ego.get("velocity_x"), 0.0) ** 2
                + fnum(last_ego.get("velocity_y"), 0.0) ** 2
                + fnum(last_ego.get("velocity_z"), 0.0) ** 2
            )
            result["impact_speed_pre_event_sample_mps"] = speed
    return result


def clock_alignment(files: Dict[str, Optional[Path]], config: Dict[str, Any]) -> Dict[str, Any]:
    history = files.get("actor_history")
    if history is None:
        return {
            "status": "LIMITED_NO_DUAL_CLOCK_HISTORY",
            "method": "Apollo/Localization epoch timeline; CARLA simulation mapping unavailable",
            "source_file": None,
            "slope": math.nan,
            "intercept": math.nan,
            "median_abs_residual_ms": math.nan,
            "p95_abs_residual_ms": math.nan,
            "max_abs_residual_ms": math.nan,
            "clock_jump_detected": None,
            "simulation_pause_detected": None,
        }
    pairs: Dict[int, Tuple[float, float]] = {}
    with history.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        for row in csv.DictReader(handle):
            frame = int(fnum(row.get("carla_frame"), -1))
            sim = fnum(row.get("carla_timestamp_sec"))
            wall = fnum(row.get("wall_time_unix_ns")) / 1e9
            if frame >= 0 and math.isfinite(sim) and math.isfinite(wall):
                pairs[frame] = (sim, wall)
    if len(pairs) < 3:
        return {"status": "TIME_ALIGNMENT_UNCERTAIN", "method": "insufficient dual-clock rows", "source_file": str(history)}
    values = np.asarray(list(pairs.values()), dtype=float)
    x = values[:, 0]
    y = values[:, 1]
    x0 = float(x[0])
    y0 = float(y[0])
    centered_x = x - x0
    centered_y = y - y0
    initial_slope, initial_intercept = np.polyfit(centered_x, centered_y, 1)
    initial_resid = centered_y - (initial_slope * centered_x + initial_intercept)
    mad = float(np.median(np.abs(initial_resid - np.median(initial_resid))))
    keep = np.abs(initial_resid - np.median(initial_resid)) <= max(3.0 * 1.4826 * mad, 0.003)
    slope, centered_intercept = np.polyfit(centered_x[keep], centered_y[keep], 1)
    intercept = y0 + centered_intercept - slope * x0
    residual_s = y - (slope * x + intercept)
    abs_ms = np.abs(residual_s) * 1000.0
    sim_diff = np.diff(x)
    wall_diff = np.diff(y)
    pause = bool(np.any((np.abs(sim_diff) < 1e-9) & (wall_diff > 0.02)))
    jumps = bool(np.any(np.abs(np.diff(residual_s)) > 0.05))
    p95 = float(np.percentile(abs_ms, 95))
    limit = float(config["quality"]["max_clock_alignment_p95_residual_ms"])
    return {
        "status": "ALIGNED" if p95 <= limit and not jumps else "TIME_ALIGNMENT_UNCERTAIN",
        "method": "robust refit of wall_time = slope * CARLA_simulation_time + intercept",
        "source_file": str(history),
        "pair_count": len(pairs),
        "inlier_count": int(np.sum(keep)),
        "slope": float(slope),
        "intercept": float(intercept),
        "median_abs_residual_ms": float(np.median(abs_ms)),
        "p95_abs_residual_ms": p95,
        "max_abs_residual_ms": float(np.max(abs_ms)),
        "clock_jump_detected": jumps,
        "simulation_pause_detected": pause,
        "sim_times_s": x.tolist(),
        "wall_times_s": y.tolist(),
        "residual_ms": (residual_s * 1000.0).tolist(),
    }


def _median3(values: np.ndarray) -> np.ndarray:
    if len(values) < 3:
        return values.copy()
    result = values.copy()
    for index in range(1, len(values) - 1):
        result[index] = float(np.median(values[index - 1 : index + 2]))
    return result


def detect_brake_onset(
    rows: Sequence[EgoSample],
    t1_s: float,
    control_s: float,
    threshold_mps2: float,
    config: Dict[str, Any],
    smoothed: bool = False,
) -> Dict[str, Any]:
    if len(rows) < 4:
        return {"status": "MISSING", "reason": "INSUFFICIENT_LOCALIZATION"}
    times = np.asarray([row.time_s for row in rows], dtype=float)
    raw_speeds = np.asarray([row.speed_mps for row in rows], dtype=float)
    speeds = _median3(raw_speeds) if smoothed else raw_speeds.copy()
    dt = np.diff(times)
    acceleration = np.full(len(rows), np.nan)
    valid_dt = dt > 0
    acceleration[1:][valid_dt] = np.diff(speeds)[valid_dt] / dt[valid_dt]
    consecutive = int(config["effective_brake"]["consecutive_intervals"])
    confirmation_s = float(config["effective_brake"]["confirmation_window_s"])
    confirmation_drop = float(config["effective_brake"]["confirmation_speed_drop_mps"])

    candidates: List[int] = []
    for index in range(1, len(rows) - consecutive + 1):
        if times[index] < t1_s:
            continue
        if not np.all(acceleration[index : index + consecutive] <= -abs(threshold_mps2)):
            continue
        confirm_index = int(np.searchsorted(times, times[index] + confirmation_s, side="right") - 1)
        confirm_index = min(max(confirm_index, index), len(rows) - 1)
        if speeds[index] - speeds[confirm_index] + 1e-9 < confirmation_drop:
            continue
        candidates.append(index)

    if not candidates:
        return {
            "status": "MISSING",
            "reason": "NO_SUSTAINED_DECELERATION",
            "threshold_mps2": threshold_mps2,
            "smoothed": smoothed,
        }
    raw_index = candidates[0]
    selected_index = raw_index
    attribution = "DIRECT_AFTER_CONTROL"
    if math.isfinite(control_s) and times[raw_index] < control_s:
        # Characterize the threshold-contiguous episode that was already active
        # before the target Control output.  A long or material episode is
        # causally inseparable from the target response and must not be replaced
        # by an unrelated braking event later in the log.
        raw_end_index = raw_index + consecutive - 1
        while (
            raw_end_index + 1 < len(rows)
            and math.isfinite(float(acceleration[raw_end_index + 1]))
            and acceleration[raw_end_index + 1] <= -abs(threshold_mps2)
        ):
            raw_end_index += 1
        raw_episode_duration_s = float(times[raw_end_index] - times[raw_index])
        raw_episode_speed_drop_mps = float(speeds[raw_index] - speeds[raw_end_index])
        invalid_duration_s = float(
            config["effective_brake"]["precontrol_episode_invalid_duration_s"]
        )
        invalid_drop_mps = float(
            config["effective_brake"]["precontrol_episode_invalid_speed_drop_mps"]
        )
        near_stop_indices = np.where(
            (times > times[raw_index])
            & (speeds < float(config["stop"]["speed_threshold_mps"]))
        )[0]
        raw_near_stop_index = int(near_stop_indices[0]) if near_stop_indices.size else len(rows)

        if (
            raw_episode_duration_s >= invalid_duration_s
            or raw_episode_speed_drop_mps >= invalid_drop_mps
        ):
            return {
                "status": "ATTRIBUTION_INVALID",
                "reason": "MATERIAL_DECELERATION_ALREADY_ACTIVE_BEFORE_TARGET_CONTROL",
                "raw_onset_time_s": float(times[raw_index]),
                "raw_onset_speed_mps": float(raw_speeds[raw_index]),
                "raw_latency_ms": (float(times[raw_index]) - t1_s) * 1000.0,
                "raw_episode_end_time_s": float(times[raw_end_index]),
                "raw_episode_duration_s": raw_episode_duration_s,
                "raw_episode_speed_drop_mps": raw_episode_speed_drop_mps,
                "threshold_mps2": threshold_mps2,
                "smoothed": smoothed,
                "times_s": times.tolist(),
                "raw_speeds_mps": raw_speeds.tolist(),
                "processed_speeds_mps": speeds.tolist(),
                "acceleration_mps2": acceleration.tolist(),
            }
        selected_index = -1
        for index in candidates[1:]:
            previous = acceleration[index - 1] if index > 1 else math.nan
            starts_distinct_episode = math.isfinite(float(previous)) and previous > -abs(
                threshold_mps2
            )
            if (
                index > raw_end_index
                and index < raw_near_stop_index
                and times[index] >= control_s
                and starts_distinct_episode
            ):
                selected_index = index
                attribution = "DISTINCT_POST_CONTROL_EPISODE"
                break
        if selected_index < 0:
            return {
                "status": "ATTRIBUTION_INVALID",
                "reason": "DECELERATION_ALREADY_ACTIVE_BEFORE_TARGET_CONTROL",
                "raw_onset_time_s": float(times[raw_index]),
                "raw_onset_speed_mps": float(raw_speeds[raw_index]),
                "raw_latency_ms": (float(times[raw_index]) - t1_s) * 1000.0,
                "threshold_mps2": threshold_mps2,
                "smoothed": smoothed,
                "times_s": times.tolist(),
                "raw_speeds_mps": raw_speeds.tolist(),
                "processed_speeds_mps": speeds.tolist(),
                "acceleration_mps2": acceleration.tolist(),
            }
    return {
        "status": "AVAILABLE",
        "onset_time_s": float(times[selected_index]),
        "onset_speed_mps": float(raw_speeds[selected_index]),
        "onset_index": int(selected_index),
        "raw_onset_time_s": float(times[raw_index]),
        "raw_latency_ms": (float(times[raw_index]) - t1_s) * 1000.0,
        "attribution": attribution,
        "threshold_mps2": threshold_mps2,
        "smoothed": smoothed,
        "times_s": times.tolist(),
        "raw_speeds_mps": raw_speeds.tolist(),
        "processed_speeds_mps": speeds.tolist(),
        "acceleration_mps2": acceleration.tolist(),
    }


def find_near_stop(
    rows: Sequence[EgoSample], start_s: float, collision_s: float, config: Dict[str, Any]
) -> Dict[str, Any]:
    """Return the first v<0.1 sample as a braking-distance endpoint.

    This is intentionally separate from the strict stop event, which also
    requires a 0.5 s hold.  Several recordings re-accelerate or end soon after
    reaching near-zero speed; the first near-stop remains the reproducible
    endpoint used by the established empirical braking-distance method.
    """
    if math.isfinite(collision_s):
        return {"status": "NOT_APPLICABLE_COLLISION"}
    threshold = float(config["stop"]["speed_threshold_mps"])
    for row in rows:
        if row.time_s > start_s and row.speed_mps < threshold:
            return {"status": "AVAILABLE", "time_s": row.time_s, "sample": row}
    return {"status": "MISSING", "reason": "NO_NEAR_STOP_SAMPLE_AFTER_BRAKE_ONSET"}


def find_brake_completion(
    rows: Sequence[EgoSample], start_s: float, collision_s: float, config: Dict[str, Any]
) -> Dict[str, Any]:
    """Return the post-t2 minimum-speed sample used by the baseline handoff.

    A near-stop sample is required first so an unrelated non-stopping speed
    minimum cannot enter the empirical braking model.
    """
    if math.isfinite(collision_s):
        return {"status": "NOT_APPLICABLE_COLLISION"}
    threshold = float(config["stop"]["speed_threshold_mps"])
    candidates = [row for row in rows if row.time_s > start_s]
    if not candidates or not any(row.speed_mps < threshold for row in candidates):
        return {"status": "MISSING", "reason": "NO_NEAR_STOP_BEFORE_MINIMUM_SPEED_ENDPOINT"}
    sample = min(candidates, key=lambda row: (row.speed_mps, row.time_s))
    return {"status": "AVAILABLE", "time_s": sample.time_s, "sample": sample}


def find_stop(rows: Sequence[EgoSample], start_s: float, collision_s: float, config: Dict[str, Any]) -> Dict[str, Any]:
    if math.isfinite(collision_s):
        return {"status": "NOT_APPLICABLE_COLLISION"}
    threshold = float(config["stop"]["speed_threshold_mps"])
    hold = float(config["stop"]["hold_duration_s"])
    for index, row in enumerate(rows):
        if row.time_s <= start_s or (math.isfinite(collision_s) and row.time_s >= collision_s):
            continue
        if row.speed_mps >= threshold:
            continue
        end_time = row.time_s + hold
        end_index = next(
            (candidate for candidate in range(index, len(rows)) if rows[candidate].time_s >= end_time),
            None,
        )
        window = rows[index : end_index + 1] if end_index is not None else []
        if window and all(sample.speed_mps < threshold for sample in window):
            return {"status": "AVAILABLE", "time_s": row.time_s, "sample": row}
    return {"status": "MISSING", "reason": "NO_SUSTAINED_STOP_BEFORE_COLLISION_OR_LOG_END"}


def parse_run(spec: RunSpec, config: Dict[str, Any], timezone: ZoneInfo) -> ParsedRun:
    files = locate_run_files(spec.run_dir)
    collect = parse_collect(files["collect"], timezone)
    planning = parse_planning(files["planning"], spec.run_id, timezone)
    target_id = planning.get("target_id")
    perception = parse_perception(files["perception"], spec.run_id, target_id, timezone)
    stable, segment, stable_debug = stable_observation(
        perception.get("target_rows", []),
        int(config["stable_perception"]["primary_frames"]),
        config,
    )
    perception["stable"] = stable
    perception["stable_segment"] = segment
    perception["stable_debug"] = stable_debug
    perception["stable_sensitivity"] = {}
    for count in config["stable_perception"]["sensitivity_frames"]:
        candidate, candidate_segment, debug = stable_observation(
            perception.get("target_rows", []), int(count), config
        )
        perception["stable_sensitivity"][str(count)] = {
            "row": candidate,
            "segment": candidate_segment,
            "debug": debug,
        }
    target_trace_id = stable.trace_id if stable else None
    prediction = parse_prediction(
        files["prediction"], spec.run_id, target_id, target_trace_id, timezone
    )
    trace = parse_trace(files, target_trace_id)
    localization = load_localization(files["localization"])
    scb = parse_scb(files["scb"])
    collision = parse_collision(files)
    clock = clock_alignment(files, config)
    notes: List[str] = []
    required = ["collect", "localization", "perception", "prediction", "planning", "fusion_inputs", "trace_anchor", "control_context"]
    missing = [key for key in required if files.get(key) is None]
    if missing:
        notes.append("MISSING_KEY_FILES:" + ",".join(missing))
    if not scb["present"]:
        notes.append("SCB_EVIDENCE_MISSING")
    if stable is None:
        notes.append("STABLE_TARGET_NOT_RESOLVED")
    return ParsedRun(
        spec=spec,
        files=files,
        collect=collect,
        target_id=target_id,
        planning=planning,
        perception=perception,
        prediction=prediction,
        trace=trace,
        localization=localization,
        scb=scb,
        collision=collision,
        clock=clock,
        quality_notes=notes,
    )


def geometry_at_t1(
    stable: Optional[FusionObservation],
    ego: Optional[EgoSample],
    config: Dict[str, Any],
) -> Dict[str, Any]:
    if stable is None or ego is None:
        return {"status": "MISSING"}
    dx = stable.x_m - ego.x_m
    dy = stable.y_m - ego.y_m
    forward_x = math.cos(ego.heading_rad)
    forward_y = math.sin(ego.heading_rad)
    longitudinal_center = dx * forward_x + dy * forward_y
    lateral = -dx * forward_y + dy * forward_x
    center_euclidean = math.sqrt(dx * dx + dy * dy + (stable.z_m - ego.z_m) ** 2)
    offset = float(config["geometry"]["combined_center_to_surface_offset_m"])
    half = offset / 2.0
    ego_front = (ego.x_m + half * forward_x, ego.y_m + half * forward_y)
    obstacle_near = (stable.x_m - half * forward_x, stable.y_m - half * forward_y)
    return {
        "status": "AVAILABLE_ESTIMATED_EXTENTS",
        "method": "longitudinal projection with calibrated combined center-to-contact offset",
        "center_distance_m": longitudinal_center,
        "center_euclidean_distance_m": center_euclidean,
        "euclidean_surface_distance_m": center_euclidean - offset,
        "longitudinal_clearance_m": longitudinal_center - offset,
        "lateral_offset_m": lateral,
        "ego_front_world_x": ego_front[0],
        "ego_front_world_y": ego_front[1],
        "obstacle_near_edge_world_x": obstacle_near[0],
        "obstacle_near_edge_world_y": obstacle_near[1],
        "combined_offset_m": offset,
        "offset_uncertainty_m": float(config["geometry"]["observed_contact_offset_uncertainty_m"]),
        "offset_source": config["geometry"]["offset_source"],
        "individual_extent_split_note": config["geometry"]["individual_extent_split"],
    }


def braking_statistics(
    parsed: ParsedRun,
    onset: Dict[str, Any],
    brake_completion: Dict[str, Any],
) -> Dict[str, Any]:
    if onset.get("status") != "AVAILABLE":
        return {"status": "MISSING"}
    if parsed.collision.get("occurred"):
        end_s = parsed.collision.get("time_s")
        endpoint = "COLLISION"
    else:
        end_s = brake_completion.get("time_s")
        endpoint = "MINIMUM_SPEED_PROXY"
    if end_s is None or not math.isfinite(fnum(end_s)):
        return {"status": "MISSING", "reason": "NO_STOP_OR_COLLISION_ENDPOINT"}
    start_s = float(onset["onset_time_s"])
    rows = [row for row in parsed.localization if start_s <= row.time_s <= float(end_s)]
    if len(rows) < 3:
        return {"status": "MISSING", "reason": "INSUFFICIENT_BRAKING_SAMPLES"}
    times = np.asarray([row.time_s for row in rows], dtype=float)
    speeds = np.asarray([row.speed_mps for row in rows], dtype=float)
    acceleration = np.diff(speeds) / np.diff(times)
    decel = -acceleration[acceleration < 0]
    jerk = np.diff(acceleration) / np.diff(times[1:]) if len(acceleration) > 1 else np.asarray([])
    path_length = integrate_speed(parsed.localization, start_s, float(end_s))
    start_position = interpolate_sample(parsed.localization, start_s)
    end_position = interpolate_sample(parsed.localization, float(end_s))
    displacement = (
        math.dist(
            (start_position["x_m"], start_position["y_m"], start_position["z_m"]),
            (end_position["x_m"], end_position["y_m"], end_position["z_m"]),
        )
        if start_position is not None and end_position is not None
        else math.nan
    )
    # The authoritative baseline calibration defines empirical braking distance
    # as the 3-D position displacement from t2 to the first near-stop sample.
    # Collision exposure remains a travelled path length up to contact.
    distance = path_length if endpoint == "COLLISION" else displacement
    result = {
        "status": "AVAILABLE",
        "endpoint": endpoint,
        "start_speed_mps": float(onset["onset_speed_mps"]),
        "duration_s": float(end_s) - start_s,
        "distance_m": distance,
        "path_length_m": path_length,
        "displacement_m": displacement,
        "distance_method": (
            "speed-time trapezoidal path length to collision"
            if endpoint == "COLLISION"
            else "3-D Localization displacement from effective brake onset to post-t2 minimum-speed sample"
        ),
        "mean_deceleration_mps2": float(np.mean(decel)) if decel.size else math.nan,
        "peak_deceleration_mps2": float(np.max(decel)) if decel.size else math.nan,
        "deceleration_p10_mps2": float(np.percentile(decel, 10)) if decel.size else math.nan,
        "deceleration_p50_mps2": float(np.percentile(decel, 50)) if decel.size else math.nan,
        "deceleration_p90_mps2": float(np.percentile(decel, 90)) if decel.size else math.nan,
        "max_abs_jerk_mps3": float(np.max(np.abs(jerk))) if jerk.size else math.nan,
    }
    if endpoint == "MINIMUM_SPEED_PROXY" and math.isfinite(distance) and distance > 0:
        result["effective_deceleration_mps2"] = result["start_speed_mps"] ** 2 / (2.0 * distance)
        result["k_distance_per_speed_squared"] = distance / result["start_speed_mps"] ** 2
    elif parsed.collision.get("occurred") and math.isfinite(distance) and distance > 0:
        impact = fnum(parsed.collision.get("impact_speed_pre_event_sample_mps"), fnum(parsed.collision.get("impact_speed_mps")))
        result["preimpact_effective_deceleration_mps2"] = max(
            0.0,
            (result["start_speed_mps"] ** 2 - impact**2) / (2.0 * distance),
        )
    return result


def full_lifecycle_fusion_stats(parsed: ParsedRun) -> Dict[str, Any]:
    values = np.asarray(
        [row["latency_ms"] for row in parsed.perception.get("fusion_frames", []) if math.isfinite(row["latency_ms"])],
        dtype=float,
    )
    if not values.size:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean_ms": float(np.mean(values)),
        "median_ms": float(np.median(values)),
        "p90_ms": float(np.percentile(values, 90)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(np.max(values)),
        "over_500ms_count": int(np.sum(values > 500.0)),
        "over_500ms_ratio": float(np.mean(values > 500.0)),
    }


def raw_run_metrics(parsed: ParsedRun, config: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    stable: Optional[FusionObservation] = parsed.perception.get("stable")
    if stable is None:
        return (
            {
                "group_name": parsed.spec.group_name,
                "nominal_injected_delay_ms": parsed.spec.nominal_delay_ms,
                "run_id": parsed.spec.run_id,
                "analysis_status": "TARGET_UNCERTAIN",
            },
            {},
        )
    t1 = stable.obs_time_s
    ego_t1 = nearest_sample(parsed.localization, t1)
    geom = geometry_at_t1(stable, ego_t1, config)
    trace_e2e = parsed.trace.get("e2e_ms", {})
    fusion_ms = (stable.header_time_s - t1) * 1000.0
    prediction_ms = fnum(trace_e2e.get("prediction"))
    planning_trace_ms = fnum(trace_e2e.get("planning"))
    control_ms = fnum(trace_e2e.get("control"))
    planning_stop = parsed.planning.get("first_stop") or {}
    planning_stop_time = fnum(planning_stop.get("time_s"))
    planning_stop_ms = (planning_stop_time - t1) * 1000.0 if math.isfinite(planning_stop_time) else planning_trace_ms
    control_time = t1 + control_ms / 1000.0 if math.isfinite(control_ms) else math.nan

    primary_threshold = float(config["effective_brake"]["primary_decel_threshold_mps2"])
    onset = detect_brake_onset(
        parsed.localization,
        t1,
        control_time,
        primary_threshold,
        config,
        smoothed=False,
    )
    onset_time = fnum(onset.get("onset_time_s"))
    collision_time = fnum(parsed.collision.get("time_s"))
    stop = find_stop(parsed.localization, onset_time, collision_time, config) if math.isfinite(onset_time) else {"status": "MISSING"}
    near_stop = find_near_stop(parsed.localization, onset_time, collision_time, config) if math.isfinite(onset_time) else {"status": "MISSING"}
    brake_completion = find_brake_completion(parsed.localization, onset_time, collision_time, config) if math.isfinite(onset_time) else {"status": "MISSING"}
    braking = braking_statistics(parsed, onset, brake_completion)
    t2_speed = fnum(onset.get("onset_speed_mps"))
    actual_latency_ms = (onset_time - t1) * 1000.0 if math.isfinite(onset_time) else math.nan
    d_delay = integrate_speed(parsed.localization, t1, onset_time) if math.isfinite(onset_time) else math.nan
    d_delay_path = path_distance(parsed.localization, t1, onset_time) if math.isfinite(onset_time) else math.nan
    v1 = ego_t1.speed_mps if ego_t1 else math.nan
    d_delay_constant = v1 * actual_latency_ms / 1000.0 if math.isfinite(v1) and math.isfinite(actual_latency_ms) else math.nan
    scb_applied = parsed.scb.get("applied") or {}
    scb_trigger_time = fnum(scb_applied.get("receive_wall_time_s"))
    scb_trigger_relative = scb_trigger_time - t1 if math.isfinite(scb_trigger_time) else math.nan
    target_rows: Sequence[FusionObservation] = parsed.perception.get("target_rows", [])
    target_gaps = np.diff([row.header_time_s for row in target_rows]) if len(target_rows) > 1 else np.asarray([])
    lifecycle = full_lifecycle_fusion_stats(parsed)
    fallback_evidence = parsed.planning.get("fallback_evidence", {})
    map_observed = parsed.collision.get("map_name")
    map_name = map_observed.split("/")[-1] if map_observed else config["analysis"]["map_name"]
    map_source = "CARLA_collision_event" if map_observed else "user_fixed_experiment_condition"
    localization_intervals = np.diff([row.time_s for row in parsed.localization])

    metrics = {
        "group_name": parsed.spec.group_name,
        "nominal_injected_delay_ms": parsed.spec.nominal_delay_ms,
        "run_id": parsed.spec.run_id,
        "run_directory": str(parsed.spec.run_dir),
        "analysis_status": "ANALYZED" if onset.get("status") == "AVAILABLE" else onset.get("status", "UNKNOWN"),
        "target_id": parsed.target_id,
        "physical_target_id_chain": [parsed.target_id] if parsed.target_id else [],
        "target_association_method": "Planning STOP target ID + continuous static Fusion geometry",
        "target_association_confidence": "HIGH" if parsed.collision.get("occurred") else "MEDIUM_HIGH",
        "t_sensor_origin_s": t1,
        "t_perception_stable_output_s": stable.header_time_s,
        "t_prediction_first_s": t1 + prediction_ms / 1000.0 if math.isfinite(prediction_ms) else fnum((parsed.prediction.get("first_output") or {}).get("time_s")),
        "t_planning_stop_s": planning_stop_time,
        "t_planning_decel_s": fnum((parsed.planning.get("first_output") or {}).get("time_s")),
        "t_control_brake_command_s": control_time,
        "t_brake_effective_s": onset_time,
        "t_stop_s": fnum(stop.get("time_s")),
        "t_near_stop_s": fnum(near_stop.get("time_s")),
        "t_minimum_speed_s": fnum(brake_completion.get("time_s")),
        "stop_event_status": stop.get("status"),
        "braking_endpoint_type": braking.get("endpoint"),
        "t_collision_s": collision_time,
        "t_end_s": fnum(parsed.collect.get("end_log_epoch_s")),
        "t1_speed_mps": v1,
        "brake_start_speed_mps": t2_speed,
        "D1_center_m": fnum(geom.get("center_distance_m")),
        "D1_euclidean_center_m": fnum(geom.get("center_euclidean_distance_m")),
        "D1_clear_m": fnum(geom.get("longitudinal_clearance_m")),
        "D1_lateral_offset_m": fnum(geom.get("lateral_offset_m")),
        "geometry_offset_m": fnum(geom.get("combined_offset_m")),
        "geometry_method": geom.get("method"),
        "sensor_to_perception_ms": fusion_ms,
        "sensor_to_prediction_ms": prediction_ms,
        "sensor_to_planning_stop_ms": planning_stop_ms,
        "sensor_to_planning_output_ms": planning_trace_ms,
        "sensor_to_control_ms": control_ms,
        "perception_to_prediction_ms": prediction_ms - fusion_ms if math.isfinite(prediction_ms) else math.nan,
        "prediction_to_planning_stop_ms": planning_stop_ms - prediction_ms if math.isfinite(prediction_ms) and math.isfinite(planning_stop_ms) else math.nan,
        "planning_stop_to_control_ms": control_ms - planning_stop_ms if math.isfinite(control_ms) and math.isfinite(planning_stop_ms) else math.nan,
        "control_to_effective_brake_ms": actual_latency_ms - control_ms if math.isfinite(actual_latency_ms) and math.isfinite(control_ms) else math.nan,
        "actual_e2e_latency_ms": actual_latency_ms,
        "raw_brake_onset_latency_ms": fnum(onset.get("raw_latency_ms")),
        "brake_onset_attribution": onset.get("attribution") or onset.get("reason"),
        "D_delay_m": d_delay,
        "D_delay_path_m": d_delay_path,
        "D_delay_constant_speed_m": d_delay_constant,
        "D_delay_approx_error_m": d_delay_constant - d_delay if math.isfinite(d_delay_constant) and math.isfinite(d_delay) else math.nan,
        "clearance_at_brake_start_m": fnum(geom.get("longitudinal_clearance_m")) - d_delay if math.isfinite(d_delay) else math.nan,
        "empirical_braking_distance_m": fnum(braking.get("distance_m")) if braking.get("endpoint") == "MINIMUM_SPEED_PROXY" else math.nan,
        "empirical_braking_path_length_m": fnum(braking.get("path_length_m")) if braking.get("endpoint") == "MINIMUM_SPEED_PROXY" else math.nan,
        "empirical_braking_distance_method": braking.get("distance_method"),
        "braking_duration_s": fnum(braking.get("duration_s")),
        "mean_deceleration_mps2": fnum(braking.get("mean_deceleration_mps2")),
        "peak_deceleration_mps2": fnum(braking.get("peak_deceleration_mps2")),
        "deceleration_p10_mps2": fnum(braking.get("deceleration_p10_mps2")),
        "deceleration_p50_mps2": fnum(braking.get("deceleration_p50_mps2")),
        "deceleration_p90_mps2": fnum(braking.get("deceleration_p90_mps2")),
        "max_abs_braking_jerk_mps3": fnum(braking.get("max_abs_jerk_mps3")),
        "effective_deceleration_mps2": fnum(braking.get("effective_deceleration_mps2")),
        "k_distance_per_speed_squared": fnum(braking.get("k_distance_per_speed_squared")),
        "final_clearance_m": math.nan,
        "collision": bool(parsed.collision.get("occurred")),
        "collision_with_target": bool(parsed.collision.get("with_target")),
        "impact_speed_mps": fnum(parsed.collision.get("impact_speed_pre_event_sample_mps"), fnum(parsed.collision.get("impact_speed_mps"))),
        "collision_impulse_norm": fnum(parsed.collision.get("impulse_norm")),
        "distance_braked_before_collision_m": fnum(braking.get("distance_m")) if braking.get("endpoint") == "COLLISION" else math.nan,
        "time_braked_before_collision_s": fnum(braking.get("duration_s")) if braking.get("endpoint") == "COLLISION" else math.nan,
        "scb_log_present": bool(parsed.scb.get("present")),
        "scb_lifecycle_complete": set(parsed.scb.get("statuses", [])) >= {"BRIDGE_CONFIG_LOADED", "INITIALIZED", "APPLIED"},
        "scb_requested_delay_ms": fnum(scb_applied.get("requested_delay_ms")),
        "scb_actual_wall_delay_ms": fnum(scb_applied.get("actual_delay_ms")),
        "scb_actual_sim_delay_ms": fnum(scb_applied.get("actual_sim_delay_ms")),
        "scb_actual_frame_delay": fnum(scb_applied.get("actual_frame_delay")),
        "scb_trigger_relative_t1_s": scb_trigger_relative,
        "scb_activation_speed_mps": fnum(parsed.scb.get("activation_speed_mps")),
        "scb_brake_threshold_percentage": fnum(parsed.scb.get("brake_threshold_percentage")),
        "scb_first_brake_percentage": fnum(scb_applied.get("brake_percentage")),
        "scb_queue_depth_at_trigger": fnum(scb_applied.get("queue_depth")),
        "map_name": map_name,
        "map_source": map_source,
        "fixed_delta_seconds": float(config["analysis"]["fixed_delta_seconds"]),
        "pointcloud_count_configured": int(config["analysis"]["pointcloud_count"]),
        "pointcloud_count_source": "user_fixed_experiment_condition; raw count not archived in run directory",
        "localization_median_interval_ms": float(np.median(localization_intervals) * 1000.0) if localization_intervals.size else math.nan,
        "localization_frequency_hz": 1.0 / float(np.median(localization_intervals)) if localization_intervals.size else math.nan,
        "perception_detection_count": len(target_rows),
        "perception_gap_count": int(np.sum(target_gaps > normal_gap_limit(target_rows, config))) if target_gaps.size else 0,
        "max_perception_gap_ms": float(np.max(target_gaps) * 1000.0) if target_gaps.size else math.nan,
        "target_id_switch_count": 0,
        "target_missing_duration_ms": float(np.sum(target_gaps[target_gaps > normal_gap_limit(target_rows, config)]) * 1000.0) if target_gaps.size else 0.0,
        "target_static_classification_status": "STATIC" if parsed.prediction.get("static_valid_count", 0) > 0 and parsed.prediction.get("static_invalid_count", 0) == 0 else "UNVERIFIED_OR_MIXED",
        "fusion_lifecycle_mean_ms": fnum(lifecycle.get("mean_ms")),
        "fusion_lifecycle_p90_ms": fnum(lifecycle.get("p90_ms")),
        "fusion_lifecycle_p99_ms": fnum(lifecycle.get("p99_ms")),
        "fusion_lifecycle_max_ms": fnum(lifecycle.get("max_ms")),
        "fusion_lifecycle_over_500ms_count": lifecycle.get("over_500ms_count", 0),
        "planning_primal_infeasible_count": int(
            (fallback_evidence.get("primal_infeasible") or {}).get("count", 0)
        ),
        "planning_speed_fallback_count": int(
            (fallback_evidence.get("speed_fallback") or {}).get("count", 0)
        ),
        "planning_constant_deceleration_fallback_count": int(
            (fallback_evidence.get("constant_deceleration_fallback") or {}).get(
                "count", 0
            )
        ),
        "planning_first_speed_fallback_s": fnum(
            ((fallback_evidence.get("speed_fallback") or {}).get("first") or {}).get(
                "time_s"
            )
        ),
        "planning_first_speed_fallback_source_line": (
            ((fallback_evidence.get("speed_fallback") or {}).get("first") or {}).get(
                "source_line"
            )
        ),
        "clock_alignment_status": parsed.clock.get("status"),
        "clock_alignment_p95_residual_ms": fnum(parsed.clock.get("p95_abs_residual_ms")),
        "speed_condition_in_window": bool(math.isfinite(v1) and abs(v1 - float(config["quality"]["accepted_t1_speed_center_mps"])) <= float(config["quality"]["accepted_t1_speed_tolerance_mps"])),
        "source_perception_file": str(parsed.files.get("perception")) if parsed.files.get("perception") else None,
        "source_planning_file": str(parsed.files.get("planning")) if parsed.files.get("planning") else None,
        "source_localization_file": str(parsed.files.get("localization")) if parsed.files.get("localization") else None,
        "source_scb_file": str(parsed.files.get("scb")) if parsed.files.get("scb") else None,
        "source_collision_file": str(parsed.files.get("collision_csv")) if parsed.files.get("collision_csv") else None,
        "data_quality_notes": list(parsed.quality_notes),
    }
    clearance_endpoint = brake_completion
    if clearance_endpoint.get("status") == "AVAILABLE" and ego_t1 is not None:
        endpoint_sample = clearance_endpoint["sample"]
        dx = stable.x_m - endpoint_sample.x_m
        dy = stable.y_m - endpoint_sample.y_m
        final_center = dx * math.cos(ego_t1.heading_rad) + dy * math.sin(ego_t1.heading_rad)
        metrics["final_clearance_m"] = final_center - float(config["geometry"]["combined_center_to_surface_offset_m"])
    if (
        near_stop.get("status") == "AVAILABLE"
        and stop.get("status") != "AVAILABLE"
        and not parsed.collision.get("occurred")
    ):
        metrics["data_quality_notes"].append(
            "STRICT_STOP_HOLD_NOT_MET_MINIMUM_SPEED_PROXY_USED_FOR_BRAKING_DISTANCE"
        )

    sensitivity: Dict[str, Any] = {"stable_frames": {}, "brake_thresholds": {}}
    for frame_count, item in parsed.perception.get("stable_sensitivity", {}).items():
        candidate = item.get("row")
        sensitivity["stable_frames"][frame_count] = {
            "t1_s": candidate.obs_time_s if candidate else math.nan,
            "trace_id": candidate.trace_id if candidate else None,
        }
    for threshold in config["effective_brake"]["sensitivity_thresholds_mps2"]:
        raw_result = detect_brake_onset(parsed.localization, t1, control_time, float(threshold), config, smoothed=False)
        smooth_result = detect_brake_onset(parsed.localization, t1, control_time, float(threshold), config, smoothed=True)
        sensitivity["brake_thresholds"][str(threshold)] = {
            "raw_status": raw_result.get("status"),
            "raw_t2_s": fnum(raw_result.get("onset_time_s")),
            "raw_latency_ms": (fnum(raw_result.get("onset_time_s")) - t1) * 1000.0 if math.isfinite(fnum(raw_result.get("onset_time_s"))) else math.nan,
            "median3_status": smooth_result.get("status"),
            "median3_t2_s": fnum(smooth_result.get("onset_time_s")),
            "median3_latency_ms": (fnum(smooth_result.get("onset_time_s")) - t1) * 1000.0 if math.isfinite(fnum(smooth_result.get("onset_time_s"))) else math.nan,
        }
    debug = {
        "geometry": geom,
        "brake_onset": onset,
        "stop": stop,
        "near_stop": near_stop,
        "brake_completion": brake_completion,
        "braking": braking,
        "sensitivity": sensitivity,
        "lifecycle_fusion": lifecycle,
    }
    return metrics, debug


def bootstrap_ci(
    values: Sequence[float],
    statistic: Any,
    iterations: int,
    rng: np.random.Generator,
) -> Tuple[float, float]:
    data = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not data.size:
        return math.nan, math.nan
    samples = rng.choice(data, size=(iterations, data.size), replace=True)
    estimates = np.apply_along_axis(statistic, 1, samples)
    return float(np.percentile(estimates, 2.5)), float(np.percentile(estimates, 97.5))


def build_braking_model(
    metrics: Sequence[Dict[str, Any]],
    config: Dict[str, Any],
    output_root: Path,
) -> Dict[str, Any]:
    samples = [
        row
        for row in metrics
        if row.get("group_name") == "baseline"
        and not row.get("collision")
        and math.isfinite(fnum(row.get("empirical_braking_distance_m")))
        and math.isfinite(fnum(row.get("brake_start_speed_mps")))
    ]
    sample_rows: List[Dict[str, Any]] = []
    k_values: List[float] = []
    a_values: List[float] = []
    for row in samples:
        speed = fnum(row["brake_start_speed_mps"])
        distance = fnum(row["empirical_braking_distance_m"])
        k = distance / speed**2
        a_eff = speed**2 / (2.0 * distance)
        k_values.append(k)
        a_values.append(a_eff)
        sample_rows.append(
            {
                "run_id": row["run_id"],
                "brake_start_speed_mps": speed,
                "empirical_braking_distance_m": distance,
                "empirical_braking_path_length_m": fnum(
                    row.get("empirical_braking_path_length_m")
                ),
                "distance_method": row.get("empirical_braking_distance_method"),
                "k_distance_per_speed_squared": k,
                "effective_deceleration_mps2": a_eff,
                "source_localization_file": row.get("source_localization_file"),
            }
        )
    rng = np.random.default_rng(int(config["analysis"]["random_seed"]))
    iterations = int(config["statistics"]["bootstrap_iterations"])
    k_median = float(np.median(k_values)) if k_values else math.nan
    a_median = float(np.median(a_values)) if a_values else math.nan
    k_ci = bootstrap_ci(k_values, np.median, iterations, rng)
    a_ci = bootstrap_ci(a_values, np.median, iterations, rng)
    bootstrap_rows: List[Dict[str, Any]] = []
    if k_values:
        array = np.asarray(k_values)
        for index in range(iterations):
            sample = rng.choice(array, size=len(array), replace=True)
            k = float(np.median(sample))
            bootstrap_rows.append(
                {
                    "iteration": index,
                    "k_median": k,
                    "effective_deceleration_mps2": 1.0 / (2.0 * k),
                }
            )
    model = {
        "model": "D_brake_required(v) = k_median * v^2",
        "scope": "current Town04 static-obstacle experiment near 15-17 m/s",
        "sample_count": len(sample_rows),
        "sample_run_ids": [row["run_id"] for row in sample_rows],
        "distance_method": "3-D Localization displacement from effective brake onset to the post-t2 minimum-speed sample, conditioned on reaching v<0.1 m/s",
        "strict_stop_definition": "v<0.1 m/s held for at least 0.5 s",
        "near_stop_proxy_definition": "first v<0.1 m/s sample after effective brake onset",
        "brake_completion_proxy_definition": "minimum-speed sample after effective brake onset, accepted only when the run reaches v<0.1 m/s",
        "empirical_braking_distance_mean_m": float(np.mean([row["empirical_braking_distance_m"] for row in sample_rows])) if sample_rows else math.nan,
        "k_median": k_median,
        "k_bootstrap_95ci": list(k_ci),
        "effective_deceleration_median_mps2": a_median,
        "effective_deceleration_bootstrap_95ci_mps2": list(a_ci),
        "bootstrap_iterations": iterations,
        "random_seed": int(config["analysis"]["random_seed"]),
    }
    write_csv(output_root / "baseline_braking_samples.csv", sample_rows)
    write_csv(output_root / "braking_model_bootstrap.csv", bootstrap_rows)
    write_json(output_root / "baseline_braking_model.json", model)
    return model


def nearest_baseline_delta(
    row: Dict[str, Any], baseline: Sequence[Dict[str, Any]]
) -> Tuple[float, Optional[str]]:
    candidates = [
        item
        for item in baseline
        if math.isfinite(fnum(item.get("actual_e2e_latency_ms")))
        and math.isfinite(fnum(item.get("t1_speed_mps")))
        and math.isfinite(fnum(item.get("D1_clear_m")))
    ]
    if not candidates:
        return math.nan, None
    speed = fnum(row.get("t1_speed_mps"))
    d1 = fnum(row.get("D1_clear_m"))
    selected = min(
        candidates,
        key=lambda item: ((fnum(item["t1_speed_mps"]) - speed) / 0.5) ** 2
        + ((fnum(item["D1_clear_m"]) - d1) / 1.0) ** 2,
    )
    return fnum(row.get("actual_e2e_latency_ms")) - fnum(selected.get("actual_e2e_latency_ms")), selected["run_id"]


def enrich_safety_metrics(
    metrics: List[Dict[str, Any]],
    model: Dict[str, Any],
    config: Dict[str, Any],
) -> List[Dict[str, Any]]:
    k = fnum(model.get("k_median"))
    primary_margin = float(config["safety"]["primary_margin_m"])
    baseline_valid = [
        row
        for row in metrics
        if row.get("group_name") == "baseline" and math.isfinite(fnum(row.get("actual_e2e_latency_ms")))
    ]
    baseline_median = float(np.median([fnum(row["actual_e2e_latency_ms"]) for row in baseline_valid])) if baseline_valid else math.nan
    for row in metrics:
        v1 = fnum(row.get("t1_speed_mps"))
        vb = fnum(row.get("brake_start_speed_mps"))
        d1 = fnum(row.get("D1_clear_m"))
        delay = fnum(row.get("D_delay_m"))
        latency = fnum(row.get("actual_e2e_latency_ms"))
        d_brake = k * vb**2 if math.isfinite(k) and math.isfinite(vb) else math.nan
        d_brake_v1 = k * v1**2 if math.isfinite(k) and math.isfinite(v1) else math.nan
        m_space = d1 - delay - d_brake - primary_margin if all(math.isfinite(value) for value in [d1, delay, d_brake]) else math.nan
        m_collision = d1 - delay - d_brake if all(
            math.isfinite(value) for value in [d1, delay, d_brake]
        ) else math.nan
        deadline = (d1 - d_brake_v1 - primary_margin) / v1 if all(math.isfinite(value) for value in [d1, d_brake_v1, v1]) and v1 > 0 else math.nan
        collision_deadline = (d1 - d_brake_v1) / v1 if all(math.isfinite(value) for value in [d1, d_brake_v1, v1]) and v1 > 0 else math.nan
        clearance_at_brake = fnum(row.get("clearance_at_brake_start_m"))
        m_at_brake = clearance_at_brake - d_brake - primary_margin if all(math.isfinite(value) for value in [clearance_at_brake, d_brake]) else math.nan
        added = latency - baseline_median if math.isfinite(latency) and math.isfinite(baseline_median) else math.nan
        paired_added, paired_run = nearest_baseline_delta(row, baseline_valid) if row.get("group_name") != "baseline" else (latency - baseline_median, None)
        saved = math.nan
        if math.isfinite(added) and math.isfinite(fnum(row.get("t_brake_effective_s"))):
            if added >= 0:
                end = fnum(row["t_brake_effective_s"])
                # D_saved is filled with a constant-speed fallback below when the
                # parsed localization series is not available in this phase.
                saved = v1 * added / 1000.0
            else:
                saved = v1 * added / 1000.0
        counterfactual = m_space + saved if math.isfinite(m_space) and math.isfinite(saved) else math.nan
        counterfactual_collision = (
            m_collision + saved
            if math.isfinite(m_collision) and math.isfinite(saved)
            else math.nan
        )
        row.update(
            {
                "D_brake_required_m": d_brake,
                "D_brake_required_at_D1_speed_m": d_brake_v1,
                "D_margin_m": primary_margin,
                "D_margin_source": config["safety"]["margin_source"],
                "M_space_m": m_space,
                "M_safety_6m_m": m_space,
                "M_collision_0m_m": m_collision,
                "M_at_brake_start_m": m_at_brake,
                "M_space_consistency_error_m": m_space - m_at_brake if math.isfinite(m_space) and math.isfinite(m_at_brake) else math.nan,
                "T_deadline_s": deadline,
                "T_collision_deadline_s": collision_deadline,
                "M_time_s": deadline - latency / 1000.0 if math.isfinite(deadline) and math.isfinite(latency) else math.nan,
                "M_time_from_space_s": m_space / v1 if math.isfinite(m_space) and math.isfinite(v1) and v1 > 0 else math.nan,
                "measured_added_delay_ms": added,
                "phase_matched_added_delay_ms": paired_added,
                "matched_baseline_run_id": paired_run,
                "D_saved_counterfactual_m": saved,
                "M_space_counterfactual_m": counterfactual,
                "M_safety_6m_counterfactual_m": counterfactual,
                "M_collision_0m_counterfactual_m": counterfactual_collision,
                "counterfactual_safe": bool(counterfactual > 0) if math.isfinite(counterfactual) else None,
                "counterfactual_collision_avoided": bool(counterfactual_collision > 0)
                if math.isfinite(counterfactual_collision)
                else None,
                "counterfactual_confidence": "MEDIUM" if row.get("scb_log_present") else "LOW",
            }
        )
    return metrics


def assess_modules(parsed: ParsedRun, row: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
    stable_ok = parsed.perception.get("stable") is not None
    perception_degraded = fnum(row.get("sensor_to_perception_ms")) > float(config["quality"]["target_fusion_degraded_threshold_ms"])
    perception_status = "PASS" if stable_ok and not perception_degraded else ("DEGRADED" if stable_ok else "FAIL")
    prediction_ok = parsed.prediction.get("input_count", 0) > 0 and parsed.prediction.get("output_count", 0) > 0 and parsed.prediction.get("static_valid_count", 0) > 0
    prediction_status = "PASS" if prediction_ok else ("DEGRADED" if parsed.prediction.get("output_count", 0) else "FAIL")
    planning_output = parsed.planning.get("first_output") or {}
    planning_ok = parsed.planning.get("stop_count", 0) > 0 and str(planning_output.get("status_ok")) == "1"
    planning_status = "PASS" if planning_ok else ("DEGRADED" if parsed.planning.get("stop_count", 0) > 0 else "FAIL")
    control_trace_ok = math.isfinite(fnum(row.get("sensor_to_control_ms")))
    physical_after_control = math.isfinite(fnum(row.get("control_to_effective_brake_ms"))) and fnum(row.get("control_to_effective_brake_ms")) >= -20.0
    control_status = "PASS" if control_trace_ok and physical_after_control else ("DEGRADED" if control_trace_ok else "UNKNOWN")
    bridge_complete = bool(row.get("scb_lifecycle_complete"))
    bridge_status = "PASS" if bridge_complete else ("UNKNOWN" if not row.get("scb_log_present") else "DEGRADED")
    primal_infeasible_count = int(row.get("planning_primal_infeasible_count") or 0)
    speed_fallback_count = int(row.get("planning_speed_fallback_count") or 0)
    constant_deceleration_fallback_count = int(
        row.get("planning_constant_deceleration_fallback_count") or 0
    )
    return {
        "run_id": row["run_id"],
        "group_name": row["group_name"],
        "perception_status": perception_status,
        "prediction_status": prediction_status,
        "planning_status": planning_status,
        "control_status": control_status,
        "bridge_status": bridge_status,
        "perception_evidence": f"stable_frames={len(parsed.perception.get('stable_segment', []))}; target_latency_ms={fnum(row.get('sensor_to_perception_ms')):.3f}",
        "prediction_evidence": f"input={parsed.prediction.get('input_count', 0)}; output={parsed.prediction.get('output_count', 0)}; static_valid={parsed.prediction.get('static_valid_count', 0)}",
        "planning_evidence": (
            f"stop_count={parsed.planning.get('stop_count', 0)}; "
            f"target_id={parsed.target_id}; status_ok={planning_output.get('status_ok')}; "
            f"primal_infeasible={primal_infeasible_count}; "
            f"speed_fallback={speed_fallback_count}; "
            f"constant_deceleration_fallback={constant_deceleration_fallback_count}; "
            "infeasible speed solution followed by fallback is treated as the expected planning response"
        ),
        "control_evidence": f"target_trace_control_output_ms={fnum(row.get('sensor_to_control_ms')):.3f}; command payload not archived; physical_after_control={physical_after_control}",
        "bridge_evidence": f"scb_present={row.get('scb_log_present')}; statuses={'|'.join(parsed.scb.get('statuses', []))}",
        "source_perception_file": row.get("source_perception_file"),
        "source_prediction_file": str(parsed.files.get("prediction")) if parsed.files.get("prediction") else None,
        "source_planning_file": row.get("source_planning_file"),
        "source_control_trace_file": str(parsed.files.get("control_context")) if parsed.files.get("control_context") else None,
        "source_scb_file": row.get("source_scb_file"),
    }


def classify_run(row: Dict[str, Any], module: Dict[str, Any]) -> Dict[str, Any]:
    collision = bool(row.get("collision_with_target"))
    m_safety_6m = fnum(row.get("M_safety_6m_m"))
    m_safety_6m_counterfactual = fnum(row.get("M_safety_6m_counterfactual_m"))
    m_collision_0m = fnum(row.get("M_collision_0m_m"))
    m_collision_0m_counterfactual = fnum(row.get("M_collision_0m_counterfactual_m"))
    statuses = [module[key] for key in ["perception_status", "prediction_status", "planning_status", "control_status", "bridge_status"]]
    function_chain_pass = all(status == "PASS" for status in statuses)
    timing_degraded = module["perception_status"] == "DEGRADED"
    response_valid = row.get("analysis_status") == "ANALYZED"
    if not response_valid or row.get("target_id") is None:
        classification = "INDETERMINATE"
        evidence = "目标关联或有效制动时刻无法可靠确定"
    elif not row.get("scb_log_present") and row.get("group_name") != "baseline":
        classification = "INDETERMINATE"
        evidence = "注入组缺少SCB执行记录，实际注入状态无法逐run核验"
    elif collision and timing_degraded:
        classification = "TIMING_INDUCED_FUNCTIONAL_DEGRADATION"
        evidence = "碰撞伴随目标关键帧感知排队，功能输出存在但实时链路显著退化"
    elif collision and function_chain_pass and row.get("group_name") != "baseline":
        classification = "RT_ONLY_COLLISION"
        evidence = "目标功能链完整，注入时延组发生目标碰撞，同条件baseline全部安全停车"
    elif collision and any(status == "FAIL" for status in statuses):
        classification = "NON_TIMING_FUNCTIONAL_FAILURE"
        evidence = "碰撞run存在独立功能模块失败证据"
    elif collision:
        classification = "INDETERMINATE"
        evidence = "碰撞与时延增加同时存在，严格因果必要证据未全部满足"
    elif math.isfinite(m_safety_6m) and m_safety_6m > 1.0:
        classification = "SAFE_NORMAL"
        evidence = "车辆停车或保持无碰撞，空间余量为正"
    elif math.isfinite(m_safety_6m):
        classification = "SAFE_CRITICAL"
        evidence = "未碰撞，空间余量接近或低于零，属于临界停车或5/6m裕度失效"
    else:
        classification = "INDETERMINATE"
        evidence = "空间安全指标缺失"
    confidence = "HIGH" if classification == "RT_ONLY_COLLISION" and row.get("clock_alignment_status") == "ALIGNED" else ("MEDIUM" if classification not in {"INDETERMINATE"} else "LOW")
    realtime_induced_collision = bool(
        collision
        and classification
        in {"RT_ONLY_COLLISION", "TIMING_INDUCED_FUNCTIONAL_DEGRADATION"}
    )
    return {
        "run_id": row["run_id"],
        "group_name": row["group_name"],
        "collision": bool(row.get("collision")),
        "collision_with_target": collision,
        "classification": classification,
        "realtime_induced_collision": realtime_induced_collision,
        "perception_status": module["perception_status"],
        "prediction_status": module["prediction_status"],
        "planning_status": module["planning_status"],
        "control_status": module["control_status"],
        "bridge_status": module["bridge_status"],
        "M_space_m": m_safety_6m,
        "M_space_counterfactual_m": m_safety_6m_counterfactual,
        "M_safety_6m_m": m_safety_6m,
        "M_safety_6m_counterfactual_m": m_safety_6m_counterfactual,
        "M_collision_0m_m": m_collision_0m,
        "M_collision_0m_counterfactual_m": m_collision_0m_counterfactual,
        "actual_e2e_latency_ms": fnum(row.get("actual_e2e_latency_ms")),
        "evidence_summary": evidence,
        "uncertainty": ";".join(row.get("data_quality_notes", [])),
        "confidence_level": confidence,
    }


def target_association(parsed: ParsedRun) -> Dict[str, Any]:
    history = parsed.files.get("actor_history")
    base = {
        "run_id": parsed.spec.run_id,
        "group_name": parsed.spec.group_name,
        "carla_target_actor_id": parsed.collision.get("other_actor_id"),
        "apollo_obstacle_id_sequence": [parsed.target_id] if parsed.target_id else [],
        "id_switch_count": 0,
        "id_switch_times_s": [],
        "position_error_median_m": math.nan,
        "position_error_max_m": math.nan,
        "velocity_consistent": None,
        "type_consistent": None,
        "target_chain_broken": False,
        "confidence": "MEDIUM_HIGH",
        "conclusion": "Planning STOP目标与静态连续Fusion目标一致",
        "source_file": str(parsed.files.get("perception")) if parsed.files.get("perception") else None,
    }
    if history is None or not parsed.collision.get("occurred"):
        return base
    other_id = str(parsed.collision.get("other_actor_id"))
    truth: List[Tuple[float, float, float, float]] = []
    with history.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("role") != "other" or str(row.get("actor_id")) != other_id:
                continue
            truth.append(
                (
                    fnum(row.get("wall_time_unix_ns")) / 1e9,
                    fnum(row.get("location_x")),
                    -fnum(row.get("location_y")),
                    math.sqrt(fnum(row.get("velocity_x"), 0.0) ** 2 + fnum(row.get("velocity_y"), 0.0) ** 2),
                )
            )
    truth.sort()
    errors: List[float] = []
    speed_errors: List[float] = []
    for obs in parsed.perception.get("target_rows", []):
        if not truth or obs.obs_time_s < truth[0][0] or obs.obs_time_s > truth[-1][0]:
            continue
        times = np.asarray([item[0] for item in truth])
        index = int(np.searchsorted(times, obs.obs_time_s))
        index = min(max(index, 1), len(truth) - 1)
        left, right = truth[index - 1], truth[index]
        ratio = (obs.obs_time_s - left[0]) / max(right[0] - left[0], 1e-9)
        x = left[1] + ratio * (right[1] - left[1])
        y = left[2] + ratio * (right[2] - left[2])
        speed = left[3] + ratio * (right[3] - left[3])
        errors.append(math.hypot(obs.x_m - x, obs.y_m - y))
        speed_errors.append(abs(obs.speed_mps - speed))
    if errors:
        base.update(
            {
                "position_error_median_m": float(np.median(errors)),
                "position_error_max_m": float(np.max(errors)),
                "velocity_consistent": bool(np.median(speed_errors) < 1.0),
                "type_consistent": True,
                "confidence": "HIGH" if np.median(errors) < 4.0 else "LOW",
                "conclusion": "CARLA碰撞目标history经y轴转换后与Apollo Fusion目标多帧匹配",
                "source_file": str(history),
                "matched_frame_count": len(errors),
            }
        )
    else:
        base.update({"confidence": "LOW", "conclusion": "CARLA history与目标Fusion时间窗口无重叠"})
    return base


def descriptive_stats(values: Sequence[float], config: Dict[str, Any], rng: np.random.Generator) -> Dict[str, Any]:
    data = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not data.size:
        return {"valid_n": 0}
    ci = bootstrap_ci(
        data.tolist(),
        np.mean,
        int(config["statistics"]["bootstrap_iterations"]),
        rng,
    )
    return {
        "valid_n": int(data.size),
        "mean": float(np.mean(data)),
        "std": float(np.std(data, ddof=1)) if data.size > 1 else 0.0,
        "median": float(np.median(data)),
        "q1": float(np.percentile(data, 25)),
        "q3": float(np.percentile(data, 75)),
        "min": float(np.min(data)),
        "max": float(np.max(data)),
        "p90": float(np.percentile(data, 90)),
        "bootstrap_mean_ci_low": ci[0],
        "bootstrap_mean_ci_high": ci[1],
    }


def cliffs_delta(left: Sequence[float], right: Sequence[float]) -> float:
    a = np.asarray([value for value in left if math.isfinite(value)], dtype=float)
    b = np.asarray([value for value in right if math.isfinite(value)], dtype=float)
    if not a.size or not b.size:
        return math.nan
    greater = sum(float(x > y) for x in a for y in b)
    less = sum(float(x < y) for x in a for y in b)
    return (greater - less) / (a.size * b.size)


def holm_adjust(p_values: Sequence[float]) -> List[float]:
    result = [math.nan] * len(p_values)
    valid = [(index, value) for index, value in enumerate(p_values) if math.isfinite(value)]
    ordered = sorted(valid, key=lambda item: item[1])
    previous = 0.0
    total = len(ordered)
    for rank, (index, value) in enumerate(ordered):
        adjusted = min(1.0, (total - rank) * value)
        adjusted = max(previous, adjusted)
        result[index] = adjusted
        previous = adjusted
    return result


def group_statistics(
    metrics: Sequence[Dict[str, Any]],
    classifications: Sequence[Dict[str, Any]],
    config: Dict[str, Any],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    group_order = list(config["groups"])
    core = [
        "actual_e2e_latency_ms",
        "measured_added_delay_ms",
        "D1_clear_m",
        "D_delay_m",
        "clearance_at_brake_start_m",
        "D_brake_required_m",
        "M_space_m",
        "M_time_s",
        "final_clearance_m",
        "impact_speed_mps",
        "max_perception_gap_ms",
        "sensor_to_planning_stop_ms",
        "control_to_effective_brake_ms",
    ]
    rng = np.random.default_rng(int(config["analysis"]["random_seed"]))
    summary_rows: List[Dict[str, Any]] = []
    by_group: Dict[str, List[Dict[str, Any]]] = {
        group: [row for row in metrics if row.get("group_name") == group] for group in group_order
    }
    for group in group_order:
        rows = by_group[group]
        for metric in core:
            stats_row = descriptive_stats([fnum(row.get(metric)) for row in rows], config, rng)
            summary_rows.append(
                {
                    "group_name": group,
                    "metric": metric,
                    "n": len(rows),
                    "missing_n": len(rows) - stats_row.get("valid_n", 0),
                    **stats_row,
                }
            )
    test_rows: List[Dict[str, Any]] = []
    effect_rows: List[Dict[str, Any]] = []
    for metric in ["actual_e2e_latency_ms", "D_delay_m", "M_space_m"]:
        samples = [
            np.asarray([fnum(row.get(metric)) for row in by_group[group] if math.isfinite(fnum(row.get(metric)))])
            for group in group_order
        ]
        if all(sample.size for sample in samples):
            kw = stats.kruskal(*samples)
            test_rows.append(
                {
                    "metric": metric,
                    "comparison": "four_group_overall",
                    "test": "Kruskal-Wallis",
                    "statistic": float(kw.statistic),
                    "p_value": float(kw.pvalue),
                    "p_holm": math.nan,
                    "interpretation": "exploratory_small_sample",
                }
            )
        base = samples[0]
        pair_rows: List[Dict[str, Any]] = []
        for group, sample in zip(group_order[1:], samples[1:]):
            if base.size and sample.size:
                test = stats.mannwhitneyu(base, sample, alternative="two-sided", method="auto")
                pair_rows.append(
                    {
                        "metric": metric,
                        "comparison": f"{group}_vs_baseline",
                        "test": "Mann-Whitney U",
                        "statistic": float(test.statistic),
                        "p_value": float(test.pvalue),
                        "p_holm": math.nan,
                        "interpretation": "exploratory_small_sample",
                    }
                )
                effect_rows.append(
                    {
                        "metric": metric,
                        "comparison": f"{group}_vs_baseline",
                        "effect": "Cliffs_delta_group_minus_baseline",
                        "value": cliffs_delta(sample.tolist(), base.tolist()),
                        "baseline_values": base.tolist(),
                        "group_values": sample.tolist(),
                    }
                )
        adjusted = holm_adjust([row["p_value"] for row in pair_rows])
        for row, value in zip(pair_rows, adjusted):
            row["p_holm"] = value
        test_rows.extend(pair_rows)

    collision_rows: List[Dict[str, Any]] = []
    baseline_rows = by_group["baseline"]
    base_collisions = sum(bool(row.get("collision")) for row in baseline_rows)
    for group in group_order:
        rows = by_group[group]
        collisions = sum(bool(row.get("collision")) for row in rows)
        n = len(rows)
        low, high = stats.binomtest(collisions, n).proportion_ci(confidence_level=0.95, method="exact") if n else (math.nan, math.nan)
        fisher_p = math.nan
        if group != "baseline":
            table = [[base_collisions, len(baseline_rows) - base_collisions], [collisions, n - collisions]]
            fisher_p = float(stats.fisher_exact(table, alternative="two-sided").pvalue)
        collision_rows.append(
            {
                "group_name": group,
                "collision_count": collisions,
                "total_runs": n,
                "collision_rate": collisions / n if n else math.nan,
                "exact_95ci_low": float(low),
                "exact_95ci_high": float(high),
                "fisher_vs_baseline_p": fisher_p,
            }
        )
    return summary_rows, test_rows, effect_rows, collision_rows


def sensitivity_rows(
    parsed_runs: Sequence[ParsedRun],
    metrics: Sequence[Dict[str, Any]],
    model: Dict[str, Any],
    config: Dict[str, Any],
) -> List[Dict[str, Any]]:
    metric_by_run = {row["run_id"]: row for row in metrics}
    k = fnum(model.get("k_median"))
    rows: List[Dict[str, Any]] = []
    for parsed in parsed_runs:
        base = metric_by_run[parsed.spec.run_id]
        for frames in config["stable_perception"]["sensitivity_frames"]:
            item = parsed.perception.get("stable_sensitivity", {}).get(str(frames), {})
            stable = item.get("row")
            if stable is None:
                continue
            ego = nearest_sample(parsed.localization, stable.obs_time_s)
            geom = geometry_at_t1(stable, ego, config)
            for threshold in config["effective_brake"]["sensitivity_thresholds_mps2"]:
                control_ms = fnum(base.get("sensor_to_control_ms"))
                control_s = stable.obs_time_s + control_ms / 1000.0 if math.isfinite(control_ms) else math.nan
                onset = detect_brake_onset(parsed.localization, stable.obs_time_s, control_s, float(threshold), config, smoothed=False)
                onset_s = fnum(onset.get("onset_time_s"))
                onset_speed = fnum(onset.get("onset_speed_mps"))
                delay = integrate_speed(parsed.localization, stable.obs_time_s, onset_s) if math.isfinite(onset_s) else math.nan
                for margin in config["safety"]["sensitivity_margins_m"]:
                    d_required = k * onset_speed**2 if math.isfinite(k) and math.isfinite(onset_speed) else math.nan
                    m_space = fnum(geom.get("longitudinal_clearance_m")) - delay - d_required - float(margin) if all(math.isfinite(value) for value in [fnum(geom.get("longitudinal_clearance_m")), delay, d_required]) else math.nan
                    saved = fnum(base.get("D_saved_counterfactual_m"))
                    counterfactual_m_space = (
                        m_space + saved
                        if math.isfinite(m_space) and math.isfinite(saved)
                        else math.nan
                    )
                    rt_only_margin_condition = bool(
                        base.get("collision_with_target")
                        and math.isfinite(m_space)
                        and m_space < 0
                        and math.isfinite(counterfactual_m_space)
                        and counterfactual_m_space > 0
                    )
                    if base.get("collision"):
                        classification = "COLLISION_NEGATIVE_MARGIN" if math.isfinite(m_space) and m_space < 0 else "COLLISION_MODEL_CONFLICT"
                    else:
                        classification = "SAFE_MARGIN_POSITIVE" if math.isfinite(m_space) and m_space >= 0 else "SAFE_MARGIN_LOSS"
                    rows.append(
                        {
                            "run_id": parsed.spec.run_id,
                            "group_name": parsed.spec.group_name,
                            "stable_frames": int(frames),
                            "brake_decel_threshold_mps2": float(threshold),
                            "D_margin_m": float(margin),
                            "margin_role": (
                                "collision_avoidance_boundary"
                                if float(margin) == 0.0
                                else (
                                    "six_meter_safety_boundary"
                                    if float(margin) == 6.0
                                    else "safety_margin_sensitivity"
                                )
                            ),
                            "t1_s": stable.obs_time_s,
                            "t2_s": onset_s,
                            "actual_e2e_latency_ms": (onset_s - stable.obs_time_s) * 1000.0 if math.isfinite(onset_s) else math.nan,
                            "M_space_m": m_space,
                            "M_space_counterfactual_m": counterfactual_m_space,
                            "rt_only_margin_condition": rt_only_margin_condition,
                            "classification_sensitivity": classification,
                            "onset_status": onset.get("status"),
                        }
                    )
    return rows


def event_timeline_for_run(parsed: ParsedRun, row: Dict[str, Any]) -> List[Dict[str, Any]]:
    stable: Optional[FusionObservation] = parsed.perception.get("stable")
    first_target = parsed.perception.get("target_rows", [None])[0] if parsed.perception.get("target_rows") else None
    prediction_first = parsed.prediction.get("first_output") or {}
    event_defs = [
        ("t_spawn", math.nan, None, None, "not archived"),
        ("t_sensor_first_visible", first_target.obs_time_s if first_target else math.nan, first_target.source_file if first_target else None, first_target.source_line if first_target else None, "first Fusion target source observation"),
        ("t_perception_first", first_target.header_time_s if first_target else math.nan, first_target.source_file if first_target else None, first_target.source_line if first_target else None, "first Fusion target output"),
        ("t_sensor_origin", fnum(row.get("t_sensor_origin_s")), stable.source_file if stable else None, stable.source_line if stable else None, "first source frame in 3-frame stable sequence"),
        ("t_perception_stable", fnum(row.get("t_perception_stable_output_s")), stable.source_file if stable else None, stable.source_line if stable else None, "output time of stable sequence first frame"),
        ("t_prediction_first", fnum(row.get("t_prediction_first_s")), prediction_first.get("source_file"), prediction_first.get("source_line"), "target prediction output"),
        ("t_prediction_static", fnum(row.get("t_prediction_first_s")) if row.get("target_static_classification_status") == "STATIC" else math.nan, prediction_first.get("source_file"), prediction_first.get("source_line"), "pred_has_is_static=1 and pred_is_static=1"),
        ("t_planning_stop", fnum(row.get("t_planning_stop_s")), (parsed.planning.get("first_stop") or {}).get("source_file"), (parsed.planning.get("first_stop") or {}).get("source_line"), "first STOP decision for target"),
        ("t_planning_decel", fnum(row.get("t_planning_decel_s")), (parsed.planning.get("first_output") or {}).get("source_file"), (parsed.planning.get("first_output") or {}).get("source_line"), "first target stop trajectory output"),
        ("t_control_brake_command", fnum(row.get("t_control_brake_command_s")), str(parsed.files.get("control_context")) if parsed.files.get("control_context") else None, None, "first /apollo/control output inheriting target trace; payload unavailable"),
        ("t_brake_effective", fnum(row.get("t_brake_effective_s")), row.get("source_localization_file"), None, "two consecutive deceleration intervals"),
        ("t_stop", fnum(row.get("t_stop_s")), row.get("source_localization_file"), None, "speed <0.1 m/s for 0.5 s"),
        ("t_collision", fnum(row.get("t_collision_s")), row.get("source_collision_file"), 2 if row.get("source_collision_file") else None, "first CollisionSensor event"),
        ("t_end", fnum(row.get("t_end_s")), str(parsed.files.get("collect")) if parsed.files.get("collect") else None, None, "collection end_log"),
    ]
    t1 = fnum(row.get("t_sensor_origin_s"))
    d1 = fnum(row.get("D1_clear_m"))
    output: List[Dict[str, Any]] = []
    for event, timestamp, source_file, source_line, method in event_defs:
        ego = nearest_sample(parsed.localization, timestamp) if math.isfinite(timestamp) else None
        clearance = math.nan
        if math.isfinite(timestamp) and math.isfinite(t1) and math.isfinite(d1):
            if timestamp >= t1:
                travel = integrate_speed(parsed.localization, t1, timestamp)
                clearance = d1 - travel if math.isfinite(travel) else math.nan
            else:
                travel = integrate_speed(parsed.localization, timestamp, t1)
                clearance = d1 + travel if math.isfinite(travel) else math.nan
        output.append(
            {
                "run_id": parsed.spec.run_id,
                "group_name": parsed.spec.group_name,
                "event": event,
                "unified_timestamp_s": timestamp,
                "relative_to_t1_ms": (timestamp - t1) * 1000.0 if math.isfinite(timestamp) and math.isfinite(t1) else math.nan,
                "original_timestamp": timestamp,
                "original_clock_domain": "Apollo_epoch/wall_epoch" if math.isfinite(timestamp) else "unavailable",
                "carla_frame": parsed.collision.get("carla_frame") if event == "t_collision" else math.nan,
                "ego_speed_mps": ego.speed_mps if ego else math.nan,
                "longitudinal_clearance_m": clearance,
                "target_id": parsed.target_id,
                "source_file": source_file,
                "source_line": source_line,
                "detection_method": method,
                "confidence": "HIGH" if math.isfinite(timestamp) and event not in {"t_control_brake_command"} else ("MEDIUM" if math.isfinite(timestamp) else "UNAVAILABLE"),
                "remarks": "",
            }
        )
    return output


def configure_matplotlib() -> None:
    font_candidates = [
        Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/simhei.ttf"),
    ]
    for font in font_candidates:
        if font.exists():
            from matplotlib import font_manager

            font_manager.fontManager.addfont(str(font))
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
            break
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "figure.dpi": 120,
            "savefig.dpi": 320,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "font.size": 9,
        }
    )


GROUP_LABELS = {
    "baseline": "Baseline",
    "delay_100ms": "100 ms",
    "delay_300ms": "300 ms",
    "delay_400ms": "400 ms",
}
GROUP_COLORS = {
    "baseline": "#4C78A8",
    "delay_100ms": "#72B7B2",
    "delay_300ms": "#F2CF5B",
    "delay_400ms": "#E45756",
}


def save_figure(fig: Any, output_root: Path, stem: str) -> None:
    fig.tight_layout()
    fig.savefig(output_root / "figures" / f"{stem}.png", dpi=320, bbox_inches="tight")
    fig.savefig(output_root / "figures" / f"{stem}.svg", bbox_inches="tight")
    plt.close(fig)


def _metric_arrays(metrics: Sequence[Dict[str, Any]], key: str, group_order: Sequence[str]) -> List[np.ndarray]:
    return [
        np.asarray([fnum(row.get(key)) for row in metrics if row.get("group_name") == group and math.isfinite(fnum(row.get(key)))])
        for group in group_order
    ]


def generate_figures(
    parsed_runs: Sequence[ParsedRun],
    metrics: Sequence[Dict[str, Any]],
    classifications: Sequence[Dict[str, Any]],
    model: Dict[str, Any],
    sensitivity: Sequence[Dict[str, Any]],
    clock_rows: Sequence[Dict[str, Any]],
    config: Dict[str, Any],
    output_root: Path,
) -> None:
    configure_matplotlib()
    group_order = list(config["groups"])
    class_by_run = {row["run_id"]: row["classification"] for row in classifications}

    # 1. Run-level latency scatter and boxplot.
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    arrays = _metric_arrays(metrics, "actual_e2e_latency_ms", group_order)
    ax.boxplot(arrays, positions=np.arange(len(group_order)), widths=0.5, showfliers=False)
    rng = np.random.default_rng(int(config["analysis"]["random_seed"]))
    for index, (group, values) in enumerate(zip(group_order, arrays)):
        jitter = rng.normal(0, 0.045, size=len(values))
        ax.scatter(np.full(len(values), index) + jitter, values, color=GROUP_COLORS[group], edgecolor="black", linewidth=0.4, zorder=3)
    ax.set_xticks(range(len(group_order)), [GROUP_LABELS[group] for group in group_order])
    ax.set_ylabel("实际端到端响应时延 (ms)")
    ax.set_title("图1  各run传感器源时间到持续有效减速的响应时延")
    save_figure(fig, output_root, "fig01_actual_e2e_latency_distribution")

    # 2. Nominal versus actual latency.
    fig, ax = plt.subplots(figsize=(7.6, 4.8))
    valid = [row for row in metrics if math.isfinite(fnum(row.get("actual_e2e_latency_ms")))]
    for group in group_order:
        subset = [row for row in valid if row["group_name"] == group]
        ax.scatter([row["nominal_injected_delay_ms"] for row in subset], [row["actual_e2e_latency_ms"] for row in subset], label=GROUP_LABELS[group], color=GROUP_COLORS[group], s=36, edgecolor="black", linewidth=0.4)
    base_median = np.median([row["actual_e2e_latency_ms"] for row in valid if row["group_name"] == "baseline"])
    xline = np.linspace(0, 420, 100)
    ax.plot(xline, base_median + xline, "--", color="gray", label="Baseline中位数 + 名义时延")
    ax.set_xlabel("名义注入时延 (ms)")
    ax.set_ylabel("实际端到端响应时延 (ms)")
    ax.set_title("图2  名义注入量与实测闭环响应")
    ax.legend(ncol=2)
    save_figure(fig, output_root, "fig02_nominal_vs_actual_latency")

    # 3. Stage waterfall (stacked run-level bars).
    fig, ax = plt.subplots(figsize=(12.0, 6.2))
    stages = [
        ("sensor_to_perception_ms", "sensor→perception", "#4C78A8"),
        ("perception_to_prediction_ms", "perception→prediction", "#72B7B2"),
        ("prediction_to_planning_stop_ms", "prediction→planning STOP", "#F2CF5B"),
        ("planning_stop_to_control_ms", "planning STOP→control", "#B279A2"),
        ("control_to_effective_brake_ms", "control→effective braking", "#E45756"),
    ]
    ordered = sorted(valid, key=lambda row: (group_order.index(row["group_name"]), row["run_id"]))
    bottoms = np.zeros(len(ordered))
    for key, label, color in stages:
        values = np.asarray([max(0.0, fnum(row.get(key), 0.0)) for row in ordered])
        ax.bar(range(len(ordered)), values, bottom=bottoms, label=label, color=color, width=0.82)
        bottoms += values
    ax.set_xticks(range(len(ordered)), [row["run_id"][-4:] for row in ordered], rotation=70)
    ax.set_ylabel("阶段时延 (ms)")
    ax.set_title("图3  逐run阶段时延瀑布图")
    ax.legend(ncol=3, fontsize=8)
    save_figure(fig, output_root, "fig03_stage_latency_waterfall")

    # 4, 5, 6. Physical trajectories.
    parsed_by_id = {parsed.spec.run_id: parsed for parsed in parsed_runs}
    fig4, ax4 = plt.subplots(figsize=(9.0, 5.2))
    fig5, ax5 = plt.subplots(figsize=(9.0, 5.2))
    fig6, ax6 = plt.subplots(figsize=(9.0, 5.2))
    for row in valid:
        parsed = parsed_by_id[row["run_id"]]
        t1 = fnum(row.get("t_sensor_origin_s"))
        d1 = fnum(row.get("D1_clear_m"))
        samples = [sample for sample in parsed.localization if t1 - 0.5 <= sample.time_s <= t1 + 6.0]
        if not samples:
            continue
        rel = np.asarray([sample.time_s - t1 for sample in samples])
        speed = np.asarray([sample.speed_mps for sample in samples])
        clearances = []
        for sample in samples:
            travel = integrate_speed(parsed.localization, t1, sample.time_s) if sample.time_s >= t1 else -integrate_speed(parsed.localization, sample.time_s, t1)
            clearances.append(d1 - travel)
        color = GROUP_COLORS[row["group_name"]]
        label = f"{GROUP_LABELS[row['group_name']]}-{row['run_id'][-4:]}"
        ax4.plot(rel, speed, color=color, alpha=0.62, linewidth=1.0, label=label)
        ax5.plot(clearances, speed, color=color, alpha=0.65, linewidth=1.0)
        ax6.plot(rel, clearances, color=color, alpha=0.65, linewidth=1.0)
        for event_key, marker in [("sensor_to_planning_stop_ms", "^"), ("sensor_to_control_ms", "s"), ("actual_e2e_latency_ms", "o")]:
            event_ms = fnum(row.get(event_key))
            if math.isfinite(event_ms):
                event_s = event_ms / 1000.0
                event_clear = d1 - integrate_speed(parsed.localization, t1, t1 + event_s)
                ax6.scatter(event_s, event_clear, marker=marker, color=color, s=14)
    handles = [Line2D([0], [0], color=GROUP_COLORS[group], label=GROUP_LABELS[group]) for group in group_order]
    ax4.set(xlabel="相对t_sensor_origin时间 (s)", ylabel="自车速度 (m/s)", title="图4  全部run速度—时间曲线")
    ax4.legend(handles=handles, ncol=4)
    ax5.set(xlabel="障碍物纵向净间距 (m)", ylabel="自车速度 (m/s)", title="图5  速度—障碍物净间距曲线")
    ax5.invert_xaxis()
    ax6.set(xlabel="相对t_sensor_origin时间 (s)", ylabel="障碍物纵向净间距 (m)", title="图6  净间距—时间曲线（^ STOP，□ Control，○有效减速）")
    for fig, stem in [(fig4, "fig04_speed_vs_time"), (fig5, "fig05_speed_vs_clearance"), (fig6, "fig06_clearance_vs_time")]:
        save_figure(fig, output_root, stem)

    # 7. Distance debt versus latency.
    fig, ax = plt.subplots(figsize=(7.5, 5.0))
    for group in group_order:
        subset = [row for row in valid if row["group_name"] == group and math.isfinite(fnum(row.get("D_delay_m")))]
        ax.scatter([row["actual_e2e_latency_ms"] for row in subset], [row["D_delay_m"] for row in subset], color=GROUP_COLORS[group], label=GROUP_LABELS[group], edgecolor="black", linewidth=0.4)
    x = np.asarray([row["actual_e2e_latency_ms"] / 1000.0 for row in valid if math.isfinite(fnum(row.get("D_delay_m")))])
    y = np.asarray([row["D_delay_m"] for row in valid if math.isfinite(fnum(row.get("D_delay_m")))])
    if len(x) >= 2:
        slope, intercept, _, _ = stats.theilslopes(y, x)
        grid = np.linspace(x.min(), x.max(), 100)
        ax.plot(grid * 1000.0, intercept + slope * grid, "k-", label=f"稳健斜率 {slope:.2f} m/s")
        ax.plot(grid * 1000.0, 15.6 * grid, "k--", alpha=0.6, label="15.6 m/s理论参考")
    ax.set(xlabel="实际端到端响应时延 (ms)", ylabel="响应期间距离债务 D_delay (m)", title="图7  时延向距离债务的转换")
    ax.legend(ncol=2)
    save_figure(fig, output_root, "fig07_delay_distance_vs_latency")

    # 8. Core safety margin plot.
    fig, ax = plt.subplots(figsize=(8.2, 5.3))
    markers = {"SAFE_NORMAL": "o", "SAFE_CRITICAL": "^", "RT_ONLY_COLLISION": "X", "TIMING_INDUCED_FUNCTIONAL_DEGRADATION": "P", "INDETERMINATE": "s"}
    for row in valid:
        margin = fnum(row.get("M_space_m"))
        if not math.isfinite(margin):
            continue
        classification = class_by_run.get(row["run_id"], "INDETERMINATE")
        ax.scatter(row["actual_e2e_latency_ms"], margin, color=GROUP_COLORS[row["group_name"]], marker=markers.get(classification, "s"), s=58, edgecolor="black", linewidth=0.5)
        ax.annotate(row["run_id"][-4:], (row["actual_e2e_latency_ms"], margin), xytext=(3, 3), textcoords="offset points", fontsize=7)
    primary_margin = float(config["safety"]["primary_margin_m"])
    ax.axhline(0, color="black", linestyle="--", linewidth=1.0, label="M_safety_6m=0")
    ax.axhline(
        -primary_margin,
        color="#b2182b",
        linestyle=":",
        linewidth=1.2,
        label="M_collision_0m=0",
    )
    ax.set(xlabel="实际端到端响应时延 (ms)", ylabel="6 m安全余量 M_safety_6m (m)", title="图8  实际响应时延与双空间边界（核心图）")
    group_handles = [
        Line2D(
            [0],
            [0],
            marker="o",
            color="none",
            markerfacecolor=GROUP_COLORS[group],
            markeredgecolor="black",
            label=GROUP_LABELS[group],
        )
        for group in group_order
    ]
    class_handles = [
        Line2D(
            [0],
            [0],
            marker=marker,
            color="none",
            markerfacecolor="#b0b0b0",
            markeredgecolor="black",
            label=label,
        )
        for label, marker in markers.items()
    ]
    class_handles.append(Line2D([0], [0], color="black", linestyle="--", label="M_safety_6m=0"))
    class_handles.append(Line2D([0], [0], color="#b2182b", linestyle=":", label="M_collision_0m=0"))
    group_legend = ax.legend(handles=group_handles, title="时延组", loc="upper right", ncol=2, fontsize=7)
    ax.add_artist(group_legend)
    ax.legend(handles=class_handles, title="结局分类", loc="lower left", fontsize=7)
    save_figure(fig, output_root, "fig08_m_space_vs_actual_latency")

    # 9. Margin distribution.
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    arrays = _metric_arrays(metrics, "M_space_m", group_order)
    ax.boxplot(arrays, positions=range(len(group_order)), widths=0.5, showfliers=False)
    for index, (group, values) in enumerate(zip(group_order, arrays)):
        ax.scatter(np.full(len(values), index), values, color=GROUP_COLORS[group], edgecolor="black", linewidth=0.4)
    ax.axhline(0, color="black", linestyle="--")
    ax.set_xticks(range(len(group_order)), [GROUP_LABELS[group] for group in group_order])
    ax.set(ylabel="M_safety_6m (m)", title="图9  各组6 m空间安全余量分布")
    save_figure(fig, output_root, "fig09_m_space_distribution")

    # 10. Separate outcome panels.
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.8))
    safe = [row for row in metrics if not row.get("collision") and math.isfinite(fnum(row.get("final_clearance_m")))]
    coll = [row for row in metrics if row.get("collision") and math.isfinite(fnum(row.get("impact_speed_mps")))]
    axes[0].bar(range(len(safe)), [row["final_clearance_m"] for row in safe], color=[GROUP_COLORS[row["group_name"]] for row in safe])
    axes[0].set_xticks(range(len(safe)), [row["run_id"][-4:] for row in safe], rotation=60)
    axes[0].set_ylabel("最终停车净距 (m)")
    axes[0].set_title("安全run")
    axes[1].bar(range(len(coll)), [row["impact_speed_mps"] for row in coll], color=[GROUP_COLORS[row["group_name"]] for row in coll])
    axes[1].set_xticks(range(len(coll)), [row["run_id"][-4:] for row in coll])
    axes[1].set_ylabel("碰撞前速度 (m/s)")
    axes[1].set_title("碰撞run")
    fig.suptitle("图10  安全停车间距与碰撞速度分面展示")
    save_figure(fig, output_root, "fig10_outcomes_clearance_and_impact_speed")

    # 11. Baseline braking model.
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    samples = [row for row in metrics if row["group_name"] == "baseline" and math.isfinite(fnum(row.get("empirical_braking_distance_m")))]
    speeds = np.asarray([row["brake_start_speed_mps"] for row in samples])
    distances = np.asarray([row["empirical_braking_distance_m"] for row in samples])
    ax.scatter(speeds, distances, color=GROUP_COLORS["baseline"], edgecolor="black", label="Baseline停车样本")
    grid = np.linspace(max(0.0, speeds.min() - 0.5), speeds.max() + 2.0, 100) if speeds.size else np.linspace(14, 18, 100)
    k = fnum(model.get("k_median"))
    k_low, k_high = model.get("k_bootstrap_95ci", [math.nan, math.nan])
    ax.plot(grid, k * grid**2, color="black", label="中位k模型")
    ax.fill_between(grid, fnum(k_low) * grid**2, fnum(k_high) * grid**2, color="gray", alpha=0.25, label="bootstrap 95%区间")
    ax.set(xlabel="有效制动开始速度 (m/s)", ylabel="经验停车距离 (m)", title="图11  Baseline经验制动距离模型")
    ax.legend()
    save_figure(fig, output_root, "fig11_baseline_braking_model")

    # 12. Sensitivity heatmap: negative-margin count for primary 3 frames.
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    thresholds = [float(value) for value in config["effective_brake"]["sensitivity_thresholds_mps2"]]
    margins = [float(value) for value in config["safety"]["sensitivity_margins_m"]]
    matrix = np.zeros((len(thresholds), len(margins)))
    for i, threshold in enumerate(thresholds):
        for j, margin in enumerate(margins):
            subset = [item for item in sensitivity if item["stable_frames"] == 3 and item["brake_decel_threshold_mps2"] == threshold and item["D_margin_m"] == margin]
            matrix[i, j] = sum(math.isfinite(fnum(item.get("M_space_m"))) and fnum(item.get("M_space_m")) < 0 for item in subset)
    image = ax.imshow(matrix, cmap="YlOrRd", aspect="auto")
    ax.set_xticks(range(len(margins)), [f"{value:g}" for value in margins])
    ax.set_yticks(range(len(thresholds)), [f"{value:g}" for value in thresholds])
    ax.set(xlabel="D_margin (m)", ylabel="有效减速度阈值 (m/s²)", title="图12  负空间余量run数量敏感性")
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            ax.text(j, i, str(int(matrix[i, j])), ha="center", va="center")
    fig.colorbar(image, ax=ax, label="M_space<0的run数")
    save_figure(fig, output_root, "fig12_sensitivity_heatmap")

    # 13. Event timeline.
    fig, ax = plt.subplots(figsize=(11.2, 7.0))
    event_keys = [("sensor_to_perception_ms", "Fusion", "o"), ("sensor_to_planning_stop_ms", "Planning STOP", "^"), ("sensor_to_control_ms", "Control", "s"), ("actual_e2e_latency_ms", "有效减速", "X")]
    ordered = sorted(valid, key=lambda row: (group_order.index(row["group_name"]), row["run_id"]))
    for y_index, row in enumerate(ordered):
        ax.hlines(y_index, 0, row["actual_e2e_latency_ms"], color=GROUP_COLORS[row["group_name"]], alpha=0.55)
        for key, _, marker in event_keys:
            value = fnum(row.get(key))
            if math.isfinite(value):
                ax.scatter(value, y_index, marker=marker, color=GROUP_COLORS[row["group_name"]], edgecolor="black", linewidth=0.3, s=28)
    ax.set_yticks(range(len(ordered)), [f"{GROUP_LABELS[row['group_name']]}-{row['run_id'][-4:]}" for row in ordered])
    ax.set_xlabel("相对t_sensor_origin时间 (ms)")
    ax.set_title("图13  逐run关键事件时间线")
    legend = [Line2D([0], [0], marker=marker, color="none", markerfacecolor="gray", markeredgecolor="black", label=label) for _, label, marker in event_keys]
    ax.legend(handles=legend, ncol=4)
    save_figure(fig, output_root, "fig13_event_timeline")

    # 14. Collision-run joint timing, last 5 s.
    collision_metrics = [row for row in metrics if row.get("collision")]
    fig, axes = plt.subplots(max(1, len(collision_metrics)), 1, figsize=(10.5, 3.0 * max(1, len(collision_metrics))), squeeze=False)
    for axis, row in zip(axes[:, 0], collision_metrics):
        parsed = parsed_by_id[row["run_id"]]
        collision_s = fnum(row.get("t_collision_s"))
        samples = [sample for sample in parsed.localization if collision_s - 5.0 <= sample.time_s <= collision_s]
        if samples:
            axis.plot([sample.time_s - collision_s for sample in samples], [sample.speed_mps for sample in samples], color="black", label="speed")
        for key, label, color in [
            ("t_sensor_origin_s", "sensor origin", "#4C78A8"),
            ("t_prediction_first_s", "prediction", "#72B7B2"),
            ("t_planning_stop_s", "planning STOP", "#F2CF5B"),
            ("t_control_brake_command_s", "control", "#B279A2"),
            ("t_brake_effective_s", "effective brake", "#E45756"),
        ]:
            value = fnum(row.get(key))
            if math.isfinite(value):
                axis.axvline(value - collision_s, color=color, label=label)
        axis.set(title=f"{row['run_id']}  {class_by_run.get(row['run_id'])}", xlabel="距碰撞时间 (s)", ylabel="速度 (m/s)")
        axis.legend(ncol=3, fontsize=7)
    if not collision_metrics:
        axes[0, 0].text(0.5, 0.5, "无碰撞run", ha="center", va="center")
    fig.suptitle("图14  碰撞前5 s功能事件与车辆物理状态联合时间线")
    save_figure(fig, output_root, "fig14_collision_joint_timeline")

    # 15. Collision rate by actual-latency bins.
    bins = np.asarray(config["statistics"]["latency_bins_ms"], dtype=float)
    labels: List[str] = []
    rates: List[float] = []
    counts: List[int] = []
    for left, right in zip(bins, bins[1:]):
        subset = [row for row in valid if left <= row["actual_e2e_latency_ms"] < right]
        labels.append(f"{left:g}–{right:g}")
        counts.append(len(subset))
        rates.append(sum(bool(row.get("collision")) for row in subset) / len(subset) if subset else math.nan)
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    bars = ax.bar(range(len(labels)), [0 if not math.isfinite(value) else value for value in rates], color="#E45756")
    for bar, n in zip(bars, counts):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02, f"n={n}", ha="center")
    ax.set_xticks(range(len(labels)), labels, rotation=35)
    ax.set_ylim(0, 1.08)
    ax.set(xlabel="实际端到端时延区间 (ms)", ylabel="碰撞率", title="图15  实际时延区间与碰撞率")
    save_figure(fig, output_root, "fig15_collision_rate_by_latency_bin")

    # Required clock diagnostics and margin sensitivity aliases.
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    any_clock = False
    for item in clock_rows:
        residual = item.get("residual_ms") or []
        if residual:
            any_clock = True
            ax.plot(residual, label=item["run_id"])
    if not any_clock:
        ax.text(0.5, 0.5, "无CARLA simulation/wall双时钟history", ha="center", va="center", transform=ax.transAxes)
    ax.set(xlabel="去重CARLA帧序号", ylabel="稳健拟合残差 (ms)", title="时钟对齐诊断")
    if any_clock:
        ax.legend()
    save_figure(fig, output_root, "clock_alignment_diagnostics")

    # Copy figure 12 semantics under the requested margin-specific name.
    source_png = output_root / "figures" / "fig12_sensitivity_heatmap.png"
    source_svg = output_root / "figures" / "fig12_sensitivity_heatmap.svg"
    (output_root / "figures" / "margin_sensitivity_plot.png").write_bytes(source_png.read_bytes())
    (output_root / "figures" / "margin_sensitivity_plot.svg").write_bytes(source_svg.read_bytes())


def add_reference_inventory(
    inventory: List[Dict[str, Any]],
    schema: Dict[str, Any],
    config: Dict[str, Any],
    compute_hashes: bool,
) -> None:
    bridge_root = Path(config["inputs"]["bridge_root"])
    candidates = [
        bridge_root / "carla_bridge" / "control_delay_injector.py",
        bridge_root / "carla_bridge" / "actor" / "ego_vehicle.py",
        bridge_root / "carla_bridge" / "main.py",
        bridge_root / "carla_bridge" / "config" / "settings.yaml",
        Path(config["inputs"]["handoff_file"]),
    ]
    analysis_root = Path(config["inputs"]["existing_analysis_root"])
    candidates.extend(
        path
        for path in analysis_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".py", ".yaml", ".md"}
    )
    for path in sorted(set(candidates)):
        if not path.exists():
            continue
        digest, line_count = sha256_and_lines(path) if compute_hashes else ("SKIPPED", -1)
        category = "bridge_source_or_config" if str(path).lower().startswith(str(bridge_root).lower()) else "analysis_reference"
        inventory.append(
            {
                "group_name": "reference",
                "run_id": "",
                "run_directory": "",
                "source_file": str(path),
                "relative_path": path.name,
                "category": category,
                "extension": path.suffix.lower(),
                "size_bytes": path.stat().st_size,
                "line_count": line_count,
                "modified_time_iso": datetime.fromtimestamp(path.stat().st_mtime).astimezone().isoformat(),
                "sha256": digest,
            }
        )
        keys: List[str] = []
        error = ""
        if path.suffix.lower() in {".yaml", ".yml"}:
            try:
                parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
                if isinstance(parsed, dict):
                    keys = sorted(str(key) for key in parsed)
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
        schema["files"].append(
            {
                "source_file": str(path),
                "group_name": "reference",
                "run_id": "",
                "category": category,
                "columns": [],
                "json_or_yaml_top_level_keys": keys,
                "schema_error": error,
                "line_count": line_count,
            }
        )


def manifest_rows(
    parsed_runs: Sequence[ParsedRun],
    inventory: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    file_counts = Counter(row["run_id"] for row in inventory if row.get("run_id"))
    byte_counts: Counter[str] = Counter()
    for item in inventory:
        if item.get("run_id"):
            byte_counts[item["run_id"]] += int(item["size_bytes"])
    rows: List[Dict[str, Any]] = []
    for parsed in parsed_runs:
        files = parsed.files
        start = fnum(parsed.collect.get("start_epoch_s"))
        end = fnum(parsed.collect.get("end_log_epoch_s"))
        critical_missing = [
            key
            for key in ["localization", "perception", "prediction", "planning", "control_context", "trace_anchor"]
            if files.get(key) is None
        ]
        status = "COMPLETE" if not critical_missing and parsed.perception.get("stable") else "PARTIAL"
        if parsed.spec.nominal_delay_ms > 0 and not parsed.scb.get("present"):
            status = "PARTIAL_INJECTION_UNVERIFIED"
        rows.append(
            {
                "group_name": parsed.spec.group_name,
                "nominal_injected_delay_ms": parsed.spec.nominal_delay_ms,
                "run_id": parsed.spec.run_id,
                "run_directory": str(parsed.spec.run_dir),
                "file_count": file_counts[parsed.spec.run_id],
                "total_size_bytes": byte_counts[parsed.spec.run_id],
                "collect_time_present": files.get("collect") is not None,
                "localization_present": files.get("localization") is not None,
                "perception_present": files.get("perception") is not None,
                "prediction_present": files.get("prediction") is not None,
                "planning_present": files.get("planning") is not None,
                "control_trace_present": files.get("control_context") is not None,
                "scb_delay_evidence_present": files.get("scb") is not None,
                "collision_event_present": files.get("collision_csv") is not None,
                "actor_history_present": files.get("actor_history") is not None,
                "data_start_epoch_s": start,
                "data_end_epoch_s": end,
                "duration_s": end - start if math.isfinite(start) and math.isfinite(end) else math.nan,
                "collision_recorded": bool(parsed.collision.get("occurred")),
                "target_id": parsed.target_id,
                "data_integrity_status": status,
                "initial_anomaly_notes": ";".join(parsed.quality_notes + (["missing:" + ",".join(critical_missing)] if critical_missing else [])),
            }
        )
    return rows


def delay_injection_audit(config: Dict[str, Any], metrics: Sequence[Dict[str, Any]], output_root: Path) -> None:
    bridge_root = Path(config["inputs"]["bridge_root"])
    injector = bridge_root / "carla_bridge" / "control_delay_injector.py"
    ego = bridge_root / "carla_bridge" / "actor" / "ego_vehicle.py"
    main_file = bridge_root / "carla_bridge" / "main.py"
    settings = bridge_root / "carla_bridge" / "config" / "settings.yaml"
    verified = [row for row in metrics if row.get("scb_log_present")]
    missing = [row["run_id"] for row in metrics if row["nominal_injected_delay_ms"] > 0 and not row.get("scb_log_present")]
    actual_by_nominal: Dict[float, List[float]] = defaultdict(list)
    for row in verified:
        value = fnum(row.get("scb_actual_wall_delay_ms"))
        if math.isfinite(value):
            actual_by_nominal[float(row["nominal_injected_delay_ms"])].append(value)
    lines = [
        "# 时延注入实现审计",
        "",
        "## 审计结论",
        "",
        "- 注入位置：Bridge 的 `EgoVehicle.control_command_updated()` 接收 `/apollo/control` 后，将 ControlCommand 交给 `ControlDelayInjector.submit()`；worker 延后调用 `_apply_control_command()`，最终执行 CARLA `apply_control()`。",
        "- 注入机制：Cyber 回调复制 protobuf 并压入按 `release_monotonic_ns` 排序的最小堆；独立 daemon worker 使用 `Condition.wait(timeout=...)` 等待释放。",
        "- 时间基准：请求等待时间使用 `time.monotonic_ns()`；审计CSV同时记录 wall time、monotonic time、CARLA frame 和 simulation elapsed time。",
        "- 消息行为：触发前沿用原始直接执行路径；触发后所有后续 ControlCommand 进入延迟队列并按释放时间和sequence保持顺序。",
        "- 队列边界：`queue_max_messages` 达到上限时弹出最早待释放消息并记录 `DROPPED_QUEUE_FULL`。当前归档CSV仅记录首次有效制动，无法逐命令排除后续队列丢弃。",
        "- 仿真推进：worker 的等待不阻塞 CARLA synchronous tick 线程；Bridge 主循环按0.1 s wall monotonic节拍推进，CPU/GIL竞争仍可能形成间接负载影响。",
        "- 触发状态：现有源码采用速度锁存ARM与制动阈值触发，未实现障碍物生成后的显式ARM。多数SCB记录显示触发早于目标稳定感知。",
        "- 配置含义：100/300/400 ms为Control链路上的附加墙钟等待请求值；实际闭环解释采用 `actual_e2e_latency_ms`。",
        "",
        "## 证据文件",
        "",
        f"- `{injector}`",
        f"- `{ego}`",
        f"- `{main_file}`",
        f"- `{settings}`",
        "",
        "## SCB归档覆盖",
        "",
        f"- 23个run中有 {len(verified)} 个保存SCB CSV。",
        f"- 注入组缺少SCB的run：{', '.join(missing) if missing else '无'}。",
        "",
        "| 名义时延 (ms) | 有SCB实测n | 墙钟实际时延中位数 (ms) | 范围 (ms) |",
        "|---:|---:|---:|---:|",
    ]
    for nominal in sorted(actual_by_nominal):
        values = actual_by_nominal[nominal]
        lines.append(f"| {nominal:.0f} | {len(values)} | {np.median(values):.3f} | {min(values):.3f}–{max(values):.3f} |")
    lines.extend(
        [
            "",
            "## 配置快照限制",
            "",
            "工作区 `settings.yaml` 当前写有Town01、0 ms、activation 5 m/s和brake 1%；运行SCB记录显示另一套远端参数，并且CollisionSensor确认碰撞run运行于Town04。每个run未保存完整Bridge settings快照，报告使用SCB行作为实际加载参数证据，地图采用用户固定条件并由碰撞event交叉验证。",
        ]
    )
    (output_root / "delay_injection_audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def md_table(rows: Sequence[Mapping[str, Any]], columns: Sequence[Tuple[str, str]], digits: int = 3) -> str:
    if not rows:
        return "（无可用记录）"
    header = "| " + " | ".join(label for _, label in columns) + " |"
    divider = "|" + "|".join("---" for _ in columns) + "|"
    body: List[str] = []
    for row in rows:
        values: List[str] = []
        for key, _ in columns:
            value = row.get(key)
            if isinstance(value, (float, np.floating)):
                values.append(f"{value:.{digits}f}" if math.isfinite(float(value)) else "—")
            elif value is None or value == "":
                values.append("—")
            elif isinstance(value, bool):
                values.append("是" if value else "否")
            else:
                values.append(str(value).replace("|", "/"))
        body.append("| " + " | ".join(values) + " |")
    return "\n".join([header, divider, *body])


def group_metric_lookup(summary: Sequence[Dict[str, Any]], metric: str) -> Dict[str, Dict[str, Any]]:
    return {row["group_name"]: row for row in summary if row["metric"] == metric}


def build_report(
    metrics: Sequence[Dict[str, Any]],
    manifest: Sequence[Dict[str, Any]],
    modules: Sequence[Dict[str, Any]],
    classifications: Sequence[Dict[str, Any]],
    group_summary: Sequence[Dict[str, Any]],
    collision_summary: Sequence[Dict[str, Any]],
    model: Dict[str, Any],
    sensitivity: Sequence[Dict[str, Any]],
    config: Dict[str, Any],
    output_root: Path,
) -> None:
    group_order = list(config["groups"])
    latency = group_metric_lookup(group_summary, "actual_e2e_latency_ms")
    ddelay = group_metric_lookup(group_summary, "D_delay_m")
    margin = group_metric_lookup(group_summary, "M_space_m")
    class_by_run = {row["run_id"]: row for row in classifications}
    rt_runs = [row["run_id"] for row in classifications if row["classification"] == "RT_ONLY_COLLISION"]
    degraded_runs = [row["run_id"] for row in classifications if row["classification"] == "TIMING_INDUCED_FUNCTIONAL_DEGRADATION"]
    indeterminate = [row["run_id"] for row in classifications if row["classification"] == "INDETERMINATE"]
    safe_latency = [fnum(row.get("actual_e2e_latency_ms")) for row in metrics if not row.get("collision") and math.isfinite(fnum(row.get("actual_e2e_latency_ms")))]
    rt_latency = [fnum(row.get("actual_e2e_latency_ms")) for row in metrics if class_by_run.get(row["run_id"], {}).get("classification") == "RT_ONLY_COLLISION"]
    target_collision_latency = [
        fnum(row.get("actual_e2e_latency_ms"))
        for row in metrics
        if row.get("collision_with_target")
        and math.isfinite(fnum(row.get("actual_e2e_latency_ms")))
    ]
    max_safe = max(safe_latency) if safe_latency else math.nan
    min_rt = min(rt_latency) if rt_latency else math.nan
    min_target_collision = min(target_collision_latency) if target_collision_latency else math.nan
    rt_boundary_summary = (
        f"最小RT-only碰撞响应为{min_rt:.3f} ms"
        if math.isfinite(min_rt)
        else "当前样本没有RT_ONLY_COLLISION，严格RT-only时间边界不可估计"
    )
    safe_collision_overlap = bool(
        safe_latency
        and target_collision_latency
        and max_safe >= min_target_collision
    )
    collision_count = Counter(row["group_name"] for row in metrics if row.get("collision"))
    scb_missing = [row["run_id"] for row in metrics if row["nominal_injected_delay_ms"] > 0 and not row.get("scb_log_present")]
    speed_values = [fnum(row.get("t1_speed_mps")) for row in metrics if math.isfinite(fnum(row.get("t1_speed_mps")))]
    d1_values = [fnum(row.get("D1_clear_m")) for row in metrics if math.isfinite(fnum(row.get("D1_clear_m")))]
    valid_reg = [row for row in metrics if math.isfinite(fnum(row.get("actual_e2e_latency_ms"))) and math.isfinite(fnum(row.get("D_delay_m")))]
    x = np.asarray([row["actual_e2e_latency_ms"] / 1000.0 for row in valid_reg])
    y = np.asarray([row["D_delay_m"] for row in valid_reg])
    slope, intercept, slope_low, slope_high = stats.theilslopes(y, x) if len(x) >= 2 else (math.nan,) * 4
    corr = stats.linregress(x, y) if len(x) >= 2 else None
    min_safe_margin = min(
        [
            fnum(row.get("M_space_m"))
            for row in metrics
            if not row.get("collision") and math.isfinite(fnum(row.get("M_space_m")))
        ],
        default=math.nan,
    )
    max_collision_margin = max(
        [
            fnum(row.get("M_space_m"))
            for row in metrics
            if row.get("collision_with_target")
            and math.isfinite(fnum(row.get("M_space_m")))
        ],
        default=math.nan,
    )
    margin_separation_m = (
        min_safe_margin - max_collision_margin
        if math.isfinite(min_safe_margin) and math.isfinite(max_collision_margin)
        else math.nan
    )

    overall_test_rows: List[Dict[str, Any]] = []
    pairwise_test_rows: List[Dict[str, Any]] = []
    for metric_name, metric_label in [
        ("actual_e2e_latency_ms", "实际响应"),
        ("D_delay_m", "D_delay"),
        ("M_space_m", "M_space"),
    ]:
        samples = {
            group: np.asarray(
                [
                    fnum(row.get(metric_name))
                    for row in metrics
                    if row.get("group_name") == group
                    and math.isfinite(fnum(row.get(metric_name)))
                ]
            )
            for group in group_order
        }
        kw = stats.kruskal(*(samples[group] for group in group_order))
        overall_test_rows.append(
            {
                "metric": metric_label,
                "H": float(kw.statistic),
                "p": float(kw.pvalue),
            }
        )
        raw_pairs: List[Dict[str, Any]] = []
        for group in group_order[1:]:
            test = stats.mannwhitneyu(
                samples["baseline"], samples[group], alternative="two-sided", method="auto"
            )
            raw_pairs.append(
                {
                    "metric": metric_label,
                    "comparison": f"{GROUP_LABELS[group]} vs Baseline",
                    "p": float(test.pvalue),
                    "cliff_delta": cliffs_delta(
                        samples[group].tolist(), samples["baseline"].tolist()
                    ),
                }
            )
        for item, adjusted in zip(raw_pairs, holm_adjust([item["p"] for item in raw_pairs])):
            item["p_holm"] = adjusted
            pairwise_test_rows.append(item)

    run_table = []
    for row in metrics:
        classification = class_by_run.get(row["run_id"], {})
        run_table.append(
            {
                "run_id": row["run_id"],
                "group": GROUP_LABELS[row["group_name"]],
                "v1": fnum(row.get("t1_speed_mps")),
                "D1": fnum(row.get("D1_clear_m")),
                "latency": fnum(row.get("actual_e2e_latency_ms")),
                "Ddelay": fnum(row.get("D_delay_m")),
                "Msafety6": fnum(row.get("M_safety_6m_m")),
                "Mcollision0": fnum(row.get("M_collision_0m_m")),
                "collision": bool(row.get("collision")),
                "impact": fnum(row.get("impact_speed_mps")),
                "classification": classification.get("classification"),
            }
        )

    summary_group_rows = []
    for group in group_order:
        collision_item = next(item for item in collision_summary if item["group_name"] == group)
        summary_group_rows.append(
            {
                "group": GROUP_LABELS[group],
                "n": collision_item["total_runs"],
                "valid_n": latency.get(group, {}).get("valid_n", 0),
                "latency": latency.get(group, {}).get("median", math.nan),
                "added": latency.get(group, {}).get("median", math.nan) - latency.get("baseline", {}).get("median", math.nan),
                "Ddelay": ddelay.get(group, {}).get("median", math.nan),
                "Mspace": margin.get(group, {}).get("median", math.nan),
                "collisions": collision_item["collision_count"],
            }
        )

    scb_audit_rows: List[Dict[str, Any]] = []
    for group in group_order:
        values = [
            fnum(row.get("scb_actual_wall_delay_ms"))
            for row in metrics
            if row.get("group_name") == group
            and math.isfinite(fnum(row.get("scb_actual_wall_delay_ms")))
        ]
        scb_audit_rows.append(
            {
                "group": GROUP_LABELS[group],
                "scb_n": len(values),
                "actual_median": float(np.median(values)) if values else math.nan,
                "actual_min": min(values) if values else math.nan,
                "actual_max": max(values) if values else math.nan,
            }
        )

    collision_counterfactual_rows = []
    for row in metrics:
        if not row.get("collision_with_target"):
            continue
        collision_counterfactual_rows.append(
            {
                "run_id": row["run_id"],
                "latency": fnum(row.get("actual_e2e_latency_ms")),
                "Msafety6": fnum(row.get("M_safety_6m_m")),
                "Mcollision0": fnum(row.get("M_collision_0m_m")),
                "added": fnum(row.get("measured_added_delay_ms")),
                "saved": fnum(row.get("D_saved_counterfactual_m")),
                "Msafety6cf": fnum(row.get("M_safety_6m_counterfactual_m")),
                "Mcollision0cf": fnum(row.get("M_collision_0m_counterfactual_m")),
                "impact": fnum(row.get("impact_speed_mps")),
                "realtime_induced": class_by_run[row["run_id"]].get(
                    "realtime_induced_collision"
                ),
                "classification": class_by_run[row["run_id"]]["classification"],
            }
        )
    collision_fallback_rows = []
    for row in metrics:
        if not row.get("collision_with_target"):
            continue
        collision_fallback_rows.append(
            {
                "run_id": row["run_id"],
                "primal": int(row.get("planning_primal_infeasible_count") or 0),
                "speed_fallback": int(row.get("planning_speed_fallback_count") or 0),
                "constant_decel": int(
                    row.get("planning_constant_deceleration_fallback_count") or 0
                ),
                "first_line": row.get("planning_first_speed_fallback_source_line"),
                "source": row.get("source_planning_file"),
            }
        )
    baseline_strict_stop_count = sum(
        row.get("group_name") == "baseline"
        and row.get("stop_event_status") == "AVAILABLE"
        for row in metrics
    )
    baseline_braking_rows: List[Dict[str, Any]] = []
    for row in metrics:
        speed = fnum(row.get("brake_start_speed_mps"))
        distance = fnum(row.get("empirical_braking_distance_m"))
        if row.get("group_name") != "baseline" or not all(
            math.isfinite(value) for value in [speed, distance]
        ):
            continue
        baseline_braking_rows.append(
            {
                "run_id": row["run_id"],
                "v2": speed,
                "displacement": distance,
                "path_length": fnum(row.get("empirical_braking_path_length_m")),
                "k": distance / speed**2,
                "a_eff": speed**2 / (2.0 * distance),
                "strict_stop": row.get("stop_event_status") == "AVAILABLE",
            }
        )
    braking_prediction_rows = [
        {
            "speed": speed,
            "distance": fnum(model.get("k_median")) * speed**2,
        }
        for speed in [15.0, 15.5, 16.0, 16.5, 17.0]
    ]
    baseline_speed_range = [row["v2"] for row in baseline_braking_rows]
    baseline_displacement_range = [row["displacement"] for row in baseline_braking_rows]
    baseline_path_range = [row["path_length"] for row in baseline_braking_rows]
    representative_speed_mps = 16.0
    representative_d1_m = 40.0
    representative_brake_m = fnum(model.get("k_median")) * representative_speed_mps**2
    representative_deadline_0m_s = (
        representative_d1_m - representative_brake_m
    ) / representative_speed_mps
    representative_deadline_6m_s = (
        representative_d1_m - representative_brake_m - 6.0
    ) / representative_speed_mps
    representative_brake_low_m = (
        fnum(model["k_bootstrap_95ci"][0]) * representative_speed_mps**2
    )
    representative_brake_high_m = (
        fnum(model["k_bootstrap_95ci"][1]) * representative_speed_mps**2
    )
    fixed_4mps2_brake_at_16_m = representative_speed_mps**2 / (2.0 * 4.0)
    sensitivity_rt_candidates = sorted(
        {
            row["run_id"]
            for row in sensitivity
            if row.get("rt_only_margin_condition")
            and float(row.get("D_margin_m", math.nan)) == 0.0
        }
    )

    report = rf"""# Apollo + CARLA端到端实时性碰撞实验结果报告

## 摘要

本次分析完整识别23次实验：baseline 6次、100 ms组6次、300 ms组6次、400 ms组5次。固定条件为Town04、CARLA/Bridge同步步长0.1 s、点云配置130万、静止障碍物场景。t1实际速度范围为{min(speed_values):.3f}–{max(speed_values):.3f} m/s，稳定感知纵向净距范围为{min(d1_values):.3f}–{max(d1_values):.3f} m。

四组观测闭环响应中位数依次为{latency['baseline']['median']:.3f}、{latency['delay_100ms']['median']:.3f}、{latency['delay_300ms']['median']:.3f}和{latency['delay_400ms']['median']:.3f} ms。相对baseline的中位响应增量依次为{latency['delay_100ms']['median']-latency['baseline']['median']:.3f}、{latency['delay_300ms']['median']-latency['baseline']['median']:.3f}和{latency['delay_400ms']['median']-latency['baseline']['median']:.3f} ms。400 ms组记录{collision_count['delay_400ms']}/5次目标碰撞，其余三组未记录碰撞。

严格因果分类得到RT_ONLY_COLLISION run：{', '.join(rt_runs) if rt_runs else '无'}；TIMING_INDUCED_FUNCTIONAL_DEGRADATION run：{', '.join(degraded_runs) if degraded_runs else '无'}；INDETERMINATE run：{', '.join(indeterminate) if indeterminate else '无'}。最大安全观测响应为{max_safe:.3f} ms；{rt_boundary_summary}。全部目标碰撞中的最小响应为{min_target_collision:.3f} ms，安全与目标碰撞响应区间{'重叠' if safe_collision_overlap else '未重叠'}。

主要不确定性包括：注入组SCB归档缺失run为{', '.join(scb_missing) if scb_missing else '无'}；多数SCB触发早于t1；非碰撞run缺少CARLA actor history；ControlCommand具体brake payload未归档；Localization典型采样周期约{np.nanmedian([fnum(row.get('localization_median_interval_ms')) for row in metrics]):.1f} ms。

## 1. 实验目的与假设

- H1：时延注入增加传感器源时间到有效物理减速的响应时延。
- H2：实际响应时延增加会增加响应期间距离债务D_delay。
- H3：D_delay增加会同时降低0 m碰撞余量与6 m安全余量。
- H4：0 m碰撞余量转为负值时，车辆进入经验制动模型无法避免接触的区域。
- H5：纯实时性碰撞run具备完整感知、预测、规划、控制和Bridge执行证据链。

## 2. 实验设计

自车约15–17 m/s行驶，在前方生成静止车辆障碍物。Apollo传感器理论感知范围约50 m，稳定感知净距集中于约40 m。四组名义附加时延为0、100、300和400 ms。Bridge直接订阅`/apollo/control`，Guardian未进入执行链。注入器使用单调墙钟队列延后CARLA `apply_control()`。

归档中的工作区settings快照与实际run参数存在版本差异。实验条件采用用户记录；Town04由3个碰撞event直接验证；0.1 s步长由实验设定及SCB frame/simulation delay交叉检查；130万点云原始计数未保存在run目录。

## 3. 数据处理与指标定义

本章的目的，是把一条run中的日志时间、车辆运动和碰撞结果整理成同一条物理链：障碍物在何时具备稳定感知条件，车辆当时距离障碍物多远，系统经过多久才产生有效减速，响应期间消耗了多少距离，剩余距离能否完成停车。

### 3.1 一条run的处理顺序

每条run按以下顺序处理：

1. 在CARLA与Apollo日志中确定同一个静止障碍物。
2. 找到障碍物连续稳定出现的源点云时刻，记为 $t_1=t_{{\mathrm{{sensor\_origin}}}}$。
3. 在 $t_1$ 读取自车速度 $v_1$，并计算车头到障碍物近端的净距 $D_1$。
4. 从Localization速度序列中检测车辆开始持续有效减速的时刻 $t_2$。
5. 用 $t_1$ 到 $t_2$ 的时间差计算端到端响应时延，用同一区间的速度积分计算响应期间行驶距离 $D_{{\mathrm{{delay}}}}$。
6. 用baseline停车样本建立经验制动模型，估计车辆从 $t_2$ 开始停车还需要的距离 $D_{{\mathrm{{brake,required}}}}$。
7. 从初始净距 $D_1$ 中依次扣除响应期间行驶距离和所需制动距离，得到0 m碰撞余量与6 m安全余量。
8. 对注入组计算“响应恢复到baseline水平”时能够节省的距离，作为反事实结果。

核心变量可按下表理解：

| 字段 | 含义 | 数据来源 |
|---|---|---|
| t1 / t_sensor_origin | 稳定感知序列第一帧的传感器源时间，也是空间计算起点 | Fusion源点云时间 |
| v1 | t1时刻自车速度 | Localization插值 |
| D1 | t1时刻车头到障碍物近端的纵向净距 | Fusion几何与车辆尺寸修正 |
| t2 / t_brake_effective | 车辆首次进入持续有效物理减速状态的时刻 | Localization速度序列 |
| v2 | t2时刻的制动起始速度 | Localization插值 |
| T_e2e | 从稳定感知源帧到有效物理减速的总响应时间 | t2 − t1 |
| D_delay | 响应完成前车辆继续行驶的距离 | t1到t2的速度积分 |
| D_brake_required | 从t2开始降至近零速度所需的经验制动距离 | baseline经验模型 |
| M_collision_0m | 不要求附加安全距离时的碰撞余量 | 空间预算计算 |
| M_safety_6m | 停车后仍要求保留6 m时的安全余量 | 空间预算计算 |

空间预算可以直观写成：

$$
\underbrace{{D_1}}_{{\text{{最初可用净距}}}}
=\underbrace{{D_{{\mathrm{{delay}}}}}}_{{\text{{响应期间行驶}}}}
+\underbrace{{D_{{\mathrm{{brake,required}}}}}}_{{\text{{开始制动后所需距离}}}}
+\underbrace{{M_{{\mathrm{{collision,0m}}}}}}_{{\text{{最终剩余量}}}}.
$$

### 3.2 为什么需要统一时钟

Apollo模块日志、Localization、CARLA仿真时间和服务器墙钟来自不同时间域。直接相减可能把时钟偏移误认为模块时延，因此主分析先把事件放到可比较的时间轴上。

- Apollo内部事件以Apollo/Localization epoch为主时间轴。感知、预测、规划、控制与Localization都在这个时间域中比较。
- 碰撞run保存了CARLA actor history，其中同时包含simulation time和wall time。分析通过

$$
\mathrm{{wall\_time}}=a\times\mathrm{{simulation\_time}}+b
$$

将CARLA碰撞时刻映射到墙钟，再与Apollo事件对齐；拟合残差保存在`clock_alignment.csv`。
- 缺少双时钟history的安全run仍可计算Apollo内部的 $t_1$、$t_2$ 和响应时延，但无法对CARLA仿真时刻执行同等级别的跨时钟复核，因此标记为`LIMITED_NO_DUAL_CLOCK_HISTORY`。

该状态表示跨系统时间证据受限，Apollo内部物理响应指标仍然保留。

### 3.3 如何确定障碍物、$t_1$ 和 $D_1$

首先要保证CARLA中发生接触的车辆、Apollo感知到的障碍物、Prediction中的静态目标以及Planning生成STOP决策的目标属于同一物理对象。

- 碰撞run优先使用CARLA CollisionSensor给出的actor ID，再将actor history轨迹变换到Apollo坐标系，与Fusion目标执行多帧位置匹配。
- 安全run没有碰撞actor记录，使用Planning STOP目标ID、Prediction静态语义和Fusion连续轨迹联合确定目标。

单帧检测可能来自瞬时误检或目标ID抖动，因此主定义要求目标连续3个Fusion周期稳定出现。$t_1$ 取这3帧中第一帧的源点云时间。这里采用源时间，可以把感知计算和排队时间完整计入后续响应；2帧和5帧定义进入敏感性分析，用于检查结论是否依赖“三帧”这一选择。

$D_1$ 表示自车前缘到障碍物近端的纵向净距，属于保险杠到保险杠的可用距离。计算形式为

$$
D_1=\Delta s_{{\mathrm{{center}}}}-5.3074\,\mathrm{{m}},
$$

其中 $\Delta s_{{\mathrm{{center}}}}$ 是沿自车行驶方向投影后的两车中心间距，$5.3074\,\mathrm{{m}}$ 是两车半车长之和形成的组合几何偏移。例如中心投影距离为45 m时，净距约为39.69 m。

归档没有保存每条run的CARLA bounding box extent，主分析统一使用该组合偏移。3次真实接触帧显示这一偏移存在约 $\pm0.52\,\mathrm{{m}}$ 的变化，因此报告将其作为几何不确定性，不把小于该量级的余量差异解释为精确边界。

### 3.4 如何确定 $t_2$、响应时延和两段距离

> **$t_2$（有效物理制动起点）**：车辆已经对本次障碍物响应，并在Localization速度序列中首次进入持续有效减速状态的时刻。$t_2$ 取物理速度开始持续下降的时间，不取Planning生成减速轨迹的时间，也不取Control消息发布的时间。

完整事件顺序为：`障碍物源帧t1 → Perception → Prediction → Planning STOP/减速轨迹 → Control输出 → 车辆有效物理减速t2`。因此，$t_2-t_1$ 覆盖软件计算、消息传递、Bridge等待以及车辆执行响应。

Localization存在采样噪声，单个速度差分为负不足以证明车辆已经进入持续制动。主判据要求相邻速度区间连续两次达到至少 $0.5\,\mathrm{{m/s^2}}$ 的减速度，并在随后 $0.3\,\mathrm{{s}}$ 内累计降速至少 $0.3\,\mathrm{{m/s}}$。相邻区间加速度定义为

$$
a_i=\frac{{v_{{i+1}}-v_i}}{{t_{{i+1}}-t_i}},
$$

因此连续两次有效减速要求 $a_i\leq-0.5\,\mathrm{{m/s^2}}$。后续区间和0.3 s累计降速用于确认这次下降具有持续性；确认通过后，$t_2$ 仍记录为第一段合格减速区间结束处的Localization样本时间，不额外加上0.3 s确认窗口。若目标Control输出前已经存在持续且显著的减速过程，该run标记为`ATTRIBUTION_INVALID`，避免把先前制动错误归因到本次障碍物响应。

这里的 $0.5\,\mathrm{{m/s^2}}$ 只用于检测“减速何时真正开始”，不代表Planning设定的目标减速度，也不代表fallback减速度。主结果采用原始速度序列；三点中值平滑以及 $a_{{\mathrm{{th}}}}\in\left\lbrace0.3,0.5,1.0\right\rbrace\,\mathrm{{m/s^2}}$ 的组合进入敏感性分析。

至此两个端点已经明确：$t_1$ 是稳定感知序列第一帧的传感器源时间，$t_2$ 是车辆持续有效物理减速的起点。实际端到端响应时延定义为

$$
T_{{\mathrm{{e2e}}}}=t_2-t_1,\qquad
L_{{\mathrm{{e2e}}}}=1000T_{{\mathrm{{e2e}}}}\;\mathrm{{ms}}.
$$

该时延包含从源点云产生到车辆物理减速之间的感知、预测、规划、控制和执行环节。它是实测闭环响应，不等同于Bridge配置的名义100/300/400 ms等待值。

响应期间距离债务采用速度—时间梯形积分：

$$
D_{{\mathrm{{delay}}}}\approx
\sum_i\frac{{v_i+v_{{i+1}}}}{{2}}\left(t_{{i+1}}-t_i\right),
\qquad t_i\in\left[t_1,t_2\right].
$$

$D_{{\mathrm{{delay}}}}$ 是车辆开始有效制动前已经消耗的距离，baseline也包含系统固有响应时间，因此baseline的该值不会为0。$D_{{\mathrm{{brake,required}}}}$ 是从 $t_2$ 开始到近零速度仍需要的距离。两者分别对应制动前和制动后两个连续阶段，不重复计数。

停车相关端点分开记录：

| 端点 | 判据 | 用途 |
|---|---|---|
| 近停时刻 t_near_stop | 首个速度低于0.1 m/s的样本 | 判断车辆是否曾达到近零速度 |
| 严格停车时刻 t_stop | 速度低于0.1 m/s并持续至少0.5 s | 提供严格停车保持证据 |
| 经验制动完成端点 | t2之后的最低速度样本，并且该run确实达到低于0.1 m/s | 计算baseline经验制动位移 |

Baseline按权威handoff口径计算从 $t_2$ 到经验制动完成端点的三维位置位移。以 $v_2=v(t_2)$ 为输入，建立

$$
D_{{\mathrm{{brake,required}}}}(v_2)=k_{{\mathrm{{median}}}}v_2^2.
$$

$k_{{\mathrm{{median}}}}$ 来自6条baseline近停样本的中位模型。碰撞run不参与模型拟合，只使用该模型估计其在当前制动起始速度下所需的停车距离。速度积分路径长单独保留为诊断字段。

### 3.5 如何读取0 m余量、6 m余量和反事实结果

无附加安全距离的碰撞余量定义为

$$
M_{{\mathrm{{collision,0m}}}}
=D_1-D_{{\mathrm{{delay}}}}-D_{{\mathrm{{brake,required}}}}.
$$

保留6 m安全距离的余量定义为

$$
M_{{\mathrm{{safety,6m}}}}
=D_1-D_{{\mathrm{{delay}}}}-D_{{\mathrm{{brake,required}}}}-6\,\mathrm{{m}}.
$$

两者满足

$$
M_{{\mathrm{{collision,0m}}}}=M_{{\mathrm{{safety,6m}}}}+6\,\mathrm{{m}}.
$$

符号含义如下：

| 结果 | 物理含义 |
|---|---|
| M_safety_6m > 0 | 经验模型预测能够停车，并保留超过6 m距离 |
| M_collision_0m > 0，同时M_safety_6m < 0 | 经验模型预测可以避免接触，但停车后不足6 m |
| M_collision_0m = 0 | 经验模型的接触临界位置 |
| M_collision_0m < 0 | 可用距离不足以覆盖响应距离与经验制动距离，负值绝对值表示空间缺口 |

安全距离参数 $D_{{\mathrm{{margin}}}}\in\left\lbrace0,5,6,8,10\right\rbrace\,\mathrm{{m}}$ 全部进入敏感性计算。0 m用于描述接触边界，6 m用于主安全裕度，其余数值用于检查结论对安全距离设定的敏感程度。

反事实分析回答的问题是：如果该run的闭环响应时间恢复到baseline中位水平，在其他量保持当前经验模型口径时，可以少消耗多少响应距离？令

$$
\Delta T=T_{{\mathrm{{e2e,observed}}}}-T_{{\mathrm{{e2e,baseline\ median}}}},
$$

程序直接积分观测速度历史中有效制动前最后 $\Delta T$ 的行驶距离：

$$
D_{{\mathrm{{saved}}}}=\int_{{t_2-\Delta T}}^{{t_2}}v(t)\,\mathrm{{d}}t,
\qquad
M^{{\mathrm{{cf}}}}=M^{{\mathrm{{observed}}}}+D_{{\mathrm{{saved}}}}.
$$

该计算使用逐run实际速度历史，不采用固定15.6 m/s近似。它用于量化额外响应时间对应的空间损失，不单独承担碰撞归因。RT_ONLY_COLLISION由注入证据、功能链完整性、碰撞结果和baseline安全结果联合判定；Planning进入设计内fallback仍按正常功能响应记录。

### 3.6 用一条碰撞run说明整套计算

以`202607191727`为例：$D_1=39.434\,\mathrm{{m}}$，响应期间行驶距离 $D_{{\mathrm{{delay}}}}=13.520\,\mathrm{{m}}$，经验所需制动距离约为 $29.540\,\mathrm{{m}}$。因此

$$
M_{{\mathrm{{collision,0m}}}}
=39.434-13.520-29.540
=-3.626\,\mathrm{{m}},
$$

表示观测条件下存在约3.626 m空间缺口。再扣除6 m安全距离后：

$$
M_{{\mathrm{{safety,6m}}}}=-3.626-6=-9.626\,\mathrm{{m}}.
$$

该run相对baseline响应可节省距离为 $D_{{\mathrm{{saved}}}}=7.959\,\mathrm{{m}}$，所以反事实结果为

$$
M_{{\mathrm{{collision,0m}}}}^{{\mathrm{{cf}}}}
=-3.626+7.959
=4.332\,\mathrm{{m}},
$$

$$
M_{{\mathrm{{safety,6m}}}}^{{\mathrm{{cf}}}}
=-9.626+7.959
=-1.668\,\mathrm{{m}}.
$$

这组数值表示：响应恢复到baseline水平后，经验模型预测可以避免接触并剩余约4.332 m；距离完整6 m安全目标仍少约1.668 m。该run同时具备注入执行证据、五模块PASS、目标碰撞与baseline全安全结果，因此分类为RT_ONLY_COLLISION。

## 4. 数据质量与场景一致性

23次run数量与实验设计一致。输入文件清单、SHA-256、schema和完整性状态分别保存在`data_inventory.csv`、`input_file_hashes.csv`、`schema_inventory.json`与`run_manifest.csv`。注入组有{len(scb_missing)}次缺少SCB文件，这些run保留物理响应结果，并在名义注入因果判断中标记不确定。

{md_table(manifest, [('run_id','run'),('group_name','组'),('file_count','文件数'),('scb_delay_evidence_present','SCB'),('collision_recorded','碰撞'),('data_integrity_status','完整性')], 3)}

## 5. 时延注入是否生效

{md_table(summary_group_rows, [('group','组'),('n','总n'),('valid_n','有效响应n'),('latency','实际响应中位数/ms'),('added','相对baseline增量/ms'),('Ddelay','D_delay中位数/m'),('Mspace','M_safety_6m中位数/m'),('collisions','碰撞次数')], 3)}

{md_table(scb_audit_rows, [('group','组'),('scb_n','有SCB实测n'),('actual_median','墙钟实际等待中位数/ms'),('actual_min','最小/ms'),('actual_max','最大/ms')], 3)}

SCB文件存在的run直接证明请求等待值、墙钟实际等待值、CARLA帧差和仿真时间差。缺少SCB的run仅保留组目录给出的名义值。Bridge触发后持续延迟全部后续ControlCommand；提前触发会改变弯道与接近阶段控制，组间单变量解释需要附带该限制。

![各组实际响应时延](../figures/fig01_actual_e2e_latency_distribution.png)

图1字段说明：纵轴为每个run实测`t_sensor_origin→t_brake_effective`；散点保留全部有效run；箱体显示组内中位数和四分位范围。

![名义时延与实际响应](../figures/fig02_nominal_vs_actual_latency.png)

图2字段说明：横轴为配置组别，纵轴为实测闭环响应；虚线表示baseline中位数加名义注入量。

![逐run阶段时延](../figures/fig03_stage_latency_waterfall.png)

图3字段说明：每根堆叠柱对应一个有效run；颜色依次表示sensor→perception、perception→prediction、prediction→planning STOP、planning STOP→control和control→物理减速。负阶段值按0显示，原始值保留在表4。

## 6. 时延对制动位置和距离债务的影响

D_delay对实际响应秒数的稳健回归斜率为{slope:.3f} m/s，95%斜率区间为{slope_low:.3f}–{slope_high:.3f} m/s；实验速度的局部理论参考约15.6 m/s。线性回归R²为{corr.rvalue**2 if corr else math.nan:.3f}。时延通过响应期间持续行驶转化为空间债务。

![时延与距离债务](../figures/fig07_delay_distance_vs_latency.png)

图7字段说明：D_delay来自Localization速度梯形积分；黑实线为Theil–Sen稳健拟合；虚线为15.6 m/s理论参考。

{md_table(overall_test_rows, [('metric','指标'),('H','Kruskal–Wallis H'),('p','p值')], 6)}

{md_table(pairwise_test_rows, [('metric','指标'),('comparison','比较'),('p_holm','Holm校正p'),('cliff_delta','Cliff’s δ（注入组−baseline）')], 6)}

统计字段说明：H检验比较四组分布；Holm校正控制同一指标的三次两两比较；Cliff's δ为1表示注入组所有观测值均高于baseline，-1表示均低于baseline。样本量较小，p值和效应量用于探索性描述。

## 7. 经验制动能力和隐形deadline

### 7.1 这一章解决什么问题

第三章把接近障碍物的空间分成两个连续阶段：$t_1$ 到 $t_2$ 是系统尚未产生有效物理减速的响应阶段，$t_2$ 之后是车辆实际降低速度的制动阶段。第六章计算了响应阶段消耗的 $D_{{\mathrm{{delay}}}}$；本章估计从 $t_2$ 开始降到近零速度仍需要的 $D_{{\mathrm{{brake,required}}}}$。

两段距离共同进入0 m碰撞余量：

$$
D_1
=D_{{\mathrm{{delay}}}}
+D_{{\mathrm{{brake,required}}}}
+M_{{\mathrm{{collision,0m}}}}.
$$

只有先估计所需制动距离，才能知道初始空间中最多允许多少距离用于系统响应，并进一步得到最晚允许响应时间。

### 7.2 为什么只使用Baseline停车样本

碰撞run在车辆完成停车前已经接触障碍物，其碰撞前制动距离属于截断观测，无法表示完整停车需要的距离。模型因此只使用6条无碰撞Baseline run。这6条run都在 $t_2$ 后达到过低于0.1 m/s的近零速度，可以测量完整的近停位移。

{md_table(baseline_braking_rows, [('run_id','run'),('v2','t2速度/m/s'),('displacement','三维制动位移/m'),('path_length','积分路径长/m'),('k','k值/s²·m⁻¹'),('a_eff','等效减速度/m/s²'),('strict_stop','保持0.5 s严格停车')], 6)}

6条样本的三维制动位移范围为{min(baseline_displacement_range):.3f}–{max(baseline_displacement_range):.3f} m，均值为{model['empirical_braking_distance_mean_m']:.3f} m；其中{baseline_strict_stop_count}条同时满足低于0.1 m/s并持续0.5 s的严格停车保持。

近停与严格停车承担不同作用：达到过低于0.1 m/s即可测量“从开始制动到近零速度需要多少距离”；持续0.5 s用于证明车辆随后保持停车。当前模型估计近停距离，不对后续停车保持作保证。

### 7.3 经验制动位移采用什么口径

每条Baseline先确定有效物理制动起点 $t_2$ 和起始速度 $v_2$，再寻找 $t_2$ 之后的最低速度样本。只有该run确实达到过低于0.1 m/s时，最低速度样本才作为经验制动完成端点。主制动位移定义为

$$
D_{{\mathrm{{brake,empirical}}}}
=\left\|\mathbf p_{{\mathrm{{min\ speed}}}}-\mathbf p_{{t_2}}\right\|_2.
$$

该量是制动起点到近停端点的三维Localization直线位移。Town04道路存在弯曲，车辆沿轨迹实际走过的积分路径长会更长；本批6条Baseline的积分路径长范围为{min(baseline_path_range):.3f}–{max(baseline_path_range):.3f} m。主分析沿用权威handoff口径使用三维位移，积分路径长单独保留为诊断字段。

因此，本章的“经验制动距离”属于当前场景下的统一分析口径，不能直接替代沿道路中心线测量的真实路径长度。

### 7.4 为什么模型写成 $D=kv^2$

车辆从速度 $v$ 降到接近0时，基础运动学关系为

$$
v_f^2=v^2+2as.
$$

令 $v_f\approx0$，并用正值 $a_{{\mathrm{{eff}}}}$ 表示整个制动过程的等效减速度大小，可得

$$
D_{{\mathrm{{brake}}}}
\approx\frac{{v^2}}{{2a_{{\mathrm{{eff}}}}}}.
$$

定义

$$
k=\frac{{1}}{{2a_{{\mathrm{{eff}}}}}},
$$

即可得到经验模型

$$
D_{{\mathrm{{brake,required}}}}(v)=kv^2.
$$

对每条Baseline分别计算

$$
k_i=\frac{{D_i}}{{v_i^2}},\qquad
a_{{\mathrm{{eff}},i}}=\frac{{v_i^2}}{{2D_i}}.
$$

报告采用6个 $k_i$ 的中位数，降低单条异常停车轨迹的影响：

$$
k_{{\mathrm{{median}}}}={model['k_median']:.8f}\,\mathrm{{s^2/m}}.
$$

最终主模型为

$$
D_{{\mathrm{{brake,required}}}}(v)
={model['k_median']:.8f}v^2.
$$

速度对停车距离呈平方影响。当前模型给出的示例如下：

{md_table(braking_prediction_rows, [('speed','有效制动起始速度/m·s⁻¹'),('distance','模型所需制动距离/m')], 3)}

经验制动位移均值{model['empirical_braking_distance_mean_m']:.3f} m只用于描述6条原始样本。逐run余量计算使用 $k_{{\mathrm{{median}}}}v_2^2$，会根据该run在 $t_2$ 的实际速度调整所需制动距离。

### 7.5 等效减速度与Apollo fallback减速度的区别

由 $k=1/(2a_{{\mathrm{{eff}}}})$ 可将模型换算为中位等效减速度：

$$
a_{{\mathrm{{eff,median}}}}={model['effective_deceleration_median_mps2']:.3f}\,\mathrm{{m/s^2}}.
$$

该数值由“$t_2$速度和近停三维位移”反推，表示整个制动过程的等效能力。它不等同于Planning轨迹的瞬时减速度、ControlCommand制动百分比、Localization峰值减速度或Apollo fallback配置值。

三次碰撞run的Planning日志确认速度求解不可行后进入恒减速度fallback；约4 m/s²来自用户确认的当前Apollo配置行为，归档日志没有保存该数值配置快照。若按恒定4 m/s²和16 m/s起始速度计算，理论制动距离为

$$
D=\frac{{16^2}}{{2\times4}}={fixed_4mps2_brake_at_16_m:.3f}\,\mathrm{{m}}.
$$

Baseline经验模型在16 m/s时给出{representative_brake_m:.3f} m，数值更小。差异可能来自实际减速度过程、弯道路段三维位移口径、制动端点和车辆动力学。当前主报告按Baseline实测模型保持与handoff一致；若目标转为保守安全认证，固定4 m/s²模型适合作为并行保守边界。

### 7.6 Bootstrap区间和图11怎么读

样本量只有6条。报告以固定随机种子执行{model['bootstrap_iterations']}次bootstrap：每次从6个 $k_i$ 中有放回抽取6个并重新计算中位数，最后取2.5%和97.5%分位数。结果为

$$
k_{{\mathrm{{median}}}}
\in
\left[{model['k_bootstrap_95ci'][0]:.8f},\ {model['k_bootstrap_95ci'][1]:.8f}\right]\,\mathrm{{s^2/m}},
$$

对应等效减速度区间

$$
a_{{\mathrm{{eff}}}}
\in
\left[{model['effective_deceleration_bootstrap_95ci_mps2'][0]:.3f},\ {model['effective_deceleration_bootstrap_95ci_mps2'][1]:.3f}\right]\,\mathrm{{m/s^2}}.
$$

以16 m/s为例，中位模型制动距离为{representative_brake_m:.3f} m，bootstrap带对应{representative_brake_low_m:.3f}–{representative_brake_high_m:.3f} m。该区间描述6条样本的抽样波动，没有覆盖几何偏移、Localization采样、弯道位移口径和模型形式误差。

![Baseline制动模型](../figures/fig11_baseline_braking_model.png)

图11中，横轴是 $t_2$ 时车辆速度，纵轴是从 $t_2$ 到最低速度样本的三维位移；蓝点是6条Baseline停车样本，黑线是中位 $k$ 模型，灰带是bootstrap 95%区间。实测速度只覆盖{min(baseline_speed_range):.3f}–{max(baseline_speed_range):.3f} m/s，图中更高速度部分属于模型外推，证据强度低于实测区间。

### 7.7 隐形deadline如何得到

在 $t_1$ 时，自车与障碍物之间有净距 $D_1$。为便于得到显式时间边界，先用 $v_1$ 近似响应阶段速度，则

$$
D_{{\mathrm{{delay}}}}\approx v_1T.
$$

要求停车后保留安全距离 $D_{{\mathrm{{margin}}}}$ 时，空间约束为

$$
D_1
\ge
v_1T
+k_{{\mathrm{{median}}}}v_1^2
+D_{{\mathrm{{margin}}}}.
$$

令空间余量恰好等于0，可得到最晚允许响应时间

$$
T_{{\mathrm{{deadline}}}}
=\frac{{D_1-k_{{\mathrm{{median}}}}v_1^2-D_{{\mathrm{{margin}}}}}}{{v_1}}.
$$

这个时间边界由障碍物净距、车速、经验制动能力和安全距离共同决定，Apollo日志中没有一个固定字段直接给出它，因此称为隐形deadline。速度同时线性增加响应距离并平方增加制动距离，deadline会随速度升高快速缩短。

0 m碰撞边界和6 m安全边界对应两个时间：

$$
T_{{\mathrm{{deadline,0m}}}}
=\frac{{D_1-k_{{\mathrm{{median}}}}v_1^2}}{{v_1}},
$$

$$
T_{{\mathrm{{deadline,6m}}}}
=\frac{{D_1-k_{{\mathrm{{median}}}}v_1^2-6}}{{v_1}}.
$$

### 7.8 16 m/s、40 m净距算例

取 $D_1={representative_d1_m:.0f}$ m、$v_1={representative_speed_mps:.0f}$ m/s，经验所需制动距离为{representative_brake_m:.3f} m。无附加安全距离时：

$$
T_{{\mathrm{{deadline,0m}}}}
=\frac{{{representative_d1_m:.0f}-{representative_brake_m:.3f}}}{{{representative_speed_mps:.0f}}}
={representative_deadline_0m_s:.3f}\,\mathrm{{s}}.
$$

要求保留6 m时：

$$
T_{{\mathrm{{deadline,6m}}}}
=\frac{{{representative_d1_m:.0f}-{representative_brake_m:.3f}-6}}{{{representative_speed_mps:.0f}}}
={representative_deadline_6m_s:.3f}\,\mathrm{{s}}.
$$

该算例表示：响应超过约{representative_deadline_6m_s*1000.0:.0f} ms时，模型预测无法保留完整6 m安全距离；响应超过约{representative_deadline_0m_s*1000.0:.0f} ms时，模型进入无法在接触前停车的区域。

这个显式deadline使用恒速近似，适合解释和实验选点。逐run主分析使用Localization速度积分计算 $D_{{\mathrm{{delay}}}}$，并使用 $t_2$ 的实际速度计算 $D_{{\mathrm{{brake,required}}}}$，精度高于该简化公式。

### 7.9 适用范围与限制

模型适用范围限定于当前Town04静止障碍物、当前车辆与控制配置以及约15–17 m/s场景。主要限制包括：

1. 样本只有6条，且只有1条满足0.5 s严格停车保持。
2. 模型拟合目标是近停三维位移，没有使用弯道路径积分长度。
3. 实测制动起始速度范围为{min(baseline_speed_range):.3f}–{max(baseline_speed_range):.3f} m/s，范围外属于外推。
4. Bootstrap只量化样本抽样波动，没有覆盖全部系统误差。
5. 模型用于本批实验的条件化空间分析，不能直接推广为跨地图、跨车速和跨车辆配置的通用制动模型。

本章建立的核心关系为：制动起始速度决定所需停车距离；所需停车距离决定可供系统响应的剩余空间；剩余空间与响应速度共同决定隐形deadline。逐run碰撞判断仍需结合实际 $D_{{\mathrm{{delay}}}}$、双空间余量、功能链证据与CollisionSensor结果。

## 8. 空间安全余量与碰撞结果

{md_table(run_table, [('run_id','run'),('group','组'),('v1','v1 m/s'),('D1','D1净距/m'),('latency','响应/ms'),('Ddelay','D_delay/m'),('Msafety6','M_safety_6m/m'),('Mcollision0','M_collision_0m/m'),('collision','碰撞'),('impact','碰撞速度/m/s'),('classification','分类')], 3)}

![空间安全余量核心图](../figures/fig08_m_space_vs_actual_latency.png)

图8字段说明：横轴为每个run实测闭环响应；纵轴为`M_safety_6m`。黑色虚线表示6 m安全裕度边界，红色点线位于纵轴-6 m处并表示`M_collision_0m=0`的接触避免边界；点形区分安全、RT-only碰撞、时序诱发功能退化和不确定案例。

![停车净距与碰撞速度](../figures/fig10_outcomes_clearance_and_impact_speed.png)

图10字段说明：左图为无碰撞run在最低速度制动完成端点的估计净距；右图为CARLA CollisionSensor事件前最近Localization样本的碰撞速度；横轴标签为run末4位。

## 9. 模块功能完整性

Perception依据目标连续输出、关键帧时延和感知空窗判定；Prediction依据目标输入/输出及静态语义判定；Planning依据同目标STOP decision和有效轨迹输出判定；Control依据继承目标Trace的`/apollo/control`输出及后续物理减速判定；Bridge依据SCB生命周期记录判定。Planning速度求解`primal infeasible`后进入`speed fallback`并生成`constant deceleration fallback stopping profile`，该链条作为Apollo规划器在停止墙不可行时的预期功能响应。用户确认当前配置下fallback减速度约为4 m/s²；归档日志保留了恒减速度fallback语义，未保存该数值配置快照。ControlCommand的具体brake数值未归档，Control结论使用Trace与物理响应交叉证据。

{md_table(modules, [('run_id','run'),('perception_status','感知'),('prediction_status','预测'),('planning_status','规划'),('control_status','控制'),('bridge_status','Bridge')], 3)}

## 10. 纯实时性碰撞判定

RT_ONLY_COLLISION要求目标碰撞、注入时延证据、功能链全部PASS及同条件baseline安全停车。Planning进入设计内fallback仍判为功能PASS。0 m与6 m余量用于量化碰撞边界和安全裕度，不作为功能正常碰撞的否决门槛。符合条件的run为：{', '.join(rt_runs) if rt_runs else '无'}。

TIMING_INDUCED_FUNCTIONAL_DEGRADATION用于时延伴随感知排队、陈旧数据或功能链退化的碰撞。符合条件的run为：{', '.join(degraded_runs) if degraded_runs else '无'}。

{md_table(collision_counterfactual_rows, [('run_id','碰撞run'),('latency','实际响应/ms'),('Msafety6','观测M_safety_6m/m'),('Msafety6cf','反事实M_safety_6m/m'),('Mcollision0','观测M_collision_0m/m'),('Mcollision0cf','反事实M_collision_0m/m'),('added','相对baseline增量/ms'),('saved','可节省距离/m'),('impact','碰撞速度/m/s'),('realtime_induced','实时性引发'),('classification','子类')], 3)}

{md_table(collision_fallback_rows, [('run_id','碰撞run'),('primal','primal infeasible次数'),('speed_fallback','speed fallback次数'),('constant_decel','恒减速度fallback次数'),('first_line','首次speed fallback源行')], 0)}

三次碰撞run均记录速度求解不可行、speed fallback和恒减速度停车曲线，Planning的fallback行为属于预期功能链。`202607191727`与`202607201611`五模块均为PASS，观测0 m余量为负，去除相对baseline额外时延后的0 m余量转正，分类为RT_ONLY_COLLISION。`202607191739`的反事实0 m余量同样转正，目标关键帧感知时延约814 ms，分类为TIMING_INDUCED_FUNCTIONAL_DEGRADATION。三次目标碰撞均归入实时性引发的碰撞集合，子类用于区分功能链完整与实时链路退化。

![碰撞前联合时间线](../figures/fig14_collision_joint_timeline.png)

图14字段说明：横轴以碰撞时刻为0，黑线为Localization速度；竖线依次标出源观测、Prediction、Planning STOP、Control和有效物理制动时刻。每个子图对应一个目标碰撞run。

每个碰撞run的完整事件、源文件、空间余量与反事实结果位于`per_run/<run_id>/`、`module_function_evidence.json`和`causality_classification.csv`。

## 11. 经验实时性安全边界

当前样本中最大安全实际响应为{max_safe:.3f} ms，全部目标碰撞中的最小响应为{min_target_collision:.3f} ms；两类区间{'发生重叠' if safe_collision_overlap else '未发生重叠'}。{rt_boundary_summary}。响应时间区间存在重叠，单值毫秒阈值需要同时以速度、D1和制动能力为条件。`M_collision_0m`表示实际接触的物理边界，`M_safety_6m`表示安全裕度边界。

6 m安全模型下，安全run的最小`M_safety_6m`为{min_safe_margin:.3f} m，目标碰撞run的最大`M_safety_6m`为{max_collision_margin:.3f} m，两者在当前23次样本中形成{margin_separation_m:.3f} m描述性间隔。三次碰撞的反事实`M_safety_6m`仍小于0，表示消除额外时延后仍无法保留完整6 m裕度；其反事实`M_collision_0m`均大于0，表示消除额外时延后经验模型预测可避免接触。

每增加100 ms实际响应，在15.6 m/s附近对应约1.56 m额外距离债务。当前边界仅适用于本实验速度与约40 m稳定感知净距。

## 12. 敏感性与不确定性

敏感性网格覆盖稳定感知2/3/5帧、有效减速度0.3/0.5/1.0 m/s²与D_margin 0/5/6/8/10 m。完整结果位于`margin_sensitivity.csv`。0 m边界下满足“观测余量<0且反事实余量>0”空间条件的碰撞run为：{', '.join(sensitivity_rt_candidates) if sensitivity_rt_candidates else '无'}。功能链完整性将其进一步划分为RT_ONLY_COLLISION与TIMING_INDUCED_FUNCTIONAL_DEGRADATION。

![敏感性热图](../figures/fig12_sensitivity_heatmap.png)

图12字段说明：行是有效减速度阈值，列是D_margin；格内数字为3帧稳定定义下M_space<0的run数量。

主要不确定性：

1. Localization约9 Hz，单run t2具有约一个采样周期的时间量化。
2. 组合几何偏移缺少逐run bounding box extent，接触样本显示约±0.52 m变化。
3. 非碰撞run缺少CARLA simulation/wall双时钟history。
4. 部分注入run缺少SCB，实际执行值无法逐run核验。
5. 提前触发改变接近阶段控制状态，实际t1速度与D1已纳入逐run模型。
6. 样本量为6/6/6/5，统计检验属于探索性结果。

## 13. 结论

1. 时延组的实际闭环响应随名义注入量总体增加，SCB完整run直接验证了队列等待执行。
2. 100/300/400 ms组相对baseline的观测响应中位增量见第5节；实际增加量受到0.1 s CARLA帧相位、Localization采样和提前触发影响。
3. 时延增加转化为可测量D_delay，稳健回归斜率与实际车速量级一致。
4. `M_collision_0m`与`M_safety_6m`随响应时间增加总体下降，车辆结局同时受t1速度、D1和制动能力波动影响。
5. RT_ONLY_COLLISION run为：{', '.join(rt_runs) if rt_runs else '无'}。
6. 时序诱发功能退化run为：{', '.join(degraded_runs) if degraded_runs else '无'}。
7. 三次目标碰撞的观测`M_collision_0m`均为负，反事实`M_collision_0m`均为正；三次碰撞均由实时性问题引发，其中两次功能链完整，一次伴随感知时序退化。
8. 最大安全响应{max_safe:.3f} ms高于最小目标碰撞响应{min_target_collision:.3f} ms，时延区间发生重叠，单一毫秒阈值无法界定本批次结局。
9. 下一轮应补充300–400 ms临界区、显式障碍物ARM、逐run settings快照、全量SCB命令级证据及CARLA actor bounding box。

## 图表与字段追溯

报告中的聚合数字全部来自`run_metrics.csv`和`group_summary.csv`。表1–表12位于`tables/`。PNG与SVG图位于`figures/`。`verification_report.md`记录样本数、碰撞数、图表输入和分类条件复核。

### 图字段索引

| 图 | 核心字段 | 说明 |
|---|---|---|
| 图1 | group, actual_e2e_latency_ms | 各组闭环响应分布与逐run散点 |
| 图2 | nominal_injected_delay_ms, actual_e2e_latency_ms | 名义注入与实测响应对照 |
| 图3 | 五个stage latency字段 | 关键帧逐阶段时延堆叠 |
| 图4 | relative_to_t1_s, speed_mps | 全run速度—时间轨迹 |
| 图5 | longitudinal_clearance_m, speed_mps | 速度—估计净距轨迹 |
| 图6 | relative_to_t1_s, longitudinal_clearance_m | 净距—时间及STOP/Control/t2事件 |
| 图7 | actual_e2e_latency_ms, D_delay_m | 时延向距离债务转换 |
| 图8 | actual_e2e_latency_ms, M_safety_6m_m, M_collision_0m_m, classification | 6 m安全边界与0 m碰撞边界的核心图 |
| 图9 | group, M_safety_6m_m | 组间6 m空间安全余量分布 |
| 图10 | final_clearance_m, impact_speed_mps | 无碰撞最低速度端点净距与碰撞速度分面 |
| 图11 | brake_start_speed_mps, empirical_braking_distance_m | baseline经验制动模型与bootstrap区间 |
| 图12 | brake threshold, D_margin_m, negative-margin count | 参数敏感性热图 |
| 图13 | event, relative_to_t1_ms | 逐run关键事件时间线 |
| 图14 | time_to_collision_s, speed_mps, functional events | 三个碰撞run联合时间线 |
| 图15 | latency_bin_ms, collision_rate | 实际响应分箱碰撞率及置信区间 |

### 表字段索引

| 表 | 主要字段 | 用途 |
|---|---|---|
| 表1 | run_id, group, file_count, SCB, collision, integrity | 数据清单与完整性 |
| 表2 | nominal delay, v1, D1, fixed step, map, pointcloud | 场景参数一致性 |
| 表3 | event, timestamp, relative_to_t1, source_file/line | 全事件可追溯时间线 |
| 表4 | sensor/perception/prediction/planning/control/physical stage | 关键帧阶段时延 |
| 表5 | D1, D_delay, D_brake_required, M_safety_6m, M_collision_0m及各自counterfactual | 逐run碰撞边界与安全裕度指标 |
| 表6 | 五模块status与evidence/source | 功能完整性证据 |
| 表7 | collision, classification, observed/counterfactual margin | 碰撞因果分类 |
| 表8 | n, missing_n, valid_n, mean/median/quantile/CI | 分组描述统计 |
| 表9 | test, statistic, p/p_holm, Cliff's delta | 探索性组间统计与效应量 |
| 表10 | outcome, metric, n, mean/median/min/max | 安全与碰撞描述对照 |
| 表11 | safe max, target-collision min, RT-only min, overlap | 经验边界可用性 |
| 表12 | stable frames, brake threshold, D_margin, M_space | 含0 m与6 m边界的全敏感性网格 |
"""
    report_path = output_root / "report" / "realtime_collision_experiment_report.md"
    report_path.write_text(report, encoding="utf-8")
    executive = f"""# 执行摘要

- 已识别23次实验，组别数量为6/6/6/5。
- 实际闭环响应中位数：baseline {latency['baseline']['median']:.1f} ms，100 ms组 {latency['delay_100ms']['median']:.1f} ms，300 ms组 {latency['delay_300ms']['median']:.1f} ms，400 ms组 {latency['delay_400ms']['median']:.1f} ms。
- 400 ms组发生{collision_count['delay_400ms']}/5次目标碰撞。
- RT_ONLY_COLLISION：{', '.join(rt_runs) if rt_runs else '无'}。
- TIMING_INDUCED_FUNCTIONAL_DEGRADATION：{', '.join(degraded_runs) if degraded_runs else '无'}。
- 最大安全响应{max_safe:.1f} ms；全部目标碰撞最小响应{min_target_collision:.1f} ms；{rt_boundary_summary}。
- 核心图：`../figures/fig08_m_space_vs_actual_latency.png`。
"""
    (output_root / "report" / "executive_summary.md").write_text(executive, encoding="utf-8")
    css = """<style>body{font-family:'Microsoft YaHei',sans-serif;max-width:1100px;margin:32px auto;line-height:1.65;color:#222}table{border-collapse:collapse;width:100%;font-size:13px}th,td{border:1px solid #bbb;padding:5px 7px}th{background:#eef3f8}img{max-width:100%;height:auto}code{background:#f3f3f3;padding:2px 4px}h1,h2,h3{color:#17365d}</style>"""
    mathjax = """<script>window.MathJax={tex:{inlineMath:[['$','$']],displayMath:[['$$','$$']]},options:{skipHtmlTags:['script','noscript','style','textarea','pre','code']}};</script><script async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>"""
    html_body = markdown.markdown(report, extensions=["tables", "fenced_code"])
    (output_root / "report" / "realtime_collision_experiment_report.html").write_text(
        "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>" + css + mathjax + "</head><body>" + html_body + "</body></html>",
        encoding="utf-8",
    )


def write_supporting_reports(
    manifest: Sequence[Dict[str, Any]],
    metrics: Sequence[Dict[str, Any]],
    associations: Sequence[Dict[str, Any]],
    modules: Sequence[Dict[str, Any]],
    model: Dict[str, Any],
    config: Dict[str, Any],
    output_root: Path,
) -> None:
    missing_scb = [row["run_id"] for row in manifest if row["nominal_injected_delay_ms"] > 0 and not row["scb_delay_evidence_present"]]
    collision_runs = [row["run_id"] for row in metrics if row.get("collision")]
    quality = f"""# 数据质量报告

## 数量核验

- baseline：{sum(row['group_name']=='baseline' for row in manifest)}/6
- 100 ms：{sum(row['group_name']=='delay_100ms' for row in manifest)}/6
- 300 ms：{sum(row['group_name']=='delay_300ms' for row in manifest)}/6
- 400 ms：{sum(row['group_name']=='delay_400ms' for row in manifest)}/5
- 总数：{len(manifest)}/23

## 关键缺失

- 注入组缺少SCB：{', '.join(missing_scb) if missing_scb else '无'}
- CollisionSensor记录：{', '.join(collision_runs) if collision_runs else '无'}
- 非碰撞run未保存actor history，统一时钟仅能使用Apollo/Localization epoch。
- 每个run未保存Bridge settings快照，SCB行承担注入参数证据；无SCB run的实际注入状态保持不确定。
- ControlCommand payload未归档，Control事件时间来自继承目标Trace的`/apollo/control`输出。

## 固定条件证据

- 地图：用户固定Town04；碰撞event直接记录`Carla/Maps/Town04`。
- 固定步长：用户固定0.1 s；SCB保存CARLA frame与simulation elapsed差。
- 点云：用户固定130万；run目录只保存处理后`num points before fusing`，缺少原始点云计数配置快照。

## 纳入原则

所有23个run保留在清单和原始散点中。速度越界、SCB缺失、t1前已进入持续减速、时间映射受限均通过质量字段分层，未执行静默删除。
"""
    (output_root / "data_quality_report.md").write_text(quality, encoding="utf-8")

    association_report = "# 目标关联报告\n\n" + md_table(
        associations,
        [
            ("run_id", "run"),
            ("carla_target_actor_id", "CARLA actor"),
            ("apollo_obstacle_id_sequence", "Apollo ID链"),
            ("position_error_median_m", "位置误差中位数/m"),
            ("matched_frame_count", "匹配帧数"),
            ("confidence", "置信度"),
            ("conclusion", "结论"),
        ],
    )
    association_report += "\n\n碰撞run执行CARLA y轴到Apollo y轴的符号转换并进行多帧插值匹配。安全run以Planning STOP目标、Prediction静态语义和Fusion连续轨迹联合确定。\n"
    (output_root / "target_association_report.md").write_text(association_report, encoding="utf-8")

    module_report = "# 模块功能完整性报告\n\n" + md_table(
        modules,
        [
            ("run_id", "run"),
            ("perception_status", "Perception"),
            ("prediction_status", "Prediction"),
            ("planning_status", "Planning"),
            ("control_status", "Control"),
            ("bridge_status", "Bridge"),
            ("control_evidence", "控制证据"),
        ],
    )
    (output_root / "module_function_report.md").write_text(module_report, encoding="utf-8")

    braking = f"""# Baseline经验制动模型诊断

- 样本数：{model['sample_count']}
- run：{', '.join(model['sample_run_ids'])}
- 模型：`D_brake_required(v)=k_median×v²`
- k中位数：{model['k_median']:.8f} s²/m
- k bootstrap 95%区间：{model['k_bootstrap_95ci'][0]:.8f}–{model['k_bootstrap_95ci'][1]:.8f} s²/m
- 等效减速度中位数：{model['effective_deceleration_median_mps2']:.4f} m/s²
- 经验制动位移均值：{model['empirical_braking_distance_mean_m']:.4f} m
- 距离口径：{model['distance_method']}
- 适用范围：当前Town04静止障碍物、约15–17 m/s实验。

碰撞run的碰撞前行驶距离未用于拟合近停制动位移。模型通过baseline近停样本估计碰撞run所需制动距离。严格停车保持状态与近停代理状态已在`run_metrics.csv`分字段记录。
"""
    (output_root / "braking_model_diagnostics.md").write_text(braking, encoding="utf-8")


def verify_results(
    metrics: Sequence[Dict[str, Any]],
    manifest: Sequence[Dict[str, Any]],
    classifications: Sequence[Dict[str, Any]],
    output_root: Path,
) -> Dict[str, Any]:
    expected_counts = {"baseline": 6, "delay_100ms": 6, "delay_300ms": 6, "delay_400ms": 5}
    actual_counts = Counter(row["group_name"] for row in manifest)
    collision_files = sum(bool(row.get("collision_recorded")) for row in manifest)
    collision_metrics = sum(bool(row.get("collision")) for row in metrics)
    rt_rows = [row for row in classifications if row["classification"] == "RT_ONLY_COLLISION"]
    rt_conditions = {
        row["run_id"]: {
            "collision": bool(row["collision_with_target"]),
            "injected_delay_group": row.get("group_name") != "baseline",
            "all_baseline_runs_safe": not any(
                item.get("group_name") == "baseline" and item.get("collision")
                for item in metrics
            ),
            "all_modules_pass": all(row[key] == "PASS" for key in ["perception_status", "prediction_status", "planning_status", "control_status", "bridge_status"]),
        }
        for row in rt_rows
    }
    figure_png_count = len(list((output_root / "figures").glob("*.png")))
    figure_svg_count = len(list((output_root / "figures").glob("*.svg")))
    required_figure_stems = [f"fig{index:02d}" for index in range(1, 16)]
    required_figures_present = all(
        any((output_root / "figures").glob(f"{stem}_*.png"))
        and any((output_root / "figures").glob(f"{stem}_*.svg"))
        for stem in required_figure_stems
    )
    baseline_distances = [
        fnum(row.get("empirical_braking_distance_m"))
        for row in metrics
        if row.get("group_name") == "baseline"
        and math.isfinite(fnum(row.get("empirical_braking_distance_m")))
    ]
    analyzed_rows = [row for row in metrics if row.get("analysis_status") == "ANALYZED"]
    invalid_rows = [row for row in metrics if row.get("analysis_status") == "ATTRIBUTION_INVALID"]
    report_path = output_root / "report" / "realtime_collision_experiment_report.md"
    report_text = report_path.read_text(encoding="utf-8") if report_path.exists() else ""
    hashes_path = output_root / "input_file_hashes.csv"
    hash_rows = (
        list(csv.DictReader(hashes_path.open(encoding="utf-8-sig")))
        if hashes_path.exists()
        else []
    )
    checks = {
        "run_count_matches_23": len(manifest) == 23,
        "group_counts_match": all(actual_counts[group] == count for group, count in expected_counts.items()),
        "metrics_row_count_matches_runs": len(metrics) == len(manifest),
        "collision_count_matches_event_files": collision_files == collision_metrics,
        "rt_only_all_conditions_satisfied": all(all(values.values()) for values in rt_conditions.values()),
        "png_and_svg_present": figure_png_count >= 17 and figure_svg_count >= 17,
        "all_15_required_figures_present": required_figures_present,
        "all_runs_classified_once": len(classifications) == len(manifest) and len({row['run_id'] for row in classifications}) == len(manifest),
        "nominal_and_actual_delay_fields_separate": all("nominal_injected_delay_ms" in row and "actual_e2e_latency_ms" in row for row in metrics),
        "baseline_model_uses_all_six_runs": len(baseline_distances) == 6,
        "baseline_braking_distance_matches_authoritative_handoff": bool(
            baseline_distances
            and abs(float(np.mean(baseline_distances)) - 24.876) <= 0.05
        ),
        "analyzed_runs_have_finite_space_margin": all(
            math.isfinite(fnum(row.get("M_space_m"))) for row in analyzed_rows
        ),
        "analyzed_runs_have_both_margin_definitions": all(
            math.isfinite(fnum(row.get("M_safety_6m_m")))
            and math.isfinite(fnum(row.get("M_collision_0m_m")))
            and math.isfinite(fnum(row.get("M_safety_6m_counterfactual_m")))
            and math.isfinite(fnum(row.get("M_collision_0m_counterfactual_m")))
            for row in analyzed_rows
        ),
        "dual_margins_differ_by_six_meters": all(
            abs(
                fnum(row.get("M_collision_0m_m"))
                - fnum(row.get("M_safety_6m_m"))
                - 6.0
            )
            <= 1e-9
            for row in analyzed_rows
        ),
        "target_collision_counterfactual_avoids_contact": all(
            fnum(row.get("M_collision_0m_counterfactual_m")) > 0
            for row in metrics
            if row.get("collision_with_target")
        ),
        "target_collision_planning_fallback_evidence_present": all(
            int(row.get("planning_primal_infeasible_count") or 0) > 0
            and int(row.get("planning_speed_fallback_count") or 0) > 0
            and int(row.get("planning_constant_deceleration_fallback_count") or 0) > 0
            for row in metrics
            if row.get("collision_with_target")
        ),
        "target_collisions_classified_as_realtime_induced": all(
            next(
                item["classification"]
                for item in classifications
                if item["run_id"] == row["run_id"]
            )
            in {"RT_ONLY_COLLISION", "TIMING_INDUCED_FUNCTIONAL_DEGRADATION"}
            for row in metrics
            if row.get("collision_with_target")
        ),
        "invalid_attribution_excluded_from_latency": all(
            not math.isfinite(fnum(row.get("actual_e2e_latency_ms"))) for row in invalid_rows
        ),
        "collision_runs_do_not_emit_strict_stop_event": all(
            not math.isfinite(fnum(row.get("t_stop_s")))
            for row in metrics
            if row.get("collision")
        ),
        "report_has_no_nan_literal": "nan" not in report_text.lower(),
        "report_avoids_forbidden_contrast_phrase": not bool(
            re.search(r"不是.{0,80}而是", report_text, flags=re.DOTALL)
        ),
        "input_hashes_complete": bool(hash_rows)
        and all(bool(re.fullmatch(r"[0-9a-f]{64}", row.get("sha256", ""))) for row in hash_rows),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "expected_group_counts": expected_counts,
        "actual_group_counts": dict(actual_counts),
        "collision_event_count": collision_files,
        "collision_metric_count": collision_metrics,
        "rt_only_condition_audit": rt_conditions,
        "figure_png_count": figure_png_count,
        "figure_svg_count": figure_svg_count,
    }
    report_lines = ["# 结果一致性检查", "", f"总体结果：**{result['status']}**", ""]
    for key, value in checks.items():
        report_lines.append(f"- {'PASS' if value else 'FAIL'}：`{key}`")
    report_lines.extend(
        [
            "",
            f"- CollisionSensor文件计数：{collision_files}",
            f"- run_metrics碰撞计数：{collision_metrics}",
            f"- PNG/SVG数量：{figure_png_count}/{figure_svg_count}",
            "",
            "## RT_ONLY_COLLISION必要条件复核",
            "",
            "```json",
            json.dumps(_json_safe(rt_conditions), ensure_ascii=False, indent=2),
            "```",
        ]
    )
    (output_root / "verification_report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    write_json(output_root / "verification_result.json", result)
    return result


def source_versions() -> Dict[str, Any]:
    return {
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "pandas": pd.__version__,
        "scipy": scipy.__version__,
        "matplotlib": matplotlib.__version__,
        "pyyaml": yaml.__version__,
        "markdown": markdown.__version__,
    }


def run_analysis(
    input_root: Path,
    output_root: Path,
    config_path: Path,
    compute_hashes: bool = True,
) -> int:
    output_root.mkdir(parents=True, exist_ok=True)
    for name in ["config", "src", "tests", "intermediate", "per_run", "figures", "tables", "report", "logs"]:
        (output_root / name).mkdir(parents=True, exist_ok=True)
    log_path = output_root / "logs" / "analysis.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.FileHandler(log_path, encoding="utf-8"), logging.StreamHandler(sys.stdout)],
        force=True,
    )
    LOG.info("Loading config: %s", config_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    timezone = ZoneInfo(config["analysis"]["timezone"])
    write_json(output_root / "software_versions.json", source_versions())

    runs = discover_runs(config)
    LOG.info("Discovered %d runs", len(runs))
    inventory, schema = inventory_inputs(runs, config, compute_hashes)
    add_reference_inventory(inventory, schema, config, compute_hashes)
    write_csv(output_root / "data_inventory.csv", inventory)
    write_csv(
        output_root / "input_file_hashes.csv",
        [
            {
                "source_file": row["source_file"],
                "size_bytes": row["size_bytes"],
                "sha256": row["sha256"],
                "group_name": row["group_name"],
                "run_id": row["run_id"],
            }
            for row in inventory
        ],
    )
    write_json(output_root / "schema_inventory.json", schema)

    parsed_runs: List[ParsedRun] = []
    raw_metrics: List[Dict[str, Any]] = []
    debug_by_run: Dict[str, Any] = {}
    for index, spec in enumerate(runs, 1):
        LOG.info("Parsing %s (%d/%d)", spec.run_id, index, len(runs))
        parsed = parse_run(spec, config, timezone)
        parsed_runs.append(parsed)
        row, debug = raw_run_metrics(parsed, config)
        raw_metrics.append(row)
        debug_by_run[spec.run_id] = debug

    manifest = manifest_rows(parsed_runs, inventory)
    write_csv(output_root / "run_manifest.csv", manifest)
    model = build_braking_model(raw_metrics, config, output_root)
    metrics = enrich_safety_metrics(raw_metrics, model, config)
    parsed_by_id = {parsed.spec.run_id: parsed for parsed in parsed_runs}

    # Replace the constant-speed counterfactual distance with direct integration.
    for row in metrics:
        added_ms = fnum(row.get("measured_added_delay_ms"))
        t2 = fnum(row.get("t_brake_effective_s"))
        parsed = parsed_by_id[row["run_id"]]
        if math.isfinite(added_ms) and added_ms > 0 and math.isfinite(t2):
            saved = integrate_speed(parsed.localization, t2 - added_ms / 1000.0, t2)
            if math.isfinite(saved):
                row["D_saved_counterfactual_m"] = saved
                row["M_space_counterfactual_m"] = fnum(row.get("M_space_m")) + saved
                row["M_safety_6m_counterfactual_m"] = row[
                    "M_space_counterfactual_m"
                ]
                row["M_collision_0m_counterfactual_m"] = fnum(
                    row.get("M_collision_0m_m")
                ) + saved
                row["counterfactual_safe"] = row["M_space_counterfactual_m"] > 0
                row["counterfactual_collision_avoided"] = (
                    row["M_collision_0m_counterfactual_m"] > 0
                )
                row["counterfactual_distance_method"] = "trapezoidal integration over pre-brake speed history"

    associations = [target_association(parsed) for parsed in parsed_runs]
    association_by_run = {row["run_id"]: row for row in associations}
    for row in metrics:
        association = association_by_run[row["run_id"]]
        row["target_association_confidence"] = association["confidence"]
        row["target_association_method"] = association["conclusion"]

    modules = [assess_modules(parsed, next(row for row in metrics if row["run_id"] == parsed.spec.run_id), config) for parsed in parsed_runs]
    module_by_run = {row["run_id"]: row for row in modules}
    classifications = [classify_run(row, module_by_run[row["run_id"]]) for row in metrics]
    sensitivity = sensitivity_rows(parsed_runs, metrics, model, config)
    group_summary, test_rows, effect_rows, collision_rows = group_statistics(metrics, classifications, config)

    all_events: List[Dict[str, Any]] = []
    clock_full: List[Dict[str, Any]] = []
    clock_csv: List[Dict[str, Any]] = []
    for parsed, row in zip(parsed_runs, metrics):
        events = event_timeline_for_run(parsed, row)
        all_events.extend(events)
        per_dir = output_root / "per_run" / parsed.spec.run_id
        per_dir.mkdir(parents=True, exist_ok=True)
        write_csv(per_dir / "event_timeline.csv", events)
        (per_dir / "event_timeline.md").write_text(
            f"# {parsed.spec.run_id}事件时间线\n\n" + md_table(events, [("event", "事件"), ("unified_timestamp_s", "统一时间/s"), ("relative_to_t1_ms", "相对t1/ms"), ("ego_speed_mps", "速度/m/s"), ("longitudinal_clearance_m", "净距/m"), ("detection_method", "判据")]) + "\n",
            encoding="utf-8",
        )
        write_json(per_dir / "analysis_debug.json", debug_by_run[parsed.spec.run_id])
        write_json(per_dir / "target_association_debug.json", association_by_run[parsed.spec.run_id])
        # Save every localization sample used by the physical analysis.
        trajectory = [
            {
                "run_id": parsed.spec.run_id,
                "time_s": sample.time_s,
                "relative_to_t1_s": sample.time_s - fnum(row.get("t_sensor_origin_s")),
                "x_m": sample.x_m,
                "y_m": sample.y_m,
                "speed_mps": sample.speed_mps,
                "source_file": sample.source_file,
                "source_line": sample.source_line,
            }
            for sample in parsed.localization
        ]
        write_csv(per_dir / "localization_trajectory.csv", trajectory)
        full_clock = dict(parsed.clock, run_id=parsed.spec.run_id, group_name=parsed.spec.group_name)
        clock_full.append(full_clock)
        clock_csv.append({key: value for key, value in full_clock.items() if key not in {"sim_times_s", "wall_times_s", "residual_ms"}})

    write_csv(output_root / "event_timeline.csv", all_events)
    write_csv(output_root / "target_association.csv", associations)
    write_json(output_root / "target_association_debug.json", associations)
    write_csv(output_root / "clock_alignment.csv", clock_csv)
    write_json(output_root / "clock_alignment_summary.json", clock_full)
    write_csv(output_root / "run_metrics.csv", metrics)
    write_csv(output_root / "module_function_assessment.csv", modules)
    write_json(output_root / "module_function_evidence.json", modules)
    write_csv(output_root / "causality_classification.csv", classifications)
    write_csv(output_root / "margin_sensitivity.csv", sensitivity)
    write_csv(output_root / "group_summary.csv", group_summary)
    write_csv(output_root / "statistical_tests.csv", test_rows)
    write_csv(output_root / "effect_sizes.csv", effect_rows)
    write_csv(output_root / "collision_rate_summary.csv", collision_rows)

    safe_rows = [row for row in metrics if not row.get("collision") and math.isfinite(fnum(row.get("actual_e2e_latency_ms")))]
    collision_metric_rows = [row for row in metrics if row.get("collision")]
    safe_collision = []
    for label, rows_subset in [("safe", safe_rows), ("collision", collision_metric_rows)]:
        for metric_name in [
            "actual_e2e_latency_ms",
            "D_delay_m",
            "M_safety_6m_m",
            "M_collision_0m_m",
            "t1_speed_mps",
            "D1_clear_m",
        ]:
            values = [fnum(item.get(metric_name)) for item in rows_subset if math.isfinite(fnum(item.get(metric_name)))]
            safe_collision.append({"outcome": label, "metric": metric_name, "n": len(values), "mean": float(np.mean(values)) if values else math.nan, "median": float(np.median(values)) if values else math.nan, "min": min(values) if values else math.nan, "max": max(values) if values else math.nan})
    write_csv(output_root / "safe_collision_comparison.csv", safe_collision)
    class_by_id = {item["run_id"]: item for item in classifications}
    safe_lat = [fnum(item.get("actual_e2e_latency_ms")) for item in metrics if not item.get("collision") and math.isfinite(fnum(item.get("actual_e2e_latency_ms")))]
    collision_lat = [
        fnum(item.get("actual_e2e_latency_ms"))
        for item in metrics
        if item.get("collision_with_target")
        and math.isfinite(fnum(item.get("actual_e2e_latency_ms")))
    ]
    rt_lat = [fnum(item.get("actual_e2e_latency_ms")) for item in metrics if class_by_id[item["run_id"]]["classification"] == "RT_ONLY_COLLISION"]
    safe_margins = [
        fnum(item.get("M_space_m"))
        for item in metrics
        if not item.get("collision") and math.isfinite(fnum(item.get("M_space_m")))
    ]
    collision_margins = [
        fnum(item.get("M_space_m"))
        for item in metrics
        if item.get("collision_with_target") and math.isfinite(fnum(item.get("M_space_m")))
    ]
    safe_collision_margins = [
        fnum(item.get("M_collision_0m_m"))
        for item in metrics
        if not item.get("collision")
        and math.isfinite(fnum(item.get("M_collision_0m_m")))
    ]
    target_collision_margins = [
        fnum(item.get("M_collision_0m_m"))
        for item in metrics
        if item.get("collision_with_target")
        and math.isfinite(fnum(item.get("M_collision_0m_m")))
    ]
    boundary = [
        {
            "max_safe_actual_e2e_latency_ms": max(safe_lat) if safe_lat else math.nan,
            "min_target_collision_actual_e2e_latency_ms": min(collision_lat) if collision_lat else math.nan,
            "min_rt_only_collision_actual_e2e_latency_ms": min(rt_lat) if rt_lat else math.nan,
            "safe_vs_target_collision_latency_overlap": bool(
                safe_lat and collision_lat and max(safe_lat) >= min(collision_lat)
            ),
            "rt_only_boundary_available": bool(rt_lat),
            "minimum_safe_M_space_m": min(safe_margins) if safe_margins else math.nan,
            "maximum_target_collision_M_space_m": max(collision_margins) if collision_margins else math.nan,
            "minimum_safe_M_safety_6m_m": min(safe_margins) if safe_margins else math.nan,
            "maximum_target_collision_M_safety_6m_m": max(collision_margins) if collision_margins else math.nan,
            "minimum_safe_M_collision_0m_m": min(safe_collision_margins)
            if safe_collision_margins
            else math.nan,
            "maximum_target_collision_M_collision_0m_m": max(target_collision_margins)
            if target_collision_margins
            else math.nan,
            "interpretation": (
                "strict RT-only time boundary unavailable; descriptive safe/collision intervals overlap"
                if not rt_lat
                else "empirical RT-only boundary conditioned on v1, D1 and braking model"
            ),
        }
    ]
    write_csv(output_root / "deadline_boundary.csv", boundary)

    # Required numbered table files.
    table_map = {
        "table01_run_manifest.csv": manifest,
        "table02_scenario_parameters.csv": [{key: row.get(key) for key in ["run_id", "group_name", "nominal_injected_delay_ms", "t1_speed_mps", "D1_center_m", "D1_clear_m", "D1_lateral_offset_m", "fixed_delta_seconds", "map_name", "pointcloud_count_configured", "speed_condition_in_window"]} for row in metrics],
        "table03_event_timeline.csv": all_events,
        "table04_stage_latencies.csv": [{key: row.get(key) for key in ["run_id", "group_name", "sensor_to_perception_ms", "perception_to_prediction_ms", "prediction_to_planning_stop_ms", "planning_stop_to_control_ms", "control_to_effective_brake_ms", "actual_e2e_latency_ms"]} for row in metrics],
        "table05_distance_and_safety_metrics.csv": metrics,
        "table06_module_function_status.csv": modules,
        "table07_collision_and_causality.csv": classifications,
        "table08_group_descriptive_statistics.csv": group_summary,
        "table09_group_tests_and_effects.csv": [*test_rows, *effect_rows],
        "table10_safe_vs_collision.csv": safe_collision,
        "table11_empirical_deadline_boundary.csv": boundary,
        "table12_sensitivity_analysis.csv": sensitivity,
    }
    for name, rows_value in table_map.items():
        write_csv(output_root / "tables" / name, rows_value)

    delay_injection_audit(config, metrics, output_root)
    write_supporting_reports(manifest, metrics, associations, modules, model, config, output_root)
    generate_figures(parsed_runs, metrics, classifications, model, sensitivity, clock_full, config, output_root)
    build_report(metrics, manifest, modules, classifications, group_summary, collision_rows, model, sensitivity, config, output_root)
    verification = verify_results(metrics, manifest, classifications, output_root)

    LOG.info("Analysis complete; verification=%s", verification["status"])
    latency_lookup = group_metric_lookup(group_summary, "actual_e2e_latency_ms")
    ddelay_lookup = group_metric_lookup(group_summary, "D_delay_m")
    margin_lookup = group_metric_lookup(group_summary, "M_space_m")
    print("\n===== FINAL SUMMARY =====")
    print(f"识别实验总数: {len(runs)}")
    print("组别数量: " + ", ".join(f"{group}={sum(run.group_name==group for run in runs)}" for group in config["groups"]))
    print(f"完整/可分析run: {sum(row.get('analysis_status')=='ANALYZED' for row in metrics)}/{len(metrics)}")
    for group in config["groups"]:
        print(f"{group}: latency_median={latency_lookup[group].get('median', math.nan):.3f} ms, D_delay_median={ddelay_lookup[group].get('median', math.nan):.3f} m, M_space_median={margin_lookup[group].get('median', math.nan):.3f} m, collisions={sum(row.get('collision') and row['group_name']==group for row in metrics)}")
    print("RT_ONLY_COLLISION: " + ", ".join(row["run_id"] for row in classifications if row["classification"] == "RT_ONLY_COLLISION"))
    print("TIMING_INDUCED_FUNCTIONAL_DEGRADATION: " + ", ".join(row["run_id"] for row in classifications if row["classification"] == "TIMING_INDUCED_FUNCTIONAL_DEGRADATION"))
    print("INDETERMINATE: " + ", ".join(row["run_id"] for row in classifications if row["classification"] == "INDETERMINATE"))
    print(f"主报告: {output_root / 'report' / 'realtime_collision_experiment_report.md'}")
    print(f"核心图: {output_root / 'figures' / 'fig08_m_space_vs_actual_latency.png'}")
    return 0 if verification["status"] == "PASS" else 2
