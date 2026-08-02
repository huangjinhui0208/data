#!/usr/bin/env python3
"""Fast Apollo/Carla collision classifier.

Consumes existing logs, CSV, JSON, and JSONL only. It never parses cyber
records and never uses control/guardian for the final verdict.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import re
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


PERCEPTION_TAGS = {"FUSION_OBS_FRAME", "FUSION_OBS", "PLANNING_EGO_STATE", "LOCALIZATION_POSE"}
PREDICTION_TAGS = {"PREDICTION_INPUT_OBS", "PREDICTION_OUTPUT_OBS"}
PLANNING_TAGS = {
    "PLANNING_INPUT_PRED_FRAME",
    "PLANNING_INPUT_OBS",
    "PLANNING_DECISION",
    "PLANNING_ST_BOUNDARY",
    "PLANNING_OUTPUT",
}
TAGS = PERCEPTION_TAGS | PREDICTION_TAGS | PLANNING_TAGS
FINAL_VERDICTS = {
    "PERCEPTION_ABNORMAL",
    "PREDICTION_ABNORMAL",
    "PLANNING_ABNORMAL",
    "FUNCTION_NORMAL_BUT_TOO_LATE",
    "PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING",
    "UNKNOWN_OR_DATA_INSUFFICIENT",
}
LONGITUDINAL = {"STOP", "FOLLOW", "YIELD", "OVERTAKE"}
VIRTUAL_PREFIXES = (
    "TL_",
    "SS_",
    "CW_",
    "DEST",
    "PULL_OVER",
    "CLEAR_ZONE",
    "REFERENCE_END",
    "KEEP_CLEAR",
    "VIRTUAL",
)


def json_safe(value: Any) -> Any:
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return None if np.isnan(value) else float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if value is pd.NA:
        return None
    try:
        if pd.isna(value):
            return None
    except Exception:
        pass
    return value


def normalize_planning_id(obs_id: Any) -> str:
    if obs_id is None:
        return ""
    try:
        if pd.isna(obs_id):
            return ""
    except Exception:
        pass
    text = str(obs_id).strip()
    if not text:
        return ""
    if text.endswith(".0"):
        text = text[:-2]
    if "_" in text and text.rsplit("_", 1)[-1].isdigit():
        text = text.rsplit("_", 1)[0]
    return text


def is_real_obstacle_id(obs_id: Any) -> bool:
    text = normalize_planning_id(obs_id)
    if not text:
        return False
    upper = text.upper()
    if any(upper.startswith(prefix) for prefix in VIRTUAL_PREFIXES):
        return False
    return text.isdigit()


def normalize_type(value: Any) -> str:
    if value is None:
        return ""
    text = str(value).strip()
    if text.endswith(".0"):
        text = text[:-2]
    upper = text.upper()
    if upper == "0":
        return "UNKNOWN"
    if upper == "1":
        return "UNKNOWN_MOVABLE"
    if upper == "2":
        return "UNKNOWN_UNMOVABLE"
    if upper in {"5", "VEHICLE", "CAR", "TRUCK", "BUS"} or upper.startswith("VEHICLE."):
        return "VEHICLE"
    if upper in {"3", "PEDESTRIAN"}:
        return "PEDESTRIAN"
    if upper in {"4", "BICYCLE", "CYCLIST"}:
        return "BICYCLE"
    return upper


def normalize_decision_type(value: Any) -> str:
    if value is None:
        return ""
    upper = str(value).strip().upper()
    for token in ["STOP", "FOLLOW", "YIELD", "OVERTAKE", "IGNORE"]:
        if upper == token or upper.endswith(f"_{token}") or token in upper:
            return token
    return upper


def to_float(value: Any) -> Optional[float]:
    try:
        if value is None or pd.isna(value):
            return None
        result = float(value)
        if math.isnan(result):
            return None
        return result
    except Exception:
        return None


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


def normalize_angle(angle: Optional[float]) -> float:
    if angle is None:
        return math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return abs(angle)


def parse_datetime_to_epoch(text: str) -> Optional[float]:
    try:
        dt = datetime.fromisoformat(str(text).replace("Z", "+00:00"))
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        return dt.timestamp()
    except Exception:
        return None


def parse_apollo_log_time(line: str, reference_epoch: Optional[float]) -> Optional[float]:
    match = re.search(r"[IWEF](\d{2})(\d{2})\s+(\d{2}):(\d{2}):(\d{2})\.(\d+)", line)
    if not match:
        return None
    hour = int(match.group(3))
    minute = int(match.group(4))
    second = int(match.group(5))
    frac = float(f"0.{match.group(6)}")
    seconds = hour * 3600 + minute * 60 + second + frac
    if reference_epoch and reference_epoch > 1e8:
        utc_midnight = math.floor(reference_epoch / 86400.0) * 86400.0
        candidates = []
        for day_shift in (-86400.0, 0.0, 86400.0):
            for offset_hours in range(-14, 15):
                candidates.append(utc_midnight + day_shift + seconds + offset_hours * 3600.0)
        return min(candidates, key=lambda value: abs(value - reference_epoch))
    return seconds


def convert_value(key: str, text: str) -> Any:
    value = text.strip().strip('"')
    if key in {"id", "obs_id", "obstacle_id", "stop_id", "blocking_obstacle_id", "planning_obstacle_id", "perception_id"}:
        return normalize_planning_id(value)
    lower = value.lower()
    if lower in {"true", "false"}:
        return lower == "true"
    if re.fullmatch(r"[-+]?\d+", value):
        try:
            return int(value)
        except Exception:
            return value
    if re.fullmatch(r"[-+]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][-+]?\d+)?", value) or re.fullmatch(r"[-+]?\d+[eE][-+]?\d+", value):
        try:
            return float(value)
        except Exception:
            return value
    return value


def first_present(mapping: Dict[str, Any], names: Sequence[str]) -> Any:
    lower_to_key = {str(k).lower(): k for k in mapping}
    for name in names:
        key = lower_to_key.get(name.lower())
        if key is None:
            continue
        value = mapping.get(key)
        if value is None:
            continue
        try:
            if pd.isna(value):
                continue
        except Exception:
            pass
        if str(value).strip() == "":
            continue
        return value
    return None


def to_int(value: Any) -> Optional[int]:
    try:
        if value is None or pd.isna(value):
            return None
        return int(float(value))
    except Exception:
        return None


def to_bool(value: Any) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y", "ok", "success", "succeeded"}:
        return True
    if text in {"false", "0", "no", "n", "fail", "failed", "error"}:
        return False
    return None


def parse_tagged_log_line(line: str, source_file: str = "", line_no: int = 0) -> Optional[Dict[str, Any]]:
    """Parse one tagged Apollo key=value log line."""
    tag_match = re.search(r"\[([A-Z][A-Z0-9_]*)\]", line)
    if not tag_match or tag_match.group(1) not in TAGS:
        return None
    tag = tag_match.group(1)
    payload = line[tag_match.end() :]
    row: Dict[str, Any] = {"tag": tag, "source_file": source_file, "line_no": line_no, "raw_line": line.rstrip("\n")}
    for key, value in re.findall(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)", payload):
        row[key] = convert_value(key, value)
    return row


def first_existing(columns: Iterable[str], names: Sequence[str]) -> Optional[str]:
    by_lower = {c.lower(): c for c in columns}
    for name in names:
        if name.lower() in by_lower:
            return by_lower[name.lower()]
    return None


@dataclass
class Config:
    analysis: Dict[str, Any] = field(default_factory=lambda: {
        "pre_collision_window_sec": 15.0,
        "post_collision_window_sec": 2.0,
        "planning_target_window_sec": 3.0,
        "max_time_match_diff_sec": 0.10,
    })
    target_resolution: Dict[str, Any] = field(default_factory=lambda: {
        "min_score": 0.60,
        "min_score_margin": 0.08,
        "require_planning_or_cli_target": False,
        "reject_fallback_when_carla_history_available": True,
        "max_abs_rel_left_m": 6.0,
    })
    classification: Dict[str, Any] = field(default_factory=lambda: {
        "min_prediction_response_time_sec": 0.20,
        "min_planning_response_time_sec": 0.30,
        "min_effective_constraint_time_sec": 0.50,
        "max_perception_gap_sec": 0.50,
        "min_type_stable_ratio": 0.70,
        "min_stable_perception_frames": 2,
        "enable_turn_id_switch_reacquire": True,
        "turn_context_heading_change_rad": 0.25,
    })
    same_target: Dict[str, Any] = field(default_factory=lambda: {
        "base_position_error_m": 1.0,
        "velocity_uncertainty_mps": 1.0,
        "max_accel_mps2": 4.0,
        "max_velocity_error_mps": 3.0,
        "max_heading_error_rad": 0.8,
        "min_same_target_score": 0.70,
    })
    carla_target_match: Dict[str, Any] = field(default_factory=lambda: {
        "enabled": True,
        "min_identity_score": 0.60,
        "min_identity_margin": 0.10,
        "min_primary_matched_frames": 2,
        "max_median_position_error_m": 4.0,
        "max_single_position_error_m": 5.0,
        "position_score_scale_m": 3.0,
        "velocity_score_scale_mps": 2.0,
        "heading_score_scale_rad": 0.8,
        "stationary_heading_ignore_speed_mps": 0.5,
        "position_weight": 0.55,
        "velocity_weight": 0.20,
        "heading_weight": 0.10,
        "type_weight": 0.15,
        "planning_aux_weight": 0.05,
        "id_switch_max_gap_sec": 1.20,
        "id_switch_max_position_jump_m": 2.50,
        "alias_min_identity_score": 0.60,
    })

    @classmethod
    def load(cls, path: Optional[Path]) -> "Config":
        cfg = cls()
        if not path or not path.exists() or yaml is None:
            return cfg
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for section in ("analysis", "target_resolution", "classification", "same_target", "carla_target_match"):
            if isinstance(loaded.get(section), dict):
                getattr(cfg, section).update(loaded[section])
        return cfg


@dataclass
class ModuleVerdict:
    module: str
    verdict: str
    reason_code: str
    metrics: Dict[str, Any] = field(default_factory=dict)
    evidence_files: List[str] = field(default_factory=list)


class CaseDataLoader:
    def __init__(self, case_dir: Path, out_dir: Path) -> None:
        self.case_dir = case_dir.resolve()
        self.out_dir = out_dir.resolve()
        self.files = [p for p in self.case_dir.rglob("*") if p.is_file() and not self._is_excluded(p)]
        self._tables: Dict[Path, pd.DataFrame] = {}

    def _is_excluded(self, path: Path) -> bool:
        resolved = path.resolve()
        try:
            resolved.relative_to(self.out_dir)
            return True
        except ValueError:
            pass
        parts = path.relative_to(self.case_dir).parts
        return any(part in {"tools", "tests", "__pycache__"} for part in parts)

    def rel(self, path: Path) -> str:
        return str(path.relative_to(self.case_dir))

    def read_table(self, path: Path) -> Optional[pd.DataFrame]:
        if path in self._tables:
            return self._tables[path].copy()
        try:
            suffix = path.suffix.lower()
            if suffix == ".csv":
                df = pd.read_csv(path, low_memory=False)
            elif suffix in {".jsonl", ".ndjson"}:
                rows = []
                with path.open("r", encoding="utf-8", errors="ignore") as fh:
                    for line in fh:
                        line = line.strip()
                        if line:
                            rows.append(json.loads(line))
                df = pd.DataFrame(rows)
            elif suffix == ".json":
                data = json.loads(path.read_text(encoding="utf-8", errors="ignore"))
                df = pd.DataFrame(data) if isinstance(data, list) else pd.json_normalize(data)
            else:
                return None
            self._tables[path] = df
            return df.copy()
        except Exception as exc:
            logging.debug("Could not read %s: %s", path, exc)
            return None


class TimeHelper:
    def __init__(self, t2: Optional[float] = None) -> None:
        self.t2 = t2

    def csv_time(self, df: pd.DataFrame) -> pd.Series:
        for col in df.columns:
            lower = col.lower()
            if lower.startswith("relative_to_") and lower.endswith("_s"):
                anchor = self._relative_anchor(lower)
                values = pd.to_numeric(df[col], errors="coerce")
                return values + anchor if anchor is not None else values
        col = first_existing(df.columns, ["obs_time", "header_time", "time", "time_sec", "timestamp", "record_time", "apollo_time", "frame_time", "wall_time_unix_ns", "wall_time_iso", "log_time", "data_ts_ns"])
        if not col:
            return pd.Series(np.nan, index=df.index)
        lower = col.lower()
        if lower == "wall_time_iso":
            return df[col].astype(str).map(parse_datetime_to_epoch)
        if lower in {"wall_time_unix_ns", "data_ts_ns"}:
            return pd.to_numeric(df[col], errors="coerce") / 1_000_000_000.0
        if lower == "log_time":
            return df[col].astype(str).map(lambda x: self._log_time_to_epoch(x))
        return pd.to_numeric(df[col], errors="coerce")

    def _relative_anchor(self, col: str) -> Optional[float]:
        match = re.search(r"relative_to_(\d{2})_(\d{2})_(\d{2})_s", col)
        if not match:
            return None
        seconds = int(match.group(1)) * 3600 + int(match.group(2)) * 60 + int(match.group(3))
        if self.t2 and self.t2 > 1e8:
            return self._closest_wall_clock_epoch(float(seconds))
        return float(seconds)

    def _log_time_to_epoch(self, text: str) -> Optional[float]:
        match = re.search(r"(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?", str(text))
        if not match:
            return parse_datetime_to_epoch(str(text))
        seconds = int(match.group(1)) * 3600 + int(match.group(2)) * 60 + int(match.group(3))
        if match.group(4):
            seconds += float(f"0.{match.group(4)}")
        if self.t2 and self.t2 > 1e8:
            return self._closest_wall_clock_epoch(float(seconds))
        return float(seconds)

    def _closest_wall_clock_epoch(self, seconds_since_midnight: float) -> float:
        if not self.t2 or self.t2 <= 1e8:
            return seconds_since_midnight
        utc_midnight = math.floor(self.t2 / 86400.0) * 86400.0
        candidates = []
        for day_shift in (-86400.0, 0.0, 86400.0):
            for offset_hours in range(-14, 15):
                candidates.append(utc_midnight + day_shift + seconds_since_midnight + offset_hours * 3600.0)
        return min(candidates, key=lambda value: abs(value - self.t2))


class CaseArtifacts:
    def __init__(self) -> None:
        self.fusion_frames: List[Dict[str, Any]] = []
        self.fusion_obs: List[Dict[str, Any]] = []
        self.ego_states: List[Dict[str, Any]] = []
        self.planning_evidence: List[Dict[str, Any]] = []
        self.prediction_rows: List[Dict[str, Any]] = []
        self.prediction_inputs: List[Dict[str, Any]] = []
        self.prediction_outputs: List[Dict[str, Any]] = []
        self.planning_input_frames: List[Dict[str, Any]] = []
        self.planning_inputs: List[Dict[str, Any]] = []
        self.planning_decisions: List[Dict[str, Any]] = []
        self.planning_st_boundaries: List[Dict[str, Any]] = []
        self.planning_outputs: List[Dict[str, Any]] = []
        self.internal_planning_events: List[Dict[str, Any]] = []
        self.tag_counts = {tag: 0 for tag in TAGS}
        self.tag_fields: Dict[str, List[str]] = {tag: [] for tag in TAGS}
        self.warnings: List[str] = []


class LogAndTableParser:
    def __init__(self, loader: CaseDataLoader, config: Config, t2: Optional[float]) -> None:
        self.loader = loader
        self.config = config
        self.t2 = t2
        self.time = TimeHelper(t2)
        if t2 is not None:
            pre = max(float(config.analysis["pre_collision_window_sec"]), float(config.analysis["planning_target_window_sec"]))
            post = float(config.analysis["post_collision_window_sec"])
            pad = float(config.analysis["max_time_match_diff_sec"])
            self.parse_start = t2 - pre - pad
            self.parse_end = t2 + post + pad
        else:
            self.parse_start = None
            self.parse_end = None

    def parse(self) -> CaseArtifacts:
        artifacts = CaseArtifacts()
        for path in self.loader.files:
            name = path.name.lower()
            suffix = path.suffix.lower()
            if suffix in {".csv", ".json", ".jsonl"}:
                self._parse_table(path, artifacts)
            elif ".log.info" in name or name.endswith(".log"):
                self._parse_log(path, artifacts)
        self._add_relative_kinematics(artifacts)
        return artifacts

    def _parse_log(self, path: Path, artifacts: CaseArtifacts) -> None:
        rel = self.loader.rel(path)
        rel_lower = rel.lower()
        try:
            with path.open("r", encoding="utf-8", errors="ignore") as fh:
                for line_no, line in enumerate(fh, 1):
                    if self._line_has_supported_tag(line):
                        line_time = parse_apollo_log_time(line, self.t2)
                        parsed = parse_tagged_log_line(line, rel, line_no)
                        if parsed:
                            parsed["_log_time"] = line_time
                        if parsed and self._tagged_row_in_parse_window(parsed):
                            artifacts.tag_counts[parsed["tag"]] += 1
                            artifacts.tag_fields[parsed["tag"]] = sorted(set(artifacts.tag_fields[parsed["tag"]]) | set(parsed))
                            self._route_tagged(parsed, artifacts)
                    if "planning" in rel_lower and self._planning_line_has_signal(line):
                        line_time = parse_apollo_log_time(line, self.t2)
                        if self._time_in_parse_window(line_time):
                            self._parse_planning_log_line(line, rel, line_no, line_time, artifacts)
                    elif "prediction" in rel_lower and self._prediction_line_has_signal(line):
                        line_time = parse_apollo_log_time(line, self.t2)
                        if self._time_in_parse_window(line_time):
                            self._parse_prediction_log_line(line, rel, line_no, line_time, artifacts)
        except Exception as exc:
            artifacts.warnings.append(f"failed_to_read_log:{rel}:{exc}")

    @staticmethod
    def _line_has_supported_tag(line: str) -> bool:
        return any(f"[{tag}]" in line for tag in TAGS)

    @staticmethod
    def _planning_line_has_signal(line: str) -> bool:
        if "Blocking obstacle ID[" in line or "print_STOP" in line:
            return True
        lower = line.lower()
        return any(key in lower for key in ["fallback", "empty trajectory", "invalid trajectory", "replan"])

    @staticmethod
    def _prediction_line_has_signal(line: str) -> bool:
        lower = line.lower()
        return "obstacle" in lower or "perception_id" in lower or re.search(r"\bid\s*[:=\[]\s*[0-9]+", line, flags=re.I) is not None

    def _time_in_parse_window(self, value: Optional[float]) -> bool:
        if self.parse_start is None or self.parse_end is None:
            return True
        return value is not None and self.parse_start <= value <= self.parse_end

    def _tagged_row_in_parse_window(self, row: Dict[str, Any]) -> bool:
        tag = row.get("tag")
        if tag == "FUSION_OBS_FRAME":
            t = to_float(row.get("header_time"))
        elif tag == "FUSION_OBS":
            t = to_float(row.get("obs_time")) or to_float(row.get("header_time"))
        elif tag == "LOCALIZATION_POSE":
            t = to_float(row.get("measurement_time")) or to_float(row.get("header_time"))
        elif tag == "PLANNING_EGO_STATE":
            t = to_float(row.get("vehicle_state_time")) or to_float(row.get("planning_start_time")) or to_float(row.get("localization_measurement_time"))
        else:
            t = self._tagged_time(row)
        return self._time_in_parse_window(t)

    @staticmethod
    def _tagged_time(row: Dict[str, Any]) -> Optional[float]:
        if row.get("tag") in PREDICTION_TAGS | PLANNING_TAGS:
            log_time = to_float(row.get("_log_time"))
            if log_time is not None:
                return log_time
        value = first_present(row, [
            "time",
            "timestamp",
            "header_time",
            "obs_time",
            "prediction_time",
            "prediction_header_time",
            "perception_header_time",
            "planning_start_time",
            "current_time",
            "_log_time",
        ])
        return to_float(value)

    def _route_tagged(self, row: Dict[str, Any], artifacts: CaseArtifacts) -> None:
        tag = row["tag"]
        if tag == "FUSION_OBS_FRAME":
            artifacts.fusion_frames.append(self._with_time(row, ["header_time"]))
        elif tag == "FUSION_OBS":
            artifacts.fusion_obs.append(self._normalize_fusion_obs(row))
        elif tag == "LOCALIZATION_POSE":
            artifacts.ego_states.append(self._normalize_ego(row, "localization"))
        elif tag == "PLANNING_EGO_STATE":
            artifacts.ego_states.append(self._normalize_ego(row, "planning"))
        elif tag == "PREDICTION_INPUT_OBS":
            pred = self._normalize_prediction_obs(row, "input")
            if pred.get("id"):
                artifacts.prediction_inputs.append(pred)
                artifacts.prediction_rows.append(pred)
        elif tag == "PREDICTION_OUTPUT_OBS":
            pred = self._normalize_prediction_obs(row, "output")
            if pred.get("id"):
                artifacts.prediction_outputs.append(pred)
                artifacts.prediction_rows.append(pred)
        elif tag == "PLANNING_INPUT_PRED_FRAME":
            artifacts.planning_input_frames.append(self._normalize_planning_frame(row))
        elif tag == "PLANNING_INPUT_OBS":
            obs = self._normalize_planning_input_obs(row)
            if obs.get("id"):
                artifacts.planning_inputs.append(obs)
                artifacts.planning_evidence.append(dict(obs, planning_evidence_type="planning_input_obs"))
        elif tag == "PLANNING_DECISION":
            decision = self._normalize_planning_decision(row)
            if decision.get("id"):
                artifacts.planning_decisions.append(decision)
                decision_type = str(decision.get("decision_type") or "UNKNOWN").lower()
                artifacts.planning_evidence.append(dict(decision, planning_evidence_type=f"decision_{decision_type}"))
        elif tag == "PLANNING_ST_BOUNDARY":
            boundary = self._normalize_planning_st_boundary(row)
            if boundary.get("id"):
                artifacts.planning_st_boundaries.append(boundary)
                artifacts.planning_evidence.append(dict(boundary, planning_evidence_type="st_boundary"))
        elif tag == "PLANNING_OUTPUT":
            output = self._normalize_planning_output(row)
            artifacts.planning_outputs.append(output)
            if output.get("status_ok") is False or (output.get("trajectory_point_count") == 0):
                artifacts.internal_planning_events.append({
                    "time": output.get("time"),
                    "event": "planning_output_invalid",
                    "raw_line": row.get("raw_line", ""),
                    "source_file": row.get("source_file"),
                    "line_no": row.get("line_no"),
                })

    def _normalize_prediction_obs(self, row: Dict[str, Any], stage: str) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["prediction_stage"] = stage
        out["id"] = normalize_planning_id(first_present(row, ["id", "obstacle_id", "obs_id", "perception_id", "prediction_obstacle_id", "prediction_id"]))
        out["perception_id"] = normalize_planning_id(first_present(row, ["perception_id", "id", "obstacle_id", "obs_id"]))
        out["type"] = normalize_type(first_present(row, ["type", "obstacle_type", "perception_type"]))
        out["pos_x"] = to_float(first_present(row, ["pos_x", "position_x", "x"]))
        out["pos_y"] = to_float(first_present(row, ["pos_y", "position_y", "y"]))
        out["theta"] = to_float(first_present(row, ["theta", "heading"]))
        out["vel_x"] = to_float(first_present(row, ["vel_x", "velocity_x", "vx"]))
        out["vel_y"] = to_float(first_present(row, ["vel_y", "velocity_y", "vy"]))
        out["speed"] = to_float(first_present(row, ["speed", "v"]))
        out["trajectory_count"] = to_int(first_present(row, ["trajectory_count", "traj_count", "num_trajectories", "trajectory_num"]))
        out["trajectory_point_count"] = to_int(first_present(row, ["trajectory_point_count", "traj0_point_count", "traj_point_count", "point_count", "trajectory_points"]))
        out["trajectory_probability"] = to_float(first_present(row, ["trajectory_probability", "traj0_probability", "probability"]))
        out["is_static"] = to_bool(first_present(row, ["pred_is_static", "is_static", "is_status", "prediction_is_static"]))
        out["has_is_static"] = to_bool(first_present(row, ["pred_has_is_static", "has_is_static", "has_static_status"]))
        first_t = to_float(first_present(row, ["traj0_first_t", "first_t", "first_relative_time", "trajectory_first_t"]))
        last_t = to_float(first_present(row, ["traj0_last_t", "last_t", "last_relative_time", "trajectory_last_t"]))
        out["trajectory_first_t"] = first_t
        out["trajectory_last_t"] = last_t
        out["horizon_sec"] = (last_t - first_t) if first_t is not None and last_t is not None else to_float(first_present(row, ["horizon_sec", "prediction_horizon_sec"]))
        return out

    def _normalize_planning_frame(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["prediction_header_time"] = to_float(first_present(row, ["prediction_header_time", "prediction_time", "header_time"]))
        out["input_obstacle_count"] = to_int(first_present(row, ["input_obstacle_count", "prediction_obstacle_count", "obstacle_count", "count"]))
        out["planning_seq"] = first_present(row, ["planning_seq", "seq", "sequence_num"])
        return out

    def _normalize_planning_input_obs(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["id"] = normalize_planning_id(first_present(row, ["id", "obstacle_id", "obs_id", "perception_id", "prediction_obstacle_id"]))
        out["perception_id"] = normalize_planning_id(first_present(row, ["perception_id", "id", "obstacle_id", "obs_id"]))
        out["type"] = normalize_type(first_present(row, ["type", "obstacle_type", "perception_type"]))
        out["is_virtual"] = to_bool(first_present(row, ["is_virtual", "virtual"]))
        out["is_static"] = to_bool(first_present(row, ["is_static", "static"]))
        out["pos_x"] = to_float(first_present(row, ["pos_x", "position_x", "x"]))
        out["pos_y"] = to_float(first_present(row, ["pos_y", "position_y", "y"]))
        out["vel_x"] = to_float(first_present(row, ["vel_x", "velocity_x", "vx"]))
        out["vel_y"] = to_float(first_present(row, ["vel_y", "velocity_y", "vy"]))
        out["trajectory_count"] = to_int(first_present(row, ["trajectory_count", "traj_count", "num_trajectories", "trajectory_num"]))
        out["trajectory_point_count"] = to_int(first_present(row, ["trajectory_point_count", "traj0_point_count", "traj_point_count", "point_count", "trajectory_points"]))
        return out

    def _normalize_planning_decision(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["id"] = normalize_planning_id(first_present(row, ["id", "obstacle_id", "obs_id", "perception_id", "planning_obstacle_id"]))
        decision = first_present(row, ["decision_type", "decision", "longitudinal_decision"])
        out["decision_type"] = normalize_decision_type(decision)
        out["tag_source"] = first_present(row, ["tag", "decision_tag", "source_tag"])
        out["stop_x"] = to_float(first_present(row, ["stop_x", "stop_point_x"]))
        out["stop_y"] = to_float(first_present(row, ["stop_y", "stop_point_y"]))
        out["distance_s"] = to_float(first_present(row, ["distance_s", "stop_distance_s"]))
        return out

    def _normalize_planning_st_boundary(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["id"] = normalize_planning_id(first_present(row, ["id", "obstacle_id", "obs_id", "perception_id", "planning_obstacle_id"]))
        out["boundary_type"] = str(first_present(row, ["boundary_type", "st_boundary_type", "type"]) or "").upper()
        out["source"] = first_present(row, ["source", "boundary_source"])
        out["point_count"] = to_int(first_present(row, ["point_count", "boundary_point_count", "st_point_count"]))
        out["t_min"] = to_float(first_present(row, ["t_min", "min_t"]))
        out["t_max"] = to_float(first_present(row, ["t_max", "max_t"]))
        out["s_min"] = to_float(first_present(row, ["s_min", "min_s"]))
        out["s_max"] = to_float(first_present(row, ["s_max", "max_s"]))
        out["has_prediction_traj"] = to_bool(first_present(row, ["has_prediction_traj", "has_prediction_trajectory"]))
        out["prediction_point_count"] = to_int(first_present(row, ["prediction_point_count", "prediction_traj_point_count"]))
        return out

    def _normalize_planning_output(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(row)
        out["time"] = self._tagged_time(row)
        out["status_ok"] = to_bool(first_present(row, ["status_ok", "ok", "success"]))
        out["trajectory_type"] = first_present(row, ["trajectory_type", "traj_type"])
        out["trajectory_point_count"] = to_int(first_present(row, ["trajectory_point_count", "traj_point_count", "point_count", "trajectory_points"]))
        out["total_time_ms"] = to_float(first_present(row, ["total_time_ms", "latency_ms", "planning_latency_ms"]))
        out["gear"] = first_present(row, ["gear"])
        out["estop"] = to_bool(first_present(row, ["estop", "is_estop"]))
        out["first_v"] = to_float(first_present(row, ["first_v", "first_speed"]))
        out["last_v"] = to_float(first_present(row, ["last_v", "last_speed"]))
        out["min_v"] = to_float(first_present(row, ["min_v", "min_speed"]))
        out["max_abs_decel"] = to_float(first_present(row, ["max_abs_decel", "max_decel"]))
        out["main_stop_reason"] = first_present(row, ["main_stop_reason", "stop_reason"])
        return out

    def _parse_planning_log_line(self, line: str, rel: str, line_no: int, line_time: Optional[float], artifacts: CaseArtifacts) -> None:
        block = re.search(r"Blocking obstacle ID\[\s*([^\]]*?)\s*\]", line)
        if block and is_real_obstacle_id(block.group(1)):
            artifacts.planning_evidence.append({
                "time": line_time,
                "id": normalize_planning_id(block.group(1)),
                "planning_evidence_type": "blocking_obstacle",
                "source_file": rel,
                "line_no": line_no,
            })
        stop = re.search(r"print_STOP_?\[?([A-Za-z0-9_]+)\]?(?:_obs_st_bounds|obs_st_bounds)", line)
        if stop and is_real_obstacle_id(stop.group(1)):
            artifacts.planning_evidence.append({
                "time": line_time,
                "id": normalize_planning_id(stop.group(1)),
                "planning_evidence_type": "print_stop",
                "source_file": rel,
                "line_no": line_no,
            })
        lower = line.lower()
        if any(key in lower for key in ["fallback", "empty trajectory", "invalid trajectory", "replan"]):
            artifacts.internal_planning_events.append({
                "time": line_time,
                "event": "planning_internal_log",
                "raw_line": line.strip(),
                "source_file": rel,
                "line_no": line_no,
            })

    def _parse_prediction_log_line(self, line: str, rel: str, line_no: int, line_time: Optional[float], artifacts: CaseArtifacts) -> None:
        ids = re.findall(r"(?:obstacle id|obstacle_id|perception_id|id)\s*[:=\[]\s*([0-9]+)", line, flags=re.I)
        for obs_id in ids:
            artifacts.prediction_rows.append({
                "time": line_time,
                "id": normalize_planning_id(obs_id),
                "source_file": rel,
                "line_no": line_no,
            })

    def _parse_table(self, path: Path, artifacts: CaseArtifacts) -> None:
        df = self.loader.read_table(path)
        if df is None or df.empty:
            return
        rel = self.loader.rel(path)
        lower = rel.lower()
        df = df.copy()
        df["_time"] = self.time.csv_time(df)
        df = self._filter_table_to_parse_window(df)
        if df.empty:
            return
        if "fusion_obs_aligned" in lower or ("obs_id" in df.columns and "obs_tracking_time" in df.columns):
            self._parse_fusion_table(df, rel, artifacts)
        if "planning" in lower:
            self._parse_planning_table(df, rel, artifacts)
        if "prediction" in lower or "handoff" in lower or "propagation" in lower:
            self._parse_prediction_table(df, rel, artifacts)

    def _filter_table_to_parse_window(self, df: pd.DataFrame) -> pd.DataFrame:
        if self.parse_start is None or self.parse_end is None or "_time" not in df.columns:
            return df
        times = pd.to_numeric(df["_time"], errors="coerce")
        return df[(times >= self.parse_start) & (times <= self.parse_end)].copy()

    def _parse_fusion_table(self, df: pd.DataFrame, rel: str, artifacts: CaseArtifacts) -> None:
        for _, row in df.iterrows():
            if pd.notna(row.get("obstacle_count")):
                artifacts.fusion_frames.append({
                    "tag": "FUSION_OBS_FRAME_CSV",
                    "time": to_float(row.get("header_time")) or to_float(row.get("frame_time")) or to_float(row.get("_time")),
                    "seq": json_safe(row.get("seq")),
                    "header_time": json_safe(row.get("header_time")),
                    "obstacle_count": json_safe(row.get("obstacle_count")),
                    "source_file": rel,
                })
            if not is_real_obstacle_id(row.get("obs_id")):
                continue
            artifacts.fusion_obs.append({
                "tag": "FUSION_OBS_CSV",
                "time": to_float(row.get("obs_time")) or to_float(row.get("header_time")) or to_float(row.get("_time")),
                "obs_time": to_float(row.get("obs_time")) or to_float(row.get("_time")),
                "output_time": to_float(row.get("header_time")) or to_float(row.get("_time")),
                "log_time": to_float(row.get("log_time")),
                "id": normalize_planning_id(row.get("obs_id")),
                "type": normalize_type(row.get("obs_type")),
                "pos_x": to_float(row.get("pos_x")),
                "pos_y": to_float(row.get("pos_y")),
                "pos_z": to_float(row.get("pos_z")),
                "theta": to_float(row.get("theta")),
                "vel_x": to_float(row.get("vel_x")),
                "vel_y": to_float(row.get("vel_y")),
                "vel_z": to_float(row.get("vel_z")),
                "speed": to_float(row.get("speed")),
                "length": to_float(row.get("obs_length")),
                "width": to_float(row.get("obs_width")),
                "height": to_float(row.get("obs_height")),
                "tracking_time": to_float(row.get("obs_tracking_time")),
                "confidence": to_float(row.get("obs_confidence")),
                "source_file": rel,
            })

    def _parse_planning_table(self, df: pd.DataFrame, rel: str, artifacts: CaseArtifacts) -> None:
        for _, row in df.iterrows():
            row_dict = dict(row)
            row_dict["source_file"] = rel
            row_dict["time"] = to_float(row.get("_time"))
            source_fields = [
                ("stop_id", "stop_id"),
                ("blocking_obstacle_id", "blocking_obstacle"),
                ("planning_obstacle_id", "planning_obstacle"),
                ("perception_id", "planning_perception_id"),
                ("obstacle_id", "planning_obstacle"),
            ]
            for col, evidence_type in source_fields:
                if col in row and is_real_obstacle_id(row.get(col)):
                    artifacts.planning_evidence.append({
                        "time": to_float(row.get("_time")),
                        "id": normalize_planning_id(row.get(col)),
                        "planning_evidence_type": evidence_type,
                        "source_file": rel,
                        "planning_seq": json_safe(row.get("planning_seq")),
                    })
            decision = normalize_decision_type(row.get("decision", row.get("decision_type", row.get("longitudinal_decision", ""))))
            st_type = str(row.get("st_boundary_type", "")).upper()
            if (decision in LONGITUDINAL or st_type in LONGITUDINAL) and any(col in row for col, _ in source_fields):
                for col, _ in source_fields:
                    if col in row and is_real_obstacle_id(row.get(col)):
                        artifacts.planning_evidence.append({
                            "time": to_float(row.get("_time")),
                            "id": normalize_planning_id(row.get(col)),
                            "planning_evidence_type": f"{decision or st_type}_constraint",
                            "source_file": rel,
                        })
            input_id_col = first_existing(df.columns, ["planning_obstacle_id", "perception_id", "obstacle_id", "id"])
            if input_id_col and is_real_obstacle_id(row.get(input_id_col)):
                planning_input = self._normalize_planning_input_obs(dict(row_dict, id=row.get(input_id_col)))
                artifacts.planning_inputs.append(planning_input)
            if decision in LONGITUDINAL or decision == "IGNORE":
                decision_id_col = input_id_col
                if decision_id_col and is_real_obstacle_id(row.get(decision_id_col)):
                    artifacts.planning_decisions.append(self._normalize_planning_decision(dict(row_dict, id=row.get(decision_id_col), decision_type=decision)))
            if st_type or first_existing(df.columns, ["st_boundary_type", "boundary_type", "point_count", "boundary_point_count"]):
                st_id_col = input_id_col
                if st_id_col and is_real_obstacle_id(row.get(st_id_col)):
                    artifacts.planning_st_boundaries.append(self._normalize_planning_st_boundary(dict(row_dict, id=row.get(st_id_col), boundary_type=st_type)))
            if first_existing(df.columns, ["trajectory_point_count", "traj_point_count", "trajectory_points", "trajectory_type", "status_ok"]):
                artifacts.planning_outputs.append(self._normalize_planning_output(row_dict))
            raw = " ".join(str(row.get(col, "")) for col in ["raw_message", "task_name", "event_type", "trajectory_type"])
            if any(key in raw.lower() for key in ["fallback", "empty trajectory", "invalid trajectory", "replan"]):
                artifacts.internal_planning_events.append({
                    "time": to_float(row.get("_time")),
                    "event": "planning_internal_table",
                    "raw_line": raw,
                    "source_file": rel,
                })

    def _parse_prediction_table(self, df: pd.DataFrame, rel: str, artifacts: CaseArtifacts) -> None:
        id_col = first_existing(df.columns, ["perception_id", "obstacle_id", "id", "target_id", "prediction_id"])
        if not id_col:
            return
        for _, row in df.iterrows():
            if is_real_obstacle_id(row.get(id_col)):
                row_dict = dict(row)
                row_dict["source_file"] = rel
                row_dict["time"] = to_float(row.get("_time"))
                row_dict["id"] = row.get(id_col)
                stage = str(first_present(row_dict, ["prediction_stage", "stage"]) or "").lower()
                pred = self._normalize_prediction_obs(row_dict, "input" if "input" in stage else "output")
                if pred["prediction_stage"] == "input":
                    artifacts.prediction_inputs.append(pred)
                else:
                    artifacts.prediction_outputs.append(pred)
                artifacts.prediction_rows.append(pred)

    def _with_time(self, row: Dict[str, Any], fields: Sequence[str]) -> Dict[str, Any]:
        out = dict(row)
        for field_name in fields:
            value = to_float(out.get(field_name))
            if value is not None:
                out["time"] = value
                break
        return out

    def _normalize_fusion_obs(self, row: Dict[str, Any]) -> Dict[str, Any]:
        out = self._with_time(row, ["obs_time", "header_time"])
        out["obs_time"] = to_float(out.get("obs_time")) or to_float(out.get("time"))
        out["output_time"] = (
            to_float(out.get("header_time"))
            or to_float(out.get("_log_time"))
            or to_float(out.get("time"))
        )
        out["log_time"] = to_float(out.get("_log_time"))
        out["id"] = normalize_planning_id(out.get("id"))
        out["type"] = normalize_type(out.get("type"))
        for old, new in [("pos_x", "pos_x"), ("pos_y", "pos_y"), ("pos_z", "pos_z"), ("vel_x", "vel_x"), ("vel_y", "vel_y"), ("vel_z", "vel_z"), ("speed", "speed"), ("theta", "theta")]:
            out[new] = to_float(out.get(old))
        return out

    def _normalize_ego(self, row: Dict[str, Any], source: str) -> Dict[str, Any]:
        out = dict(row)
        out["ego_source"] = source
        if source == "planning":
            out["time"] = to_float(out.get("vehicle_state_time")) or to_float(out.get("planning_start_time")) or to_float(out.get("localization_measurement_time"))
            out["ego_vx"] = to_float(out.get("loc_vx"))
            out["ego_vy"] = to_float(out.get("loc_vy"))
        else:
            out["time"] = to_float(out.get("measurement_time")) or to_float(out.get("header_time"))
        out["ego_x"] = to_float(out.get("ego_x")) or to_float(out.get("loc_x"))
        out["ego_y"] = to_float(out.get("ego_y")) or to_float(out.get("loc_y"))
        out["ego_heading"] = to_float(out.get("ego_heading")) if "ego_heading" in out else to_float(out.get("heading")) or to_float(out.get("loc_heading"))
        if out.get("ego_vx") is None:
            out["ego_vx"] = to_float(out.get("ego_vx")) or to_float(out.get("loc_vx"))
        if out.get("ego_vy") is None:
            out["ego_vy"] = to_float(out.get("ego_vy")) or to_float(out.get("loc_vy"))
        return out

    def _add_relative_kinematics(self, artifacts: CaseArtifacts) -> None:
        localization = sorted([e for e in artifacts.ego_states if e.get("ego_source") == "localization" and e.get("time") is not None], key=lambda r: r["time"])
        planning = sorted([e for e in artifacts.ego_states if e.get("ego_source") == "planning" and e.get("time") is not None], key=lambda r: r["time"])
        if not localization and not planning:
            if artifacts.fusion_obs:
                artifacts.warnings.append("no_LOCALIZATION_POSE_or_PLANNING_EGO_STATE_for_relative_position")
            return
        for obs in artifacts.fusion_obs:
            t = obs.get("time")
            ego = self._nearest_ego(localization, t) or self._nearest_ego(planning, t)
            if not ego:
                continue
            self._attach_relative(obs, ego)

    def _nearest_ego(self, rows: Sequence[Dict[str, Any]], t: Optional[float]) -> Optional[Dict[str, Any]]:
        if t is None or not rows:
            return None
        max_diff = float(self.config.analysis["max_time_match_diff_sec"])
        best = min(rows, key=lambda r: abs(float(r["time"]) - float(t)))
        return best if abs(float(best["time"]) - float(t)) <= max_diff else None

    @staticmethod
    def _attach_relative(obs: Dict[str, Any], ego: Dict[str, Any]) -> None:
        ox, oy = to_float(obs.get("pos_x")), to_float(obs.get("pos_y"))
        ex, ey = to_float(ego.get("ego_x")), to_float(ego.get("ego_y"))
        heading = to_float(ego.get("ego_heading"))
        if ox is None or oy is None or ex is None or ey is None or heading is None:
            return
        dx, dy = ox - ex, oy - ey
        c, s = math.cos(heading), math.sin(heading)
        obs["ego_x"] = ex
        obs["ego_y"] = ey
        obs["ego_heading"] = heading
        obs["rel_forward"] = c * dx + s * dy
        obs["rel_left"] = -s * dx + c * dy
        dist = math.hypot(dx, dy)
        obs["rel_distance"] = dist
        ovx, ovy = to_float(obs.get("vel_x")), to_float(obs.get("vel_y"))
        evx, evy = to_float(ego.get("ego_vx")), to_float(ego.get("ego_vy"))
        if ovx is None or ovy is None or evx is None or evy is None or dist <= 1e-6:
            return
        dvx, dvy = ovx - evx, ovy - evy
        obs["rel_v_forward"] = c * dvx + s * dvy
        obs["rel_v_left"] = -s * dvx + c * dvy
        closing = -((dx * dvx + dy * dvy) / dist)
        obs["closing_speed"] = closing
        obs["ttc"] = dist / closing if closing > 0 else None


class SchemaInspector:
    def __init__(self, loader: CaseDataLoader, artifacts: CaseArtifacts) -> None:
        self.loader = loader
        self.artifacts = artifacts

    def inspect(self) -> Dict[str, Any]:
        files = []
        for path in self.loader.files:
            df = self.loader.read_table(path)
            rel = self.loader.rel(path)
            columns = list(df.columns) if df is not None else []
            files.append({
                "path": rel,
                "size_bytes": path.stat().st_size,
                "columns": columns,
                "module_guess": self._module_guess(rel, columns),
                "usable_for": self._usable_for(rel, columns),
            })
        return {
            "case_dir": str(self.loader.case_dir),
            "tag_counts": self.artifacts.tag_counts,
            "tag_fields": self.artifacts.tag_fields,
            "parsed_counts": {
                "fusion_frames": len(self.artifacts.fusion_frames),
                "fusion_obs": len(self.artifacts.fusion_obs),
                "ego_states": len(self.artifacts.ego_states),
                "planning_evidence": len(self.artifacts.planning_evidence),
                "prediction_rows": len(self.artifacts.prediction_rows),
                "prediction_inputs": len(self.artifacts.prediction_inputs),
                "prediction_outputs": len(self.artifacts.prediction_outputs),
                "planning_input_frames": len(self.artifacts.planning_input_frames),
                "planning_inputs": len(self.artifacts.planning_inputs),
                "planning_decisions": len(self.artifacts.planning_decisions),
                "planning_st_boundaries": len(self.artifacts.planning_st_boundaries),
                "planning_outputs": len(self.artifacts.planning_outputs),
                "internal_planning_events": len(self.artifacts.internal_planning_events),
            },
            "files": files,
            "warnings": self.artifacts.warnings,
        }

    @staticmethod
    def _module_guess(path: str, columns: Sequence[str]) -> str:
        lower = path.lower()
        cols = {c.lower() for c in columns}
        if "collision" in lower:
            return "collision"
        if "perception" in lower or "fusion" in lower or "obs_id" in cols:
            return "perception"
        if "planning" in lower or {"stop_id", "blocking_obstacle_id"} & cols:
            return "planning"
        if "prediction" in lower:
            return "prediction"
        if "localization" in lower:
            return "localization"
        return "unknown"

    @staticmethod
    def _usable_for(path: str, columns: Sequence[str]) -> List[str]:
        lower = path.lower()
        cols = {c.lower() for c in columns}
        uses = []
        if "collision" in lower:
            uses.append("collision_time")
        if "obs_id" in cols or "fusion_obs" in lower:
            uses.append("fusion_obstacle_timeline")
        if {"stop_id", "blocking_obstacle_id", "raw_message"} & cols or "planning" in lower:
            uses.append("planning_target_evidence")
        if "prediction" in lower:
            uses.append("prediction_target_evidence")
        return uses


class CollisionTimeResolver:
    def __init__(self, loader: CaseDataLoader) -> None:
        self.loader = loader

    def resolve(self, cli_time: Optional[float]) -> Tuple[Optional[float], str, List[str]]:
        if cli_time is not None:
            return float(cli_time), "cli", []
        preferred = ["first_collision_event.json", "collision_events_in_window.csv"]

        def priority(path: Path) -> Tuple[int, str]:
            lower = path.name.lower()
            if path.name in preferred or "collision_events" in lower:
                return 0, str(path)
            return 1, str(path)

        files = sorted(self.loader.files, key=priority)
        for path in files:
            lower = path.name.lower()
            if "actor_history" in lower or "collision_history" in lower:
                continue
            if "collision" not in lower and path.name not in preferred:
                continue
            df = self.loader.read_table(path)
            if df is None or df.empty:
                continue
            row = df.iloc[0]
            for col in ["timestamp", "apollo_time", "record_time", "log_time", "time", "time_sec", "header_time"]:
                if col in row and pd.notna(row[col]):
                    value = to_float(row[col])
                    if value is not None:
                        return value, f"{self.loader.rel(path)}:{col}", []
            if "wall_time_unix_ns" in row and pd.notna(row["wall_time_unix_ns"]):
                return float(row["wall_time_unix_ns"]) / 1_000_000_000.0, f"{self.loader.rel(path)}:wall_time_unix_ns", []
            if "wall_time_iso" in row and pd.notna(row["wall_time_iso"]):
                value = parse_datetime_to_epoch(str(row["wall_time_iso"]))
                if value is not None:
                    return value, f"{self.loader.rel(path)}:wall_time_iso", []
            if "carla_timestamp_sec" in row and pd.notna(row["carla_timestamp_sec"]):
                return float(row["carla_timestamp_sec"]), f"{self.loader.rel(path)}:carla_timestamp_sec", []
        return None, "missing", ["MISSING_COLLISION_TIME"]


class CarlaHistoryTargetMatcher:
    """Resolve the Apollo obstacle id from the CARLA collision actor history.

    CARLA history is treated as physical ground truth. Planning evidence is only
    a small auxiliary score and can never make a geometrically invalid candidate
    pass the hard position gates.
    """

    def __init__(self, loader: CaseDataLoader, artifacts: CaseArtifacts, config: Config, t2: float) -> None:
        self.loader = loader
        self.artifacts = artifacts
        self.config = config
        self.cfg = config.carla_target_match
        self.t2 = t2

    def resolve(self) -> Dict[str, Any]:
        empty = {
            "available": False,
            "resolved": False,
            "selected_target_id": None,
            "physical_target_id_chain": [],
            "confidence": 0.0,
            "method": "carla_history_multi_frame_interpolation",
            "candidates": [],
            "warnings": [],
        }
        if not bool(self.cfg.get("enabled", True)):
            return dict(empty, warnings=["CARLA_HISTORY_MATCH_DISABLED"])

        event = self._collision_event()
        if event is None:
            return dict(empty, warnings=["CARLA_COLLISION_EVENT_NOT_FOUND"])
        history = self._collision_history(event)
        if not history:
            return dict(empty, warnings=["CARLA_COLLISION_OTHER_HISTORY_NOT_FOUND"], collision_event=event)

        matched_by_id: Dict[str, List[Dict[str, Any]]] = {}
        start, end = history[0]["time"], history[-1]["time"]
        for obs in self.artifacts.fusion_obs:
            obs_id = normalize_planning_id(obs.get("id"))
            obs_time = to_float(obs.get("time"))
            pos_x, pos_y = to_float(obs.get("pos_x")), to_float(obs.get("pos_y"))
            if not is_real_obstacle_id(obs_id) or obs_time is None or pos_x is None or pos_y is None:
                continue
            if obs_time < start or obs_time > end:
                continue
            gt = self._interpolate(history, obs_time)
            if gt is None:
                continue
            vel_x, vel_y = to_float(obs.get("vel_x")), to_float(obs.get("vel_y"))
            theta = to_float(obs.get("theta"))
            pos_error = math.hypot(pos_x - gt["pos_x"], pos_y - gt["pos_y"])
            vel_error = None
            if vel_x is not None and vel_y is not None:
                vel_error = math.hypot(vel_x - gt["vel_x"], vel_y - gt["vel_y"])
            obs_speed = math.hypot(vel_x or 0.0, vel_y or 0.0)
            heading_error = None
            heading_speed = float(self.cfg["stationary_heading_ignore_speed_mps"])
            if theta is not None and gt["speed"] >= heading_speed and obs_speed >= heading_speed:
                heading_error = normalize_angle(theta - gt["theta"])
            matched_by_id.setdefault(obs_id, []).append({
                "obs": obs,
                "gt": gt,
                "time": obs_time,
                "position_error_m": pos_error,
                "velocity_error_mps": vel_error,
                "heading_error_rad": heading_error,
            })

        internal_candidates = [self._score_candidate(obs_id, rows, event) for obs_id, rows in matched_by_id.items()]
        internal_candidates.sort(key=lambda row: row["score"], reverse=True)
        qualified = [row for row in internal_candidates if row["primary_qualified"]]
        public_candidates = [self._public_candidate(row) for row in internal_candidates]
        result = dict(
            empty,
            available=True,
            collision_event=event,
            history_source=history[0]["source_file"],
            history_time_range=[start, end],
            history_frame_count=len(history),
            coordinate_transform={
                "x_apollo": "x_carla",
                "y_apollo": "-y_carla",
                "z_apollo": "z_carla",
                "vx_apollo": "vx_carla",
                "vy_apollo": "-vy_carla",
                "heading_apollo": "-yaw_carla*pi/180",
            },
            candidates=public_candidates,
        )
        if not qualified:
            result["warnings"] = ["NO_CARLA_HISTORY_CANDIDATE_PASSED_HARD_GATES"]
            return result

        selected = qualified[0]
        second_score = qualified[1]["score"] if len(qualified) > 1 else 0.0
        margin = selected["score"] - second_score
        if selected["score"] < float(self.cfg["min_identity_score"]) or margin < float(self.cfg["min_identity_margin"]):
            result["warnings"] = [
                "CARLA_HISTORY_CANDIDATE_SCORE_OR_MARGIN_INSUFFICIENT",
                f"best_score={selected['score']:.3f}",
                f"second_score={second_score:.3f}",
            ]
            return result

        chain = self._build_id_chain(selected, internal_candidates, event)
        result.update({
            "resolved": True,
            "selected_target_id": selected["id"],
            "physical_target_id_chain": chain,
            "confidence": selected["score"],
            "score_margin": margin,
            "selected_reason": "CARLA collision actor trajectory matched Apollo FUSION_OBS after time interpolation and coordinate conversion.",
        })
        return result

    def _collision_event(self) -> Optional[Dict[str, Any]]:
        files = sorted(
            [p for p in self.loader.files if "collision_events" in p.name.lower() and "actor_history" not in p.name.lower()],
            key=lambda p: (p.suffix.lower() != ".csv", str(p)),
        )
        for path in files:
            df = self.loader.read_table(path)
            if df is None or df.empty:
                continue
            row = dict(df.iloc[0])
            event_time = to_float(row.get("wall_time_unix_ns"))
            if event_time is not None:
                event_time /= 1_000_000_000.0
            if event_time is None:
                event_time = to_float(first_present(row, ["timestamp", "apollo_time", "time", "header_time"]))
            return {
                "time": event_time,
                "other_actor_id": normalize_planning_id(row.get("other_actor_id")),
                "other_actor_type": str(row.get("other_type_id") or ""),
                "ego_actor_id": normalize_planning_id(row.get("ego_actor_id")),
                "history_path": str(row.get("collision_history_path") or ""),
                "source_file": self.loader.rel(path),
            }
        return None

    def _collision_history(self, event: Dict[str, Any]) -> List[Dict[str, Any]]:
        paths = [p for p in self.loader.files if "collision_actor_history" in p.name.lower() and p.suffix.lower() == ".csv"]
        history_name = Path(str(event.get("history_path") or "")).name
        if history_name:
            paths.sort(key=lambda p: (p.name != history_name, str(p)))
        else:
            paths.sort(key=str)
        other_actor_id = normalize_planning_id(event.get("other_actor_id"))
        for path in paths:
            df = self.loader.read_table(path)
            if df is None or df.empty:
                continue
            rows: List[Dict[str, Any]] = []
            for _, raw in df.iterrows():
                role = str(raw.get("role") or "").strip().lower()
                actor_id = normalize_planning_id(raw.get("actor_id"))
                if role != "other" and (not other_actor_id or actor_id != other_actor_id):
                    continue
                time_ns = to_float(raw.get("wall_time_unix_ns"))
                x, y = to_float(raw.get("location_x")), to_float(raw.get("location_y"))
                if time_ns is None or x is None or y is None:
                    continue
                yaw_deg = to_float(raw.get("rotation_yaw")) or 0.0
                vx = to_float(raw.get("velocity_x")) or 0.0
                vy = to_float(raw.get("velocity_y")) or 0.0
                rows.append({
                    "time": time_ns / 1_000_000_000.0,
                    "pos_x": x,
                    "pos_y": -y,
                    "pos_z": to_float(raw.get("location_z")),
                    "theta": -math.radians(yaw_deg),
                    "vel_x": vx,
                    "vel_y": -vy,
                    "source_file": self.loader.rel(path),
                })
            if rows:
                rows.sort(key=lambda row: row["time"])
                return rows
        return []

    @staticmethod
    def _interpolate(history: Sequence[Dict[str, Any]], value: float) -> Optional[Dict[str, Any]]:
        if not history or value < history[0]["time"] or value > history[-1]["time"]:
            return None
        lo, hi = 0, len(history) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if history[mid]["time"] <= value:
                lo = mid
            else:
                hi = mid - 1
        left = history[lo]
        if lo >= len(history) - 1:
            right, ratio = left, 0.0
        else:
            right = history[lo + 1]
            dt = right["time"] - left["time"]
            ratio = 0.0 if dt <= 0 else (value - left["time"]) / dt
        lerp = lambda key: float(left[key]) + ratio * (float(right[key]) - float(left[key]))
        angle_delta = (float(right["theta"]) - float(left["theta"]) + math.pi) % (2.0 * math.pi) - math.pi
        theta = float(left["theta"]) + ratio * angle_delta
        vel_x, vel_y = lerp("vel_x"), lerp("vel_y")
        return {
            "time": value,
            "pos_x": lerp("pos_x"),
            "pos_y": lerp("pos_y"),
            "pos_z": lerp("pos_z") if left.get("pos_z") is not None and right.get("pos_z") is not None else None,
            "theta": theta,
            "vel_x": vel_x,
            "vel_y": vel_y,
            "speed": math.hypot(vel_x, vel_y),
        }

    def _score_candidate(self, obs_id: str, rows: List[Dict[str, Any]], event: Dict[str, Any]) -> Dict[str, Any]:
        rows.sort(key=lambda row: row["time"])
        position_errors = [float(row["position_error_m"]) for row in rows]
        velocity_errors = [float(row["velocity_error_mps"]) for row in rows if row["velocity_error_mps"] is not None]
        heading_errors = [float(row["heading_error_rad"]) for row in rows if row["heading_error_rad"] is not None]
        types = [normalize_type(row["obs"].get("type")) for row in rows]
        actor_is_vehicle = str(event.get("other_actor_type") or "").lower().startswith("vehicle.")
        type_values = []
        for obs_type in types:
            if actor_is_vehicle:
                type_values.append(1.0 if obs_type == "VEHICLE" else 0.6 if obs_type == "UNKNOWN_MOVABLE" else 0.3 if obs_type.startswith("UNKNOWN") else 0.0)
            else:
                type_values.append(1.0)

        pos_median = float(np.median(position_errors))
        vel_median = float(np.median(velocity_errors)) if velocity_errors else None
        heading_median = float(np.median(heading_errors)) if heading_errors else None
        type_score = float(np.mean(type_values)) if type_values else 0.0
        components = {
            "position": math.exp(-0.5 * (pos_median / float(self.cfg["position_score_scale_m"])) ** 2),
            "type": type_score,
        }
        weights = {
            "position": float(self.cfg["position_weight"]),
            "type": float(self.cfg["type_weight"]),
        }
        if vel_median is not None:
            components["velocity"] = math.exp(-0.5 * (vel_median / float(self.cfg["velocity_score_scale_mps"])) ** 2)
            weights["velocity"] = float(self.cfg["velocity_weight"])
        if heading_median is not None:
            components["heading"] = math.exp(-0.5 * (heading_median / float(self.cfg["heading_score_scale_rad"])) ** 2)
            weights["heading"] = float(self.cfg["heading_weight"])
        identity_score = sum(weights[key] * components[key] for key in components) / max(sum(weights.values()), 1e-9)
        has_planning = any(normalize_planning_id(row.get("id")) == obs_id for row in self.artifacts.planning_evidence)
        planning_score = 1.0 if has_planning else 0.0
        planning_weight = clamp(float(self.cfg["planning_aux_weight"]), 0.0, 0.25)
        score = (1.0 - planning_weight) * identity_score + planning_weight * planning_score
        primary_qualified = (
            len(rows) >= int(self.cfg["min_primary_matched_frames"])
            and pos_median <= float(self.cfg["max_median_position_error_m"])
            and min(position_errors) <= float(self.cfg["max_single_position_error_m"])
        )
        return {
            "id": obs_id,
            "score": float(score),
            "identity_score": float(identity_score),
            "planning_aux_score": planning_score,
            "primary_qualified": primary_qualified,
            "matched_frame_count": len(rows),
            "first_matched_time": rows[0]["time"],
            "last_matched_time": rows[-1]["time"],
            "position_error_m": {
                "min": min(position_errors),
                "median": pos_median,
                "max": max(position_errors),
            },
            "velocity_error_mps_median": vel_median,
            "heading_error_rad_median": heading_median,
            "heading_ignored_for_stationary_target": not heading_errors,
            "vehicle_type_ratio": (types.count("VEHICLE") / len(types)) if types else 0.0,
            "component_scores": components,
            "component_weights_used": weights,
            "_matches": rows,
        }

    @staticmethod
    def _public_candidate(candidate: Dict[str, Any]) -> Dict[str, Any]:
        return {key: value for key, value in candidate.items() if not key.startswith("_")}

    def _build_id_chain(self, selected: Dict[str, Any], candidates: Sequence[Dict[str, Any]], event: Dict[str, Any]) -> List[str]:
        chain = [selected["id"]]
        previous = selected
        remaining = sorted(
            [row for row in candidates if row["id"] != selected["id"] and row["first_matched_time"] > selected["last_matched_time"]],
            key=lambda row: row["first_matched_time"],
        )
        for candidate in remaining:
            if candidate["identity_score"] < float(self.cfg["alias_min_identity_score"]):
                continue
            if candidate["position_error_m"]["median"] > float(self.cfg["max_median_position_error_m"]):
                continue
            gap = candidate["first_matched_time"] - previous["last_matched_time"]
            if gap < 0 or gap > float(self.cfg["id_switch_max_gap_sec"]):
                continue
            old_obs = previous["_matches"][-1]["obs"]
            new_obs = candidate["_matches"][0]["obs"]
            old_x, old_y = to_float(old_obs.get("pos_x")), to_float(old_obs.get("pos_y"))
            new_x, new_y = to_float(new_obs.get("pos_x")), to_float(new_obs.get("pos_y"))
            if None in {old_x, old_y, new_x, new_y}:
                continue
            pred_x = float(old_x) + (to_float(old_obs.get("vel_x")) or 0.0) * gap
            pred_y = float(old_y) + (to_float(old_obs.get("vel_y")) or 0.0) * gap
            if math.hypot(float(new_x) - pred_x, float(new_y) - pred_y) > float(self.cfg["id_switch_max_position_jump_m"]):
                continue
            if str(event.get("other_actor_type") or "").lower().startswith("vehicle.") and candidate["vehicle_type_ratio"] <= 0.0:
                continue
            chain.append(candidate["id"])
            previous = candidate
        return chain


def _observation_time(row: Dict[str, Any]) -> Optional[float]:
    return to_float(row.get("obs_time")) or to_float(row.get("time"))


def _availability_time(row: Dict[str, Any]) -> Optional[float]:
    return (
        to_float(row.get("output_time"))
        or to_float(row.get("header_time"))
        or to_float(row.get("log_time"))
        or to_float(row.get("_log_time"))
        or to_float(row.get("time"))
    )


def same_physical_target_score(old: Dict[str, Any], new: Dict[str, Any], cfg: Config, turn_context: bool = False) -> float:
    t0, t1 = _observation_time(old), _observation_time(new)
    x0, y0, x1, y1 = to_float(old.get("pos_x")), to_float(old.get("pos_y")), to_float(new.get("pos_x")), to_float(new.get("pos_y"))
    vx0, vy0 = to_float(old.get("vel_x")) or 0.0, to_float(old.get("vel_y")) or 0.0
    vx1, vy1 = to_float(new.get("vel_x")) or 0.0, to_float(new.get("vel_y")) or 0.0
    if None in {t0, t1, x0, y0, x1, y1}:
        return 0.0
    dt = max(0.0, float(t1) - float(t0))
    pred_x = float(x0) + vx0 * dt
    pred_y = float(y0) + vy0 * dt
    pos_error = math.hypot(float(x1) - pred_x, float(y1) - pred_y)
    vel_error = math.hypot(vx1 - vx0, vy1 - vy0)
    heading_error = normalize_angle((to_float(new.get("theta")) or 0.0) - (to_float(old.get("theta")) or 0.0))
    allowed = cfg.same_target["base_position_error_m"] + cfg.same_target["velocity_uncertainty_mps"] * dt + 0.5 * cfg.same_target["max_accel_mps2"] * dt * dt
    position_score = clamp(1.0 - pos_error / max(allowed, 1e-6))
    velocity_score = clamp(1.0 - vel_error / cfg.same_target["max_velocity_error_mps"])
    heading_score = clamp(1.0 - heading_error / cfg.same_target["max_heading_error_rad"])
    if turn_context:
        rel_scores: List[float] = []
        for key, scale in (("rel_forward", 10.0), ("rel_left", 4.0), ("rel_distance", 10.0)):
            old_value, new_value = to_float(old.get(key)), to_float(new.get(key))
            if old_value is not None and new_value is not None:
                rel_scores.append(clamp(1.0 - abs(new_value - old_value) / scale))
        old_closing, new_closing = to_float(old.get("closing_speed")), to_float(new.get("closing_speed"))
        if old_closing is not None and new_closing is not None:
            rel_scores.append(1.0 if old_closing == 0 or new_closing == 0 or old_closing * new_closing > 0 else 0.0)
        weighted = 0.65 * position_score + 0.20 * velocity_score + 0.05 * heading_score
        weight_sum = 0.90
        if rel_scores:
            weighted += 0.10 * (sum(rel_scores) / len(rel_scores))
            weight_sum += 0.10
        return weighted / weight_sum
    return 0.60 * position_score + 0.25 * velocity_score + 0.15 * heading_score


def _row_time(row: Dict[str, Any]) -> Optional[float]:
    """Return when a perception result became available to downstream modules."""
    return _availability_time(row)


def _segment_summary(rows: List[Dict[str, Any]], t2: float, cfg: Config, stable_key: str = "stable_to_segment_end") -> Dict[str, Any]:
    times = [_row_time(r) for r in rows if _row_time(r) is not None]
    gaps = [b - a for a, b in zip(times, times[1:])]
    types = [normalize_type(r.get("type")) for r in rows]
    summary = {
        "id": normalize_planning_id(rows[0].get("id")) if rows else None,
        "start_time": min(times) if times else None,
        "end_time": max(times) if times else None,
        "frame_count": len(rows),
        "max_gap_sec": float(max(gaps)) if gaps else 0.0,
        "vehicle_type_ratio": (sum(t == "VEHICLE" for t in types) / len(types)) if types else 0.0,
        stable_key: True,
    }
    if stable_key == "stable_to_t2":
        summary[stable_key] = _stable_to_t2(rows, t2, cfg)
    return summary


def _split_segments(rows: List[Dict[str, Any]], cfg: Config) -> List[List[Dict[str, Any]]]:
    ordered = sorted(
        [r for r in rows if _row_time(r) is not None],
        key=lambda r: _row_time(r) or float("inf"),
    )
    if not ordered:
        return []
    max_gap = float(cfg.classification["max_perception_gap_sec"])
    segments: List[List[Dict[str, Any]]] = [[ordered[0]]]
    for row in ordered[1:]:
        prev_time = _row_time(segments[-1][-1])
        row_time = _row_time(row)
        if prev_time is not None and row_time is not None and row_time - prev_time <= max_gap:
            segments[-1].append(row)
        else:
            segments.append([row])
    return segments


def _segment_has_position_jump(segment: List[Dict[str, Any]], cfg: Config, turn_context: bool = False) -> bool:
    min_score = float(cfg.same_target["min_same_target_score"])
    for old, new in zip(segment, segment[1:]):
        if to_float(old.get("pos_x")) is None or to_float(new.get("pos_x")) is None:
            continue
        if same_physical_target_score(old, new, cfg, turn_context=turn_context) < min_score:
            return True
    return False


def _detect_turn_context(artifacts: CaseArtifacts, start: float, end: float, cfg: Config) -> Dict[str, Any]:
    rows = [
        row for row in artifacts.ego_states
        if (t := to_float(row.get("time"))) is not None
        and start <= t <= end
        and to_float(row.get("ego_heading")) is not None
    ]
    rows.sort(key=lambda row: to_float(row.get("time")) or 0.0)
    threshold = float(cfg.classification.get("turn_context_heading_change_rad", 0.25))
    if len(rows) < 2:
        return {
            "is_turn": False,
            "heading_change_rad": 0.0,
            "cumulative_heading_change_rad": 0.0,
            "threshold_rad": threshold,
            "frame_count": len(rows),
            "source": "ego_heading",
        }
    headings = [to_float(row.get("ego_heading")) or 0.0 for row in rows]
    heading_change = normalize_angle(headings[-1] - headings[0])
    cumulative = sum(normalize_angle(b - a) for a, b in zip(headings, headings[1:]))
    return {
        "is_turn": heading_change >= threshold or cumulative >= threshold,
        "heading_change_rad": heading_change,
        "cumulative_heading_change_rad": cumulative,
        "threshold_rad": threshold,
        "frame_count": len(rows),
        "source": "ego_heading",
    }


def _risk_continuity_score(old: Dict[str, Any], new: Dict[str, Any]) -> float:
    scores: List[float] = []
    for key, scale in (("rel_forward", 10.0), ("rel_left", 4.0), ("rel_distance", 10.0)):
        old_value, new_value = to_float(old.get(key)), to_float(new.get(key))
        if old_value is not None and new_value is not None:
            scores.append(clamp(1.0 - abs(new_value - old_value) / scale))
    old_closing, new_closing = to_float(old.get("closing_speed")), to_float(new.get("closing_speed"))
    if old_closing is not None and new_closing is not None:
        scores.append(1.0 if old_closing == 0 or new_closing == 0 or old_closing * new_closing > 0 else 0.0)
    return sum(scores) / len(scores) if scores else 0.0


def _has_planning_evidence(obs_id: str, artifacts: CaseArtifacts, start: float, end: float) -> bool:
    return any(
        normalize_planning_id(ev.get("id")) == obs_id and (t := to_float(ev.get("time"))) is not None and start <= t <= end
        for ev in artifacts.planning_evidence
    )


def _strong_planning_rows(obs_id: str, artifacts: CaseArtifacts, start: float, end: float) -> List[Dict[str, Any]]:
    strong_tokens = {"stop_id", "blocking_obstacle", "print_stop"}
    rows: List[Dict[str, Any]] = []
    for ev in artifacts.planning_evidence:
        ev_time = to_float(ev.get("time"))
        ev_type = str(ev.get("planning_evidence_type", "")).lower()
        if normalize_planning_id(ev.get("id")) != obs_id or ev_time is None or not (start <= ev_time <= end):
            continue
        decision = str(ev.get("decision_type", "")).upper()
        st_points = to_int(ev.get("point_count"))
        if (
            ev_type in strong_tokens
            or "constraint" in ev_type
            or "stop" in ev_type
            or decision in LONGITUDINAL
            or (ev_type == "st_boundary" and st_points is not None and st_points > 0)
        ):
            rows.append(ev)
    return rows


def _is_relevant_target_obs(row: Dict[str, Any]) -> bool:
    rel_forward = to_float(row.get("rel_forward"))
    rel_left = to_float(row.get("rel_left"))
    rel_distance = to_float(row.get("rel_distance"))
    if rel_forward is None and rel_left is None and rel_distance is None:
        return True
    if rel_forward is not None and rel_forward < -2.0:
        return False
    if rel_left is not None and abs(rel_left) <= 4.0:
        return True
    if rel_distance is not None and rel_distance <= 30.0:
        return True
    return rel_forward is not None and rel_forward >= 0.0


def _target_missing_when_relevant(target_id: str, rows: List[Dict[str, Any]], artifacts: CaseArtifacts, t2: float, cfg: Config) -> bool:
    planning_start = t2 - float(cfg.analysis["planning_target_window_sec"])
    strong_rows = _strong_planning_rows(target_id, artifacts, planning_start, t2)
    if not strong_rows:
        return False
    if not rows:
        return True
    timed_rows = [row for row in rows if _row_time(row) is not None]
    if not timed_rows:
        return True
    latest = max(timed_rows, key=lambda row: _row_time(row) or -1.0)
    latest_time = _row_time(latest)
    if latest_time is None:
        return True
    missing_duration = t2 - latest_time
    if missing_duration <= float(cfg.classification["max_perception_gap_sec"]):
        return False
    planning_after_missing = any((to_float(ev.get("time")) or -1e9) >= latest_time for ev in strong_rows)
    return planning_after_missing and _is_relevant_target_obs(latest)


def _stable_to_t2(rows: List[Dict[str, Any]], t2: float, cfg: Config) -> bool:
    times = [_row_time(r) for r in rows if _row_time(r) is not None]
    min_frames = max(2, int(cfg.classification.get("min_stable_perception_frames", 2)))
    max_gap = float(cfg.classification["max_perception_gap_sec"])
    if len(times) < min_frames or max(times) > t2 or (t2 - max(times)) > max_gap:
        return False
    ordered = sorted(times)
    return all((b - a) <= max_gap for a, b in zip(ordered, ordered[1:]))


def _stable_observation_segment(rows: List[Dict[str, Any]], cfg: Config, min_count: int = 2) -> List[Dict[str, Any]]:
    segments = [segment for segment in _split_segments(rows, cfg) if len(segment) >= min_count]
    if not segments:
        return []
    return max(segments, key=lambda segment: ((_row_time(segment[-1]) or 0.0) - (_row_time(segment[0]) or 0.0), len(segment)))


def _candidate_id_score(candidate_rows: List[Dict[str, Any]], old_obs: Optional[Dict[str, Any]], artifacts: CaseArtifacts, t2: float, cfg: Config, turn_context: bool) -> Tuple[float, float, float, float]:
    first = candidate_rows[0]
    same_score = same_physical_target_score(old_obs, first, cfg, turn_context=turn_context) if old_obs is not None else 0.0
    risk_score = _risk_continuity_score(old_obs, first) if old_obs is not None else 0.0
    stable_score = 1.0 if _stable_to_t2(candidate_rows, t2, cfg) else 0.0
    total = 0.60 * same_score + 0.20 * risk_score + 0.20 * stable_score
    obs_id = normalize_planning_id(first.get("id"))
    first_available = _row_time(first)
    if first_available is not None and _has_planning_evidence(obs_id, artifacts, first_available, t2):
        total += 0.10
    if normalize_type(first.get("type")) == "VEHICLE":
        total += 0.05
    return total, same_score, risk_score, stable_score


def _find_reacquire_candidate(
    old_obs: Dict[str, Any],
    used_ids: set[str],
    artifacts: CaseArtifacts,
    t2: float,
    cfg: Config,
    turn_context: bool,
) -> Tuple[Optional[str], List[Dict[str, Any]], Optional[Dict[str, Any]]]:
    old_time = _row_time(old_obs)
    if old_time is None:
        return None, [], None
    min_same = float(cfg.same_target["min_same_target_score"])
    windows = [(old_time, min(t2, old_time + 2.0)), (old_time, t2)]
    best: Tuple[float, Optional[str], List[Dict[str, Any]], Optional[Dict[str, Any]]] = (-1.0, None, [], None)
    for start, end in windows:
        candidate_ids: set[str] = set()
        for row in artifacts.fusion_obs:
            obs_id = normalize_planning_id(row.get("id"))
            row_time = _row_time(row)
            if obs_id in used_ids or not is_real_obstacle_id(obs_id) or row_time is None or not (start <= row_time <= end):
                continue
            if to_float(row.get("pos_x")) is None or to_float(row.get("pos_y")) is None:
                continue
            candidate_ids.add(obs_id)
        for obs_id in candidate_ids:
            rows = [
                row for row in artifacts.fusion_obs
                if normalize_planning_id(row.get("id")) == obs_id and (row_time := _row_time(row)) is not None and start <= row_time <= t2
            ]
            segments = _split_segments(rows, cfg)
            if not segments:
                continue
            first_segment = segments[0]
            total, same_score, risk_score, stable_score = _candidate_id_score(first_segment, old_obs, artifacts, t2, cfg, turn_context)
            if same_score < min_same:
                continue
            if total > best[0]:
                best = (total, obs_id, first_segment, {
                    "same_score": same_score,
                    "risk_score": risk_score,
                    "stable_score": stable_score,
                    "total_score": total,
                })
        if best[1] is not None:
            return best[1], best[2], best[3]
    return None, [], None


def _initial_candidate_score(obs_id: str, rows: List[Dict[str, Any]], artifacts: CaseArtifacts, t2: float, cfg: Config) -> float:
    if not rows:
        return 0.0
    first_time = _row_time(rows[0])
    planning_start = t2 - float(cfg.analysis["planning_target_window_sec"])
    planning = 1.0 if first_time is not None and _strong_planning_rows(obs_id, artifacts, planning_start, t2) else 0.0
    if planning <= 0.0:
        return 0.0
    risk = 0.0
    latest = rows[-1]
    if (to_float(latest.get("rel_forward")) or -1e9) > 0:
        risk += 0.35
    if abs(to_float(latest.get("rel_left")) or 1e9) < 3.0:
        risk += 0.25
    if (to_float(latest.get("rel_distance")) or 1e9) < 30.0:
        risk += 0.25
    if (to_float(latest.get("closing_speed")) or -1e9) > 0:
        risk += 0.15
    stable = 1.0 if _stable_to_t2(rows, t2, cfg) else 0.0
    vehicle = sum(normalize_type(row.get("type")) == "VEHICLE" for row in rows) / len(rows)
    return 0.45 * planning + 0.30 * risk + 0.20 * stable + 0.05 * vehicle


def _find_initial_candidate(target_id: str, artifacts: CaseArtifacts, t2: float, cfg: Config) -> Tuple[Optional[str], List[Dict[str, Any]]]:
    w1_start = t2 - float(cfg.analysis["pre_collision_window_sec"])
    by_id: Dict[str, List[Dict[str, Any]]] = {}
    for row in artifacts.fusion_obs:
        obs_id = normalize_planning_id(row.get("id"))
        row_time = _row_time(row)
        if obs_id == target_id or not is_real_obstacle_id(obs_id) or row_time is None or not (w1_start <= row_time <= t2):
            continue
        if to_float(row.get("pos_x")) is None or to_float(row.get("pos_y")) is None:
            continue
        by_id.setdefault(obs_id, []).append(row)
    best_id, best_rows, best_score = None, [], 0.0
    for obs_id, rows in by_id.items():
        segments = _split_segments(rows, cfg)
        if not segments:
            continue
        segment = segments[0]
        if not _stable_to_t2(segment, t2, cfg):
            continue
        score = _initial_candidate_score(obs_id, segment, artifacts, t2, cfg)
        if score > best_score:
            best_id, best_rows, best_score = obs_id, segment, score
    return best_id, best_rows


def _switch_debug(old_obs: Dict[str, Any], new_obs: Dict[str, Any], score: float) -> Dict[str, Any]:
    old_time, new_time = _row_time(old_obs), _row_time(new_obs)
    return {
        "from_id": normalize_planning_id(old_obs.get("id")),
        "to_id": normalize_planning_id(new_obs.get("id")),
        "gap_sec": (new_time - old_time) if old_time is not None and new_time is not None else None,
        "same_physical_target_score": score,
        "old_time": old_time,
        "new_time": new_time,
        "old_pos_x": old_obs.get("pos_x"),
        "old_pos_y": old_obs.get("pos_y"),
        "new_pos_x": new_obs.get("pos_x"),
        "new_pos_y": new_obs.get("pos_y"),
        "old_rel_forward": old_obs.get("rel_forward"),
        "old_rel_left": old_obs.get("rel_left"),
        "new_rel_forward": new_obs.get("rel_forward"),
        "new_rel_left": new_obs.get("rel_left"),
    }


def _empty_physical_chain(reason_code: str) -> Dict[str, Any]:
    return {
        "chain_ids": [],
        "segments": [],
        "id_switch": False,
        "switches": [],
        "turn_context": None,
        "id_switch_reacquire_enabled": False,
        "first_seen_time": None,
        "last_seen_time": None,
        "max_chain_gap_sec": None,
        "vehicle_type_ratio": 0.0,
        "stable_to_t2": False,
        "reason_code": reason_code,
        "verdict": "FAIL",
    }


def build_physical_target_chain(
    target_id: str,
    artifacts: CaseArtifacts,
    t2: float,
    config: Config,
    resolved_chain_ids: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
    w1_start = t2 - float(config.analysis["pre_collision_window_sec"])
    max_gap = float(config.classification["max_perception_gap_sec"])
    min_type_ratio = float(config.classification["min_type_stable_ratio"])
    turn_context = _detect_turn_context(artifacts, w1_start, t2, config)
    id_switch_reacquire_enabled = bool(config.classification.get("enable_turn_id_switch_reacquire", True) and turn_context.get("is_turn"))
    target_id = normalize_planning_id(target_id)
    trusted_chain_ids: List[str] = []
    for obs_id in resolved_chain_ids or []:
        normalized = normalize_planning_id(obs_id)
        if is_real_obstacle_id(normalized) and normalized not in trusted_chain_ids:
            trusted_chain_ids.append(normalized)
    if target_id and target_id not in trusted_chain_ids:
        trusted_chain_ids.insert(0, target_id)
    ground_truth_identity = bool(resolved_chain_ids)
    rows = [
        obs for obs in artifacts.fusion_obs
        if normalize_planning_id(obs.get("id")) == target_id and (t := _row_time(obs)) is not None and w1_start <= t <= t2
    ]
    if not rows:
        late_output_rows = [
            obs for obs in artifacts.fusion_obs
            if normalize_planning_id(obs.get("id")) == target_id
            and (obs_time := _observation_time(obs)) is not None
            and obs_time <= t2
            and (output_time := _availability_time(obs)) is not None
            and output_time > t2
        ]
        if late_output_rows:
            missing_reason = "PERCEPTION_OUTPUT_AFTER_COLLISION"
        else:
            missing_reason = "TARGET_MISSING_WHEN_RELEVANT" if _target_missing_when_relevant(target_id, [], artifacts, t2, config) else "TARGET_NOT_IN_FUSION"
        if missing_reason == "PERCEPTION_OUTPUT_AFTER_COLLISION":
            return _empty_physical_chain(missing_reason)
        candidate_id, candidate_rows = (None, [])
        planning_start = t2 - float(config.analysis["planning_target_window_sec"])
        if not _strong_planning_rows(target_id, artifacts, planning_start, t2):
            candidate_id, candidate_rows = _find_initial_candidate(target_id, artifacts, t2, config)
        if not candidate_id or not candidate_rows:
            return _empty_physical_chain(missing_reason)
        target_id = candidate_id
        rows = candidate_rows

    chain_segments: List[List[Dict[str, Any]]] = []
    segment_summaries: List[Dict[str, Any]] = []
    switches: List[Dict[str, Any]] = []
    used_ids = {target_id}
    position_jump = False

    initial_segments = _split_segments(rows, config)
    if not initial_segments:
        return _empty_physical_chain("TARGET_NOT_IN_FUSION")
    current_segment = initial_segments[0]
    chain_segments.append(current_segment)
    if _segment_has_position_jump(current_segment, config, turn_context=bool(turn_context.get("is_turn"))):
        position_jump = True

    if ground_truth_identity:
        for chain_id in trusted_chain_ids:
            if chain_id == target_id:
                continue
            chain_rows = [
                obs for obs in artifacts.fusion_obs
                if normalize_planning_id(obs.get("id")) == chain_id
                and (t := _row_time(obs)) is not None
                and w1_start <= t <= t2
            ]
            segments = _split_segments(chain_rows, config)
            for segment in segments:
                if not segment:
                    continue
                chain_segments.append(segment)
                used_ids.add(chain_id)
                if _segment_has_position_jump(segment, config, turn_context=bool(turn_context.get("is_turn"))):
                    position_jump = True
        chain_segments.sort(key=lambda segment: _row_time(segment[0]) or float("inf"))

    while not ground_truth_identity and id_switch_reacquire_enabled and len(chain_segments) < 3 and not _stable_to_t2(chain_segments[-1], t2, config):
        old_obs = chain_segments[-1][-1]
        new_id, new_segment, score_detail = _find_reacquire_candidate(old_obs, used_ids, artifacts, t2, config, turn_context=True)
        if not new_id or not new_segment or score_detail is None:
            break
        used_ids.add(new_id)
        switch = _switch_debug(old_obs, new_segment[0], score_detail["same_score"])
        switch.update({
            "risk_continuity_score": score_detail["risk_score"],
            "stable_to_t2_score": score_detail["stable_score"],
            "new_id_score": score_detail["total_score"],
        })
        switches.append(switch)
        chain_segments.append(new_segment)
        if _segment_has_position_jump(new_segment, config, turn_context=True):
            position_jump = True

    for idx, segment in enumerate(chain_segments):
        key = "stable_to_t2" if idx == len(chain_segments) - 1 else "stable_to_segment_end"
        segment_summaries.append(_segment_summary(segment, t2, config, key))

    flat = [row for segment in chain_segments for row in segment]
    times = [_row_time(row) for row in flat if _row_time(row) is not None]
    observation_times = [_observation_time(row) for row in flat if _observation_time(row) is not None]
    chain_ids = [normalize_planning_id(segment[0].get("id")) for segment in chain_segments if segment]
    all_gaps = [b - a for a, b in zip(sorted(times), sorted(times)[1:])]
    if times:
        all_gaps.append(max(0.0, t2 - max(times)))
    types = [normalize_type(row.get("type")) for row in flat]
    vehicle_ratio = (sum(t == "VEHICLE" for t in types) / len(types)) if types else 0.0
    stable_to_t2 = bool(times) and (t2 - max(times)) <= max_gap
    result = {
        "chain_ids": chain_ids,
        "segments": segment_summaries,
        "id_switch": len(chain_ids) > 1,
        "switches": switches,
        "turn_context": turn_context,
        "id_switch_reacquire_enabled": id_switch_reacquire_enabled,
        "first_seen_time": min(times) if times else None,
        "last_seen_time": max(times) if times else None,
        "first_observation_time": min(observation_times) if observation_times else None,
        "last_observation_time": max(observation_times) if observation_times else None,
        "time_semantics": "first/last_seen_time are output availability times",
        "max_chain_gap_sec": float(max(all_gaps)) if all_gaps else 0.0,
        "vehicle_type_ratio": vehicle_ratio,
        "stable_to_t2": stable_to_t2,
        "reason_code": "TARGET_CHAIN_BROKEN_UNEXPLAINED",
        "verdict": "FAIL",
    }

    if not stable_to_t2:
        if ground_truth_identity or _target_missing_when_relevant(target_id, flat, artifacts, t2, config):
            result["reason_code"] = "TARGET_MISSING_WHEN_RELEVANT"
        return result
    if position_jump:
        result["reason_code"] = "FUSION_TARGET_POSITION_JUMP"
        return result
    if vehicle_ratio < min_type_ratio:
        result["reason_code"] = "FUSION_TARGET_TYPE_UNSTABLE"
        return result
    result["verdict"] = "PASS"
    if len(chain_ids) > 1:
        result["reason_code"] = "REACQUIRED_WITH_ID_SWITCH_NON_PERCEPTION_CAUSE"
    elif result["first_seen_time"] is not None and result["first_seen_time"] <= w1_start + max_gap:
        result["reason_code"] = "CONTINUOUS_SINGLE_ID"
    else:
        result["reason_code"] = "LATE_STABLE_DETECTION"
    return result


class TargetResolver:
    def __init__(self, artifacts: CaseArtifacts, config: Config, t2: float, carla_history_match: Optional[Dict[str, Any]] = None) -> None:
        self.artifacts = artifacts
        self.config = config
        self.t2 = t2
        self.carla_history_match = carla_history_match or {}

    def resolve(self, cli_target: Optional[str]) -> Dict[str, Any]:
        if cli_target and is_real_obstacle_id(cli_target):
            selected = normalize_planning_id(cli_target)
            candidate = self._score_candidate(selected, source="cli")
            candidate["score"] = 1.0
            debug = self._debug(selected, "cli", 1.0, "CLI --target-id was provided.", [candidate], [])
            if self.carla_history_match:
                debug["carla_history_match"] = self.carla_history_match
            return debug
        if self.carla_history_match.get("resolved"):
            selected = normalize_planning_id(self.carla_history_match.get("selected_target_id"))
            confidence = float(self.carla_history_match.get("confidence") or 0.0)
            candidate = self._score_candidate(selected, source="carla_history")
            candidate["score"] = confidence
            candidate["target_source"] = "carla_history_multi_frame_interpolation"
            candidate["carla_identity_evidence"] = self.carla_history_match
            debug = self._debug(
                selected,
                "carla_history_multi_frame_interpolation",
                confidence,
                str(self.carla_history_match.get("selected_reason") or "Selected by CARLA collision actor history."),
                [candidate],
                list(self.carla_history_match.get("warnings") or []),
            )
            debug["physical_target_id_chain"] = list(self.carla_history_match.get("physical_target_id_chain") or [selected])
            debug["carla_history_match"] = self.carla_history_match
            return debug
        if (
            self.carla_history_match.get("available")
            and self.config.target_resolution.get(
                "reject_fallback_when_carla_history_available", True
            )
        ):
            warnings = list(self.carla_history_match.get("warnings") or [])
            warnings.append("CARLA_HISTORY_AMBIGUOUS_NO_TARGET_FALLBACK")
            debug = self._debug(
                None,
                "unresolved_carla_history_ambiguous",
                float(self.carla_history_match.get("confidence") or 0.0),
                "CARLA collision history was available but did not resolve a unique Apollo ID; post-collision fallback is forbidden.",
                [],
                warnings,
            )
            debug["carla_history_match"] = self.carla_history_match
            return debug
        candidates = sorted(self._candidate_ids())
        if not candidates:
            debug = self._debug(None, "unresolved", 0.0, "No real numeric candidate id from planning/fusion.", [], ["TARGET_ID_UNRESOLVED"])
            if self.carla_history_match:
                debug["carla_history_match"] = self.carla_history_match
            return debug
        scored = [self._score_candidate(obs_id) for obs_id in candidates]
        scored.sort(key=lambda row: row["score"], reverse=True)
        best = scored[0]
        second_score = scored[1]["score"] if len(scored) > 1 else 0.0
        min_score = self.config.target_resolution["min_score"]
        margin = self.config.target_resolution["min_score_margin"]
        strong_planning = best["planning_evidence"]["from_stop_id"] or best["planning_evidence"]["from_print_stop"] or best["planning_evidence"]["from_blocking_obstacle_id"]
        has_fusion = best["perception_evidence"]["exists_before_collision"]
        planning_required = self.config.target_resolution.get("require_planning_or_cli_target", False)
        has_planning = best["score_detail"]["planning_score"] >= 0.70
        high_geometry_risk = best["score_detail"]["collision_geometry_or_risk_score"] >= 0.70
        good_continuity = best["score_detail"]["perception_continuity_score"] >= 0.60
        relaxed_ok = (
            (strong_planning and has_fusion and best["score"] >= min(0.50, min_score))
            or (has_fusion and high_geometry_risk and has_planning and best["score"] >= 0.40)
            or (has_fusion and high_geometry_risk and good_continuity and best["score"] >= 0.45)
        )
        margin_ok = best["score"] - second_score >= margin or relaxed_ok
        if (best["score"] >= min_score or relaxed_ok) and margin_ok and (has_planning or not planning_required):
            reason = "Selected best fusion-first collision target candidate."
            if relaxed_ok and best["score"] < min_score:
                reason += " Strong pre-collision fusion/geometry evidence allowed relaxed score threshold."
            return self._debug(best["id"], best["target_source"], best["score"], reason, scored, [])
        warnings = ["TARGET_ID_UNRESOLVED", f"best_score={best['score']:.3f}", f"second_score={second_score:.3f}"]
        return self._debug(None, "unresolved", best["score"], "Candidate score or margin was insufficient.", scored, warnings)

    def _candidate_ids(self) -> set[str]:
        ids: set[str] = set()
        pre_start = self.t2 - self.config.analysis["pre_collision_window_sec"]
        planning_start = self.t2 - self.config.analysis["planning_target_window_sec"]
        target_end = self.t2
        for ev in self.artifacts.planning_evidence:
            if is_real_obstacle_id(ev.get("id")) and self._in_time(ev.get("time"), planning_start, target_end):
                ids.add(normalize_planning_id(ev["id"]))
        for obs in self.artifacts.fusion_obs:
            if is_real_obstacle_id(obs.get("id")) and self._in_time(_availability_time(obs), pre_start, target_end):
                ids.add(normalize_planning_id(obs["id"]))
        return ids

    def _score_candidate(self, obs_id: str, source: str = "auto") -> Dict[str, Any]:
        post = self._post_collision_score(obs_id)
        risk = self._risk_score(obs_id)
        perception = self._perception_score(obs_id)
        planning = self._planning_score(obs_id, source)
        type_score = self._type_score(obs_id)
        score = 0.45 * risk[0] + 0.30 * perception[0] + 0.15 * planning[0] + 0.10 * type_score[0]
        if source == "auto" and risk[1].get("lateral_gate_passed") is False:
            score = 0.0
        source_name = self._composite_source(planning[1], risk[0], perception[1], post[1], source)
        return {
            "id": obs_id,
            "target_source": source_name,
            "score": float(score),
            "score_detail": {
                "post_collision_score": post[0],
                "collision_geometry_or_risk_score": risk[0],
                "perception_continuity_score": perception[0],
                "planning_score": planning[0],
                "type_score": type_score[0],
                "weights": {
                    "post_collision_score": 0.0,
                    "collision_geometry_or_risk_score": 0.45,
                    "perception_continuity_score": 0.30,
                    "planning_score": 0.15,
                    "type_score": 0.10,
                },
                "risk_score": risk[0],
                "perception_score": perception[0],
            },
            "planning_evidence": planning[2],
            "perception_evidence": perception[1],
            "risk_evidence": risk[1],
            "type_evidence": type_score[1],
            "post_collision_evidence": post[1],
            "source_files": sorted(set(planning[2].get("source_files", []) + perception[1].get("source_files", []) + risk[1].get("source_files", []) + type_score[1].get("source_files", []) + post[1].get("source_files", []))),
        }

    @staticmethod
    def _composite_source(planning_source: str, risk_score: float, perception: Dict[str, Any], post: Dict[str, Any], fallback_source: str) -> str:
        parts: List[str] = []
        if planning_source != "perception_only":
            parts.append(planning_source)
        if perception.get("exists_before_collision"):
            parts.append("fusion_obs")
        if risk_score > 0:
            parts.append("ego_relative_kinematics")
        if post.get("stable_same_id") or post.get("same_physical_target_score", 0) > 0:
            parts.append("post_collision_check")
        if not parts:
            parts.append(fallback_source)
        return "+".join(parts)

    def _planning_score(self, obs_id: str, source: str) -> Tuple[float, str, Dict[str, Any]]:
        if source == "cli":
            return 1.0, "cli", {"from_cli": True, "from_stop_id": False, "from_blocking_obstacle_id": False, "from_print_stop": False, "first_planning_time": None, "last_planning_time_before_collision": None, "source_files": []}
        start = self.t2 - self.config.analysis["planning_target_window_sec"]
        end = self.t2
        rows = [ev for ev in self.artifacts.planning_evidence if normalize_planning_id(ev.get("id")) == obs_id and self._in_time(ev.get("time"), start, end)]
        types = {str(ev.get("planning_evidence_type", "")) for ev in rows}
        times = [to_float(ev.get("time")) for ev in rows if to_float(ev.get("time")) is not None]
        pre_times = [t for t in times if t <= self.t2]
        detail = {
            "from_cli": False,
            "from_stop_id": "stop_id" in types,
            "from_blocking_obstacle_id": "blocking_obstacle" in types,
            "from_print_stop": "print_stop" in types,
            "from_planning_decision": any(str(ev.get("decision_type", "")).upper() in LONGITUDINAL for ev in rows),
            "from_st_boundary": any(str(ev.get("planning_evidence_type", "")).lower() == "st_boundary" and (to_int(ev.get("point_count")) or 0) > 0 for ev in rows),
            "first_planning_time": min(times) if times else None,
            "last_planning_time_before_collision": max(pre_times) if pre_times else None,
            "last_planning_time_in_target_window": max(times) if times else None,
            "planning_target_window": [start, end],
            "source_files": sorted({ev.get("source_file", "") for ev in rows if ev.get("source_file")}),
        }
        if detail["from_stop_id"] or detail["from_print_stop"]:
            return 1.0, "planning_stop_id", detail
        if detail["from_blocking_obstacle_id"]:
            return 0.95, "planning_blocking_obstacle", detail
        if detail["from_planning_decision"] or detail["from_st_boundary"] or any(t.endswith("_CONSTRAINT") or "constraint" in t for t in types):
            return 0.90, "planning_st_boundary", detail
        if rows:
            return 0.70, "planning_obstacle_id", detail
        return 0.20, "perception_only", detail

    def _risk_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        rows = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(_availability_time(obs), self.t2 - 3.0, self.t2)]
        rows = [r for r in rows if to_float(r.get("rel_distance")) is not None]
        if not rows:
            return 0.0, {"rel_forward": None, "rel_left": None, "rel_distance": None, "closing_speed": None, "ttc": None, "lateral_gate_passed": None, "source_files": []}
        row = max(rows, key=lambda r: _availability_time(r) or -1)
        rel_left = to_float(row.get("rel_left"))
        max_abs_rel_left = float(self.config.target_resolution.get("max_abs_rel_left_m", 6.0))
        lateral_gate_passed = rel_left is None or abs(rel_left) <= max_abs_rel_left
        score = 0.0
        if (to_float(row.get("rel_forward")) or -1e9) > 0:
            score += 0.35
        if rel_left is not None and abs(rel_left) < 3.0:
            score += 0.25
        if (to_float(row.get("rel_distance")) or 1e9) < 30.0:
            score += 0.20
        if (to_float(row.get("closing_speed")) or -1e9) > 0:
            score += 0.10
        ttc = to_float(row.get("ttc"))
        if ttc is not None and ttc < 5.0:
            score += 0.10
        if not lateral_gate_passed:
            score = 0.0
        return min(score, 1.0), {
            "rel_forward": row.get("rel_forward"),
            "rel_left": row.get("rel_left"),
            "rel_distance": row.get("rel_distance"),
            "closing_speed": row.get("closing_speed"),
            "ttc": row.get("ttc"),
            "lateral_gate_passed": lateral_gate_passed,
            "max_abs_rel_left_m": max_abs_rel_left,
            "source_files": [row.get("source_file", "")],
        }

    def _perception_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        rows = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(_availability_time(obs), self.t2 - self.config.analysis["pre_collision_window_sec"], self.t2)]
        times = [_availability_time(r) for r in rows if _availability_time(r) is not None]
        observation_times = [_observation_time(r) for r in rows if _observation_time(r) is not None]
        first = min(times) if times else None
        last = max(times) if times else None
        segments = _split_segments(rows, self.config)
        target_segment = max(segments, key=lambda segment: _row_time(segment[-1]) or -1.0) if segments else []
        stable_to_t2 = _stable_to_t2(target_segment, self.t2, self.config) if target_segment else False
        position_jump = _segment_has_position_jump(target_segment, self.config) if len(target_segment) > 1 else False
        max_gap = 0.0
        if target_segment:
            seg_times = [_row_time(r) for r in target_segment if _row_time(r) is not None]
            gaps = [b - a for a, b in zip(seg_times, seg_times[1:])]
            max_gap = float(max(gaps)) if gaps else 0.0
        if not rows or last is None:
            score = 0.0
        elif stable_to_t2 and not position_jump:
            score = 1.0
        elif stable_to_t2:
            score = 0.65
        elif self.t2 - last <= 1.0:
            score = 0.60
        elif self.t2 - last <= 3.0:
            score = 0.35
        else:
            score = 0.15
        return score, {
            "exists_before_collision": bool(rows),
            "first_seen_time": first,
            "last_seen_time_before_collision": last,
            "first_observation_time": min(observation_times) if observation_times else None,
            "last_observation_time_before_collision": max(observation_times) if observation_times else None,
            "time_semantics": "first/last_seen_time use perception output_time; observation fields use obs_time",
            "stable_from_first_seen_to_t2": stable_to_t2,
            "position_continuity_ok": not position_jump,
            "selected_segment_frame_count": len(target_segment),
            "selected_segment_max_gap_sec": max_gap,
            "source_files": sorted({r.get("source_file", "") for r in rows if r.get("source_file")}),
        }

    def _type_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        start = self.t2 - self.config.analysis["pre_collision_window_sec"]
        end = self.t2
        rows = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(_availability_time(obs), start, end)]
        types = [normalize_type(r.get("type")) for r in rows if r.get("type") is not None]
        if not types:
            return 0.0, {"vehicle_type_ratio": 0.0, "unknown_movable_ratio": 0.0, "dominant_type": None, "source_files": []}
        counts = {t: types.count(t) for t in set(types)}
        vehicle_ratio = counts.get("VEHICLE", 0) / len(types)
        unknown_movable_ratio = counts.get("UNKNOWN_MOVABLE", 0) / len(types)
        unknown_ratio = (counts.get("UNKNOWN", 0) + counts.get("UNKNOWN_UNMOVABLE", 0)) / len(types)
        bicycle_ped_ratio = (counts.get("BICYCLE", 0) + counts.get("PEDESTRIAN", 0)) / len(types)
        score = max(
            vehicle_ratio,
            0.6 * unknown_movable_ratio,
            0.3 * unknown_ratio,
            0.4 * bicycle_ped_ratio,
        )
        dominant_type = max(counts.items(), key=lambda item: item[1])[0]
        return clamp(score), {
            "vehicle_type_ratio": vehicle_ratio,
            "unknown_movable_ratio": unknown_movable_ratio,
            "unknown_ratio": unknown_ratio,
            "bicycle_pedestrian_ratio": bicycle_ped_ratio,
            "dominant_type": dominant_type,
            "source_files": sorted({r.get("source_file", "") for r in rows if r.get("source_file")}),
        }

    def _post_collision_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        start = self.t2
        end = self.t2 + 1.0
        same_id = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(_availability_time(obs), start, end)]
        stable_same = _stable_observation_segment(same_id, self.config)
        if stable_same:
            return 1.0, {
                "stable_same_id": True,
                "post_collision_stable_id": obs_id,
                "same_physical_target_score": 1.0,
                "post_collision_frame_count": len(stable_same),
                "post_collision_window": [start, end],
                "source_files": sorted({r.get("source_file", "") for r in stable_same if r.get("source_file")}),
            }
        before = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(_availability_time(obs), self.t2 - 1.0, self.t2)]
        after = [obs for obs in self.artifacts.fusion_obs if self._in_time(_availability_time(obs), start, end)]
        after_by_id: Dict[str, List[Dict[str, Any]]] = {}
        for row in after:
            row_id = normalize_planning_id(row.get("id"))
            if is_real_obstacle_id(row_id):
                after_by_id.setdefault(row_id, []).append(row)
        best_score, best_id, files, best_count = 0.0, None, [], 0
        if before:
            old = max(before, key=lambda r: _availability_time(r) or -1)
            for row_id, rows in after_by_id.items():
                stable_segment = _stable_observation_segment(rows, self.config)
                if not stable_segment:
                    continue
                new = stable_segment[0]
                score = same_physical_target_score(old, new, self.config)
                if score > best_score:
                    best_score = score
                    best_id = row_id
                    best_count = len(stable_segment)
                    files = [old.get("source_file", "")] + [r.get("source_file", "") for r in stable_segment]
        if best_score >= self.config.same_target["min_same_target_score"]:
            return 0.7, {"stable_same_id": False, "post_collision_stable_id": best_id, "same_physical_target_score": best_score, "post_collision_frame_count": best_count, "post_collision_window": [start, end], "source_files": sorted(set(files))}
        return 0.0, {"stable_same_id": False, "post_collision_stable_id": best_id, "same_physical_target_score": best_score, "post_collision_frame_count": best_count, "post_collision_window": [start, end], "source_files": sorted(set(files))}

    def _debug(self, selected: Optional[str], source: str, confidence: float, reason: str, candidates: List[Dict[str, Any]], warnings: List[str]) -> Dict[str, Any]:
        result = {
            "selected_target_id": selected,
            "target_source": source,
            "confidence": confidence,
            "t2": self.t2,
            "pre_window": [self.t2 - self.config.analysis["pre_collision_window_sec"], self.t2],
            "planning_target_window": [self.t2 - self.config.analysis["planning_target_window_sec"], self.t2],
            "target_post_window": [self.t2, self.t2 + 1.0],
            "post_window": [self.t2, self.t2 + 1.0],
            "selected_reason": reason,
            "candidates": candidates,
            "warnings": warnings,
        }
        if self.carla_history_match:
            result["carla_history_match"] = self.carla_history_match
        return result

    @staticmethod
    def _in_time(value: Any, start: float, end: float) -> bool:
        t = to_float(value)
        return t is not None and start <= t <= end


class Classifier:
    def __init__(self, case_dir: Path, out_dir: Path, config: Config) -> None:
        self.case_dir = case_dir.resolve()
        self.out_dir = out_dir.resolve()
        self.config = config
        self.loader = CaseDataLoader(self.case_dir, self.out_dir)

    def run(self, cli_target: Optional[str], cli_collision_time: Optional[float]) -> Dict[str, Any]:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._remove_obsolete_outputs()
        t2, t2_source, missing = CollisionTimeResolver(self.loader).resolve(cli_collision_time)
        if t2 is None:
            artifacts = CaseArtifacts()
            schema = SchemaInspector(self.loader, artifacts).inspect()
            result = self._unknown("MISSING_COLLISION_TIME", None, None, missing, [], None, {})
            self._write_json("schema_inventory.json", schema)
            self._write_json("target_resolution_debug.json", {})
            self._write_json("classification_result.json", result)
            return result
        artifacts = LogAndTableParser(self.loader, self.config, t2).parse()
        schema = SchemaInspector(self.loader, artifacts).inspect()
        carla_history_match = CarlaHistoryTargetMatcher(self.loader, artifacts, self.config, t2).resolve()
        target_debug = TargetResolver(artifacts, self.config, t2, carla_history_match).resolve(cli_target)
        target_id = target_debug.get("selected_target_id")
        self._write_json("schema_inventory.json", schema)
        self._write_json("target_resolution_debug.json", target_debug)
        if not target_id:
            result = self._unknown("TARGET_ID_UNRESOLVED", t2, t2_source, ["target_id"], artifacts.warnings + target_debug.get("warnings", []), None, target_debug)
            self._write_json("classification_result.json", result)
            return result
        result = self._classify(target_id, t2, t2_source, artifacts, target_debug)
        self._write_json("classification_result.json", result)
        return result

    def _classify(self, target_id: str, t2: float, t2_source: str, artifacts: CaseArtifacts, target_debug: Dict[str, Any]) -> Dict[str, Any]:
        w_start = t2 - self.config.analysis["pre_collision_window_sec"]
        w1 = {"start": w_start, "end": t2, "duration_sec": t2 - w_start}
        module_verdicts: List[ModuleVerdict] = []

        perception = self._check_perception(
            target_id,
            t2,
            artifacts,
            target_debug.get("physical_target_id_chain"),
        )
        module_verdicts.append(perception)
        t1 = perception.metrics.get("first_seen_time")
        chain_ids = perception.metrics.get("id_chain") or [target_id]
        if perception.verdict == "FAIL":
            return self._result("PERCEPTION_ABNORMAL", perception.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

        prediction = self._check_prediction(chain_ids, t2, t1, artifacts)
        module_verdicts.append(prediction)
        if prediction.verdict == "UNKNOWN":
            return self._result("UNKNOWN_OR_DATA_INSUFFICIENT", prediction.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if prediction.verdict == "TOO_LATE":
            return self._result("FUNCTION_NORMAL_BUT_TOO_LATE", prediction.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if prediction.verdict == "FAIL":
            if t1 is not None and t2 - t1 < self.config.classification["min_prediction_response_time_sec"]:
                return self._result("FUNCTION_NORMAL_BUT_TOO_LATE", "PREDICTION_TARGET_TOO_LATE", t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
            return self._result("PREDICTION_ABNORMAL", prediction.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

        planning = self._check_planning(chain_ids, t2, t1, artifacts)
        module_verdicts.append(planning)
        if planning.verdict == "FAIL":
            if planning.reason_code in {"PLANNING_FALLBACK", "FREQUENT_REPLAN", "EMPTY_TRAJECTORY"}:
                return self._result("PLANNING_ABNORMAL", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
            return self._result("PLANNING_ABNORMAL", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if planning.verdict == "TOO_LATE":
            return self._result("FUNCTION_NORMAL_BUT_TOO_LATE", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if planning.verdict == "UNKNOWN":
            return self._result("UNKNOWN_OR_DATA_INSUFFICIENT", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        return self._result("PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING", "FUNCTIONS_PASS_COLLISION_AFTER_PLANNING", t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

    def _check_perception(
        self,
        target_id: str,
        t2: float,
        artifacts: CaseArtifacts,
        resolved_chain_ids: Optional[Sequence[str]] = None,
    ) -> ModuleVerdict:
        chain = build_physical_target_chain(target_id, artifacts, t2, self.config, resolved_chain_ids)
        chain_ids = set(chain.get("chain_ids") or [target_id])
        evidence_rows = [r for r in artifacts.fusion_obs if normalize_planning_id(r.get("id")) in chain_ids]
        metrics = {
            "verdict": chain["verdict"],
            "reason_code": chain["reason_code"],
            "id_chain": chain["chain_ids"],
            "id_switch": chain["id_switch"],
            "segments": chain["segments"],
            "switches": chain["switches"],
            "turn_context": chain.get("turn_context"),
            "id_switch_reacquire_enabled": chain.get("id_switch_reacquire_enabled", False),
            "first_seen_time": chain["first_seen_time"],
            "last_seen_time": chain["last_seen_time"],
            "first_observation_time": chain.get("first_observation_time"),
            "last_observation_time": chain.get("last_observation_time"),
            "time_semantics": chain.get("time_semantics"),
            "max_chain_gap_sec": chain["max_chain_gap_sec"],
            "vehicle_type_ratio": chain["vehicle_type_ratio"],
            "stable_to_t2": chain["stable_to_t2"],
            "note": "length/width/height are debug only and not used as hard identity checks.",
        }
        return ModuleVerdict("perception", chain["verdict"], chain["reason_code"], metrics, self._sources(evidence_rows))

    def _check_prediction(self, target_ids: Sequence[str], t2: float, t1: Optional[float], artifacts: CaseArtifacts) -> ModuleVerdict:
        ids = [normalize_planning_id(obs_id) for obs_id in target_ids if normalize_planning_id(obs_id)]
        id_set = set(ids)
        start = t2 - self.config.analysis["pre_collision_window_sec"]
        inputs = self._rows_for_ids(artifacts.prediction_inputs, id_set, start, t2)
        outputs = self._rows_for_ids(artifacts.prediction_outputs, id_set, start, t2)
        legacy = self._rows_for_ids(artifacts.prediction_rows, id_set, start, t2)
        planning_consumption = self._rows_for_ids(artifacts.planning_inputs, id_set, start, t2)
        planning_consumption += [
            r for r in artifacts.planning_evidence
            if normalize_planning_id(r.get("id")) in id_set
            and self._time_in(r, start, t2)
        ]
        times = [to_float(r.get("time")) for r in inputs + outputs if to_float(r.get("time")) is not None]
        first_seen = min(times) if times else None
        last_seen = max(times) if times else None
        matched_ids = sorted({normalize_planning_id(r.get("id")) for r in inputs + outputs + legacy if normalize_planning_id(r.get("id"))})
        has_detailed_prediction = bool(artifacts.prediction_inputs or artifacts.prediction_outputs)
        has_detailed_planning_input = bool(artifacts.planning_input_frames or artifacts.planning_inputs)
        metrics = {
            "verdict": "UNKNOWN",
            "reason_code": "PREDICTION_DATA_INSUFFICIENT",
            "id_chain": ids,
            "matched_ids": matched_ids,
            "input_valid": bool(inputs or outputs),
            "output_valid": bool(outputs),
            "timing_valid": None,
            "downstream_consumed": bool(planning_consumption) if (has_detailed_planning_input or planning_consumption) else None,
            "trajectory_valid": None,
            "static_output_valid": None,
            "output_semantics_valid": None,
            "first_seen_time": first_seen,
            "last_seen_time": last_seen,
            "input_row_count": len(inputs),
            "output_row_count": len(outputs),
            "legacy_row_count": len(legacy),
            "planning_consumption_count": len(planning_consumption),
        }
        if not has_detailed_prediction:
            if legacy:
                metrics["reason_code"] = "PREDICTION_DATA_INSUFFICIENT"
                return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_DATA_INSUFFICIENT", metrics, self._sources(legacy))
            return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_DATA_INSUFFICIENT", metrics, [])
        if not inputs and not outputs:
            if t1 is not None and t2 - t1 >= self.config.classification["min_prediction_response_time_sec"]:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_MISSING"
                return ModuleVerdict("prediction", "FAIL", "PREDICTION_TARGET_CHAIN_MISSING", metrics, [])
            metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE"
            return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE", metrics, [])
        if not outputs:
            if t1 is not None and t2 - t1 < self.config.classification["min_prediction_response_time_sec"]:
                metrics["verdict"] = "TOO_LATE"
                metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE"
                return ModuleVerdict("prediction", "TOO_LATE", "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE", metrics, self._sources(inputs))
            metrics["verdict"] = "FAIL"
            metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_MISSING"
            return ModuleVerdict("prediction", "FAIL", "PREDICTION_TARGET_CHAIN_MISSING", metrics, self._sources(inputs))
        if first_seen is not None and t2 - first_seen < self.config.classification["min_prediction_response_time_sec"]:
            metrics["verdict"] = "TOO_LATE"
            metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE"
            metrics["timing_valid"] = False
            return ModuleVerdict("prediction", "TOO_LATE", "PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE", metrics, self._sources(inputs + outputs))
        if last_seen is not None and t2 - last_seen > self.config.classification["max_perception_gap_sec"]:
            metrics["verdict"] = "FAIL"
            metrics["reason_code"] = "PREDICTION_TRAJECTORY_STALE"
            metrics["timing_valid"] = False
            return ModuleVerdict("prediction", "FAIL", "PREDICTION_TRAJECTORY_STALE", metrics, self._sources(outputs))
        valid_trajectory = [r for r in outputs if self._prediction_trajectory_valid(r)]
        valid_static = [r for r in outputs if self._prediction_static_output_valid(r)]
        invalid_static_status = [r for r in outputs if self._prediction_static_status_invalid(r)]
        known_empty = [r for r in outputs if self._prediction_trajectory_empty(r)]
        metrics["trajectory_valid"] = bool(valid_trajectory)
        metrics["static_output_valid"] = bool(valid_static)
        metrics["output_semantics_valid"] = bool(valid_trajectory or valid_static)
        metrics["dynamic_trajectory_output_count"] = len(valid_trajectory)
        metrics["static_output_count"] = len(valid_static)
        metrics["invalid_static_status_count"] = len(invalid_static_status)
        if not valid_trajectory and not valid_static:
            if invalid_static_status:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PREDICTION_STATIC_STATUS_INVALID"
                return ModuleVerdict("prediction", "FAIL", "PREDICTION_STATIC_STATUS_INVALID", metrics, self._sources(invalid_static_status))
            if known_empty:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PREDICTION_TRAJECTORY_EMPTY"
                return ModuleVerdict("prediction", "FAIL", "PREDICTION_TRAJECTORY_EMPTY", metrics, self._sources(outputs))
            metrics["reason_code"] = "PREDICTION_DATA_INSUFFICIENT"
            return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_DATA_INSUFFICIENT", metrics, self._sources(outputs))
        horizons = [to_float(r.get("horizon_sec")) for r in valid_trajectory if to_float(r.get("horizon_sec")) is not None]
        if horizons and max(horizons) < self.config.classification["min_planning_response_time_sec"]:
            metrics["verdict"] = "FAIL"
            metrics["reason_code"] = "PREDICTION_TRAJECTORY_STALE"
            return ModuleVerdict("prediction", "FAIL", "PREDICTION_TRAJECTORY_STALE", metrics, self._sources(valid_trajectory))
        if not planning_consumption:
            if has_detailed_planning_input:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PREDICTION_DOWNSTREAM_NOT_CONSUMED"
                metrics["downstream_consumed"] = False
                return ModuleVerdict("prediction", "FAIL", "PREDICTION_DOWNSTREAM_NOT_CONSUMED", metrics, self._sources(outputs))
            metrics["reason_code"] = "PREDICTION_DOWNSTREAM_CONSUMPTION_UNKNOWN"
            return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_DOWNSTREAM_CONSUMPTION_UNKNOWN", metrics, self._sources(outputs))
        metrics["verdict"] = "PASS"
        if valid_static and not valid_trajectory:
            metrics["reason_code"] = "PREDICTION_STATIC_TARGET_VALID"
        else:
            metrics["reason_code"] = "PREDICTION_TARGET_CHAIN_PRESENT"
        metrics["timing_valid"] = True
        metrics["downstream_consumed"] = True
        return ModuleVerdict("prediction", "PASS", metrics["reason_code"], metrics, self._sources(inputs + outputs + planning_consumption))

    def _check_planning(self, target_ids: Sequence[str], t2: float, t1: Optional[float], artifacts: CaseArtifacts) -> ModuleVerdict:
        ids = [normalize_planning_id(obs_id) for obs_id in target_ids if normalize_planning_id(obs_id)]
        id_set = set(ids)
        internal = [r for r in artifacts.internal_planning_events if self._time_in(r, t2 - self.config.analysis["pre_collision_window_sec"], t2)]
        internal_text = " ".join(str(r.get("raw_line", "")) for r in internal).lower()
        if "empty trajectory" in internal_text:
            return ModuleVerdict("planning", "FAIL", "EMPTY_TRAJECTORY", {"verdict": "FAIL", "reason_code": "EMPTY_TRAJECTORY", "id_chain": ids, "internal_event_count": len(internal)}, self._sources(internal))
        if "replan" in internal_text and len(internal) > 3:
            return ModuleVerdict("planning", "FAIL", "FREQUENT_REPLAN", {"verdict": "FAIL", "reason_code": "FREQUENT_REPLAN", "id_chain": ids, "internal_event_count": len(internal)}, self._sources(internal))
        if "fallback" in internal_text:
            return ModuleVerdict("planning", "FAIL", "PLANNING_FALLBACK", {"verdict": "FAIL", "reason_code": "PLANNING_FALLBACK", "id_chain": ids, "internal_event_count": len(internal)}, self._sources(internal))

        start = t2 - self.config.analysis["pre_collision_window_sec"]
        input_rows = self._rows_for_ids(artifacts.planning_inputs, id_set, start, t2)
        decision_rows = self._rows_for_ids(artifacts.planning_decisions, id_set, start, t2)
        st_rows = self._rows_for_ids(artifacts.planning_st_boundaries, id_set, start, t2)
        output_rows = [r for r in artifacts.planning_outputs if self._time_in(r, start, t2)]
        rows = [r for r in artifacts.planning_evidence if normalize_planning_id(r.get("id")) in id_set and self._time_in(r, start, t2)]
        detailed_planning = bool(artifacts.planning_inputs or artifacts.planning_decisions or artifacts.planning_st_boundaries or artifacts.planning_outputs)
        times = [to_float(r.get("time")) for r in input_rows + rows if to_float(r.get("time")) is not None]
        first = min(times) if times else None
        valid_decision_rows = [r for r in decision_rows if str(r.get("decision_type", "")).upper() in LONGITUDINAL]
        ignore_rows = [r for r in decision_rows if str(r.get("decision_type", "")).upper() == "IGNORE"]
        valid_st_rows = [r for r in st_rows if self._st_boundary_valid(r)]
        constraint_rows = valid_decision_rows + valid_st_rows + [
            r for r in rows
            if str(r.get("planning_evidence_type", "")).lower() in {"stop_id", "print_stop"}
            or "constraint" in str(r.get("planning_evidence_type", "")).lower()
        ]
        constraint_times = [to_float(r.get("time")) for r in constraint_rows if to_float(r.get("time")) is not None]
        first_constraint = min(constraint_times) if constraint_times else first
        valid_outputs = [r for r in output_rows if self._planning_output_valid(r)]
        invalid_outputs = [r for r in output_rows if self._planning_output_invalid(r)]
        metrics = {
            "verdict": "UNKNOWN",
            "reason_code": "PLANNING_DATA_INSUFFICIENT",
            "id_chain": ids,
            "matched_ids": sorted({normalize_planning_id(r.get("id")) for r in input_rows + decision_rows + st_rows + rows if normalize_planning_id(r.get("id"))}),
            "input_valid": bool(input_rows or rows),
            "output_valid": bool(valid_outputs),
            "timing_valid": None,
            "downstream_consumed": None,
            "target_planning_evidence_count": len(rows),
            "planning_input_count": len(input_rows),
            "decision_count": len(decision_rows),
            "st_boundary_count": len(st_rows),
            "planning_output_count": len(output_rows),
            "first_planning_time": first,
            "first_constraint_time": first_constraint,
            "has_longitudinal_constraint": bool(constraint_rows),
            "internal_event_count": len(internal),
        }
        if not detailed_planning:
            if rows:
                return ModuleVerdict("planning", "UNKNOWN", "PLANNING_DATA_INSUFFICIENT", metrics, self._sources(rows))
            if not rows and not internal:
                return ModuleVerdict("planning", "UNKNOWN", "PLANNING_DATA_INSUFFICIENT", metrics, [])
        if not input_rows and not rows:
            if t1 is not None and t2 - t1 >= self.config.classification["min_planning_response_time_sec"]:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PLANNING_TARGET_EVIDENCE_MISSING"
                return ModuleVerdict("planning", "FAIL", "PLANNING_TARGET_EVIDENCE_MISSING", metrics, [])
            metrics["verdict"] = "TOO_LATE"
            metrics["reason_code"] = "PLANNING_TARGET_TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_TARGET_TOO_LATE", metrics, [])
        if first is not None and t2 - first < self.config.classification["min_planning_response_time_sec"]:
            metrics["verdict"] = "TOO_LATE"
            metrics["reason_code"] = "PLANNING_TARGET_TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_TARGET_TOO_LATE", metrics, self._sources(rows))
        if first_constraint is not None and t2 - first_constraint < self.config.classification["min_effective_constraint_time_sec"]:
            metrics["verdict"] = "TOO_LATE"
            metrics["reason_code"] = "PLANNING_CONSTRAINT_TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_CONSTRAINT_TOO_LATE", metrics, self._sources(rows + decision_rows + st_rows))
        if decision_rows and not valid_decision_rows and ignore_rows:
            metrics["verdict"] = "FAIL"
            metrics["reason_code"] = "PLANNING_TARGET_IGNORED"
            return ModuleVerdict("planning", "FAIL", "PLANNING_TARGET_IGNORED", metrics, self._sources(ignore_rows))
        if not constraint_rows:
            if decision_rows or st_rows or artifacts.planning_decisions or artifacts.planning_st_boundaries:
                metrics["verdict"] = "FAIL"
                metrics["reason_code"] = "PLANNING_TARGET_NOT_CONSTRAINED"
                return ModuleVerdict("planning", "FAIL", "PLANNING_TARGET_NOT_CONSTRAINED", metrics, self._sources(input_rows + decision_rows + st_rows + rows))
            metrics["reason_code"] = "PLANNING_DECISION_DATA_INSUFFICIENT"
            return ModuleVerdict("planning", "UNKNOWN", "PLANNING_DECISION_DATA_INSUFFICIENT", metrics, self._sources(input_rows + rows))
        if not output_rows:
            metrics["reason_code"] = "PLANNING_OUTPUT_DATA_INSUFFICIENT"
            return ModuleVerdict("planning", "UNKNOWN", "PLANNING_OUTPUT_DATA_INSUFFICIENT", metrics, self._sources(input_rows + decision_rows + st_rows + rows))
        if not valid_outputs:
            metrics["verdict"] = "FAIL"
            metrics["reason_code"] = "EMPTY_TRAJECTORY" if invalid_outputs else "PLANNING_OUTPUT_INVALID"
            return ModuleVerdict("planning", "FAIL", metrics["reason_code"], metrics, self._sources(output_rows))
        metrics["verdict"] = "PASS"
        metrics["reason_code"] = "PLANNING_PASS"
        metrics["input_valid"] = True
        metrics["output_valid"] = True
        metrics["timing_valid"] = True
        metrics["downstream_consumed"] = None
        return ModuleVerdict("planning", "PASS", "PLANNING_PASS", metrics, self._sources(input_rows + decision_rows + st_rows + output_rows + rows))

    def _rows_for_ids(self, rows: Sequence[Dict[str, Any]], id_set: set, start: float, end: float) -> List[Dict[str, Any]]:
        return [
            r for r in rows
            if normalize_planning_id(r.get("id")) in id_set and self._time_in(r, start, end)
        ]

    @staticmethod
    def _prediction_trajectory_valid(row: Dict[str, Any]) -> bool:
        traj_count = to_int(row.get("trajectory_count"))
        point_count = to_int(row.get("trajectory_point_count"))
        return traj_count is not None and traj_count > 0 and point_count is not None and point_count > 0

    @staticmethod
    def _prediction_trajectory_empty(row: Dict[str, Any]) -> bool:
        traj_count = to_int(row.get("trajectory_count"))
        point_count = to_int(row.get("trajectory_point_count"))
        return (traj_count is not None and traj_count <= 0) or (point_count is not None and point_count <= 0)

    @staticmethod
    def _prediction_static_output_valid(row: Dict[str, Any]) -> bool:
        is_static = to_bool(row.get("is_static"))
        has_is_static = to_bool(row.get("has_is_static"))
        return is_static is True and has_is_static is not False

    @staticmethod
    def _prediction_static_status_invalid(row: Dict[str, Any]) -> bool:
        return to_bool(row.get("is_static")) is True and to_bool(row.get("has_is_static")) is False

    @staticmethod
    def _st_boundary_valid(row: Dict[str, Any]) -> bool:
        point_count = to_int(row.get("point_count"))
        return point_count is not None and point_count > 0

    @staticmethod
    def _planning_output_valid(row: Dict[str, Any]) -> bool:
        status_ok = to_bool(row.get("status_ok"))
        estop = to_bool(row.get("estop"))
        point_count = to_int(row.get("trajectory_point_count"))
        return status_ok is not False and estop is not True and point_count is not None and point_count > 0

    @staticmethod
    def _planning_output_invalid(row: Dict[str, Any]) -> bool:
        status_ok = to_bool(row.get("status_ok"))
        estop = to_bool(row.get("estop"))
        point_count = to_int(row.get("trajectory_point_count"))
        return status_ok is False or estop is True or (point_count is not None and point_count <= 0)

    def _has_position_jump(self, rows: List[Dict[str, Any]]) -> bool:
        usable = [r for r in rows if to_float(r.get("pos_x")) is not None and to_float(r.get("pos_y")) is not None]
        usable.sort(key=lambda r: to_float(r.get("time")) or 0.0)
        for old, new in zip(usable, usable[1:]):
            dt = (to_float(new.get("time")) or 0.0) - (to_float(old.get("time")) or 0.0)
            if dt <= 0:
                continue
            score = same_physical_target_score(old, new, self.config)
            if score < self.config.same_target["min_same_target_score"]:
                return True
        return False

    def _result(self, verdict: str, reason: str, t2: float, t2_source: str, target_id: str, w1: Dict[str, Any], target_debug: Dict[str, Any], module_verdicts: List[ModuleVerdict], artifacts: CaseArtifacts) -> Dict[str, Any]:
        by_module = {m.module: m for m in module_verdicts}
        perception_ids = by_module.get("perception").metrics.get("id_chain") if by_module.get("perception") else None
        timeline = self._target_timeline(target_id, artifacts, perception_ids)
        truncated = len(timeline) > 500
        timeline = timeline[:500]
        evidence = sorted({src for m in module_verdicts for src in m.evidence_files} | {src for row in timeline for src in [row.get("source_file")] if src})
        return {
            "case_id": self.case_dir.name,
            "final_verdict": verdict,
            "reason_code": reason,
            "t2_collision_time": t2,
            "collision_time_source": t2_source,
            "target_id": target_id,
            "w1": w1,
            "target_resolution": {
                "target_source": target_debug.get("target_source"),
                "confidence": target_debug.get("confidence"),
                "selected_reason": target_debug.get("selected_reason"),
                "score_detail": self._selected_candidate_field(target_debug, "score_detail"),
                "planning_evidence": self._selected_candidate_field(target_debug, "planning_evidence"),
                "perception_evidence": self._selected_candidate_field(target_debug, "perception_evidence"),
                "risk_evidence": self._selected_candidate_field(target_debug, "risk_evidence"),
                "type_evidence": self._selected_candidate_field(target_debug, "type_evidence"),
                "post_collision_evidence": self._selected_candidate_field(target_debug, "post_collision_evidence"),
            },
            "perception": self._module_metrics(by_module, "perception"),
            "prediction": self._module_metrics(by_module, "prediction"),
            "planning": self._module_metrics(by_module, "planning"),
            "module_verdicts": [asdict(m) for m in module_verdicts],
            "target_timeline": timeline,
            "target_timeline_truncated": truncated,
            "missing_fields": [],
            "evidence_files": evidence,
            "warnings": artifacts.warnings,
            "scope_note": "control and guardian are not analyzed or used in the final verdict.",
        }

    def _unknown(self, reason: str, t2: Optional[float], t2_source: Optional[str], missing: List[str], warnings: List[str], target_id: Optional[str], target_debug: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "case_id": self.case_dir.name,
            "final_verdict": "UNKNOWN_OR_DATA_INSUFFICIENT",
            "reason_code": reason,
            "t2_collision_time": t2,
            "collision_time_source": t2_source,
            "target_id": target_id,
            "w1": None,
            "target_resolution": target_debug,
            "perception": {"verdict": "UNKNOWN"},
            "prediction": {"verdict": "UNKNOWN"},
            "planning": {"verdict": "UNKNOWN"},
            "module_verdicts": [],
            "target_timeline": [],
            "target_timeline_truncated": False,
            "missing_fields": missing,
            "evidence_files": [],
            "warnings": warnings,
            "scope_note": "control and guardian are not analyzed or used in the final verdict.",
        }

    def _target_timeline(self, target_id: str, artifacts: CaseArtifacts, chain_ids: Optional[Sequence[str]] = None) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        perception_ids = {normalize_planning_id(obs_id) for obs_id in (chain_ids or [target_id]) if normalize_planning_id(obs_id)}
        planning_ids = set(perception_ids) | {normalize_planning_id(target_id)}
        for obs in artifacts.fusion_obs:
            obs_id = normalize_planning_id(obs.get("id"))
            if obs_id not in perception_ids:
                continue
            rows.append({
                "time": _availability_time(obs),
                "obs_time": _observation_time(obs),
                "output_time": _availability_time(obs),
                "log_time": to_float(obs.get("log_time")) or to_float(obs.get("_log_time")),
                "module": "perception",
                "id": obs_id,
                "type": normalize_type(obs.get("type")),
                "pos_x": obs.get("pos_x"),
                "pos_y": obs.get("pos_y"),
                "theta": obs.get("theta"),
                "vel_x": obs.get("vel_x"),
                "vel_y": obs.get("vel_y"),
                "speed": obs.get("speed"),
                "ego_x": obs.get("ego_x"),
                "ego_y": obs.get("ego_y"),
                "ego_heading": obs.get("ego_heading"),
                "rel_forward": obs.get("rel_forward"),
                "rel_left": obs.get("rel_left"),
                "rel_distance": obs.get("rel_distance"),
                "closing_speed": obs.get("closing_speed"),
                "ttc": obs.get("ttc"),
                "planning_evidence_type": None,
                "source_file": obs.get("source_file"),
            })
        for pred in artifacts.prediction_inputs + artifacts.prediction_outputs:
            pred_id = normalize_planning_id(pred.get("id"))
            if pred_id not in planning_ids:
                continue
            rows.append({
                "time": pred.get("time"),
                "module": "prediction",
                "id": pred_id,
                "prediction_stage": pred.get("prediction_stage"),
                "type": normalize_type(pred.get("type")),
                "pos_x": pred.get("pos_x"),
                "pos_y": pred.get("pos_y"),
                "theta": pred.get("theta"),
                "vel_x": pred.get("vel_x"),
                "vel_y": pred.get("vel_y"),
                "speed": pred.get("speed"),
                "trajectory_count": pred.get("trajectory_count"),
                "trajectory_point_count": pred.get("trajectory_point_count"),
                "horizon_sec": pred.get("horizon_sec"),
                "source_file": pred.get("source_file"),
            })
        for ev in artifacts.planning_evidence:
            ev_id = normalize_planning_id(ev.get("id"))
            if ev_id not in planning_ids:
                continue
            rows.append({
                "time": ev.get("time"),
                "module": "planning",
                "id": ev_id,
                "type": None,
                "pos_x": ev.get("pos_x"),
                "pos_y": ev.get("pos_y"),
                "theta": None,
                "vel_x": ev.get("vel_x"),
                "vel_y": ev.get("vel_y"),
                "speed": None,
                "ego_x": None,
                "ego_y": None,
                "ego_heading": None,
                "rel_forward": None,
                "rel_left": None,
                "rel_distance": None,
                "closing_speed": None,
                "ttc": None,
                "planning_evidence_type": ev.get("planning_evidence_type"),
                "decision_type": ev.get("decision_type"),
                "boundary_type": ev.get("boundary_type"),
                "point_count": ev.get("point_count"),
                "trajectory_count": ev.get("trajectory_count"),
                "trajectory_point_count": ev.get("trajectory_point_count"),
                "source_file": ev.get("source_file"),
            })
        for out in artifacts.planning_outputs:
            rows.append({
                "time": out.get("time"),
                "module": "planning_output",
                "id": None,
                "status_ok": out.get("status_ok"),
                "trajectory_type": out.get("trajectory_type"),
                "trajectory_point_count": out.get("trajectory_point_count"),
                "total_time_ms": out.get("total_time_ms"),
                "estop": out.get("estop"),
                "max_abs_decel": out.get("max_abs_decel"),
                "source_file": out.get("source_file"),
            })
        rows.sort(key=lambda r: (float(r["time"]) if r.get("time") is not None else float("inf"), r["module"]))
        return rows

    @staticmethod
    def _sources(rows: Sequence[Dict[str, Any]]) -> List[str]:
        return sorted({str(r.get("source_file")) for r in rows if r.get("source_file")})

    @staticmethod
    def _module_metrics(by_module: Dict[str, ModuleVerdict], module: str) -> Dict[str, Any]:
        verdict = by_module.get(module)
        if verdict is None:
            return {"verdict": "UNKNOWN"}
        return verdict.metrics or {"verdict": verdict.verdict, "reason_code": verdict.reason_code}

    @staticmethod
    def _selected_candidate_field(target_debug: Dict[str, Any], field_name: str) -> Any:
        selected = target_debug.get("selected_target_id")
        for candidate in target_debug.get("candidates", []):
            if candidate.get("id") == selected:
                return candidate.get(field_name)
        return None

    @staticmethod
    def _time_in(row: Dict[str, Any], start: float, end: float) -> bool:
        t = to_float(row.get("time"))
        return t is not None and start <= t <= end

    def _write_json(self, name: str, data: Any) -> None:
        (self.out_dir / name).write_text(json.dumps(json_safe(data), ensure_ascii=False, indent=2), encoding="utf-8")

    def _remove_obsolete_outputs(self) -> None:
        for name in ["evidence_report.md", "module_verdicts.csv", "target_timeline.csv"]:
            path = self.out_dir / name
            if path.exists():
                path.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apollo/Carla collision case classifier")
    parser.add_argument("--case-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("collision_classifier_config.yaml"))
    parser.add_argument("--target-id", type=str)
    parser.add_argument("--collision-time", type=float)
    parser.add_argument("--max-window-sec", type=float, default=None, help="Optional override for analysis.pre_collision_window_sec")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")
    cfg = Config.load(args.config)
    if args.max_window_sec is not None:
        cfg.analysis["pre_collision_window_sec"] = float(args.max_window_sec)
    result = Classifier(args.case_dir, args.out_dir, cfg).run(args.target_id, args.collision_time)
    logging.info("Final verdict: %s (%s)", result["final_verdict"], result["reason_code"])
    logging.info("Outputs written to %s", args.out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
