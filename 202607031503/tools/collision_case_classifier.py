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


TAGS = {"FUSION_OBS_FRAME", "FUSION_OBS", "PLANNING_EGO_STATE", "LOCALIZATION_POSE"}
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
    if upper in {"5", "VEHICLE", "CAR", "TRUCK", "BUS"} or upper.startswith("VEHICLE."):
        return "VEHICLE"
    if upper in {"3", "PEDESTRIAN"}:
        return "PEDESTRIAN"
    if upper in {"4", "BICYCLE", "CYCLIST"}:
        return "BICYCLE"
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


def parse_tagged_log_line(line: str, source_file: str = "", line_no: int = 0) -> Optional[Dict[str, Any]]:
    """Parse one tagged Apollo key=value log line."""
    tag_match = re.search(r"\[(FUSION_OBS_FRAME|FUSION_OBS|PLANNING_EGO_STATE|LOCALIZATION_POSE)\]", line)
    if not tag_match:
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
        "pre_collision_window_sec": 5.0,
        "post_collision_window_sec": 2.0,
        "planning_target_window_sec": 3.0,
        "max_time_match_diff_sec": 0.10,
    })
    target_resolution: Dict[str, Any] = field(default_factory=lambda: {
        "min_score": 0.60,
        "min_score_margin": 0.08,
        "require_planning_or_cli_target": True,
    })
    classification: Dict[str, Any] = field(default_factory=lambda: {
        "min_prediction_response_time_sec": 0.20,
        "min_planning_response_time_sec": 0.30,
        "min_effective_constraint_time_sec": 0.50,
        "max_perception_gap_sec": 0.50,
        "min_type_stable_ratio": 0.70,
    })
    same_target: Dict[str, Any] = field(default_factory=lambda: {
        "base_position_error_m": 1.0,
        "velocity_uncertainty_mps": 1.0,
        "max_accel_mps2": 4.0,
        "max_velocity_error_mps": 3.0,
        "max_heading_error_rad": 0.8,
        "min_same_target_score": 0.70,
    })

    @classmethod
    def load(cls, path: Optional[Path]) -> "Config":
        cfg = cls()
        if not path or not path.exists() or yaml is None:
            return cfg
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for section in ("analysis", "target_resolution", "classification", "same_target"):
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
                df = pd.read_csv(path)
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
        try:
            with path.open("r", encoding="utf-8", errors="ignore") as fh:
                for line_no, line in enumerate(fh, 1):
                    parsed = parse_tagged_log_line(line, rel, line_no)
                    if parsed:
                        artifacts.tag_counts[parsed["tag"]] += 1
                        artifacts.tag_fields[parsed["tag"]] = sorted(set(artifacts.tag_fields[parsed["tag"]]) | set(parsed))
                        self._route_tagged(parsed, artifacts)
                    if "planning" in rel.lower():
                        self._parse_planning_log_line(line, rel, line_no, artifacts)
                    elif "prediction" in rel.lower():
                        self._parse_prediction_log_line(line, rel, line_no, artifacts)
        except Exception as exc:
            artifacts.warnings.append(f"failed_to_read_log:{rel}:{exc}")

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

    def _parse_planning_log_line(self, line: str, rel: str, line_no: int, artifacts: CaseArtifacts) -> None:
        line_time = parse_apollo_log_time(line, self.t2)
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

    def _parse_prediction_log_line(self, line: str, rel: str, line_no: int, artifacts: CaseArtifacts) -> None:
        ids = re.findall(r"(?:obstacle id|obstacle_id|perception_id|id)\s*[:=\[]\s*([0-9]+)", line, flags=re.I)
        for obs_id in ids:
            artifacts.prediction_rows.append({
                "time": parse_apollo_log_time(line, self.t2),
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
        if "fusion_obs_aligned" in lower or ("obs_id" in df.columns and "obs_tracking_time" in df.columns):
            self._parse_fusion_table(df, rel, artifacts)
        if "planning" in lower:
            self._parse_planning_table(df, rel, artifacts)
        if "prediction" in lower or "handoff" in lower or "propagation" in lower:
            self._parse_prediction_table(df, rel, artifacts)

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
            decision = str(row.get("decision", row.get("decision_type", row.get("longitudinal_decision", "")))).upper()
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
                artifacts.prediction_rows.append({
                    "time": to_float(row.get("_time")),
                    "id": normalize_planning_id(row.get(id_col)),
                    "source_file": rel,
                })

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
        files = sorted(self.loader.files, key=lambda p: (p.name not in preferred, str(p)))
        for path in files:
            if "collision" not in path.name.lower() and path.name not in preferred:
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


def same_physical_target_score(old: Dict[str, Any], new: Dict[str, Any], cfg: Config) -> float:
    t0, t1 = to_float(old.get("time")), to_float(new.get("time"))
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
    return 0.60 * position_score + 0.25 * velocity_score + 0.15 * heading_score


class TargetResolver:
    def __init__(self, artifacts: CaseArtifacts, config: Config, t2: float) -> None:
        self.artifacts = artifacts
        self.config = config
        self.t2 = t2

    def resolve(self, cli_target: Optional[str]) -> Dict[str, Any]:
        pre = self.config.analysis["pre_collision_window_sec"]
        post = self.config.analysis["post_collision_window_sec"]
        if cli_target and is_real_obstacle_id(cli_target):
            selected = normalize_planning_id(cli_target)
            candidate = self._score_candidate(selected, source="cli")
            candidate["score"] = 1.0
            return self._debug(selected, "cli", 1.0, "CLI --target-id was provided.", [candidate], [])
        candidates = sorted(self._candidate_ids())
        if not candidates:
            return self._debug(None, "unresolved", 0.0, "No real numeric candidate id from planning/fusion.", [], ["TARGET_ID_UNRESOLVED"])
        scored = [self._score_candidate(obs_id) for obs_id in candidates]
        scored.sort(key=lambda row: row["score"], reverse=True)
        best = scored[0]
        second_score = scored[1]["score"] if len(scored) > 1 else 0.0
        min_score = self.config.target_resolution["min_score"]
        margin = self.config.target_resolution["min_score_margin"]
        strong_planning = best["planning_evidence"]["from_stop_id"] or best["planning_evidence"]["from_print_stop"] or best["planning_evidence"]["from_blocking_obstacle_id"]
        has_fusion = best["perception_evidence"]["exists_before_collision"]
        relaxed_ok = strong_planning and has_fusion and best["score"] >= min(0.50, min_score)
        margin_ok = best["score"] - second_score >= margin or relaxed_ok
        planning_required = self.config.target_resolution["require_planning_or_cli_target"]
        has_planning = best["score_detail"]["planning_score"] >= 0.70
        if (best["score"] >= min_score or relaxed_ok) and margin_ok and (has_planning or not planning_required):
            reason = "Selected best planning/fusion candidate."
            if relaxed_ok and best["score"] < min_score:
                reason += " Strong planning stop/blocking evidence plus matching fusion id allowed relaxed score threshold."
            return self._debug(best["id"], best["target_source"], best["score"], reason, scored, [])
        warnings = ["TARGET_ID_UNRESOLVED", f"best_score={best['score']:.3f}", f"second_score={second_score:.3f}"]
        return self._debug(None, "unresolved", best["score"], "Candidate score or margin was insufficient.", scored, warnings)

    def _candidate_ids(self) -> set[str]:
        ids: set[str] = set()
        pre_start = self.t2 - self.config.analysis["pre_collision_window_sec"]
        post_end = self.t2 + self.config.analysis["post_collision_window_sec"]
        for ev in self.artifacts.planning_evidence:
            if is_real_obstacle_id(ev.get("id")) and self._in_time(ev.get("time"), pre_start, self.t2):
                ids.add(normalize_planning_id(ev["id"]))
        for obs in self.artifacts.fusion_obs:
            if is_real_obstacle_id(obs.get("id")) and self._in_time(obs.get("time"), pre_start, post_end):
                ids.add(normalize_planning_id(obs["id"]))
        return ids

    def _score_candidate(self, obs_id: str, source: str = "auto") -> Dict[str, Any]:
        planning = self._planning_score(obs_id, source)
        risk = self._risk_score(obs_id)
        perception = self._perception_score(obs_id)
        post = self._post_collision_score(obs_id)
        score = 0.45 * planning[0] + 0.25 * risk[0] + 0.15 * perception[0] + 0.15 * post[0]
        source_name = planning[1] if planning[1] != "perception_only" else source
        return {
            "id": obs_id,
            "target_source": source_name,
            "score": float(score),
            "score_detail": {
                "planning_score": planning[0],
                "risk_score": risk[0],
                "perception_score": perception[0],
                "post_collision_score": post[0],
            },
            "planning_evidence": planning[2],
            "perception_evidence": perception[1],
            "risk_evidence": risk[1],
            "post_collision_evidence": post[1],
            "source_files": sorted(set(planning[2].get("source_files", []) + perception[1].get("source_files", []) + risk[1].get("source_files", []) + post[1].get("source_files", []))),
        }

    def _planning_score(self, obs_id: str, source: str) -> Tuple[float, str, Dict[str, Any]]:
        if source == "cli":
            return 1.0, "cli", {"from_cli": True, "from_stop_id": False, "from_blocking_obstacle_id": False, "from_print_stop": False, "first_planning_time": None, "last_planning_time_before_collision": None, "source_files": []}
        rows = [ev for ev in self.artifacts.planning_evidence if normalize_planning_id(ev.get("id")) == obs_id and self._in_time(ev.get("time"), self.t2 - self.config.analysis["pre_collision_window_sec"], self.t2)]
        types = {str(ev.get("planning_evidence_type", "")) for ev in rows}
        times = [to_float(ev.get("time")) for ev in rows if to_float(ev.get("time")) is not None]
        detail = {
            "from_cli": False,
            "from_stop_id": "stop_id" in types,
            "from_blocking_obstacle_id": "blocking_obstacle" in types,
            "from_print_stop": "print_stop" in types,
            "first_planning_time": min(times) if times else None,
            "last_planning_time_before_collision": max(times) if times else None,
            "source_files": sorted({ev.get("source_file", "") for ev in rows if ev.get("source_file")}),
        }
        if detail["from_stop_id"] or detail["from_print_stop"]:
            return 1.0, "planning_stop_id", detail
        if detail["from_blocking_obstacle_id"]:
            return 0.95, "planning_blocking_obstacle", detail
        if any(t.endswith("_CONSTRAINT") or "constraint" in t for t in types):
            return 0.90, "planning_st_boundary", detail
        if rows:
            return 0.70, "planning_obstacle_id", detail
        return 0.20, "perception_only", detail

    def _risk_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        rows = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(obs.get("time"), self.t2 - 3.0, self.t2)]
        rows = [r for r in rows if to_float(r.get("rel_distance")) is not None]
        if not rows:
            return 0.0, {"rel_forward": None, "rel_left": None, "rel_distance": None, "closing_speed": None, "ttc": None, "source_files": []}
        row = max(rows, key=lambda r: to_float(r.get("time")) or -1)
        score = 0.0
        if (to_float(row.get("rel_forward")) or -1e9) > 0:
            score += 0.35
        if abs(to_float(row.get("rel_left")) or 1e9) < 3.0:
            score += 0.25
        if (to_float(row.get("rel_distance")) or 1e9) < 30.0:
            score += 0.20
        if (to_float(row.get("closing_speed")) or -1e9) > 0:
            score += 0.10
        ttc = to_float(row.get("ttc"))
        if ttc is not None and ttc < 5.0:
            score += 0.10
        return min(score, 1.0), {
            "rel_forward": row.get("rel_forward"),
            "rel_left": row.get("rel_left"),
            "rel_distance": row.get("rel_distance"),
            "closing_speed": row.get("closing_speed"),
            "ttc": row.get("ttc"),
            "source_files": [row.get("source_file", "")],
        }

    def _perception_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        rows = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(obs.get("time"), self.t2 - self.config.analysis["pre_collision_window_sec"], self.t2)]
        times = [to_float(r.get("time")) for r in rows if to_float(r.get("time")) is not None]
        last = max(times) if times else None
        if last is None:
            score = 0.0
        elif self.t2 - last <= 1.0:
            score = 1.0
        elif self.t2 - last <= 3.0:
            score = 0.7
        else:
            score = 0.4
        types = [normalize_type(r.get("type")) for r in rows if r.get("type") is not None]
        vehicle_ratio = sum(t == "VEHICLE" for t in types) / len(types) if types else 0.0
        if vehicle_ratio > 0:
            score = min(1.0, score + 0.1 * vehicle_ratio)
        return score, {
            "exists_before_collision": bool(rows),
            "first_seen_time": min(times) if times else None,
            "last_seen_time_before_collision": last,
            "vehicle_type_ratio": vehicle_ratio,
            "source_files": sorted({r.get("source_file", "") for r in rows if r.get("source_file")}),
        }

    def _post_collision_score(self, obs_id: str) -> Tuple[float, Dict[str, Any]]:
        start = self.t2 + 0.1
        end = self.t2 + self.config.analysis["post_collision_window_sec"]
        same_id = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(obs.get("time"), start, end)]
        if same_id:
            return 1.0, {"stable_same_id": True, "post_collision_stable_id": obs_id, "same_physical_target_score": 1.0, "source_files": sorted({r.get("source_file", "") for r in same_id if r.get("source_file")})}
        before = [obs for obs in self.artifacts.fusion_obs if normalize_planning_id(obs.get("id")) == obs_id and self._in_time(obs.get("time"), self.t2 - 1.0, self.t2)]
        after = [obs for obs in self.artifacts.fusion_obs if self._in_time(obs.get("time"), start, end)]
        best_score, best_id, files = 0.0, None, []
        if before:
            old = max(before, key=lambda r: to_float(r.get("time")) or -1)
            for new in after:
                score = same_physical_target_score(old, new, self.config)
                if score > best_score:
                    best_score = score
                    best_id = normalize_planning_id(new.get("id"))
                    files = [old.get("source_file", ""), new.get("source_file", "")]
        if best_score >= self.config.same_target["min_same_target_score"]:
            return 0.7, {"stable_same_id": False, "post_collision_stable_id": best_id, "same_physical_target_score": best_score, "source_files": sorted(set(files))}
        return 0.0, {"stable_same_id": False, "post_collision_stable_id": best_id, "same_physical_target_score": best_score, "source_files": sorted(set(files))}

    def _debug(self, selected: Optional[str], source: str, confidence: float, reason: str, candidates: List[Dict[str, Any]], warnings: List[str]) -> Dict[str, Any]:
        return {
            "selected_target_id": selected,
            "target_source": source,
            "confidence": confidence,
            "t2": self.t2,
            "pre_window": [self.t2 - self.config.analysis["pre_collision_window_sec"], self.t2],
            "post_window": [self.t2 + 0.1, self.t2 + self.config.analysis["post_collision_window_sec"]],
            "selected_reason": reason,
            "candidates": candidates,
            "warnings": warnings,
        }

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
        target_debug = TargetResolver(artifacts, self.config, t2).resolve(cli_target)
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
        fusion_rows = sorted([r for r in artifacts.fusion_obs if normalize_planning_id(r.get("id")) == target_id and self._time_in(r, t2 - self.config.analysis["pre_collision_window_sec"], t2)], key=lambda r: r["time"])
        t1 = fusion_rows[0]["time"] if fusion_rows else None
        w_start = max(t1, t2 - self.config.analysis["pre_collision_window_sec"]) if t1 is not None else t2 - self.config.analysis["pre_collision_window_sec"]
        w1 = {"start": w_start, "end": t2, "duration_sec": t2 - w_start}
        module_verdicts: List[ModuleVerdict] = []

        perception = self._check_perception(target_id, t2, fusion_rows)
        module_verdicts.append(perception)
        if perception.verdict == "FAIL":
            return self._result("PERCEPTION_ABNORMAL", perception.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

        prediction = self._check_prediction(target_id, t2, t1, artifacts)
        module_verdicts.append(prediction)
        if prediction.verdict == "FAIL":
            if t1 is not None and t2 - t1 < self.config.classification["min_prediction_response_time_sec"]:
                return self._result("FUNCTION_NORMAL_BUT_TOO_LATE", "PREDICTION_TARGET_TOO_LATE", t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
            return self._result("PREDICTION_ABNORMAL", prediction.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

        planning = self._check_planning(target_id, t2, t1, artifacts)
        module_verdicts.append(planning)
        if planning.verdict == "FAIL":
            if planning.reason_code in {"PLANNING_FALLBACK", "FREQUENT_REPLAN", "EMPTY_TRAJECTORY"}:
                return self._result("PLANNING_ABNORMAL", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
            return self._result("PLANNING_ABNORMAL", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if planning.verdict == "TOO_LATE":
            return self._result("FUNCTION_NORMAL_BUT_TOO_LATE", planning.reason_code, t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        if prediction.verdict == "UNKNOWN":
            return self._result("UNKNOWN_OR_DATA_INSUFFICIENT", "PREDICTION_DATA_INSUFFICIENT", t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)
        return self._result("PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING", "FUNCTIONS_PASS_COLLISION_AFTER_PLANNING", t2, t2_source, target_id, w1, target_debug, module_verdicts, artifacts)

    def _check_perception(self, target_id: str, t2: float, rows: List[Dict[str, Any]]) -> ModuleVerdict:
        if not rows:
            return ModuleVerdict("perception", "FAIL", "TARGET_NOT_IN_FUSION", {"target_id": target_id})
        times = [float(r["time"]) for r in rows if r.get("time") is not None]
        internal_gap = max(np.diff(times)) if len(times) > 1 else 0.0
        terminal_gap = max(0.0, t2 - max(times)) if times else 0.0
        max_gap = max(float(internal_gap), float(terminal_gap))
        types = [normalize_type(r.get("type")) for r in rows]
        type_ratio = sum(t == "VEHICLE" for t in types) / len(types) if types else 0.0
        jump = self._has_position_jump(rows)
        metrics = {
            "verdict": "PASS",
            "first_seen_time": min(times) if times else None,
            "last_seen_time_before_collision": max(times) if times else None,
            "frame_count": len(rows),
            "max_gap_sec": float(max_gap),
            "internal_max_gap_sec": float(internal_gap),
            "terminal_gap_sec": terminal_gap,
            "vehicle_type_ratio": type_ratio,
            "position_jump": jump,
            "note": "length/width/height are debug only and not used as hard identity checks.",
        }
        if max_gap > self.config.classification["max_perception_gap_sec"]:
            metrics["verdict"] = "FAIL"
            return ModuleVerdict("perception", "FAIL", "FUSION_TARGET_GAP_TOO_LARGE", metrics, self._sources(rows))
        if type_ratio < self.config.classification["min_type_stable_ratio"]:
            metrics["verdict"] = "FAIL"
            return ModuleVerdict("perception", "FAIL", "FUSION_TARGET_TYPE_UNSTABLE", metrics, self._sources(rows))
        if jump:
            metrics["verdict"] = "FAIL"
            return ModuleVerdict("perception", "FAIL", "FUSION_TARGET_POSITION_JUMP", metrics, self._sources(rows))
        return ModuleVerdict("perception", "PASS", "PERCEPTION_PASS", metrics, self._sources(rows))

    def _check_prediction(self, target_id: str, t2: float, t1: Optional[float], artifacts: CaseArtifacts) -> ModuleVerdict:
        rows = [r for r in artifacts.prediction_rows if normalize_planning_id(r.get("id")) == target_id]
        rows = [r for r in rows if self._time_in(r, t2 - self.config.analysis["pre_collision_window_sec"], t2)]
        if rows:
            times = [to_float(r.get("time")) for r in rows if to_float(r.get("time")) is not None]
            return ModuleVerdict("prediction", "PASS", "PREDICTION_TARGET_PRESENT", {"first_seen_time": min(times) if times else None, "row_count": len(rows)}, self._sources(rows))
        if not artifacts.prediction_rows:
            return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_DATA_INSUFFICIENT", {"row_count": 0}, [])
        if t1 is not None and t2 - t1 >= self.config.classification["min_prediction_response_time_sec"]:
            return ModuleVerdict("prediction", "FAIL", "PREDICTION_TARGET_MISSING", {"row_count": 0}, [])
        return ModuleVerdict("prediction", "UNKNOWN", "PREDICTION_TARGET_APPEARED_TOO_LATE", {"row_count": 0}, [])

    def _check_planning(self, target_id: str, t2: float, t1: Optional[float], artifacts: CaseArtifacts) -> ModuleVerdict:
        internal = [r for r in artifacts.internal_planning_events if self._time_in(r, t2 - self.config.analysis["pre_collision_window_sec"], t2)]
        internal_text = " ".join(str(r.get("raw_line", "")) for r in internal).lower()
        if "empty trajectory" in internal_text:
            return ModuleVerdict("planning", "FAIL", "EMPTY_TRAJECTORY", {"internal_event_count": len(internal)}, self._sources(internal))
        if "replan" in internal_text and len(internal) > 3:
            return ModuleVerdict("planning", "FAIL", "FREQUENT_REPLAN", {"internal_event_count": len(internal)}, self._sources(internal))
        if "fallback" in internal_text:
            return ModuleVerdict("planning", "FAIL", "PLANNING_FALLBACK", {"internal_event_count": len(internal)}, self._sources(internal))

        rows = [r for r in artifacts.planning_evidence if normalize_planning_id(r.get("id")) == target_id and self._time_in(r, t2 - self.config.analysis["pre_collision_window_sec"], t2)]
        times = [to_float(r.get("time")) for r in rows if to_float(r.get("time")) is not None]
        first = min(times) if times else None
        strong = bool(rows)
        constraint_rows = [r for r in rows if str(r.get("planning_evidence_type", "")).lower() in {"stop_id", "print_stop"} or "constraint" in str(r.get("planning_evidence_type", "")).lower()]
        constraint_times = [to_float(r.get("time")) for r in constraint_rows if to_float(r.get("time")) is not None]
        first_constraint = min(constraint_times) if constraint_times else first
        metrics = {
            "verdict": "PASS",
            "target_planning_evidence_count": len(rows),
            "first_planning_time": first,
            "first_constraint_time": first_constraint,
            "has_longitudinal_constraint": bool(constraint_rows),
            "internal_event_count": len(internal),
        }
        if not strong:
            if t1 is not None and t2 - t1 >= self.config.classification["min_planning_response_time_sec"]:
                metrics["verdict"] = "FAIL"
                return ModuleVerdict("planning", "FAIL", "PLANNING_TARGET_EVIDENCE_MISSING", metrics, [])
            metrics["verdict"] = "TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_TARGET_TOO_LATE", metrics, [])
        if first is not None and t2 - first < self.config.classification["min_planning_response_time_sec"]:
            metrics["verdict"] = "TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_TARGET_TOO_LATE", metrics, self._sources(rows))
        if first_constraint is not None and t2 - first_constraint < self.config.classification["min_effective_constraint_time_sec"]:
            metrics["verdict"] = "TOO_LATE"
            return ModuleVerdict("planning", "TOO_LATE", "PLANNING_CONSTRAINT_TOO_LATE", metrics, self._sources(rows))
        return ModuleVerdict("planning", "PASS", "PLANNING_PASS", metrics, self._sources(rows))

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
        timeline = self._target_timeline(target_id, artifacts)
        truncated = len(timeline) > 500
        timeline = timeline[:500]
        by_module = {m.module: m for m in module_verdicts}
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

    def _target_timeline(self, target_id: str, artifacts: CaseArtifacts) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        for obs in artifacts.fusion_obs:
            if normalize_planning_id(obs.get("id")) != target_id:
                continue
            rows.append({
                "time": obs.get("time"),
                "module": "perception",
                "id": target_id,
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
        for ev in artifacts.planning_evidence:
            if normalize_planning_id(ev.get("id")) != target_id:
                continue
            rows.append({
                "time": ev.get("time"),
                "module": "planning",
                "id": target_id,
                "type": None,
                "pos_x": None,
                "pos_y": None,
                "theta": None,
                "vel_x": None,
                "vel_y": None,
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
                "source_file": ev.get("source_file"),
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
