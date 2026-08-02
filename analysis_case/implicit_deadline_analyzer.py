#!/usr/bin/env python3
"""Offline speed-conditioned implicit-deadline evidence analysis.

This module deliberately separates Apollo's traced perception->planning compute
latency from the physical response latency (stable observation->brake actually
applied by the Bridge).  The physical latency is the quantity compared with the
speed/distance-derived safety deadline.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import re
import statistics
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

from collision_case_classifier import (
    CaseDataLoader,
    Config,
    LogAndTableParser,
    json_safe,
    normalize_planning_id,
    normalize_type,
    to_float,
)
from timing_anomaly_detector import calculate_frame_latencies


FUNCTIONALLY_NORMAL_VERDICT = "PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING"


def percentile(values: Sequence[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stats_ms(values: Iterable[Optional[float]]) -> Dict[str, Any]:
    usable = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return {
        "sample_count": len(usable),
        "median_ms": statistics.median(usable) if usable else None,
        "p90_ms": percentile(usable, 0.90),
        "p95_ms": percentile(usable, 0.95),
        "max_ms": max(usable) if usable else None,
    }


def deadline_model(
    speed_kmh: float,
    decel_mps2: float,
    safety_margin_m: float,
    actuator_delay_s: float,
    d1_m: Optional[float] = None,
    target_deadline_ms: Optional[float] = None,
) -> Dict[str, Optional[float]]:
    if speed_kmh <= 0.0 or decel_mps2 <= 0.0:
        raise ValueError("speed_kmh and decel_mps2 must be positive")
    speed_mps = speed_kmh / 3.6
    brake_distance = speed_mps * speed_mps / (2.0 * decel_mps2)
    actuator_distance = speed_mps * max(0.0, actuator_delay_s)
    d2_safety = actuator_distance + brake_distance + max(0.0, safety_margin_m)
    d2_collision = actuator_distance + brake_distance
    desired_d1 = None
    desired_collision_d1 = None
    if target_deadline_ms is not None:
        desired_d1 = d2_safety + speed_mps * target_deadline_ms / 1000.0
        desired_collision_d1 = (
            d2_collision + speed_mps * target_deadline_ms / 1000.0
        )
    safety_deadline_ms = None
    collision_deadline_ms = None
    if d1_m is not None:
        safety_deadline_ms = (d1_m - d2_safety) / speed_mps * 1000.0
        collision_deadline_ms = (d1_m - d2_collision) / speed_mps * 1000.0
    return {
        "speed_kmh": speed_kmh,
        "speed_mps": speed_mps,
        "decel_mps2": decel_mps2,
        "brake_distance_m": brake_distance,
        "actuator_distance_m": actuator_distance,
        "d2_safety_m": d2_safety,
        "d2_collision_m": d2_collision,
        "target_deadline_ms": target_deadline_ms,
        "target_deadline_kind": "SAFETY_MARGIN",
        "desired_d1_m": desired_d1,
        "desired_d1_for_safety_deadline_m": desired_d1,
        "desired_d1_for_collision_deadline_m": desired_collision_d1,
        "measured_d1_m": d1_m,
        "safety_deadline_ms": safety_deadline_ms,
        "collision_deadline_ms": collision_deadline_ms,
    }


def configured_desired_d1(cfg: Dict[str, Any]) -> float:
    explicit = to_float(cfg.get("expected_stable_distance_m"))
    if explicit is not None:
        return explicit
    model = deadline_model(
        float(cfg.get("speed_kmh", 80.0)),
        float(cfg.get("decel_mps2", 10.0)),
        float(cfg.get("safety_margin_m", 5.0)),
        float(cfg.get("actuator_delay_s", 0.0)),
        None,
        float(cfg.get("target_safety_deadline_ms", 200.0)),
    )
    return float(model["desired_d1_m"])


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    text = path.read_text(encoding="utf-8")
    if yaml is not None:
        loaded = yaml.safe_load(text) or {}
    else:
        # Both SCB files intentionally use a small, flat YAML schema.  Keep the
        # analyzer runnable in Apollo containers that do not ship PyYAML.
        loaded = {}
        current = loaded
        for raw in text.splitlines():
            content = raw.split("#", 1)[0].rstrip()
            if not content.strip() or ":" not in content:
                continue
            indent = len(content) - len(content.lstrip(" "))
            key, value = content.strip().split(":", 1)
            if not value.strip():
                section = {}
                loaded[key.strip()] = section
                current = section
                continue
            target = current if indent > 0 else loaded
            scalar = value.strip().strip('"').strip("'")
            lower = scalar.lower()
            if lower in {"null", "none", "~", ""}:
                parsed: Any = None
            elif lower in {"true", "yes"}:
                parsed = True
            elif lower in {"false", "no"}:
                parsed = False
            else:
                try:
                    parsed = float(scalar) if any(char in scalar for char in ".eE") else int(scalar)
                except ValueError:
                    parsed = scalar
            target[key.strip()] = parsed
    return loaded if isinstance(loaded, dict) else {}


def load_deadline_config(path: Path) -> Dict[str, Any]:
    loaded = load_yaml(path)
    cfg = loaded.get("implicit_deadline")
    if not isinstance(cfg, dict):
        raise ValueError("Missing implicit_deadline section")
    return cfg


def load_experiment_metadata(case_dir: Path) -> Dict[str, Any]:
    path = case_dir / "scb_experiment.yaml"
    if not path.exists():
        return {}
    return load_yaml(path)


def load_json(path: Optional[Path]) -> Dict[str, Any]:
    if path is None or not path.exists():
        return {}
    loaded = json.loads(path.read_text(encoding="utf-8"))
    return loaded if isinstance(loaded, dict) else {}


def resolve_collision(case_dir: Path, functional: Dict[str, Any]) -> Dict[str, Any]:
    value = to_float(functional.get("t2_collision_time"))
    if value is not None:
        return {"occurred": True, "time_sec": value, "source": "functional_result"}
    rows: List[Tuple[float, str]] = []
    for path in case_dir.rglob("carla_collision_events_*.csv"):
        with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as fh:
            for row in csv.DictReader(fh):
                ns = to_float(row.get("wall_time_unix_ns"))
                if ns is not None:
                    rows.append((ns / 1_000_000_000.0, str(path.relative_to(case_dir))))
    if rows:
        value, source = min(rows)
        return {"occurred": True, "time_sec": value, "source": source}
    return {"occurred": False, "time_sec": None, "source": None}


def resolve_epoch_reference(
    case_dir: Path,
    metadata: Dict[str, Any],
    post_event_sec: float = 5.0,
) -> Optional[float]:
    """Find a parse-window end for a safe run with no collision.

    Prefer the end of the response event, not obstacle spawn.  Using spawn as
    t2 clipped later prediction/planning/braking evidence in longer safe runs.
    """
    end_ns = to_float(metadata.get("analysis_end_wall_time_unix_ns"))
    if end_ns is not None and end_ns > 1e17:
        return end_ns / 1e9
    bridge_times: List[float] = []
    for path in sorted(case_dir.rglob("scb_control_delay_*.csv")):
        with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as fh:
            for row in csv.DictReader(fh):
                if str(row.get("status") or "") != "APPLIED":
                    continue
                ns = (
                    to_float(row.get("apply_call_end_wall_time_unix_ns"))
                    or to_float(row.get("apply_wall_time_unix_ns"))
                )
                if ns is not None and ns > 1e17:
                    bridge_times.append(ns / 1e9)
    if bridge_times:
        return max(bridge_times) + max(0.0, float(post_event_sec))
    spawn_ns = to_float(metadata.get("obstacle_spawn_wall_time_unix_ns"))
    if spawn_ns is not None and spawn_ns > 1e17:
        return spawn_ns / 1e9 + max(0.0, float(post_event_sec))
    pattern = re.compile(r"\bheader_time=([0-9.eE+-]+)")
    latest = None
    for path in sorted((case_dir / "log").glob("perception.log.INFO*")):
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if "[FUSION_OBS_FRAME]" not in line:
                    continue
                match = pattern.search(line)
                value = to_float(match.group(1)) if match else None
                if value is not None and value > 1e8:
                    latest = value if latest is None else max(latest, value)
    return latest


def _row_time(row: Dict[str, Any]) -> Optional[float]:
    return to_float(row.get("output_time")) or to_float(row.get("time"))


def _obs_time(row: Dict[str, Any]) -> Optional[float]:
    return to_float(row.get("obs_time")) or to_float(row.get("time"))


def _nearest_ego(ego_rows: Sequence[Dict[str, Any]], timestamp: Optional[float]) -> Optional[Dict[str, Any]]:
    usable = [row for row in ego_rows if to_float(row.get("time")) is not None]
    if timestamp is None or not usable:
        return None
    return min(usable, key=lambda row: abs(float(row["time"]) - timestamp))


def _ego_speed(row: Optional[Dict[str, Any]]) -> Optional[float]:
    if not row:
        return None
    explicit = to_float(row.get("ego_speed"))
    if explicit is not None:
        return abs(explicit)
    vx, vy = to_float(row.get("ego_vx")), to_float(row.get("ego_vy"))
    if vx is None or vy is None:
        return None
    return math.hypot(vx, vy)


def stable_segment(
    rows: Sequence[Dict[str, Any]],
    ego_rows: Sequence[Dict[str, Any]],
    cfg: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    required = max(2, int(cfg.get("stable_perception_frames", 3)))
    max_gap = float(cfg.get("stable_max_frame_gap_sec", 0.25))
    configured_min_speed = cfg.get("target_search_min_ego_speed_mps")
    if configured_min_speed is None:
        configured_speed_mps = float(cfg.get("speed_kmh", 80.0)) / 3.6
        min_ego_speed = configured_speed_mps * float(
            cfg.get("target_search_min_ego_speed_ratio", 0.70)
        )
    else:
        min_ego_speed = float(configured_min_speed)
    ordered = sorted(
        [row for row in rows if _row_time(row) is not None],
        key=lambda row: float(_row_time(row)),
    )
    unique: List[Dict[str, Any]] = []
    seen = set()
    for row in ordered:
        key = round(float(_row_time(row)), 6)
        if key not in seen:
            seen.add(key)
            unique.append(row)
    for index in range(0, len(unique) - required + 1):
        window = unique[index:index + required]
        times = [float(_row_time(row)) for row in window]
        gaps = [new - old for old, new in zip(times, times[1:])]
        if any(gap <= 0.0 or gap > max_gap for gap in gaps):
            continue
        first = window[0]
        observation_time = _obs_time(first)
        ego = _nearest_ego(ego_rows, observation_time)
        speed = _ego_speed(ego)
        if speed is None:
            speed = to_float(first.get("closing_speed"))
        if speed is not None and speed < min_ego_speed:
            continue
        return {
            "first_row": first,
            "confirmation_row": window[-1],
            "frames": window,
            "stable_observation_time_sec": observation_time,
            "stable_output_time_sec": _row_time(first),
            "stable_confirmation_output_time_sec": _row_time(window[-1]),
            "ego_speed_mps": speed,
        }
    return None


def resolve_target(
    artifacts: Any,
    cfg: Dict[str, Any],
    requested_ids: Sequence[str],
) -> Dict[str, Any]:
    normalized = []
    for value in requested_ids:
        obs_id = normalize_planning_id(value)
        if obs_id and obs_id not in normalized:
            normalized.append(obs_id)
    by_id: Dict[str, List[Dict[str, Any]]] = {}
    for row in artifacts.fusion_obs:
        obs_id = normalize_planning_id(row.get("id"))
        if obs_id:
            by_id.setdefault(obs_id, []).append(row)

    if normalized:
        for obs_id in normalized:
            segment = stable_segment(by_id.get(obs_id, []), artifacts.ego_states, cfg)
            if segment:
                return {
                    "status": "RESOLVED",
                    "target_id": obs_id,
                    "target_id_chain": normalized,
                    "source": "provided_or_functional",
                    "confidence": 1.0,
                    "segment": segment,
                    "candidates": [],
                }
        return {
            "status": "UNRESOLVED",
            "target_id": normalized[0],
            "target_id_chain": normalized,
            "source": "provided_or_functional",
            "confidence": 0.0,
            "segment": None,
            "candidates": [],
            "reason": "PROVIDED_TARGET_HAS_NO_STABLE_SEGMENT",
        }

    expected = configured_desired_d1(cfg)
    max_distance = float(cfg.get("target_search_max_distance_m", 55.0))
    lateral_limit = float(cfg.get("target_lateral_tolerance_m", 3.0))
    static_limit = float(cfg.get("static_target_max_speed_mps", 2.0))
    candidates = []
    for obs_id, rows in by_id.items():
        segment = stable_segment(rows, artifacts.ego_states, cfg)
        if not segment:
            continue
        first = segment["first_row"]
        forward = to_float(first.get("rel_forward"))
        lateral = to_float(first.get("rel_left"))
        target_speed = to_float(first.get("speed"))
        if forward is None or lateral is None or forward <= 0.0 or forward > max_distance:
            continue
        distance_scale = max(
            0.1, float(cfg.get("target_distance_score_scale_m", 30.0))
        )
        distance_score = max(0.0, 1.0 - abs(forward - expected) / distance_scale)
        lateral_score = max(0.0, 1.0 - abs(lateral) / max(lateral_limit, 0.1))
        static_score = 1.0 if target_speed is not None and abs(target_speed) <= static_limit else 0.0
        type_score = 1.0 if normalize_type(first.get("type")) == "VEHICLE" else 0.0
        score = 0.45 * distance_score + 0.25 * lateral_score + 0.20 * static_score + 0.10 * type_score
        candidates.append({
            "target_id": obs_id,
            "score": score,
            "rel_forward_m": forward,
            "rel_left_m": lateral,
            "target_speed_mps": target_speed,
            "type": normalize_type(first.get("type")),
            "segment": segment,
        })
    candidates.sort(key=lambda item: item["score"], reverse=True)
    minimum = float(cfg.get("target_candidate_min_score", 0.65))
    margin_min = float(cfg.get("target_candidate_min_margin", 0.10))
    margin = candidates[0]["score"] - candidates[1]["score"] if len(candidates) > 1 else 1.0
    debug = [{key: value for key, value in candidate.items() if key != "segment"} for candidate in candidates[:20]]
    if not candidates or candidates[0]["score"] < minimum or margin < margin_min:
        return {
            "status": "UNRESOLVED",
            "target_id": None,
            "target_id_chain": [],
            "source": "static_in_lane_candidate_search",
            "confidence": candidates[0]["score"] if candidates else 0.0,
            "score_margin": margin if candidates else None,
            "segment": None,
            "candidates": debug,
            "reason": "TARGET_CANDIDATES_AMBIGUOUS_OR_WEAK",
        }
    selected = candidates[0]
    return {
        "status": "RESOLVED",
        "target_id": selected["target_id"],
        "target_id_chain": [selected["target_id"]],
        "source": "static_in_lane_candidate_search",
        "confidence": selected["score"],
        "score_margin": margin,
        "segment": selected["segment"],
        "candidates": debug,
    }


def load_brake_apply(case_dir: Path, stable_time: Optional[float]) -> Dict[str, Any]:
    candidates = []
    files = sorted(case_dir.rglob("scb_control_delay_*.csv"))
    for path in files:
        with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as fh:
            for row in csv.DictReader(fh):
                if str(row.get("first_effective_brake", "")).strip().lower() not in {"1", "true"}:
                    continue
                if str(row.get("status") or "") != "APPLIED":
                    continue
                api_end_ns = to_float(row.get("apply_call_end_wall_time_unix_ns"))
                apply_ns = api_end_ns or to_float(row.get("apply_wall_time_unix_ns"))
                receive_ns = to_float(row.get("receive_wall_time_unix_ns"))
                if apply_ns is None or receive_ns is None:
                    continue
                candidates.append((apply_ns / 1e9, receive_ns / 1e9, path, row))
    if not candidates:
        return {
            "status": "MISSING",
            "reason": "SCB_CONTROL_DELAY_EVIDENCE_NOT_FOUND",
            "source_file": None,
        }
    if stable_time is not None:
        after = [item for item in candidates if item[0] >= stable_time]
        if not after:
            return {
                "status": "MISSING",
                "reason": "NO_EFFECTIVE_BRAKE_AT_OR_AFTER_T1",
                "source_file": None,
                "pre_t1_candidate_count": len(candidates),
            }
        candidates = after
    apply_time, receive_time, path, row = min(candidates, key=lambda item: item[0])
    schema_version = str(row.get("schema_version") or "")
    boundary = (
        "CARLA_API_CALL_END"
        if to_float(row.get("apply_call_end_wall_time_unix_ns")) is not None
        else "LEGACY_APPLY_TIMESTAMP_SEMANTICS_UNKNOWN"
    )
    return {
        "status": "AVAILABLE",
        "source_file": str(path.relative_to(case_dir)),
        "schema_version": schema_version or None,
        "application_boundary": boundary,
        "bridge_entry_file": row.get("bridge_entry_file") or None,
        "settings_source_file": row.get("settings_source_file") or None,
        "injector_source_file": row.get("injector_source_file") or None,
        "bridge_process_id": to_float(row.get("process_id")),
        "bridge_process_working_directory": (
            row.get("process_working_directory") or None
        ),
        "activation_speed_mps": to_float(row.get("activation_speed_mps")),
        "configured_brake_threshold_percentage": to_float(
            row.get("brake_threshold_percentage")
        ),
        "log_all_delayed_commands": str(
            row.get("log_all_delayed_commands") or ""
        ).strip().lower() in {"1", "true", "yes"},
        "control_header_time_sec": to_float(row.get("control_header_time_sec")),
        "receive_wall_time_sec": receive_time,
        "release_wall_time_sec": (
            to_float(row.get("release_wall_time_unix_ns")) / 1e9
            if to_float(row.get("release_wall_time_unix_ns")) is not None
            else None
        ),
        "api_apply_call_start_wall_time_sec": (
            to_float(row.get("apply_call_start_wall_time_unix_ns")) / 1e9
            if to_float(row.get("apply_call_start_wall_time_unix_ns")) is not None
            else None
        ),
        "api_apply_call_end_wall_time_sec": apply_time,
        # Backward-compatible alias.  This is an API boundary, not proof that
        # the vehicle has begun producing braking force.
        "apply_wall_time_sec": apply_time,
        "requested_delay_ms": to_float(row.get("requested_delay_ms")),
        "actual_delay_ms": to_float(row.get("actual_delay_ms")),
        "api_completion_delay_ms": to_float(row.get("api_completion_delay_ms")),
        "api_call_duration_ms": to_float(row.get("api_call_duration_ms")),
        "actual_frame_delay": to_float(row.get("actual_frame_delay")),
        "actual_sim_delay_ms": to_float(row.get("actual_sim_delay_ms")),
        "brake_percentage": to_float(row.get("brake_percentage")),
        "ego_speed_mps_at_receive": to_float(row.get("ego_speed_mps_at_receive")),
        "receive_carla_frame": to_float(row.get("receive_carla_frame")),
        "apply_carla_frame": to_float(row.get("apply_carla_frame")),
    }


def detect_effective_brake_onset(
    artifacts: Any,
    api_apply_time: Optional[float],
    cfg: Dict[str, Any],
) -> Dict[str, Any]:
    """Detect the first sustained speed decrease after CARLA API submission.

    The Bridge API call is not a physical brake-onset timestamp.  In the
    synchronous simulator, localization speed is sampled after CARLA ticks, so
    the first sustained deceleration is the conservative observable boundary.
    """
    if api_apply_time is None:
        return {"status": "UNAVAILABLE", "reason": "CARLA_API_APPLY_TIME_MISSING"}
    threshold = max(
        0.0, float(cfg.get("effective_brake_min_decel_mps2", 0.5))
    )
    required = max(
        1, int(cfg.get("effective_brake_consecutive_intervals", 2))
    )
    max_lag = max(
        0.1, float(cfg.get("effective_brake_max_lag_sec", 1.5))
    )
    precheck = max(
        0.0, float(cfg.get("effective_brake_precheck_sec", 0.3))
    )
    localization = [
        row for row in artifacts.ego_states
        if row.get("ego_source") == "localization"
    ]
    source_rows = localization or list(artifacts.ego_states)
    samples = []
    for row in source_rows:
        timestamp = to_float(row.get("time"))
        speed = _ego_speed(row)
        if timestamp is None or speed is None:
            continue
        if api_apply_time - precheck - 0.5 <= timestamp <= api_apply_time + max_lag + 0.5:
            samples.append((timestamp, speed))
    samples.sort()
    unique = []
    for timestamp, speed in samples:
        if unique and abs(timestamp - unique[-1][0]) < 1e-6:
            unique[-1] = (timestamp, speed)
        else:
            unique.append((timestamp, speed))
    intervals = []
    for previous, current in zip(unique, unique[1:]):
        dt = current[0] - previous[0]
        if dt <= 1e-4 or dt > 0.5:
            continue
        decel = (previous[1] - current[1]) / dt
        intervals.append({
            "start_time_sec": previous[0],
            "end_time_sec": current[0],
            "start_speed_mps": previous[1],
            "end_speed_mps": current[1],
            "decel_mps2": decel,
        })
    preexisting = [
        item for item in intervals
        if api_apply_time - precheck <= item["end_time_sec"] < api_apply_time
        and item["decel_mps2"] >= threshold
    ]
    if preexisting:
        return {
            "status": "INVALID_PREEXISTING_DECELERATION",
            "reason": "EGO_ALREADY_DECELERATING_BEFORE_SELECTED_BRAKE_COMMAND",
            "threshold_mps2": threshold,
            "preexisting_interval_count": len(preexisting),
            "sample_count": len(unique),
        }
    eligible = [
        item for item in intervals
        if item["end_time_sec"] >= api_apply_time
        and item["end_time_sec"] <= api_apply_time + max_lag
    ]
    for index in range(0, len(eligible) - required + 1):
        window = eligible[index:index + required]
        if any(item["decel_mps2"] < threshold for item in window):
            continue
        if any(
            abs(new["start_time_sec"] - old["end_time_sec"]) > 1e-4
            for old, new in zip(window, window[1:])
        ):
            continue
        onset = window[0]["end_time_sec"]
        confirmation = window[-1]["end_time_sec"]
        return {
            "status": "AVAILABLE",
            "source": "LOCALIZATION_SPEED_DERIVATIVE",
            "effective_brake_onset_time_sec": onset,
            "confirmation_time_sec": confirmation,
            "api_apply_to_effective_onset_ms": (onset - api_apply_time) * 1000.0,
            "threshold_mps2": threshold,
            "required_consecutive_intervals": required,
            "observed_decel_mps2": [item["decel_mps2"] for item in window],
            "sample_count": len(unique),
        }
    return {
        "status": "UNAVAILABLE",
        "reason": "SUSTAINED_DECELERATION_NOT_FOUND_AFTER_BRAKE_COMMAND",
        "threshold_mps2": threshold,
        "required_consecutive_intervals": required,
        "sample_count": len(unique),
        "max_search_lag_sec": max_lag,
    }


def estimate_deceleration(artifacts: Any, brake_time: Optional[float], end_time: Optional[float]) -> Dict[str, Any]:
    if brake_time is None:
        return {"status": "UNAVAILABLE", "reason": "BRAKE_APPLY_TIME_MISSING"}
    rows = []
    for row in artifacts.ego_states:
        timestamp = to_float(row.get("time"))
        speed = _ego_speed(row)
        x, y = to_float(row.get("ego_x")), to_float(row.get("ego_y"))
        if timestamp is None or speed is None or x is None or y is None:
            continue
        if timestamp < brake_time - 0.15 or timestamp > (end_time or brake_time + 8.0) + 0.2:
            continue
        rows.append((timestamp, speed, x, y))
    rows.sort()
    if len(rows) < 5:
        return {"status": "UNAVAILABLE", "reason": "INSUFFICIENT_EGO_SPEED_SAMPLES", "sample_count": len(rows)}
    start_index = min(range(len(rows)), key=lambda index: abs(rows[index][0] - brake_time))
    rows = rows[start_index:]
    if len(rows) < 5:
        return {"status": "UNAVAILABLE", "reason": "INSUFFICIENT_POST_BRAKE_SAMPLES", "sample_count": len(rows)}
    start_speed = rows[0][1]
    distance = 0.0
    points = [(0.0, start_speed * start_speed)]
    stop_distance = None
    stop_time = None
    previous = rows[0]
    for current in rows[1:]:
        distance += math.hypot(current[2] - previous[2], current[3] - previous[3])
        points.append((distance, current[1] * current[1]))
        if stop_distance is None and current[1] <= 0.5:
            stop_distance, stop_time = distance, current[0]
        previous = current
    equivalent = None
    if stop_distance is not None and stop_distance > 0.1:
        equivalent = start_speed * start_speed / (2.0 * stop_distance)
    fitted = None
    usable = [point for point in points if point[0] <= (stop_distance if stop_distance is not None else points[-1][0])]
    if len(usable) >= 5 and start_speed - math.sqrt(max(0.0, usable[-1][1])) >= 1.0:
        mean_x = statistics.mean(point[0] for point in usable)
        mean_y = statistics.mean(point[1] for point in usable)
        denominator = sum((point[0] - mean_x) ** 2 for point in usable)
        if denominator > 1e-6:
            slope = sum((point[0] - mean_x) * (point[1] - mean_y) for point in usable) / denominator
            if slope < 0.0:
                fitted = -slope / 2.0
    return {
        "status": "AVAILABLE" if equivalent is not None or fitted is not None else "UNAVAILABLE",
        "sample_count": len(rows),
        "start_speed_mps": start_speed,
        "stop_detected": stop_distance is not None,
        "stop_time_sec": stop_time,
        "measured_braking_distance_m": stop_distance,
        "equivalent_decel_mps2": equivalent,
        "fitted_decel_mps2": fitted,
        "recommended_run_decel_mps2": equivalent if equivalent is not None else fitted,
    }


def semantic_chain_evidence(
    artifacts: Any,
    target_ids: Sequence[str],
    start_time: float,
    end_time: float,
) -> Dict[str, Any]:
    """Lightweight chain evidence for non-collision calibration runs.

    This is not a replacement for the collision functional classifier.  It only
    prevents a calibration run with a missing target handoff/output from entering
    the baseline statistics.
    """
    ids = {normalize_planning_id(value) for value in target_ids if normalize_planning_id(value)}

    def in_window(row: Dict[str, Any]) -> bool:
        timestamp = to_float(row.get("time"))
        return timestamp is not None and start_time <= timestamp <= end_time

    prediction_outputs = [
        row for row in artifacts.prediction_outputs
        if normalize_planning_id(row.get("id")) in ids and in_window(row)
    ]
    static_outputs = [row for row in prediction_outputs if row.get("is_static") is True]
    planning_inputs = [
        row for row in artifacts.planning_inputs
        if normalize_planning_id(row.get("id")) in ids and in_window(row)
    ]
    decisions = [
        row for row in artifacts.planning_decisions
        if normalize_planning_id(row.get("id")) in ids and in_window(row)
        and str(row.get("decision_type") or "").upper() in {"STOP", "FOLLOW", "YIELD", "OVERTAKE"}
    ]
    boundaries = [
        row for row in artifacts.planning_st_boundaries
        if normalize_planning_id(row.get("id")) in ids and in_window(row)
        and (to_float(row.get("point_count")) or 0) > 0
    ]
    planning_outputs = [row for row in artifacts.planning_outputs if in_window(row)]
    valid_planning_outputs = [
        row for row in planning_outputs
        if row.get("status_ok") is not False
        and row.get("estop") is not True
        and (to_float(row.get("trajectory_point_count")) or 0) > 0
    ]
    checks = {
        "prediction_target_output_present": bool(prediction_outputs),
        "prediction_static_output_present": bool(static_outputs),
        "planning_target_input_present": bool(planning_inputs),
        "planning_target_constraint_present": bool(decisions or boundaries),
        "planning_valid_output_present": bool(valid_planning_outputs),
    }
    return {
        "scope": "calibration_quality_gate_not_collision_classifier",
        "checks": checks,
        "complete": all(checks.values()),
        "counts": {
            "prediction_outputs": len(prediction_outputs),
            "prediction_static_outputs": len(static_outputs),
            "planning_inputs": len(planning_inputs),
            "planning_constraints": len(decisions) + len(boundaries),
            "planning_valid_outputs": len(valid_planning_outputs),
        },
    }


def trace_evidence(
    case_dir: Path,
    stable_time: float,
    end_time: float,
    target_ids: Sequence[str],
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    thresholds = {name: 1e12 for name in ("perception", "prediction", "planning", "e2e")}
    try:
        rows, metadata = calculate_frame_latencies(
            case_dir, stable_time, end_time, thresholds, target_ids
        )
    except Exception as exc:
        return {"status": "UNAVAILABLE", "reason": str(exc)}, []
    target_rows = [row for row in rows if row.get("target_present")]
    complete = [row for row in (target_rows or rows) if row.get("frame_status") == "COMPLETE"]
    nearest = min(rows, key=lambda row: abs(float(row["data_time_sec"]) - stable_time)) if rows else None
    modules = {}
    for module in ("perception", "prediction", "planning", "e2e"):
        modules[module] = stats_ms(row.get(f"{module}_ms") for row in complete)
    return {
        "status": "AVAILABLE" if rows else "UNAVAILABLE",
        "frame_count": len(rows),
        "target_frame_count": len(target_rows),
        "complete_frame_ratio": metadata.get("complete_frame_ratio"),
        "statistics_ms": modules,
        "stable_frame": {
            "trace_id": nearest.get("trace_id"),
            "data_time_sec": nearest.get("data_time_sec"),
            "time_error_ms": abs(float(nearest["data_time_sec"]) - stable_time) * 1000.0,
            "perception_ms": nearest.get("perception_ms"),
            "prediction_ms": nearest.get("prediction_ms"),
            "planning_ms": nearest.get("planning_ms"),
            "e2e_ms": nearest.get("e2e_ms"),
            "target_present": nearest.get("target_present"),
        } if nearest else None,
    }, rows


def _write_trace_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    fields = [
        "frame_index", "trace_id", "data_time_sec", "target_present", "frame_status",
        "perception_ms", "prediction_ms", "planning_ms", "e2e_ms",
        "perception_to_prediction_handoff_ms", "prediction_to_planning_handoff_ms",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def run_implicit_deadline_analysis(
    case_dir: Path,
    out_dir: Path,
    config_path: Path,
    functional_result_path: Optional[Path] = None,
    functional_config_path: Optional[Path] = None,
    calibration_path: Optional[Path] = None,
    target_id: Optional[str] = None,
    config_overrides: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    case_dir = case_dir.resolve()
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_deadline_config(config_path)
    cfg.update({
        key: value
        for key, value in (config_overrides or {}).items()
        if value is not None
    })
    functional = load_json(functional_result_path)
    metadata = load_experiment_metadata(case_dir)
    calibration = load_json(calibration_path)
    collision = resolve_collision(case_dir, functional)

    functional_cfg = Config.load(functional_config_path) if functional_config_path else Config()
    loader = CaseDataLoader(case_dir, out_dir)
    log_parser = LogAndTableParser(loader, functional_cfg, collision["time_sec"])
    if collision["time_sec"] is None:
        epoch_reference = resolve_epoch_reference(
            case_dir,
            metadata,
            float(cfg.get("trace_window_after_stable_sec", 5.0)),
        )
        if epoch_reference is not None:
            log_parser.t2 = epoch_reference
            log_parser.time.t2 = epoch_reference
            pre = max(
                float(functional_cfg.analysis["pre_collision_window_sec"]),
                float(functional_cfg.analysis["planning_target_window_sec"]),
            )
            post = float(functional_cfg.analysis["post_collision_window_sec"])
            pad = float(functional_cfg.analysis["max_time_match_diff_sec"])
            log_parser.parse_start = epoch_reference - pre - pad
            log_parser.parse_end = epoch_reference + post + pad
    artifacts = log_parser.parse()
    chain = [str(value) for value in ((functional.get("perception") or {}).get("id_chain") or [])]
    requested = [value for value in [target_id, metadata.get("apollo_target_id"), functional.get("target_id"), *chain] if value]
    resolution_cfg = dict(cfg)
    calibrated_search_d1 = to_float(
        (calibration.get("deadline_model_at_baseline_medians") or {}).get(
            "desired_d1_m"
        )
    )
    if calibrated_search_d1 is not None:
        resolution_cfg["expected_stable_distance_m"] = calibrated_search_d1
    target = resolve_target(artifacts, resolution_cfg, requested)

    base_result = {
        "case_id": case_dir.name,
        "status": "UNKNOWN_DATA_INSUFFICIENT",
        "implicit_safety_deadline_miss": None,
        "collision_deadline_miss": None,
        "causal_assessment": "INSUFFICIENT_EVIDENCE",
        "functional_context": {
            "result_available": bool(functional),
            "verdict": functional.get("final_verdict"),
            "reason_code": functional.get("reason_code"),
            "functionally_normal": functional.get("final_verdict") == FUNCTIONALLY_NORMAL_VERDICT if functional else None,
        },
        "experiment_metadata": metadata,
        "collision": collision,
        "target_resolution": {key: value for key, value in target.items() if key != "segment"},
        "warnings": [],
        "output_files": {
            "summary": "implicit_deadline_result.json",
            "trace_frames": "implicit_deadline_trace_frames.csv",
        },
    }
    if target.get("status") != "RESOLVED" or not target.get("segment"):
        base_result["warnings"].append("Target or stable perception onset could not be resolved.")
        (out_dir / "implicit_deadline_result.json").write_text(
            json.dumps(json_safe(base_result), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        _write_trace_csv(out_dir / "implicit_deadline_trace_frames.csv", [])
        return base_result

    segment = target["segment"]
    first = segment["first_row"]
    t1_observation = to_float(segment["stable_observation_time_sec"])
    t1_output = to_float(segment["stable_output_time_sec"])
    if t1_observation is None:
        base_result["warnings"].append("Stable target has no usable observation timestamp.")
        (out_dir / "implicit_deadline_result.json").write_text(
            json.dumps(json_safe(base_result), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        _write_trace_csv(out_dir / "implicit_deadline_trace_frames.csv", [])
        return base_result
    d1_raw = to_float(first.get("rel_forward"))
    offset = float(cfg.get("longitudinal_distance_offset_m", 0.0))
    d1 = d1_raw + offset if d1_raw is not None else None
    d1_reference = str(
        cfg.get("d1_distance_reference", "actor_center_to_actor_center")
    )
    d1_reference_verified = bool(
        cfg.get("d1_reference_verified_for_physical_model", False)
    )
    measured_speed_mps = to_float(segment.get("ego_speed_mps"))
    configured_speed_kmh = float(cfg.get("speed_kmh", 80.0))
    model_speed_kmh = measured_speed_mps * 3.6 if measured_speed_mps and measured_speed_mps > 0 else configured_speed_kmh
    decel = to_float((calibration.get("recommended") or {}).get("conservative_decel_mps2"))
    if decel is None:
        decel = float(cfg.get("decel_mps2", 10.0))
    model = deadline_model(
        model_speed_kmh,
        decel,
        float(cfg.get("safety_margin_m", 5.0)),
        float(cfg.get("actuator_delay_s", 0.0)),
        d1,
        float(cfg.get("target_safety_deadline_ms", 200.0)),
    )
    brake = load_brake_apply(case_dir, t1_observation)
    api_apply_time = to_float(brake.get("api_apply_call_end_wall_time_sec"))
    if api_apply_time is None:
        api_apply_time = to_float(brake.get("apply_wall_time_sec"))
    receive_time = to_float(brake.get("receive_wall_time_sec"))
    brake_onset = detect_effective_brake_onset(artifacts, api_apply_time, cfg)
    effective_onset_time = to_float(
        brake_onset.get("effective_brake_onset_time_sec")
    )
    response_ms = (
        (effective_onset_time - t1_observation) * 1000.0
        if effective_onset_time is not None
        else None
    )
    api_response_ms = (
        (api_apply_time - t1_observation) * 1000.0
        if api_apply_time is not None
        else None
    )
    to_receive_ms = (receive_time - t1_observation) * 1000.0 if receive_time is not None and t1_observation is not None else None
    perception_availability_ms = (t1_output - t1_observation) * 1000.0 if t1_output is not None and t1_observation is not None else None
    end_time = collision["time_sec"] or (t1_observation + float(cfg.get("trace_window_after_stable_sec", 5.0)))
    if end_time <= t1_observation:
        end_time = t1_observation + float(cfg.get("trace_window_after_stable_sec", 5.0))
    trace, trace_rows = trace_evidence(case_dir, t1_observation, end_time, target["target_id_chain"])
    deceleration = estimate_deceleration(
        artifacts, effective_onset_time, collision["time_sec"]
    )
    semantic_chain = semantic_chain_evidence(
        artifacts,
        target["target_id_chain"],
        t1_observation,
        end_time,
    )

    safety_deadline = to_float(model.get("safety_deadline_ms"))
    collision_deadline = to_float(model.get("collision_deadline_ms"))
    deadline_computable = (
        d1 is not None
        and safety_deadline is not None
        and collision_deadline is not None
    )
    safety_miss = response_ms > safety_deadline if response_ms is not None and safety_deadline is not None else None
    collision_miss = response_ms > collision_deadline if response_ms is not None and collision_deadline is not None else None
    calibrated_expected_d1 = to_float(
        (calibration.get("deadline_model_at_baseline_medians") or {}).get("desired_d1_m")
    )
    expected_d1 = (
        calibrated_expected_d1
        if calibrated_expected_d1 is not None
        else configured_desired_d1(cfg)
    )
    d1_tolerance = float(cfg.get("stable_distance_tolerance_m", 2.0))
    speed_tolerance = float(cfg.get("speed_tolerance_kmh", 2.0))
    d1_valid = d1 is not None and abs(d1 - expected_d1) <= d1_tolerance
    speed_valid = abs(model_speed_kmh - configured_speed_kmh) <= speed_tolerance
    clock_valid = response_ms is not None and 0.0 <= response_ms <= 10000.0
    target_static = to_float(first.get("speed")) is not None and abs(float(first.get("speed"))) <= float(cfg.get("static_target_max_speed_mps", 2.0))
    trace_ratio = to_float(trace.get("complete_frame_ratio"))
    trace_ratio_required = float(cfg.get("min_trace_complete_ratio", 0.80))
    trace_sufficient = (
        trace.get("status") == "AVAILABLE"
        and trace_ratio is not None
        and trace_ratio >= trace_ratio_required
    )
    configured_brake_threshold = float(cfg.get("brake_threshold_percentage", 1.0))
    applied_brake_percentage = to_float(brake.get("brake_percentage"))
    brake_threshold_match = (
        applied_brake_percentage is not None
        and applied_brake_percentage >= configured_brake_threshold
    )

    spawn_ns = to_float(metadata.get("obstacle_spawn_wall_time_unix_ns"))
    spawn_time = spawn_ns / 1e9 if spawn_ns is not None else None
    spawn_distance = to_float(metadata.get("obstacle_spawn_distance_m"))
    stable_lag_ms = (t1_observation - spawn_time) * 1000.0 if spawn_time is not None else None
    stable_lag_distance = spawn_distance - d1 if spawn_distance is not None and d1 is not None else None
    functionally_normal = base_result["functional_context"]["functionally_normal"]
    injected = to_float(brake.get("requested_delay_ms")) is not None and float(brake.get("requested_delay_ms")) > 0.0
    bridge_threshold = to_float(
        brake.get("configured_brake_threshold_percentage")
    )
    bridge_threshold_consistent = (
        bridge_threshold is None
        or abs(bridge_threshold - configured_brake_threshold) <= 1e-6
    )

    causal = "NO_COLLISION"
    if collision["occurred"]:
        if functionally_normal is not True:
            causal = "COLLISION_NOT_ATTRIBUTABLE_FUNCTION_NOT_PROVEN_NORMAL"
        elif (
            not trace_sufficient
            or not brake_threshold_match
            or not bridge_threshold_consistent
            or brake_onset.get("status") != "AVAILABLE"
        ):
            causal = "COLLISION_TIMING_EVIDENCE_QUALITY_INSUFFICIENT"
        elif (
            not d1_valid
            or not speed_valid
            or not target_static
            or not d1_reference_verified
        ):
            causal = "COLLISION_EXPERIMENT_CONDITIONS_NOT_MATCHED"
        elif collision_miss is True and injected:
            causal = "REALTIME_INJECTION_CAUSAL_CANDIDATE_REQUIRES_GROUP_COMPARISON"
        elif safety_miss is True and collision_miss is not True:
            causal = "SAFETY_MARGIN_DEADLINE_MISS_BUT_COLLISION_NOT_EXPLAINED_BY_MODEL"
        elif response_ms is None:
            causal = "COLLISION_BRAKE_APPLY_EVIDENCE_MISSING"
        else:
            causal = "COLLISION_NOT_EXPLAINED_BY_DEADLINE_MODEL"

    result = dict(base_result)
    result.update({
        "status": (
            "ANALYZED"
            if response_ms is not None and clock_valid and deadline_computable
            else "UNKNOWN_DATA_INSUFFICIENT"
        ),
        "implicit_safety_deadline_miss": safety_miss,
        "collision_deadline_miss": collision_miss,
        "causal_assessment": causal,
        "stable_perception": {
            "target_id": target["target_id"],
            "target_id_chain": target["target_id_chain"],
            "required_consecutive_frames": int(cfg.get("stable_perception_frames", 3)),
            "stable_observation_time_sec": t1_observation,
            "stable_output_time_sec": t1_output,
            "stable_confirmation_output_time_sec": segment["stable_confirmation_output_time_sec"],
            "observation_to_output_ms": perception_availability_ms,
            "d1_raw_rel_forward_m": d1_raw,
            "distance_offset_m": offset,
            "distance_reference": d1_reference,
            "distance_reference_verified_for_physical_model": d1_reference_verified,
            "d1_m": d1,
            "rel_left_m": to_float(first.get("rel_left")),
            "target_speed_mps": to_float(first.get("speed")),
            "ego_speed_mps": measured_speed_mps,
            "ego_speed_kmh": model_speed_kmh,
            "stable_detection_lag_ms": stable_lag_ms,
            "stable_detection_lag_distance_m": stable_lag_distance,
        },
        "deadline_model": model,
        "expected_conditions": {
            "expected_d1_m": expected_d1,
            "expected_d1_source": (
                "calibration"
                if calibrated_expected_d1 is not None
                else (
                    "implicit_deadline_config"
                    if to_float(cfg.get("expected_stable_distance_m")) is not None
                    else "derived_from_configured_physics"
                )
            ),
            "d1_tolerance_m": d1_tolerance,
            "configured_speed_kmh": configured_speed_kmh,
            "speed_tolerance_kmh": speed_tolerance,
            "brake_threshold_percentage": configured_brake_threshold,
            "min_trace_complete_ratio": trace_ratio_required,
        },
        "brake_application": brake,
        "effective_brake_onset": brake_onset,
        "physical_response": {
            "stable_observation_to_control_receive_ms": to_receive_ms,
            "stable_observation_to_carla_api_return_ms": api_response_ms,
            "stable_observation_to_effective_brake_onset_ms": response_ms,
            # Backward-compatible alias used by the calibration/group tools.
            "stable_observation_to_brake_apply_ms": response_ms,
            "response_endpoint": "LOCALIZATION_SUSTAINED_DECELERATION_ONSET",
            "safety_deadline_slack_ms": safety_deadline - response_ms if safety_deadline is not None and response_ms is not None else None,
            "collision_deadline_slack_ms": collision_deadline - response_ms if collision_deadline is not None and response_ms is not None else None,
        },
        "apollo_trace_internal": trace,
        "calibration_semantic_chain": semantic_chain,
        "measured_deceleration": deceleration,
        "condition_checks": {
            "functionally_normal": functionally_normal,
            "speed_matches_config": speed_valid,
            "d1_matches_expected": d1_valid,
            "d1_reference_verified_for_physical_model": d1_reference_verified,
            "target_static": target_static,
            "clock_alignment_plausible": clock_valid,
            "deadline_model_computable": deadline_computable,
            "bridge_injection_evidence_present": brake.get("status") == "AVAILABLE",
            "brake_threshold_matches": brake_threshold_match,
            "bridge_and_analyzer_brake_threshold_match": bridge_threshold_consistent,
            "effective_brake_onset_present": brake_onset.get("status") == "AVAILABLE",
            "trace_evidence_present": trace.get("status") == "AVAILABLE",
            "trace_complete_ratio_sufficient": trace_sufficient,
            "collision_occurred": collision["occurred"],
        },
    })
    if not d1_valid:
        result["warnings"].append(
            "Measured stable-detection D1 does not instantiate the configured deadline; spawning at 50 m does not by itself make D1=34.1 m."
        )
    if not d1_reference_verified:
        result["warnings"].append(
            "D1 distance reference is not verified for the physical braking model; actor-center distance must be corrected to the chosen collision-clearance reference."
        )
    if safety_miss is True and collision_miss is not True:
        result["warnings"].append(
            "The 5 m safety-margin deadline was missed, but the ideal-model collision deadline was not."
        )
    if not clock_valid:
        result["warnings"].append("Apollo observation time and Bridge wall time are not demonstrably aligned.")
    if trace.get("status") == "AVAILABLE" and not trace_sufficient:
        result["warnings"].append("Trace complete-frame ratio is below the configured minimum.")
    if brake.get("status") == "AVAILABLE" and not brake_threshold_match:
        result["warnings"].append("Applied brake evidence does not meet the configured effective-brake threshold.")
    if not bridge_threshold_consistent:
        result["warnings"].append(
            "Bridge and analyzer use different effective-brake thresholds."
        )
    if brake_onset.get("status") != "AVAILABLE":
        result["warnings"].append(
            "Physical brake onset could not be established from sustained localization deceleration."
        )
    _write_trace_csv(out_dir / "implicit_deadline_trace_frames.csv", trace_rows)
    (out_dir / "implicit_deadline_result.json").write_text(
        json.dumps(json_safe(result), ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze a speed-conditioned implicit deadline")
    parser.add_argument("--case-dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).with_name("implicit_deadline_config.yaml"),
    )
    parser.add_argument("--functional-result", type=Path)
    parser.add_argument(
        "--functional-config", type=Path,
        default=Path(__file__).with_name("collision_classifier_config.yaml"),
    )
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--target-id")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--calculate-only", action="store_true")
    parser.add_argument("--speed-kmh", type=float)
    parser.add_argument("--deadline-ms", type=float)
    parser.add_argument("--decel", type=float)
    parser.add_argument("--safety-margin-m", type=float)
    parser.add_argument("--actuator-delay-s", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")
    if args.calculate_only:
        cfg = load_deadline_config(args.config)
        result = deadline_model(
            args.speed_kmh if args.speed_kmh is not None else float(cfg.get("speed_kmh", 80.0)),
            args.decel if args.decel is not None else float(cfg.get("decel_mps2", 10.0)),
            args.safety_margin_m if args.safety_margin_m is not None else float(cfg.get("safety_margin_m", 5.0)),
            args.actuator_delay_s if args.actuator_delay_s is not None else float(cfg.get("actuator_delay_s", 0.0)),
            None,
            args.deadline_ms if args.deadline_ms is not None else float(cfg.get("target_safety_deadline_ms", 200.0)),
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    if args.case_dir is None or args.out_dir is None:
        logging.error("--case-dir and --out-dir are required unless --calculate-only is used")
        return 2
    try:
        result = run_implicit_deadline_analysis(
            args.case_dir, args.out_dir, args.config, args.functional_result,
            args.functional_config, args.calibration, args.target_id,
            {
                "speed_kmh": args.speed_kmh,
                "target_safety_deadline_ms": args.deadline_ms,
                "decel_mps2": args.decel,
                "safety_margin_m": args.safety_margin_m,
                "actuator_delay_s": args.actuator_delay_s,
            },
        )
    except Exception as exc:
        logging.error("Implicit deadline analysis failed: %s", exc)
        return 2
    logging.info("SCB status=%s assessment=%s", result["status"], result["causal_assessment"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
