#!/usr/bin/env python3
"""Read-only diagnostic comparison for runs 202607271131 and 202607271211."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from zoneinfo import ZoneInfo


EXPERIMENT_ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = EXPERIMENT_ROOT.parent
CORE_ROOT = WORKSPACE_ROOT / "realtime_collision_analysis" / "src"
RUNTIME_PACKAGES = (
    WORKSPACE_ROOT / "realtime_collision_analysis" / ".python_packages"
)
sys.path.insert(0, str(RUNTIME_PACKAGES))
sys.path.insert(0, str(CORE_ROOT))

import numpy as np
import yaml

import realtime_collision_core as core


def make_config() -> dict[str, object]:
    path = (
        WORKSPACE_ROOT
        / "realtime_collision_analysis"
        / "config"
        / "analysis_config.yaml"
    )
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    config["analysis"]["pointcloud_count"] = 560_000
    config["groups"] = {
        "delay_300ms": {
            "root": str(EXPERIMENT_ROOT / "300ms"),
            "nominal_injected_delay_ms": 300.0,
            "expected_runs": 5,
        }
    }
    config["stable_perception"]["sensitivity_frames"] = [
        config["stable_perception"]["primary_frames"]
    ]
    config["effective_brake"]["sensitivity_thresholds_mps2"] = [
        config["effective_brake"]["primary_decel_threshold_mps2"]
    ]
    return config


def state(rows: list[core.EgoSample], timestamp_s: float) -> dict[str, float]:
    value = core.interpolate_sample(rows, timestamp_s)
    return value or {}


def relative_geometry(
    target_x: float,
    target_y: float,
    target_z: float,
    ego: dict[str, float],
    offset_m: float,
) -> dict[str, float]:
    dx = target_x - ego["x_m"]
    dy = target_y - ego["y_m"]
    dz = target_z - ego["z_m"]
    heading = ego["heading_rad"]
    return {
        "center_longitudinal_m": dx * math.cos(heading)
        + dy * math.sin(heading),
        "clear_longitudinal_m": dx * math.cos(heading)
        + dy * math.sin(heading)
        - offset_m,
        "lateral_m": -dx * math.sin(heading) + dy * math.cos(heading),
        "euclidean_center_m": math.sqrt(dx * dx + dy * dy + dz * dz),
        "euclidean_surface_m": math.sqrt(dx * dx + dy * dy + dz * dz)
        - offset_m,
    }


def first_crossing(
    rows: list[core.EgoSample],
    start_s: float,
    end_s: float,
    threshold_mps: float,
) -> dict[str, float] | None:
    for row in rows:
        if start_s <= row.time_s <= end_s and row.speed_mps <= threshold_mps:
            return {
                "elapsed_s": row.time_s - start_s,
                "time_s": row.time_s,
                "speed_mps": row.speed_mps,
            }
    return None


def sample_after(
    rows: list[core.EgoSample],
    start_s: float,
    elapsed_s: float,
) -> dict[str, float] | None:
    sample = core.interpolate_sample(rows, start_s + elapsed_s)
    if sample is None:
        return None
    return {
        "elapsed_s": elapsed_s,
        "speed_mps": sample["speed_mps"],
        "distance_m": core.integrate_speed(
            rows, start_s, start_s + elapsed_s
        ),
    }


def actor_history_summary(path: Path | None) -> dict[str, object]:
    if path is None:
        return {}
    roles: dict[str, list[dict[str, str]]] = {}
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        for row in csv.DictReader(handle):
            roles.setdefault(str(row.get("role")), []).append(row)
    result: dict[str, object] = {}
    for role, rows in roles.items():
        last = rows[-1]
        result[role] = {
            "count": len(rows),
            "actor_id": last.get("actor_id"),
            "type": last.get("actor_type"),
            "last_time_s": core.fnum(last.get("wall_time_unix_ns")) / 1e9,
            "last_x": core.fnum(last.get("location_x")),
            "last_y": core.fnum(last.get("location_y")),
            "last_z": core.fnum(last.get("location_z")),
            "last_speed_mps": math.sqrt(
                core.fnum(last.get("velocity_x"), 0.0) ** 2
                + core.fnum(last.get("velocity_y"), 0.0) ** 2
                + core.fnum(last.get("velocity_z"), 0.0) ** 2
            ),
        }
    return result


def run_diagnostic(
    parsed: core.ParsedRun,
    raw: dict[str, object],
    debug: dict[str, object],
    config: dict[str, object],
) -> dict[str, object]:
    stable = parsed.perception["stable"]
    t1_s = core.fnum(raw.get("t_sensor_origin_s"))
    t2_s = core.fnum(raw.get("t_brake_effective_s"))
    collision_s = core.fnum(raw.get("t_collision_s"))
    endpoint_s = (
        collision_s
        if bool(raw.get("collision"))
        else core.fnum(debug.get("brake_completion", {}).get("time_s"))
    )
    t1_state = state(parsed.localization, t1_s)
    t2_state = state(parsed.localization, t2_s)
    endpoint_state = state(parsed.localization, endpoint_s)
    offset_m = float(config["geometry"]["combined_center_to_surface_offset_m"])
    target_rows: list[core.FusionObservation] = parsed.perception.get(
        "target_rows", []
    )
    before_endpoint = [
        row for row in target_rows if row.header_time_s <= endpoint_s
    ]
    last_target = before_endpoint[-1] if before_endpoint else None
    target_positions = np.asarray(
        [[row.x_m, row.y_m, row.z_m] for row in target_rows], dtype=float
    )
    braking = debug.get("braking", {})
    result = {
        "run_id": parsed.spec.run_id,
        "collision": bool(raw.get("collision")),
        "impact_speed_mps": core.fnum(raw.get("impact_speed_mps")),
        "t1_s": t1_s,
        "t2_s": t2_s,
        "endpoint_s": endpoint_s,
        "t1_to_t2_ms": core.fnum(raw.get("actual_e2e_latency_ms")),
        "v1_mps": t1_state.get("speed_mps"),
        "v2_mps": t2_state.get("speed_mps"),
        "D1_clear_m": relative_geometry(
            stable.x_m, stable.y_m, stable.z_m, t1_state, offset_m
        )["clear_longitudinal_m"],
        "D_delay_m": core.integrate_speed(parsed.localization, t1_s, t2_s),
        "D2_clear_m": relative_geometry(
            stable.x_m, stable.y_m, stable.z_m, t2_state, offset_m
        )["clear_longitudinal_m"],
        "stable_target": {
            "id": stable.obstacle_id,
            "x": stable.x_m,
            "y": stable.y_m,
            "z": stable.z_m,
            "speed_mps": stable.speed_mps,
            "obs_time_s": stable.obs_time_s,
            "header_time_s": stable.header_time_s,
        },
        "target_observation_count": len(target_rows),
        "target_position_min": target_positions.min(axis=0).tolist(),
        "target_position_max": target_positions.max(axis=0).tolist(),
        "last_target_before_endpoint": (
            {
                "x": last_target.x_m,
                "y": last_target.y_m,
                "z": last_target.z_m,
                "speed_mps": last_target.speed_mps,
                "header_time_s": last_target.header_time_s,
            }
            if last_target
            else None
        ),
        "t1_ego": t1_state,
        "t2_ego": t2_state,
        "endpoint_ego": endpoint_state,
        "endpoint_geometry_using_stable_target": relative_geometry(
            stable.x_m,
            stable.y_m,
            stable.z_m,
            endpoint_state,
            offset_m,
        ),
        "endpoint_geometry_using_last_target": (
            relative_geometry(
                last_target.x_m,
                last_target.y_m,
                last_target.z_m,
                endpoint_state,
                offset_m,
            )
            if last_target
            else None
        ),
        "braking": {
            key: value
            for key, value in braking.items()
            if key
            in {
                "status",
                "endpoint",
                "start_speed_mps",
                "duration_s",
                "distance_m",
                "path_length_m",
                "displacement_m",
                "mean_deceleration_mps2",
                "peak_deceleration_mps2",
                "deceleration_p10_mps2",
                "deceleration_p50_mps2",
                "deceleration_p90_mps2",
                "effective_deceleration_mps2",
                "preimpact_effective_deceleration_mps2",
            }
        },
        "speed_threshold_crossings": {
            str(threshold): first_crossing(
                parsed.localization, t2_s, endpoint_s, threshold
            )
            for threshold in [15.0, 12.0, 10.0, 8.0, 5.0, 1.0, 0.1]
        },
        "post_t2_samples": [
            sample_after(parsed.localization, t2_s, elapsed)
            for elapsed in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
            if t2_s + elapsed <= endpoint_s
        ],
        "planning_fallback": {
            "primal_infeasible": raw.get("planning_primal_infeasible_count"),
            "speed_fallback": raw.get("planning_speed_fallback_count"),
            "constant_deceleration_fallback": raw.get(
                "planning_constant_deceleration_fallback_count"
            ),
        },
        "scb": {
            "requested_delay_ms": raw.get("scb_requested_delay_ms"),
            "actual_first_applied_delay_ms": raw.get(
                "scb_actual_wall_delay_ms"
            ),
        },
        "actor_history": actor_history_summary(
            parsed.files.get("actor_history")
        ),
    }
    return result


def main() -> None:
    config = make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    results: list[dict[str, object]] = []
    for spec in core.discover_runs(config):
        if spec.run_id not in {"202607271131", "202607271211"}:
            continue
        parsed = core.parse_run(spec, config, timezone)
        raw, debug = core.raw_run_metrics(parsed, config)
        results.append(run_diagnostic(parsed, raw, debug, config))
    print(json.dumps(results, ensure_ascii=False, indent=2, default=str))


if __name__ == "__main__":
    main()
