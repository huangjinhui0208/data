#!/usr/bin/env python3
"""Recover the pre-collision Fusion gap in run 202607271131.

Observed quantities and counterfactual calculations are stored separately.
The contact distance is the wall-clock trapezoidal speed integral from t2 to
the recorded CARLA collision event; no braking-distance prediction replaces
the observed run result.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

import diagnose_202607271131_vs_202607271211 as diagnostic


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "analysis_results" / "chapter5_fusion_gap_recovery_1131.json"
COLLISION_RUN = "202607271131"
REFERENCE_RUN = "202607271211"


def load_runs():
    core = diagnostic.core
    config = diagnostic.make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    parsed = {}
    raw = {}
    debug = {}
    for spec in core.discover_runs(config):
        if spec.run_id not in {COLLISION_RUN, REFERENCE_RUN}:
            continue
        run = core.parse_run(spec, config, timezone)
        metrics, diagnostics = core.raw_run_metrics(run, config)
        parsed[spec.run_id] = run
        raw[spec.run_id] = metrics
        debug[spec.run_id] = diagnostics
    return core, parsed, raw, debug


def recovery_state(core, run, t2_s, collision_s, action_s):
    state = core.interpolate_sample(run.localization, action_s)
    if state is None:
        raise RuntimeError(f"No Localization state at {action_s}")
    contact_path_m = core.integrate_speed(run.localization, t2_s, collision_s)
    used_path_m = core.integrate_speed(run.localization, t2_s, action_s)
    remaining_path_m = contact_path_m - used_path_m
    speed_mps = float(state["speed_mps"])
    required_decel_mps2 = speed_mps**2 / (2.0 * remaining_path_m)
    return {
        "action_time_s": action_s,
        "action_elapsed_from_t2_s": action_s - t2_s,
        "time_to_recorded_collision_s": collision_s - action_s,
        "speed_mps": speed_mps,
        "observed_path_used_from_t2_m": used_path_m,
        "observed_path_remaining_to_collision_m": remaining_path_m,
        "required_constant_deceleration_to_stop_at_contact_mps2": (
            required_decel_mps2
        ),
        "required_stop_time_s": speed_mps / required_decel_mps2,
    }


def normalized_reference_replay(
    core,
    collision_run,
    reference_run,
    collision_t2_s,
    reference_t2_s,
    reference_end_s,
    collision_contact_path_m,
    action_s,
):
    """Replay the reference run's normalized post-t2 speed profile.

    The collision run remains unchanged before action_s. From action_s onward,
    the reference run's speed curve at the same elapsed time after t2 is scaled
    to match the collision run's speed at the splice point.
    """

    action_elapsed_s = action_s - collision_t2_s
    collision_state = core.interpolate_sample(collision_run.localization, action_s)
    reference_state = core.interpolate_sample(
        reference_run.localization, reference_t2_s + action_elapsed_s
    )
    if collision_state is None or reference_state is None:
        raise RuntimeError("Missing Localization state for reference replay")
    scale = collision_state["speed_mps"] / reference_state["speed_mps"]
    observed_before_action_m = core.integrate_speed(
        collision_run.localization, collision_t2_s, action_s
    )
    required_after_action_m = collision_contact_path_m - observed_before_action_m
    reference_end_elapsed_s = reference_end_s - reference_t2_s
    elapsed = np.arange(action_elapsed_s, reference_end_elapsed_s, 0.0005)
    if elapsed.size == 0 or elapsed[-1] < reference_end_elapsed_s:
        elapsed = np.append(elapsed, reference_end_elapsed_s)
    speed = np.asarray(
        [
            core.interpolate_sample(
                reference_run.localization, reference_t2_s + float(value)
            )["speed_mps"]
            for value in elapsed
        ],
        dtype=float,
    )
    speed *= scale
    distance = np.zeros(elapsed.size, dtype=float)
    distance[1:] = np.cumsum(
        0.5 * (speed[:-1] + speed[1:]) * np.diff(elapsed)
    )
    crossings = np.flatnonzero(distance >= required_after_action_m)
    result = {
        "method": (
            "Observed collision-run trajectory before the splice; afterward, "
            "the observed non-collision reference speed curve at the same "
            "elapsed time after t2, normalized to the splice speed."
        ),
        "action_elapsed_from_t2_s": action_elapsed_s,
        "speed_scale": float(scale),
    }
    if crossings.size == 0:
        result.update(
            {
                "reaches_recorded_collision_point": False,
                "remaining_margin_m": float(
                    required_after_action_m - distance[-1]
                ),
                "terminal_speed_mps": float(speed[-1]),
            }
        )
        return result

    index = int(crossings[0])
    prior = max(index - 1, 0)
    segment_distance = distance[index] - distance[prior]
    fraction = (
        (required_after_action_m - distance[prior]) / segment_distance
        if segment_distance > 0.0
        else 0.0
    )
    contact_elapsed_s = elapsed[prior] + fraction * (
        elapsed[index] - elapsed[prior]
    )
    contact_speed_mps = speed[prior] + fraction * (
        speed[index] - speed[prior]
    )
    result.update(
        {
            "reaches_recorded_collision_point": True,
            "contact_elapsed_from_t2_s": float(contact_elapsed_s),
            "contact_speed_mps": float(contact_speed_mps),
        }
    )
    return result


def main() -> None:
    core, parsed, raw, debug = load_runs()
    collision_run = parsed[COLLISION_RUN]
    reference_run = parsed[REFERENCE_RUN]
    collision_raw = raw[COLLISION_RUN]

    t2_s = core.fnum(collision_raw["t_brake_effective_s"])
    collision_s = core.fnum(collision_raw["t_collision_s"])
    fusion_frames = sorted(
        [
            row
            for row in collision_run.perception["fusion_frames"]
            if t2_s <= row["header_time_s"] <= collision_s
        ],
        key=lambda row: row["header_time_s"],
    )
    gap_s, before_gap, after_gap = max(
        (
            (
                fusion_frames[index + 1]["header_time_s"]
                - fusion_frames[index]["header_time_s"],
                fusion_frames[index],
                fusion_frames[index + 1],
            )
            for index in range(len(fusion_frames) - 1)
        ),
        key=lambda item: item[0],
    )

    gap_start_s = before_gap["header_time_s"]
    gap_end_s = after_gap["header_time_s"]
    source_period_s = (
        after_gap["sensor_time_s"] - before_gap["sensor_time_s"]
    )
    expected_next_fusion_s = gap_start_s + source_period_s
    software_after_fusion_ms = sum(
        core.fnum(collision_raw[key])
        for key in [
            "perception_to_prediction_ms",
            "prediction_to_planning_stop_ms",
            "planning_stop_to_control_ms",
        ]
    )
    scb_delay_ms = core.fnum(collision_raw["scb_actual_wall_delay_ms"])
    expected_control_s = expected_next_fusion_s + software_after_fusion_ms / 1000.0
    expected_bridge_apply_s = expected_control_s + scb_delay_ms / 1000.0

    gap_start_state = core.interpolate_sample(
        collision_run.localization, gap_start_s
    )
    gap_end_state = core.interpolate_sample(collision_run.localization, gap_end_s)
    gap_distance_m = core.integrate_speed(
        collision_run.localization, gap_start_s, gap_end_s
    )
    contact_path_m = core.integrate_speed(
        collision_run.localization, t2_s, collision_s
    )
    observed_gap_decel_mps2 = (
        gap_start_state["speed_mps"] - gap_end_state["speed_mps"]
    ) / gap_s

    action_points = {
        "optimistic_gap_start": gap_start_s,
        "expected_next_fusion_immediate_effect": expected_next_fusion_s,
        "expected_control_after_software_only": expected_control_s,
        "expected_bridge_apply_after_software_and_scb": expected_bridge_apply_s,
        "actual_next_fusion_immediate_effect": gap_end_s,
    }
    recovery_states = {
        name: recovery_state(
            core, collision_run, t2_s, collision_s, action_s
        )
        for name, action_s in action_points.items()
    }

    reference_t2_s = core.fnum(raw[REFERENCE_RUN]["t_brake_effective_s"])
    reference_end_s = core.fnum(
        debug[REFERENCE_RUN]["brake_completion"]["time_s"]
    )
    replay = normalized_reference_replay(
        core,
        collision_run,
        reference_run,
        t2_s,
        reference_t2_s,
        reference_end_s,
        contact_path_m,
        expected_next_fusion_s,
    )

    braking = debug[COLLISION_RUN]["braking"]
    result = {
        "scope": {
            "collision_run": COLLISION_RUN,
            "display_name": "碰撞_1131",
            "reference_run": REFERENCE_RUN,
            "reference_display_name": "未碰撞_1211",
            "distance_basis": (
                "Wall-clock trapezoidal speed integral to the recorded CARLA "
                "collision event."
            ),
        },
        "observed": {
            "t2_s": t2_s,
            "collision_s": collision_s,
            "contact_path_from_t2_m": contact_path_m,
            "impact_speed_mps": core.fnum(collision_raw["impact_speed_mps"]),
            "fusion_gap_start_s": gap_start_s,
            "fusion_gap_end_s": gap_end_s,
            "fusion_output_gap_ms": gap_s * 1000.0,
            "adjacent_sensor_period_ms": source_period_s * 1000.0,
            "excess_gap_over_source_period_ms": (
                gap_s - source_period_s
            )
            * 1000.0,
            "before_gap_fusion_lifecycle_ms": before_gap["latency_ms"],
            "after_gap_fusion_lifecycle_ms": after_gap["latency_ms"],
            "gap_start_elapsed_from_t2_s": gap_start_s - t2_s,
            "gap_end_elapsed_from_t2_s": gap_end_s - t2_s,
            "time_from_gap_end_to_collision_s": collision_s - gap_end_s,
            "gap_start_speed_mps": gap_start_state["speed_mps"],
            "gap_end_speed_mps": gap_end_state["speed_mps"],
            "gap_speed_reduction_mps": (
                gap_start_state["speed_mps"] - gap_end_state["speed_mps"]
            ),
            "gap_distance_m": gap_distance_m,
            "gap_mean_speed_mps": gap_distance_m / gap_s,
            "gap_mean_deceleration_mps2": observed_gap_decel_mps2,
            "preimpact_deceleration_p90_mps2": braking[
                "deceleration_p90_mps2"
            ],
            "preimpact_peak_deceleration_mps2": braking[
                "peak_deceleration_mps2"
            ],
        },
        "counterfactual_inputs": {
            "expected_next_fusion_s": expected_next_fusion_s,
            "software_after_fusion_ms": software_after_fusion_ms,
            "scb_actual_wall_delay_ms": scb_delay_ms,
        },
        "counterfactual_recovery_states": recovery_states,
        "counterfactual_reference_replay": replay,
        "interpretation": {
            "message_only_recovery_with_unchanged_control": (
                "Collision remains unchanged because the observed vehicle "
                "trajectory is unchanged."
            ),
            "avoidance_condition": (
                "A restored message avoids collision only if it changes the "
                "applied braking enough to meet the required sustained "
                "deceleration before the recorded contact path is exhausted."
            ),
        },
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
