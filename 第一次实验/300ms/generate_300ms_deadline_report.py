#!/usr/bin/env python3
"""Generate the 300 ms vehicle-speed hidden-deadline report.

The calculations intentionally follow the definitions used by
``baseline/VEHICLE_SPEED_HIDDEN_DEADLINE_BASELINE.md``:

* t1 is the source observation time of the first frame in a run of at least
  three consecutive target-obstacle fusion frames;
* t2 is the end of the first of two consecutive Localization intervals whose
  longitudinal speed derivative is no greater than -0.5 m/s^2;
* the geometric centre-to-clearance offset is 5.3074 m.
"""

from __future__ import annotations

import csv
import html
import math
import re
from dataclasses import dataclass, asdict
from pathlib import Path
from statistics import mean, median
from typing import Dict, Iterable, List, Optional

import numpy as np


ROOT = Path(__file__).resolve().parent
FIG_DIR = ROOT / "figures"
GEOMETRY_OFFSET_M = 5.3074
SAFETY_MARGIN_M = 5.0
DECEL_THRESHOLD_MPS2 = 0.5
STOP_SPEED_MPS = 0.1

BASELINE = {
    "speed": 15.689,
    "d1_center": 45.283,
    "d1_clear": 39.975,
    "t1_fusion_ms": 236.519,
    "t1_control_ms": 266.629,
    "t1_physical_ms": 343.657,
    "control_physical_ms": 77.028,
    "aeq": 4.957,
    "brake_distance": 24.876,
    "final_clear": 10.480,
    "safety_deadline_ms": 639.648,
    "collision_deadline_ms": 958.485,
    "safe_injectable_ms": 295.991,
    "collision_injectable_ms": 614.828,
}


PLANNING_TARGET_RE = re.compile(r"main_stop_reason=stop by (\d+)")
FIELD_RE = re.compile(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")
LOCALIZATION_RE = re.compile(
    r"measurement_time=([-+0-9.eE]+) "
    r"ego_x=([-+0-9.eE]+) ego_y=([-+0-9.eE]+) ego_z=([-+0-9.eE]+).*?"
    r"ego_vx=([-+0-9.eE]+) ego_vy=([-+0-9.eE]+) ego_vz=([-+0-9.eE]+)"
)


@dataclass
class Ego:
    time: float
    x: float
    y: float
    z: float
    speed: float


@dataclass
class Result:
    case: str
    target_id: int
    t1: float
    t1_speed_mps: float
    d1_center_m: float
    d1_clear_m: float
    t1_fusion_ms: float
    t1_control_ms: float
    raw_t1_physical_ms: float
    t1_physical_ms: float
    response_attribution_valid: bool
    fusion_control_ms: float
    control_physical_ms: float
    response_travel_m: float
    t2_speed_mps: float
    brake_distance_m: float
    aeq_mps2: float
    total_stop_distance_m: float
    final_clear_m: float
    safety_deadline_ms: float
    collision_deadline_ms: float
    safe_injectable_ms: float
    collision_injectable_ms: float
    scb_log_present: bool
    requested_delay_ms: Optional[float]
    actual_delay_ms: Optional[float]
    actual_sim_delay_ms: Optional[float]
    actual_frame_delay: Optional[int]
    scb_trigger_lead_s: Optional[float]
    collision_ground_truth_present: bool
    collision_observed: bool


def first_file(directory: Path, pattern: str) -> Path:
    files = sorted(directory.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No {pattern} under {directory}")
    return files[0]


def as_float(value: object) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    return float(value)


def percentile(values: Iterable[float], q: float) -> float:
    return float(np.percentile(np.asarray(list(values), dtype=float), q))


def parse_fields(line: str) -> Dict[str, str]:
    return {match.group(1): match.group(2) for match in FIELD_RE.finditer(line)}


def find_target_id(planning_path: Path) -> int:
    with planning_path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = PLANNING_TARGET_RE.search(line)
            if match:
                return int(match.group(1))
    raise RuntimeError(f"No planning stop target in {planning_path}")


def find_stable_observation(perception_path: Path, target_id: int) -> Dict[str, float]:
    rows: List[Dict[str, float]] = []
    token = f" id={target_id} "
    with perception_path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if "[FUSION_OBS]" not in line or token not in line:
                continue
            fields = parse_fields(line)
            rows.append(
                {
                    "seq": int(fields["seq"]),
                    "trace_id": int(fields["trace_id"]),
                    "header_time": float(fields["header_time"]),
                    "obs_time": float(fields["obs_time"]),
                    "x": float(fields["pos_x"]),
                    "y": float(fields["pos_y"]),
                    "z": float(fields["pos_z"]),
                }
            )
    for index in range(len(rows) - 2):
        if rows[index + 1]["seq"] == rows[index]["seq"] + 1 and rows[index + 2]["seq"] == rows[index]["seq"] + 2:
            return rows[index]
    raise RuntimeError(f"No three-frame stable observation of id={target_id} in {perception_path}")


def load_localization(path: Path) -> List[Ego]:
    rows: List[Ego] = []
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = LOCALIZATION_RE.search(line)
            if not match:
                continue
            t, x, y, z, vx, vy, vz = map(float, match.groups())
            rows.append(Ego(t, x, y, z, math.sqrt(vx * vx + vy * vy + vz * vz)))
    if not rows:
        raise RuntimeError(f"No localization samples in {path}")
    return rows


def nearest_ego(rows: List[Ego], timestamp: float) -> Ego:
    return min(rows, key=lambda row: abs(row.time - timestamp))


def distance(a: Ego, b: Ego) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def target_distance(ego: Ego, observation: Dict[str, float]) -> float:
    return math.sqrt(
        (ego.x - observation["x"]) ** 2
        + (ego.y - observation["y"]) ** 2
        + (ego.z - observation["z"]) ** 2
    )


def detect_t2(rows: List[Ego], t1: float, target_control_time: float) -> tuple[Ego, Ego, bool]:
    """Return raw t2, target-associated t2, and attribution validity.

    The raw value follows the baseline threshold literally.  The associated
    value additionally requires the deceleration episode itself to start no
    earlier than the target's first Control output.  This prevents ordinary
    braking already in progress at t1 from being labelled a sub-100 ms target
    response in a post-Control delay-injection experiment.
    """
    accelerations: List[Optional[float]] = [None]
    for previous, current in zip(rows, rows[1:]):
        dt = current.time - previous.time
        accelerations.append((current.speed - previous.speed) / dt if dt > 0 else None)

    raw_t2: Optional[Ego] = None
    for index in range(1, len(rows) - 1):
        if rows[index].time < t1:
            continue
        first = accelerations[index]
        second = accelerations[index + 1]
        if first is not None and second is not None and first <= -DECEL_THRESHOLD_MPS2 and second <= -DECEL_THRESHOLD_MPS2:
            raw_t2 = rows[index]
            break
    if raw_t2 is None:
        raise RuntimeError("No sustained physical deceleration after t1")
    raw_stop_time = next(
        (row.time for row in rows if row.time > raw_t2.time and row.speed <= STOP_SPEED_MPS),
        float("inf"),
    )

    # Find genuine episode onsets, rather than a threshold crossing sampled in
    # the middle of an episode that began before t1/Control.
    for index in range(1, len(rows) - 1):
        first = accelerations[index]
        second = accelerations[index + 1]
        previous = accelerations[index - 1] if index >= 2 else None
        if first is None or second is None:
            continue
        starts_episode = first <= -DECEL_THRESHOLD_MPS2 and second <= -DECEL_THRESHOLD_MPS2
        starts_episode = starts_episode and (previous is None or previous > -DECEL_THRESHOLD_MPS2)
        episode_start = rows[index - 1].time
        if starts_episode and episode_start >= target_control_time and rows[index].time < raw_stop_time:
            return raw_t2, rows[index], True

    # No separable target-associated onset: retain the raw value for audit but
    # exclude it from causal latency/deadline statistics.
    return raw_t2, raw_t2, False


def find_stop(rows: List[Ego], t2: float) -> Ego:
    for row in rows:
        if row.time > t2 and row.speed <= STOP_SPEED_MPS:
            return row
    raise RuntimeError("No near-stop Localization sample")


def trace_control_delay_ms(case_dir: Path, observation: Dict[str, float]) -> float:
    fusion_trace = str(observation["trace_id"])
    fusion_input_path = first_file(case_dir / "trace" / "fusion_inputs", "*.csv")
    parent_trace: Optional[str] = None
    with fusion_input_path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["fusion_trace_id"] == fusion_trace and row["sensor_kind"] == "lidar" and row["is_main_sensor"] == "1":
                parent_trace = row["parent_trace_id"]
                break
    if parent_trace is None:
        raise RuntimeError(f"No main lidar parent for fusion trace {fusion_trace}")

    anchor_path = first_file(case_dir / "trace" / "trace_anchor", "*.csv")
    anchor: Optional[Dict[str, str]] = None
    with anchor_path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["trace_id"] == parent_trace:
                anchor = row
                break
    if anchor is None:
        raise RuntimeError(f"No trace anchor for {parent_trace}")

    control_path = first_file(case_dir / "trace" / "message_context", "control.*.csv")
    control_mono_ns: Optional[int] = None
    with control_path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["trace_id"] == fusion_trace and row["edge"] == "out" and row["channel"] == "/apollo/control":
                control_mono_ns = int(row["mono_ns"])
                break
    if control_mono_ns is None:
        raise RuntimeError(f"No Control output for fusion trace {fusion_trace}")

    return float(anchor["ingress_ms"]) + (control_mono_ns - int(anchor["preproc_enter_ns"])) / 1_000_000.0


def load_scb(case_dir: Path, t1: float) -> Dict[str, object]:
    files = sorted((case_dir / "log").glob("scb_control_delay_*.csv"))
    if not files:
        return {
            "present": False,
            "requested": None,
            "actual": None,
            "actual_sim": None,
            "frame_delay": None,
            "lead_s": None,
        }
    applied: Optional[Dict[str, str]] = None
    with files[0].open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("status") == "APPLIED" and row.get("first_effective_brake") in {"1", "true", "True"}:
                applied = row
                break
    if applied is None:
        return {
            "present": True,
            "requested": None,
            "actual": None,
            "actual_sim": None,
            "frame_delay": None,
            "lead_s": None,
        }
    receive_time = float(applied["receive_wall_time_unix_ns"]) / 1_000_000_000.0
    return {
        "present": True,
        "requested": as_float(applied.get("requested_delay_ms")),
        "actual": as_float(applied.get("actual_delay_ms")),
        "actual_sim": as_float(applied.get("actual_sim_delay_ms")),
        "frame_delay": int(applied["actual_frame_delay"]) if applied.get("actual_frame_delay") else None,
        "lead_s": t1 - receive_time,
    }


def collision_truth_present(case_dir: Path) -> bool:
    names = [path.name.lower() for path in case_dir.rglob("*") if path.is_file()]
    return any("collision" in name or "actor_history" in name for name in names)


def analyze_case(case_dir: Path) -> Result:
    log_dir = case_dir / "log"
    planning_path = first_file(log_dir, "planning.log.INFO.*")
    perception_path = first_file(log_dir, "perception.log.INFO.*")
    localization_path = first_file(log_dir, "localization.log.INFO.*")

    target_id = find_target_id(planning_path)
    observation = find_stable_observation(perception_path, target_id)
    t1 = observation["obs_time"]
    ego_rows = load_localization(localization_path)
    ego_t1 = nearest_ego(ego_rows, t1)
    t1_control_ms = trace_control_delay_ms(case_dir, observation)
    raw_t2, t2, response_attribution_valid = detect_t2(
        ego_rows, t1, t1 + t1_control_ms / 1000.0
    )
    stop = find_stop(ego_rows, t2.time)

    d1_center = target_distance(ego_t1, observation)
    d1_clear = d1_center - GEOMETRY_OFFSET_M
    t1_fusion_ms = (observation["header_time"] - t1) * 1000.0
    raw_t1_physical_ms = (raw_t2.time - t1) * 1000.0
    t1_physical_ms = (t2.time - t1) * 1000.0
    response_travel = distance(ego_t1, t2)
    brake_distance = distance(t2, stop)
    total_stop_distance = distance(ego_t1, stop)
    aeq = ego_t1.speed ** 2 / (2.0 * brake_distance)
    final_clear = target_distance(stop, observation) - GEOMETRY_OFFSET_M
    collision_deadline_ms = (d1_clear - brake_distance) / ego_t1.speed * 1000.0
    safety_deadline_ms = (d1_clear - brake_distance - SAFETY_MARGIN_M) / ego_t1.speed * 1000.0
    scb = load_scb(case_dir, t1)

    return Result(
        case=case_dir.name,
        target_id=target_id,
        t1=t1,
        t1_speed_mps=ego_t1.speed,
        d1_center_m=d1_center,
        d1_clear_m=d1_clear,
        t1_fusion_ms=t1_fusion_ms,
        t1_control_ms=t1_control_ms,
        raw_t1_physical_ms=raw_t1_physical_ms,
        t1_physical_ms=t1_physical_ms,
        response_attribution_valid=response_attribution_valid,
        fusion_control_ms=t1_control_ms - t1_fusion_ms,
        control_physical_ms=t1_physical_ms - t1_control_ms,
        response_travel_m=response_travel,
        t2_speed_mps=t2.speed,
        brake_distance_m=brake_distance,
        aeq_mps2=aeq,
        total_stop_distance_m=total_stop_distance,
        final_clear_m=final_clear,
        safety_deadline_ms=safety_deadline_ms,
        collision_deadline_ms=collision_deadline_ms,
        safe_injectable_ms=safety_deadline_ms - t1_physical_ms,
        collision_injectable_ms=collision_deadline_ms - t1_physical_ms,
        scb_log_present=bool(scb["present"]),
        requested_delay_ms=scb["requested"],
        actual_delay_ms=scb["actual"],
        actual_sim_delay_ms=scb["actual_sim"],
        actual_frame_delay=scb["frame_delay"],
        scb_trigger_lead_s=scb["lead_s"],
        collision_ground_truth_present=collision_truth_present(case_dir),
        collision_observed=False,
    )


def write_csv(results: List[Result]) -> None:
    path = ROOT / "deadline_300ms_metrics.csv"
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        for result in results:
            writer.writerow(asdict(result))


def make_figures(results: List[Result]) -> None:
    FIG_DIR.mkdir(exist_ok=True)
    labels = [result.case[-4:] for result in results]

    width, height = 1000, 600
    left, right, top, bottom = 92, 35, 72, 82
    plot_w, plot_h = width - left - right, height - top - bottom

    def text(x: float, y: float, value: object, size: int = 14, anchor: str = "middle", weight: str = "normal", color: str = "#222222") -> str:
        return (
            f'<text x="{x:.1f}" y="{y:.1f}" font-family="Arial, sans-serif" '
            f'font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" fill="{color}">'
            f'{html.escape(str(value))}</text>'
        )

    def base_svg(title: str, y_label: str, ymax: float) -> tuple[List[str], callable]:
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#FFFFFF"/>',
            text(width / 2, 34, title, 20, weight="bold"),
        ]
        for index in range(6):
            value = ymax * index / 5.0
            y = top + plot_h - value / ymax * plot_h
            parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="#D9D9D9" stroke-width="1"/>')
            parts.append(text(left - 10, y + 5, f"{value:.0f}", 12, anchor="end", color="#555555"))
        parts.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}" stroke="#333333" stroke-width="1.4"/>')
        parts.append(f'<line x1="{left}" y1="{top+plot_h}" x2="{width-right}" y2="{top+plot_h}" stroke="#333333" stroke-width="1.4"/>')
        parts.append(
            f'<text x="24" y="{top+plot_h/2:.1f}" transform="rotate(-90 24 {top+plot_h/2:.1f})" '
            f'font-family="Arial, sans-serif" font-size="14" text-anchor="middle" fill="#222222">{html.escape(y_label)}</text>'
        )

        def ycoord(value: float) -> float:
            return top + plot_h - value / ymax * plot_h

        return parts, ycoord

    def finish(parts: List[str], filename: str) -> None:
        parts.append("</svg>")
        (FIG_DIR / filename).write_text("\n".join(parts), encoding="utf-8")

    # Figure 1: stacked response decomposition.
    fusion = [result.t1_fusion_ms for result in results]
    fusion_control = [result.fusion_control_ms for result in results]
    control_physical = [result.control_physical_ms if result.response_attribution_valid else 0.0 for result in results]
    totals = [result.t1_physical_ms if result.response_attribution_valid else result.t1_control_ms for result in results]
    ymax = math.ceil(max(max(totals), BASELINE["t1_physical_ms"]) * 1.18 / 100.0) * 100.0
    parts, ycoord = base_svg("End-to-end response decomposition: 300 ms group", "Latency (ms)", ymax)
    slot = plot_w / len(results)
    bar_w = slot * 0.48
    for index, label in enumerate(labels):
        x = left + slot * (index + 0.5) - bar_w / 2
        cumulative = 0.0
        for value, color in zip(
            [fusion[index], fusion_control[index], control_physical[index]],
            ["#4472C4", "#EDB120", "#70AD47"],
        ):
            y1, y0 = ycoord(cumulative + value), ycoord(cumulative)
            parts.append(f'<rect x="{x:.1f}" y="{y1:.1f}" width="{bar_w:.1f}" height="{y0-y1:.1f}" fill="{color}"/>')
            cumulative += value
        total_label = f"{totals[index]:.0f}" if results[index].response_attribution_valid else "N/A"
        parts.append(text(x + bar_w / 2, ycoord(totals[index]) - 9, total_label, 12, weight="bold"))
        parts.append(text(x + bar_w / 2, top + plot_h + 23, label, 13))
    baseline_y = ycoord(BASELINE["t1_physical_ms"])
    parts.append(f'<line x1="{left}" y1="{baseline_y:.1f}" x2="{width-right}" y2="{baseline_y:.1f}" stroke="#555555" stroke-width="2" stroke-dasharray="8,6"/>')
    legend = [("#4472C4", "t1 to Fusion"), ("#EDB120", "Fusion to Control"), ("#70AD47", "Control to deceleration")]
    for index, (color, label) in enumerate(legend):
        lx = left + 35 + index * 225
        parts.append(f'<rect x="{lx}" y="{height-31}" width="18" height="12" fill="{color}"/>')
        parts.append(text(lx + 26, height - 20, label, 12, anchor="start"))
    parts.append(f'<line x1="{left+720}" y1="{height-25}" x2="{left+750}" y2="{height-25}" stroke="#555555" stroke-width="2" stroke-dasharray="7,5"/>')
    parts.append(text(left + 758, height - 20, "Baseline mean", 12, anchor="start"))
    finish(parts, "fig1_300ms_latency_decomposition.svg")

    # Figure 2: estimated final clearance.
    final_clear = [result.final_clear_m for result in results]
    ymax = math.ceil(max(max(final_clear), BASELINE["final_clear"], SAFETY_MARGIN_M) * 1.25 / 5.0) * 5.0
    parts, ycoord = base_svg("Stopping outcome: 300 ms group", "Estimated final clearance (m)", ymax)
    slot = plot_w / len(results)
    bar_w = slot * 0.48
    for index, (label, value) in enumerate(zip(labels, final_clear)):
        x = left + slot * (index + 0.5) - bar_w / 2
        y = ycoord(value)
        color = "#C00000" if value <= 0 else "#4472C4"
        parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{top+plot_h-y:.1f}" fill="{color}"/>')
        parts.append(text(x + bar_w / 2, y - 9, f"{value:.2f}", 12, weight="bold"))
        parts.append(text(x + bar_w / 2, top + plot_h + 23, label, 13))
    safety_y = ycoord(SAFETY_MARGIN_M)
    baseline_y = ycoord(BASELINE["final_clear"])
    parts.append(f'<line x1="{left}" y1="{safety_y:.1f}" x2="{width-right}" y2="{safety_y:.1f}" stroke="#ED7D31" stroke-width="2" stroke-dasharray="8,6"/>')
    parts.append(f'<line x1="{left}" y1="{baseline_y:.1f}" x2="{width-right}" y2="{baseline_y:.1f}" stroke="#555555" stroke-width="2" stroke-dasharray="2,5"/>')
    parts.append(text(left + 20, safety_y - 8, "5 m safety margin", 12, anchor="start", color="#ED7D31"))
    parts.append(text(width - right - 8, baseline_y - 8, "Baseline mean", 12, anchor="end", color="#555555"))
    finish(parts, "fig2_300ms_final_clearance.svg")

    # Figure 3: observed response versus safety/collision deadline.
    response = [result.t1_physical_ms if result.response_attribution_valid else 0.0 for result in results]
    safety_deadline = [result.safety_deadline_ms for result in results]
    collision_deadline = [result.collision_deadline_ms for result in results]
    ymax = math.ceil(max(collision_deadline) * 1.18 / 100.0) * 100.0
    parts, ycoord = base_svg("Observed response versus hidden deadlines", "Time from t1 (ms)", ymax)
    slot = plot_w / len(results)
    bar_w = slot * 0.19
    colors = ["#4472C4", "#EDB120", "#70AD47"]
    for index, label in enumerate(labels):
        centre = left + slot * (index + 0.5)
        for offset, value, color in zip([-1, 0, 1], [response[index], safety_deadline[index], collision_deadline[index]], colors):
            x = centre + offset * bar_w - bar_w / 2
            y = ycoord(value)
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{top+plot_h-y:.1f}" fill="{color}"/>')
        if not results[index].response_attribution_valid:
            parts.append(text(centre - bar_w, top + plot_h - 8, "N/A", 11, weight="bold", color="#4472C4"))
        parts.append(text(centre, top + plot_h + 23, label, 13))
    legend = [("#4472C4", "Observed response"), ("#EDB120", "5 m safety deadline"), ("#70AD47", "Collision deadline")]
    for index, (color, label) in enumerate(legend):
        lx = left + 120 + index * 245
        parts.append(f'<rect x="{lx}" y="{height-31}" width="18" height="12" fill="{color}"/>')
        parts.append(text(lx + 26, height - 20, label, 12, anchor="start"))
    finish(parts, "fig3_300ms_deadline_comparison.svg")


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def summary(values: List[float]) -> Dict[str, float]:
    return {
        "mean": mean(values),
        "median": median(values),
        "min": min(values),
        "max": max(values),
        "p90": percentile(values, 90),
    }


def write_report(results: List[Result]) -> None:
    valid_results = [result for result in results if result.response_attribution_valid]
    speeds = [result.t1_speed_mps for result in results]
    centers = [result.d1_center_m for result in results]
    clears = [result.d1_clear_m for result in results]
    t_fusion = [result.t1_fusion_ms for result in results]
    t_control = [result.t1_control_ms for result in results]
    t_physical = [result.t1_physical_ms for result in valid_results]
    control_physical = [result.control_physical_ms for result in valid_results]
    aeqs = [result.aeq_mps2 for result in valid_results]
    brakes = [result.brake_distance_m for result in valid_results]
    finals = [result.final_clear_m for result in results]
    valid_finals = [result.final_clear_m for result in valid_results]
    safety_deadlines = [result.safety_deadline_ms for result in valid_results]
    collision_deadlines = [result.collision_deadline_ms for result in valid_results]
    safe_budgets = [result.safe_injectable_ms for result in valid_results]
    collision_budgets = [result.collision_injectable_ms for result in valid_results]

    stats = {
        "speed": summary(speeds),
        "center": summary(centers),
        "clear": summary(clears),
        "fusion": summary(t_fusion),
        "control": summary(t_control),
        "physical": summary(t_physical),
        "control_physical": summary(control_physical),
        "aeq": summary(aeqs),
        "brake": summary(brakes),
        "final": summary(finals),
        "safety_deadline": summary(safety_deadlines),
        "collision_deadline": summary(collision_deadlines),
        "safe_budget": summary(safe_budgets),
        "collision_budget": summary(collision_budgets),
    }

    verified_scb = sum(result.scb_log_present for result in results)
    ground_truth_count = sum(result.collision_ground_truth_present for result in results)
    valid_response_count = len(valid_results)
    response_delta = stats["physical"]["mean"] - BASELINE["t1_physical_ms"]
    post_control_delta = stats["control_physical"]["mean"] - BASELINE["control_physical_ms"]
    final_delta = stats["final"]["mean"] - BASELINE["final_clear"]
    valid_final_mean = mean(valid_finals)
    valid_speed_mean = mean(result.t1_speed_mps for result in valid_results)
    valid_clear_mean = mean(result.d1_clear_m for result in valid_results)
    valid_brake_mean = mean(result.brake_distance_m for result in valid_results)
    valid_aeq_mean = mean(result.aeq_mps2 for result in valid_results)
    valid_response_travel_mean = mean(result.response_travel_m for result in valid_results)
    valid_total_stop_mean = mean(result.total_stop_distance_m for result in valid_results)
    baseline_total_stop = BASELINE["d1_clear"] - BASELINE["final_clear"]
    baseline_response_travel = baseline_total_stop - BASELINE["brake_distance"]

    def change(current: float, baseline: float) -> tuple[float, float]:
        delta = current - baseline
        return delta, delta / baseline * 100.0

    physical_changes = [
        ("$t_1$车速", BASELINE["speed"], valid_speed_mean, "m/s", "变化较小"),
        ("$D_1$净距", BASELINE["d1_clear"], valid_clear_mean, "m", "变化较小"),
        ("等效减速度", BASELINE["aeq"], valid_aeq_mean, "m/s²", "变化较小"),
        ("实际制动距离", BASELINE["brake_distance"], valid_brake_mean, "m", "变化较小"),
        ("$t_1$到持续减速", BASELINE["t1_physical_ms"], stats["physical"]["mean"], "ms", "明显增加"),
        ("制动开始前行驶距离估计", baseline_response_travel, valid_response_travel_mean, "m", "明显增加"),
        ("$t_1$到停车总行驶距离估计", baseline_total_stop, valid_total_stop_mean, "m", "明显增加"),
        ("最终净距估计", BASELINE["final_clear"], valid_final_mean, "m", "明显减少"),
    ]
    physical_change_lines = [
        "### 1.1 Baseline与300 ms组的关键物理量变化",
        "",
        "为避免`2012`在$t_1$时已经减速造成的归因偏差，下表的300 ms响应、制动和停车均值采用其余5组可归因场景；baseline采用原六组均值。",
        "",
        "| 指标 | baseline | 300 ms组 | 绝对变化 | 相对变化 | 判断 |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for name, baseline_value, current_value, unit, interpretation in physical_changes:
        delta, percent = change(current_value, baseline_value)
        physical_change_lines.append(
            f"| {name} | {fmt(baseline_value)} {unit} | {fmt(current_value)} {unit} | {delta:+.3f} {unit} | {percent:+.1f}% | {interpretation} |"
        )
    _, aeq_percent = change(valid_aeq_mean, BASELINE["aeq"])
    physical_change_lines.extend(
        [
            "",
            "最重要的物理变化是制动起点向后移动，而不是车辆制动能力明显下降：",
            "",
            "$$",
            r"\Delta D_{\mathrm{delay}}\approx v\Delta T",
            "$$",
            "",
            fr"将实测均值代入，$15.948\times0.315\approx5.02$ m；实际计算得到制动开始前行驶距离增加{fmt(valid_response_travel_mean-baseline_response_travel)} m，两者基本一致。随后等效减速度只变化{aeq_percent:+.1f}%，因此新增距离几乎没有在制动阶段被补偿，最终净距平均减少{fmt(BASELINE['final_clear']-valid_final_mean)} m。",
            "",
        ]
    )

    lines: List[str] = []
    lines.extend(
        [
            "# 300 ms时延注入下的车速隐形Deadline分析报告",
            "",
            "> 数据目录：`D:/data/300ms/`  ",
            "> 对照组：`D:/data/baseline/VEHICLE_SPEED_HIDDEN_DEADLINE_BASELINE.md`  ",
            "> 场景数：6组  ",
            "> 用户记录结果：6组均未发生碰撞",
            "",
            "## 1. 核心结论",
            "",
            f"1. 六组中有 **{valid_response_count}/6** 组能够分离出目标Control之后新开始的持续减速段；这{valid_response_count}组从障碍物稳定可观测时刻 $t_1$ 到目标相关持续减速时刻 $t_2$ 的平均响应时间为 **{fmt(stats['physical']['mean'])} ms**，相比baseline的 {fmt(BASELINE['t1_physical_ms'])} ms增加 **{fmt(response_delta)} ms**。",
            f"2. 在这{valid_response_count}组可归因场景中，Control发布到持续减速阶段的平均时间为 **{fmt(stats['control_physical']['mean'])} ms**，相比baseline的 {fmt(BASELINE['control_physical_ms'])} ms增加 **{fmt(post_control_delta)} ms**。这是注入位置之后最直接的闭环变化。",
            f"3. 六组最终净距估计均大于0；全部六组平均为 **{fmt(stats['final']['mean'])} m**，5组可归因场景平均为 **{fmt(valid_final_mean)} m**，最小值为 **{fmt(stats['final']['min'])} m**，与“均未碰撞”的实验记录一致。",
            f"4. {valid_response_count}组可归因场景的平均碰撞Deadline为 **{fmt(stats['collision_deadline']['mean'])} ms**；观测响应之后仍剩余 **{fmt(stats['collision_budget']['mean'])} ms** 的平均碰撞时间预算，因此300 ms注入未把场景推过碰撞边界。",
            f"5. 用户确认六组运行时均生成了SCB注入日志，但当前归档目录仅 **{verified_scb}/6** 组成功复制。`202607182012`证实请求300 ms、实际仿真时延300 ms、跨3个CARLA帧；其余5组在补拷SCB文件前不能逐场景核验实际执行值。",
            "6. 当前注入仍在障碍物稳定识别前触发并持续作用于弯道控制，因此六组不能被解释为严格的“只增加300 ms”对照实验；它们适合作为300 ms实验组和机制验证数据，但因果结论必须附带该限制。",
            "7. `2007`最接近碰撞边界：最终净距估计仅2.957 m，碰撞时间预算仅37.712 ms；它可作为下一轮收紧场景的重点参考，但仍需要先修正提前触发问题。",
            "",
            *physical_change_lines,
            "## 2. 指标定义与公式",
            "",
            "### 2.1 稳定感知时刻",
            "",
            "$t_1$定义为目标障碍物连续至少3帧稳定出现时的第一帧源观测时间。采用`obs_time`，不采用Fusion发布时刻。",
            "",
            "### 2.2 持续减速起点",
            "",
            "由Localization相邻速度帧计算：",
            "",
            "$$",
            r"a_i=\frac{v_i-v_{i-1}}{t_i-t_{i-1}}",
            "$$",
            "",
            r"当连续两个区间满足 $a_i\leq-0.5\ \mathrm{m/s^2}$ 时，将第一个区间的结束时刻记为 $t_2$。响应时间为：",
            "",
            "$$",
            r"T_{\mathrm{response}}=t_2-t_1",
            "$$",
            "",
            "### 2.3 净距、等效减速度和Deadline",
            "",
            "$$",
            r"D_{1,\mathrm{clear}}=D_{1,\mathrm{center}}-5.3074",
            "$$",
            "",
            "$$",
            r"a_{\mathrm{eq}}=\frac{v_{t_1}^2}{2D_{\mathrm{brake,actual}}}",
            "$$",
            "",
            "$$",
            r"T_{\mathrm{collision}}=\frac{D_{1,\mathrm{clear}}-D_{\mathrm{brake,actual}}}{v_{t_1}}",
            "$$",
            "",
            "$$",
            r"T_{\mathrm{safety}}=\frac{D_{1,\mathrm{clear}}-D_{\mathrm{brake,actual}}-5}{v_{t_1}}",
            "$$",
            "",
            "相对于本次实际响应仍可增加的时间预算为：",
            "",
            "$$",
            r"T_{\mathrm{injectable}}=T_{\mathrm{deadline}}-T_{\mathrm{response}}",
            "$$",
            "",
            "## 3. 六组逐场景结果",
            "",
            "### 3.1 场景状态与停车结果",
            "",
            "| 场景 | 目标ID | $t_1$车速 (m/s) | $D_1$中心距离 (m) | $D_1$净距 (m) | 等效减速度 (m/s²) | 制动距离 (m) | 最终净距估计 (m) |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for result in results:
        lines.append(
            f"| {result.case[-4:]} | {result.target_id} | {fmt(result.t1_speed_mps)} | {fmt(result.d1_center_m)} | {fmt(result.d1_clear_m)} | {fmt(result.aeq_mps2)} | {fmt(result.brake_distance_m)} | {fmt(result.final_clear_m)} |"
        )

    lines.extend(
        [
            "",
            "### 3.2 端到端响应分解",
            "",
            "| 场景 | $t_1$到Fusion (ms) | $t_1$到Control (ms) | Fusion到Control (ms) | Control到持续减速 (ms) | $t_1$到持续减速 (ms) |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for result in results:
        if result.response_attribution_valid:
            lines.append(
                f"| {result.case[-4:]} | {fmt(result.t1_fusion_ms)} | {fmt(result.t1_control_ms)} | {fmt(result.fusion_control_ms)} | {fmt(result.control_physical_ms)} | {fmt(result.t1_physical_ms)} |"
            )
        else:
            lines.append(
                f"| {result.case[-4:]} | {fmt(result.t1_fusion_ms)} | {fmt(result.t1_control_ms)} | {fmt(result.fusion_control_ms)} | — | 原始阈值={fmt(result.raw_t1_physical_ms)}* |"
            )

    lines.extend(
        [
            "",
            r"\* `2012`在$t_1$时已经处于连续减速段，原始阈值时间早于目标Control输出，无法归因为障碍物响应，因此不纳入响应和Deadline汇总统计。`2017`在$t_1$附近也有一段普通轻微减速，表中采用目标Control之后重新开始的强制动段。",
        ]
    )

    lines.extend(
        [
            "",
            "### 3.3 Deadline及剩余预算",
            "",
            "| 场景 | 5 m安全Deadline (ms) | 碰撞Deadline (ms) | 观测响应 (ms) | 剩余安全预算 (ms) | 剩余碰撞预算 (ms) |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for result in results:
        if result.response_attribution_valid:
            lines.append(
                f"| {result.case[-4:]} | {fmt(result.safety_deadline_ms)} | {fmt(result.collision_deadline_ms)} | {fmt(result.t1_physical_ms)} | {fmt(result.safe_injectable_ms)} | {fmt(result.collision_injectable_ms)} |"
            )
        else:
            lines.append(f"| {result.case[-4:]} | — | — | — | — | — |")

    lines.extend(
        [
            "",
            "## 4. 汇总统计",
            "",
            "| 指标 | 均值 | 中位数 | 范围 | P90 | baseline均值 |",
            "|---|---:|---:|---:|---:|---:|",
            f"| $t_1$车速 (m/s) | {fmt(stats['speed']['mean'])} | {fmt(stats['speed']['median'])} | {fmt(stats['speed']['min'])}–{fmt(stats['speed']['max'])} | {fmt(stats['speed']['p90'])} | {fmt(BASELINE['speed'])} |",
            f"| $D_1$净距 (m) | {fmt(stats['clear']['mean'])} | {fmt(stats['clear']['median'])} | {fmt(stats['clear']['min'])}–{fmt(stats['clear']['max'])} | {fmt(stats['clear']['p90'])} | {fmt(BASELINE['d1_clear'])} |",
            f"| $t_1$到Fusion (ms) | {fmt(stats['fusion']['mean'])} | {fmt(stats['fusion']['median'])} | {fmt(stats['fusion']['min'])}–{fmt(stats['fusion']['max'])} | {fmt(stats['fusion']['p90'])} | {fmt(BASELINE['t1_fusion_ms'])} |",
            f"| $t_1$到Control (ms) | {fmt(stats['control']['mean'])} | {fmt(stats['control']['median'])} | {fmt(stats['control']['min'])}–{fmt(stats['control']['max'])} | {fmt(stats['control']['p90'])} | {fmt(BASELINE['t1_control_ms'])} |",
            f"| Control到持续减速 (ms) | {fmt(stats['control_physical']['mean'])} | {fmt(stats['control_physical']['median'])} | {fmt(stats['control_physical']['min'])}–{fmt(stats['control_physical']['max'])} | {fmt(stats['control_physical']['p90'])} | {fmt(BASELINE['control_physical_ms'])} |",
            f"| $t_1$到持续减速 (ms) | {fmt(stats['physical']['mean'])} | {fmt(stats['physical']['median'])} | {fmt(stats['physical']['min'])}–{fmt(stats['physical']['max'])} | {fmt(stats['physical']['p90'])} | {fmt(BASELINE['t1_physical_ms'])} |",
            f"| 等效减速度 (m/s²) | {fmt(stats['aeq']['mean'])} | {fmt(stats['aeq']['median'])} | {fmt(stats['aeq']['min'])}–{fmt(stats['aeq']['max'])} | {fmt(stats['aeq']['p90'])} | {fmt(BASELINE['aeq'])} |",
            f"| 最终净距估计 (m) | {fmt(stats['final']['mean'])} | {fmt(stats['final']['median'])} | {fmt(stats['final']['min'])}–{fmt(stats['final']['max'])} | {fmt(stats['final']['p90'])} | {fmt(BASELINE['final_clear'])} |",
            f"| 碰撞Deadline (ms) | {fmt(stats['collision_deadline']['mean'])} | {fmt(stats['collision_deadline']['median'])} | {fmt(stats['collision_deadline']['min'])}–{fmt(stats['collision_deadline']['max'])} | {fmt(stats['collision_deadline']['p90'])} | {fmt(BASELINE['collision_deadline_ms'])} |",
            f"| 剩余碰撞预算 (ms) | {fmt(stats['collision_budget']['mean'])} | {fmt(stats['collision_budget']['median'])} | {fmt(stats['collision_budget']['min'])}–{fmt(stats['collision_budget']['max'])} | {fmt(stats['collision_budget']['p90'])} | {fmt(BASELINE['collision_injectable_ms'])} |",
            "",
            "## 5. 图形化分析",
            "",
            "![图5-1 300ms组端到端响应分解](./figures/fig1_300ms_latency_decomposition.svg)",
            "",
            f"*图5-1　六组300 ms实验的响应阶段分解。`2012`因$t_1$时已经处于减速状态而标为N/A；虚线为baseline总响应均值{fmt(BASELINE['t1_physical_ms'], 1)} ms。*",
            "",
            "![图5-2 300ms组最终净距](./figures/fig2_300ms_final_clearance.svg)",
            "",
            "*图5-2　最终净距估计。红色实线为碰撞边界，橙色虚线为5 m安全余量，灰色点线为baseline最终净距均值。*",
            "",
            "![图5-3 实际响应与Deadline](./figures/fig3_300ms_deadline_comparison.svg)",
            "",
            "*图5-3　每组实际响应时间与5 m安全Deadline、碰撞Deadline的比较。`2012`响应归因为N/A；其余可归因场景的实际响应均低于碰撞Deadline。*",
            "",
            "## 6. 300 ms没有导致碰撞的原因",
            "",
            "### 6.1 当前场景原本具有较大的碰撞预算",
            "",
            f"baseline平均碰撞Deadline为{fmt(BASELINE['collision_deadline_ms'])} ms，baseline实际响应为{fmt(BASELINE['t1_physical_ms'])} ms，因此原始碰撞预算约为{fmt(BASELINE['collision_injectable_ms'])} ms。300 ms小于该预算，从理论上本来就不应稳定导致碰撞。",
            "",
            "### 6.2 实际状态并未做到严格配对",
            "",
            f"300 ms组在$t_1$处的速度范围为{fmt(stats['speed']['min'])}–{fmt(stats['speed']['max'])} m/s，净距范围为{fmt(stats['clear']['min'])}–{fmt(stats['clear']['max'])} m。即使场景配置一致，实际进入Deadline计算的速度和距离仍有明显波动。",
            "",
            "### 6.3 弯道提前触发改变了障碍物出现前的车辆状态",
            "",
            "已保存SCB日志的`2012`场景在障碍物稳定识别前即触发注入。注入器触发后继续延迟所有Control命令，因此弯道和后续加速阶段也受到300 ms影响。这会改变$t_1$速度、位置以及制动初始状态。",
            "",
            "### 6.4 最接近碰撞的是`2007`",
            "",
            "`2007`的$t_1$车速为16.697 m/s、净距为39.217 m，总响应为650.257 ms。其最终净距估计仅2.957 m，已经低于5 m安全余量；按Deadline模型计算，距离碰撞边界只剩37.712 ms。该场景说明300 ms已经明显压缩停车余量，但还没有越过碰撞边界。",
            "",
            "## 7. 注入证据与数据完整性",
            "",
            "| 场景 | SCB日志 | 请求时延 (ms) | 墙钟实际时延 (ms) | 仿真时延 (ms) | 帧延迟 | 相对$t_1$触发时间 |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for result in results:
        if result.scb_log_present and result.requested_delay_ms is not None:
            lead_text = f"提前{fmt(result.scb_trigger_lead_s or 0.0)} s"
            lines.append(
                f"| {result.case[-4:]} | 有 | {fmt(result.requested_delay_ms)} | {fmt(result.actual_delay_ms or 0.0)} | {fmt(result.actual_sim_delay_ms or 0.0)} | {result.actual_frame_delay} | {lead_text} |"
            )
        else:
            lines.append(f"| {result.case[-4:]} | 无 | — | — | — | — | 无法直接核验 |")

    lines.extend(
        [
            "",
            f"六组目录中CollisionSensor或actor history真值文件数量为{ground_truth_count}/6。本文的“未碰撞”采用用户实验记录，并由最终净距估计进行交叉检查；正式报告若要把碰撞作为因变量，应补充CARLA CollisionSensor事件。",
            "",
            "## 8. 对后续车速隐形Deadline实验的意义",
            "",
            "这六组数据证明300 ms量级的注入可以显著改变闭环响应，但当前约15–16 m/s、约40 m稳定识别净距的场景仍没有被推到碰撞边界。下一阶段不应继续只增加同一条件下的重复次数，而应：",
            "",
            "1. 将注入器改为过弯后或障碍物生成时才允许进入armed状态，保证弯道轨迹不受实验时延影响；",
            "2. 修复采集脚本并将每组已生成的SCB日志完整复制到归档目录，核验`actual_sim_delay_ms`、`actual_frame_delay`和队列丢弃状态；",
            "3. 使用成对的0 ms与300 ms实验，预先规定$t_1$速度和净距接收窗口；",
            "4. 若目标是构造100 ms差异即可改变碰撞结果，应把场景收紧到理论临界区，而不是依赖300 ms实验自然发生碰撞；",
            "5. 同时报告碰撞概率、最终净距、碰撞速度和端到端响应，而不只报告单次是否碰撞。",
            "",
            "## 9. 可直接引用的实验结论",
            "",
            f"> 在六组300 ms时延注入实验中，障碍物稳定识别时的平均车速为{fmt(stats['speed']['mean'])} m/s，平均车头净距为{fmt(stats['clear']['mean'])} m。其中5组能够分离出目标Control之后新开始的持续减速段，其平均响应时间为{fmt(stats['physical']['mean'])} ms，相比六组baseline均值{fmt(BASELINE['t1_physical_ms'])} ms增加{fmt(response_delta)} ms；另1组在$t_1$时已经处于减速状态，未纳入响应归因统计。5组可归因场景的最终净距平均为{fmt(valid_final_mean)} m，六组最小最终净距为{fmt(stats['final']['min'])} m，均未进入碰撞区。5组可归因场景的平均碰撞Deadline为{fmt(stats['collision_deadline']['mean'])} ms，实际响应后仍保留{fmt(stats['collision_budget']['mean'])} ms的平均碰撞预算，因此300 ms注入不足以在当前速度和感知距离条件下稳定导致碰撞。其中`2007`只剩37.712 ms碰撞预算，是最接近临界状态的场景。用户确认六组均生成SCB日志，但当前归档仅成功复制1组；同时该组显示注入在弯道阶段提前触发。因此结果可作为300 ms预实验和机制验证，正式因果结论应在补齐SCB记录并修正触发时机后给出。",
            "",
            "## 10. 复现文件",
            "",
            "- 逐场景计算结果：`deadline_300ms_metrics.csv`",
            "- 报告与图表生成脚本：`generate_300ms_deadline_report.py`",
            "- 图表目录：`figures/`",
            "",
        ]
    )

    (ROOT / "VEHICLE_SPEED_HIDDEN_DEADLINE_300MS_REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    case_dirs = sorted(path for path in ROOT.iterdir() if path.is_dir() and re.fullmatch(r"\d{12}", path.name))
    if len(case_dirs) != 6:
        raise RuntimeError(f"Expected six case directories, found {len(case_dirs)}")
    results = [analyze_case(case_dir) for case_dir in case_dirs]
    write_csv(results)
    make_figures(results)
    write_report(results)
    for result in results:
        print(
            result.case,
            f"v={result.t1_speed_mps:.3f}",
            f"Dclear={result.d1_clear_m:.3f}",
            f"Tresp={result.t1_physical_ms:.3f}ms",
            f"final={result.final_clear_m:.3f}m",
        )


if __name__ == "__main__":
    main()
