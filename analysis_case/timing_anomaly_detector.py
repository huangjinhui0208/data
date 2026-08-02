#!/usr/bin/env python3
"""Post-collision per-frame explicit configured-threshold detector.

The detector reads t1/t2 from the functional classifier result (or explicit
CLI overrides), pairs the same trace frame across the full lidar perception
pipeline, prediction, and planning, and writes both all-frame and anomalous-
frame evidence files.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import re
import statistics
from html import escape
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


MODULE_PATTERNS = {
    "perception_start": "perception.pointcloud_preprocess.*.csv",
    "perception_end": "perception.multi_sensor_fusion.*.csv",
    "prediction": "prediction.*.csv",
    "planning": "planning.*.csv",
}

FRAME_FIELDS = [
    "frame_index",
    "trace_id",
    "parent_trace_id",
    "data_ts_ns",
    "data_time_sec",
    "target_present",
    "frame_status",
    "perception_ms",
    "prediction_ms",
    "planning_ms",
    "e2e_ms",
    "perception_threshold_ms",
    "prediction_threshold_ms",
    "planning_threshold_ms",
    "e2e_threshold_ms",
    "e2e_deadline_miss",
    "perception_deadline_miss",
    "prediction_deadline_miss",
    "planning_deadline_miss",
    "cause_modules",
    "e2e_overrun_ms",
    "perception_overrun_ms",
    "prediction_overrun_ms",
    "planning_overrun_ms",
    "perception_to_prediction_handoff_ms",
    "prediction_to_planning_handoff_ms",
    "perception_start_mono_ns",
    "perception_output_mono_ns",
    "prediction_start_mono_ns",
    "prediction_output_mono_ns",
    "planning_start_mono_ns",
    "planning_output_mono_ns",
    "input_seq",
    "output_seq",
    "object_count",
]


def _to_float(value: Any) -> Optional[float]:
    try:
        if value is None or str(value).strip() == "":
            return None
        result = float(value)
        return result if math.isfinite(result) else None
    except Exception:
        return None


def _to_int(value: Any) -> Optional[int]:
    try:
        if value is None or str(value).strip() == "":
            return None
        return int(value)
    except Exception:
        return None


def _to_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y"}:
        return True
    if text in {"0", "false", "no", "n"}:
        return False
    return None


def _iter_csv(paths: Iterable[Path]) -> Iterator[Dict[str, str]]:
    for path in paths:
        with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as fh:
            yield from csv.DictReader(fh)


def _percentile(values: Sequence[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _stats(values: Sequence[float]) -> Dict[str, Any]:
    if not values:
        return {
            "sample_count": 0,
            "median_ms": None,
            "p90_ms": None,
            "p95_ms": None,
            "max_ms": None,
        }
    return {
        "sample_count": len(values),
        "median_ms": float(statistics.median(values)),
        "p90_ms": _percentile(values, 0.90),
        "p95_ms": _percentile(values, 0.95),
        "max_ms": float(max(values)),
    }


def _duration_ms(start: Optional[int], end: Optional[int]) -> Optional[float]:
    if start is None or end is None or end < start:
        return None
    return (end - start) / 1_000_000.0


def _overrun(value: Optional[float], threshold: float) -> Optional[float]:
    if value is None:
        return None
    return max(0.0, value - threshold)


def _miss(value: Optional[float], threshold: float) -> Optional[bool]:
    return None if value is None else value > threshold


def _format_csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, bool):
        return int(value)
    return value


def _simple_yaml_scalar(text: str) -> Any:
    value = text.strip().strip('"').strip("'")
    lower = value.lower()
    if lower in {"true", "yes"}:
        return True
    if lower in {"false", "no"}:
        return False
    if lower in {"null", "none", "~", ""}:
        return None
    number = _to_float(value)
    return number if number is not None else value


def _load_timing_yaml_without_dependency(text: str) -> Dict[str, Any]:
    """Parse the small timing config schema when PyYAML is unavailable."""
    timing: Dict[str, Any] = {}
    current_section: Optional[str] = None
    current_subsection: Optional[str] = None
    for raw in text.splitlines():
        content = raw.split("#", 1)[0].rstrip()
        if not content.strip():
            continue
        indent = len(content) - len(content.lstrip(" "))
        stripped = content.strip()
        if stripped == "timing:":
            continue
        if indent == 2 and stripped.endswith(":"):
            current_section = stripped[:-1]
            current_subsection = None
            timing.setdefault(current_section, {})
            continue
        if indent == 2 and ":" in stripped:
            key, value = stripped.split(":", 1)
            timing[key.strip()] = _simple_yaml_scalar(value)
            current_section = None
            current_subsection = None
            continue
        if indent == 4 and stripped.endswith(":") and current_section:
            current_subsection = stripped[:-1]
            section = timing.setdefault(current_section, {})
            if isinstance(section, dict):
                section.setdefault(current_subsection, [])
            continue
        if indent == 4 and ":" in stripped and current_section:
            key, value = stripped.split(":", 1)
            section = timing.setdefault(current_section, {})
            if isinstance(section, dict):
                section[key.strip()] = _simple_yaml_scalar(value)
            continue
        if indent >= 6 and stripped.startswith("- ") and current_section and current_subsection:
            section = timing.setdefault(current_section, {})
            if isinstance(section, dict):
                values = section.setdefault(current_subsection, [])
                if isinstance(values, list):
                    values.append(_simple_yaml_scalar(stripped[2:]))
    return timing


def load_threshold_config(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Timing threshold config not found: {path}")
    text = path.read_text(encoding="utf-8")
    loaded = yaml.safe_load(text) if yaml is not None else {"timing": _load_timing_yaml_without_dependency(text)}
    loaded = loaded or {}
    timing = loaded.get("timing") if isinstance(loaded, dict) else None
    if not isinstance(timing, dict):
        raise ValueError("Missing 'timing' section in threshold config")
    thresholds = timing.get("thresholds_ms")
    if not isinstance(thresholds, dict):
        raise ValueError("Missing timing.thresholds_ms section")
    return timing


def resolve_thresholds(timing_config: Dict[str, Any], overrides: Optional[Dict[str, Optional[float]]] = None) -> Dict[str, float]:
    configured = timing_config.get("thresholds_ms") or {}
    result: Dict[str, float] = {}
    for module in ("perception", "prediction", "planning", "e2e"):
        override = (overrides or {}).get(module)
        value = override if override is not None else _to_float(configured.get(module))
        if value is None or value <= 0:
            raise ValueError(f"A positive timing threshold is required for {module}")
        result[module] = float(value)
    return result


def load_functional_window(functional_result_path: Path, t1_override: Optional[float], t2_override: Optional[float]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    if functional_result_path.exists():
        result = json.loads(functional_result_path.read_text(encoding="utf-8"))
    t1 = t1_override
    t2 = t2_override
    if t1 is None:
        t1 = _to_float((result.get("perception") or {}).get("first_seen_time"))
    if t2 is None:
        t2 = _to_float(result.get("t2_collision_time"))
    if t1 is None or t2 is None:
        raise ValueError("Unable to resolve t1/t2; provide a functional result or explicit --t1/--t2")
    if t2 <= t1:
        raise ValueError(f"Invalid timing window: t1={t1}, t2={t2}")
    target_id = str(result.get("target_id") or "")
    chain = [str(value) for value in ((result.get("perception") or {}).get("id_chain") or [])]
    if target_id and target_id not in chain:
        chain.insert(0, target_id)
    return {
        "t1": float(t1),
        "t2": float(t2),
        "target_id": target_id or None,
        "target_id_chain": chain,
        "functional_verdict": result.get("final_verdict"),
        "functional_reason_code": result.get("reason_code"),
        "functional_result_path": str(functional_result_path),
    }


def _find_files(trace_dir: Path, category: str, pattern: str) -> List[Path]:
    directory = trace_dir / category
    return sorted(directory.glob(pattern)) if directory.exists() else []


def _load_fusion_frames(trace_dir: Path, t1: float, t2: float) -> List[Dict[str, Any]]:
    context_paths = _find_files(trace_dir, "message_context", "perception.multi_sensor_fusion.*.csv")
    if not context_paths:
        raise FileNotFoundError("Missing perception multi-sensor-fusion message_context trace")
    by_trace: Dict[str, Dict[str, Any]] = {}
    for row in _iter_csv(context_paths):
        if row.get("edge") != "out" or _to_bool(row.get("trace_valid")) is not True:
            continue
        trace_id = str(row.get("trace_id") or "").strip()
        data_ts_ns = _to_int(row.get("data_ts_ns"))
        if not trace_id or data_ts_ns is None or data_ts_ns <= 0:
            continue
        data_time = data_ts_ns / 1_000_000_000.0
        if not (t1 <= data_time <= t2):
            continue
        candidate = {
            "trace_id": trace_id,
            "parent_trace_id": str(row.get("primary_parent_trace_id") or "").strip(),
            "data_ts_ns": data_ts_ns,
            "data_time_sec": data_time,
            "input_seq": _to_int(row.get("input_seq")),
            "output_seq": _to_int(row.get("output_seq")),
            "object_count": _to_int(row.get("object_count")),
        }
        old = by_trace.get(trace_id)
        if old is None or data_ts_ns < old["data_ts_ns"]:
            by_trace[trace_id] = candidate
    return sorted(by_trace.values(), key=lambda row: (row["data_ts_ns"], row["trace_id"]))


def _load_phase_index(trace_dir: Path, frame_trace_ids: Set[str], parent_trace_ids: Set[str]) -> Dict[Tuple[str, str, str], List[int]]:
    index: Dict[Tuple[str, str, str], List[int]] = defaultdict(list)
    specifications = [
        ("perception_start", "perception.pointcloud_preprocess", parent_trace_ids, {"proc_enter"}),
        ("perception_end", "perception.multi_sensor_fusion", frame_trace_ids, {"output_pub"}),
        ("prediction", "prediction", frame_trace_ids, {"proc_enter", "output_pub"}),
        ("planning", "planning", frame_trace_ids, {"proc_enter", "output_pub"}),
    ]
    for label, expected_module, accepted_ids, phases in specifications:
        paths = _find_files(trace_dir, "events", MODULE_PATTERNS[label])
        if not paths:
            logging.warning("Missing trace event files for %s", label)
            continue
        for row in _iter_csv(paths):
            trace_id = str(row.get("trace_id") or "").strip()
            phase = str(row.get("phase") or "").strip()
            mono_ns = _to_int(row.get("mono_ns"))
            if trace_id not in accepted_ids or phase not in phases or mono_ns is None:
                continue
            module = str(row.get("module") or expected_module)
            if module != expected_module:
                continue
            index[(label, trace_id, phase)].append(mono_ns)
    return index


def _phase(index: Dict[Tuple[str, str, str], List[int]], label: str, trace_id: str, phase: str, choose: str) -> Optional[int]:
    values = index.get((label, trace_id, phase)) or []
    if not values:
        return None
    return min(values) if choose == "first" else max(values)


def _target_trace_ids(case_dir: Path, target_ids: Sequence[str]) -> Set[str]:
    ids = {str(value) for value in target_ids if str(value)}
    if not ids:
        return set()
    traces: Set[str] = set()
    pattern = re.compile(r"\btrace_id=([^\s]+).*?\bid=([^\s]+)")
    for path in sorted((case_dir / "log").glob("perception.log.INFO*")):
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if "[FUSION_OBS]" not in line:
                    continue
                match = pattern.search(line)
                if match and match.group(2) in ids:
                    traces.add(match.group(1))
    return traces


def calculate_frame_latencies(
    case_dir: Path,
    t1: float,
    t2: float,
    thresholds: Dict[str, float],
    target_ids: Sequence[str],
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    trace_dir = case_dir / "trace"
    if not trace_dir.exists():
        raise FileNotFoundError(f"Trace directory not found: {trace_dir}")
    frames = _load_fusion_frames(trace_dir, t1, t2)
    if not frames:
        raise ValueError("No valid fusion trace frames found inside t1-t2")
    frame_ids = {row["trace_id"] for row in frames}
    parent_ids = {row["parent_trace_id"] for row in frames if row["parent_trace_id"] and row["parent_trace_id"] != "0"}
    phases = _load_phase_index(trace_dir, frame_ids, parent_ids)
    target_traces = _target_trace_ids(case_dir, target_ids)
    output: List[Dict[str, Any]] = []
    for frame_index, frame in enumerate(frames, 1):
        trace_id = frame["trace_id"]
        parent_id = frame["parent_trace_id"]
        p_start = _phase(phases, "perception_start", parent_id, "proc_enter", "first")
        p_out = _phase(phases, "perception_end", trace_id, "output_pub", "last")
        d_start = _phase(phases, "prediction", trace_id, "proc_enter", "first")
        d_out = _phase(phases, "prediction", trace_id, "output_pub", "last")
        l_start = _phase(phases, "planning", trace_id, "proc_enter", "first")
        l_out = _phase(phases, "planning", trace_id, "output_pub", "last")
        latencies = {
            "perception": _duration_ms(p_start, p_out),
            "prediction": _duration_ms(d_start, d_out),
            "planning": _duration_ms(l_start, l_out),
            "e2e": _duration_ms(p_start, l_out),
        }
        misses = {module: _miss(latencies[module], thresholds[module]) for module in thresholds}
        e2e_abnormal = misses["e2e"] is True
        cause_modules: List[str] = []
        if e2e_abnormal:
            cause_modules = [module for module in ("perception", "prediction", "planning") if misses[module] is True]
            if not cause_modules:
                cause_modules = ["PIPELINE_ACCUMULATION_OR_HANDOFF"]
        complete = all(latencies[module] is not None for module in ("perception", "prediction", "planning", "e2e"))
        output.append({
            "frame_index": frame_index,
            "trace_id": trace_id,
            "parent_trace_id": parent_id,
            "data_ts_ns": frame["data_ts_ns"],
            "data_time_sec": frame["data_time_sec"],
            "target_present": trace_id in target_traces,
            "frame_status": "COMPLETE" if complete else "PARTIAL_TRACE",
            "perception_ms": latencies["perception"],
            "prediction_ms": latencies["prediction"],
            "planning_ms": latencies["planning"],
            "e2e_ms": latencies["e2e"],
            "perception_threshold_ms": thresholds["perception"],
            "prediction_threshold_ms": thresholds["prediction"],
            "planning_threshold_ms": thresholds["planning"],
            "e2e_threshold_ms": thresholds["e2e"],
            "e2e_deadline_miss": misses["e2e"],
            "perception_deadline_miss": misses["perception"],
            "prediction_deadline_miss": misses["prediction"],
            "planning_deadline_miss": misses["planning"],
            "cause_modules": ";".join(cause_modules),
            "e2e_overrun_ms": _overrun(latencies["e2e"], thresholds["e2e"]),
            "perception_overrun_ms": _overrun(latencies["perception"], thresholds["perception"]),
            "prediction_overrun_ms": _overrun(latencies["prediction"], thresholds["prediction"]),
            "planning_overrun_ms": _overrun(latencies["planning"], thresholds["planning"]),
            "perception_to_prediction_handoff_ms": _duration_ms(p_out, d_start),
            "prediction_to_planning_handoff_ms": _duration_ms(d_out, l_start),
            "perception_start_mono_ns": p_start,
            "perception_output_mono_ns": p_out,
            "prediction_start_mono_ns": d_start,
            "prediction_output_mono_ns": d_out,
            "planning_start_mono_ns": l_start,
            "planning_output_mono_ns": l_out,
            "input_seq": frame["input_seq"],
            "output_seq": frame["output_seq"],
            "object_count": frame["object_count"],
        })
    complete_count = sum(row["frame_status"] == "COMPLETE" for row in output)
    metadata = {
        "frame_count": len(output),
        "complete_frame_count": complete_count,
        "complete_frame_ratio": complete_count / len(output) if output else 0.0,
        "target_frame_count": sum(bool(row["target_present"]) for row in output),
    }
    return output, metadata


def _module_summary(rows: Sequence[Dict[str, Any]], module: str, threshold: float) -> Dict[str, Any]:
    key = f"{module}_ms"
    miss_key = f"{module}_deadline_miss"
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    summary = _stats(values)
    miss_rows = [row for row in rows if row.get(miss_key) is True]
    summary.update({
        "threshold_ms": threshold,
        "miss_count": len(miss_rows),
        "miss_ratio": len(miss_rows) / len(values) if values else None,
        "max_overrun_ms": max((_to_float(row.get(f"{module}_overrun_ms")) or 0.0) for row in miss_rows) if miss_rows else 0.0,
    })
    return summary


def _cause_modules_ms(row: Dict[str, Any]) -> Dict[str, Optional[float]]:
    values: Dict[str, Optional[float]] = {}
    for cause in str(row.get("cause_modules") or "").split(";"):
        if cause in {"perception", "prediction", "planning"}:
            values[cause] = _to_float(row.get(f"{cause}_ms"))
        elif cause == "PIPELINE_ACCUMULATION_OR_HANDOFF":
            handoffs = [
                _to_float(row.get("perception_to_prediction_handoff_ms")),
                _to_float(row.get("prediction_to_planning_handoff_ms")),
            ]
            usable = [value for value in handoffs if value is not None]
            values[cause] = sum(usable) if usable else None
    return values


def _cause_modules_overrun_ms(row: Dict[str, Any]) -> Dict[str, Optional[float]]:
    values: Dict[str, Optional[float]] = {}
    for cause in str(row.get("cause_modules") or "").split(";"):
        if cause in {"perception", "prediction", "planning"}:
            values[cause] = _to_float(row.get(f"{cause}_overrun_ms"))
        elif cause == "PIPELINE_ACCUMULATION_OR_HANDOFF":
            values[cause] = None
    return values


def build_result(
    case_dir: Path,
    window: Dict[str, Any],
    thresholds: Dict[str, float],
    timing_config: Dict[str, Any],
    rows: Sequence[Dict[str, Any]],
    metadata: Dict[str, Any],
) -> Dict[str, Any]:
    anomaly_rows = [row for row in rows if row.get("e2e_deadline_miss") is True]
    min_ratio = float(timing_config.get("min_complete_frame_ratio", 0.95))
    sufficient = metadata["complete_frame_ratio"] >= min_ratio
    causes = Counter()
    for row in anomaly_rows:
        for cause in str(row.get("cause_modules") or "").split(";"):
            if cause:
                causes[cause] += 1
    return {
        "case_id": case_dir.name,
        "status": "PASS" if sufficient and not anomaly_rows else "FAIL" if sufficient else "UNKNOWN_DATA_INSUFFICIENT",
        "explicit_deadline_threshold_miss": bool(anomaly_rows) if sufficient else None,
        # Backward-compatible legacy field. Physical implicit deadlines are
        # computed by implicit_deadline_analyzer.py, not by this detector.
        "implicit_deadline_miss": bool(anomaly_rows) if sufficient else None,
        "window": {
            "t1": window["t1"],
            "t2": window["t2"],
            "duration_sec": window["t2"] - window["t1"],
            "source": window["functional_result_path"],
        },
        "functional_context": {
            "verdict": window["functional_verdict"],
            "reason_code": window["functional_reason_code"],
            "target_id": window["target_id"],
            "target_id_chain": window["target_id_chain"],
        },
        "thresholds_ms": thresholds,
        "trace_quality": {
            **metadata,
            "required_complete_frame_ratio": min_ratio,
            "sufficient": sufficient,
        },
        "modules": {
            module: _module_summary(rows, module, thresholds[module])
            for module in ("perception", "prediction", "planning", "e2e")
        },
        "e2e_anomaly_frame_count": len(anomaly_rows),
        "e2e_anomaly_target_frame_count": sum(bool(row.get("target_present")) for row in anomaly_rows),
        "cause_module_frame_counts": dict(causes),
        "worst_e2e_frames": [
            {
                "trace_id": row["trace_id"],
                "data_ts_ns": row["data_ts_ns"],
                "e2e_ms": row["e2e_ms"],
                "e2e_over_threshold_ms": row["e2e_overrun_ms"],
                "cause_modules": row["cause_modules"],
                "cause_modules_ms": _cause_modules_ms(row),
                "cause_modules_overrun_ms": _cause_modules_overrun_ms(row),
                "target_present": row["target_present"],
            }
            for row in sorted(anomaly_rows, key=lambda item: float(item["e2e_ms"] or 0.0), reverse=True)[:20]
        ],
        "output_files": {
            "all_frames": "timing_frame_latencies.csv",
            "anomaly_frames": "timing_anomaly_frames.csv",
            "summary": "timing_analysis_result.json",
            "e2e_scatter": "timing_e2e_scatter.svg",
        },
    }


def write_e2e_scatter(path: Path, rows: Sequence[Dict[str, Any]], threshold_ms: float, case_id: str) -> None:
    valid = [row for row in rows if _to_float(row.get("e2e_ms")) is not None]
    width, height = 1200, 620
    left, right, top, bottom = 90, 35, 55, 75
    plot_width = width - left - right
    plot_height = height - top - bottom
    values = [float(row["e2e_ms"]) for row in valid]
    y_max = max(values + [threshold_ms, 1.0]) * 1.10
    max_frame = max([int(row.get("frame_index") or 0) for row in valid] + [1])

    def x_pos(frame_index: int) -> float:
        return left + ((frame_index - 1) / max(max_frame - 1, 1)) * plot_width

    def y_pos(value: float) -> float:
        return top + plot_height - (value / y_max) * plot_height

    svg: List[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="30" text-anchor="middle" font-family="sans-serif" font-size="20">{escape(case_id)} t1-t2 E2E latency scatter</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_height}" stroke="#333" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top+plot_height}" x2="{left+plot_width}" y2="{top+plot_height}" stroke="#333" stroke-width="1"/>',
    ]
    for tick in range(6):
        value = y_max * tick / 5.0
        y = y_pos(value)
        svg.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left+plot_width}" y2="{y:.2f}" stroke="#e6e6e6" stroke-width="1"/>')
        svg.append(f'<text x="{left-10}" y="{y+4:.2f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:.1f}</text>')
    for tick in range(6):
        frame = 1 + round((max_frame - 1) * tick / 5.0)
        x = x_pos(frame)
        svg.append(f'<line x1="{x:.2f}" y1="{top+plot_height}" x2="{x:.2f}" y2="{top+plot_height+5}" stroke="#333"/>')
        svg.append(f'<text x="{x:.2f}" y="{top+plot_height+22}" text-anchor="middle" font-family="sans-serif" font-size="12">{frame}</text>')
    threshold_y = y_pos(threshold_ms)
    svg.append(f'<line x1="{left}" y1="{threshold_y:.2f}" x2="{left+plot_width}" y2="{threshold_y:.2f}" stroke="#d62728" stroke-width="1.5" stroke-dasharray="8,5"/>')
    svg.append(f'<text x="{left+plot_width-4}" y="{threshold_y-7:.2f}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#d62728">threshold {threshold_ms:.3f} ms</text>')
    for row in valid:
        x = x_pos(int(row["frame_index"]))
        y = y_pos(float(row["e2e_ms"]))
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.7" fill="#1f77b4" opacity="0.72"><title>frame={row["frame_index"]}, trace={escape(str(row["trace_id"]))}, E2E={float(row["e2e_ms"]):.3f} ms</title></circle>')
        if row.get("e2e_deadline_miss") is True:
            svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="6.0" fill="none" stroke="#d62728" stroke-width="2.2"><title>deadline miss: +{float(row["e2e_overrun_ms"]):.3f} ms, cause={escape(str(row.get("cause_modules") or ""))}</title></circle>')
    svg.extend([
        f'<text x="{left+plot_width/2:.2f}" y="{height-20}" text-anchor="middle" font-family="sans-serif" font-size="14">Frame index inside t1-t2</text>',
        f'<text x="22" y="{top+plot_height/2:.2f}" text-anchor="middle" font-family="sans-serif" font-size="14" transform="rotate(-90 22 {top+plot_height/2:.2f})">E2E latency (ms)</text>',
        f'<circle cx="{left+15}" cy="{top+18}" r="3" fill="#1f77b4"/><text x="{left+25}" y="{top+22}" font-family="sans-serif" font-size="12">frame</text>',
        f'<circle cx="{left+85}" cy="{top+18}" r="6" fill="none" stroke="#d62728" stroke-width="2"/><text x="{left+97}" y="{top+22}" font-family="sans-serif" font-size="12">E2E deadline miss</text>',
        '</svg>',
    ])
    path.write_text("\n".join(svg), encoding="utf-8")


def write_outputs(out_dir: Path, rows: Sequence[Dict[str, Any]], result: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    all_path = out_dir / "timing_frame_latencies.csv"
    anomaly_path = out_dir / "timing_anomaly_frames.csv"
    for path, selected in ((all_path, rows), (anomaly_path, [row for row in rows if row.get("e2e_deadline_miss") is True])):
        with path.open("w", encoding="utf-8-sig", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=FRAME_FIELDS, extrasaction="ignore")
            writer.writeheader()
            for row in selected:
                writer.writerow({key: _format_csv_value(row.get(key)) for key in FRAME_FIELDS})
    (out_dir / "timing_analysis_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_e2e_scatter(
        out_dir / "timing_e2e_scatter.svg",
        rows,
        float(result["thresholds_ms"]["e2e"]),
        str(result.get("case_id") or "case"),
    )


def run_timing_analysis(
    case_dir: Path,
    out_dir: Path,
    functional_result_path: Path,
    threshold_config_path: Path,
    threshold_overrides: Optional[Dict[str, Optional[float]]] = None,
    t1_override: Optional[float] = None,
    t2_override: Optional[float] = None,
) -> Dict[str, Any]:
    case_dir = case_dir.resolve()
    out_dir = out_dir.resolve()
    timing_config = load_threshold_config(threshold_config_path)
    thresholds = resolve_thresholds(timing_config, threshold_overrides)
    window = load_functional_window(functional_result_path, t1_override, t2_override)
    rows, metadata = calculate_frame_latencies(
        case_dir,
        window["t1"],
        window["t2"],
        thresholds,
        window["target_id_chain"],
    )
    result = build_result(case_dir, window, thresholds, timing_config, rows, metadata)
    write_outputs(out_dir, rows, result)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect per-frame configured-threshold overruns inside t1-t2")
    parser.add_argument("--case-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--functional-result", type=Path, help="classification_result.json; defaults to OUT_DIR/classification_result.json")
    parser.add_argument(
        "--threshold-config",
        type=Path,
        default=Path(__file__).with_name("timing_threshold_config.yaml"),
    )
    parser.add_argument("--t1", type=float)
    parser.add_argument("--t2", type=float)
    parser.add_argument("--perception-threshold-ms", type=float)
    parser.add_argument("--prediction-threshold-ms", type=float)
    parser.add_argument("--planning-threshold-ms", type=float)
    parser.add_argument("--e2e-threshold-ms", type=float)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")
    functional_result = args.functional_result or (args.out_dir / "classification_result.json")
    overrides = {
        "perception": args.perception_threshold_ms,
        "prediction": args.prediction_threshold_ms,
        "planning": args.planning_threshold_ms,
        "e2e": args.e2e_threshold_ms,
    }
    try:
        result = run_timing_analysis(
            args.case_dir,
            args.out_dir,
            functional_result,
            args.threshold_config,
            overrides,
            args.t1,
            args.t2,
        )
    except Exception as exc:
        logging.error("Timing analysis failed: %s", exc)
        return 2
    logging.info(
        "Timing status=%s explicit_threshold_miss=%s anomaly_frames=%s",
        result["status"],
        result["explicit_deadline_threshold_miss"],
        result["e2e_anomaly_frame_count"],
    )
    logging.info("Timing outputs written to %s", args.out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
