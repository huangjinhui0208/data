#!/usr/bin/env python3
"""Summarize baseline/e2e200/e2e300 SCB experiment groups."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional

from collision_case_classifier import json_safe, to_float
from implicit_deadline_analyzer import load_experiment_metadata, percentile


GROUP_ORDER = ["baseline", "e2e200", "e2e300"]


def _find_result(case_dir: Path) -> Optional[Path]:
    preferred = [
        case_dir / "analysis" / "implicit_deadline_result.json",
        case_dir / "scb_analysis" / "implicit_deadline_result.json",
        case_dir / "implicit_deadline_result.json",
    ]
    for path in preferred:
        if path.exists():
            return path
    found = sorted(case_dir.rglob("implicit_deadline_result.json"))
    return found[0] if found else None


def _median(values: List[Optional[float]]) -> Optional[float]:
    usable = [float(value) for value in values if value is not None]
    return statistics.median(usable) if usable else None


def summarize(cases_root: Path, out_dir: Path) -> Dict[str, Any]:
    cases_root = cases_root.resolve()
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    for case_dir in sorted(path for path in cases_root.iterdir() if path.is_dir()):
        metadata = load_experiment_metadata(case_dir)
        group = str(metadata.get("group") or "")
        if group not in GROUP_ORDER:
            continue
        result_path = _find_result(case_dir)
        result = json.loads(result_path.read_text(encoding="utf-8")) if result_path else {}
        stable = result.get("stable_perception") or {}
        brake = result.get("brake_application") or {}
        physical = result.get("physical_response") or {}
        runs.append({
            "case_id": case_dir.name,
            "group": group,
            "repeat_index": metadata.get("repeat_index"),
            "analysis_available": bool(result),
            "analysis_status": result.get("status"),
            "functionally_normal": (result.get("functional_context") or {}).get("functionally_normal"),
            "collision": (result.get("collision") or {}).get("occurred"),
            "speed_kmh": stable.get("ego_speed_kmh"),
            "d1_m": stable.get("d1_m"),
            "configured_delay_ms": metadata.get("configured_bridge_delay_ms"),
            "requested_delay_ms": brake.get("requested_delay_ms"),
            "actual_delay_ms": brake.get("actual_delay_ms"),
            "actual_sim_delay_ms": brake.get("actual_sim_delay_ms"),
            "physical_response_ms": physical.get("stable_observation_to_brake_apply_ms"),
            "safety_deadline_slack_ms": physical.get("safety_deadline_slack_ms"),
            "collision_deadline_slack_ms": physical.get("collision_deadline_slack_ms"),
            "safety_deadline_miss": result.get("implicit_safety_deadline_miss"),
            "collision_deadline_miss": result.get("collision_deadline_miss"),
            "causal_assessment": result.get("causal_assessment"),
            "result_file": str(result_path) if result_path else None,
        })

    groups: Dict[str, Any] = {}
    for group in GROUP_ORDER:
        selected = [row for row in runs if row["group"] == group]
        analyzed = [row for row in selected if row["analysis_status"] == "ANALYZED"]
        response = [to_float(row["physical_response_ms"]) for row in analyzed]
        response = [value for value in response if value is not None]
        actual_delay = [to_float(row["actual_delay_ms"]) for row in analyzed]
        actual_delay = [value for value in actual_delay if value is not None]
        collision_known = [
            row for row in selected if row["collision"] is not None
        ]
        collisions = [bool(row["collision"]) for row in collision_known]
        function_values = [
            row["functionally_normal"]
            for row in selected
            if row["functionally_normal"] is not None
        ]
        delay_consistent = [
            row for row in analyzed
            if to_float(row["configured_delay_ms"]) is not None
            and to_float(row["requested_delay_ms"]) is not None
            and abs(
                float(row["configured_delay_ms"])
                - float(row["requested_delay_ms"])
            ) <= 1.0
        ]
        group_ready = (
            len(selected) >= 10
            and len(analyzed) >= 8
            and len(collision_known) == len(selected)
            and len(delay_consistent) == len(analyzed)
        )
        groups[group] = {
            "run_count": len(selected),
            "analyzable_run_count": len(analyzed),
            "analysis_success_ratio": len(analyzed) / len(selected) if selected else None,
            "collision_known_run_count": len(collision_known),
            "ready_for_comparison": group_ready,
            "functionally_normal_count": sum(value is True for value in function_values),
            "functionally_normal_ratio": sum(value is True for value in function_values) / len(function_values) if function_values else None,
            "collision_count": sum(collisions),
            "collision_rate": sum(collisions) / len(collisions) if collisions else None,
            "safety_deadline_miss_count": sum(row["safety_deadline_miss"] is True for row in analyzed),
            "collision_deadline_miss_count": sum(row["collision_deadline_miss"] is True for row in analyzed),
            "physical_response_median_ms": _median(response),
            "physical_response_p90_ms": percentile(response, 0.90),
            "actual_injection_median_ms": _median(actual_delay),
            "configured_delay_matches_evidence_count": len(delay_consistent),
            "speed_median_kmh": _median([to_float(row["speed_kmh"]) for row in analyzed]),
            "d1_median_m": _median([to_float(row["d1_m"]) for row in analyzed]),
        }

    baseline = groups["baseline"]
    g200, g300 = groups["e2e200"], groups["e2e300"]
    response_ordered = all(
        value is not None
        for value in [baseline["physical_response_median_ms"], g200["physical_response_median_ms"], g300["physical_response_median_ms"]]
    ) and baseline["physical_response_median_ms"] < g200["physical_response_median_ms"] < g300["physical_response_median_ms"]
    collision_non_decreasing = all(
        value is not None
        for value in [baseline["collision_rate"], g200["collision_rate"], g300["collision_rate"]]
    ) and baseline["collision_rate"] <= g200["collision_rate"] <= g300["collision_rate"]
    matched_conditions = None
    d1s = [groups[group]["d1_median_m"] for group in GROUP_ORDER]
    speeds = [groups[group]["speed_median_kmh"] for group in GROUP_ORDER]
    if all(value is not None for value in d1s + speeds):
        matched_conditions = max(d1s) - min(d1s) <= 2.0 and max(speeds) - min(speeds) <= 2.0
    all_groups_ready = all(groups[group]["ready_for_comparison"] for group in GROUP_ORDER)
    strong_candidate = (
        all_groups_ready
        and baseline["collision_rate"] == 0.0
        and response_ordered and collision_non_decreasing and matched_conditions is True
        and (g200["collision_deadline_miss_count"] + g300["collision_deadline_miss_count"]) > 0
    )
    result = {
        "status": (
            "PASS"
            if all_groups_ready
            else (
                "INSUFFICIENT_ANALYZABLE_RUNS"
                if all(groups[group]["run_count"] >= 10 for group in GROUP_ORDER)
                else "INCOMPLETE_GROUPS"
            )
        ),
        "groups": groups,
        "causal_checks": {
            "at_least_10_runs_each_group": all(groups[group]["run_count"] >= 10 for group in GROUP_ORDER),
            "at_least_8_analyzable_runs_each_group": all(
                groups[group]["analyzable_run_count"] >= 8
                for group in GROUP_ORDER
            ),
            "collision_outcome_known_for_every_run": all(
                groups[group]["collision_known_run_count"] == groups[group]["run_count"]
                for group in GROUP_ORDER
            ),
            "configured_delay_matches_bridge_evidence": all(
                groups[group]["configured_delay_matches_evidence_count"]
                == groups[group]["analyzable_run_count"]
                for group in GROUP_ORDER
            ),
            "baseline_zero_collision": baseline["collision_rate"] == 0.0 if baseline["collision_rate"] is not None else None,
            "physical_response_increases_with_dose": response_ordered,
            "collision_rate_non_decreasing_with_dose": collision_non_decreasing,
            "speed_and_d1_matched_across_groups": matched_conditions,
            "collision_deadline_miss_observed": (g200["collision_deadline_miss_count"] + g300["collision_deadline_miss_count"]) > 0,
        },
        "conclusion": (
            "STRONG_SYSTEM_LEVEL_DELAY_CAUSAL_CANDIDATE"
            if strong_candidate
            else "CAUSALITY_NOT_YET_ESTABLISHED"
        ),
        "scope_note": "Bridge injection demonstrates system-level latency sensitivity; it does not by itself prove an endogenous Apollo module slowdown.",
        "runs": runs,
    }
    fields = list(runs[0]) if runs else ["case_id", "group"]
    with (out_dir / "scb_all_runs.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(runs)
    (out_dir / "scb_group_summary.json").write_text(
        json.dumps(json_safe(result), ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize SCB experiment groups")
    parser.add_argument("--cases-root", type=Path, default=Path(r"D:\data\scb_data"))
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.cases_root, args.out_dir)
    print(json.dumps({"status": result["status"], "conclusion": result["conclusion"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
