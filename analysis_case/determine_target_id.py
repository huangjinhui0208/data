#!/usr/bin/env python3
"""Determine the Apollo target id for a CARLA collision case.

The script is read-only: it prints the target identity result as JSON and does
not create classifier output files.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from collision_case_classifier import (
    CarlaHistoryTargetMatcher,
    CaseDataLoader,
    CollisionTimeResolver,
    Config,
    LogAndTableParser,
    TargetResolver,
    json_safe,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resolve Apollo target_id from CARLA collision history")
    parser.add_argument("--case-dir", required=True, type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("collision_classifier_config.yaml"),
    )
    parser.add_argument("--target-id", type=str, help="Optional manual override for comparison/debugging")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    case_dir = args.case_dir.resolve()
    config = Config.load(args.config)
    # This path only tells CaseDataLoader what to exclude; it is never created.
    loader = CaseDataLoader(case_dir, case_dir / ".target_id_no_output")
    t2, t2_source, missing = CollisionTimeResolver(loader).resolve(None)
    if t2 is None:
        print(json.dumps({
            "resolved": False,
            "target_id": None,
            "physical_target_id_chain": [],
            "reason": "MISSING_COLLISION_TIME",
            "missing": missing,
        }, ensure_ascii=False, indent=2))
        return 2

    artifacts = LogAndTableParser(loader, config, t2).parse()
    carla_match = CarlaHistoryTargetMatcher(loader, artifacts, config, t2).resolve()
    target_debug = TargetResolver(artifacts, config, t2, carla_match).resolve(args.target_id)
    selected = target_debug.get("selected_target_id")
    result = {
        "resolved": bool(selected),
        "target_id": selected,
        "physical_target_id_chain": target_debug.get("physical_target_id_chain") or ([selected] if selected else []),
        "target_source": target_debug.get("target_source"),
        "confidence": target_debug.get("confidence"),
        "collision_time": t2,
        "collision_time_source": t2_source,
        "method": {
            "ground_truth": "CARLA collision other_actor history",
            "time_alignment": "linear interpolation at Apollo FUSION_OBS obs_time",
            "coordinate_transform": "x=x_carla, y=-y_carla, vx=vx_carla, vy=-vy_carla, heading=-yaw*pi/180",
            "identity_features": ["multi-frame position", "velocity", "heading when moving", "actor type"],
            "planning_role": "auxiliary score only; never a required condition or hard identity source",
        },
        "carla_history_match": carla_match,
        "warnings": target_debug.get("warnings", []),
    }
    print(json.dumps(json_safe(result), ensure_ascii=False, indent=2))
    return 0 if selected else 2


if __name__ == "__main__":
    raise SystemExit(main())
