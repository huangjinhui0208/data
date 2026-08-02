#!/usr/bin/env python3
"""Unified entry point for the four-group realtime collision analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
LOCAL_PACKAGES = ROOT / ".python_packages"
if LOCAL_PACKAGES.exists():
    sys.path.insert(0, str(LOCAL_PACKAGES))
sys.path.insert(0, str(ROOT / "src"))

from realtime_collision_core import run_analysis  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze Apollo + CARLA realtime collision experiment runs"
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("D:/data"),
        help="Parent directory containing baseline/100ms/300ms/400ms",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT,
        help="Analysis output directory",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "analysis_config.yaml",
    )
    parser.add_argument(
        "--skip-hashes",
        action="store_true",
        help="Skip SHA-256 hashing during fast development runs",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return run_analysis(
        input_root=args.input.resolve(),
        output_root=args.output.resolve(),
        config_path=args.config.resolve(),
        compute_hashes=not args.skip_hashes,
    )


if __name__ == "__main__":
    raise SystemExit(main())
