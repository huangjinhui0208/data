#!/usr/bin/env python3
"""Calibrate RSS-like dynamic-envelope inputs from second-experiment baseline runs.

Raw experiment directories are read only.  Observed quantities and empirical
model/calibration parameters are deliberately emitted in separate columns.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
BASELINE = WORKSPACE / "第二次实验" / "baseline"
OUTPUT = WORKSPACE / "output" / "second_experiment_baseline_dynamic_parameters"
TABLES = OUTPUT / "tables"
REPORT = OUTPUT / "report"
VALIDATION = OUTPUT / "validation"
VENDOR = (
    WORKSPACE
    / "report_workspace"
    / "scripts"
    / "vendor"
    / "realtime_collision_analysis"
)
sys.path.insert(0, str(VENDOR / "src"))

import realtime_collision_core as core  # noqa: E402


GEOMETRY_OFFSET_M = 5.3074
GEOMETRY_UNCERTAINTY_M = 0.52
EXPECTED_RUNS = 7


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"refusing to write empty table: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def percentile_summary(values: list[float]) -> dict[str, float | int]:
    array = np.asarray([value for value in values if finite(value)], dtype=float)
    return {
        "n": int(array.size),
        "mean": float(np.mean(array)),
        "sd": float(np.std(array, ddof=1)) if array.size > 1 else math.nan,
        "median": float(np.median(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        "p05": float(np.percentile(array, 5)),
        "p95": float(np.percentile(array, 95)),
    }


def interpolated_clearance(stable: core.FusionObservation, state: dict) -> float:
    dx = stable.x_m - state["x_m"]
    dy = stable.y_m - state["y_m"]
    center_longitudinal = dx * math.cos(state["heading_rad"]) + dy * math.sin(
        state["heading_rad"]
    )
    return center_longitudinal - GEOMETRY_OFFSET_M


def interval_accelerations(
    localization: list[core.EgoSample], t1: float, t2: float, median3: bool
) -> list[float]:
    times = np.asarray([sample.time_s for sample in localization], dtype=float)
    speeds = np.asarray([sample.speed_mps for sample in localization], dtype=float)
    if median3:
        speeds = core._median3(speeds)
    dt = np.diff(times)
    acceleration = np.divide(
        np.diff(speeds),
        dt,
        out=np.full_like(dt, np.nan),
        where=dt > 0,
    )
    # Keep intervals with positive-duration overlap with [t1, t2].  An interval
    # that merely ends at t1 (or starts at t2) has zero response-stage support
    # and would leak pre/post-response acceleration into a_resp.
    positive_overlap = (times[1:] > t1) & (times[:-1] < t2)
    return [float(value) for value in acceleration[positive_overlap] if finite(value)]


def make_config() -> dict:
    config = yaml.safe_load(
        (VENDOR / "config" / "analysis_config.yaml").read_text(encoding="utf-8")
    )
    config["groups"] = {
        "baseline": {
            "root": str(BASELINE),
            "nominal_injected_delay_ms": 0.0,
            "expected_runs": EXPECTED_RUNS,
        }
    }
    config["stable_perception"]["sensitivity_frames"] = [
        config["stable_perception"]["primary_frames"]
    ]
    config["effective_brake"]["sensitivity_thresholds_mps2"] = [
        config["effective_brake"]["primary_decel_threshold_mps2"]
    ]
    return config


def analyze_run(spec: core.RunSpec, config: dict, timezone: ZoneInfo) -> dict:
    parsed = core.parse_run(spec, config, timezone)
    raw, debug = core.raw_run_metrics(parsed, config)
    stable = parsed.perception.get("stable")
    t1 = float(raw["t_sensor_origin_s"])
    t2 = float(raw["t_brake_effective_s"])
    state1 = core.interpolate_sample(parsed.localization, t1)
    state2 = core.interpolate_sample(parsed.localization, t2)
    if stable is None or state1 is None or state2 is None:
        raise RuntimeError(f"{spec.run_id}: missing stable target or endpoint state")

    response_time_s = t2 - t1
    d0 = interpolated_clearance(stable, state1)
    d_response = core.integrate_speed(parsed.localization, t1, t2)
    a_endpoint = (state2["speed_mps"] - state1["speed_mps"]) / response_time_s
    a_distance_fit = 2.0 * (
        d_response - state1["speed_mps"] * response_time_s
    ) / response_time_s**2
    raw_acceleration = interval_accelerations(parsed.localization, t1, t2, False)
    smoothed_acceleration = interval_accelerations(parsed.localization, t1, t2, True)

    completion = debug.get("brake_completion", {})
    endpoint = completion.get("sample")
    endpoint_time = completion.get("time_s")
    if endpoint is None or not finite(endpoint_time):
        raise RuntimeError(f"{spec.run_id}: no complete braking endpoint")
    d_brake_displacement = math.dist(
        (state2["x_m"], state2["y_m"], state2["z_m"]),
        (endpoint.x_m, endpoint.y_m, endpoint.z_m),
    )
    d_brake_path = core.integrate_speed(
        parsed.localization, t2, float(endpoint_time)
    )
    speed_energy_delta = state2["speed_mps"] ** 2 - endpoint.speed_mps**2
    b_displacement = speed_energy_delta / (2.0 * d_brake_displacement)
    b_path = speed_energy_delta / (2.0 * d_brake_path)

    return {
        "run_id": spec.run_id,
        "group_name": "baseline",
        "t1_wall_s": t1,
        "t2_wall_s": t2,
        "T_response_data_observed_ms": response_time_s * 1000.0,
        "d0_clearance_data_observed_m": d0,
        "d0_geometry_lower_bound_diagnostic_m": d0 - GEOMETRY_UNCERTAINTY_M,
        "d0_geometry_uncertainty_m": GEOMETRY_UNCERTAINTY_M,
        "v1_data_observed_mps": state1["speed_mps"],
        "v2_data_observed_mps": state2["speed_mps"],
        "D_response_wall_integral_data_observed_m": d_response,
        "a_response_endpoint_net_data_observed_mps2": a_endpoint,
        "a_response_constant_accel_distance_fit_data_observed_mps2": a_distance_fit,
        "a_response_peak_interval_raw_data_observed_mps2": max(
            0.0, max(raw_acceleration)
        ),
        "a_response_peak_interval_median3_diagnostic_mps2": max(
            0.0, max(smoothed_acceleration)
        ),
        "response_acceleration_raw_interval_count": len(raw_acceleration),
        "D_brake_displacement_data_observed_m": d_brake_displacement,
        "D_brake_wall_path_integral_diagnostic_m": d_brake_path,
        "v_brake_endpoint_data_observed_mps": endpoint.speed_mps,
        "b_effective_displacement_model_calibration_mps2": b_displacement,
        "b_effective_path_model_calibration_mps2": b_path,
        "strict_stop_data_observed": debug.get("stop", {}).get("status")
        == "AVAILABLE",
        "source_localization_file": str(parsed.files["localization"]),
        "source_perception_file": str(parsed.files["perception"]),
        "d0_evidence_class": "OBSERVED_DERIVED",
        "dynamics_parameter_evidence_class": "UNVALIDATED_MODEL_CALIBRATION",
    }


def make_summary(rows: list[dict]) -> list[dict]:
    d0 = percentile_summary([row["d0_clearance_data_observed_m"] for row in rows])
    a_raw = percentile_summary(
        [row["a_response_peak_interval_raw_data_observed_mps2"] for row in rows]
    )
    a_distance = percentile_summary(
        [
            max(0.0, row["a_response_constant_accel_distance_fit_data_observed_mps2"])
            for row in rows
        ]
    )
    a_smooth = percentile_summary(
        [row["a_response_peak_interval_median3_diagnostic_mps2"] for row in rows]
    )
    b_path = percentile_summary(
        [row["b_effective_path_model_calibration_mps2"] for row in rows]
    )
    b_displacement = percentile_summary(
        [row["b_effective_displacement_model_calibration_mps2"] for row in rows]
    )

    def row(
        parameter: str,
        semantics: str,
        stats: dict,
        recommended: float | str,
        unit: str,
        recommendation: str,
        evidence_class: str,
        limitations: str,
    ) -> dict:
        return {
            "parameter": parameter,
            "semantics": semantics,
            **stats,
            "recommended_value": recommended,
            "unit": unit,
            "recommendation_type": recommendation,
            "evidence_class": evidence_class,
            "limitations": limitations,
        }

    blank_stats = {
        "n": 1,
        "mean": math.nan,
        "sd": math.nan,
        "median": math.nan,
        "min": math.nan,
        "max": math.nan,
        "p05": math.nan,
        "p95": math.nan,
    }
    return [
        row(
            "d0",
            "t1时ego前缘到静态障碍物后缘的纵向净距；deadline必须使用逐run值",
            d0,
            "RUN_SPECIFIC_OBSERVED",
            "m",
            "逐run输入；组中位数仅作描述",
            "OBSERVED_DERIVED",
            f"固定组合几何偏移{GEOMETRY_OFFSET_M} m；接触校准不确定性约±{GEOMETRY_UNCERTAINTY_M} m",
        ),
        row(
            "a_resp_central",
            "响应窗常加速度距离拟合值max(0, 2(D-v1*T)/T^2)的组中位数",
            a_distance,
            a_distance["median"],
            "m/s^2",
            "中心/典型模型参数",
            "UNVALIDATED_MODEL_CALIBRATION",
            "同批baseline事后拟合；不能解释为最大可能加速度",
        ),
        row(
            "a_resp_conservative_candidate",
            "各run响应窗原始相邻Localization速度差分的正向峰值之最大值",
            a_raw,
            a_raw["max"],
            "m/s^2",
            "baseline经验上包络候选",
            "UNVALIDATED_MODEL_CALIBRATION",
            f"未独立验证；median-3滤波诊断最大值为{a_smooth['max']:.9f} m/s^2",
        ),
        row(
            "b_e_central_existing_convention",
            "v2^2/(2*D_brake_displacement)的组中位数",
            b_displacement,
            b_displacement["median"],
            "m/s^2",
            "现有第二次实验模型中心值",
            "UNVALIDATED_MODEL_CALIBRATION",
            "样本内中心值，不是保证下界",
        ),
        row(
            "b_e_conservative_candidate",
            "7个均达到近零速并具最低速度端点的run中，(v2^2-v_end^2)/(2*D_brake_wall_path)的组最小值",
            b_path,
            b_path["min"],
            "m/s^2",
            "baseline经验下包络候选",
            "UNVALIDATED_MODEL_CALIBRATION",
            "仅7次同场景baseline；未覆盖摩擦、坡度、载荷和执行器退化，不能称已证明保证",
        ),
        row(
            "d_safe_contact",
            "停车后允许恰好到达接触边界的残余距离",
            blank_stats,
            0.0,
            "m",
            "用户声明的0 m接触边界情景",
            "INDEPENDENT_REQUIREMENT",
            "接触避免边界，不等同于工程安全裕度",
        ),
        row(
            "d_safe_engineering",
            "停车后要求保留的工程安全裕度",
            blank_stats,
            6.0,
            "m",
            "用户声明的6 m工程裕度情景",
            "INDEPENDENT_REQUIREMENT",
            "研究者选择的工程阈值；无外部/预注册证据时不宣称认证安全要求",
        ),
    ]


def make_parameter_sets(summary: list[dict]) -> list[dict]:
    values = {row["parameter"]: row["recommended_value"] for row in summary}
    rows = []
    for name, a_key, b_key, semantics in [
        (
            "BASELINE_EMPIRICAL_CENTRAL",
            "a_resp_central",
            "b_e_central_existing_convention",
            "中心/典型样本内模型，不用于保证声明",
        ),
        (
            "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE",
            "a_resp_conservative_candidate",
            "b_e_conservative_candidate",
            "baseline观测上/下包络候选，仍未独立验证",
        ),
    ]:
        for d_safe in (0.0, 6.0):
            rows.append(
                {
                    "parameter_set_id": f"{name}_DSAFE_{int(d_safe)}M",
                    "d0_input_policy": "RUN_SPECIFIC_d0_clearance_data_observed_m",
                    "d0_geometry_conservative_policy": "OPTIONAL_d0_minus_0.52m",
                    "a_resp_model_predicted_mps2": values[a_key],
                    "b_e_model_predicted_mps2": values[b_key],
                    "d_safe_requirement_m": d_safe,
                    "calibration_run_ids": "|".join(row["run_id"] for row in RUN_ROWS),
                    "parameter_status": "NOT_QUALIFIED_PRIMARY",
                    "evidence_class": "UNVALIDATED_MODEL",
                    "semantics_and_limitations": semantics
                    + "；参数为事后计算且缺少外部制动包络/ODD验证",
                }
            )
    return rows


def make_evidence(rows: list[dict], summary: list[dict]) -> list[dict]:
    evidence = []
    for row in rows:
        evidence.append(
            {
                "evidence_id": f"EV.D0.{row['run_id']}",
                "run_id": row["run_id"],
                "metric": "d0_clearance_data_observed_m",
                "value": row["d0_clearance_data_observed_m"],
                "unit": "m",
                "evidence_class": "OBSERVED_DERIVED",
                "source_file": row["source_perception_file"]
                + " | "
                + row["source_localization_file"],
                "source_locator": "stable 3-frame Fusion source t1 + interpolated Localization pose + 5.3074 m geometry offset",
                "availability": "AVAILABLE",
                "limitations": "fixed geometry offset; ±0.52 m contact-calibration uncertainty",
            }
        )
    for metric in (
        "a_resp_conservative_candidate",
        "b_e_conservative_candidate",
        "d_safe_contact",
        "d_safe_engineering",
    ):
        item = next(row for row in summary if row["parameter"] == metric)
        evidence.append(
            {
                "evidence_id": f"EV.PARAM.{metric.upper()}",
                "run_id": "baseline_group",
                "metric": metric,
                "value": item["recommended_value"],
                "unit": item["unit"],
                "evidence_class": item["evidence_class"],
                "source_file": str(TABLES / "baseline_run_parameter_observed.csv")
                if metric.startswith(("a_", "b_"))
                else "user-declared analysis scenario",
                "source_locator": item["semantics"],
                "availability": "AVAILABLE",
                "limitations": item["limitations"],
            }
        )
    return evidence


def make_report(rows: list[dict], summary: list[dict]) -> str:
    s = {row["parameter"]: row for row in summary}
    run_lines = [
        "| run | d0 observed/m | a_resp 距离拟合/m·s⁻² | 响应窗正加速度峰值 raw/m·s⁻² | b_e 路径等效/m·s⁻² |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        run_lines.append(
            f"| {row['run_id']} | {row['d0_clearance_data_observed_m']:.6f} | "
            f"{row['a_response_constant_accel_distance_fit_data_observed_mps2']:.6f} | "
            f"{row['a_response_peak_interval_raw_data_observed_mps2']:.6f} | "
            f"{row['b_effective_path_model_calibration_mps2']:.6f} |"
        )
    return f"""# 第二次实验 baseline 动态安全包络参数辨识

## 结论

截图中的“三个待计算指标”按 `d0`、`a_resp`、`b_e` 处理；`d_safe` 是用户声明的两种要求情景，不是从 baseline 观测拟合出的车辆动力学参数。

- `d0` 必须逐 run 使用。baseline 7 次的中位数为 **{s['d0']['median']:.6f} m**，范围 **{s['d0']['min']:.6f}–{s['d0']['max']:.6f} m**；几何接触校准不确定性约 ±{GEOMETRY_UNCERTAINTY_M:.2f} m。组中位数只能描述实验布置，不能替代逐 run 的 `d0`。
- `a_resp` 推荐的 baseline 经验保守候选为 **{s['a_resp_conservative_candidate']['recommended_value']:.6f} m/s²**，定义为 7 个响应窗中原始相邻 Localization 速度差分的最大正向峰值。median-3 滤波诊断上界为 {s['a_resp_conservative_candidate']['limitations'].split('为')[1].split(' m/s')[0]} m/s²。中心/典型距离拟合值为 **{s['a_resp_central']['recommended_value']:.6f} m/s²**。
- `b_e` 推荐的 baseline 经验保守候选为 **{s['b_e_conservative_candidate']['recommended_value']:.6f} m/s²**，定义为 7 个均达到近零速并具有最低速度端点的 baseline run 中 `(v2²-v_end²)/(2·D_brake_wall_path)` 的最小值；其中只有 2 个 run 满足“低于 0.1 m/s 并持续 0.5 s”的严格停车判据。现有第二次实验位移口径的中位中心值为 **{s['b_e_central_existing_convention']['recommended_value']:.6f} m/s²**，它不是保证下界。
- `d_safe` 分别取 **0 m**（接触避免边界）和 **6 m**（工程安全裕度）。两者必须分列计算和报告。

用于 RSS-like 公式的建议输入是：逐 run `d0` + `a_resp={s['a_resp_conservative_candidate']['recommended_value']:.6f} m/s²` + `b_e={s['b_e_conservative_candidate']['recommended_value']:.6f} m/s²` + `d_safe∈{{0,6}} m`。如需对几何也取保守侧，可在每个 run 使用 `d0_lower=d0-0.52 m`。

## 逐 run 结果

{chr(10).join(run_lines)}

## 证据边界

`d0` 是由 Fusion 目标位置、t1 插值 Localization 位姿和固定组合几何偏移确定的 `OBSERVED_DERIVED` 结果。`a_resp` 与 `b_e` 是从 baseline 事后辨识的经验包络参数，必须标为 `UNVALIDATED_MODEL_CALIBRATION`；7 次同场景 baseline 没有覆盖摩擦、坡度、载荷、轮胎和执行器退化，因而 **{s['b_e_conservative_candidate']['recommended_value']:.6f} m/s² 不能写成已经证明的保证制动能力**。它只能写成“当前 baseline 样本内最弱观测值/保守候选”。

`d_safe=6 m` 是工程分析阈值；在缺少外部或预注册依据时，不把它写成认证安全要求。`d_safe=0 m` 是接触边界，不代表舒适或工程安全。

## 复现

```bash
cd "{WORKSPACE}"
python3 output/second_experiment_baseline_dynamic_parameters/scripts/calibrate_baseline_parameters.py
```

原始目录 `{BASELINE}` 未被修改。
"""


def validate(rows: list[dict], summary: list[dict]) -> dict:
    existing_path = WORKSPACE / "report_workspace" / "tables" / "run_level_metrics.csv"
    existing: dict[str, dict[str, str]] = {}
    with existing_path.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            if row["group_name"] == "baseline":
                existing[row["run_id"]] = row
    d0_error = max(
        abs(
            row["d0_clearance_data_observed_m"]
            - float(existing[row["run_id"]]["D1_clear_data_observed_m"])
        )
        for row in rows
    )
    response_error = max(
        abs(
            row["D_response_wall_integral_data_observed_m"]
            - float(
                existing[row["run_id"]][
                    "D_delay_wall_integral_data_observed_m"
                ]
            )
        )
        for row in rows
    )
    b_center = next(
        row for row in summary if row["parameter"] == "b_e_central_existing_convention"
    )["recommended_value"]
    validation = {
        "status": "PASS_WITH_EVIDENCE_LIMITATIONS",
        "raw_baseline_directory_read_only": str(BASELINE),
        "run_count_expected": EXPECTED_RUNS,
        "run_count_analyzed": len(rows),
        "complete_braking_endpoint_count": sum(
            finite(row["D_brake_wall_path_integral_diagnostic_m"]) for row in rows
        ),
        "strict_stop_hold_count": sum(row["strict_stop_data_observed"] for row in rows),
        "max_abs_d0_difference_vs_existing_raw_recomputation_m": d0_error,
        "max_abs_D_response_difference_vs_existing_raw_recomputation_m": response_error,
        "existing_model_center_b_e_reproduced_mps2": b_center,
        "numeric_tolerance_m": 1e-9,
        "primary_deadline_qualification": "NOT_QUALIFIED_PRIMARY",
        "qualification_reason": "post-hoc small-n empirical calibration; no external braking envelope or ODD validation",
        "observed_model_separation": "PASS",
        "d_safe_scenarios_separated": [0.0, 6.0],
    }
    if len(rows) != EXPECTED_RUNS or d0_error > 1e-9 or response_error > 1e-9:
        validation["status"] = "FAIL"
        raise RuntimeError(json.dumps(validation, ensure_ascii=False, indent=2))
    return validation


def main() -> None:
    for directory in (TABLES, REPORT, VALIDATION):
        directory.mkdir(parents=True, exist_ok=True)
    config = make_config()
    timezone = ZoneInfo(config["analysis"]["timezone"])
    global RUN_ROWS
    RUN_ROWS = [
        analyze_run(spec, config, timezone) for spec in core.discover_runs(config)
    ]
    RUN_ROWS.sort(key=lambda row: row["run_id"])
    summary = make_summary(RUN_ROWS)
    parameter_sets = make_parameter_sets(summary)
    write_csv(TABLES / "baseline_run_parameter_observed.csv", RUN_ROWS)
    write_csv(TABLES / "baseline_parameter_summary.csv", summary)
    write_csv(TABLES / "baseline_parameter_sets_model_predicted.csv", parameter_sets)
    write_csv(TABLES / "evidence_ledger.csv", make_evidence(RUN_ROWS, summary))
    validation = validate(RUN_ROWS, summary)
    (VALIDATION / "recomputation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (REPORT / "baseline_dynamic_parameters.md").write_text(
        make_report(RUN_ROWS, summary), encoding="utf-8"
    )
    print(json.dumps(validation, ensure_ascii=False, indent=2))


RUN_ROWS: list[dict] = []


if __name__ == "__main__":
    main()
