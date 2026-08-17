#!/usr/bin/env python3
"""TCPS-PA v3.1 single-run analysis for second-experiment run 202607271643.

The evaluated run remains the only source of observed results. The seven
baseline runs are used only as an explicitly unvalidated, disjoint calibration
source for model/predicted dynamic-deadline diagnostics.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path


HERE = Path(__file__).resolve()
WORKSPACE = HERE.parents[3]
SOURCE_SCRIPT = (
    WORKSPACE
    / "output/second_experiment_1131_tcps_pa_v3_1/scripts/analyze_1131_single_run.py"
)
BASELINE_PARAMETER_SETS = (
    WORKSPACE
    / "output/second_experiment_baseline_dynamic_parameters/tables/"
    "baseline_parameter_sets_model_predicted.csv"
)
BASELINE_SUMMARY = (
    WORKSPACE
    / "output/second_experiment_baseline_dynamic_parameters/tables/"
    "baseline_parameter_summary.csv"
)


def load_template() -> str:
    text = SOURCE_SCRIPT.read_text(encoding="utf-8")
    replacements = {
        "202607271131": "202607271643",
        "1131": "1643",
        "476004": "1218462",
        "target-11": "target-6",
        "target 11": "target 6",
        "target=11": "target=6",
        "PASS_TARGET_11": "PASS_TARGET_6",
        "single_run_only": "single_run_observed_plus_disjoint_baseline_model",
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


namespace: dict[str, object] = {"__name__": "tcps_1643_generated_template"}
exec(compile(load_template(), str(SOURCE_SCRIPT), "exec"), namespace)

base_build = namespace["build"]
base_write_csv = namespace["write_csv"]
TABLES: Path = namespace["TABLES"]
REPORT: Path = namespace["REPORT"]
VALIDATION: Path = namespace["VALIDATION"]
RUN_ID = "202607271643"


def model_tau(d_clear: float, v_ego: float, a_resp: float, b_ego: float, d_safe: float) -> tuple[float, str]:
    """Return the RSS-like deadline in seconds for a stationary target."""
    a_quad = 0.5 * a_resp + a_resp**2 / (2.0 * b_ego)
    b_quad = v_ego + v_ego * a_resp / b_ego
    c_quad = v_ego**2 / (2.0 * b_ego) + d_safe - d_clear
    if c_quad > 0.0:
        return 0.0, "ALREADY_UNSAFE_AT_STATE_EPOCH"
    discriminant = b_quad**2 - 4.0 * a_quad * c_quad
    if discriminant < 0.0:
        return math.nan, "CONSTRUCTION_INVALID"
    return (-b_quad + math.sqrt(discriminant)) / (2.0 * a_quad), "SOLVED"


def interpolate(points: list[tuple[float, float]], t: float) -> float:
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t0 <= t <= t1:
            if t1 == t0:
                return v0
            fraction = (t - t0) / (t1 - t0)
            return v0 + fraction * (v1 - v0)
    raise ValueError(f"time {t} is not bracketed by the velocity trajectory")


def integrate(points: list[tuple[float, float]], start: float, end: float) -> float:
    if end <= start:
        return 0.0
    samples = [(start, interpolate(points, start))]
    samples.extend((t, v) for t, v in points if start < t < end)
    samples.append((end, interpolate(points, end)))
    return sum(
        (tb - ta) * (va + vb) / 2.0
        for (ta, va), (tb, vb) in zip(samples, samples[1:])
    )


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"refusing to write empty table: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_model_outputs() -> dict[str, object]:
    observed = read_csv(TABLES / "run_level_observed.csv")[0]
    velocity_rows = read_csv(TABLES / "velocity_trajectory_observed.csv")
    velocity = [
        (float(row["t_wall_s"]), float(row["speed_mps"])) for row in velocity_rows
    ]
    parameter_sets = read_csv(BASELINE_PARAMETER_SETS)
    summary = {row["parameter"]: row for row in read_csv(BASELINE_SUMMARY)}

    t1 = float(observed["t1_wall_s"])
    t2 = float(observed["t2_wall_s"])
    tr_ms = float(observed["T_e2e_data_observed_ms"])
    d_clear = float(observed["D1_clear_data_observed_m"])
    v_ego = float(observed["v1_data_observed_mps"])
    d2_clear = float(observed["D2_clear_data_observed_m"])
    v2_observed = float(observed["v2_data_observed_mps"])
    impact_speed_observed = float(observed["impact_speed_data_observed_mps"])
    calibration_run_ids = parameter_sets[0]["calibration_run_ids"]

    model_rows: list[dict[str, object]] = []
    construction_rows: list[dict[str, object]] = []
    requirement_rows: list[dict[str, object]] = []
    for parameter_set in parameter_sets:
        parameter_set_id = parameter_set["parameter_set_id"]
        a_resp = float(parameter_set["a_resp_model_predicted_mps2"])
        b_ego = float(parameter_set["b_e_model_predicted_mps2"])
        d_safe = float(parameter_set["d_safe_requirement_m"])
        tau_s, construction_status = model_tau(
            d_clear, v_ego, a_resp, b_ego, d_safe
        )
        tau_ms = tau_s * 1000.0
        deadline_wall_s = t1 + tau_s
        debt = integrate(velocity, deadline_wall_s, t2) if deadline_wall_s < t2 else 0.0
        # Deadline construction above uses only t1 state. The following model
        # outcome comparator is deliberately separate and uses the observed t2
        # state to predict the post-response braking outcome.
        braking_distance = v2_observed**2 / (2.0 * b_ego)
        contact_margin = d2_clear - braking_distance
        safety_margin = contact_margin - d_safe
        predicted_impact = (
            math.sqrt(max(0.0, -2.0 * b_ego * contact_margin))
            if contact_margin < 0.0
            else 0.0
        )
        impact_error = predicted_impact - impact_speed_observed
        requirement_id = f"REQ.MODEL.1643.{parameter_set_id}"
        construction_id = f"DDL.MODEL.1643.{parameter_set_id}"
        model_rows.append(
            {
                "run_id": RUN_ID,
                "parameter_set_id": parameter_set_id,
                "model_name": "RSS_LIKE_BASELINE_EMPIRICAL_DYNAMIC_ENVELOPE",
                "model_version": "TCPS-PA-v3.1-baseline-empirical-v1",
                "evidence_class": "UNVALIDATED_MODEL",
                "qualification": "NOT_QUALIFIED_PRIMARY",
                "d_clear_run_observed_m": d_clear,
                "v_ego_run_observed_mps": v_ego,
                "a_resp_model_predicted_mps2": a_resp,
                "b_ego_model_predicted_mps2": b_ego,
                "d_safe_requirement_m": d_safe,
                "tau_model_predicted_ms": tau_ms,
                "t_deadline_model_predicted_s": deadline_wall_s,
                "T_R_data_observed_ms": tr_ms,
                "timing_slack_model_predicted_ms": tau_ms - tr_ms,
                "deadline_miss_model_predicted": tr_ms > tau_ms,
                "D_debt_model_predicted_m": debt,
                "D_brake_model_predicted_m": braking_distance,
                "M_collision_0m_model_predicted_m": contact_margin,
                "M_safety_model_predicted_m": safety_margin,
                "impact_speed_model_predicted_mps": predicted_impact,
                "impact_speed_data_observed_comparator_mps": impact_speed_observed,
                "impact_speed_signed_error_model_minus_observed_mps": impact_error,
                "impact_speed_absolute_error_model_mps": abs(impact_error),
                "impact_speed_relative_error": (
                    abs(impact_error) / impact_speed_observed
                    if impact_speed_observed > 0.0
                    else ""
                ),
                "calibration_run_ids": calibration_run_ids,
                "evaluation_run_ids": RUN_ID,
                "current_run_post_t1_data_used_for_deadline": False,
                "current_run_outcome_used_for_deadline": False,
                "outcome_model_comparator_inputs": (
                    "1643 observed t2 v2/D2 used only for braking and impact prediction; "
                    "not used for deadline construction"
                ),
                "assumptions_and_scope_note": (
                    parameter_set["semantics_and_limitations"]
                    + "；deadline仅使用1643在t1可获得的d_clear/v_ego；"
                    "未使用1643的t2、碰撞标签或碰撞后轨迹构造deadline。"
                ),
            }
        )
        construction_rows.append(
            {
                "construction_id": construction_id,
                "requirement_id": requirement_id,
                "run_id_or_group": RUN_ID,
                "method": "RSS_LIKE_LONGITUDINAL",
                "method_version": "TCPS-PA-v3.1-baseline-empirical-v1",
                "state_time": t1,
                "state_time_basis": "wall_epoch_s",
                "state_available_by_t1": True,
                "input_cutoff_time": t1,
                "latest_input_time": t1,
                "input_provenance_json": json.dumps(
                    {
                        "d_clear_m": "1643 Fusion+Localization state at t1",
                        "v_ego_mps": "1643 Localization interpolation at t1",
                        "v_front_mps": "stationary-target assumption",
                        "d_safe_m": "user-declared scenario",
                        "a_ego_response_max_mps2": parameter_set_id,
                        "b_ego_min_mps2": parameter_set_id,
                        "b_front_max_mps2": "dummy positive value; front term is zero because v_front=0",
                    },
                    ensure_ascii=False,
                ),
                "parameter_selection_time": "post-hoc analysis",
                "parameter_selection_locked_by_t1": False,
                "current_run_post_t1_data_used": False,
                "current_run_outcome_used": False,
                "d_clear_m": d_clear,
                "v_ego_mps": v_ego,
                "v_front_mps": 0.0,
                "d_safe_m": d_safe,
                "a_ego_response_max_mps2": a_resp,
                "b_ego_min_mps2": b_ego,
                "b_front_max_mps2": 1.0,
                "parameter_bounds_json": json.dumps(
                    {
                        "d_clear_m": [d_clear, d_clear],
                        "v_ego_mps": [v_ego, v_ego],
                        "v_front_mps": [0.0, 0.0],
                        "d_safe_m": [d_safe, d_safe],
                        "a_ego_response_max_mps2": [a_resp, a_resp],
                        "b_ego_min_mps2": [b_ego, b_ego],
                        "b_front_max_mps2": [1.0, 1.0],
                    },
                    ensure_ascii=False,
                ),
                "uncertainty_method": "DEGENERATE_PARAMETER_BOX_PER_SCENARIO; completeness does not imply qualification",
                "uncertainty_sample_count": 1,
                "braking_envelope_id": parameter_set_id,
                "braking_envelope_provenance": str(BASELINE_PARAMETER_SETS),
                "braking_envelope_status": "UNVALIDATED",
                "validation_dataset_independent": False,
                "validation_scope": "NONE; seven same-scene baselines only",
                "target_motion_assumption": "stationary obstacle; v_front=0",
                "calibration_run_ids": calibration_run_ids,
                "evaluation_run_ids": RUN_ID,
                "road_condition_assumption": "same nominal simulated scene; friction/grade/load bounds not archived",
                # The shared validator normalizes numeric zero to an empty
                # string during equality checks. Keep the auditable state as
                # ALREADY_UNSAFE_AT_STATE_EPOCH and retain the explicit 0 ms
                # in requirement/model tables, while leaving these three
                # construction cells blank to match that normalization.
                "tau_req_low_ms": tau_ms if tau_ms > 0.0 else "",
                "tau_req_center_ms": tau_ms if tau_ms > 0.0 else "",
                "tau_req_high_ms": tau_ms if tau_ms > 0.0 else "",
                "parameter_bounds_complete": True,
                "construction_status": construction_status,
                "qualification": "NOT_QUALIFIED_PRIMARY",
                "source_evidence_ids": "EV.TARGET.1643|EV.MODEL_DEADLINE.1643",
                "notes": (
                    "Calibration and evaluation run IDs are disjoint, but no independent validation set, "
                    "ODD bounds, guaranteed braking lower bound, or pre-t1 parameter lock exists."
                ),
            }
        )
        requirement_rows.append(
            {
                "requirement_id": requirement_id,
                "run_id_or_group": RUN_ID,
                "requirement_name": f"baseline empirical model dynamic deadline: {parameter_set_id}",
                "requirement_value": tau_ms,
                "unit": "ms",
                "requirement_provenance": str(BASELINE_PARAMETER_SETS),
                "pre_registered": False,
                "external_or_internal": "INTERNAL_POST_HOC_MODEL",
                "safety_meaning": (
                    "contact avoidance boundary"
                    if d_safe == 0.0
                    else "researcher-selected 6 m engineering margin"
                ),
                "deadline_type": "DYNAMIC_CONSTRUCTED_MODEL",
                "evidence_class": "UNVALIDATED_MODEL",
                "tau_req_low_ms": "",
                "tau_req_center_ms": tau_ms,
                "tau_req_high_ms": "",
                "validation_scope": "NONE; same-scene baseline calibration only",
                "p_deadline_qualification": "NOT_QUALIFIED_PRIMARY",
                "notes": "Eligible only for MODEL_SUPPORTED_ONLY diagnostics; not primary C4/C5 evidence.",
            }
        )

    write_csv(TABLES / "run_level_model_predicted.csv", model_rows)
    write_csv(TABLES / "dynamic_deadline_construction.csv", construction_rows)
    write_csv(TABLES / "requirement_registry.csv", requirement_rows)

    geometry_rows: list[dict[str, object]] = []
    conservative_sets = [
        row
        for row in parameter_sets
        if "CONSERVATIVE_CANDIDATE" in row["parameter_set_id"]
    ]
    for parameter_set in conservative_sets:
        a_resp = float(parameter_set["a_resp_model_predicted_mps2"])
        b_ego = float(parameter_set["b_e_model_predicted_mps2"])
        d_safe = float(parameter_set["d_safe_requirement_m"])
        for geometry_policy, d_input in (
            ("RUN_SPECIFIC_D0", d_clear),
            ("D0_MINUS_0P52M", d_clear - 0.52),
        ):
            tau_s, status = model_tau(d_input, v_ego, a_resp, b_ego, d_safe)
            deadline = t1 + tau_s
            geometry_rows.append(
                {
                    "run_id": RUN_ID,
                    "parameter_set_id": parameter_set["parameter_set_id"],
                    "geometry_policy": geometry_policy,
                    "d_clear_input_m": d_input,
                    "d_safe_m": d_safe,
                    "tau_model_predicted_ms": tau_s * 1000.0,
                    "timing_slack_model_predicted_ms": tau_s * 1000.0 - tr_ms,
                    "D_debt_model_predicted_m": (
                        integrate(velocity, deadline, t2) if deadline < t2 else 0.0
                    ),
                    "construction_status": status,
                    "qualification": "NOT_QUALIFIED_PRIMARY",
                    "evidence_class": "UNVALIDATED_MODEL",
                }
            )
    write_csv(TABLES / "dynamic_deadline_geometry_sensitivity.csv", geometry_rows)

    # Evaluation-run response acceleration is a validation diagnostic only; it
    # never enters any deadline calculation above.
    times = [point[0] for point in velocity]
    speeds = [point[1] for point in velocity]
    response_accelerations = [
        (speeds[index + 1] - speeds[index]) / (times[index + 1] - times[index])
        for index in range(len(times) - 1)
        if times[index + 1] > t1 and times[index] < t2
    ]
    a_peak_1643 = max(0.0, max(response_accelerations))
    a_candidate = float(summary["a_resp_conservative_candidate"]["recommended_value"])
    validation_rows = [
        {
            "validation_id": "VAL.A_RESP.1643",
            "run_id": RUN_ID,
            "parameter": "a_resp_conservative_candidate",
            "baseline_candidate_mps2": a_candidate,
            "evaluation_observed_peak_mps2": a_peak_1643,
            "difference_evaluation_minus_candidate_mps2": a_peak_1643 - a_candidate,
            "validation_result": "VIOLATED" if a_peak_1643 > a_candidate else "NOT_VIOLATED",
            "evidence_class": "OBSERVED_DERIVED_VALIDATION_DIAGNOSTIC",
            "deadline_input_usage": "PROHIBITED_CURRENT_RUN_POST_T1",
            "notes": "1643 post-t1 observation tests the baseline envelope but is not fed back into its deadline.",
        },
        {
            "validation_id": "VAL.B_E.1643",
            "run_id": RUN_ID,
            "parameter": "b_e_conservative_candidate",
            "baseline_candidate_mps2": summary["b_e_conservative_candidate"]["recommended_value"],
            "evaluation_observed_peak_mps2": "",
            "difference_evaluation_minus_candidate_mps2": "",
            "validation_result": "NOT_TESTABLE_COLLISION_RIGHT_CENSORING",
            "evidence_class": "MISSING",
            "deadline_input_usage": "NOT_USED",
            "notes": "Collision prevents a complete 1643 stopping endpoint and independent b_e validation.",
        },
    ]
    write_csv(TABLES / "baseline_envelope_validation.csv", validation_rows)

    conservative_contact = next(
        row
        for row in model_rows
        if row["parameter_set_id"]
        == "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_0M"
    )
    conservative_safety = next(
        row
        for row in model_rows
        if row["parameter_set_id"]
        == "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_6M"
    )
    central_contact = next(
        row
        for row in model_rows
        if row["parameter_set_id"] == "BASELINE_EMPIRICAL_CENTRAL_DSAFE_0M"
    )
    central_safety = next(
        row
        for row in model_rows
        if row["parameter_set_id"] == "BASELINE_EMPIRICAL_CENTRAL_DSAFE_6M"
    )
    geometry_conservative_contact = next(
        row
        for row in geometry_rows
        if row["parameter_set_id"]
        == "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_0M"
        and row["geometry_policy"] == "D0_MINUS_0P52M"
    )
    return {
        "observed": observed,
        "model_rows": model_rows,
        "a_peak_1643": a_peak_1643,
        "a_candidate": a_candidate,
        "conservative_contact": conservative_contact,
        "conservative_safety": conservative_safety,
        "central_contact": central_contact,
        "central_safety": central_safety,
        "geometry_conservative_contact": geometry_conservative_contact,
    }


def replace_report(model: dict[str, object]) -> None:
    observed = model["observed"]
    conservative_contact = model["conservative_contact"]
    conservative_safety = model["conservative_safety"]
    central_contact = model["central_contact"]
    central_safety = model["central_safety"]
    report = (REPORT / "six_layer_analysis_report.md").read_text(encoding="utf-8")
    report = report.replace(
        "**范围：仅 `202607271643`；未使用其他 run 的基线、制动模型、deadline 或反事实轨迹**",
        "**范围：1643 的 data/observed 仅来自 `202607271643`；7组 baseline 只用于独立分表的 model/predicted deadline，不回填观测结果**",
    )
    report = report.replace("目标 11", "目标 6").replace("target 11", "target 6")
    report = report.replace(
        "最重要的否定性结论是：**本 run 无合格、事前锁定且独立验证的动态物理 deadline**，也无 WCRT/后缀上界。因此 `C4_OBS=NOT_TESTABLE`、`G1_GUARANTEE=NOT_ESTABLISHED`，不存在可报告的“失去时间保证时刻”；主 `D_debt` 也不可用。",
        "最重要的证据边界是：**7组 baseline 能构造模型 deadline，但它没有资格成为主物理 deadline**。因此 `P_DEADLINE=MODEL_SUPPORTED_ONLY`、`C4_OBS=NOT_TESTABLE`、`G1_GUARANTEE=NOT_ESTABLISHED`；可报告模型包络耗尽时刻和模型距离债务，但主 `D_debt` 仍不可用。",
    )
    report = report.replace(
        "v3.1 独立状态：`P_CLOCK=PASS`、`P_TARGET=PASS`、`P_FUNC=PARTIAL`、`P_PHASE=NOT_TESTABLE`、`P_DEADLINE=NOT_TESTABLE`、`G1_GUARANTEE=NOT_ESTABLISHED`、`E1_EMPIRICAL=DESCRIPTIVE_ONLY`。",
        "v3.1 独立状态：`P_CLOCK=PASS`、`P_TARGET=PASS`、`P_FUNC=PARTIAL`、`P_PHASE=NOT_TESTABLE`、`P_DEADLINE=MODEL_SUPPORTED_ONLY`、`G1_GUARANTEE=NOT_ESTABLISHED`、`E1_EMPIRICAL=DESCRIPTIVE_ONLY`。",
    )
    report = report.replace(
        "median-3 平滑则延至 899.502 ms",
        "median-3 平滑则延至 993.107 ms",
    )
    report = report.replace(
        "Control→$t_2$ 的 486.037 ms 是最大分段。以已记录的 300.047 ms 注入精度做**非因果算术分解**，剩余 **241.156 ms**；",
        "Control→$t_2$ 的 541.256 ms 是最大分段。以已记录的 300.100 ms 注入精度做**非因果算术分解**，剩余 **241.156 ms**；",
    )
    report = report.replace(
        "直接证据确认的 300.047 ms 外部注入型固定时延",
        "直接证据确认的 300.100 ms 外部注入型固定时延",
    )
    report = report.replace(
        "首帧 source→Fusion 为 292.885 ms，主要由 101.614 ms 入口年龄、61.478 ms Detection 前等待和 98.163 ms Detection 处理组成。",
        "首帧 source→Fusion 为 319.285 ms，主要由 109.791 ms 入口年龄、93.373 ms Detection 前等待和 96.037 ms Detection 处理组成。",
    )
    report = report.replace(
        "但不是引起 486 ms 的处理时间瓶颈。",
        "但不是引起 541 ms 的处理时间瓶颈。",
    )
    report = report.replace(
        "**Control→物理效应（L3）**：486.037 ms",
        "**Control→物理效应（L3）**：541.256 ms",
    )
    report = report.replace(
        "**$t_2$ 后持续闭环（L2/L3）**：507.439 ms 输出缺口与约 700 ms lifecycle 峰值发生在 $t_2$ 之后。它们对“为何首次 $t_2$ 延迟”已被时序反证，但对碰撞前持续制动新鲜度仍是未解候选。",
        "**$t_2$ 后持续闭环（L2/L3）**：最大目标输出间隔仅 115.906 ms、lifecycle 最大 322.269 ms，没有出现1131的507 ms级空档；这不支持把1643碰撞归因于同类持续新鲜度崩塌。",
    )
    report = report.replace(
        "5. **$t_2$ 后持续闭环（L2/L3）**：507.439 ms 输出缺口与约 700 ms lifecycle 峰值发生在 $t_2$ 之后。它们对“为何首次制动晚”已被时序反证，但对碰撞前持续制动新鲜度仍是未解候选。",
        "5. **$t_2$ 后持续闭环（L2/L3）**：最大目标输出间隔仅 115.906 ms、lifecycle 最大 322.269 ms，没有出现1131的507 ms级空档；这不支持把1643碰撞归因于同类持续新鲜度崩塌。",
    )
    report = report.replace(
        "| L4 / C4 | NOT_TESTABLE | 已观测 $T_R$，但无合格 $\\tau_{req}$ |",
        "| L4 / C4 | NOT_TESTABLE | 模型支持miss，但主C4因无合格 $\\tau_{req}$ 仍不可检验 |",
    )
    report = report.replace(
        "| L5 / C5 | NOT_TESTABLE | $D_{response}$ 可用；requirement-constrained $D_{debt}$ 不可用 |",
        "| L5 / C5 | NOT_TESTABLE | 模型债务单列为MODEL_SUPPORTED_ONLY；主$D_{debt}$不可用 |",
    )
    report = report.replace(
        "| Reaction $R$ | 893.870 ms | $t_1\\to t_2$ | 可观测；无 deadline，不可判 miss |",
        "| Reaction $R$ | 893.870 ms | $t_1\\to t_2$ | 可观测；模型支持miss，但无qualified deadline |",
    )
    start = report.index("## 时间保证与动态契约")
    end = report.index("## Space budget / 空间预算、物理传播与安全损失")
    deadline_section = f"""## 时间保证与动态契约

1643 自身在 $t_1$ 可获得的状态为 $d_0={float(observed['D1_clear_data_observed_m']):.3f}$ m、$v_1={float(observed['v1_data_observed_mps']):.3f}$ m/s；目标按静态障碍物处理。将7组 baseline 事后辨识参数作为**不合格模型**输入，可复算得到：

| 参数情景 | $d_{{safe}}$ | $\\tau_{{model}}$ | 相对 $T_R$ | $D_{{debt,model}}$ |
|---|---:|---:|---:|---:|
| baseline中心参数 | 0 m | {float(central_contact['tau_model_predicted_ms']):.3f} ms | 失约 {abs(float(central_contact['timing_slack_model_predicted_ms'])):.3f} ms | {float(central_contact['D_debt_model_predicted_m']):.3f} m |
| baseline保守候选 | 0 m | **{float(conservative_contact['tau_model_predicted_ms']):.3f} ms** | 模型失约 **{abs(float(conservative_contact['timing_slack_model_predicted_ms'])):.3f} ms** | **{float(conservative_contact['D_debt_model_predicted_m']):.3f} m** |
| baseline中心参数 | 6 m | {float(central_safety['tau_model_predicted_ms']):.3f} ms | 失约 {abs(float(central_safety['timing_slack_model_predicted_ms'])):.3f} ms | {float(central_safety['D_debt_model_predicted_m']):.3f} m |
| baseline保守候选 | 6 m | 0 ms | $t_1$ 时已在模型包络外 | {float(conservative_safety['D_debt_model_predicted_m']):.3f} m |

若按baseline报告建议把几何净距取保守侧 $d_0-0.52$ m，则保守候选的0 m模型deadline进一步降为 **{float(model['geometry_conservative_contact']['tau_model_predicted_ms']):.3f} ms**，模型债务增为 **{float(model['geometry_conservative_contact']['D_debt_model_predicted_m']):.3f} m**。这只是几何敏感性，不是新的主结果。

但这不能升级为 primary deadline：参数是事后小样本校准，只有2/7满足严格停车判据，缺少摩擦、坡度、载荷和执行器退化的ODD边界，也没有独立验证集。更重要的是，1643 响应窗的事后观测正向加速度峰值为 **{float(model['a_peak_1643']):.3f} m/s²**，超过 baseline 候选 **{float(model['a_candidate']):.3f} m/s²**，直接反证其为有效上包络；碰撞右截尾又使 `b_e` 无法在1643完整验证。

因此主状态仍是 `P_DEADLINE=MODEL_SUPPORTED_ONLY`、`C4_OBS=NOT_TESTABLE`、`C5_PRIMARY=NOT_TESTABLE`。可报告的是“baseline经验模型包络在 $t_1+{float(conservative_contact['tau_model_predicted_ms']):.3f}$ ms 耗尽”，不能把它写成系统已有保证实际丧失的时刻；相应 **{float(conservative_contact['D_debt_model_predicted_m']):.3f} m** 只能叫 `D_debt_model_predicted`。

"""
    report = report[:start] + deadline_section + report[end:]
    report = report.replace(
        "| 主 $D_{debt}$ | 不可用 | 无 qualified $\\tau_{req}$ |",
        f"| 主 $D_{{debt}}$ | 不可用 | 无 qualified $\\tau_{{req}}$；保守候选模型债务为 {float(conservative_contact['D_debt_model_predicted_m']):.3f} m，另表保存 |",
    )
    report = report.replace(
        "| timing 因果物理损失 | 不可定量 | 无 qualified deadline 和事前锁定的现实反事实轨迹 |",
        f"| timing 因果物理损失 | 主结论不可定量 | baseline保守候选模型给出 {float(conservative_contact['D_debt_model_predicted_m']):.3f} m 债务，但带 `MODEL_TAINT` |",
    )
    report = report.replace(
        "| 什么时候失去时间保证？ | 不可判定。没有已建立的合格 deadline/WCRT/suffix bound，因而没有合法的 guarantee-loss 时刻。 |",
        f"| 什么时候失去时间保证？ | 系统保证的真实丧失时刻仍不可判定；baseline保守候选模型的0 m包络在 $t_1+{float(conservative_contact['tau_model_predicted_ms']):.3f}$ ms 耗尽，但该模型未资格化。 |",
    )
    core_start = report.index("## 核心问题的最终回答")
    core_end = report.index("## 验证、方法完备性与复现")
    core_section = f"""## 核心问题的最终回答

| 问题 | 1643 run 可支持的回答 |
|---|---|
| 哪里出了实时性问题？ | 首次响应的主要时间消耗在 source→Fusion（319.285 ms）和 Control→物理 $t_2$（541.256 ms）；300.100 ms Bridge固定延时是后者的主要候选构成。 |
| 它是什么性质？ | 已证明的是外部注入型Bridge固定时延与事件级长响应；Planning常减速fallback使其同时是功能/时间多因素事件，不是Apollo内生实时缺陷的单-run证明。 |
| 什么时候失去时间保证？ | 系统保证的真实丧失时刻仍不可判定；baseline保守候选模型的0 m包络在 $t_1+{float(conservative_contact['tau_model_predicted_ms']):.3f}$ ms 耗尽，但该模型未资格化。 |
| 为什么？ | 候选机制是持续300 ms Bridge延时、Perception入口年龄与Detection前等待、Planning fallback，以及Control→物理未分解残差。1643没有1131式507 ms输出空档，因此持续新鲜度崩塌不是首要解释。 |
| 造成多少物理安全损失？ | 直接观测为15.451 m响应距离、23.378 m碰撞截尾制动距离、11.728 m/s撞击速度和24070.4冲量模。15.451 m不能写成主deadline债务；baseline保守候选模型债务为{float(conservative_contact['D_debt_model_predicted_m']):.3f} m，只是带`MODEL_TAINT`的诊断量。 |

"""
    report = report[:core_start] + core_section + report[core_end:]
    report = report.replace(
        "- data/observed 与 model/predicted 分表；模型表明确写 `NOT_COMPUTED`，没有以模型补观测缺失。",
        "- data/observed 与 model/predicted 分表；baseline模型只写入 `run_level_model_predicted.csv`，没有以模型补观测缺失。",
    )
    report = report.replace(
        "This directory is generated from the selected run's raw data only.",
        "Observed results are generated from the selected run only; baseline data enter only the separate unvalidated model diagnostics.",
    )
    (REPORT / "six_layer_analysis_report.md").write_text(report, encoding="utf-8")


def augment_quality_and_summary(model: dict[str, object]) -> None:
    audit_path = VALIDATION / "data_quality_audit.md"
    audit = audit_path.read_text(encoding="utf-8")
    audit = audit.replace(
        "- No cross-run measurements, calibrations, or counterfactuals were imported.",
        "- No cross-run value enters data/observed. Seven baseline runs enter only separate UNVALIDATED_MODEL deadline diagnostics; calibration/evaluation IDs are disjoint.",
    )
    audit += (
        "\n## Baseline model validation boundary\n\n"
        f"- 1643 response-window observed positive-acceleration peak: {float(model['a_peak_1643']):.6f} m/s².\n"
        f"- Baseline conservative candidate: {float(model['a_candidate']):.6f} m/s².\n"
        "- The evaluation observation exceeds the candidate, so it is not a valid response-acceleration upper envelope.\n"
        "- Collision right-censoring prevents a complete evaluation-run braking-envelope validation.\n"
    )
    audit_path.write_text(audit, encoding="utf-8")

    summary_path = VALIDATION / "analysis_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    conservative_contact = model["conservative_contact"]
    summary.update(
        {
            "primary_deadline_status": "NOT_QUALIFIED_PRIMARY",
            "p_deadline_status": "MODEL_SUPPORTED_ONLY",
            "baseline_conservative_contact_tau_model_ms": conservative_contact[
                "tau_model_predicted_ms"
            ],
            "baseline_conservative_contact_D_debt_model_m": conservative_contact[
                "D_debt_model_predicted_m"
            ],
            "baseline_a_resp_candidate_validation": "VIOLATED_BY_1643_POST_T1_DIAGNOSTIC",
        }
    )
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def augment_ledgers(model: dict[str, object]) -> None:
    """Expose model evidence without letting it satisfy primary C4/C5 gates."""
    conservative_contact = model["conservative_contact"]
    evidence_path = TABLES / "evidence_ledger.csv"
    evidence = read_csv(evidence_path)
    for row in evidence:
        if row["evidence_id"] == "EV.NODEADLINE.1643":
            row["metric"] = "qualified_primary_deadline_status"
            row["value"] = "primary unavailable; unvalidated model available"
            row["limitations"] = (
                "baseline model supports diagnostics only; no qualified tau_req, "
                "guarantee-loss point, or primary D_debt"
            )
    evidence.extend(
        [
            {
                "evidence_id": "EV.MODEL_DEADLINE.1643",
                "run_id": RUN_ID,
                "layer": "P_DEADLINE/L4",
                "metric": "tau_model_baseline_conservative_contact",
                "value": conservative_contact["tau_model_predicted_ms"],
                "unit": "ms",
                "evidence_class": "UNVALIDATED_MODEL",
                "clock_domain": "wall epoch construction",
                "source_file": str(TABLES / "run_level_model_predicted.csv"),
                "source_locator": "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_0M",
                "availability": "AVAILABLE",
                "confidence": "LOW",
                "supports_claim_ids": "P_DEADLINE.1643|C4.1643",
                "challenges_claim_ids": "P_DEADLINE.1643|C4.1643|C7.1643",
                "limitations": "unvalidated post-hoc baseline model; not primary deadline evidence",
                "semantic_role": "MODEL_DEADLINE_DIAGNOSTIC",
                "reference_type": "DISJOINT_BASELINE_CALIBRATION",
                "distribution_scope": "SEVEN_BASELINE_RUNS",
                "causal_lineage_grade": "",
            },
            {
                "evidence_id": "EV.MODEL_DEBT.1643",
                "run_id": RUN_ID,
                "layer": "L5",
                "metric": "D_debt_model_predicted_m",
                "value": conservative_contact["D_debt_model_predicted_m"],
                "unit": "m",
                "evidence_class": "UNVALIDATED_MODEL",
                "clock_domain": "wall epoch observed velocity + model deadline",
                "source_file": str(TABLES / "run_level_model_predicted.csv"),
                "source_locator": "BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_0M",
                "availability": "AVAILABLE",
                "confidence": "LOW",
                "supports_claim_ids": "C5.1643",
                "challenges_claim_ids": "C5.1643|C7.1643",
                "limitations": "MODEL_TAINT; not requirement-constrained primary debt",
                "semantic_role": "MODEL_DISTANCE_DEBT_DIAGNOSTIC",
                "reference_type": "DISJOINT_BASELINE_CALIBRATION",
                "distribution_scope": "SEVEN_BASELINE_RUNS",
                "causal_lineage_grade": "",
            },
        ]
    )
    write_csv(evidence_path, evidence)

    claims_path = TABLES / "claim_ledger.csv"
    claims = read_csv(claims_path)
    for row in claims:
        if row["claim_id"] == "P_DEADLINE.1643":
            row["verdict"] = "MODEL_SUPPORTED_ONLY"
            row["supporting_evidence_ids"] = "EV.NODEADLINE.1643|EV.MODEL_DEADLINE.1643"
            row["challenging_evidence_ids"] = "EV.NODEADLINE.1643|EV.MODEL_DEADLINE.1643"
            row["residual_uncertainty"] = "A disjoint baseline model exists, but it lacks independent validation, complete bounds, and pre-t1 lock."
            row["allowed_language"] = "An unvalidated baseline model deadline is available; primary tau_req is unavailable."
        elif row["claim_id"] == "C4.1643":
            row["supporting_evidence_ids"] = "EV.REACTION.1643|EV.NODEADLINE.1643|EV.MODEL_DEADLINE.1643"
            row["challenging_evidence_ids"] = "EV.NODEADLINE.1643|EV.MODEL_DEADLINE.1643"
            row["residual_uncertainty"] = "T_R exceeds the unvalidated model deadline, but no qualified tau_req exists."
            row["allowed_language"] = "Model evidence supports a possible deadline miss; observed primary C4 remains not testable."
            row["forbidden_language"] = "A collision proves a deadline miss.|The 893.870 ms response violated a qualified requirement."
        elif row["claim_id"] == "C5.1643":
            row["supporting_evidence_ids"] = "EV.DRESPONSE.1643|EV.MODEL_DEBT.1643"
            row["challenging_evidence_ids"] = "EV.NODEADLINE.1643|EV.MODEL_DEBT.1643"
            row["residual_uncertainty"] = "Observed response distance is available; model debt retains MODEL_TAINT and primary debt is unavailable."
            row["allowed_language"] = "15.451 m is observed response distance; 13.018 m is model-predicted debt, not primary debt."
            row["forbidden_language"] = "15.451 m is timing-caused safety loss.|13.018 m is observed primary deadline debt."
        elif row["claim_id"] == "C7.1643":
            row["allowed_language"] = (
                "An injected temporal stressor, delayed physical response, Planning fallback, "
                "and collision co-occur in one chain; causal share is unresolved."
            )
            row["forbidden_language"] = (
                "The delay caused the collision.|Timing was the sole cause.|"
                "The injected delay caused 15.451 m of safety loss."
            )
    write_csv(claims_path, claims)

    method_path = TABLES / "method_completeness_matrix.csv"
    methods = read_csv(method_path)
    for row in methods:
        if row.get("requirement") == "single-run scope; no other-run evidence":
            row["requirement"] = "single-run observed scope; baseline model separated"
            row["evidence_or_gap"] = (
                "1643 alone supplies data/observed; seven baseline runs supply only "
                "UNVALIDATED_MODEL parameters"
            )
        elif row.get("requirement") == "observed/model separation":
            row["status"] = "PASS"
            row["evidence_or_gap"] = (
                "1643 data/observed is isolated from the separate seven-baseline "
                "UNVALIDATED_MODEL deadline and outcome diagnostics"
            )
        elif row.get("requirement") == "prospective dynamic deadline":
            row["status"] = "MODEL_SUPPORTED_ONLY"
            row["evidence_or_gap"] = (
                "disjoint baseline calibration produces a numerical model deadline; "
                "independent validation, ODD bounds and pre-t1 lock are missing"
            )
    write_csv(method_path, methods)

    chart_path = TABLES / "chart_map.csv"
    charts = read_csv(chart_path)
    for row in charts:
        if row.get("figure") == "event_chain_timeline.png":
            row["takeaway"] = (
                "893.870 ms to physical braking; Control-to-t2 is the largest segment."
            )
        elif row.get("figure") == "speed_and_events.png":
            row["takeaway"] = (
                "Speed rises until the t2 sample, then falls but remains 11.728 m/s at impact."
            )
        elif row.get("figure") == "target_freshness_timeline.png":
            row["takeaway"] = (
                "No 507 ms-class gap exists; the 115.906 ms maximum gap begins after t2."
            )
        row.pop("answer", None)
    write_csv(chart_path, charts)

    diagnosis_path = TABLES / "diagnosis_hypothesis_ledger.csv"
    diagnoses = read_csv(diagnosis_path)
    for row in diagnoses:
        if row.get("hypothesis_id") == "H.INITIAL_GAP.1643":
            row["hypothesis"] = (
                "The run-wide 115.906 ms target gap caused the first t1-to-t2 response delay."
            )
            row["residual_uncertainty"] = (
                "No 507 ms-class gap exists in 1643; the largest observed gap still occurs after t2."
            )
    write_csv(diagnosis_path, diagnoses)

    defeater_path = TABLES / "defeater_ledger.csv"
    defeaters = read_csv(defeater_path)
    for row in defeaters:
        if row.get("defeater_id") == "D_UPDATE_GAP.C7.1643":
            row["resolution"] = (
                "115.906 ms maximum gap is after t2 and is refuted for the initial response; "
                "no 507 ms-class gap is present."
            )
        if row.get("defeater_id") in {
            "D_INITIAL_SPEED.C7.1643",
            "D_PREHAZARD_STATE.C7.1643",
        }:
            row["resolution"] = row.get("resolution", "").replace("25.997", "22.237")
    write_csv(defeater_path, defeaters)

    prehazard_path = TABLES / "pre_hazard_state_audit.csv"
    prehazard = read_csv(prehazard_path)
    for row in prehazard:
        row["notes"] = row.get("notes", "").replace("25.997", "22.237")
    write_csv(prehazard_path, prehazard)

    stage_path = TABLES / "stage_timing_and_freshness.csv"
    stage_rows = read_csv(stage_path)
    for row in stage_rows:
        row["interpretation"] = row.get("interpretation", "").replace(
            "300.047 ms", "300.100 ms"
        )
    write_csv(stage_path, stage_rows)


def build() -> dict[str, object]:
    summary = base_build()
    model = build_model_outputs()
    replace_report(model)
    augment_quality_and_summary(model)
    augment_ledgers(model)
    return {
        **summary,
        "p_deadline_status": "MODEL_SUPPORTED_ONLY",
        "baseline_contact_tau_model_ms": model["conservative_contact"][
            "tau_model_predicted_ms"
        ],
        "baseline_contact_D_debt_model_m": model["conservative_contact"][
            "D_debt_model_predicted_m"
        ],
        "a_resp_baseline_candidate_validation": "VIOLATED",
    }


if __name__ == "__main__":
    print(json.dumps(build(), ensure_ascii=False, indent=2))
