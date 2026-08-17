#!/usr/bin/env python3
"""Map the three late 1131 timing anomalies to Apollo and assess collision impact.

The script intentionally separates direct observations from model-only counterfactuals.
Raw run files are read-only; generated evidence stays in the v4.2 output tree.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path("/Users/huangjinhui/Desktop/萨卡班/data")
RUN_DIR = WORKSPACE / "第二次实验/300ms/202607271131"
OUTPUT = WORKSPACE / "output/second_experiment_1131_tcps_pa_v4_2_all_instances"
TABLES = OUTPUT / "tables"
REPORT = OUTPUT / "report"
RUN_ID = "202607271131"

ACTOR_HISTORY = RUN_DIR / "log/carla_collision_actor_history_20260727113204.csv"
PLANNING_LOG = RUN_DIR / "log/planning.log.INFO.20260727-112936.476000"

APOLLO_ARCHITECTURE_URL = (
    "https://apollo.baidu.com/docs/apollo/10.x/"
    "md_docs_2_xE6_xA1_x86_xE6_x9E_xB6_xE8_xAE_xA1_2_xE6_xA0_xB8_xE5_xBF_x83_xE6_xA8_xA1_4c7232e9aaaaf30846a51b133b9b71bf.html"
)
APOLLO_LIDAR_DAG_URL = (
    "https://apollo.baidu.com/docs/apollo/10.x/lidar__fusion__output_8dag_source.html"
)
APOLLO_PLANNING_README_URL = (
    "https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_component/README_cn.md"
)
APOLLO_PLANNING_ARCH_URL = (
    "https://github.com/ApolloAuto/apollo/blob/master/docs/07_Prediction/Class_Architecture_Planning.md?plain=1"
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def first(rows: list[dict[str, str]], **conditions: str) -> dict[str, str]:
    return next(
        row
        for row in rows
        if all(str(row.get(key)) == str(value) for key, value in conditions.items())
    )


def upsert_rows(path: Path, new_rows: list[dict], key: str) -> None:
    existing = read_csv(path) if path.exists() else []
    new_keys = {str(row[key]) for row in new_rows}
    kept = [row for row in existing if str(row.get(key)) not in new_keys]
    fields: list[str] = []
    for row in kept + new_rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    write_csv(path, kept + new_rows, fields)


def actor_trajectory() -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for row in read_csv(ACTOR_HISTORY):
        if row["role"] != "ego":
            continue
        speed = math.hypot(float(row["velocity_x"]), float(row["velocity_y"]))
        points.append(
            (float(row["wall_time_unix_ns"]) / 1e9, float(row["location_x"]), speed)
        )
    return sorted(points)


def interpolate(points: list[tuple[float, float, float]], t: float, index: int) -> float:
    times = [point[0] for point in points]
    position = bisect.bisect_right(times, t)
    if position == 0:
        return points[0][index]
    if position == len(points):
        return points[-1][index]
    left = points[position - 1]
    right = points[position]
    fraction = (t - left[0]) / (right[0] - left[0])
    return left[index] + fraction * (right[index] - left[index])


def wall_speed_integral(
    points: list[tuple[float, float, float]], start: float, end: float
) -> float:
    samples = [(start, interpolate(points, start, 2))]
    samples.extend((t, speed) for t, _, speed in points if start < t < end)
    samples.append((end, interpolate(points, end, 2)))
    return sum(
        (right[0] - left[0]) * (left[1] + right[1]) / 2.0
        for left, right in zip(samples, samples[1:])
    )


def update_claim_evidence(
    anomaly_start: float,
    anomaly_end: float,
    t_phys: float,
    collision: float,
    speed_start: float,
    speed_end: float,
    clear_start: float,
    required_decel: float,
) -> None:
    evidence_rows = [
        {
            "evidence_id": "EV.LATEANOM.ORDER.1131",
            "run_id": RUN_ID,
            "layer": "L2/L3/L6",
            "metric": "late_anomaly_event_order",
            "value": f"start={anomaly_start:.6f};t_phys={t_phys:.6f};collision={collision:.6f}",
            "unit": "wall_epoch_s",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_basis": "aligned wall epoch",
            "source": str(TABLES / "anomaly_apollo_impact_mapping.csv"),
            "locator": "three selected anomaly rows",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C2.1131|C3.1131",
            "challenges_claim_ids": "C7.1131",
            "limitations": "post-t_phys ordering excludes explanation of initial t_phys but not late closed-loop degradation",
            "evidence_role": "EVENT_ORDER_AND_ATTRIBUTION_BOUND",
        },
        {
            "evidence_id": "EV.LATEANOM.PRIORPLAN.1131",
            "run_id": RUN_ID,
            "layer": "P_FUNC/L3",
            "metric": "pre_anomaly_planning_output",
            "value": "STOP_BY_11;trajectory_type=3;max_abs_decel=4",
            "unit": "planning output",
            "evidence_class": "DIRECT_OBSERVED",
            "clock_basis": "Apollo log wall epoch",
            "source": str(PLANNING_LOG),
            "locator": "line 230375, followed by lines 230525/230655/230778",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "P_FUNC.1131|C3.1131",
            "challenges_claim_ids": "C7.1131",
            "limitations": "Control payload and exact trajectory-reuse semantics are not archived",
            "evidence_role": "PRIOR_VALID_BRAKING_PLAN",
        },
        {
            "evidence_id": "EV.LATEANOM.PHYSSTATE.1131",
            "run_id": RUN_ID,
            "layer": "L5/L6",
            "metric": "physical_state_during_late_anomaly",
            "value": (
                f"v={speed_start:.6f}->{speed_end:.6f};"
                f"clear_start={clear_start:.6f};required_decel={required_decel:.6f}"
            ),
            "unit": "m/s;m;m/s2",
            "evidence_class": "OBSERVED_DERIVED",
            "clock_basis": "CARLA actor wall epoch; perception-target longitudinal endpoint",
            "source": str(ACTOR_HISTORY),
            "locator": f"interpolated [{anomaly_start:.6f},{anomaly_end:.6f}]",
            "availability": "AVAILABLE",
            "confidence": "HIGH",
            "supports_claim_ids": "C6.1131",
            "challenges_claim_ids": "C7.1131",
            "limitations": "required deceleration is a kinematic diagnostic, not a qualified contract",
            "evidence_role": "LATE_PHYSICAL_ENVELOPE",
        },
        {
            "evidence_id": "EV.LATEANOM.COUNTERFACTUAL.1131",
            "run_id": RUN_ID,
            "layer": "L4_MODEL/L6",
            "metric": "remove_three_late_anomalies_counterfactual",
            "value": "NOT_IDENTIFIABLE_OBSERVED;MODEL_COLLISION_REMAINS_LIKELY",
            "unit": "status",
            "evidence_class": "UNVALIDATED_MODEL",
            "clock_basis": "wall epoch with saved actor state",
            "source": str(TABLES / "anomaly_collision_counterfactual.csv"),
            "locator": "observed/model-separated scenarios",
            "availability": "PARTIAL",
            "confidence": "MEDIUM",
            "supports_claim_ids": "",
            "challenges_claim_ids": "C7.1131",
            "limitations": "single run, no replay without anomalies, no event-local Control payload, model domain unvalidated",
            "evidence_role": "COUNTERFACTUAL_BOUND",
        },
    ]
    upsert_rows(TABLES / "evidence_ledger.csv", evidence_rows, "evidence_id")

    claim_path = TABLES / "claim_ledger.csv"
    claims = read_csv(claim_path)
    for row in claims:
        if row.get("claim_id") != "C7.1131":
            continue
        ids = [item for item in row.get("challenging_evidence_ids", "").split("|") if item]
        for evidence_id in (
            "EV.LATEANOM.ORDER.1131",
            "EV.LATEANOM.PRIORPLAN.1131",
            "EV.LATEANOM.PHYSSTATE.1131",
            "EV.LATEANOM.COUNTERFACTUAL.1131",
        ):
            if evidence_id not in ids:
                ids.append(evidence_id)
        row["challenging_evidence_ids"] = "|".join(ids)
        row["residual_uncertainty"] = (
            "The three selected spikes occur after t_phys; a STOP/fallback plan predates them. "
            "Their attributable collision effect is not identifiable without replay and payload/apply evidence."
        )
        row["allowed_language"] = (
            "The three late anomalies degraded refresh integrity, but removing only them is not "
            "observationally shown to prevent the collision; the declared braking models indicate "
            "the vehicle was already outside the contact-stop envelope."
        )
    write_csv(claim_path, claims)


def update_artifact(impact_rows: list[dict], counterfactual_rows: list[dict]) -> None:
    path = REPORT / "artifact.json"
    artifact = json.loads(path.read_text(encoding="utf-8"))
    generated_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    artifact["manifest"]["generatedAt"] = generated_at
    artifact["snapshot"]["generatedAt"] = generated_at
    artifact["manifest"]["description"] = (
        "TCPS-PA v4.2：全 run 逐实例、Apollo 链路映射，以及 observed/model 分离的碰撞反事实。"
    )

    impact_dataset = [
        {
            "order": index,
            "module": row["module"],
            "anomaly_ms": round(float(row["duration_ms"]), 3),
            "p50_ms": round(float(row["p50_ms"]), 3),
            "apollo_effect": row["apollo_system_effect"],
            "collision_status": row["collision_causal_status"],
        }
        for index, row in enumerate(impact_rows, 1)
    ]
    counterfactual_dataset = [
        {
            "order": index,
            "scenario": row["scenario"],
            "evidence_class": row["evidence_class"],
            "safety_metric": row["safety_metric"],
            "value": row["value"],
            "verdict": row["verdict"],
        }
        for index, row in enumerate(counterfactual_rows, 1)
    ]
    artifact["snapshot"]["datasets"]["anomaly_apollo_impact"] = impact_dataset
    artifact["snapshot"]["datasets"]["collision_counterfactual"] = counterfactual_dataset

    tables = artifact["manifest"]["tables"]
    tables = [
        table
        for table in tables
        if table["id"] not in {"anomaly_apollo_impact_table", "collision_counterfactual_table"}
    ]
    tables.extend(
        [
            {
                "id": "anomaly_apollo_impact_table",
                "title": "三个异常映射到 Apollo 后的直接影响",
                "subtitle": "三者均晚于 t_phys；后期刷新退化不等于首次制动延迟的原因。",
                "dataset": "anomaly_apollo_impact",
                "sourceId": "impact_analysis",
                "defaultSort": {"field": "module", "direction": "asc"},
                "density": "dense",
                "columns": [
                    {"field": "module", "label": "模块", "type": "text"},
                    {"field": "anomaly_ms", "label": "异常实例 ms", "type": "number"},
                    {"field": "p50_ms", "label": "run P50 ms", "type": "number"},
                    {"field": "apollo_effect", "label": "Apollo 系统影响", "type": "text"},
                    {"field": "collision_status", "label": "碰撞因果状态", "type": "text"},
                ],
            },
            {
                "id": "collision_counterfactual_table",
                "title": "碰撞反事实：直接观测与模型诊断分开",
                "subtitle": "模型结果不能覆盖实际数据结果；‘删除晚期三异常’与‘删除初始响应延迟’是不同反事实。",
                "dataset": "collision_counterfactual",
                "sourceId": "counterfactual_analysis",
                "defaultSort": {"field": "scenario", "direction": "asc"},
                "density": "dense",
                "columns": [
                    {"field": "scenario", "label": "情景", "type": "text"},
                    {"field": "evidence_class", "label": "证据类别", "type": "text"},
                    {"field": "safety_metric", "label": "安全量", "type": "text"},
                    {"field": "value", "label": "值", "type": "text"},
                    {"field": "verdict", "label": "结论", "type": "text"},
                ],
            },
        ]
    )
    artifact["manifest"]["tables"] = tables

    blocks = [
        block
        for block in artifact["manifest"]["blocks"]
        if block["id"]
        not in {
            "collision_impact_summary",
            "apollo_mapping_heading",
            "anomaly_apollo_impact",
            "counterfactual_heading",
            "collision_counterfactual",
        }
    ]
    summary_index = next(
        index for index, block in enumerate(blocks) if block["id"] == "technical_summary"
    )
    blocks[summary_index]["body"] = (
        "## 技术摘要\n\n"
        "三个约 0.5 s 的异常均发生在首次物理制动 `t_phys` 之后。它们确实造成后期闭环刷新退化："
        "感知目标更新出现 507.439 ms 空洞，Planning 新轨迹刷新停顿，Control 对旧 trace 重复发布。"
        "但异常前已有针对障碍物 11 的 STOP/最大减速度 4 m/s² 轨迹，异常期间车辆仍持续减速。\n\n"
        "因此，**删除这三个晚期异常并不能由本 run 直接推出‘不会碰撞’**。直接观测的反事实为 NOT_IDENTIFIABLE；"
        "在已声明但未完全验证的制动模型中，车辆在 `t_phys` 以及最早异常开始时均已越过 0 m 接触停止边界，"
        "所以只消除这三处晚期尖峰，碰撞仍很可能发生。"
    )
    new_blocks = [
        {
            "id": "collision_impact_summary",
            "type": "markdown",
            "body": (
                "## 结论先行\n\n"
                "- 三异常的可证影响是**晚期 Closed-Loop Timing Integrity 退化**，不是首次制动开始过晚的解释。\n"
                "- 最早异常开始时，车辆约 `12.544 m/s`，一致口径净距约 `6.729 m`；瞬时停车至少需要约 `11.692 m/s²`。\n"
                "- 直接观测不能回答无异常 replay 的最终结果；模型诊断则指向‘只删除这三异常仍会碰撞’。"
            ),
        },
        {"id": "apollo_mapping_heading", "type": "markdown", "body": "## 映射到 Apollo 自动驾驶链路"},
        {"id": "anomaly_apollo_impact", "type": "table", "tableId": "anomaly_apollo_impact_table"},
        {"id": "counterfactual_heading", "type": "markdown", "body": "## 无三异常时是否安全：反事实边界"},
        {"id": "collision_counterfactual", "type": "table", "tableId": "collision_counterfactual_table"},
    ]
    artifact["manifest"]["blocks"] = (
        blocks[: summary_index + 1] + new_blocks + blocks[summary_index + 1 :]
    )

    source_entry = {
        "id": "impact_analysis",
        "label": "1131 三异常 Apollo 映射与碰撞反事实",
        "path": str(Path(__file__)),
        "query": {
            "engine": "duckdb",
            "language": "sql",
            "sql": (
                "SELECT CASE WHEN module IS NULL THEN metric_name ELSE module END AS module, "
                "duration_ms AS anomaly_ms, p50_ms, apollo_system_effect AS apollo_effect, "
                "collision_causal_status AS collision_status "
                f"FROM read_csv_auto('{TABLES / 'anomaly_apollo_impact_mapping.csv'}', header=true) "
                "ORDER BY module"
            ),
            "description": "读取三个选定异常的逐实例 trace 映射，展示执行时间、Apollo 下游影响与碰撞因果边界。",
            "tables_used": ["anomaly_apollo_impact_mapping.csv"],
            "metric_definitions": [
                "anomaly_ms：同一 Orin monotonic_ns 的组件起止端点差。",
                "collision_status：基于事件先后和 trace lineage 的证据状态，不是概率。",
            ],
        },
    }
    counterfactual_source = {
        "id": "counterfactual_analysis",
        "label": "1131 observed/model 分离的碰撞反事实",
        "path": str(TABLES / "anomaly_collision_counterfactual.csv"),
        "query": {
            "engine": "duckdb",
            "language": "sql",
            "sql": (
                "SELECT scenario, evidence_class, safety_metric, value, verdict "
                f"FROM read_csv_auto('{TABLES / 'anomaly_collision_counterfactual.csv'}', header=true) "
                "ORDER BY scenario"
            ),
            "description": "读取直接观测和模型预测分离保存的反事实情景；不使用模型值填补实际结果。",
            "tables_used": ["anomaly_collision_counterfactual.csv"],
            "metric_definitions": [
                "data/observed：本 run 直接测得或由一致端点推导。",
                "model/predicted：未完全验证的制动模型敏感性，不覆盖 observed 结果。",
            ],
        },
    }
    for container in (artifact["manifest"]["sources"], artifact["sources"]):
        container[:] = [
            source
            for source in container
            if source.get("id") not in {"impact_analysis", "counterfactual_analysis"}
        ]
        container.append(source_entry.copy())
        container.append(counterfactual_source.copy())
    path.write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    component_rows = read_csv(TABLES / "component_timing_all_instances.csv")
    distributions = read_csv(TABLES / "component_timing_distribution.csv")
    lineage_rows = read_csv(TABLES / "all_instance_lineage_timing.csv")
    observed = read_csv(TABLES / "run_level_observed.csv")[0]
    model_rows = read_csv(TABLES / "run_level_model_predicted.csv")

    t_sample = float(observed["t1_wall_s"])
    t_phys = float(observed["t2_wall_s"])
    collision = float(observed["t_outcome_wall_s"])
    d1_center = float(observed["D1_center_data_observed_m"])
    d1_clear = float(observed["D1_clear_data_observed_m"])
    d2_clear = float(observed["D2_clear_data_observed_m"])
    v2 = float(observed["v2_data_observed_mps"])
    impact_speed = float(observed["impact_speed_data_observed_mps"])

    actors = actor_trajectory()
    target_x = interpolate(actors, t_sample, 1) + d1_center
    geometry_offset = d1_center - d1_clear

    selected_specs = [
        (
            "Ground Detection",
            "ground_detection_processing",
            "72131690813719852",
            "17293896665878496501",
            "地图 ROI 点云",
            "/perception/lidar/pointcloud_ground_detection",
            "阻塞同 trace 的 Lidar Detection；其后 source→Fusion=705.980 ms，source→Control=731.829 ms。",
            "使下一轮障碍物检测、Fusion、Prediction、Planning 和 Control 刷新整体后移。",
        ),
        (
            "Lidar Detection",
            "lidar_detection_processing",
            "72131690813719851",
            "17293896665878496500",
            "/perception/lidar/pointcloud_ground_detection",
            "LiDAR 障碍物检测结果",
            "直接拉长对应 source→Fusion 到 699.268 ms，并形成 507.439 ms 的目标 Fusion 输出空洞。",
            "Prediction/Planning/Control 在空洞期间只能依赖此前目标状态和轨迹刷新。",
        ),
        (
            "Planning RunOnce",
            "planning_runonce",
            "17293896665878496499",
            "17293896665878496499",
            "Prediction 障碍物 11",
            "ADCTrajectory",
            "Prediction→Planning output=480.043 ms；新轨迹到 04.423057 才发布，首个 Control 到 04.429530。",
            "延迟新 STOP 轨迹刷新，但没有观测到制动命令被取消；此前 STOP/最大减速度 4 轨迹仍在先。",
        ),
    ]
    impact_rows: list[dict] = []
    for (
        module,
        metric,
        trace_id,
        downstream_trace,
        apollo_input,
        apollo_output,
        lineage_effect,
        system_effect,
    ) in selected_specs:
        instance = first(component_rows, metric_name=metric, trace_id=trace_id)
        distribution = first(distributions, metric_name=metric)
        start = float(instance["start_wall_s"])
        end = float(instance["end_wall_s"])
        duration = float(instance["duration_ms"])
        p50 = float(distribution["p50_ms"])
        impact_rows.append(
            {
                "run_id": RUN_ID,
                "module": module,
                "metric_name": metric,
                "trace_id": trace_id,
                "downstream_fusion_trace_id": downstream_trace,
                "start_wall_s": start,
                "end_wall_s": end,
                "duration_ms": duration,
                "p50_ms": p50,
                "local_excess_over_p50_ms": duration - p50,
                "best_case_local_finish_if_p50_wall_s": start + p50 / 1000.0,
                "starts_after_t_phys_ms": (start - t_phys) * 1000.0,
                "ends_before_collision_ms": (collision - end) * 1000.0,
                "apollo_input": apollo_input,
                "apollo_output": apollo_output,
                "trace_lineage_effect": lineage_effect,
                "apollo_system_effect": system_effect,
                "effect_on_initial_t_phys": "NONE_BY_EVENT_ORDER_POST_T_PHYS",
                "collision_causal_status": "LATE_DEGRADATION_SUPPORTED; COLLISION_PREVENTION_NOT_IDENTIFIABLE",
                "threshold_provenance": "RESEARCH median+6*MAD; not an architectural/calibrated deadline",
                "evidence_grade": "A_TO_CONTROL_FOR_LINEAGE; PHYSICAL_COUNTERFACTUAL_UNAVAILABLE",
            }
        )
    write_csv(TABLES / "anomaly_apollo_impact_mapping.csv", impact_rows)

    anomaly_start = min(float(row["start_wall_s"]) for row in impact_rows)
    anomaly_end = max(float(row["end_wall_s"]) for row in impact_rows)
    speed_start = interpolate(actors, anomaly_start, 2)
    speed_end = interpolate(actors, anomaly_end, 2)
    ego_x_start = interpolate(actors, anomaly_start, 1)
    clear_start = target_x - ego_x_start - geometry_offset
    required_decel = speed_start**2 / (2.0 * clear_start)
    distance_during_envelope = wall_speed_integral(actors, anomaly_start, anomaly_end)

    central = first(model_rows, parameter_set_id="BASELINE_EMPIRICAL_CENTRAL_DSAFE_0M")
    conservative = first(
        model_rows, parameter_set_id="BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_0M"
    )
    b_central = float(central["b_ego_model_predicted_mps2"])
    b_conservative = float(conservative["b_ego_model_predicted_mps2"])
    zero_delay_margin_central = d1_clear - float(
        central["required_distance_zero_delay_model_predicted_m"]
    )
    zero_delay_margin_conservative = d1_clear - float(
        conservative["required_distance_zero_delay_model_predicted_m"]
    )
    margin_tphys_central = d2_clear - v2**2 / (2.0 * b_central)
    margin_tphys_conservative = d2_clear - v2**2 / (2.0 * b_conservative)
    margin_start_central = clear_start - speed_start**2 / (2.0 * b_central)
    margin_start_planner = clear_start - speed_start**2 / (2.0 * 4.0)
    margin_start_conservative = clear_start - speed_start**2 / (2.0 * b_conservative)

    central_6m = first(model_rows, parameter_set_id="BASELINE_EMPIRICAL_CENTRAL_DSAFE_6M")
    conservative_6m = first(
        model_rows, parameter_set_id="BASELINE_EMPIRICAL_CONSERVATIVE_CANDIDATE_DSAFE_6M"
    )
    zero_delay_6m_central = d1_clear - float(
        central_6m["required_distance_zero_delay_model_predicted_m"]
    )
    zero_delay_6m_conservative = d1_clear - float(
        conservative_6m["required_distance_zero_delay_model_predicted_m"]
    )

    counterfactual_rows = [
        {
            "run_id": RUN_ID,
            "scenario": "删除这三个晚期异常：对首次 t_phys 的影响",
            "evidence_class": "data/observed event ordering",
            "safety_metric": "anomaly_start_minus_t_phys",
            "value": f"{(anomaly_start - t_phys) * 1000.0:.3f} ms",
            "verdict": "三者均晚于 t_phys，不能解释或提前本次首次物理制动端点。",
            "qualification": "DIRECT_ORDERING_BOUND",
        },
        {
            "run_id": RUN_ID,
            "scenario": "三个异常共同覆盖期间的实际车辆状态",
            "evidence_class": "data/observed",
            "safety_metric": "speed_and_wall_distance",
            "value": (
                f"v {speed_start:.3f}->{speed_end:.3f} m/s; "
                f"D_wall={distance_during_envelope:.3f} m"
            ),
            "verdict": "车辆持续减速；这段行驶距离是观测状态，不是可归因给异常的 deadline debt。",
            "qualification": "OBSERVED_NOT_ATTRIBUTABLE_LOSS",
        },
        {
            "run_id": RUN_ID,
            "scenario": "删除这三个晚期异常：最终是否无碰撞",
            "evidence_class": "data/observed counterfactual",
            "safety_metric": "collision_outcome_without_anomalies",
            "value": "UNAVAILABLE",
            "verdict": "NOT_IDENTIFIABLE：没有无异常 replay，且缺少 Control payload/Bridge apply。",
            "qualification": "COUNTERFACTUAL_NOT_OBSERVED",
        },
        {
            "run_id": RUN_ID,
            "scenario": "在 t_phys 立即按基线制动能力停车（0 m 接触边界）",
            "evidence_class": "model/predicted; unvalidated calibration",
            "safety_metric": "contact_stopping_margin",
            "value": (
                f"central={margin_tphys_central:.3f} m; "
                f"conservative={margin_tphys_conservative:.3f} m"
            ),
            "verdict": "两套模型均为负：到首次物理制动时已越过接触停止边界。",
            "qualification": "MODEL_SUPPORTED_ONLY",
        },
        {
            "run_id": RUN_ID,
            "scenario": "在最早异常开始时立即持续制动（0 m 接触边界）",
            "evidence_class": "observed state + model/predicted braking",
            "safety_metric": "required_decel_and_contact_margin",
            "value": (
                f"clear={clear_start:.3f} m; v={speed_start:.3f} m/s; "
                f"a_req={required_decel:.3f} m/s2; margins central/planner4/conservative="
                f"{margin_start_central:.3f}/{margin_start_planner:.3f}/{margin_start_conservative:.3f} m"
            ),
            "verdict": "到三异常开始时更不可能仅靠正常/降级制动避免接触。",
            "qualification": "MODEL_SUPPORTED_ONLY",
        },
        {
            "run_id": RUN_ID,
            "scenario": "不同反事实：删除 t_sample→t_phys 的全部初始响应距离",
            "evidence_class": "model/predicted; unvalidated calibration",
            "safety_metric": "zero_delay_contact_margin",
            "value": (
                f"central={zero_delay_margin_central:.3f} m; "
                f"conservative={zero_delay_margin_conservative:.3f} m"
            ),
            "verdict": "0 m 接触边界下可能避免碰撞，但这不是删除本次三个晚期异常的效果。",
            "qualification": "MODEL_SUPPORTED_ONLY_DIFFERENT_COUNTERFACTUAL",
        },
        {
            "run_id": RUN_ID,
            "scenario": "不同反事实：删除全部初始响应距离并要求 6 m 安全余量",
            "evidence_class": "model/predicted; research d_safe",
            "safety_metric": "zero_delay_6m_margin",
            "value": (
                f"central={zero_delay_6m_central:.3f} m; "
                f"conservative={zero_delay_6m_conservative:.3f} m"
            ),
            "verdict": "结论不稳健：central 为正、conservative 略为负。",
            "qualification": "MODEL_SUPPORTED_ONLY_BOUNDARY_SENSITIVE",
        },
    ]
    write_csv(TABLES / "anomaly_collision_counterfactual.csv", counterfactual_rows)

    report = f"""# 1131 run 三个晚期异常的 Apollo 系统影响与碰撞反事实

## 结论

这三个异常会在 Apollo 中造成**后期闭环刷新完整性退化**，但现有证据不支持把它们认定为首次制动过晚或最终碰撞的必要原因。

- 三者最早在 `t_phys` 后 **{(anomaly_start - t_phys) * 1000.0:.3f} ms** 才开始，故删除它们不会改变本run已经发生的首次物理制动端点。
- 异常前约 **{(anomaly_start - 1785123123.857026) * 1000.0:.3f} ms**，Planning 已输出 `stop by 11`、`trajectory_type=3`、`max_abs_decel=4` 的非空降级轨迹。异常中的 Planning 输出仍是相同停车语义。
- 三异常覆盖期间，CARLA actor速度从 **{speed_start:.3f} m/s** 降到 **{speed_end:.3f} m/s**，按墙钟梯形积分前进 **{distance_during_envelope:.3f} m**。这证明制动没有在该段消失；该距离不能直接称为异常造成的安全损失。
- “没有这三个异常是否安全”的直接数据答案是 **NOT_IDENTIFIABLE**：本run没有无异常重放，Control payload与逐命令Bridge apply也缺失。
- 模型诊断给出更强的方向性答案：到最早异常开始时，车辆仅余 **{clear_start:.3f} m** 净距、速度 **{speed_start:.3f} m/s**，瞬时停车需约 **{required_decel:.3f} m/s²**；三套制动假设下的0 m接触余量均为负。因此只消除这三个晚期尖峰，**碰撞仍很可能发生**。

## 三个异常如何映射到 Apollo

Apollo 10 的 LiDAR DAG 显示，Ground Detection 的输出 `/perception/lidar/pointcloud_ground_detection` 是 Lidar Detection 的输入，检测结果再进入跟踪/融合链；Planning 则由 Prediction 触发并发布供 Control 执行的轨迹。参见 [Apollo LiDAR Fusion DAG]({APOLLO_LIDAR_DAG_URL})、[Planning Component README]({APOLLO_PLANNING_README_URL}) 与 [Planning 架构输出说明]({APOLLO_PLANNING_ARCH_URL})。

| 异常 | 本地实例 | 映射到Apollo后的直接影响 | 对碰撞的证据结论 |
|---|---:|---|---|
| Ground Detection | 481.354 ms | 阻塞同trace的Lidar Detection；对应下一条`source→Fusion=705.980 ms`、`source→Control=731.829 ms`，整轮感知到控制刷新后移。 | 支持晚期刷新退化；不支持它改变首次`t_phys`。 |
| Lidar Detection | 507.315 ms | 目标检测结果晚到，使对应`source→Fusion=699.268 ms`，并在相邻Fusion目标输出之间形成507.439 ms空洞。 | Prediction/Planning看到的目标状态变旧，但此前制动轨迹已经存在。 |
| Planning RunOnce | 473.557 ms | `Prediction→Planning=480.043 ms`，新轨迹到04.423057才发布，首个Control到04.429530。 | 延迟轨迹刷新；但异常前后都输出`stop by 11/max_abs_decel=4`，未观测到制动被取消。 |

这三者是相邻trace上的并发异常，不是同一帧：Planning尖峰属于Fusion trace `...6499`，Lidar尖峰属于父trace `...9851`并进入Fusion `...6500`，Ground尖峰属于父trace `...9852`并进入Fusion `...6501`。

## 物理状态与碰撞影响

### 直接观测（data/observed）

1. `t_phys={t_phys:.6f}`；最早异常开始 `{anomaly_start:.6f}`；碰撞 `{collision:.6f}`，事件先后关系明确。
2. 异常开始时的净距按本报告既有的目标11纵向端点和几何扣除口径计算为 **{clear_start:.3f} m**。它不是CARLA actor中心欧氏距离，避免与主`D1_clear/D2_clear`口径混列。
3. 上一帧Planning在异常前已经产生停车轨迹；三异常期间车辆速度持续下降。
4. 碰撞及 **{impact_speed:.3f} m/s** 碰撞速度是直接观测；但“三异常造成多少物理安全损失”不可从单run分离，因为不存在同状态无异常轨迹。

所以，数据能够证明的是“约0.5 s的新鲜信息/轨迹刷新机会被推迟”，而不是“这0.5 s全部转化为额外制动起始延迟”或“造成了某个可直接相减的碰撞距离”。

### 模型诊断（model/predicted，不能覆盖实际结果）

| 状态/反事实 | central | conservative | 解释 |
|---|---:|---:|---|
| `t_phys`时0 m接触停车余量 | {margin_tphys_central:.3f} m | {margin_tphys_conservative:.3f} m | 均为负；模型认为三异常开始前已越界。 |
| 最早异常开始时0 m接触停车余量 | {margin_start_central:.3f} m | {margin_start_conservative:.3f} m | 均为负；规划降级4 m/s²余量为{margin_start_planner:.3f} m。 |
| 删除全部初始响应距离的0 m余量 | {zero_delay_margin_central:.3f} m | {zero_delay_margin_conservative:.3f} m | 均为正，但这是不同反事实。 |
| 删除全部初始响应距离的6 m余量 | {zero_delay_6m_central:.3f} m | {zero_delay_6m_conservative:.3f} m | central为正、conservative略负，不稳健。 |

模型的baseline与1131分离，但锁时无效且摩擦、坡度、曲率、载荷和制动建立时间域未验证，因此只能写作 `MODEL_SUPPORTED_ONLY`。它支持“仅删除晚期三异常不足以避免碰撞”的方向性判断，不能冒充无异常重放结果。

## 正确的反事实结论

- **仅删除这三个晚期异常**：直接结果不可识别；结合事件顺序、先前STOP轨迹和制动包络，碰撞仍很可能发生。
- **删除从`t_sample`到`t_phys`的全部初始响应延迟**：0 m接触边界下，两套模型均显示可能避免碰撞，但这与本次三个晚期尖峰不是同一个干预。
- **要求碰撞前还保留6 m余量**：即便删除全部初始响应距离，模型结论也对制动能力敏感，不能宣称稳健安全。

## 证据限制

- 无Apollo record、Control payload、逐命令Bridge receive/release/apply与Chassis反馈，无法判定每个Control动作episode及轨迹复用的精确物理效果。
- 无同初始状态“去掉三个异常”的回放，不能直接估计避免碰撞概率或异常可归因距离。
- `median+6×MAD`是research筛查阈值，不是Apollo architectural deadline或独立校准的物理deadline。
- 本报告只回答异常映射和碰撞影响，不判断三异常的共同根因。

## 复现与数据

- 映射表：`tables/anomaly_apollo_impact_mapping.csv`
- observed/model分离反事实：`tables/anomaly_collision_counterfactual.csv`
- 重绘散点图：`figures/component_timing_scatter_all_instances.png`
- 复现脚本：`scripts/analyze_1131_anomaly_collision_impact_v4_2.py`

Apollo模块关系也可从[官方核心模块架构说明]({APOLLO_ARCHITECTURE_URL})核对。
"""
    REPORT.mkdir(parents=True, exist_ok=True)
    (REPORT / "anomaly_collision_impact_report.md").write_text(report, encoding="utf-8")

    update_claim_evidence(
        anomaly_start,
        anomaly_end,
        t_phys,
        collision,
        speed_start,
        speed_end,
        clear_start,
        required_decel,
    )
    update_artifact(impact_rows, counterfactual_rows)

    validation = {
        "run_id": RUN_ID,
        "status": "PASS",
        "checks": {
            "all_three_anomalies_after_t_phys": all(
                float(row["start_wall_s"]) > t_phys for row in impact_rows
            ),
            "all_three_anomalies_before_collision": all(
                float(row["end_wall_s"]) < collision for row in impact_rows
            ),
            "speed_decreased_during_anomaly_envelope": speed_end < speed_start,
            "observed_counterfactual_not_fabricated": any(
                row["value"] == "UNAVAILABLE" for row in counterfactual_rows
            ),
            "observed_model_results_separated": all(
                "observed" in row["evidence_class"] or "model" in row["evidence_class"]
                for row in counterfactual_rows
            ),
            "consistent_clearance_endpoint_at_tphys": abs(
                (target_x - interpolate(actors, t_phys, 1) - geometry_offset)
                - float(observed["D2_clear_direct_diagnostic_m"])
            )
            < 0.02,
            "model_contact_margins_negative_before_anomalies": (
                margin_tphys_central < 0
                and margin_tphys_conservative < 0
                and margin_start_central < 0
                and margin_start_conservative < 0
            ),
        },
        "key_values": {
            "anomaly_start_minus_t_phys_ms": (anomaly_start - t_phys) * 1000.0,
            "speed_start_mps": speed_start,
            "speed_end_mps": speed_end,
            "distance_during_anomaly_envelope_wall_integral_m": distance_during_envelope,
            "clearance_at_anomaly_start_m": clear_start,
            "required_instantaneous_decel_mps2": required_decel,
            "impact_speed_observed_mps": impact_speed,
        },
    }
    if not all(validation["checks"].values()):
        validation["status"] = "FAIL"
    validation_path = OUTPUT / "validation/anomaly_collision_impact_validation.json"
    validation_path.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if validation["status"] != "PASS":
        raise RuntimeError(f"impact validation failed: {validation['checks']}")

    print(
        json.dumps(
            {
                "status": "PASS",
                "report": str(REPORT / "anomaly_collision_impact_report.md"),
                "mapping_table": str(TABLES / "anomaly_apollo_impact_mapping.csv"),
                "counterfactual_table": str(TABLES / "anomaly_collision_counterfactual.csv"),
                "anomaly_start_minus_t_phys_ms": (anomaly_start - t_phys) * 1000.0,
                "clearance_at_anomaly_start_m": clear_start,
                "required_decel_mps2": required_decel,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
