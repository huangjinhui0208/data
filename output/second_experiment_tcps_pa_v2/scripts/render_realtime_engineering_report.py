#!/usr/bin/env python3
"""Render the comprehensive real-time-systems TCPS-PA v2 report.

The report remains a view of the Claim/Evidence/Defeater ledgers.  It adds the
engineering detail needed for a first method instantiation without upgrading
claims when the underlying evidence is missing, retrospective, or model-only.
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import render_tcps_pa_v2_report as core


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
FIGURES = ROOT / "figures"
REPORT = ROOT / "report" / "six_layer_analysis_report.md"


GATE_SPECS = {
    "C1": {
        "inputs": "Temporal Fault Signature、Bridge/SCB applied rows、P_CLOCK.fault_signature",
        "metrics": "location、onset、duration、requested/actual magnitude、pattern、scope、affected messages",
        "evidence": "DIRECT_OBSERVED Bridge/SCB applied-delay evidence",
        "criterion": "实际应用证据存在，且位置、onset、幅值、pattern 与作用范围足以界定干预；未知 duration/message/drop/reorder 继续作为完整性限制",
        "next": "C1 至少 PARTIAL_PASS，且 L2 有已声明 reference",
    },
    "C2": {
        "inputs": "C1、baseline distribution、stage_timing_and_freshness、clock/phase audit",
        "metrics": "R（反应/响应变化）、A（新鲜度/数据年龄）、G（连续性/update gap）的 run-level 分布",
        "evidence": "OBSERVED_DERIVED，且 reference_type 与 distribution_scope 明确",
        "criterion": "相对 BASELINE_DISTRIBUTION 比较 P50/P90/P95/P99/MAX/IQR；缺 reference、样本覆盖或 phase 审计时不得强 PASS",
        "next": "C2 至少 PARTIAL_PASS，P_CLOCK/P_TARGET 足以支持声明的 lineage 等级",
    },
    "C3": {
        "inputs": "C1、C2、P_CLOCK、P_TARGET、event timeline、trace/provenance",
        "metrics": "T_R=t2-t1、source→Fusion→Prediction→Planning→Control→Bridge→physical response lineage grade",
        "evidence": "A/B 级 TRACE_LINEAGE 可支持强结论；C 级时间对齐只支持系统级关联",
        "criterion": "A/B 且前置条件闭合可 PASS；C 级最多 PARTIAL_PASS；D/UNKNOWN 进一步降级",
        "next": "C3 与 P_DEADLINE 均满足，才能进入主 C4 比较",
    },
    "C4": {
        "inputs": "观测 T_R、同 scope 合格 tau_req、P_CLOCK、P_TARGET、P_DEADLINE",
        "metrics": "S_T=tau_req-T_R，tau_req_low/center/high",
        "evidence": "OBSERVED_DERIVED physical T_R + INDEPENDENT_REQUIREMENT/独立验证 envelope",
        "criterion": "T_R>tau_req_high 为 CLEARLY_MISSED；位于区间内为 BOUNDARY_UNCERTAIN；只有 tau_retro/tau_model 时主 C4 NOT_TESTABLE",
        "next": "C4 有合格判定，且存在兼容墙钟速度路径",
    },
    "C5": {
        "inputs": "C4、tau_req、观测速度路径、D1、D_brake、D_safe",
        "metrics": "D_response、D_debt、M_D、ΔM_D=ΔD1-ΔD_response-ΔD_brake",
        "evidence": "REQUIREMENT_CONSTRAINED_DERIVED 主 debt；retro/model debt 保留来源标记",
        "criterion": "只有合格 C4 + 观测速度积分 + qualified debt 才可 PASS；模型或事后 deadline 只能支持模型/重建机制",
        "next": "C5 与直接物理 C6 证据共同进入 C7",
    },
    "C6": {
        "inputs": "直接碰撞/停车事件、净距、margin、minimum speed、impact、截断制动",
        "metrics": "D2、M0/M6、minimum/final clearance、impact speed/impulse、outcome severity",
        "evidence": "DIRECT_OBSERVED 且 semantic_role=PHYSICAL_OUTCOME",
        "criterion": "直接物理裕度或结局发生实验内退化；任何 critical/near/high-severity 分类必须有预先来源",
        "next": "C6 只证明物理结果；与 C4/C5/P_FUNC/defeaters 一并进入 C7",
    },
    "C7": {
        "inputs": "C1-C6、P_FUNC、全部关键 defeater、taint 与 confidence ceiling",
        "metrics": "weakest-link level、开放关键反证项、重复性与 dose-response",
        "evidence": "由前置 Claim Graph 组合，不由单一碰撞或单一 deadline 指标替代",
        "criterion": "取关键前置项、证据类别和开放反证项的最低上限；外部 Bridge 注入不能支持 Apollo 内部缺陷判断",
        "next": "输出允许语言、最高声明等级和下一轮证据需求",
    },
}

ALLOWED_ZH = {
    "C1": "外部注入的时间扰动在本地时钟域内得到直接核验。",
    "C2": "观测 T_R 与 gap 指标可用于刻画可能的时序退化，但不能越过 reference、clock 与 phase 限制。",
    "C3": "现有证据支持系统级时间关联；严格 cause-effect lineage 仍不完整。",
    "C4": "事后重建与模型证据只用于敏感性分析，主时间正确性判定保持不可检验。",
    "C5": "观测响应距离与模型结果支持一种可能的空间传播机制，主观测距离债务尚未建立。",
    "C6": "安全停车、碰撞和物理裕度属于直接观测结果；该层不单独分配时间归因。",
    "C7": "数据支持系统级时间关联与模型支持的传播机制，最高声明等级为第 2 级。",
}


RAG_METRICS = [
    ("R", "物理反应时间 T_R", "T_e2e_data_observed_ms", "ms", "t1→t2 单墙钟物理区间"),
    ("R", "Sensor→Control", "sensor_to_control_ms", "ms", "阶段诊断，跨主机置信度受限"),
    ("R", "Control→t2", "control_to_t2_ms", "ms", "Control 后至持续制动起点"),
    ("A", "t2 时目标数据年龄", "data_age_target_at_t2_data_observed_ms", "ms", "目标 header/source 到物理 t2"),
    ("A", "目标 lifecycle P90", "target_lifecycle_response_window_p90_data_observed_ms", "ms", "run 内 response window 摘要"),
    ("A", "结局时目标 source age", "target_source_age_at_outcome_ms", "ms", "诊断量；结局端点与匹配语义需保守解释"),
    ("G", "响应窗 target gap MAX", "update_gap_target_response_window_max_data_observed_ms", "ms", "少于两个输出时不可用"),
    ("G", "全窗 target gap P90", "target_gap_p90_ms", "ms", "run 内 target 输出间隔摘要"),
    ("G", "全窗 target gap MAX", "target_gap_max_ms", "ms", "单个 MAX 不能单独建立组级退化"),
]


def clean_cell(value: object) -> str:
    return str(value).replace("|", "/").replace("\n", " ").strip()


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    head = "| " + " | ".join(headers) + " |"
    rule = "|" + "|".join("---" for _ in headers) + "|"
    body = ["| " + " | ".join(clean_cell(value) for value in row) + " |" for row in rows]
    return "\n".join([head, rule, *body])


def number(value: object, digits: int = 3) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "—"
    return "—" if math.isnan(numeric) else f"{numeric:.{digits}f}"


def enrich_claim_ledger(claims: pd.DataFrame) -> pd.DataFrame:
    enriched = claims.copy()
    for column in ["gate_inputs", "gate_metrics", "admissible_evidence", "next_gate_condition"]:
        if column not in enriched.columns:
            enriched[column] = ""
    for index, row in enriched.iterrows():
        base = core.claim_base(str(row["claim_id"]))
        if base not in GATE_SPECS:
            continue
        spec = GATE_SPECS[base]
        enriched.at[index, "gate_inputs"] = spec["inputs"]
        enriched.at[index, "gate_metrics"] = spec["metrics"]
        enriched.at[index, "admissible_evidence"] = spec["evidence"]
        enriched.at[index, "gate_criterion"] = spec["criterion"]
        enriched.at[index, "next_gate_condition"] = spec["next"]
    enriched.to_csv(TABLES / "claim_ledger.csv", index=False, encoding="utf-8-sig")
    return enriched


def build_rag_summary(stage: pd.DataFrame, observed: pd.DataFrame) -> pd.DataFrame:
    included = set(observed.loc[observed["is_main"], "run_id"])
    frame = stage[stage["run_id"].isin(included)].copy()
    rows: list[dict[str, object]] = []
    for dimension, metric_name, column, unit, semantics in RAG_METRICS:
        for group_name in ["baseline", "delay_300ms"]:
            group = frame[frame["group_name"] == group_name]
            values = pd.to_numeric(group[column], errors="coerce").dropna()
            row: dict[str, object] = {
                "dimension": dimension,
                "metric": metric_name,
                "source_column": column,
                "unit": unit,
                "semantics": semantics,
                "group_name": group_name,
                "n_total_runs": len(group),
                "n_available_runs": len(values),
            }
            for label, quantile in [("p50", .50), ("p90", .90), ("p95", .95), ("p99", .99)]:
                row[label] = values.quantile(quantile) if len(values) else math.nan
            row["max"] = values.max() if len(values) else math.nan
            row["iqr"] = values.quantile(.75) - values.quantile(.25) if len(values) else math.nan
            rows.append(row)
    summary = pd.DataFrame(rows)
    summary.to_csv(TABLES / "realtime_rag_summary.csv", index=False, encoding="utf-8-sig")
    return summary


def build_space_budget(observed: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    frame = observed.copy()
    frame["D_safe_collision_boundary_m"] = 0.0
    frame["M0_recomputed_observed_m"] = (
        frame["D1_clear_data_observed_m"]
        - frame["D_response_wall_integral_data_observed_m"]
        - frame["D_brake_data_observed_m"]
    )
    frame["M6_recomputed_observed_m"] = frame["M0_recomputed_observed_m"] - 6.0
    frame["endpoint_compatible_full_stop"] = (
        frame["is_main"]
        &
        frame["D_brake_data_observed_m"].notna()
        & ~core.truthy(frame["collision_event_data_observed"])
    )
    frame["decomposition_scope"] = np.select(
        [
            ~frame["is_main"],
            frame["endpoint_compatible_full_stop"],
        ],
        [
            "EXCLUDED_OUTCOME_CONFLICT",
            "FULL_STOP_OBSERVED",
        ],
        default="UNAVAILABLE_OUTCOME_TRUNCATED_OR_CONFLICTED",
    )
    columns = [
        "run_id", "group_name", "included_main_analysis", "outcome_data_observed",
        "D1_clear_data_observed_m", "D_response_wall_integral_data_observed_m",
        "D_brake_data_observed_m", "M0_recomputed_observed_m",
        "M6_recomputed_observed_m", "endpoint_compatible_full_stop", "decomposition_scope",
    ]
    run_level = frame[columns]
    run_level.to_csv(TABLES / "space_budget_decomposition_observed.csv", index=False, encoding="utf-8-sig")

    main_full = frame[frame["is_main"] & frame["endpoint_compatible_full_stop"]].copy()
    rows = []
    for key, group in [
        ("baseline_full_stop", main_full[main_full["group_name"] == "baseline"]),
        ("delay_300ms_safe_stop", main_full[main_full["group_name"] == "delay_300ms"]),
    ]:
        rows.append({
            "comparison_group": key,
            "n": len(group),
            "D1_mean_m": group["D1_clear_data_observed_m"].mean(),
            "D_response_mean_m": group["D_response_wall_integral_data_observed_m"].mean(),
            "D_brake_mean_m": group["D_brake_data_observed_m"].mean(),
            "M0_mean_m": group["M0_recomputed_observed_m"].mean(),
        })
    group_budget = pd.DataFrame(rows)
    if len(group_budget) == 2:
        delta = {
            "comparison_group": "delay_safe_minus_baseline",
            "n": f"{int(group_budget.loc[1, 'n'])} vs {int(group_budget.loc[0, 'n'])}",
        }
        for column in ["D1_mean_m", "D_response_mean_m", "D_brake_mean_m", "M0_mean_m"]:
            delta[column] = group_budget.loc[1, column] - group_budget.loc[0, column]
        group_budget = pd.concat([group_budget, pd.DataFrame([delta])], ignore_index=True)
    group_budget.to_csv(TABLES / "space_budget_group_decomposition.csv", index=False, encoding="utf-8-sig")
    return run_level, group_budget


def build_method_completeness() -> pd.DataFrame:
    rows = [
        ("证据/模型分离", "已完成", "observed、retrospective、requirement、model 分列且 taint 传播"),
        ("Claim Graph 与六层 Gate", "已完成（结构）", "P_CLOCK/P_TARGET/P_FUNC/P_DEADLINE、C1-C7 与 canonical edges 已建立"),
        ("Gate 可执行判据", "本版补齐", "输入、指标、证据类别、criterion 与下一层条件已写入 Claim Ledger/报告"),
        ("Temporal Fault Signature", "部分完成", "onset/幅值/pattern/scope 可用；end/duration/message count/drop/reorder 未闭合"),
        ("R/A/G 与 tail latency", "部分完成", "run-level P50/P90/P95/P99/MAX 已汇总；record message-level MRT/MDA 不可用"),
        ("严格 cause-effect lineage", "未完成", "当前 grade C；缺 event/sequence/provenance 绑定"),
        ("独立 tau_req", "未完成", "只有 tau_retro 与未验证 tau_model"),
        ("主观测 Distance Debt", "未完成", "没有 REQUIREMENT_CONSTRAINED_DERIVED debt"),
        ("空间预算分解", "部分完成", "完整安全停车 run 可分解；碰撞 run 被右删失"),
        ("连续物理安全尺度", "部分完成", "D2/M0/M6/impact 可用；near/critical taxonomy 缺 threshold provenance"),
        ("Functional Correctness", "部分完成", "审计表已建但 P_FUNC 全部 PARTIAL，Control/Prediction 等多项 UNKNOWN"),
        ("Clock/Phase uncertainty", "未完成", "多数 run 无 offset/drift/resolution；12/12 未 phase scan"),
        ("Pre-hazard state divergence", "部分完成", "D1/V1 标为 possible mediator，但 fault-onset state/delta 缺失"),
        ("论文级方法有效性", "未完成", "缺多非零等级、独立验证数据、record-enabled lineage、负对照与跨系统验证"),
    ]
    frame = pd.DataFrame(rows, columns=["requirement", "status", "evidence_or_gap"])
    frame.to_csv(TABLES / "method_completeness_matrix.csv", index=False, encoding="utf-8-sig")
    return frame


def render_new_figures(stage: pd.DataFrame, observed: pd.DataFrame, group_budget: pd.DataFrame) -> None:
    plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "PingFang SC", "Heiti TC", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    included = set(observed.loc[observed["is_main"], "run_id"])
    data = stage[stage["run_id"].isin(included)].copy()
    panels = [
        ("R：物理反应时间", "T_e2e_data_observed_ms"),
        ("A：t2 时目标数据年龄", "data_age_target_at_t2_data_observed_ms"),
        ("G：响应窗 target gap MAX", "update_gap_target_response_window_max_data_observed_ms"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    rng = np.random.default_rng(20260811)
    for ax, (title, column) in zip(axes, panels):
        for x, group_name, color in [(0, "baseline", "#4C78A8"), (1, "delay_300ms", "#F28E2B")]:
            values = pd.to_numeric(data.loc[data["group_name"] == group_name, column], errors="coerce").dropna()
            jitter = rng.uniform(-.08, .08, len(values))
            ax.scatter(np.full(len(values), x) + jitter, values, s=46, alpha=.85, color=color)
            if len(values):
                ax.hlines(values.median(), x - .18, x + .18, color="#222222", linewidth=2.2)
        ax.set_xticks([0, 1], ["baseline", "300 ms"])
        ax.set_title(title)
        ax.set_ylabel("ms")
        ax.grid(axis="y", alpha=.25)
    fig.suptitle("实时性 R/A/G 的逐 run 分布（横线为中位数；缺失值不连线）", fontsize=14, weight="bold")
    fig.tight_layout()
    fig.savefig(FIGURES / "realtime_rag_run_distributions.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    if len(group_budget) >= 2:
        plot_data = group_budget.iloc[:2]
        components = ["D1_mean_m", "D_response_mean_m", "D_brake_mean_m", "M0_mean_m"]
        labels = ["D1", "D_response", "D_brake", "M0"]
        x = np.arange(len(components))
        width = .36
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.bar(x - width/2, plot_data.iloc[0][components].astype(float), width, label="baseline full stop", color="#4C78A8")
        ax.bar(x + width/2, plot_data.iloc[1][components].astype(float), width, label="300 ms safe stop", color="#F28E2B")
        ax.set_xticks(x, labels)
        ax.set_ylabel("组均值（m）")
        ax.set_title("观测空间预算分解（仅端点兼容的完整停车 run）")
        ax.axhline(0, color="#333333", linewidth=.8)
        ax.grid(axis="y", alpha=.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(FIGURES / "space_budget_decomposition_observed.png", dpi=180, bbox_inches="tight")
        plt.close(fig)


def gate_detail(claim: pd.Series, support: str, challenge: str) -> str:
    base = core.claim_base(str(claim["claim_id"]))
    spec = GATE_SPECS[base]
    prereqs = core.split_ids(claim["prerequisite_claim_ids"])
    support_ids = core.split_ids(claim["supporting_evidence_ids"])
    defeaters = core.split_ids(claim["defeater_ids"])
    preview = ", ".join(support_ids[:4])
    if len(support_ids) > 4:
        preview += f"，另有 {len(support_ids)-4} 条"
    return f"""
- **科学命题：** {core.CLAIM_ZH[base]}
- **输入：** {spec['inputs']}。
- **核心指标：** {spec['metrics']}。
- **必要前提：** {', '.join(prereqs) if prereqs else '无前置 Claim'}。
- **可接受证据：** {spec['evidence']}。
- **判定标准：** {spec['criterion']}。
- **当前支持：** {support} 证据链接：{preview or '由前置 Claim 限定'}。
- **反向证据/限制：** {challenge}
- **未闭合反证项：** {len(defeaters)} 项。
- **规则与输出：** `{claim['inference_rule_id']}` → `{claim['verdict']}`；置信度 `{claim['confidence']}`，上限 `{claim['confidence_ceiling']}`，最高声明等级 `{claim['maximum_claim_level']}`。
- **进入下一层的条件：** {spec['next']}。
- **允许结论：** {ALLOWED_ZH[base]}
- **仍未证明：** {core.RESIDUAL_ZH[base]}
""".strip()


def main() -> None:
    observed = pd.read_csv(TABLES / "run_level_observed.csv", dtype={"run_id": str})
    observed["is_main"] = core.truthy(observed["included_main_analysis"])
    stage = pd.read_csv(TABLES / "stage_timing_and_freshness.csv", dtype={"run_id": str})
    model = pd.read_csv(TABLES / "run_level_model_predicted.csv", dtype={"run_id": str})
    claims = enrich_claim_ledger(pd.read_csv(TABLES / "claim_ledger.csv").fillna(""))
    defeaters = pd.read_csv(TABLES / "defeater_ledger.csv").fillna("")
    fault = pd.read_csv(TABLES / "temporal_fault_signature.csv", dtype={"run_id": str}).fillna("")
    functional = pd.read_csv(TABLES / "functional_correctness_audit.csv", dtype={"run_id": str}).fillna("")
    clocks = pd.read_csv(TABLES / "clock_phase_audit.csv", dtype={"run_id_or_group": str}).fillna("")
    requirements = pd.read_csv(TABLES / "requirement_registry.csv", dtype={"run_id_or_group": str}).fillna("")
    prehazard = pd.read_csv(TABLES / "pre_hazard_state_audit.csv", dtype={"run_id": str}).fillna("")

    claim_by_base = {core.claim_base(row.claim_id): row for _, row in claims.iterrows()}
    expected = {
        "C1": "PASS", "C2": "PARTIAL_PASS", "C3": "PARTIAL_PASS",
        "C4": "NOT_TESTABLE", "C5": "MODEL_SUPPORTED_ONLY",
        "C6": "PASS", "C7": "PARTIAL_PASS",
    }
    if {base: claim_by_base[base]["verdict"] for base in expected} != expected:
        raise RuntimeError("Claim Ledger verdicts changed; review report mappings before rendering")

    rag = build_rag_summary(stage, observed)
    _, group_budget = build_space_budget(observed)
    completeness = build_method_completeness()
    core.render_claim_status_figure(claim_by_base)
    render_new_figures(stage, observed, group_budget)

    main_runs = observed[observed["is_main"]]
    baseline = main_runs[main_runs["group_name"] == "baseline"]
    delay = main_runs[main_runs["group_name"] == "delay_300ms"]
    tr0, tr1 = baseline["T_e2e_data_observed_ms"].median(), delay["T_e2e_data_observed_ms"].median()
    dr0 = baseline["D_response_wall_integral_data_observed_m"].median()
    dr1 = delay["D_response_wall_integral_data_observed_m"].median()
    m00 = baseline["M_collision_0m_data_observed_m"].median()
    m01 = delay["M_collision_0m_data_observed_m"].median()

    matrix_specs = [
        ("L1 / C1", "C1", "直接观测", "外部 Bridge/SCB 干预已核验；完整 signature 仍有限制"),
        ("L2 / C2", "C2", "观测派生", "T_R 相对 baseline 右移；A/G 与 phase 证据部分"),
        ("L3 / C3", "C3", "关联等级 C", "系统级时间关联，缺显式 lineage"),
        ("L4 / C4", "C4", "事后重建 + 未验证模型", "独立 tau_req 缺失"),
        ("L5 / C5", "C5", "模型来源标记", "D_response 为观测；主 debt 不可用"),
        ("L6 / C6", "C6", "直接物理观测", "净距、margin、停车与碰撞结局"),
        ("Attribution / C7", "C7", "最弱环节", "第 2 级：系统级关联与模型机制"),
    ]
    matrix_rows = []
    for label, base, basis, allowed in matrix_specs:
        claim = claim_by_base[base]
        local = defeaters[defeaters["claim_id"] == claim["claim_id"]]
        critical = local[
            local["status"].isin(["OPEN", "UNKNOWN"])
            & local["impact_on_claim"].isin(["CRITICAL", "INVALIDATES"])
        ]
        ceiling = claim["confidence_ceiling"]
        if base == "C7":
            ceiling = f"第 {int(float(claim['maximum_claim_level']))} 级 / {ceiling}"
        matrix_rows.append([label, claim["verdict"], basis, claim["confidence"], ceiling, len(critical), allowed])
    matrix = markdown_table(
        ["层级/Claim", "判定", "证据基础", "置信度", "上限", "未闭合关键反证项", "允许结论"],
        matrix_rows,
    )

    gate_rows = []
    for label, base, _basis, _allowed in matrix_specs:
        spec = GATE_SPECS[base]
        claim = claim_by_base[base]
        gate_rows.append([label, spec["inputs"], spec["metrics"], spec["criterion"], claim["verdict"], spec["next"]])
    gate_matrix = markdown_table(["Gate", "输入", "指标", "判据", "本实验输出", "进入下一步条件"], gate_rows)

    method_table = markdown_table(
        ["内容", "完成状态", "当前证据或缺口"],
        completeness[["requirement", "status", "evidence_or_gap"]].values.tolist(),
    )

    fault = fault.merge(observed[["run_id", "group_name", "is_main"]], on="run_id", how="left")
    fault_rows = []
    for group_name in ["baseline", "delay_300ms"]:
        group_fault = fault[fault["group_name"] == group_name]
        actual = pd.to_numeric(group_fault["actual_magnitude"], errors="coerce")
        trigger = pd.to_numeric(group_fault["trigger_relative_t1_s"], errors="coerce")
        fault_rows.append([
            group_name, len(group_fault), number(actual.median()), number(actual.quantile(.90)), number(actual.max()),
            f"{number(trigger.min())} 至 {number(trigger.max())}",
            int((group_fault["fault_end_wall"] == "").sum()),
            int((group_fault["affected_message_count"] == "UNKNOWN").sum()),
            "PERSISTENT_SETTING / REPEATED_PER_AFFECTED_MESSAGE",
        ])
    fault_table = markdown_table(
        ["分组", "n", "实际延时 P50(ms)", "P90", "MAX", "onset 相对 t1(s)", "end 缺失", "消息数未知", "pattern"],
        fault_rows,
    )

    rag_rows = []
    for dimension, metric_name, _column, unit, semantics in RAG_METRICS:
        subset = rag[(rag["dimension"] == dimension) & (rag["metric"] == metric_name)]
        cells = []
        for group_name in ["baseline", "delay_300ms"]:
            row = subset[subset["group_name"] == group_name].iloc[0]
            cells.append(
                f"{number(row.p50)}/{number(row.p90)}/{number(row.p95)}/{number(row.p99)}/{number(row['max'])} "
                f"(n={int(row.n_available_runs)}/{int(row.n_total_runs)})"
            )
        rag_rows.append([dimension, metric_name, unit, cells[0], cells[1], semantics])
    rag_table = markdown_table(
        ["维度", "指标", "单位", "baseline P50/P90/P95/P99/MAX", "300 ms P50/P90/P95/P99/MAX", "语义/限制"],
        rag_rows,
    )

    function_fields = [
        ("物理目标身份", "physical_target_identity"),
        ("Perception 目标存在", "perception_target_present"),
        ("Perception 连续追踪", "perception_tracking_continuity"),
        ("Prediction 目标/语义", "prediction_semantics_valid"),
        ("Planning STOP", "planning_stop_present"),
        ("Planning 目标/位置", "planning_stop_target_correct"),
        ("Planning 轨迹", "planning_trajectory_valid"),
        ("Planning fallback", "planning_fallback_status"),
        ("Control 相关轨迹", "control_received_relevant_trajectory"),
        ("Control 制动命令", "control_braking_command_present"),
        ("Control 命令连续性", "control_command_continuity"),
        ("Bridge payload receive", "bridge_payload_received"),
        ("Bridge payload apply", "bridge_payload_applied"),
        ("物理响应", "physical_response_observed"),
    ]
    function_rows = []
    for label, column in function_fields:
        counts = functional[column].value_counts().to_dict()
        function_rows.append([label, counts.get("PASS", 0), counts.get("DEGRADED", 0), counts.get("PARTIAL", 0), counts.get("UNKNOWN", 0)])
    function_table = markdown_table(["功能链项目", "PASS", "DEGRADED", "PARTIAL", "UNKNOWN"], function_rows)

    pre_rows = []
    for variable in ["D1", "V1", "A1", "HEADING", "ROUTE_PROGRESS"]:
        subset = prehazard[prehazard["state_variable"] == variable]
        pre_rows.append([
            variable,
            int((subset["value_at_fault"] != "").sum()),
            int((subset["value_at_t1"] != "").sum()),
            int((subset["delta"] != "").sum()),
            ", ".join(f"{key}:{value}" for key, value in subset["causal_role"].value_counts().to_dict().items()),
        ])
    pre_table = markdown_table(["状态变量", "fault 时可用", "t1 时可用", "delta 可用", "当前因果角色"], pre_rows)

    req_rows = [
        ["tau_retro", int((requirements["evidence_class"] == "RETROSPECTIVE_RECONSTRUCTION").sum()), "事后重建", "否", "重建/敏感性"],
        ["tau_model", int((requirements["evidence_class"] == "UNVALIDATED_MODEL").sum()), "样本内常减速度模型", "否", "模型机制"],
        ["tau_req", int(requirements["evidence_class"].isin(["INDEPENDENT_REQUIREMENT", "INDEPENDENT_CALIBRATED_MODEL", "VALIDATED_MODEL"]).sum()), "独立前瞻要求", "当前无", "主 C4"],
    ]
    req_table = markdown_table(["deadline 类别", "登记条数", "来源", "主 C4 合格", "用途"], req_rows)

    budget_rows = []
    for row in group_budget.itertuples():
        budget_rows.append([
            row.comparison_group, row.n, number(row.D1_mean_m), number(row.D_response_mean_m),
            number(row.D_brake_mean_m), number(row.M0_mean_m),
        ])
    budget_table = markdown_table(
        ["比较组", "n", "D1 mean(m)", "D_response mean(m)", "D_brake mean(m)", "M0 mean(m)"],
        budget_rows,
    )

    full_stop_model = model[model["run_id"].isin(main_runs["run_id"]) & model["D_brake_data_observed_comparator_m"].notna()]
    collision_model = model[model["run_id"].isin(main_runs["run_id"]) & model["outcome_data_observed_comparator"].eq("collision")]
    delay_model = model[model["run_id"].isin(delay["run_id"])]
    pred_collision = core.truthy(delay_model["collision_model_predicted"])
    false_positive = int((pred_collision & delay_model["outcome_data_observed_comparator"].eq("safe_stop")).sum())

    run_rows = []
    for row in observed.itertuples():
        run_rows.append([
            row.run_id, row.group_name, "主分析" if row.is_main else "排除：结局冲突",
            number(row.T_e2e_data_observed_ms), number(row.D_response_wall_integral_data_observed_m),
            number(row.D1_clear_data_observed_m), number(row.D2_clear_data_observed_m),
            number(row.M_collision_0m_data_observed_m),
            core.OUTCOME_ZH.get(row.outcome_data_observed, row.outcome_data_observed),
            number(row.impact_speed_data_observed_mps),
        ])
    run_table = markdown_table(
        ["run", "分组", "范围", "T_R(ms)", "D_response(m)", "D1(m)", "D2(m)", "M0(m)", "结局", "碰撞速度(m/s)"],
        run_rows,
    )

    c7_rows = []
    for row in defeaters[defeaters["claim_id"] == "C7.all_runs"].itertuples():
        c7_rows.append([f"`{row.defeater_id}`", row.status, row.impact_on_claim, core.defeater_description_zh(row.defeater_id)])
    c7_table = markdown_table(["反证项", "状态", "影响", "未闭合问题"], c7_rows)

    l1 = gate_detail(
        claim_by_base["C1"],
        "12/12 run 有 requested/actual applied delay；300 ms 组实际延时稳定在约 300 ms；onset 均早于 t1",
        "fault_end、实际 duration、affected message count、drop/reorder 未建立；baseline run 1031 有 19.282 ms worst-observed 实际延时",
    )
    l2 = gate_detail(
        claim_by_base["C2"],
        f"T_R 中位数 {tr0:.3f}→{tr1:.3f} ms；Control→t2 是主要描述性增量；R/A/G 表保留逐 run tail 与可用计数",
        "record message-level MRT/MDA 缺失；G 响应窗 baseline 仅 2/7 可用；phase 未扫描；P95/P99 在 n=4/7 下只代表经验插值",
    )
    l3 = gate_detail(
        claim_by_base["C3"],
        "t1/t2 单墙钟物理区间和模块事件时间线可用，形成 C 级时间对齐",
        "缺 source/Fusion/Prediction/Planning/Control/actuation 的统一 event ID、sequence 或传播 provenance；多数 run 缺双时钟历史",
    )
    l4 = gate_detail(
        claim_by_base["C4"],
        "物理 T_R 可观测；24 条 tau_retro 与 12 条 tau_model 已登记并分型",
        "独立 tau_req=0 条，tau_req_low/high 无可用值；模型在两个 300 ms 安全停车 run 上产生碰撞假阳性",
    )
    l5 = gate_detail(
        claim_by_base["C5"],
        f"D_response 墙钟积分中位数 {dr0:.3f}→{dr1:.3f} m；完整停车 run 可计算空间预算分解",
        "主观测 requirement-constrained debt 不可用；碰撞 run 无完整 D_brake/M0；组分解未控制 pre-hazard 状态且不是因果贡献估计",
    )
    l6 = gate_detail(
        claim_by_base["C6"],
        f"baseline 0/7 碰撞、300 ms 主分析 2/4；安全停车 M0 中位数 {m00:.3f}→{m01:.3f} m；碰撞速度 7.988/11.728 m/s",
        "1206 结局冲突；near/critical/high-severity 分类缺预先阈值；碰撞 run 右删失完整停止过程",
    )

    text = f"""# 第二次实验 TCPS-PA v2 实时系统工程六层分析报告

## 六层推理状态矩阵（Six-Layer Inference Status Matrix）

{matrix}

## 受 Claim Ledger 约束的执行结论

本报告覆盖 12 个 run（baseline 7、请求 300 ms 的 Bridge/SCB 干预 5）；`202607271206` 因结局来源冲突退出主结局统计，主分析为 11 个 run。原始实验目录保持只读。

直接观测表明：300 ms 组实际 Bridge 墙钟延时中位数为 **{delay['bridge_delay_actual_wall_data_observed_ms'].median():.3f} ms**；物理反应时间 `T_R` 中位数由 **{tr0:.3f} ms** 变为 **{tr1:.3f} ms**；墙钟积分响应距离由 **{dr0:.3f} m** 变为 **{dr1:.3f} m**；本批 run 的碰撞计数为 baseline **0/7**、300 ms 主分析 **2/4**。这些比例只描述当前小样本。

六层 Gate 的当前输出是：C1 `PASS`，C2/C3 `PARTIAL_PASS` 且 lineage grade C，C4 `NOT_TESTABLE`，C5 `MODEL_SUPPORTED_ONLY`，C6 `PASS`。因此 C7 为 `PARTIAL_PASS/LOW`，最高第 2 级：只支持系统级时间关联和模型支持的空间传播机制。独立时间要求、主观测距离债务、功能正确性、严格 lineage、clock/phase 与 pre-hazard 状态均未闭合。

## 报告定位与方法完成度

本报告同时承担三个角色，但必须区分其完成状态：

- **实验数据审计：** 已具备 raw-first 复算、逐 run 结果、observed/model 分离、缺失原因和右删失规则。
- **六层协议首次实例化：** 已具备 Evidence→Claim→Defeater→Gate→Claim Strength 的结构，并允许输出 `NOT_TESTABLE`。
- **论文方法有效性证据：** 尚未完成；中间的 C4/C5 与多项实时系统前置条件仍缺独立证据。

{method_table}

## 系统架构、事件语义与分析边界

- CARLA 0.9.15 位于服务器端，Apollo 10.0.0 位于 Orin，经网线连接。
- Bridge 直接读取 Control 命令；Guardian 不在本实验执行链中。
- `t_f`：Bridge/SCB 首次应用干预；`t_c/t1`：稳定目标 cause endpoint；`t_e/t2`：持续有效制动起点；`t_d=t1+tau_req`；`t_o`：停车、碰撞或收集终点。
- 主 `T_R=t2-t1` 和 `D_response=∫[t1,t2]v(t)dt_wall` 使用墙钟语义；CARLA frame/sim time 仅作诊断。
- baseline distribution 是 L2 描述性 reference，不是 L4 安全要求。
- 本实验 12/12 run 无同 run 解析 record profile；message-level reaction/data age、Control payload 与严格 provenance 因而不可补齐。

## 六层 Gate 的可执行定义

{gate_matrix}

分析执行顺序：

```text
Raw data
→ 抽取并类型化 Evidence
→ 审计 P_CLOCK/P_TARGET/P_FUNC/P_DEADLINE
→ 依次评价 C1…C6 的 criterion
→ 传播 MODEL/RETRO/CLOCK/TARGET/OUTCOME taint
→ 处理 Defeater 并取 weakest-link ceiling
→ 评价 C7
→ 从 Claim Ledger 生成允许语言和报告
```

## 前置条件审计：时钟、目标、功能与 deadline

- `P_CLOCK.fault_signature=PASS/HIGH`：本地 applied-delay 事实可用。
- `P_CLOCK.cross_host=PARTIAL_PASS/MEDIUM`：10/12 run 无双时钟历史；只有 2 个 run 有约 0.66/0.72 ms alignment residual；offset、drift、resolution 未形成完整预算。
- `P_TARGET=PARTIAL_PASS/MEDIUM`：目标存在性可见，但没有端到端 identity/provenance lineage。
- `P_FUNC=PARTIAL/MEDIUM`：12/12 run 均为 PARTIAL；功能输出存在不等于功能正确性闭合。
- `P_DEADLINE=NOT_TESTABLE/LOW`：全部 requirement 均为事后重建或未验证模型。

### Functional Correctness Audit

{function_table}

### Clock 与 phase uncertainty

所有 run 的 `phase_scan_performed=FALSE`；无法把 300/400/700/800/900 ms 聚类解释为已建立的 tick/phase 机制。单墙钟 `T_R` 可保留，但跨 Orin/server 阶段差值只能给中等置信度。

### Pre-hazard State Divergence Audit

{pre_table}

干预 onset 早于 t1。D1/V1 目前只能视为 `POSSIBLE_MEDIATOR`；因为 fault 时刻状态和 delta 缺失，不能判作独立混杂，也不能定量拆分 total closed-loop effect 与 post-t1 response effect。

## L1 时序故障特征

{l1}

### Temporal Fault Signature 汇总

{fault_table}

300 ms 干预在 t1 前约 22.237–25.997 s 已生效，属于持续 setting，而非目标出现后的一次性延时。baseline 的实际延时 MAX 为 19.282 ms，因此实时系统描述不能只报告 baseline 中位数。

## L2 时序退化：R/A/G 与 tail

{l2}

### R/A/G 逐 run 经验分布

{rag_table}

表中 P95/P99 是基于 7 个 baseline 与 4 个 delay 主分析 run 的经验分位数，不是 WCET 保证，也不是消息帧的独立重复。Sensor→Control 中位数约由 **{stage[stage['run_id'].isin(baseline['run_id'])]['sensor_to_control_ms'].median():.3f} ms** 变为 **{stage[stage['run_id'].isin(delay['run_id'])]['sensor_to_control_ms'].median():.3f} ms**；Control→t2 中位数约由 **{stage[stage['run_id'].isin(baseline['run_id'])]['control_to_t2_ms'].median():.3f} ms** 变为 **{stage[stage['run_id'].isin(delay['run_id'])]['control_to_t2_ms'].median():.3f} ms**。后半段变化更大，但 clock/phase 与 lineage 限制仍在。

## L3 Cause-Effect Chain 与物理反应区间

{l3}

当前可支持的链是：

| 事件 | 当前来源 | 匹配方式 | 证据上限 |
|---|---|---|---|
| `t_f` Bridge/SCB onset | SCB applied log | 直接记录 | 本地直接观测 |
| `t_c/t1` 稳定目标 source | Perception/Fusion + wall endpoint | 稳定序列与时间对齐 | C 级 |
| Prediction | Prediction log/trace event | 时间接近/阶段匹配 | C 级 |
| Planning STOP | Planning log | STOP 事件时间 | C 级，target/location 未闭合 |
| Control/Bridge | Control trace + SCB | 时间匹配，无 payload lineage | C 级 |
| `t_e/t2` 物理制动 | Localization wall speed | 直接观测派生 | 物理区间有效 |

缺少统一 event ID/sequence/propagated timestamp，所以 `T_R` 是系统级物理反应区间，不能改写为 formal MRT。

## L4 时间正确性与独立 deadline

{l4}

{req_table}

`tau_retro` 依赖同 run 的完整停车过程，只能回答事后还能等待多久；`tau_model` 使用 baseline 样本内常有效减速度 **5.102 m/s²**，没有独立验证和 uncertainty bounds。主 `S_T=tau_req-T_R` 因 `tau_req` 缺失而不可计算。

模型在 300 ms 主分析 4 个 run 中给出 {int(pred_collision.sum())}/4 个碰撞预测，而实际为 2/4，其中 {false_positive} 个安全停车 run 被预测为碰撞。这是主 C4 不采用该模型作为合格 requirement 的直接理由。

## L5 从时间到物理空间的传播

{l5}

主响应距离：

`D_response_wall_integral_data_observed_m = ∫[t1,t2] v(t) dt_wall`

合格主距离债务只有在独立 `tau_req` 存在时才定义：

`D_debt = max(0, ∫[t1+tau_req,t2] v(t) dt_wall)`

当前 `D_debt_requirement_constrained_derived_m` 不可用；`D_debt_retro_diagnostic_m` 与 `D_debt_model_predicted_m` 分别保留事后/模型来源。

### 观测空间预算分解

`M0 = D1 - D_response - D_brake`

`ΔM0 = ΔD1 - ΔD_response - ΔD_brake`

{budget_table}

差值行采用端点兼容完整停车 run 的组均值：baseline n=7、300 ms 安全停车 n=2。该恒等式用于描述空间预算，不是因果贡献估计；碰撞 run 因右删失不能获得完整 D_brake/M0。

## L6 物理安全退化

{l6}

连续结果链为 `D2 → M0/M6 → impact → outcome`。本实验直接保存 D2、完整停车 margin 和碰撞速度，但尚无有来源的 LOW_MARGIN/CRITICAL/NEAR_MISS/HIGH_SEVERITY 阈值，因此不在看到结局后补设等级。

碰撞 run 的完整停车距离与 full-stop margin 保持不可用；只使用碰撞前截断路径、碰撞事件、actor/geometry 证据与碰撞速度。

## C7 时间安全归因

- **判定：** `PARTIAL_PASS/LOW`，最高第 2 级。
- **允许结论：** 系统级时间关联得到支持；模型结果提供一种待独立验证的空间传播机制。
- **不能升级的原因：** C4 不可检验、C5 仅模型支持、P_FUNC 部分、pre-hazard 状态未测全、cross-host clock 与 phase 未闭合。

{c7_table}

## 模型、事后重建与误差分析（单独呈现）

- 完整停车主分析 run 的模型制动距离平均绝对误差为 **{full_stop_model['D_brake_absolute_error_model_m'].mean():.3f} m**，绝对误差中位数为 **{full_stop_model['D_brake_absolute_error_model_m'].median():.3f} m**。
- 两个碰撞 run 的碰撞速度模型绝对误差平均为 **{collision_model['impact_speed_absolute_error_model_mps'].mean():.3f} m/s**。
- 模型为样本内描述模型，未报告 a_brake 独立校准区间、外部验证集或 tau_req_low/high；因此不能承担主时间正确性判定。

## 逐 run 观测结果

{run_table}

## 论文方法章节与后续实验所需内容

当前报告已经给出方法输入、Gate、证据类型、criterion、taint、defeater、weakest-link 与输出语言；但要把 TCPS-PA 作为论文核心方法验证，还需要：

1. 同 run record 导出，建立 message-level R/A/G、MRT/MDA 与 A/B 级 lineage。
2. 独立、前瞻、带 `tau_req_low/center/high` 的场景时间要求，并使用独立数据校准/验证。
3. 100/200/300/400 ms 等多个非零等级、充分 repeats、初始状态匹配、phase 随机化或扫描。
4. fault-onset→t1 的 position/speed/acceleration/heading/route/Control/Bridge 历史，分离 total closed-loop 与 post-t1 effect。
5. Control payload、Planning target/location/trajectory 和 Bridge apply 的功能语义闭环。
6. clock offset/drift/resolution 预算、CARLA fixed step 与 phase-to-tick 审计。
7. 负对照、边界案例、endpoint/threshold 敏感性、方法消融和 Apollo 之外的迁移验证。

因此，本实验完成的是“结构化六层诊断协议的一次保守运行”，不是对所有中间机制的经验闭合。

## 图表与支持表

- [R/A/G 逐 run 分布](../figures/realtime_rag_run_distributions.png)
- [观测空间预算分解](../figures/space_budget_decomposition_observed.png)
- [六层 Gate 状态](../figures/six_layer_chain.png)
- [干预与物理响应](../figures/intervention_vs_physical_response.png)
- [响应距离与模型 deadline debt](../figures/response_distance_and_deadline_debt.png)
- 支持表：`realtime_rag_summary.csv`、`space_budget_decomposition_observed.csv`、`space_budget_group_decomposition.csv`、`method_completeness_matrix.csv`。

## 复现与验证

从 `/Users/huangjinhui/Desktop/萨卡班/data` 执行：

```bash
python3 report_workspace/scripts/analyze_second_experiment.py
python3 report_workspace/scripts/validate_outputs.py
TCPS_PA_OUTPUT_DIR='/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2' python3 output/second_experiment_six_layer_analysis/scripts/build_six_layer_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/bootstrap_inference_ledgers.py --analysis-dir '/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2'
python3 output/second_experiment_tcps_pa_v2/scripts/render_realtime_engineering_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/validate_analysis_outputs.py --analysis-dir '/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2'
```

最终语义状态以 `validation/validation.json` 为准。证据追溯入口为 Evidence Ledger、Claim Ledger、Claim Edges、Defeater Ledger 与 Claim Audit。原始实验目录未修改。
"""

    REPORT.write_text(text, encoding="utf-8")
    print(f"Wrote comprehensive real-time engineering report: {REPORT}")


if __name__ == "__main__":
    main()
