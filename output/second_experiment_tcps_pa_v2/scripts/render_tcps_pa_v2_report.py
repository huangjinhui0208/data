#!/usr/bin/env python3
"""Render the second-experiment report as a view of validated TCPS-PA v2 ledgers."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
REPORT = ROOT / "report" / "six_layer_analysis_report.md"

CLAIM_ZH = {
    "C1": "已声明且作用范围明确的时序扰动进入了闭环。",
    "C2": "相对于已声明的参考分布，时序行为出现退化。",
    "C3": "时序退化沿相关因果链形成了传播关联。",
    "C4": "观测物理反应时间超过了独立合格的时间要求。",
    "C5": "合格的时间违例造成了可量化的额外空间消耗。",
    "C6": "实验中的物理安全裕度或直接结局出现退化。",
    "C7": "在明确的证据强度边界内，将物理退化归因于时间正确性退化。",
}

RESIDUAL_ZH = {
    "C1": "该干预来自外部 Bridge/SCB，不能据此判断 Apollo 内部产生了同类扰动。",
    "C2": "update-gap 缺合格参考分布，跨主机时钟和注入相位尚未充分审计。",
    "C3": "没有显式 trace/provenance lineage，目标连续性和跨主机时钟仍是部分证据。",
    "C4": "没有独立合格的同场景时间要求，现有 deadline 仅来自事后重建或未验证模型。",
    "C5": "主 deadline gate 未成立，距离债务仍带有事后或模型来源标记。",
    "C6": "物理结局本身不完成时间归因，且 1206 的结局证据存在冲突。",
    "C7": "功能链、初始状态、制动能力、时钟、相位、几何与结局冲突等替代解释仍未闭合。",
}

DEFEATER_ZH = {
    "D_INITIAL_CLEARANCE": "初始净距可能在干预之外发生变化。",
    "D_INITIAL_SPEED": "初始或接近速度可能独立改变物理结局。",
    "D_BRAKING_CAPABILITY": "制动能力目前主要由样本内模型表示。",
    "D_FUNCTIONAL_FAILURE": "功能性替代解释仍未排除。",
    "D_TARGET_MISMATCH": "端到端目标 lineage 仍不完整。",
    "D_DATA_FRESHNESS": "新鲜度退化尚未被独立隔离。",
    "D_UPDATE_GAP": "update-gap 缺少合格参考分布。",
    "D_SOLVER_FALLBACK": "Planning fallback 或不可行状态可能独立参与。",
    "D_CLOCK": "跨主机时间对齐仍为部分证据。",
    "D_PHASE": "尚未扫描注入相位效应。",
    "D_PREHAZARD_STATE": "干预早于 t1，D1/v1 可能属于干预后的状态。",
    "D_GEOMETRY": "各 run 的几何与目标净距存在差异。",
    "D_OUTCOME_CONFLICT": "不同结局来源尚未完成一致性闭合。",
    "D_DEADLINE": "C4 缺少独立合格的时间要求。",
}

OUTCOME_ZH = {
    "safe_stop": "安全停车",
    "collision": "碰撞",
    "uncertain_geometry_event_conflict": "结局不确定（事件与几何冲突）",
}


def available(value: object) -> bool:
    if value is None:
        return False
    try:
        return not math.isnan(float(value))
    except (TypeError, ValueError):
        return str(value).strip() != ""


def fmt(value: object, digits: int = 3, suffix: str = "") -> str:
    if not available(value):
        return "—"
    return f"{float(value):.{digits}f}{suffix}"


def truthy(series: pd.Series) -> pd.Series:
    return series.astype(str).str.strip().str.lower().isin({"true", "1", "yes"})


def split_ids(value: object) -> list[str]:
    if not available(value):
        return []
    return [part.strip() for part in str(value).split("|") if part.strip()]


def claim_base(claim_id: str) -> str:
    return claim_id.split(".", 1)[0]


def defeater_description_zh(defeater_id: str) -> str:
    for prefix, description in DEFEATER_ZH.items():
        if defeater_id.startswith(prefix + "."):
            return description
    return "该反证项仍需补充独立证据。"


def render_claim_status_figure(claim_by_base: dict[str, pd.Series]) -> None:
    """Replace the legacy continuous-chain sketch with ledger gate statuses."""
    items = [
        ("L1 / C1", "C1"),
        ("L2 / C2", "C2"),
        ("L3 / C3", "C3"),
        ("L4 / C4", "C4"),
        ("L5 / C5", "C5"),
        ("L6 / C6", "C6"),
        ("Attribution / C7", "C7"),
    ]
    colors = {
        "PASS": "#B7E4C7",
        "PARTIAL_PASS": "#FFE8A1",
        "NOT_TESTABLE": "#E5E7EB",
        "MODEL_SUPPORTED_ONLY": "#D8C4F1",
    }
    plt.rcParams["font.sans-serif"] = [
        "Arial Unicode MS", "PingFang SC", "Heiti TC", "SimHei", "DejaVu Sans"
    ]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(16, 3.5))
    ax.set_xlim(-0.5, len(items) - 0.5)
    ax.set_ylim(-0.8, 1.3)
    ax.axis("off")
    for index, (label, base) in enumerate(items):
        claim = claim_by_base[base]
        verdict = claim["verdict"]
        if index:
            ax.add_patch(
                FancyArrowPatch(
                    (index - 0.60, 0.30),
                    (index - 0.40, 0.30),
                    arrowstyle="-|>",
                    mutation_scale=13,
                    linewidth=1.4,
                    color="#59636E",
                    linestyle="--" if base in {"C4", "C5", "C7"} else "-",
                )
            )
        box = FancyBboxPatch(
            (index - 0.39, -0.08),
            0.78,
            0.76,
            boxstyle="round,pad=0.04,rounding_size=0.07",
            facecolor=colors.get(verdict, "#F4D6D6"),
            edgecolor="#39424E",
            linewidth=1.4,
        )
        ax.add_patch(box)
        ax.text(index, 0.48, label, ha="center", va="center", fontsize=11, weight="bold")
        status = verdict.replace("MODEL_SUPPORTED_ONLY", "仅模型支持")
        ax.text(index, 0.16, status, ha="center", va="center", fontsize=9.5)
        if base == "C7":
            ax.text(
                index,
                -0.32,
                f"最高声明等级：{int(float(claim['maximum_claim_level']))}",
                ha="center",
                va="center",
                fontsize=9,
                color="#5A334A",
            )
    ax.text(
        3.0,
        1.02,
        "TCPS-PA v2 六层 Gate 状态——虚线表示最弱环节边界",
        ha="center",
        va="center",
        fontsize=15,
        weight="bold",
    )
    fig.tight_layout()
    fig.savefig(ROOT / "figures" / "six_layer_chain.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def report_gate_block(
    claim: pd.Series,
    evidence_note: str,
    counter_evidence: str,
    allowed_conclusion: str,
) -> str:
    base = claim_base(str(claim["claim_id"]))
    prerequisites = split_ids(claim["prerequisite_claim_ids"])
    supports = split_ids(claim["supporting_evidence_ids"])
    defeaters = split_ids(claim["defeater_ids"])
    support_preview = ", ".join(supports[:4])
    if len(supports) > 4:
        support_preview += f"，以及其余 {len(supports) - 4} 条关联证据"
    return f"""
- **科学命题：** {CLAIM_ZH[base]}
- **必要前提：** {', '.join(prerequisites) if prerequisites else '无前置 Claim'}。
- **支持证据：** {evidence_note} 账本链接：{support_preview or '本 Claim 由前置 Claim 限定'}。
- **反向证据：** {counter_evidence}
- **未闭合反证项：** {len(defeaters)} 项，详见 Defeater Ledger（反证账本）。
- **推理规则：** `{claim['inference_rule_id']}`。
- **判定：** `{claim['verdict']}`。
- **置信度：** `{claim['confidence']}`；置信度上限为 `{claim['confidence_ceiling']}`。
- **允许结论：** {allowed_conclusion}
- **仍未证明：** {RESIDUAL_ZH[base]}
""".strip()


def main() -> None:
    observed = pd.read_csv(TABLES / "run_level_observed.csv", dtype={"run_id": str})
    model = pd.read_csv(TABLES / "run_level_model_predicted.csv", dtype={"run_id": str})
    claims = pd.read_csv(TABLES / "claim_ledger.csv").fillna("")
    defeaters = pd.read_csv(TABLES / "defeater_ledger.csv").fillna("")
    group = pd.read_csv(TABLES / "group_summary_observed.csv")

    claim_by_base = {
        claim_base(row.claim_id): row
        for _, row in claims.iterrows()
    }
    expected = {
        "C1": "PASS",
        "C2": "PARTIAL_PASS",
        "C3": "PARTIAL_PASS",
        "C4": "NOT_TESTABLE",
        "C5": "MODEL_SUPPORTED_ONLY",
        "C6": "PASS",
        "C7": "PARTIAL_PASS",
    }
    actual = {base: claim_by_base[base]["verdict"] for base in expected}
    if actual != expected:
        raise RuntimeError(f"Claim Ledger changed; refusing stale prose: {actual}")
    render_claim_status_figure(claim_by_base)

    observed["is_main"] = truthy(observed["included_main_analysis"])
    main_runs = observed[observed["is_main"]].copy()
    baseline = main_runs[main_runs["group_name"] == "baseline"]
    delay = main_runs[main_runs["group_name"] == "delay_300ms"]

    tr_baseline = baseline["T_e2e_data_observed_ms"].median()
    tr_delay = delay["T_e2e_data_observed_ms"].median()
    dr_baseline = baseline["D_response_wall_integral_data_observed_m"].median()
    dr_delay = delay["D_response_wall_integral_data_observed_m"].median()
    d1_baseline = baseline["D1_clear_data_observed_m"].median()
    d1_delay = delay["D1_clear_data_observed_m"].median()
    m0_baseline = baseline["M_collision_0m_data_observed_m"].median()
    m0_delay = delay["M_collision_0m_data_observed_m"].median()
    actual_delay = delay["bridge_delay_actual_wall_data_observed_ms"].median()

    main_model = model[model["run_id"].isin(main_runs["run_id"])].copy()
    full_stop_model = main_model[main_model["D_brake_data_observed_comparator_m"].notna()]
    model_mae = full_stop_model["D_brake_absolute_error_model_m"].mean()
    model_median_ae = full_stop_model["D_brake_absolute_error_model_m"].median()
    collision_rows = main_model[
        main_model["outcome_data_observed_comparator"] == "collision"
    ]
    impact_mae = collision_rows["impact_speed_absolute_error_model_mps"].mean()
    delay_model = main_model[main_model["group_name"] == "delay_300ms"]
    model_pred_collision = int(truthy(delay_model["collision_model_predicted"]).sum())
    model_false_positive = int(
        (
            truthy(delay_model["collision_model_predicted"])
            & delay_model["outcome_data_observed_comparator"].eq("safe_stop")
        ).sum()
    )

    matrix_rows = [
        ("L1 / C1", "C1", "直接观测", "Bridge/SCB 外部注入已核验"),
        ("L2 / C2", "C2", "观测派生", "T_R 相对 baseline 分布的描述性变化"),
        ("L3 / C3", "C3", "关联等级 C", "跨链墙钟关联，缺显式 trace lineage"),
        ("L4 / C4", "C4", "事后重建 + 未验证模型", "独立 deadline 缺失"),
        ("L5 / C5", "C5", "模型来源标记", "D_response 为观测；debt 仅事后/模型"),
        ("L6 / C6", "C6", "直接观测", "净距、停止与碰撞结局"),
        ("Attribution / C7", "C7", "最弱环节", "第 2 级：系统级时间关联与模型机制"),
    ]
    matrix = [
        "| 层级/Claim | 判定 | 证据基础 | 置信度 | 上限 | 未闭合关键反证项 | 允许结论 |",
        "|---|---|---|---|---|---:|---|",
    ]
    for label, base, evidence_basis, allowed in matrix_rows:
        claim = claim_by_base[base]
        claim_defeaters = defeaters[defeaters["claim_id"] == claim["claim_id"]]
        critical_open = claim_defeaters[
            claim_defeaters["status"].isin(["OPEN", "UNKNOWN"])
            & claim_defeaters["impact_on_claim"].isin(["CRITICAL", "INVALIDATES"])
        ]
        ceiling = claim["confidence_ceiling"]
        if base == "C7":
            ceiling = f"第 {int(float(claim['maximum_claim_level']))} 级 / {ceiling}"
        matrix.append(
            f"| {label} | {claim['verdict']} | {evidence_basis} | {claim['confidence']} | "
            f"{ceiling} | {len(critical_open)} | {allowed} |"
        )

    run_rows = [
        "| run | 分组 | 分析范围 | T_R 观测值 (ms) | D_response 观测值 (m) | D1 净距 (m) | D2 净距 (m) | M0 观测值 (m) | 观测结局 | 碰撞速度观测值 (m/s) |",
        "|---|---|---|---:|---:|---:|---:|---:|---|---:|",
    ]
    for row in observed.itertuples():
        scope = "主分析" if row.is_main else "排除：结局冲突"
        outcome = OUTCOME_ZH.get(row.outcome_data_observed, row.outcome_data_observed)
        run_rows.append(
            f"| {row.run_id} | {row.group_name} | {scope} | "
            f"{fmt(row.T_e2e_data_observed_ms)} | "
            f"{fmt(row.D_response_wall_integral_data_observed_m)} | "
            f"{fmt(row.D1_clear_data_observed_m)} | {fmt(row.D2_clear_data_observed_m)} | "
            f"{fmt(row.M_collision_0m_data_observed_m)} | {outcome} | "
            f"{fmt(row.impact_speed_data_observed_mps)} |"
        )

    c7_defeaters = defeaters[defeaters["claim_id"] == "C7.all_runs"]
    defeater_rows = [
        "| 反证项 | 状态 | 对结论的影响 | 尚存问题 |",
        "|---|---|---|---|",
    ]
    for row in c7_defeaters.itertuples():
        defeater_rows.append(
            f"| `{row.defeater_id}` | {row.status} | {row.impact_on_claim} | "
            f"{defeater_description_zh(row.defeater_id)} |"
        )

    l1 = report_gate_block(
        claim_by_base["C1"],
        f"300 ms 组主分析 run 的实际 Bridge 墙钟延时中位数为 {actual_delay:.3f} ms；每个 run 均有请求值、实际值及生命周期证据。",
        "注入位于 Bridge/SCB，不能外推为 Apollo 内生缺陷。",
        "外部注入的时间扰动在本地时钟域内得到直接核验。",
    )
    l2 = report_gate_block(
        claim_by_base["C2"],
        f"T_R 主分析中位数由 {tr_baseline:.3f} ms 变为 {tr_delay:.3f} ms，描述性差值 {tr_delay-tr_baseline:.3f} ms；仅有一个非零注入等级。",
        "update-gap 只有部分 run 可用，且缺独立参考分布；注入相位未扫描，跨主机对时不完整。",
        "观测到与实验分组一致的 T_R 分布变化，但强度受参考、时钟和相位证据限制。",
    )
    l3 = report_gate_block(
        claim_by_base["C3"],
        "Sensor、Fusion、Prediction、Planning STOP、Control/Bridge、物理 t2 的事件时间线可组成 C 级时间对齐链。",
        "没有跨模块 trace/provenance ID；多数 run 无双时钟历史，目标也没有端到端显式 lineage。",
        "现有数据支持跨链时间关联，不能提高到显式因果 lineage。",
    )
    l4 = report_gate_block(
        claim_by_base["C4"],
        "每个可重建的 tau_retro 均登记为事后重建类（RETROSPECTIVE_RECONSTRUCTION）；常减速度 tau_model 登记为未验证模型类（UNVALIDATED_MODEL）。",
        "没有预注册、外部规范或独立验证的同场景 tau_req；碰撞 run 的完整停止端点被右删失。",
        "只可做事后重建和模型敏感性比较，主 deadline 判定保持不可检验。",
    )
    l5 = report_gate_block(
        claim_by_base["C5"],
        f"D_response 采用 t1 到 t2 的墙钟速度梯形积分，中位数由 {dr_baseline:.3f} m 变为 {dr_delay:.3f} m，描述性差值 {dr_delay-dr_baseline:.3f} m。",
        "缺合格 tau_req，因此基于合格要求的距离债务不可用；事后距离债务与模型距离债务保留各自来源标记。",
        "观测响应距离与模型结果支持一种可能的空间传播机制，不能将模型距离债务改写成主观测距离债务。",
    )
    l6 = report_gate_block(
        claim_by_base["C6"],
        f"baseline 为 0/7 碰撞，300 ms 组主分析为 2/4 碰撞；两个碰撞 run 的碰撞速度分别为 7.988 和 11.728 m/s。安全停车 run 的 M0 中位数由 {m0_baseline:.3f} m 变为 {m0_delay:.3f} m。",
        "1206 的事件源与固定几何发生冲突，已退出主结局统计；碰撞 run 不存在完整观测制动距离。",
        "实验内观测到物理安全裕度和结局差异；该层判定不单独分配时间归因。",
    )

    text = f"""# 第二次实验 TCPS-PA v2 六层分析

## 六层推理状态矩阵（Six-Layer Inference Status Matrix）

{chr(10).join(matrix)}

## 受 Claim Ledger 约束的执行结论

本次重新分析覆盖 12 个 run：baseline 7 个、请求 300 ms 的 Bridge/SCB 注入 5 个。`202607271206` 因碰撞事件源与固定几何不一致，仅保留在审计与敏感性材料中；主分析为 11 个 run。所有原始日志、CSV 与实验目录保持只读。

可直接观察的事实是：Bridge/SCB 注入被记录，300 ms 组主分析 run 的实际墙钟延时中位数为 **{actual_delay:.3f} ms**；物理反应时间 `T_R` 中位数从 **{tr_baseline:.3f} ms** 变为 **{tr_delay:.3f} ms**，墙钟速度积分 `D_response` 中位数从 **{dr_baseline:.3f} m** 变为 **{dr_delay:.3f} m**。baseline 组为 **0/7** 碰撞，300 ms 组主分析为 **2/4** 碰撞。小样本比例仅描述本批 run，不视为总体碰撞概率估计。

Claim Ledger（声明账本）的约束结论是：C1 为 `PASS`，C2/C3 为 `PARTIAL_PASS` 且 lineage 等级为 C；没有独立合格的物理时间要求，因此 C4 为 `NOT_TESTABLE`。C5 只能达到 `MODEL_SUPPORTED_ONLY`：`D_response` 是实际观测积分，基于合格要求的距离债务不可用，事后重建和模型距离债务必须分列。C6 为 `PASS`，因为净距、停止与碰撞属于直接物理证据。综合归因 C7 为 `PARTIAL_PASS`、LOW 置信度，最高为 **第 2 级：系统级时间关联与模型支持的传播机制**。当前证据不足以判断 Apollo 内部缺陷，也不足以量化单一因素的因果权重。

## 分析范围、系统架构、干预与参考基线

- 环境：CARLA 0.9.15 位于服务器端，Apollo 10.0.0 位于 Orin，二者经网线连接。
- 命令路径：本实验未把 Guardian 命令送入 Bridge；Bridge 读取 Control 命令。因此功能审计的路径边界按 Control→Bridge 处理。
- 干预：外部 Bridge/SCB 控制延时，请求等级为 0 ms 与 300 ms；只有一个非零等级。
- L2 参考：baseline run 的实验内分布，仅用于描述性对照，不是物理安全要求。
- L4 时间要求：没有独立、同场景、同单位且通过资格审计的 `tau_req`。
- Record：12 个 run 均没有同 run 的解析 record profile，record 指标标记为不可用，不用于补齐六层结论。

## 证据、时钟、目标、功能链与 deadline 资格审计

- `P_CLOCK.fault_signature = PASS/HIGH`：本地 Bridge/SCB 请求值与实际墙钟延时可比较。
- `P_CLOCK.cross_host = PARTIAL_PASS/MEDIUM`：两个碰撞 run 有对齐残差记录，其余 run 缺双时钟历史；未完成相位扫描。
- `P_TARGET = PARTIAL_PASS/MEDIUM`：各 run 有目标 ID/存在性，但没有跨模块显式 lineage。
- `P_FUNC = PARTIAL/MEDIUM`：Planning STOP 与物理响应可见，但 Control payload 未归档，命令连续性及目标相关正确性未闭合。
- `P_DEADLINE = NOT_TESTABLE/LOW`：现有 deadline 为事后重建或未独立验证的模型输出。
- `D1_clear` 中位数 baseline/300 ms 分别为 **{d1_baseline:.3f}/{d1_delay:.3f} m**，但个体差异、注入先于 t1 以及危险事件前状态不足仍进入 C7 反证项。

## L1 时序故障特征

{l1}

## L2 时序退化

{l2}

## L3 因果链时间传播

{l3}

## L4 时间正确性

{l4}

## L5 从时间到物理空间的传播

{l5}

主距离字段是 `D_response_wall_integral_data_observed_m = ∫[t1,t2] v(t) dt_wall`。旧字段 `D_delay_wall_integral_data_observed_m` 仅作为兼容别名保留，两者数值一致。CARLA 仿真帧数、仿真时间与 Localization 空间位移没有混入主距离列。

## L6 物理安全退化

{l6}

碰撞 run 的完整观测停止距离与 0 m/6 m 完整停止裕度记为不可用；只保留碰撞前的截断路径、事件结局与碰撞速度。安全停车 run 的实际结果不会被模型值覆盖。

## C7 时间安全归因

- **判定：** `PARTIAL_PASS`；置信度为 `LOW`；最高声明等级为 `2`。
- **允许结论：** 外部干预、T_R 分布、墙钟响应距离和物理结局在本实验中形成系统级时间关联；常减速度模型提供一种待独立验证的空间传播解释。
- **边界：** C4 缺独立时间要求，C5 带模型/事后来源标记，P_FUNC、跨主机时钟、相位、危险前状态和几何审计仍未闭合。因此 C7 不提高到实质性贡献或确定因果分配。

## 模型与事后重建分析（与主观测结果分离）

模型表与观测表物理分离。模型为 baseline 组 7 个完整停止 run 的中位有效减速度 **5.102 m/s²**，属于样本内、未交叉验证的常有效减速度描述模型。

- 在主分析可比较的 {len(full_stop_model)} 个完整停止 run 中，模型制动距离平均绝对误差为 **{model_mae:.3f} m**，绝对误差中位数为 **{model_median_ae:.3f} m**。
- 在 300 ms 主分析 4 个 run 中，模型给出 {model_pred_collision}/4 个碰撞预测；实际为 2/4，因此包含 {model_false_positive} 个安全停车假阳性。
- 两个碰撞 run 的碰撞速度模型绝对误差平均为 **{impact_mae:.3f} m/s**。
- 事后 `tau_retro_*` 依赖同 run 的完整停止轨迹，只能用于重建和敏感性分析。碰撞 run 因右删失没有完整 `tau_retro`。
- `D_debt_model_predicted_m` 继承模型来源标记；`D_debt_retro_diagnostic_m` 继承事后重建来源标记。`D_debt_requirement_constrained_derived_m` 当前不可用。

## 未闭合反证项与剩余不确定性

{chr(10).join(defeater_rows)}

这些反证项决定 C7 的最弱环节上限。下一轮若要提高结论强度，优先补充：独立场景 deadline、端到端目标/trace lineage、双时钟偏移/漂移与相位扫描、Control payload/命令连续性、注入前 D1/v1/路线状态，以及多个非零延时等级和更多独立 run。

## 逐 run 观测结果

{chr(10).join(run_rows)}

注：`M0` 观测值仅对具有完整观测停止端点的 run 可计算；碰撞 run 的空值表示右删失，不是 0，也不以模型结果补值。

## 图表

- [干预与物理响应](../figures/intervention_vs_physical_response.png)
- [时间指标与物理结局](../figures/timing_to_physical_outcome.png)
- [响应距离与 deadline 距离债务](../figures/response_distance_and_deadline_debt.png)
- [六层 Gate 状态](../figures/six_layer_chain.png)

图中的 deadline/距离债务必须按图例区分观测响应、事后重建与模型结果；它们不能改变状态矩阵的层级判定。

## 复现方法

从工作区 `/Users/huangjinhui/Desktop/萨卡班/data` 执行：

```bash
python3 report_workspace/scripts/analyze_second_experiment.py
python3 report_workspace/scripts/validate_outputs.py
TCPS_PA_OUTPUT_DIR='/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2' python3 output/second_experiment_six_layer_analysis/scripts/build_six_layer_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/bootstrap_inference_ledgers.py --analysis-dir '/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2'
python3 output/second_experiment_tcps_pa_v2/scripts/render_tcps_pa_v2_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/validate_analysis_outputs.py --analysis-dir '/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_tcps_pa_v2'
```

证据追溯入口：`tables/evidence_ledger.csv`、`tables/claim_ledger.csv`、`tables/claim_edges.csv`、`tables/defeater_ledger.csv` 与 `validation/claim_audit.md`。原始实验目录未被修改。
"""

    REPORT.write_text(text, encoding="utf-8")
    print(f"Wrote ledger-constrained report: {REPORT}")


if __name__ == "__main__":
    main()
