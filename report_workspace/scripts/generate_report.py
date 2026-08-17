#!/usr/bin/env python3
"""Generate the group-meeting report from the recomputed CSV tables."""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
REPORT_DIR = ROOT / "report"


def fmt(value: object, digits: int = 3) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "NA"
    return f"{number:.{digits}f}" if math.isfinite(number) else "NA"


def run_table(frame: pd.DataFrame) -> str:
    lines = [
        "| run | 组别 | 主分析 | T_e2e/ms | D1/m | D_delay/m | D2/m | D_brake,data/m | M0/m | 结局 |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    names = {
        "safe_stop": "停车/无碰撞事件",
        "collision": "碰撞",
        "uncertain_geometry_event_conflict": "结局不确定",
    }
    for _, row in frame.iterrows():
        outcome = names[row.outcome_data_observed]
        if row.outcome_data_observed == "collision":
            outcome += f"，{fmt(row.impact_speed_data_observed_mps)} m/s"
        lines.append(
            f"| {row.run_id} | {'baseline' if row.group_name == 'baseline' else '300 ms'} | "
            f"{'是' if row.included_main_analysis else '否'} | {fmt(row.T_e2e_data_observed_ms)} | "
            f"{fmt(row.D1_clear_data_observed_m)} | {fmt(row.D_delay_wall_integral_data_observed_m)} | "
            f"{fmt(row.D2_clear_data_observed_m)} | {fmt(row.D_brake_data_observed_m)} | "
            f"{fmt(row.M_collision_0m_data_observed_m)} | {outcome} |"
        )
    return "\n".join(lines)


def main() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    runs = pd.read_csv(TABLES / "run_level_metrics.csv", dtype={"run_id": str})
    summary = pd.read_csv(TABLES / "group_summary.csv")
    stages = pd.read_csv(TABLES / "stage_latency_summary.csv")
    compare = pd.read_csv(
        TABLES / "collision_case_comparison.csv",
        dtype={"collision_run_id": str, "safe_control_run_id": str},
    )
    counter = pd.read_csv(TABLES / "counterfactual_model.csv", dtype={"run_id": str})
    identity = pd.read_csv(TABLES / "target_identity_audit.csv", dtype={"run_id": str})
    by_id = {row.run_id: row for _, row in runs.iterrows()}

    def sv(group: str, metric: str, stat: str = "median") -> float:
        return float(summary[(summary.group_name == group) & (summary.metric == metric)].iloc[0][stat])

    def st(group: str, metric: str) -> float:
        return float(
            stages[(stages.group_name == group) & (stages.metric == metric)]
            .iloc[0]["median"]
        )

    def cp(case: str, metric: str, field: str = "collision_minus_control") -> float:
        return float(compare[(compare.collision_run_id == case) & (compare.metric == metric)].iloc[0][field])

    b_lat, d_lat = sv("baseline", "T_e2e_data_observed_ms"), sv("delay_300ms", "T_e2e_data_observed_ms")
    b_dd, d_dd = sv("baseline", "D_delay_wall_integral_data_observed_m"), sv("delay_300ms", "D_delay_wall_integral_data_observed_m")
    b_d2, d_d2 = sv("baseline", "D2_clear_data_observed_m"), sv("delay_300ms", "D2_clear_data_observed_m")
    b_m0 = sv("baseline", "M_collision_0m_data_observed_m")
    r1131, r1202, r1206, r1211, r1643 = (by_id[x] for x in ["202607271131", "202607271202", "202607271206", "202607271211", "202607271643"])
    cf1131 = counter[counter.run_id == "202607271131"].iloc[0]
    cf1643 = counter[counter.run_id == "202607271643"].iloc[0]
    id1131 = identity[identity.run_id == "202607271131"].iloc[0]
    id1643 = identity[identity.run_id == "202607271643"].iloc[0]
    main = runs[runs.included_main_analysis]
    b_scb = main[main.group_name == "baseline"].scb_actual_wall_delay_ms
    d_scb = main[main.group_name == "delay_300ms"].scb_actual_wall_delay_ms

    report = f"""# 《结果正确但响应过晚：Apollo闭环实时性对车速隐形Deadline和碰撞结局的影响》

> 组会技术报告｜第二次车速隐形Deadline实验｜原始数据统一重算

## 摘要

本实验关心的不是 Apollo 最终有没有给出停车决策，而是在约 16–18 m/s 接近静态障碍物时，正确决策是否在可用距离预算内到达车辆。本次从原始 Perception、Prediction、Planning、Localization、Trace、SCB、CollisionSensor 和 actor history 重建 12 个 run，而非转述旧报告。11 个 run 进入主分析；`202607271206` 的无碰撞事件与固定目标几何穿透冲突，因此作为结局不确定样本单列。

实测 300 ms 控制延迟使端到端响应中位数从 {b_lat:.3f} ms 增加到 {d_lat:.3f} ms，增加 {d_lat-b_lat:.3f} ms；响应阶段的墙钟速度积分距离 D_delay 从 {b_dd:.3f} m 增加到 {d_dd:.3f} m，增加 {d_dd-b_dd:.3f} m；有效制动开始时的剩余净距 D2 从 {b_d2:.3f} m 降到 {d_d2:.3f} m，减少 {b_d2-d_d2:.3f} m。baseline 组 0/7 碰撞，300 ms 主分析组 2/4 碰撞。数据支持“延迟→更晚的有效制动→更大距离债务→更小制动空间”的机制链。但当前样本不支持一个通用 700 ms 硬阈值；更准确的表述是，在本车速、起始净距和制动能力范围内，699–894 ms 落入当前样本的结局转换区。

![干预到结局的距离预算链](../figures/causal_chain.png)

## 1. 研究动机：“功能正确”不等于“闭环安全”

传统功能验证往往以“是否识别障碍物、是否生成 STOP、是否输出制动”为主。但高速接近静态障碍物时，系统拥有的是有限距离，不是无限计算时间。车辆在感知、Prediction、Planning、Control、Bridge 与动力学响应中每晚一段时间，都会按当时车速继续消耗净距。因此，必须把时延翻译成空间债务，并区分“功能链完成”与“完成时仍有足够制动空间”。

本报告不将碰撞简单等同于 Planning 失败，也不将最终车速下降当作安全证明。主问题是：300 ms 干预是否真正进入执行链；它是否转化为更长的 T_e2e 与 D_delay；两个碰撞 run 是否可以由实时性单独解释；恢复 baseline 响应后，反事实模型是否改变结局。

## 2. 系统、干预和原始数据

实验环境为 CARLA 0.9.15 服务器端、Apollo 10.0.0 Orin 端，两者经网线连接，Bridge 位于服务器侧。本部署中 Bridge 直接读取 Control 命令，Guardian 虽有 Trace，但不是 Bridge 执行链的输入。这来自工作区配置说明，属于 A 类证据，不从 Guardian Trace 是否有输出反推。

递归盘点找到 7 个 baseline 和 5 个 300 ms run，与预期清单完全一致。每个 run 都有 Perception、Prediction、Planning、Localization、Trace 和 SCB 主要证据；碰撞事件和 actor history 只存在于 `1131`、`1643`。因此，碰撞 run 能使用 CARLA 多帧物理真值对齐目标身份；无碰撞 run 只能依据停车过程、投影净距与无 CollisionSensor 事件来判定观测结局，物理真值完整性不对称。

## 3. 统一指标和证据分层

t1 是目标首次连续 3 帧稳定 Fusion 观测中第一帧的源时刻，不是 Fusion 输出时刻。t2 是 Localization 中首个满足连续 2 个间隔减速度至少 0.5 m/s²，且后续 0.3 s 内速度至少下降 0.3 m/s 的区间终点。主响应 T_e2e=t2-t1。D1 为 t1 时车辆前缘到障碍物近表面的纵向投影净距，由中心距减去 5.3074 m 组合偏移得到。

D_delay 只用 t1到t2 车速对墙钟时间的梯形积分，D2=D1-D_delay。不使用 CARLA 帧数、sim time 或 Localization 空间位移替代主 D_delay。完整停车 run 的 D_brake,data 为 t2 到后续最小速度样本的 Localization 位移，同时保留近停、严格持续停车和路程积分作诊断。碰撞 run 只能观测到接触前截断制动，所以其完整制动距离、0 m/6 m 观测余量与观测 deadline 保持 NA。

证据分三类：A 类是系统配置；B 类是日志、Trace、SCB、Localization、CollisionSensor 和 actor history 直接测量；C 类是模型或反事实。主结果和逐 run 表只用 B 类观测证据；C 类结果单独报告，不回填观测缺失值。

## 4. 干预有效性

SCB 的 `BRIDGE_CONFIG_LOADED`、`INITIALIZED`和 `APPLIED` 生命周期在 12 个 run 中均完整。300 ms 主分析组实测墙钟延迟中位数 {d_scb.median():.3f} ms，范围 {d_scb.min():.3f}–{d_scb.max():.3f} ms；baseline 中位数 {b_scb.median():.3f} ms。`1031` 首个有效命令日志为 19.282 ms，其余 baseline 接近 0.07–0.09 ms。因此“注入 300 ms”已由独立墙钟日志验证，不是从目录名推断。

但 SCB 设置 `log_all_delayed_commands=0`，主要保存首次有效制动命令，没有完整归档每一帧延迟后的 Control 载荷。Trace 能证明目标 trace 到达 Control，Localization 能证明随后物理减速，但不能逐帧核验制动百分比。这不影响干预时长识别，但限制对 Bridge 内部逐帧行为的更强声明。

## 5. 组间结果：时间债务转化为距离债务

baseline 的 T_e2e 范围 {sv('baseline','T_e2e_data_observed_ms','min'):.3f}–{sv('baseline','T_e2e_data_observed_ms','max'):.3f} ms，300 ms 主分析组为 {sv('delay_300ms','T_e2e_data_observed_ms','min'):.3f}–{sv('delay_300ms','T_e2e_data_observed_ms','max'):.3f} ms，当前样本中两组范围完全分离。中位差 {d_lat-b_lat:.3f} ms 大于注入的 300 ms，说明 t2 还受模块周期、消息相位和物理减速识别窗口影响。Control 输出到 t2 的组中位数，baseline 为 {st('baseline','control_to_t2_ms'):.3f} ms，300 ms 组为 {st('delay_300ms','control_to_t2_ms'):.3f} ms，这段同时包含 SCB、Bridge/API 和动力学被识别为有效减速所需时间。

![组间端到端响应](../figures/group_e2e_response.png)

baseline 的 D_delay 范围 {sv('baseline','D_delay_wall_integral_data_observed_m','min'):.3f}–{sv('baseline','D_delay_wall_integral_data_observed_m','max'):.3f} m，300 ms 主分析组为 {sv('delay_300ms','D_delay_wall_integral_data_observed_m','min'):.3f}–{sv('delay_300ms','D_delay_wall_integral_data_observed_m','max'):.3f} m。组间中位差 {d_dd-b_dd:.3f} m 是闭环响应变慢的直接空间代价。D_delay 的增加与 D2 的减少不完全等值，因为各 run 的 D1 也有数米波动。这说明不能只看延迟或只看碰撞率，必须把起始距离和制动能力一并放入预算。

![组间距离债务](../figures/group_distance_debt.png)

![有效制动起点的剩余净距](../figures/group_braking_position.png)

## 6. 安全陡峭和隐形Deadline

完整停车 run 的 0 m 碰撞余量 M0=D1-D_delay-D_brake,data。baseline 中位数 {b_m0:.3f} m，范围 {sv('baseline','M_collision_0m_data_observed_m','min'):.3f}–{sv('baseline','M_collision_0m_data_observed_m','max'):.3f} m；300 ms 组两个安全停车 run `1202`、`1211` 只剩 {r1202.M_collision_0m_data_observed_m:.3f} m 和 {r1211.M_collision_0m_data_observed_m:.3f} m。碰撞 run 没有完整制动端点，图中红色交叉仅标出碰撞和实测冲击速度，其纵坐标 0 是视觉边界，不是伪造的碰撞余量。

![当前样本的安全陡峭](../figures/safety_cliff.png)

若要求停车后仍保留 6 m，则 M6=M0-6。baseline 7/7 的 M6 都为负，中位数 {sv('baseline','M_safety_6m_data_observed_m'):.3f} m。因此“baseline 无碰撞”只说明满足 0 m 几何接触边界，不代表满足 6 m 工程余量。若 6 m 是必须目标，当前起始距离与车速下即使不注入延迟也已不充足。

完整停车 run 的 0 m deadline 为 (D1-D_brake,data)/v1，6 m deadline 为 (D1-D_brake,data-6)/v1。这是 run-specific 的时间边界，会随 D1、v1、制动能力和安全余量要求改变，不是所有场景共用的固定毫秒阈值。

![车速隐形Deadline](../figures/deadline_margin.png)

![距离预算分解](../figures/distance_budget_decomposition.png)

## 7. 功能链与阶段时延

目标 Trace 的阶段时间由 monotonic clock 计算，通过 trace anchor 与 Fusion 源时刻对齐。源时刻到 Fusion 输出的组中位数，baseline 为 {st('baseline','sensor_to_fusion_ms'):.3f} ms，300 ms 主分析组为 {st('delay_300ms','sensor_to_fusion_ms'):.3f} ms。Fusion→Prediction、Prediction→Planning STOP、Planning STOP→Control 大多为数毫秒到十余毫秒。ground detection 到 lidar detection 完成率组中位数为 {st('baseline','ground_detection_completion_ratio'):.3f} 和 {st('delay_300ms','ground_detection_completion_ratio'):.3f}，处理耗时组中位数 {st('baseline','ground_to_detection_process_median_ms'):.3f} 和 {st('delay_300ms','ground_to_detection_process_median_ms'):.3f} ms。这不支持“整条功能链未运行”。

所有主分析 run 都有与 Fusion 一致的 Planning STOP 目标和 Control 输出 Trace。日志可见 speed optimizer primal infeasible 及随后 fallback 停车轨迹，但未发现 empty trajectory。这表明功能链产生了停车结果，但不说明它必然来得及。对物理安全而言，STOP 存在是必要条件，还必须与 t2 时剩余距离联合评估。

## 8. 案例 `1131`：实时性主导候选，但不是纯延迟实验

`1131` 的 T_e2e={r1131.T_e2e_data_observed_ms:.3f} ms，比同设置安全 run `1211` 慢 {cp('202607271131','T_e2e_data_observed_ms'):.3f} ms；D_delay={r1131.D_delay_wall_integral_data_observed_m:.3f} m，多 {cp('202607271131','D_delay_wall_integral_data_observed_m'):.3f} m；D2={r1131.D2_clear_data_observed_m:.3f} m，少 {abs(cp('202607271131','D2_clear_data_observed_m')):.3f} m。CollisionSensor 和 actor history 记录到 {r1131.impact_speed_data_observed_mps:.3f} m/s 的碰撞前车速。车辆确实制动，但制动开始时空间已被显著消耗。

![1131阶段耗时](../figures/case_1131_latency_breakdown.png)

![1131车速轨迹](../figures/case_1131_speed.png)

![1131 S–T轨迹](../figures/case_1131_st.png)

`1131` 目标 Fusion 输出最大间隔 {r1131.target_gap_max_ms:.3f} ms，碰撞时最后可用目标的源数据年龄 {cp('202607271131','target_source_age_at_case_matched_elapsed_ms','collision_value'):.3f} ms，在相同 t1 相对时刻比 `1211` 老 {cp('202607271131','target_source_age_at_case_matched_elapsed_ms'):.3f} ms。这不是目标从未出现，而是关键阶段更新变稀、源数据变老。同时 `1131` 的 D1 比 `1211` 小 {abs(cp('202607271131','D1_clear_data_observed_m')):.3f} m。因此定性应为“实时性主导候选的多因素碰撞”，不是 RT_ONLY_COLLISION：响应过晚是直接距离债务来源，Fusion 长空档和起始净距差异是必须保留的混杂因素。

![1131 Fusion时序与数据年龄](../figures/case_1131_fusion_timeline_age.png)

## 9. 案例 `1643`：更慢响应与更小起始净距共同作用

`1643` 的 T_e2e={r1643.T_e2e_data_observed_ms:.3f} ms，是主分析中最慢 run，比 `1211` 慢 {cp('202607271643','T_e2e_data_observed_ms'):.3f} ms。它的 D1={r1643.D1_clear_data_observed_m:.3f} m，小 {abs(cp('202607271643','D1_clear_data_observed_m')):.3f} m；D_delay={r1643.D_delay_wall_integral_data_observed_m:.3f} m，多 {cp('202607271643','D_delay_wall_integral_data_observed_m'):.3f} m；D2={r1643.D2_clear_data_observed_m:.3f} m，少 {abs(cp('202607271643','D2_clear_data_observed_m')):.3f} m。最终碰撞前车速 {r1643.impact_speed_data_observed_mps:.3f} m/s，高于 `1131`，与更小剩余制动距离一致。

![1643阶段耗时](../figures/case_1643_latency_breakdown.png)

![1643车速轨迹](../figures/case_1643_speed.png)

![1643 S–T轨迹](../figures/case_1643_st.png)

`1643` 最大 Fusion 输出间隔 {r1643.target_gap_max_ms:.3f} ms，明显小于 `1131`，所以不应把两个碰撞强行归结为同一个 Fusion gap 模式。但碰撞时源数据年龄 {cp('202607271643','target_source_age_at_case_matched_elapsed_ms','collision_value'):.3f} ms，在同一相对时刻比 `1211` 老 {cp('202607271643','target_source_age_at_case_matched_elapsed_ms'):.3f} ms。这说明“输出间隔”与“源数据年龄”必须分开：输出可以持续，但源帧仍可能变老。`1643` 同样是多因素碰撞，响应过晚、更小 D1 和当次制动能力共同决定了结局。

![1643 Fusion时序与数据年龄](../figures/case_1643_fusion_timeline_age.png)

## 10. 目标身份审计

两个碰撞 run 的 Planning STOP ID 与稳定 Fusion 目标一致。将 Apollo Fusion 目标与 CARLA actor history 的 `other` 行按墙钟时间对齐，并对 CARLA y 轴做坐标转换：`1131` 匹配 {int(id1131.matched_frame_count)} 帧，二维位置误差中位数 {id1131.position_error_median_m:.3f} m、p90 {id1131.position_error_p90_m:.3f} m；`1643` 匹配 {int(id1643.matched_frame_count)} 帧，中位数 {id1643.position_error_median_m:.3f} m、p90 {id1643.position_error_p90_m:.3f} m。两者都对应 CARLA actor 155，车型 `vehicle.lincoln.mkz_2020`，速度误差中位数为 0。考虑 Fusion 障碍物中心与 actor origin 的定义偏移，这支持“STOP 目标就是碰撞物体”的高置信身份链，但不宣称亚米级几何真值完全一致。

无碰撞 run 没有 actor history，其目标身份由 Planning STOP ID、静态 Prediction 和 Fusion 空间连续性确认，置信度为中高而不是高。这是原始归档范围造成的证据不对称，不能通过给无碰撞 run 填入模型 actor 位置来消除。

## 11. 反事实模型：恢复响应时机

本节属 C 类模型结果，不是实测结局。模型将碰撞 run 的 t2 恢复为 baseline 中位响应 {b_lat:.3f} ms，使用该 run 在更早时刻实际观测的车速与 t1到恢复t2的墙钟积分；制动能力保持该碰撞 run 从实际t2到接触的能量等效减速度。它是“保持当次制动能力，只恢复响应时机”的局部反事实，不包含新闭环反馈和控制重算。

`1131` 回收 {cf1131.response_distance_recovered_model_m:.3f} m，对观测接触边界的预测余量 +{cf1131.margin_to_observed_contact_restored_model_m:.3f} m，预测避碰。`1643` 回收 {cf1643.response_distance_recovered_model_m:.3f} m，但预测余量仍为 {cf1643.margin_to_observed_contact_restored_model_m:.3f} m，仍预测碰撞，冲击速度约 {cf1643.impact_speed_model_predicted_mps:.3f} m/s。对 `1131`，恢复实时性在模型中足以改变结局；对 `1643`，它能减轻后果，但起始净距和当次制动能力使其仍越过接触边界。这正是不能用“小于 700 ms 就安全”替代距离预算的原因。

## 12. `1206`的数据质量冲突

`1206` 不缺 t1/t2 或主要模块日志。重算得 T_e2e={r1206.T_e2e_data_observed_ms:.3f} ms、D_delay={r1206.D_delay_wall_integral_data_observed_m:.3f} m、D2={r1206.D2_clear_data_observed_m:.3f} m，且有近停与最小速度端点。若按稳定 Fusion 目标和 5.3074 m 偏移计算，停车端点投影净距 {r1206.final_clearance_projected_data_observed_m:.3f} m，是明显几何穿透；但没有 CARLA 碰撞事件，也没有 actor history 判断是传感器未触发、物理实体不同、目标漂移还是记录缺失。

因此它被标记为 `OUTCOME_UNCERTAIN_COLLISION_EVENT_ABSENT_BUT_FIXED_GEOMETRY_IMPLIES_OVERLAP`。时延和距离计算保留在逐 run 表，但不进入组统计、碰撞率和安全/碰撞对照。这比旧报告只写“数据异常”更清楚：它是结局不确定，不是无法解析。

## 13. 证据边界

主分析只有 7 与 4 个样本。最强证据是干预时长、T_e2e、D_delay与D2的方向一致性，而不是 p 值或高精度碰撞概率。本报告不进行小样本显著性过度解释，也不将 2/4 外推为生产碰撞率。无碰撞 run 没有 actor history；Control 载荷没有逐帧完整归档；D1 与最终净距使用 5.3074 m 组合几何偏移，接近 0 m 时应视为边界区间，不是毫米级真值。

反事实模型固定了碰撞 run 的能量等效减速度，未模拟提前制动引发的新闭环与轨迹变化。所以它只回答“时机改善的局部效应”，不能替代重新实验。两个碰撞案例的定性使用“主导候选”和“多因素”，不使用无法由当前数据支撑的唯一原因声明。

## 14. 综合讨论：当前证据能支持什么

第一，从干预到结局的证据链不依赖某一个派生指标。SCB 直接证明 300 ms 墙钟延迟已生效；目标 Trace 证明 Fusion、Prediction、Planning STOP 和 Control 已对同一目标完成处理；Localization 独立给出 t2、车速积分距离和制动过程；CollisionSensor 与 actor history 给出碰撞结局和物理目标身份。这四类证据的时间顺序和数值方向一致，使“干预增加距离债务”比单独比较碰撞率更可信。但证据链一致并不等于所有碰撞只有一个原因；D1、Fusion时序和制动能力的 run 间差异仍需纳入解释。

第二，300 ms 干预并不应机械地使 T_e2e 只增加 300 ms。t1 是源数据时刻，Control 是模块输出时刻，t2 是车辆物理减速被 Localization 识别的时刻；三者之间还横跨异步模块周期、消息相位、Bridge 释放时机、API 执行与制动建压。因此中位差约 450 ms 不能被称为“SCB 多注入了 150 ms”，它表明一个固定命令延迟在闭环中会与周期和物理识别共同变成更大的端到端差异。后续应使用更密的逐命令日志把这部分进一步分解。

第三，距离预算比时间阈值更接近安全问题本身。同一个 T_e2e 在 10 m/s 和 18 m/s 下对应的 D_delay 完全不同；同一个 D_delay 在 36 m 和 40 m 的 D1 下对 D2 的影响也不同；即使 D2 相同，制动起点速度和路面/制动能力不同也可以产生不同结局。因而比较 T_e2e 是实时性诊断，比较 D_delay 是时间到空间的转换，而完整安全判断还必须继续评估 D2、D_brake 和所需余量。这也是本报告在碰撞 run 中宁可保留 NA，也不用 baseline 制动距离伪造观测余量的原因。

第四，`1131` 和 `1643` 展示了两种不同的实时性风险形态。`1131` 在关键阶段出现超过 500 ms 的目标 Fusion 输出空档，且碰撞时源数据已超过 1 s，它将“更新稀疏”与“控制执行过晚”叠加在一起。`1643` 的最大输出间隔并不突出，但源数据仍累积到约 600 ms，同时 D1 更小、T_e2e 更长。因此线上监控不能只看“有没有 Fusion 输出”，也不能只看相邻输出间隔；还需监控源帧年龄和源帧是否真正更新。

第五，无碰撞并不意味安全裕量充足。baseline 全部停车，但其 0 m 余量只有数米，6 m 余量则全部为负。300 ms 组两个可确认安全停车 run 的 0 m 余量已经收窄到约 0.55–1.02 m。这种状态对几何偏移、车速波动和制动下降非常敏感，即使未观测到碰撞，也应当被归入“边界运行”而不是“宽裕安全”。工程上真正需要的不是让每个 run 恰好停在接触边界前，而是使距离预算对可预见的时延、制动和几何不确定性保留足够裕量。

第六，`1206` 的价值不只是“需要排除”。它证明当前归档体系中，“无 CollisionSensor 事件”不总能与“物理上无接触”等价。如果简单把它当作安全 run，会使 300 ms 组碰撞率与余量分布受到未知方向偏置；如果删除它的所有计算，又会丢掉对结局记录链最有价值的质量告警。当前“保留计算、排除结局推断”的做法，既不回避原始数据，也不把无法验证的结局强行放入主结论。

对系统设计而言，这组数据提示 deadline 不能只在单个模块内定义。Perception 或 Planning 即使各自满足局部周期，如果消息在下游排队、被 Bridge 延迟或经动力学响应后才转化为减速，仍可能越过物理 deadline。因此工程验收应同时设置模块局部延迟、源数据年龄、端到端 t1→t2 响应和距离余量四类指标，并以距离余量作为最终安全约束。

对运行时监控而言，可将 D2 看作一个不断被消耗的资源，而不是到 t2 才离线计算的结果。当系统已知目标大致距离、自车速度和保守制动距离时，可在每个周期估计“继续保持当前响应速度将多消耗多少米”，并在距离预算进入警戒区时提前触发降速或最小风险处置。这比仅在某一模块耗时超过固定毫秒数后报警更直接对应物理风险。

对测试设计而言，下一步不应只追加更多相同 300 ms run。如果目标是识别转换区，就需要在多个延迟水平上重复，同时将 D1 和 v1 控制在更窄窗口。如果目标是区分感知新鲜度与控制延迟，就需要设计一组只改变 Fusion 源数据更新、另一组只改变 Control 延迟的正交实验。当前数据能提出机制候选，但不能用非正交差异替代严格因果分解。

对报告与沟通而言，应优先报告“观测到什么”，再报告“模型预测什么”。本次两个碰撞 run 的 D_brake 和余量保持 NA，会使部分图表不如填满数字整齐，但这是证据边界的必要表达。相反，如果将 baseline 制动距离或反事实值写入碰撞 run 的“实测余量”，就会在数据层面混淆观测与模型，也会让结论看似更精确却实际更不可验证。

最后，本结果对“实时性”的理解也应从平均运行速度转向任务级可交付性。一个模块平均耗时很低，不代表每个关键目标都在距离预算内到达下游；一次闭环最终停下，也不代表它对参数波动有可接受的裕量。因此系统应同时管理延迟分布的尾部、目标源数据的新鲜度和当前车速下的剩余制动距离。只有当这三者被映射到同一个任务级预算中，“结果正确”才能进一步变成“结果及时且有余量”。这种任务级预算还应被带入回放、闭环仿真和车端监控的同一套验收口径，避免离线报告、仿真判据和在线告警各自使用不同的时间原点、距离定义和缺失值处理。只有口径一致，后续实验才能稳定地积累证据，并支持真正可执行的工程决策。

这也是形成可持续改进闭环的前提。

## 15. 下一轮实验建议

1. 开启 `log_all_delayed_commands`，逐帧保存 Control 接收、入队、出队、API 调用开始/结束、CARLA 帧与 wall clock，把 Control→Bridge→车辆从首次触发证据提升到逐命令可核验。
2. 所有 run 保存 actor history、障碍物 actor ID、碰撞体启用状态和 CollisionSensor 健康状态，直接消除 `1206` 类型冲突。
3. 设置 0/100/200/300/400 ms 多个延迟水平，增加重复，并更严格控制 D1、v1、障碍物 actor 和路面状态，建立距离预算响应面，而不是从 4 个 300 ms 样本推断单一阈值。
4. 在线监控 Fusion 源帧年龄、输出间隔、目标消失和 ID 切换，区分“仍在输出”与“使用新鲜源数据输出”。
5. 预先分开 0 m 碰撞边界与 6 m 工程安全余量。如果 6 m 是必须目标，应在起始距离、制动能力和 deadline 设计中预先绑定。

## 16. 结论

原始数据表明，Apollo 的 Fusion、Prediction、Planning STOP 和 Control 链在碰撞 run 中并非整体消失；真正决定物理结局的是这些结果何时转化为车辆有效减速。实测 300 ms 延迟使 T_e2e 中位数增加约 {d_lat-b_lat:.1f} ms，使 D_delay 增加约 {d_dd-b_dd:.2f} m，并把 t2 剩余净距压缩约 {b_d2-d_d2:.2f} m，与 baseline 0/7、300 ms 主分析组 2/4 碰撞的方向一致。

`1131` 是实时性主导候选的多因素碰撞，Fusion 长空档和 D1 差异使它不能作为纯延迟单因素证据；`1643` 是更慢响应、更小 D1 和当次制动能力共同作用的多因素碰撞。反事实模型预测恢复 baseline 响应可使 `1131` 避碰，却只能降低 `1643` 的冲击速度。因此，车速隐形Deadline不应被固定为普适 700 ms，而应是由起始净距、车速、实测制动能力和所需安全余量共同决定的距离预算边界。

## 附表：逐 run 观测主指标

{run_table(runs)}

> 碰撞 run 的完整 D_brake,data 与观测余量为 NA；`1206` 的计算仅作数据质量诊断，不进入主组统计。完整字段与数据源见 `../tables/run_level_metrics.csv`。
"""

    output = REPORT_DIR / "group_meeting_report.md"
    output.write_text(report, encoding="utf-8")
    han_count = len(re.findall("[一-龥]", report))
    if not 5000 <= han_count <= 8000:
        raise RuntimeError(f"Chinese character count {han_count} outside 5000-8000")

    pandoc = shutil.which("pandoc")
    if pandoc:
        subprocess.run(
            [pandoc, str(output), "-o", str(REPORT_DIR / "group_meeting_report.docx")],
            check=True,
            cwd=REPORT_DIR,
        )
    print(f"wrote {output}; Chinese characters={han_count}; pandoc={pandoc or 'not found'}")


if __name__ == "__main__":
    main()
