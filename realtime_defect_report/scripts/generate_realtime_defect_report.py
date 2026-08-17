#!/usr/bin/env python3
"""Generate the realtime-defect report family from recomputed tables."""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
REPORT = ROOT / "report"
VALIDATION = ROOT / "validation"


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
            outcome += f"，撞击前{fmt(row.impact_speed_data_observed_mps)} m/s"
        lines.append(
            f"| {row.run_id} | {'baseline' if row.group_name == 'baseline' else '300 ms'} | "
            f"{'是' if row.included_main_analysis else '否'} | {fmt(row.T_e2e_data_observed_ms)} | "
            f"{fmt(row.D1_clear_data_observed_m)} | {fmt(row.D_delay_wall_integral_data_observed_m)} | "
            f"{fmt(row.D2_clear_data_observed_m)} | {fmt(row.D_brake_data_observed_m)} | "
            f"{fmt(row.M_collision_0m_data_observed_m)} | {outcome} |"
        )
    return "\n".join(lines)


def baseline_clearance_table(frame: pd.DataFrame) -> str:
    lines = [
        "| baseline run | 最终投影净距/m | 0 m碰撞余量M0/m | 说明 |",
        "|---|---:|---:|---|",
    ]
    for _, row in frame[frame.group_name == "baseline"].iterrows():
        lines.append(
            f"| {row.run_id} | {fmt(row.final_clearance_projected_data_observed_m)} | "
            f"{fmt(row.M_collision_0m_data_observed_m)} | 完整停车、无碰撞事件 |"
        )
    return "\n".join(lines)


def perception_table(frame: pd.DataFrame) -> str:
    lines = [
        "| run | sensor→Fusion/ms | Fusion最大输出间隔/ms | lifecycle最大值/ms | 结局端源数据年龄/ms | 判断 |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for _, row in frame.iterrows():
        if row.run_id == "202607271131":
            finding = "连续性与新鲜度异常"
        elif row.run_id == "202607271206":
            finding = "端点年龄异常但结局不确定"
        elif row.run_id == "202607271643":
            finding = "单帧/间隔未超阈，碰撞端源数据偏老"
        else:
            finding = "未见首个稳定Fusion响应超过500 ms"
        lines.append(
            f"| {row.run_id} | {fmt(row.sensor_to_fusion_ms)} | {fmt(row.target_gap_max_ms)} | "
            f"{fmt(row.target_lifecycle_max_ms)} | {fmt(row.target_source_age_at_outcome_ms)} | {finding} |"
        )
    return "\n".join(lines)


def main() -> None:
    REPORT.mkdir(parents=True, exist_ok=True)
    runs = pd.read_csv(TABLES / "run_level_metrics.csv", dtype={"run_id": str})
    groups = pd.read_csv(TABLES / "group_summary.csv")
    stages = pd.read_csv(TABLES / "stage_latency_summary.csv")
    compare = pd.read_csv(
        TABLES / "collision_case_comparison.csv",
        dtype={"collision_run_id": str, "safe_control_run_id": str},
    )
    counter = pd.read_csv(TABLES / "counterfactual_model.csv", dtype={"run_id": str})
    identities = pd.read_csv(TABLES / "target_identity_audit.csv", dtype={"run_id": str})
    inventory = pd.read_csv(TABLES / "run_inventory.csv", dtype={"run_id": str})
    defects = pd.read_csv(TABLES / "realtime_defect_evidence_matrix.csv", dtype={"run_id": str})
    by_id = {row.run_id: row for _, row in runs.iterrows()}

    def sv(group: str, metric: str, stat: str = "median") -> float:
        return float(groups[(groups.group_name == group) & (groups.metric == metric)].iloc[0][stat])

    def st(group: str, metric: str, stat: str = "median") -> float:
        return float(stages[(stages.group_name == group) & (stages.metric == metric)].iloc[0][stat])

    def cp(case: str, metric: str, field: str = "collision_minus_control") -> float:
        return float(compare[(compare.collision_run_id == case) & (compare.metric == metric)].iloc[0][field])

    b_lat = sv("baseline", "T_e2e_data_observed_ms")
    d_lat = sv("delay_300ms", "T_e2e_data_observed_ms")
    b_debt = sv("baseline", "D_delay_wall_integral_data_observed_m")
    d_debt = sv("delay_300ms", "D_delay_wall_integral_data_observed_m")
    b_d2 = sv("baseline", "D2_clear_data_observed_m")
    d_d2 = sv("delay_300ms", "D2_clear_data_observed_m")
    b_m0 = sv("baseline", "M_collision_0m_data_observed_m")
    delay_safe_m0 = runs[(runs.group_name == "delay_300ms") & runs.included_main_analysis & ~runs.collision_event_data_observed].M_collision_0m_data_observed_m.dropna()
    b_scb = runs[(runs.group_name == "baseline") & runs.included_main_analysis].scb_actual_wall_delay_ms
    d_scb = runs[(runs.group_name == "delay_300ms") & runs.included_main_analysis].scb_actual_wall_delay_ms
    r1131, r1202, r1206, r1211, r1643 = (
        by_id[run_id]
        for run_id in ["202607271131", "202607271202", "202607271206", "202607271211", "202607271643"]
    )
    cf1131 = counter[counter.run_id == "202607271131"].iloc[0]
    cf1643 = counter[counter.run_id == "202607271643"].iloc[0]
    id1131 = identities[identities.run_id == "202607271131"].iloc[0]
    id1643 = identities[identities.run_id == "202607271643"].iloc[0]
    amp = (d_lat - b_lat) / 300.0

    report = f"""# 从受控时序故障到安全后果：Apollo闭环实时性缺陷的实验性发现、复现与归因

> 第二次“车速隐形deadline”实验组会技术报告｜原始数据统一重算｜版本日期：2026-08-04

## 摘要

本报告把研究问题限定为：如何发现、复现、量化和归因自动驾驶闭环中的实时性缺陷，以及这些缺陷怎样传播为车辆安全后果。实验在 CARLA 0.9.15 与 Apollo 10.0.0 闭环中向 Bridge 控制链施加名义 300 ms 延迟；分析从原始 Fusion、Prediction、Planning、Control Trace、Localization、SCB、CollisionSensor 与 actor history 重新构造证据，不把旧报告作为数值真值。12 个 run 均可解析，其中 11 个进入主分析，`202607271206` 因“无碰撞事件但固定几何推算穿透”被预先标记为结局不确定。

结论分三层。第一，干预被独立日志证实：300 ms 主分析组 SCB 实测墙钟延迟中位数为 {d_scb.median():.3f} ms；端到端响应中位数从 {b_lat:.3f} ms 增至 {d_lat:.3f} ms，增加 {d_lat-b_lat:.3f} ms，相对名义注入的闭环延迟放大系数为 {amp:.2f}。第二，时间缺陷转化为空间债务：墙钟速度积分的 `D_delay` 中位数增加 {d_debt-b_debt:.3f} m，有效制动开始净距 `D2` 中位数减少 {b_d2-d_d2:.3f} m；baseline 为 0/7 碰撞，300 ms 主分析组为 2/4 碰撞。第三，两起碰撞的归因不同：`1131` 是“实时性主导候选”，并伴随 Fusion 输出连续性/数据新鲜度退化；`1643` 是“多因素碰撞”，更慢响应、更小起始净距和当次制动过程共同作用。当前样本只显示 300–400 ms 仍能以数米余量停车、699–700 ms 进入 0.55–1.02 m 临界停车、799–894 ms 出现碰撞的转换区，不能把 700 ms 宣称为普适硬阈值。

![实时性故障传播链](../figures/realtime_fault_propagation_chain.png)

## 1. 研究定位与方法借鉴

本文不是 Apollo 性能优化报告，不提出调度器、在线控制器或新的制动算法。研究对象是已经暴露的时序缺陷及其后果链：受控故障注入是否真实进入执行链，闭环响应是否改变，改变是否被系统传播或放大，车辆是否在等待期间继续行驶，距离债务是否推迟有效制动位置，最终是否压缩安全余量并导致临界停车、功能退化或碰撞。

方法上仅借鉴任务说明中给出的四类思路：用 DriveFI 式受控故障注入保持干预可复现；用 R-TOD 式端到端时序视角避免只看单模块平均耗时；用 FADE 式“时间质量退化—功能表现退化”链审查数据新鲜度和输出连续性；用 D3 式距离预算把毫秒翻译为车辆继续行驶的米数。本文不复现这些工作的模型或算法，也不把它们当作本实验的直接证据；本实验结论仍只由本地原始数据支撑。

## 2. 系统、样本与证据层级

实验部署为 CARLA 0.9.15 服务器端、Apollo 10.0.0 Orin 端，Bridge 位于服务器侧，双方经网线连接。当前部署中 Bridge 直接读取 Control 命令，Guardian 虽有 Trace，但不在车辆命令的实际执行链内，因此不把 Guardian 输出时间加入主 `T_e2e`。这一点来自工作区部署说明，属于 A 类配置证据。

递归盘点找到 454 个文件、7 个 baseline run 和 5 个 300 ms run。原始日志、Trace、SCB、CollisionSensor 和 actor history 为最高优先级；本报告脚本从它们复算；旧 CSV/JSON 只用于差异检查，旧报告文字不参与主结果生成。配置、实测和模型严格分层：约 560k 点云、名义注入量和队列设定属于 A 类实验设定；逐 run 时刻、距离、制动和碰撞属于 B 类直接观测；恢复 baseline 响应的结果属于 C 类模型。对点云数和配置队列长度，必须保留以下边界：该项来源于实验设定，当前归档中缺少独立配置文件快照。

证据盘点还揭示了物理真值不对称。12个run都有主要的Localization、Perception、Prediction、Planning、Control Trace和SCB证据，但只有`1131`、`1643`归档了CollisionSensor事件与actor history。对这两个碰撞run，可以把Apollo目标与CARLA碰撞对象进行跨系统身份核验；对停车run，只能证明功能链输出了同一静态目标的STOP、车辆随后减速并且没有归档到碰撞事件，不能假装拥有同等级别的CARLA多帧真值。报告因此把“无碰撞事件”“投影最终净距”“完整停车端点”三项合在一起描述停车结局，并把证据置信度写成中高而不是绝对物理真值。

Control和Guardian也必须区分。当前Bridge执行的是Control命令，Guardian Trace仅说明另一个模块在系统中产生活动，不证明它的命令被Bridge采用。如果把Guardian时间加入执行链，不仅会改变端点定义，也会造成“日志中存在”被误读为“物理上生效”。本报告的响应链固定为目标源时间、Fusion、Prediction、Planning STOP、Control、SCB/Bridge和Localization有效减速；所有图表与逐run表都遵守同一链路。

## 3. 可复现指标与时钟边界

`t1` 定义为目标首次连续 3 个稳定 Fusion 周期中第一帧的传感器源时间；`t2` 定义为 Localization 中首次满足“连续两个区间减速度至少 0.5 m/s²，并在后续 0.3 s 内下降至少 0.3 m/s”的第一个合格区间终点。端到端响应为 `T_e2e=t2−t1`。`D1_clear` 是 `t1` 时车辆前缘到障碍物近表面的纵向净距，按中心距减 5.3074 m 组合偏移得到。

主响应距离严格按墙钟速度梯形积分：

`D_delay = ∫[t1,t2] v(t) dt_wall`，`D2 = D1_clear − D_delay`。

不同 run 不混用 CARLA 仿真帧数、sim time 或 Localization 空间位移。完整停车 run 的 `D_brake,data` 采用 `t2` 后达到低速条件的统一最小速度端点，并保留近停、严格持续停车和墙钟路程积分诊断。碰撞 run 没有完整停车端点，所以其完整 `D_brake`、0 m/6 m观测余量和观测 deadline 一律为 NA，只报告 `t2→碰撞` 的截断制动时间、截断距离、撞击前速度及投影净距诊断。

这些定义专门避免三类常见错位。其一，`t1`取传感器源时间而不是Fusion输出时间，否则感知生命周期会被从端到端响应中删除；连续三帧条件用于排除一次性误检。其二，`t2`不取第一条Control制动命令，因为命令产生、Bridge延迟、车辆执行和Localization确认之间仍有真实闭环时间；持续减速条件可避免速度噪声把单点下降误判为有效制动。其三，主`D_delay`不取`t1`与`t2`两点位置之差，因为不同run的CARLA推进帧数和实时因子不一致，位置差与墙钟等待距离混用会破坏组间可比性。

最终净距和碰撞余量也不是同一个字段的两个名字。最终投影净距使用统一停车端点处的稳定目标几何；`M0`使用`D1−D_delay−D_brake,data`的预算链。两者接近说明距离链自洽，出现小差异则来自端点、三维位移和纵向投影定义。对于碰撞run，两者都不能替代不存在的完整停车距离。模型即使可以估计“若继续以等效减速度制动需要多远”，也必须留在预测表，不能回填为观测`D_brake`。

## 4. 干预是否真实生效，以及为何增量超过300 ms

SCB 的配置加载、初始化、触发和应用记录在 12 个 run 中均存在。300 ms 主分析组实测墙钟延迟中位数 {d_scb.median():.3f} ms，范围 {d_scb.min():.3f}–{d_scb.max():.3f} ms；baseline 中位数 {b_scb.median():.3f} ms，其中 `1031` 首次有效命令为 19.282 ms，其余约 0.067–0.091 ms。因此干预有效性来自 SCB 日志，不是从目录名反推。

![逐run端到端响应](../figures/e2e_response_by_run.png)

baseline 的 `T_e2e` 中位数 {b_lat:.3f} ms、范围 {sv('baseline','T_e2e_data_observed_ms','min'):.3f}–{sv('baseline','T_e2e_data_observed_ms','max'):.3f} ms；300 ms 主分析组中位数 {d_lat:.3f} ms、范围 {sv('delay_300ms','T_e2e_data_observed_ms','min'):.3f}–{sv('delay_300ms','T_e2e_data_observed_ms','max'):.3f} ms。中位增量 {d_lat-b_lat:.3f} ms 大于名义 300 ms。这里的额外约 {d_lat-b_lat-300:.3f} ms 不应被强行归入某一个 Apollo 模块：`t2` 还包含消息相位、模块周期、SCB/Bridge行为、车辆动力学以及减速端点识别窗口。可复现且审慎的结论是“闭环端点出现约 {amp:.2f} 倍的延迟放大”，不是“某模块额外阻塞149.5 ms”。

阶段统计进一步限定了放大发生在哪里。sensor→Control的组中位数从 {st('baseline','sensor_to_control_ms'):.3f} ms变为 {st('delay_300ms','sensor_to_control_ms'):.3f} ms，只增加约 {st('delay_300ms','sensor_to_control_ms')-st('baseline','sensor_to_control_ms'):.3f} ms；Control→t2的组中位数则从 {st('baseline','control_to_t2_ms'):.3f} ms变为 {st('delay_300ms','control_to_t2_ms'):.3f} ms。这说明名义干预主要进入Control之后的执行与物理响应段，符合Bridge延迟注入位置。但是Control→t2并不是“纯Bridge耗时”：它同时包含实测SCB等待、命令到车辆、执行器/车辆动力学变化以及有效减速识别。因此报告只把它定位为传播区段，不把整段贴成Bridge内部处理延迟。

另外，300 ms干预并未使上游功能链停止。各run都有Planning STOP和Control目标Trace，未观察到empty trajectory；日志中的速度优化不可行与fallback表明系统仍能生成停车轨迹。这个证据很重要：本实验暴露的不是简单的“功能没有输出”，而是输出存在但在有限距离预算内到达得太晚。实时性缺陷与功能缺失的测试判据因此必须分开。

![延迟放大](../figures/latency_amplification.png)

## 5. 时间缺陷如何传播为空间后果

baseline `D_delay` 中位数 {b_debt:.3f} m，300 ms 主分析组为 {d_debt:.3f} m，中位增加 {d_debt-b_debt:.3f} m；`D2` 中位数从 {b_d2:.3f} m 降至 {d_d2:.3f} m，减少 {b_d2-d_d2:.3f} m。两个变化不是严格相等，因为不同 run 的 `D1` 和速度也在波动。这正说明时延必须被放入“起始距离—响应距离—制动距离”的共同预算，而不是单独拿毫秒判断安全。

![逐run距离债务](../figures/distance_debt_by_run.png)

![逐run有效制动位置](../figures/braking_position_by_run.png)

baseline 7 个 run 均完整停车、无碰撞事件。它们最终距障碍物的投影净距如下，范围 2.735–5.331 m；按统一制动端点计算的 0 m碰撞余量范围 2.740–5.338 m，中位数 {b_m0:.3f} m。两列只存在厘米级端点/计算差异，不应与 6 m工程安全余量混为一谈。

{baseline_clearance_table(runs)}

300 ms 组的可靠安全 run `1202`、`1211` 只剩 {r1202.M_collision_0m_data_observed_m:.3f} m 和 {r1211.M_collision_0m_data_observed_m:.3f} m 观测余量，中位数 {delay_safe_m0.median():.3f} m；`1131`、`1643` 分别以 {r1131.impact_speed_data_observed_mps:.3f} m/s 和 {r1643.impact_speed_data_observed_mps:.3f} m/s 撞击。由此可见，受控时序故障没有让功能链完全消失，而是把原有数米余量压缩为临界停车或碰撞。

这里还存在一个看似矛盾、实际非常关键的结果：baseline虽然全部无碰撞，但其6 m工程余量均为负。0 m边界回答“是否与障碍物几何接触”，6 m边界回答“停车后是否仍保留指定工程空间”；两者不能混写成“安全/不安全”一个词。baseline的M0中位数为{b_m0:.3f} m，只支持“本批次能在接触前停下”，不支持“满足6 m安全目标”。如果后续把6 m设为硬要求，那么问题不仅是300 ms干预，当前速度与起始距离本身也需要重新设计。

距离预算还解释了为什么相近时延可以出现不同结局。`1202`与`1211`约699–700 ms仍停车，而`1131`约800 ms碰撞；但`1131`相对`1211`不仅晚99.469 ms，起始净距还小1.944 m，且Fusion连续性不同。`1643`比`1211`晚193.703 ms，同时D1小3.551 m。因此时延是重要轴，却不是唯一状态变量；所谓“安全陡峭”是当前D1、速度、制动能力共同形成的转换带。

![当前样本的安全陡峭](../figures/realtime_safety_cliff.png)

## 6. 感知模块是否存在时延过长

答案必须区分三种量。第一，`t1` 对应稳定目标的 sensor→Fusion 首次输出时延在全部12个 run 中为 208.315–319.285 ms，没有 run 超过归档分析配置中的 500 ms degraded threshold，因此没有证据支持“12组普遍存在单帧感知处理超过500 ms”。第二，输出间隔代表连续性；第三，Fusion 输出时目标源时间到当前输出/碰撞端点的年龄代表数据新鲜度。后两者可能异常，即使单次 sensor→Fusion 看起来正常。

ground detection→lidar detection 的 Trace 队列中位数在两组约 0.13–0.14 ms，处理段中位数约 92.18–92.40 ms，完成率中位数约 0.971–0.976。SCB 的 `queue_depth_at_trigger=1` 则是另一条命令延迟队列。两套队列已按 trace_id 和数据来源分别计算，不能把 lidar检测输入队列与 SCB命令队列合并解释；配置长度1仍属于实验设定，而非从配置快照重新验证的事实。

`1131` 是明确异常：Fusion 目标生命周期最大值 {r1131.target_lifecycle_max_ms:.3f} ms，最大输出间隔 {r1131.target_gap_max_ms:.3f} ms；碰撞时最后目标源数据年龄 {cp('202607271131','target_source_age_at_case_matched_elapsed_ms','collision_value'):.3f} ms，而 `1211` 在相同相对时刻为 {cp('202607271131','target_source_age_at_case_matched_elapsed_ms','safe_control_value'):.3f} ms。`1643` 的最大生命周期 {r1643.target_lifecycle_max_ms:.3f} ms、最大输出间隔 {r1643.target_gap_max_ms:.3f} ms，不呈现 `1131` 式长空档；但碰撞时源数据年龄 {cp('202607271643','target_source_age_at_case_matched_elapsed_ms','collision_value'):.3f} ms，相同相对时刻的 `1211` 为 {cp('202607271643','target_source_age_at_case_matched_elapsed_ms','safe_control_value'):.3f} ms。故 `1131` 可归入“数据新鲜度/输出连续性退化”，`1643` 只能说碰撞端可用目标偏老，不能说单帧感知处理超过阈值。

结局端源数据年龄需要特别防止误读。停车完成可能发生在目标停止更新之后很久，因而某些安全run在最终端点也出现较大的年龄，例如`1048`约2.8 s；这并不自动意味着接近障碍物的关键响应阶段发生了2.8 s处理超时。判断感知实时性至少需要同时查看首次稳定响应、相邻输出间隔、每次输出生命周期，以及与碰撞或同相对时刻对照的源数据年龄。只有这些信号在关键窗口相互印证时，才把它归类为数据新鲜度或连续性缺陷。

从这一判据看，`1131`的证据最完整：输出间隔越过500 ms，生命周期也越过500 ms，碰撞端源年龄又显著高于匹配时刻的安全对照。`1643`只满足最后一项，其输出间隔和生命周期均未越过配置阈值，所以应保留为次要风险信号。`1206`最终源年龄约2.4 s，但其结局本身冲突，而且前段单帧时延与输出间隔正常，故不把它升级为“感知处理超时”。

## 7. 案例1131：实时性主导候选，而非纯时延唯一致撞

![1131因果链](../figures/case_1131_causal_chain.png)

`1131` 的 `T_e2e={r1131.T_e2e_data_observed_ms:.3f} ms`，比可靠同设置安全对照 `1211` 慢 {cp('202607271131','T_e2e_data_observed_ms'):.3f} ms；`D_delay={r1131.D_delay_wall_integral_data_observed_m:.3f} m`，多 {cp('202607271131','D_delay_wall_integral_data_observed_m'):.3f} m；`D2={r1131.D2_clear_data_observed_m:.3f} m`，少 {abs(cp('202607271131','D2_clear_data_observed_m')):.3f} m。`t2` 后到碰撞持续 {r1131.T_brake_truncated_to_collision_data_observed_s:.3f} s，截断制动距离 {r1131.D_brake_truncated_to_collision_data_observed_m:.3f} m，撞击前速度 {r1131.impact_speed_data_observed_mps:.3f} m/s。固定几何墙钟投影在碰撞端为 {r1131.collision_clearance_projected_diagnostic_m:.3f} m，只是穿透方向的诊断量；CollisionSensor事件才是碰撞结局的主要证据。

`1131` 同时具有 {r1131.target_gap_max_ms:.3f} ms Fusion 长空档和 {r1131.target_lifecycle_max_ms:.3f} ms 生命周期峰值，而且 `D1` 比 `1211` 小 {abs(cp('202607271131','D1_clear_data_observed_m')):.3f} m。反事实模型把响应恢复到 baseline 中位数时，预测回收 {cf1131.response_distance_recovered_model_m:.3f} m，余量 +{cf1131.margin_to_observed_contact_restored_model_m:.3f} m并避碰；但这是 C 类模型，不是观测事实。综合直接链和混杂因素，分类为 `RT_DOMINATED_COLLISION`，置信度中高；边界是“实时性为主要候选机制”，不能写成 `RT_ONLY_COLLISION`。

![1131 Fusion时间线](../figures/case_1131_fusion_timeline.png)

## 8. 案例1643：实时性放大风险的多因素碰撞

![1643因果链](../figures/case_1643_causal_chain.png)

`1643` 的 `T_e2e={r1643.T_e2e_data_observed_ms:.3f} ms`，比 `1211` 慢 {cp('202607271643','T_e2e_data_observed_ms'):.3f} ms；`D1={r1643.D1_clear_data_observed_m:.3f} m`，比对照小 {abs(cp('202607271643','D1_clear_data_observed_m')):.3f} m；`D_delay={r1643.D_delay_wall_integral_data_observed_m:.3f} m`，多 {cp('202607271643','D_delay_wall_integral_data_observed_m'):.3f} m；`D2={r1643.D2_clear_data_observed_m:.3f} m`，少 {abs(cp('202607271643','D2_clear_data_observed_m')):.3f} m。`t2` 后到碰撞仅 {r1643.T_brake_truncated_to_collision_data_observed_s:.3f} s，截断制动距离 {r1643.D_brake_truncated_to_collision_data_observed_m:.3f} m，撞击前速度 {r1643.impact_speed_data_observed_mps:.3f} m/s，碰撞端几何投影诊断为 {r1643.collision_clearance_projected_diagnostic_m:.3f} m。

`1643` 没有 `1131` 式输出长空档，所以不能复制同一根因标签。反事实恢复 baseline 响应可回收 {cf1643.response_distance_recovered_model_m:.3f} m，但预测余量仍为 {cf1643.margin_to_observed_contact_restored_model_m:.3f} m、预测撞击速度 {cf1643.impact_speed_model_predicted_mps:.3f} m/s。故分类为 `MULTI_FACTOR_COLLISION`：实时性缺陷明确增加距离债务并放大碰撞严重度，但更小起始净距和当次制动能力差异同样不可忽略。

![1643数据新鲜度](../figures/case_1643_data_freshness.png)

## 9. 1206：为什么必须排除，而不是称为“坏数据”

`1206` 的 t1/t2、Localization、Fusion、Prediction、Planning、Control Trace 与 SCB 均完整：`T_e2e={r1206.T_e2e_data_observed_ms:.3f} ms`，`D_delay={r1206.D_delay_wall_integral_data_observed_m:.3f} m`，`D2={r1206.D2_clear_data_observed_m:.3f} m`。问题出在结局：归档没有 CollisionSensor 事件和 actor history，但固定目标几何与停车端点给出最终投影净距 {r1206.final_clearance_projected_data_observed_m:.3f} m、M0={r1206.M_collision_0m_data_observed_m:.3f} m，明显越过约 ±0.52 m的接触偏移不确定性范围。

因此它不是“无法解析”，而是 `OUTCOME_UNCERTAIN_COLLISION_EVENT_ABSENT_BUT_FIXED_GEOMETRY_IMPLIES_OVERLAP`。它的时延和距离指标仍进入质量诊断，但不进入11-run主结局比较；这项排除基于证据冲突，不基于结果方向。`1206` 的响应比 `1211`更接近`1131`，但它不能被当作安全对照，这也是选择`1211`进行碰撞案例比较的原因。

## 10. 缺陷分类、身份链与结论边界

缺陷分类表把直接证据、反证/混杂和置信度放在同一行：`1131=RT_DOMINATED_COLLISION`；`1643=MULTI_FACTOR_COLLISION`；`1206=INDETERMINATE`；`1202`与`1211`是非碰撞的 `REALTIME_MARGIN_EXHAUSTION` 观测。分类描述的是本次证据链，不是对Apollo模块的永久标签。

分类判据如下。`RESPONSE_TOO_LONG`要求统一`t1/t2`下响应相对基线显著右移，并能用墙钟积分观察到额外距离债务；`DATA_FRESHNESS/OUTPUT_CONTINUITY_DEGRADATION`要求关键窗口出现生命周期、输出间隔或匹配端源年龄异常；`TIMING_INDUCED_FUNCTIONAL_DEGRADATION`只有在时间异常先发生、随后功能输出质量退化且替代解释被排除时才成立；`MULTI_FACTOR_COLLISION`用于时延链明确存在，但起始距离、制动或其他功能信号也足以改变结局解释；`INDETERMINATE`用于关键结局证据相互冲突。当前数据没有足够证据把任何碰撞写成严格`RT_ONLY_COLLISION`。

对照run的选择也遵守这一判据。`1206`的响应更接近`1131`，表面上可能是更好的时延匹配，但它没有可信安全结局，不能作为“未碰撞”反证。`1202`和`1211`均为可靠停车run，其中`1211`在速度、起始距离和阶段响应上更适合作为共同对照。即使如此，案例比较仍是准配对而非随机对照，所以报告使用“候选”“共同作用”“放大风险”等措辞，不把差值直接等同于因果效应。

两个碰撞 run 的 Planning STOP ID 与稳定 Fusion 目标一致，并与 CARLA actor history 中的 `other` 车辆按墙钟对齐。`1131` 匹配 {int(id1131.matched_frame_count)} 帧，位置误差中位数 {id1131.position_error_median_m:.3f} m；`1643` 匹配 {int(id1643.matched_frame_count)} 帧，中位数 {id1643.position_error_median_m:.3f} m。两者都对应 actor 155、`vehicle.lincoln.mkz_2020`。这支持“被规划停车的目标就是碰撞对象”，但 Fusion中心与actor origin存在定义偏移，不宣称亚米级几何一致。

![逐run结局时间线](../figures/outcome_timeline.png)

本实验的最强结论是：受控300 ms命令时序故障在闭环端被传播并放大，形成约7.43 m额外距离债务，使有效制动位置整体后移，并在当前场景预算下把原有数米停车余量压缩为临界停车或碰撞。最强归因边界是：`1131`可称实时性主导候选，`1643`必须称多因素，`1206`必须保持不确定；“小于700 ms必安全”“两起碰撞都是感知计算超时”“300 ms以外的全部增量来自某单一模块”都不被当前证据支持。

## 11. 局限与下一轮最小验证

样本只有11个主分析run，速度、D1与制动能力未完全配对，当前转换区不能外推为通用deadline。无碰撞run缺少actor history，物理真值完整性弱于碰撞run；SCB未归档每一帧延迟后Control载荷；点云和队列配置缺少独立快照；碰撞run没有完整停车端点，反事实只能使用局部等效制动模型。

下一轮最小验证应优先补证据而非改系统：固定D1和初速度，在600–900 ms区间增加重复点；所有run都归档CollisionSensor、actor history与完整SCB载荷；保存感知与Bridge配置快照；对Fusion输出间隔、源数据年龄和端到端响应设置独立打点。这样才能区分响应过长、数据新鲜度和时序诱发功能退化，并把“候选归因”推进到可重复的机制归因。

建议把下一轮实验拆成三组最小矩阵。第一组只扫描Bridge延迟，保持D1、速度和车辆控制参数固定，用于估计响应增量到距离债务的稳定映射；第二组固定Bridge延迟，独立改变Fusion输出间隔或源数据年龄，用于判断新鲜度退化是否会改变Planning STOP与Control到达；第三组在碰撞转换区做重复实验，所有run保留同等CARLA真值，用于估计同条件下的结局概率。只有把三个因素拆开，才能把本报告的多因素链转化为可识别的机制模型。

验证时还应预先声明排除标准和主端点。排除只允许基于证据缺失或冲突，不能根据是否碰撞决定；主端点仍应是统一墙钟`t1/t2/D_delay/D2`，感知新鲜度和sim frame只作为独立诊断。报告模板也应固定“观测结果”和“模型预测”两栏，防止后续为了填满表格而用反事实补观测缺口。

## 逐run主结果

{run_table(runs)}

> 完整证据、字段定义、逐run感知表、反事实与文件来源见 [evidence_appendix.md](evidence_appendix.md)。
"""

    speech = f"""# 10–12分钟组会讲稿

各位老师、同学，今天汇报的不是Apollo优化方案，而是一项实时性缺陷研究：我们怎样通过受控故障注入，发现并复现闭环时序问题，再把毫秒级缺陷量化成车辆继续前进的距离，最后判断它在临界停车和碰撞中到底扮演什么角色。

先说结论。300 ms延迟确实进入了Bridge控制链。SCB实测中位数是{d_scb.median():.3f} ms，但端到端响应不是只增加300 ms，而是从baseline中位{b_lat:.3f} ms增加到{d_lat:.3f} ms，增量{d_lat-b_lat:.3f} ms，放大系数约{amp:.2f}。这段增量不能简单归给某个模块，因为t2包含消息相位、模块周期、Bridge、车辆动力学和减速识别窗口。

真正影响安全的是空间代价。我们用车辆速度对墙钟时间做梯形积分，得到响应阶段距离债务D_delay。它的中位数从{b_debt:.3f} m增加到{d_debt:.3f} m，多了{d_debt-b_debt:.3f} m；有效制动开始时的剩余净距D2减少{b_d2-d_d2:.3f} m。baseline 7次都停住，最终净距是2.735到5.331 m；300 ms主分析组4次中2次碰撞，两个安全停车只剩0.548和1.018 m余量。所以缺陷传播链是：受控延迟，闭环响应变慢，车辆继续行驶，距离债务增加，制动位置后移，安全余量被压缩。

这里必须强调，当前数据只显示一个样本转换区：300到400 ms时仍有数米余量，699到700 ms时只剩约半米到一米，799到894 ms发生碰撞。它不是通用700 ms阈值，因为每个run的起始净距、速度和制动能力不完全一致。

第二个问题是感知模块有没有时延过长。若看首个稳定目标的sensor到Fusion时延，12个run在208到319 ms之间，都没有超过归档配置的500 ms阈值，所以不能说12组普遍存在单帧感知计算超时。但如果看数据新鲜度和输出连续性，1131明显异常：Fusion最大输出间隔507.439 ms，生命周期最大705.892 ms，碰撞时最后目标源数据年龄1006.892 ms。1643没有这种长空档，最大间隔只有115.906 ms，但碰撞时源数据年龄599.668 ms。因此两起碰撞不能用同一个“感知慢”标签概括。

再看1131。它的端到端响应799.636 ms，比可靠安全对照1211慢99.469 ms；D_delay多1.433 m，D2少3.376 m；随后制动2.107秒、行驶27.148 m后，以7.988 m/s碰撞。它还伴随Fusion连续性退化，D1又比1211小1.944 m。恢复baseline响应的反事实模型预测可以回收8.565 m并避碰，但这只是模型。我们把它归类为实时性主导候选，不能写成纯时延唯一致撞。

1643不同。它的响应893.870 ms，比1211慢193.703 ms；起始净距小3.551 m；距离债务多3.451 m；D2少7.002 m。t2后只制动1.599秒、行驶23.378 m，就以11.728 m/s碰撞。它的Fusion输出间隔不异常。反事实恢复baseline响应可回收10.519 m，但模型仍预测以3.980 m/s接触。因此1643是多因素碰撞：实时性放大风险和后果，却不能单独解释结局。

1206要单独讲。它不是缺日志，时延和距离指标都能算；冲突是没有碰撞事件和actor history，但固定几何却推算最终净距为负1.627 m。这超出接触偏移不确定性，所以它的结局不能定性。我们把它标为不确定并排除主分析，但仍保留诊断指标。这也解释了为什么碰撞案例选1211而不是更接近1131时延的1206作为安全对照。

最后总结。本文最强的发现不是“某模块慢”，而是完整的安全传播证据：300 ms受控时序故障被闭环放大到约450 ms，形成约7.43 m额外距离债务，使制动起点净距减少约7.14 m，并在当前样本中把数米停车余量压缩为临界停车或碰撞。归因上，1131是实时性主导候选，1643是多因素，1206不确定。下一轮最重要的是固定初始距离和速度、增加600到900 ms重复点，并让所有run都有CollisionSensor、actor history、完整SCB载荷和配置快照。这样才能从“证据支持的候选归因”推进到真正可重复的机制归因。谢谢。
"""

    one_page = f"""# 实时性缺陷实验：单页摘要

## 研究问题

受控时序故障如何被Apollo—Bridge—车辆闭环传播和放大，并转化为距离债务、制动位置后移、余量压缩与碰撞？

## 样本与口径

- 7个baseline、5个300 ms；12个均解析，`1206`结局证据冲突，11个进入主分析。
- `T_e2e=t2−t1`；`D_delay`为速度对墙钟时间的梯形积分；`D2=D1−D_delay`。
- 碰撞run的完整制动距离、观测余量和deadline保持NA；模型结果单列。

## 核心结果

| 指标 | baseline | 300 ms主分析 | 变化 |
|---|---:|---:|---:|
| T_e2e中位数 | {b_lat:.3f} ms | {d_lat:.3f} ms | +{d_lat-b_lat:.3f} ms |
| D_delay中位数 | {b_debt:.3f} m | {d_debt:.3f} m | +{d_debt-b_debt:.3f} m |
| D2中位数 | {b_d2:.3f} m | {d_d2:.3f} m | −{b_d2-d_d2:.3f} m |
| 碰撞 | 0/7 | 2/4 | 当前样本转换区 |

- baseline最终净距：2.735–5.331 m；300 ms安全run余量：0.548、1.018 m。
- 延迟放大系数约{amp:.2f}，但额外增量不能强行归因于单一模块。
- sensor→Fusion为208.315–319.285 ms，未见12组普遍超过500 ms；`1131`存在507.439 ms输出空档和705.892 ms生命周期峰值。

## 缺陷归因

- `1131`：`RT_DOMINATED_COLLISION`，实时性主导候选并伴随Fusion连续性/新鲜度退化；不是纯时延唯一致撞。
- `1643`：`MULTI_FACTOR_COLLISION`，响应最慢、D1更小、制动过程不同；实时性放大风险和严重度。
- `1206`：`INDETERMINATE`，无碰撞事件但固定几何推算穿透，排除主结局分析。

## 不能下的结论

不能宣称通用700 ms阈值，不能说两起碰撞都是感知计算超时，不能把碰撞run模型制动距离写成实测值，也不能把Guardian放入当前Bridge执行链。
"""

    appendix = f"""# 证据附录

## A. 数据和脚本

- 原始目录：`../../第二次实验/`，454个文件。
- 全量文件清单：`../extracted/file_inventory.csv`；模式清单：`../extracted/schema_inventory.json`。
- 逐run清单：`../tables/run_inventory.csv`；观测主表：`../tables/run_level_metrics.csv`。
- 组统计：`../tables/group_summary.csv`；阶段统计：`../tables/stage_latency_summary.csv`。
- 案例对照：`../tables/collision_case_comparison.csv`；缺陷分类：`../tables/realtime_defect_evidence_matrix.csv`。
- 身份审计：`../tables/target_identity_audit.csv`；模型反事实：`../tables/counterfactual_model.csv`。
- 复算脚本：`../scripts/analyze_realtime_defects.py`；成文脚本：`../scripts/generate_realtime_defect_report.py`。

证据优先级为原始数据 > 可复算结果 > 旧CSV/JSON > 旧报告。主分析从原始数据重算；旧产物仅做差异核验。

## B. 事件、距离和端点定义

1. `t1`：首次连续3个稳定Fusion周期中第一帧目标的source time。
2. `t2`：首个满足连续两个间隔减速度≥0.5 m/s²且后0.3 s速度下降≥0.3 m/s的区间终点。
3. `D1_clear=D1_center−5.3074 m`。
4. `D_delay=∫v(t)dt_wall`，只用墙钟Localization速度梯形积分。
5. `D2=D1_clear−D_delay`。
6. 完整停车run的`D_brake,data`取统一低速最小速度端点；碰撞run只保留截断距离/时间。
7. `M0=D2−D_brake,data`；`M6=M0−6 m`。只对完整停车run计算观测deadline。
8. sensor→Fusion、Fusion→Prediction、Prediction→Planning STOP、Planning STOP→Control使用目标trace和单调时钟；Control→t2跨到墙钟物理端点。

## C. 逐run观测主表

{run_table(runs)}

## D. 7个baseline最终停车净距

{baseline_clearance_table(runs)}

这里的最终投影净距来自稳定目标几何与统一停车端点；M0来自`D1−D_delay−D_brake,data`。两者用途相近但计算链不同，厘米级差异用于一致性检查。

## E. 12个run的感知连续性和新鲜度

{perception_table(runs)}

注意：安全停车后很久不再输出目标会使“结局端源数据年龄”变大，例如`1048`；这不等于行驶关键阶段发生处理超时。因此结局端年龄必须与端点时刻、输出间隔和生命周期一起解释。`1206`的2399.347 ms同样受长端点与不确定结局影响，不能单独归类为感知处理缺陷。

## F. 两起碰撞的截断观测

| run | t2→碰撞/s | 截断制动距离/m | 撞击前速度/(m/s) | 碰撞端几何投影诊断/m | 完整D_brake/M0/deadline |
|---|---:|---:|---:|---:|---|
| 1131 | {r1131.T_brake_truncated_to_collision_data_observed_s:.3f} | {r1131.D_brake_truncated_to_collision_data_observed_m:.3f} | {r1131.impact_speed_data_observed_mps:.3f} | {r1131.collision_clearance_projected_diagnostic_m:.3f} | NA |
| 1643 | {r1643.T_brake_truncated_to_collision_data_observed_s:.3f} | {r1643.D_brake_truncated_to_collision_data_observed_m:.3f} | {r1643.impact_speed_data_observed_mps:.3f} | {r1643.collision_clearance_projected_diagnostic_m:.3f} | NA |

负的碰撞端投影是固定组合偏移下的穿透方向诊断，不替代CollisionSensor事件，也不用于回填完整制动余量。

## G. 反事实模型（C类，不是观测）

| run | 恢复到baseline中位响应 | 回收距离/m | 预测余量/m | 预测结局 | 预测撞击速度/(m/s) |
|---|---:|---:|---:|---|---:|
| 1131 | {cf1131.reference_baseline_latency_model_input_ms:.3f} ms | {cf1131.response_distance_recovered_model_m:.3f} | {cf1131.margin_to_observed_contact_restored_model_m:.3f} | 避碰 | {cf1131.impact_speed_model_predicted_mps:.3f} |
| 1643 | {cf1643.reference_baseline_latency_model_input_ms:.3f} ms | {cf1643.response_distance_recovered_model_m:.3f} | {cf1643.margin_to_observed_contact_restored_model_m:.3f} | 仍碰撞 | {cf1643.impact_speed_model_predicted_mps:.3f} |

模型保持各碰撞run从实际t2到接触的能量等效减速度，只提前响应时机；它不重放控制器，也不模拟新的闭环反馈。

## H. 缺陷分类矩阵

| run | 分类 | 主要缺陷 | 置信度 | 结论边界 |
|---|---|---|---|---|
"""
    for _, row in defects.iterrows():
        appendix += f"| {row.run_id} | {row.realtime_defect_class} | {row.primary_defect} | {row.confidence} | {row.evidence_boundary} |\n"
    appendix += """

## I. 时钟、身份与物理真值边界

主`t1/t2/D_delay/停车端点`使用Apollo/Localization墙钟epoch。Trace内部阶段使用monotonic clock，经trace anchor关联目标trace。只有1131和1643有actor history，可拟合CARLA sim time到wall time；其余run不能估计realtime factor。两个碰撞run的目标身份由Planning STOP ID、Fusion轨迹与CARLA actor history共同支持；无碰撞run没有同等物理真值。

## J. 方法借鉴边界

DriveFI、R-TOD、FADE、D3只作为任务说明中给出的研究组织概念：分别对应受控注入、端到端时序、时间质量到功能退化、时间到距离预算。报告未重新核验这些论文全文与正式书目信息，因此没有把任何具体论文结论作为本地数据结论的来源。
"""

    (REPORT / "group_meeting_report.md").write_text(report, encoding="utf-8")
    (REPORT / "speech_10_12min.md").write_text(speech, encoding="utf-8")
    (REPORT / "one_page_summary.md").write_text(one_page, encoding="utf-8")
    (REPORT / "evidence_appendix.md").write_text(appendix, encoding="utf-8")

    chinese_chars = len(re.findall(r"[\u4e00-\u9fff]", report))
    word_count_note = (
        "# 报告长度检查\n\n"
        f"- 主报告汉字数（含表格，不含证据附录）：{chinese_chars}\n"
        "- 目标范围：5000–8000汉字。\n"
    )
    (VALIDATION / "report_length.md").write_text(word_count_note, encoding="utf-8")

    pandoc = shutil.which("pandoc")
    docx_note = ["# DOCX导出状态", ""]
    if pandoc:
        for stem in ["group_meeting_report", "speech_10_12min"]:
            subprocess.run(
                [pandoc, str(REPORT / f"{stem}.md"), "-o", str(REPORT / f"{stem}.docx")],
                check=True,
                cwd=REPORT,
            )
        docx_note.append(f"- 已使用 `{pandoc}` 生成主报告与讲稿DOCX。")
    else:
        docx_note.append("- 当前环境未安装pandoc，因此按任务约定不生成DOCX；Markdown为正式交付。")
    (VALIDATION / "docx_export.md").write_text("\n".join(docx_note) + "\n", encoding="utf-8")
    print(f"wrote report family; main report Chinese characters={chinese_chars}")


if __name__ == "__main__":
    main()
