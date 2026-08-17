# 第二次实验 TCPS-PA v2 实时系统工程六层分析报告

## 六层推理状态矩阵（Six-Layer Inference Status Matrix）

| 层级/Claim | 判定 | 证据基础 | 置信度 | 上限 | 未闭合关键反证项 | 允许结论 |
|---|---|---|---|---|---|---|
| L1 / C1 | PASS | 直接观测 | HIGH | HIGH | 0 | 外部 Bridge/SCB 干预已核验；完整 signature 仍有限制 |
| L2 / C2 | PARTIAL_PASS | 观测派生 | MEDIUM | MEDIUM | 0 | T_R 相对 baseline 右移；A/G 与 phase 证据部分 |
| L3 / C3 | PARTIAL_PASS | 关联等级 C | MEDIUM | MEDIUM | 0 | 系统级时间关联，缺显式 lineage |
| L4 / C4 | NOT_TESTABLE | 事后重建 + 未验证模型 | LOW | LOW | 1 | 独立 tau_req 缺失 |
| L5 / C5 | MODEL_SUPPORTED_ONLY | 模型来源标记 | LOW | LOW | 1 | D_response 为观测；主 debt 不可用 |
| L6 / C6 | PASS | 直接物理观测 | MEDIUM | MEDIUM | 0 | 净距、margin、停车与碰撞结局 |
| Attribution / C7 | PARTIAL_PASS | 最弱环节 | LOW | 第 2 级 / LOW | 7 | 第 2 级：系统级关联与模型机制 |

## 受 Claim Ledger 约束的执行结论

本报告覆盖 12 个 run（baseline 7、请求 300 ms 的 Bridge/SCB 干预 5）；`202607271206` 因结局来源冲突退出主结局统计，主分析为 11 个 run。原始实验目录保持只读。

直接观测表明：300 ms 组实际 Bridge 墙钟延时中位数为 **300.124 ms**；物理反应时间 `T_R` 中位数由 **300.358 ms** 变为 **749.901 ms**；墙钟积分响应距离由 **5.284 m** 变为 **12.716 m**；本批 run 的碰撞计数为 baseline **0/7**、300 ms 主分析 **2/4**。这些比例只描述当前小样本。

六层 Gate 的当前输出是：C1 `PASS`，C2/C3 `PARTIAL_PASS` 且 lineage grade C，C4 `NOT_TESTABLE`，C5 `MODEL_SUPPORTED_ONLY`，C6 `PASS`。因此 C7 为 `PARTIAL_PASS/LOW`，最高第 2 级：只支持系统级时间关联和模型支持的空间传播机制。独立时间要求、主观测距离债务、功能正确性、严格 lineage、clock/phase 与 pre-hazard 状态均未闭合。

## 报告定位与方法完成度

本报告同时承担三个角色，但必须区分其完成状态：

- **实验数据审计：** 已具备 raw-first 复算、逐 run 结果、observed/model 分离、缺失原因和右删失规则。
- **六层协议首次实例化：** 已具备 Evidence→Claim→Defeater→Gate→Claim Strength 的结构，并允许输出 `NOT_TESTABLE`。
- **论文方法有效性证据：** 尚未完成；中间的 C4/C5 与多项实时系统前置条件仍缺独立证据。

| 内容 | 完成状态 | 当前证据或缺口 |
|---|---|---|
| 证据/模型分离 | 已完成 | observed、retrospective、requirement、model 分列且 taint 传播 |
| Claim Graph 与六层 Gate | 已完成（结构） | P_CLOCK/P_TARGET/P_FUNC/P_DEADLINE、C1-C7 与 canonical edges 已建立 |
| Gate 可执行判据 | 本版补齐 | 输入、指标、证据类别、criterion 与下一层条件已写入 Claim Ledger/报告 |
| Temporal Fault Signature | 部分完成 | onset/幅值/pattern/scope 可用；end/duration/message count/drop/reorder 未闭合 |
| R/A/G 与 tail latency | 部分完成 | run-level P50/P90/P95/P99/MAX 已汇总；record message-level MRT/MDA 不可用 |
| 严格 cause-effect lineage | 未完成 | 当前 grade C；缺 event/sequence/provenance 绑定 |
| 独立 tau_req | 未完成 | 只有 tau_retro 与未验证 tau_model |
| 主观测 Distance Debt | 未完成 | 没有 REQUIREMENT_CONSTRAINED_DERIVED debt |
| 空间预算分解 | 部分完成 | 完整安全停车 run 可分解；碰撞 run 被右删失 |
| 连续物理安全尺度 | 部分完成 | D2/M0/M6/impact 可用；near/critical taxonomy 缺 threshold provenance |
| Functional Correctness | 部分完成 | 审计表已建但 P_FUNC 全部 PARTIAL，Control/Prediction 等多项 UNKNOWN |
| Clock/Phase uncertainty | 未完成 | 多数 run 无 offset/drift/resolution；12/12 未 phase scan |
| Pre-hazard state divergence | 部分完成 | D1/V1 标为 possible mediator，但 fault-onset state/delta 缺失 |
| 论文级方法有效性 | 未完成 | 缺多非零等级、独立验证数据、record-enabled lineage、负对照与跨系统验证 |

## 系统架构、事件语义与分析边界

- CARLA 0.9.15 位于服务器端，Apollo 10.0.0 位于 Orin，经网线连接。
- Bridge 直接读取 Control 命令；Guardian 不在本实验执行链中。
- `t_f`：Bridge/SCB 首次应用干预；`t_c/t1`：稳定目标 cause endpoint；`t_e/t2`：持续有效制动起点；`t_d=t1+tau_req`；`t_o`：停车、碰撞或收集终点。
- 主 `T_R=t2-t1` 和 `D_response=∫[t1,t2]v(t)dt_wall` 使用墙钟语义；CARLA frame/sim time 仅作诊断。
- baseline distribution 是 L2 描述性 reference，不是 L4 安全要求。
- 本实验 12/12 run 无同 run 解析 record profile；message-level reaction/data age、Control payload 与严格 provenance 因而不可补齐。

## 六层 Gate 的可执行定义

| Gate | 输入 | 指标 | 判据 | 本实验输出 | 进入下一步条件 |
|---|---|---|---|---|---|
| L1 / C1 | Temporal Fault Signature、Bridge/SCB applied rows、P_CLOCK.fault_signature | location、onset、duration、requested/actual magnitude、pattern、scope、affected messages | 实际应用证据存在，且位置、onset、幅值、pattern 与作用范围足以界定干预；未知 duration/message/drop/reorder 继续作为完整性限制 | PASS | C1 至少 PARTIAL_PASS，且 L2 有已声明 reference |
| L2 / C2 | C1、baseline distribution、stage_timing_and_freshness、clock/phase audit | R（反应/响应变化）、A（新鲜度/数据年龄）、G（连续性/update gap）的 run-level 分布 | 相对 BASELINE_DISTRIBUTION 比较 P50/P90/P95/P99/MAX/IQR；缺 reference、样本覆盖或 phase 审计时不得强 PASS | PARTIAL_PASS | C2 至少 PARTIAL_PASS，P_CLOCK/P_TARGET 足以支持声明的 lineage 等级 |
| L3 / C3 | C1、C2、P_CLOCK、P_TARGET、event timeline、trace/provenance | T_R=t2-t1、source→Fusion→Prediction→Planning→Control→Bridge→physical response lineage grade | A/B 且前置条件闭合可 PASS；C 级最多 PARTIAL_PASS；D/UNKNOWN 进一步降级 | PARTIAL_PASS | C3 与 P_DEADLINE 均满足，才能进入主 C4 比较 |
| L4 / C4 | 观测 T_R、同 scope 合格 tau_req、P_CLOCK、P_TARGET、P_DEADLINE | S_T=tau_req-T_R，tau_req_low/center/high | T_R>tau_req_high 为 CLEARLY_MISSED；位于区间内为 BOUNDARY_UNCERTAIN；只有 tau_retro/tau_model 时主 C4 NOT_TESTABLE | NOT_TESTABLE | C4 有合格判定，且存在兼容墙钟速度路径 |
| L5 / C5 | C4、tau_req、观测速度路径、D1、D_brake、D_safe | D_response、D_debt、M_D、ΔM_D=ΔD1-ΔD_response-ΔD_brake | 只有合格 C4 + 观测速度积分 + qualified debt 才可 PASS；模型或事后 deadline 只能支持模型/重建机制 | MODEL_SUPPORTED_ONLY | C5 与直接物理 C6 证据共同进入 C7 |
| L6 / C6 | 直接碰撞/停车事件、净距、margin、minimum speed、impact、截断制动 | D2、M0/M6、minimum/final clearance、impact speed/impulse、outcome severity | 直接物理裕度或结局发生实验内退化；任何 critical/near/high-severity 分类必须有预先来源 | PASS | C6 只证明物理结果；与 C4/C5/P_FUNC/defeaters 一并进入 C7 |
| Attribution / C7 | C1-C6、P_FUNC、全部关键 defeater、taint 与 confidence ceiling | weakest-link level、开放关键反证项、重复性与 dose-response | 取关键前置项、证据类别和开放反证项的最低上限；外部 Bridge 注入不能支持 Apollo 内部缺陷判断 | PARTIAL_PASS | 输出允许语言、最高声明等级和下一轮证据需求 |

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

| 功能链项目 | PASS | DEGRADED | PARTIAL | UNKNOWN |
|---|---|---|---|---|
| 物理目标身份 | 0 | 0 | 12 | 0 |
| Perception 目标存在 | 12 | 0 | 0 | 0 |
| Perception 连续追踪 | 0 | 0 | 0 | 12 |
| Prediction 目标/语义 | 0 | 0 | 0 | 12 |
| Planning STOP | 12 | 0 | 0 | 0 |
| Planning 目标/位置 | 0 | 0 | 0 | 12 |
| Planning 轨迹 | 0 | 12 | 0 | 0 |
| Planning fallback | 0 | 12 | 0 | 0 |
| Control 相关轨迹 | 0 | 0 | 0 | 12 |
| Control 制动命令 | 0 | 0 | 0 | 12 |
| Control 命令连续性 | 0 | 0 | 0 | 12 |
| Bridge payload receive | 12 | 0 | 0 | 0 |
| Bridge payload apply | 0 | 0 | 0 | 12 |
| 物理响应 | 12 | 0 | 0 | 0 |

### Clock 与 phase uncertainty

所有 run 的 `phase_scan_performed=FALSE`；无法把 300/400/700/800/900 ms 聚类解释为已建立的 tick/phase 机制。单墙钟 `T_R` 可保留，但跨 Orin/server 阶段差值只能给中等置信度。

### Pre-hazard State Divergence Audit

| 状态变量 | fault 时可用 | t1 时可用 | delta 可用 | 当前因果角色 |
|---|---|---|---|---|
| D1 | 0 | 12 | 0 | POSSIBLE_MEDIATOR:12 |
| V1 | 0 | 12 | 0 | POSSIBLE_MEDIATOR:12 |
| A1 | 0 | 0 | 0 | UNKNOWN:12 |
| HEADING | 0 | 0 | 0 | UNKNOWN:12 |
| ROUTE_PROGRESS | 0 | 0 | 0 | UNKNOWN:12 |

干预 onset 早于 t1。D1/V1 目前只能视为 `POSSIBLE_MEDIATOR`；因为 fault 时刻状态和 delta 缺失，不能判作独立混杂，也不能定量拆分 total closed-loop effect 与 post-t1 response effect。

## L1 时序故障特征

- **科学命题：** 已声明且作用范围明确的时序扰动进入了闭环。
- **输入：** Temporal Fault Signature、Bridge/SCB applied rows、P_CLOCK.fault_signature。
- **核心指标：** location、onset、duration、requested/actual magnitude、pattern、scope、affected messages。
- **必要前提：** P_CLOCK.fault_signature。
- **可接受证据：** DIRECT_OBSERVED Bridge/SCB applied-delay evidence。
- **判定标准：** 实际应用证据存在，且位置、onset、幅值、pattern 与作用范围足以界定干预；未知 duration/message/drop/reorder 继续作为完整性限制。
- **当前支持：** 12/12 run 有 requested/actual applied delay；300 ms 组实际延时稳定在约 300 ms；onset 均早于 t1 证据链接：EV.L1.DELAY.202607271031, EV.L1.DELAY.202607271048, EV.L1.DELAY.202607271054, EV.L1.DELAY.202607271059，另有 8 条。
- **反向证据/限制：** fault_end、实际 duration、affected message count、drop/reorder 未建立；baseline run 1031 有 19.282 ms worst-observed 实际延时
- **未闭合反证项：** 0 项。
- **规则与输出：** `IR-C1` → `PASS`；置信度 `HIGH`，上限 `HIGH`，最高声明等级 `3`。
- **进入下一层的条件：** C1 至少 PARTIAL_PASS，且 L2 有已声明 reference。
- **允许结论：** 外部注入的时间扰动在本地时钟域内得到直接核验。
- **仍未证明：** 该干预来自外部 Bridge/SCB，不能据此判断 Apollo 内部产生了同类扰动。

### Temporal Fault Signature 汇总

| 分组 | n | 实际延时 P50(ms) | P90 | MAX | onset 相对 t1(s) | end 缺失 | 消息数未知 | pattern |
|---|---|---|---|---|---|---|---|---|
| baseline | 7 | 0.070 | 7.768 | 19.282 | -21.696 至 -5.532 | 7 | 7 | PERSISTENT_SETTING / REPEATED_PER_AFFECTED_MESSAGE |
| delay_300ms | 5 | 300.100 | 300.250 | 300.318 | -25.997 至 -22.237 | 5 | 5 | PERSISTENT_SETTING / REPEATED_PER_AFFECTED_MESSAGE |

300 ms 干预在 t1 前约 22.237–25.997 s 已生效，属于持续 setting，而非目标出现后的一次性延时。baseline 的实际延时 MAX 为 19.282 ms，因此实时系统描述不能只报告 baseline 中位数。

## L2 时序退化：R/A/G 与 tail

- **科学命题：** 相对于已声明的参考分布，时序行为出现退化。
- **输入：** C1、baseline distribution、stage_timing_and_freshness、clock/phase audit。
- **核心指标：** R（反应/响应变化）、A（新鲜度/数据年龄）、G（连续性/update gap）的 run-level 分布。
- **必要前提：** C1.all_runs, P_CLOCK.cross_host。
- **可接受证据：** OBSERVED_DERIVED，且 reference_type 与 distribution_scope 明确。
- **判定标准：** 相对 BASELINE_DISTRIBUTION 比较 P50/P90/P95/P99/MAX/IQR；缺 reference、样本覆盖或 phase 审计时不得强 PASS。
- **当前支持：** T_R 中位数 300.358→749.901 ms；Control→t2 是主要描述性增量；R/A/G 表保留逐 run tail 与可用计数 证据链接：EV.L2.TR.202607271031, EV.L2.TR.202607271048, EV.L2.TR.202607271054, EV.L2.TR.202607271059，另有 20 条。
- **反向证据/限制：** record message-level MRT/MDA 缺失；G 响应窗 baseline 仅 2/7 可用；phase 未扫描；P95/P99 在 n=4/7 下只代表经验插值
- **未闭合反证项：** 2 项。
- **规则与输出：** `IR-C2` → `PARTIAL_PASS`；置信度 `MEDIUM`，上限 `MEDIUM`，最高声明等级 `2`。
- **进入下一层的条件：** C2 至少 PARTIAL_PASS，P_CLOCK/P_TARGET 足以支持声明的 lineage 等级。
- **允许结论：** 观测 T_R 与 gap 指标可用于刻画可能的时序退化，但不能越过 reference、clock 与 phase 限制。
- **仍未证明：** update-gap 缺合格参考分布，跨主机时钟和注入相位尚未充分审计。

### R/A/G 逐 run 经验分布

| 维度 | 指标 | 单位 | baseline P50/P90/P95/P99/MAX | 300 ms P50/P90/P95/P99/MAX | 语义/限制 |
|---|---|---|---|---|---|
| R | 物理反应时间 T_R | ms | 300.358/399.688/399.719/399.744/399.750 (n=7/7) | 749.901/865.600/879.735/891.043/893.870 (n=4/4) | t1→t2 单墙钟物理区间 |
| R | Sensor→Control | ms | 265.401/291.807/297.810/302.613/303.813 (n=7/7) | 303.100/340.910/346.762/351.444/352.614 (n=4/4) | 阶段诊断，跨主机置信度受限 |
| R | Control→t2 | ms | 38.224/108.787/118.425/126.136/128.064 (n=7/7) | 447.421/524.690/532.973/539.599/541.256 (n=4/4) | Control 后至持续制动起点 |
| A | t2 时目标数据年龄 | ms | 299.977/300.471/300.556/300.624/300.641 (n=7/7) | 347.346/398.561/399.491/400.235/400.421 (n=4/4) | 目标 header/source 到物理 t2 |
| A | 目标 lifecycle P90 | ms | 234.979/258.269/263.390/267.487/268.511 (n=7/7) | 281.555/314.900/318.098/320.657/321.296 (n=4/4) | run 内 response window 摘要 |
| A | 结局时目标 source age | ms | 399.615/1419.947/2109.895/2661.854/2799.843 (n=7/7) | 799.341/974.462/990.677/1003.649/1006.892 (n=4/4) | 诊断量；结局端点与匹配语义需保守解释 |
| G | 响应窗 target gap MAX | ms | 98.270/98.517/98.548/98.573/98.579 (n=2/7) | 99.230/105.528/106.871/107.945/108.213 (n=4/4) | 少于两个输出时不可用 |
| G | 全窗 target gap P90 | ms | 112.525/124.303/130.884/136.149/137.466 (n=7/7) | 108.462/113.624/114.327/114.890/115.031 (n=4/4) | run 内 target 输出间隔摘要 |
| G | 全窗 target gap MAX | ms | 117.223/290.520/294.061/296.894/297.602 (n=7/7) | 188.141/411.683/459.561/497.863/507.439 (n=4/4) | 单个 MAX 不能单独建立组级退化 |

表中 P95/P99 是基于 7 个 baseline 与 4 个 delay 主分析 run 的经验分位数，不是 WCET 保证，也不是消息帧的独立重复。Sensor→Control 中位数约由 **265.401 ms** 变为 **303.100 ms**；Control→t2 中位数约由 **38.224 ms** 变为 **447.421 ms**。后半段变化更大，但 clock/phase 与 lineage 限制仍在。

## L3 Cause-Effect Chain 与物理反应区间

- **科学命题：** 时序退化沿相关因果链形成了传播关联。
- **输入：** C1、C2、P_CLOCK、P_TARGET、event timeline、trace/provenance。
- **核心指标：** T_R=t2-t1、source→Fusion→Prediction→Planning→Control→Bridge→physical response lineage grade。
- **必要前提：** C1.all_runs, C2.all_runs, P_CLOCK.cross_host, P_TARGET.all_runs。
- **可接受证据：** A/B 级 TRACE_LINEAGE 可支持强结论；C 级时间对齐只支持系统级关联。
- **判定标准：** A/B 且前置条件闭合可 PASS；C 级最多 PARTIAL_PASS；D/UNKNOWN 进一步降级。
- **当前支持：** t1/t2 单墙钟物理区间和模块事件时间线可用，形成 C 级时间对齐 证据链接：EV.LINEAGE.ALIGNMENT, EV.CLOCK.CROSS_HOST。
- **反向证据/限制：** 缺 source/Fusion/Prediction/Planning/Control/actuation 的统一 event ID、sequence 或传播 provenance；多数 run 缺双时钟历史
- **未闭合反证项：** 3 项。
- **规则与输出：** `IR-C3` → `PARTIAL_PASS`；置信度 `MEDIUM`，上限 `MEDIUM`，最高声明等级 `2`。
- **进入下一层的条件：** C3 与 P_DEADLINE 均满足，才能进入主 C4 比较。
- **允许结论：** 现有证据支持系统级时间关联；严格 cause-effect lineage 仍不完整。
- **仍未证明：** 没有显式 trace/provenance lineage，目标连续性和跨主机时钟仍是部分证据。

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

- **科学命题：** 观测物理反应时间超过了独立合格的时间要求。
- **输入：** 观测 T_R、同 scope 合格 tau_req、P_CLOCK、P_TARGET、P_DEADLINE。
- **核心指标：** S_T=tau_req-T_R，tau_req_low/center/high。
- **必要前提：** C3.all_runs, P_CLOCK.cross_host, P_TARGET.all_runs, P_DEADLINE.all_runs。
- **可接受证据：** OBSERVED_DERIVED physical T_R + INDEPENDENT_REQUIREMENT/独立验证 envelope。
- **判定标准：** T_R>tau_req_high 为 CLEARLY_MISSED；位于区间内为 BOUNDARY_UNCERTAIN；只有 tau_retro/tau_model 时主 C4 NOT_TESTABLE。
- **当前支持：** 物理 T_R 可观测；24 条 tau_retro 与 12 条 tau_model 已登记并分型 证据链接：EV.L2.TR.202607271031, EV.L2.TR.202607271048, EV.L2.TR.202607271054, EV.L2.TR.202607271059，另有 32 条。
- **反向证据/限制：** 独立 tau_req=0 条，tau_req_low/high 无可用值；模型在两个 300 ms 安全停车 run 上产生碰撞假阳性
- **未闭合反证项：** 1 项。
- **规则与输出：** `IR-C4` → `NOT_TESTABLE`；置信度 `LOW`，上限 `LOW`，最高声明等级 `1`。
- **进入下一层的条件：** C4 有合格判定，且存在兼容墙钟速度路径。
- **允许结论：** 事后重建与模型证据只用于敏感性分析，主时间正确性判定保持不可检验。
- **仍未证明：** 没有独立合格的同场景时间要求，现有 deadline 仅来自事后重建或未验证模型。

| deadline 类别 | 登记条数 | 来源 | 主 C4 合格 | 用途 |
|---|---|---|---|---|
| tau_retro | 24 | 事后重建 | 否 | 重建/敏感性 |
| tau_model | 12 | 样本内常减速度模型 | 否 | 模型机制 |
| tau_req | 0 | 独立前瞻要求 | 当前无 | 主 C4 |

`tau_retro` 依赖同 run 的完整停车过程，只能回答事后还能等待多久；`tau_model` 使用 baseline 样本内常有效减速度 **5.102 m/s²**，没有独立验证和 uncertainty bounds。主 `S_T=tau_req-T_R` 因 `tau_req` 缺失而不可计算。

模型在 300 ms 主分析 4 个 run 中给出 4/4 个碰撞预测，而实际为 2/4，其中 2 个安全停车 run 被预测为碰撞。这是主 C4 不采用该模型作为合格 requirement 的直接理由。

## L5 从时间到物理空间的传播

- **科学命题：** 合格的时间违例造成了可量化的额外空间消耗。
- **输入：** C4、tau_req、观测速度路径、D1、D_brake、D_safe。
- **核心指标：** D_response、D_debt、M_D、ΔM_D=ΔD1-ΔD_response-ΔD_brake。
- **必要前提：** C4.all_runs, P_CLOCK.cross_host。
- **可接受证据：** REQUIREMENT_CONSTRAINED_DERIVED 主 debt；retro/model debt 保留来源标记。
- **判定标准：** 只有合格 C4 + 观测速度积分 + qualified debt 才可 PASS；模型或事后 deadline 只能支持模型/重建机制。
- **当前支持：** D_response 墙钟积分中位数 5.284→12.716 m；完整停车 run 可计算空间预算分解 证据链接：EV.L5.D_RESPONSE.202607271031, EV.L5.D_RESPONSE.202607271048, EV.L5.D_RESPONSE.202607271054, EV.L5.D_RESPONSE.202607271059，另有 20 条。
- **反向证据/限制：** 主观测 requirement-constrained debt 不可用；碰撞 run 无完整 D_brake/M0；组分解未控制 pre-hazard 状态且不是因果贡献估计
- **未闭合反证项：** 2 项。
- **规则与输出：** `IR-C5` → `MODEL_SUPPORTED_ONLY`；置信度 `LOW`，上限 `LOW`，最高声明等级 `2`。
- **进入下一层的条件：** C5 与直接物理 C6 证据共同进入 C7。
- **允许结论：** 观测响应距离与模型结果支持一种可能的空间传播机制，主观测距离债务尚未建立。
- **仍未证明：** 主 deadline gate 未成立，距离债务仍带有事后或模型来源标记。

主响应距离：

`D_response_wall_integral_data_observed_m = ∫[t1,t2] v(t) dt_wall`

合格主距离债务只有在独立 `tau_req` 存在时才定义：

`D_debt = max(0, ∫[t1+tau_req,t2] v(t) dt_wall)`

当前 `D_debt_requirement_constrained_derived_m` 不可用；`D_debt_retro_diagnostic_m` 与 `D_debt_model_predicted_m` 分别保留事后/模型来源。

### 观测空间预算分解

`M0 = D1 - D_response - D_brake`

`ΔM0 = ΔD1 - ΔD_response - ΔD_brake`

| 比较组 | n | D1 mean(m) | D_response mean(m) | D_brake mean(m) | M0 mean(m) |
|---|---|---|---|---|---|
| baseline_full_stop | 7 | 39.255 | 5.679 | 29.986 | 3.589 |
| delay_300ms_safe_stop | 2 | 39.909 | 11.874 | 27.252 | 0.783 |
| delay_safe_minus_baseline | 2 vs 7 | 0.655 | 6.195 | -2.734 | -2.806 |

差值行采用端点兼容完整停车 run 的组均值：baseline n=7、300 ms 安全停车 n=2。该恒等式用于描述空间预算，不是因果贡献估计；碰撞 run 因右删失不能获得完整 D_brake/M0。

## L6 物理安全退化

- **科学命题：** 实验中的物理安全裕度或直接结局出现退化。
- **输入：** 直接碰撞/停车事件、净距、margin、minimum speed、impact、截断制动。
- **核心指标：** D2、M0/M6、minimum/final clearance、impact speed/impulse、outcome severity。
- **必要前提：** 无前置 Claim。
- **可接受证据：** DIRECT_OBSERVED 且 semantic_role=PHYSICAL_OUTCOME。
- **判定标准：** 直接物理裕度或结局发生实验内退化；任何 critical/near/high-severity 分类必须有预先来源。
- **当前支持：** baseline 0/7 碰撞、300 ms 主分析 2/4；安全停车 M0 中位数 3.471→0.783 m；碰撞速度 7.988/11.728 m/s 证据链接：EV.L6.OUTCOME.202607271031, EV.L6.OUTCOME.202607271048, EV.L6.OUTCOME.202607271054, EV.L6.OUTCOME.202607271059，另有 8 条。
- **反向证据/限制：** 1206 结局冲突；near/critical/high-severity 分类缺预先阈值；碰撞 run 右删失完整停止过程
- **未闭合反证项：** 1 项。
- **规则与输出：** `IR-C6` → `PASS`；置信度 `MEDIUM`，上限 `MEDIUM`，最高声明等级 `3`。
- **进入下一层的条件：** C6 只证明物理结果；与 C4/C5/P_FUNC/defeaters 一并进入 C7。
- **允许结论：** 安全停车、碰撞和物理裕度属于直接观测结果；该层不单独分配时间归因。
- **仍未证明：** 物理结局本身不完成时间归因，且 1206 的结局证据存在冲突。

连续结果链为 `D2 → M0/M6 → impact → outcome`。本实验直接保存 D2、完整停车 margin 和碰撞速度，但尚无有来源的 LOW_MARGIN/CRITICAL/NEAR_MISS/HIGH_SEVERITY 阈值，因此不在看到结局后补设等级。

碰撞 run 的完整停车距离与 full-stop margin 保持不可用；只使用碰撞前截断路径、碰撞事件、actor/geometry 证据与碰撞速度。

## C7 时间安全归因

- **判定：** `PARTIAL_PASS/LOW`，最高第 2 级。
- **允许结论：** 系统级时间关联得到支持；模型结果提供一种待独立验证的空间传播机制。
- **不能升级的原因：** C4 不可检验、C5 仅模型支持、P_FUNC 部分、pre-hazard 状态未测全、cross-host clock 与 phase 未闭合。

| 反证项 | 状态 | 影响 | 未闭合问题 |
|---|---|---|---|
| `D_INITIAL_CLEARANCE.C7.all_runs` | OPEN | CRITICAL | 初始净距可能在干预之外发生变化。 |
| `D_INITIAL_SPEED.C7.all_runs` | OPEN | CRITICAL | 初始或接近速度可能独立改变物理结局。 |
| `D_BRAKING_CAPABILITY.C7.all_runs` | OPEN | CRITICAL | 制动能力目前主要由样本内模型表示。 |
| `D_FUNCTIONAL_FAILURE.C7.all_runs` | OPEN | CRITICAL | 功能性替代解释仍未排除。 |
| `D_TARGET_MISMATCH.C7.all_runs` | UNKNOWN | CAPS_AT_PARTIAL | 端到端目标 lineage 仍不完整。 |
| `D_DATA_FRESHNESS.C7.all_runs` | UNKNOWN | CAPS_AT_PARTIAL | 新鲜度退化尚未被独立隔离。 |
| `D_UPDATE_GAP.C7.all_runs` | OPEN | CAPS_AT_PARTIAL | update-gap 缺少合格参考分布。 |
| `D_SOLVER_FALLBACK.C7.all_runs` | OPEN | CRITICAL | Planning fallback 或不可行状态可能独立参与。 |
| `D_CLOCK.C7.all_runs` | UNKNOWN | CAPS_AT_PARTIAL | 跨主机时间对齐仍为部分证据。 |
| `D_PHASE.C7.all_runs` | UNKNOWN | CAPS_AT_PARTIAL | 尚未扫描注入相位效应。 |
| `D_PREHAZARD_STATE.C7.all_runs` | OPEN | CRITICAL | 干预早于 t1，D1/v1 可能属于干预后的状态。 |
| `D_GEOMETRY.C7.all_runs` | OPEN | CAPS_AT_PARTIAL | 各 run 的几何与目标净距存在差异。 |
| `D_OUTCOME_CONFLICT.C7.all_runs` | UNKNOWN | CAPS_AT_PARTIAL | 不同结局来源尚未完成一致性闭合。 |
| `D_DEADLINE.C7.all_runs` | OPEN | CRITICAL | C4 缺少独立合格的时间要求。 |

## 模型、事后重建与误差分析（单独呈现）

- 完整停车主分析 run 的模型制动距离平均绝对误差为 **1.174 m**，绝对误差中位数为 **0.341 m**。
- 两个碰撞 run 的碰撞速度模型绝对误差平均为 **1.061 m/s**。
- 模型为样本内描述模型，未报告 a_brake 独立校准区间、外部验证集或 tau_req_low/high；因此不能承担主时间正确性判定。

## 逐 run 观测结果

| run | 分组 | 范围 | T_R(ms) | D_response(m) | D1(m) | D2(m) | M0(m) | 结局 | 碰撞速度(m/s) |
|---|---|---|---|---|---|---|---|---|---|
| 202607271031 | baseline | 主分析 | 299.655 | 4.673 | 39.891 | 35.218 | 5.338 | 安全停车 | — |
| 202607271048 | baseline | 主分析 | 300.358 | 5.254 | 38.744 | 33.490 | 3.471 | 安全停车 | — |
| 202607271054 | baseline | 主分析 | 299.977 | 5.367 | 39.632 | 34.264 | 2.740 | 安全停车 | — |
| 202607271059 | baseline | 主分析 | 399.750 | 7.006 | 40.155 | 33.149 | 2.746 | 安全停车 | — |
| 202607271104 | baseline | 主分析 | 399.646 | 6.936 | 38.784 | 31.848 | 3.968 | 安全停车 | — |
| 202607271108 | baseline | 主分析 | 300.641 | 5.284 | 38.759 | 33.475 | 3.512 | 安全停车 | — |
| 202607271113 | baseline | 主分析 | 299.796 | 5.231 | 38.817 | 33.586 | 3.351 | 安全停车 | — |
| 202607271131 | delay_300ms | 主分析 | 799.636 | 13.432 | 38.258 | 24.826 | — | 碰撞 | 7.988 |
| 202607271202 | delay_300ms | 主分析 | 699.155 | 11.749 | 39.616 | 27.867 | 0.548 | 安全停车 | — |
| 202607271206 | delay_300ms | 排除：结局冲突 | 800.753 | 13.741 | 39.790 | 26.049 | -1.636 | 结局不确定（事件与几何冲突） | — |
| 202607271211 | delay_300ms | 主分析 | 700.167 | 11.999 | 40.202 | 28.203 | 1.018 | 安全停车 | — |
| 202607271643 | delay_300ms | 主分析 | 893.870 | 15.451 | 36.651 | 21.201 | — | 碰撞 | 11.728 |

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
