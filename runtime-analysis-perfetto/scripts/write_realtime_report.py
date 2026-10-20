"""Compose the scientific narrative from reviewed results, not sample values."""
import csv
import json
import pathlib

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'prediction_realtime_report'
summary=json.loads((OUT/'analysis_summary.json').read_text(encoding='utf-8'))
global_=json.loads((ROOT/'metadata/global_summary.json').read_text(encoding='utf-8'))
def read(name):
    return list(csv.DictReader((OUT/'data'/name).open(encoding='utf-8-sig')))
def table(rows, fields):
    return '| '+' | '.join(label for key,label in fields)+' |\n|'+'|'.join('---' for _ in fields)+'|\n'+ '\n'.join('| '+' | '.join('NA' if r.get(k) in ['',None] else f'{r[k]:.3f}' if isinstance(r[k],float) else str(r[k]) for k,label in fields)+' |' for r in rows)
cases=summary['normal_reference']; ledger=read('frame_analysis.csv'); stages=read('major_stage_timings.csv')
selected=[r for r in ledger if int(r['frame']) in [398,648,653,659,675,677]]
for r in selected:
    for k in ['callback_ms','response_ms','dispatch_proxy_ms','slack_ms','running_ms','runnable_ms','sleeping_ms']:r[k]=float(r[k])
brief='''## 核心发现

本次分析把 Prediction 的持续长延迟落实到具体处理实例、应用阶段和 Linux 等待区间。5—105 秒内的 693 次完整 callback 中，90—95 秒的 F647—F680 平均耗时为 **144.631 ms**；该段出现 **29 次下一 Fusion 更新边界超时**，占窗口全部 35 次超时的 **82.86%**。这组连续实例提供了比单一最大值更稳定的取证对象。

F648 的 callback 为 **227.192 ms**，Evaluator 占 **217.801 ms**，ThreadPoolRun 占 **203.825 ms**。父线程的 **196.136 ms Sleeping** 中，约 **195.595 ms** 的 episode 关联到 Evaluator Future/Futex 阻塞栈。Worker 同时出现 PI Futex 与 nvhost syncpoint 等待路径。证据将异常定位到并行评估和具体阻塞路径；锁对象、持有者及设备完成依赖另见机制证据节。

本报告的时限采用**完整 Fusion 输出流中的下一次真实输出**作为分析边界，完成点采用 `proc_exit`，等于边界记为 meet。该策略度量 Prediction 能否在下一次上游更新前完成，属于工程分析策略。'''
contract='''## 1. 分析合同与证据范围

Run 为 `pw_20261020_141904`。Apollo 10 部署在 Orin，CARLA 0.9.15 与 bridge 位于服务器；bridge 直接读取 Control。分析保留整轮应用账本的 **1,057 次 Prediction 处理实例、1,929 个已开始 Task**，系统取证使用 Trace 起点后 **[5,105) 秒**，覆盖 F42—F734 的 693 次完整 callback 和 724 个完整 Worker Task。

分析单位是 `run + Prediction 进程生命周期 + trace_id + proc_id + enter_event_id`。F 编号是本轮按 callback 排序的索引；完整 `frame_uid`、Task ID、TID、UTID 与原始事件保存在配套表中。大整数 `trace_id` 以字符串保存。

| 时间量 | 实际端点与解释 |
|---|---|
| Callback 墙钟 | `proc_enter → proc_exit`，同一 RAW 时钟 |
| Release-to-completion response | 匹配 Fusion `output_pub` 代理端点 → Prediction `proc_exit` |
| 派发代理差值 | Fusion 输出代理 → callback 开始，包含未细分的传播/派发时间 |
| Writer 响应 | callback 开始 → `writer_done`，表示 Writer API 返回 |
| 更新边界 / Slack | 下一真实 Fusion 输出代理时刻减 `proc_exit`；带符号；`c>d` 为 miss |
| Planning 传播观察 | 相同消息身份关联的 Prediction Writer 入口代理 → Planning `RunOnce` |

Prediction `proc_enter` 是函数持有输入参数的处理入口；`output_pub` 与 `writer_enter` 使用同一 Writer 调用前时间戳，消息上下文事后复用该时间。实际 Reader 到达、线上发布完成和正式安全 Deadline 没有独立端点。本文按上述端点命名各指标。

参照方法分成 Runtime-TDFG 消息依赖、Execution DAG 任务依赖及 Dynamic Path 执行实体映射。现有表保留三者；Future Ready、完整 CRoutine 生命周期及 GPU timeline 未采集，严格完整关键路径的 `critical_flag` 和系统机制可归因增量保留 NA。'''
quality='''## 2. 数据质量、身份与时间校准

Perfetto 的整体状态为 `FAILED_OR_INCOMPLETE`：采集尾部触发内存保护停止，末次进程身份快照失败，部分栈展开不完整。本报告用已验证的局部覆盖决定可计算范围。5—105 秒内，693 个 callback 和 724 个 Task 的状态覆盖率均为 100%，Unknown 为 0，状态分解闭合误差为 0 ns。

应用打点采用 `CLOCK_MONOTONIC_RAW`，Perfetto Trace Processor 的时间采用 `CLOCK_BOOTTIME`。以本轮 47 组 Clock Snapshot 对 RAW→BOOTTIME 做分段线性偏移插值；独立留出验证在分析窗口内的最大残差为 **5.059 μs**。2,388 个原生事件的同 TID Running 支持率为 100%（容差 100 μs，最大距离 2.653 μs）。CPU/event stack 的 11,606 个检查样本在 2 μs 容差下同 TID、同 CPU 支持率为 100%；它们已经是 BOOTTIME，采用恒等转换。

进程身份由 before 快照中的 PID、starttime、namespace 与 TP 导入的进程生命周期共同核实；after 快照缺失保留质量标记。时钟支持检验与状态时间闭合是两项独立检查。

源码快照为采集后读取的当前文件，快照 SHA 已保存；采集时二进制与该源码的构建等价性没有独立证明。本文通过字段结构和事件配对核对源码解释，部署 commit、CARLA 具体场景、当时 Jetson 功耗模式、完整亲和性/调度配置及采集开销对照按 manifest 中的实际可用性保留。它们不被填成默认配置。

Record 中有 1,034 条 Prediction 输出，Trace 有 1,057 个输出实例。按 channel、trace、sequence 与消息时间身份核对：**1,001 条匹配，56 条 Trace-only 位于 Record Prediction 首条消息之前，33 条 Record-only 位于 Trace 窗口之外**。这属于采集范围覆盖差异，不能把差值 23 当作丢帧。Record 起点约为 Trace 后 7.17 秒，分析窗口开头约 2.17 秒的 Record 覆盖不足。

Prediction 输入/输出上下文、应用阶段与 Worker 实例保持独立证据。Fusion 的 parent 集合来源于活动传感器缓存；审计代码把同一触发 LiDAR 时间写入每个 parent 的 `sensor_ts_ns`。因此恢复 Header parent 集合能证明消息标签关系，不能证明各 parent 在融合算法中实际共同参与。'''
stats_rows=[]
for label,key in [('Callback 墙钟','callback_statistics_window_ms'),('Fusion代理→proc_exit','release_response_window_ms'),('callback→Writer返回','writer_return_window_ms')]:
    stats_rows.append(dict(metric=label,**global_[key]))
whole='''## 3. L1：持续长延迟集中在 90—95 秒

窗口内 callback 中位数为 **7.523 ms**，P95 为 **127.417 ms**，P99 为 **158.759 ms**，最大值为 **227.192 ms**。release-to-completion response 的最大值为 **291.587 ms**，出现在 F677；该帧 callback 为 162.925 ms，额外的 128.662 ms 是 Fusion 输出代理到 callback 入口的差值。

'''+table(stats_rows,[('metric','指标'),('count','实例数'),('mean','Mean ms'),('median','Median ms'),('p95','P95 ms'),('p99','P99 ms'),('max','Max ms')])+'''

按完整 Fusion 更新边界，693 个实例全部可判定，35 次 miss，比例 **5.05%**。90—95 秒内 34 次 callback 中有 29 次 miss，比例 **85.29%**；最长连续 miss 为 9 次，分别发生在 F654—F662 和 F664—F672。F558 与 F722 是连续高延迟段之外的独立 miss，也保留在全量账本中。

候选筛选沿用全量候选并集表：callback/阶段长尾、派发代理差值、miss、输出间隔异常；112 条候选或对照记录均保留原始选取原因。上游源帧遗漏激活与未消费输入需要完整触发合同及输入账本，当前记录不据此诊断业务丢帧。

![响应时间与更新边界 Slack](figures/fig01_response_slack.png)

F647—F680 的阶段均值显示 Evaluator 为 **136.461 ms**，其中 ThreadPoolRun 为 **128.895 ms**；Predictor 为 2.261 ms，Writer 为 0.323 ms。Evaluator、ThreadPoolRun 与 callback 存在嵌套，不能把这些均值相加。

![应用阶段](figures/fig02_major_stages.png)

完整连续甘特图及 6 页阶段图见 [F647—F680 图集](../figures/prediction_F647_F680/README.md)。'''
normal='''## 4. 对照与工作量：区分长尾、更新超时及可比基线

工作量签名按逻辑 group 保存障碍物成员数、各模型调用数及 forward 次数，避免将软件线程 TID 当作 group。F648 为 4 个任务、8 个任务内障碍物成员、20 次 forward；F675 为 5 个任务、7 个成员、15 次 forward，并含 JointlyPredictionPlanning。不同工作量不能直接相减后归因于系统。

正常运行参考 F398 按可复现规则选择：在至少重复 5 次且全部 meet 的非空 Worker 签名中，选 callback 中位数最低的签名，再选该签名中最接近中位数的实例。F398 的 callback 为 **15.251 ms**，1 个任务、1 个成员、2 次 forward。它展示正常路径，不作为 F648/F675 的同工作量因果对照。

F659 与 F653 有相同的已观测 group/模型/forward 计数。F653 满足更新边界，但 callback 达 161.200 ms，本身也是 P99 长尾；F659 callback 为 187.198 ms，相差 **25.998 ms**。两者的派发代理分别为 11.554 ms 与 42.702 ms，说明 update miss 同时取决于执行、启动和上游更新间隔。该差值是实测对比，未被解释为某种机制的可归因损失。

F648 没有重复的精确观测签名；F675 只有同样 miss 的 F674。当前窗口没有为这两帧提供低延迟的精确签名正常对照。此前 F664、F670 均为 miss，报告将其归为详细取证样本。

同签名中位数基线采用 leave-one-out、至少 4 个 peer，独立保存为 `baseline_same_signature_median_ms`。F659 的基线为 144.347 ms，差值 +42.851 ms；该量是条件统计差值，不是回归预测或因果增量。'''
states_text='''## 5. L2：父线程墙钟主要表现为 Sleeping，Worker 同时存在 Running 与 Runnable

以下各行按单线程、单实例统计，时间已转换到 BOOTTIME 后与原始 `thread_state` 求交。Running 表示 CPU 驻留；Sleeping 保留睡眠/阻塞含义。各行状态和等于该行 BOOTTIME 墙钟，RAW callback 与之存在微小时钟尺度差异。

'''+table(selected,[('frame','Frame'),('callback_ms','Callback RAW ms'),('running_ms','Running ms'),('runnable_ms','Runnable ms'),('sleeping_ms','Sleeping ms'),('dispatch_proxy_ms','派发代理 ms'),('slack_ms','Slack ms')])+'''

F648 父线程中 Running 11.104 ms、Runnable 19.956 ms、Sleeping 196.136 ms；F675 分别为 10.190、3.816、185.133 ms。父线程 Sleeping 与 Worker 执行并行重叠，不相加为 callback 增量。

![逐线程状态](figures/fig03_callback_states.png)

![F648应用与线程状态对齐图](figures/fig06_crosslayer_F648.png)

F648 的 Task 1832 在 TID 944342 上观测到 208.042 ms 任务包络，其中 Running 38.396 ms、Runnable 36.772 ms、Sleeping 132.875 ms。Task 1832 的 `task_end` 位于 packaged-task 返回之后，Future 可能已 ready；其包络长于 ThreadPoolRun 时不能强制建立 `task_end→future_get_return` 因果边。

F675 的 Task 2360 包络为 180.577 ms，Running 49.760 ms、Runnable 20.771 ms、Sleeping 110.046 ms；其余任务并行执行。各 Worker 的原始区间与提交、group_end、future_get 边界见 `case_worker_states.csv`、`case_task_dependencies.csv` 和原始 `thread_state_spans.csv`。'''
scheduling='''## 6. L3：唤醒和 CPU 占用恢复到实际事件

详细取证覆盖 9 个既有选定帧。`wakeup_events.csv` 保存 4,405 条可恢复的唤醒/调度记录：按目标 TID 和原始 Runnable 起点匹配 `sched_waking`，再关联同一 episode 中唯一成功 `sched_wakeup` 与首次 schedule-in；缺失边界保留空值。R/R+ 切出产生的 Runnable 也纳入状态时间。

`cpu_competition.csv` 保留 15,503 条等待与 CPU 驻留的重叠记录。CPU 采用目标线程随后实际恢复的 CPU；亲和性、配额、全部候选 CPU 和采集时调度策略没有完整快照，因此表中占用者是竞争候选。`cpu_overlap_candidates.csv` 的汇总是并行等待累计线程时间，不能当作单帧关键路径墙钟或某线程造成的延迟。

Waker TID 表示事件上下文；中断上下文会单独标记。唤醒者身份用于建立调度关系，等待对象在下一节用阻塞栈与任务边界核实。'''
mechanisms='''## 7. L4：Future/Futex 与设备同步等待路径得到事件栈支持

本次将 `sched_switch` 事件栈与实际 Waiting episode 关联：样本必须属于同一 frame、task 和 TID，落在该 Sleeping 之前紧邻的 Running 区间内，且该 Running 区间内候选事件栈唯一。PERF 事件栈采样发生在切换前，原始状态边界与样本之间的差值单列保存；该差值与时钟校准残差分开处理。共恢复 **4,415 个唯一关联 episode**。

F648 父线程 3 个 Future/Futex episode 共 **195.595 ms**，最长单段 **155.334 ms**，调用链包含 `_M_futex_wait_until → EvaluatorManager::Run → MessageProcess::OnPerception`。F675 对应路径累计 **179.496 ms**。源码中的 Future 等待与实际阻塞栈相互支持，说明父线程在这些区间等待并行评估完成。

F648 的 Task 1832 中，关联 PI Futex 路径的 Sleeping 为 **95.403 ms**，关联 nvhost syncpoint 路径为 **37.469 ms**。PI 路径包含 `__pthread_mutex_lock_full → futex_lock_pi → rt_mutex_wait_proxy_lock`；设备路径包含 `ioctl → nvhost_ctrlctl → nvhost_syncpt_wait_timeout`。F675 的 Task 2360 中相应为 **102.110 ms** 与 **7.896 ms**。这些是实际执行到的阻塞路径及关联 episode 时长。

![等待路径](figures/fig04_waiting_paths.png)

| 候选机制 | 已形成的证据链 | 待闭合的对象级证据 | 结论级别 |
|---|---|---|---|
| 父线程等待评估任务 | 源码 Future Get、任务身份、Future Get 区间、父线程 Futex 栈、实际 Sleeping | 精确 Ready、通知与 callback 恢复的完整对应；异步嵌套任务 | 方法 E2 + 机制线索；工作区 E2 |
| Worker PI Futex 等待 | 同 TID 的 mutex/Futex 事件栈、切出与等待 episode | 同一锁地址、持有者、释放/交接及所有者任务 | 方法 E2 + 机制线索；工作区 E2 |
| nvhost syncpoint 等待 | 驱动等待函数、同线程 episode、任务/阶段区间 | 请求/stream/event 与具体 kernel/copy 的完成链 | 方法 E2 + 机制线索；工作区 E2 |
| Worker Runnable 延迟 | 校准后的就绪区间、唤醒事件、实际恢复 CPU 占用 | CPU 资格、配额/策略、对应运行队列及替代解释 | 方法 E2；工作区 E1 |

事件栈存在 `repeated_frame`、`UNWIND_INFO` 等展开错误，完整链与错误标记一并保留。核心已观察路径可读；没有把缺失的上层调用补成已确认路径。PI Futex 现象不能直接推出优先级反转，nvhost 路径也不能单独推出 GPU 内部调度瓶颈。

本文使用 BCOZ 的 On/Off-CPU 联合观察思路组织等待线索，未执行其虚拟加速实验；使用 Kairos 的时间标签与时序约束区分消息时序，未实现论文的在线编译器传播与约束调度。[BCOZ](https://www.usenix.org/system/files/osdi24-ahn.pdf)，[Kairos](https://www.usenix.org/system/files/osdi24-li.pdf)。'''
impact='''## 8. L5：量化更新超时和消息传播，不把累计等待等同于可归因损失

'''+table(selected,[('frame','Frame'),('response_ms','Fusion代理响应 ms'),('callback_ms','callback ms'),('slack_ms','更新 Slack ms')])+'''

F677 的响应时间达到 **291.587 ms**，更新 Slack 为 **−149.844 ms**；F648 的对应值为 **276.109 ms** 与 **−116.316 ms**。F653 callback 达 P99 长尾却满足更新边界，说明“耗时长”和“更新超时”是两个独立判定维度。

693 个窗口实例中，**690 个（99.57%）**有显式消息身份关联至 Planning `RunOnce`；3 个实例缺少该端点，单独保留。此处证明局部 Prediction 输出进入实际 Planning 执行路径，全部物理源 lineage 因 Fusion 活动缓存与侧输入身份口径保留 partial。

Writer 入口代理到 Planning RunOnce 的实测间隔：中位数 **0.748 ms**、P95 **4.235 ms**、P99 **51.992 ms**、最大 **127.934 ms**。这些值说明已确认消息的处理传播间隔；完整 Data Age 需要源物理时间。是否形成车辆级行为影响需进一步关联 Control 与车辆实际响应。

本轮保留实测 callback、响应、Slack、同签名基线差值和机制可归因量四类字段。机制可归因增量为 NA；没有把父线程全部 Sleeping、并行 Worker 时间总和或 CPU 占用重叠直接计为可避免延迟，也没有据此预测优化后的 miss 数。'''
stability='''## 9. 输出节奏与时间标签属性

输入节奏采用 Prediction callback 入口差值，输出节奏采用 Writer 返回差值，均为应用同域观察端点。窗口内部相邻的 692 个间隔中，输入均值 **144.257 ms**、最大 **548.516 ms**；Writer 返回间隔均值 **144.372 ms**、P99 **388.242 ms**、最大 **562.835 ms**。该统计保留 `interval_next_frame_uid`，可追踪长间隔两端。

采用 Kairos Stability 的间隔偏差跨度：对 20 个连续输出间隔计算 `max(Δ−I)−min(Δ−I)`，固定 I 在跨度中消去，因此无需虚构期望周期。673 个完整窗口的跨度中位数 **72.196 ms**，最大 **506.844 ms**。没有正式 θs 时，报告实测波动范围。

![更新间隔和窗口跨度](figures/fig05_intervals.png)

| 属性 | 可计算覆盖 | 原因/口径 |
|---|---|---|
| Callback / 更新 Slack | 693/693 | 明确 RAW 端点、完整 Fusion 更新流 |
| 输入与 Writer 返回间隔 | 692 个相邻对 | 两个端点均处于有效窗口 |
| 20 间隔 Stability Span | 673/693 实例 | 前 20 个窗口历史端点不足 |
| 严格物理 Freshness | 0/693 | 跨主机源时钟及物理源标签未核实 |
| 严格多源 Consistency | 0/693 | 实际共同消费的独立多源时间标签未恢复 |
| 主消息链至 Planning RunOnce | 690/693 | 显式身份关联与独立 RunOnce 打点 |

REALTIME 时钟映射只解决主机时钟转换；不能替代传感器 Header 时间的物理语义核验。各 parent 重复的时间标签跨度为 0 不作为一致性良好的证据。'''
accept='''## 10. 方法阶段验收与复现

| 阶段 | 本轮交付 | 证据状态 |
|---|---|---|
| P0 数据质量 | manifest、coverage、quality、时钟/身份/完整性检查 | 5—105 秒局部通过；整轮失败原因保留 |
| P1 实例关联 | 1,057 Frame、1,929 Task、阶段与身份边 | 全量应用实例保留 |
| P2 时间校准 | 47 快照、RAW→BOOT 转换、独立验证 | 已应用于线程求交与栈归属 |
| P3 图与路径 | TDFG、Execution 节点边、时间有效 TID 区间 | 实证边可查询；完整 Future/CRoutine 关键路径部分缺失 |
| P4 时间归因 | 693 callback、724 Task 精确状态；原始区间 | 逐线程闭合；并行量与墙钟分列 |
| P5 机制证据 | 9 帧详细取证、4,415 episode、对象级缺口矩阵 | 检出与时间归因成立，局部机制路径得到支持 |
| P6 干预验证 | 保留后续实验配置表 | 本轮未开展；不生成优化收益 |

方法文档 E1/E2/E3/E4 分别表示检出/时间归因/机制/干预；工作区 E0/E1/E2/E3 分别表示覆盖不足/观测/机制支持/因果闭合或干预。报告分列两套语义，不进行编号平移。对象级链路与可量化因果增量独立验收。

复现命令（项目目录 `D:\\data\\runtime-analysis-perfetto`）：

```powershell
python scripts/analyze_realtime_report.py
python scripts/write_realtime_report.py
python scripts/query_frame.py --frame-uid "<data/frame_analysis.csv 中的完整 frame_uid>" --output frame_evidence.json
```

全量基础表的既有 30 项校验结果保存在 `../tests/validation_results.json`；本报告新增实例计数、区间/样本关系、RAW 响应重算与空值语义校验见 `validation_results.json`。图表以实际 CSV 生成；原始采集文件、Apollo 源码和采集配置保持原样。

正文写作采用用户指定的 [anti-defensive-writing Skill](https://github.com/Adkid-Zephyr/anti-defensive-writing-Skill/blob/main/skills/anti-defensive-writing/SKILL.md)：围绕已成立的证据组织论证，主张范围与可核查数据一致。配套交互报告使用 Data 报告组件保存逐图数据与来源。'''
conclusion='''## 11. 结论

本轮数据形成了“连续异常实例 → Evaluator/ThreadPoolRun 阶段 → 实际线程状态 → 阻塞事件路径 → 上游更新边界与下游消息”的可复核分析链。90—95 秒容纳窗口内 82.86% 的更新超时；F648、F675 的父线程等待与 Worker 的 PI Futex、设备同步路径可逐区间查询。该结果为后续针对等待对象、任务完成通知和实际恢复的定向取证提供了准确实例与时间窗口。

后续闭合机制时，优先补充同锁地址的持有/释放、Future Ready/通知/恢复及设备请求完成身份，并选取相同模型与任务签名的正常重复实验。受控实验分别检验锁等待、设备同步和 Runnable 延迟，使用实测响应及带符号 Slack 评价贡献。'''
sections=[brief,contract,quality,whole,normal,states_text,scheduling,mechanisms,impact,stability,accept,conclusion]
title='# Prediction 持续长延迟的逐帧跨层证据分析\n\nRun `pw_20261020_141904` · 应用全量账本与 Trace 5—105 秒系统取证\n\n'
(OUT/'prediction_realtime_analysis_report.md').write_text(title+'\n\n'.join(sections)+'\n',encoding='utf-8')
# Report blocks declare the exact query populations they discuss.
scope=[['frames','bins','cases'],['frames'],['frames'],['frames','bins'],['comparisons','frames'],['cases','workers'],['workers'],['waits','workers'],['cases','downstream'],['intervals'],['frames','waits'],['bins','waits']]
blocks=[dict(id=f'section-{i}',text=text,queries=scope[i]) for i,text in enumerate(sections)]
(OUT/'report_app/src/content/report/narrative.json').write_text(json.dumps(blocks,ensure_ascii=False,indent=2),encoding='utf-8')
snapshot=json.loads((OUT/'reviewed_snapshot.json').read_text(encoding='utf-8'))
current=json.loads((OUT/'report_app/src/data.json').read_text(encoding='utf-8'))
snapshot['id']=current['id']
snapshot['buildStatus']='creating'
for q in snapshot['queries'].values():
    key=next(k for k,v in snapshot['queries'].items() if v is q)
    names=dict(frames='frame_analysis.csv',bins='five_second_bins.csv',cases='case_callback_states.csv',workers='case_worker_states.csv',comparisons='matched_comparisons.csv',waits='waiting_path_summary.csv',downstream='downstream_runonce.csv',intervals='interval_stability.csv')
    x=OUT/'data'/names[key]
    q['source']['files']=[dict(name=x.name,sha256=__import__('hashlib').sha256(x.read_bytes()).hexdigest())]
    q['source']['metricDefinitions'][0]['componentIds']=[b['id'] for b in blocks if key in b['queries']]+['chart-'+key,'table-'+key]
(OUT/'report_app/src/data.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2),encoding='utf-8')
print(OUT/'prediction_realtime_analysis_report.md')
