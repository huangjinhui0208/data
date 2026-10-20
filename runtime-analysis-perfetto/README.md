# 5—105 秒跨层数据交付

Run：`pw_20261020_141904`。结果目录：`D:\data\runtime-analysis-perfetto`。

窗口原点为 Trace Processor `trace_bounds.start_ts=1032146343301613` BOOTTIME ns。统计窗口 `[1032151343301613,1032251343301613)`，即 Trace 相对 5—105 秒；北京时间约 2026-10-20 14:19:10.582—14:20:50.582。它与命令启动、READY 或应用首事件的零点不同。

## 交付范围与数量

全部 **1,057** 个 callback 和 **1,929** 个已开始 Task 的身份账本均保留；完整落入窗口的 **693** 个 callback、**724** 个 Task 用于系统分解。窗口外账本不计入该窗口的性能统计。完整配对应用阶段 **36,058** 个，保留嵌套、非零 proc_id 和异步阶段。

| 文件 | 行数 | 可用性 |
|---|---:|---|
| run_manifest.json | JSON | available_with_per_row_limits |
| trace_coverage.csv | 7 | available_with_per_row_limits |
| trace_quality.csv | 7 | available_with_per_row_limits |
| frame_instances.csv | 1057 | available_with_per_row_limits |
| worker_tasks.csv | 1929 | available_with_per_row_limits |
| frame_stage_spans.csv | 36058 | available_with_per_row_limits |
| identity_edges.csv | 6844 | available_with_per_row_limits |
| clock_snapshots.csv | 47 | available_with_per_row_limits |
| clock_alignment.json | JSON | available_with_per_row_limits |
| tdfg_nodes.csv | 8410 | available_with_per_row_limits |
| tdfg_edges.csv | 7353 | available_with_per_row_limits |
| execution_nodes.csv | 45451 | available_with_per_row_limits |
| execution_edges.csv | 46260 | available_with_per_row_limits |
| dynamic_path_segments.csv | 193853 | available_with_per_row_limits |
| frame_timing_metrics.csv | 1057 | available_with_per_row_limits |
| task_state_attribution.csv | 724 | available_with_per_row_limits |
| thread_state_spans.csv | 193853 | available_with_per_row_limits |
| critical_path_segments.csv | 12501 | partial_critical_path_not_certified |
| callstack_samples.csv | 9834 | available_with_per_row_limits |
| wakeup_events.csv | 4405 | available_with_per_row_limits |
| cpu_competition.csv | 15503 | available_with_per_row_limits |
| root_cause_evidence.csv | 52 | available_with_per_row_limits |
| intervention_runs.csv | 0 | headers_only_no_intervention |
| before_after_metrics.csv | 0 | headers_only_no_intervention |

所有表以 run_id/frame_uid 关联；task_id 的全局作用域是本轮进程生命周期。trace_id 和所有大整数 ID 按字符串读取。CSV 空值不是 0；原始应用 RAW 时间与转换后的 *_boot_ns 同时保留。P4/P5 的原始输入是 Trace Processor 已归一化的 BOOTTIME，不能再次加应用时钟偏移。

## P0：采集质量与覆盖

原始采集 `FAILED_OR_INCOMPLETE` 保留。三个 perf 源触发 guardrail，均发生在选定窗口之后；这不消除窗口内部的 unwind 错误。5—105 秒有 system-wide sched **24,214,611** 条；CPU-clock/perf-switch/perf-waking 样本分别 **118,800 / 1,051,246 / 1,051,938** 条，对应 unwind error **1,596 / 45,807 / 95,407**。有样本不等于栈完整。

目标进程采集前身份快照记录 PID/Host PID 942621、starttime_ticks=103208480、同 PID namespace。采集结束后身份快照失败仍为限制；窗口内用 TP 线程所属进程、utid 及生命周期交叉核对。采集时 Apollo commit、功耗模式、完整 CARLA actor/weather 配置未知，未用当前运行配置补成历史事实。

初次 SQL 导出在前七个结果集完成后，中止了耗时的 CPU 关联语句；该作业返回码 1 原样保留。仅使用已完整输出并通过 gzip CRC/行数核验的七个结果集。CPU 上下文、窗口计数及质量计数由第二个独立作业成功完成（返回码 0），随后在本机按实际等待区间关联。见 metadata/system_summary.json；这与原始采集失败标记分开记录。

## P1/P2：身份与时钟

47 组快照支持 RAW→BOOTTIME 分段线性 offset 插值。偶数快照拟合、奇数快照留出验证；最终参数保留所有快照，禁止外推。选定窗口内留出最大残差 **5.059 μs**。独立应用事件验证 **2388** 个，同 TID Running 支持率 100%，100 μs 容差下最大到运行区间距离 2653 ns。另有 **11,606** 个 perf 样本在同 TID、同 CPU 的 2 μs 容差内支持率 100%。支持率与时间误差分别报告。

时钟快照插值的误差不是严格上界，极短 Task/状态边界仍有微秒级归属不确定性。窗口外可生成时间映射，但未作为已通过本窗口系统审计的样本使用。

## P3：数据流、执行与时间指标

消息依赖按 Channel+trace_id+sequence+data_ts 匹配，并用源码语义确认；多父 Header 集合来自 fusion_inputs，而非只取 primary_parent；这些父节点来自 active sensor trace 缓存，实际算法逐输入消费未观测，故源传感器→Fusion 边保留为 unverified。窗口内主链完整关联 **690/693 (99.57%)**。此“完整”仅限 Fusion 输出消息→Prediction→Planning RunOnce 主消息链。完整物理来源血缘为 0；全部实例 lineage_status 保留 partial。Localization、Storytelling、历史 Planning 等身份也未全恢复。

Record 显式匹配结果：Trace 输出 1,057，Record 输出 1,034，共同消息 **1,001**；Trace-only **56** 个均早于 Record 第一份 Prediction 输出序号；Record-only **33** 条。全量差值 23 不等于丢帧。覆盖详见 metadata/record_trace_matches.csv 和 metadata/record_only_outputs.json。

窗口 callback 墙钟：均值 **26.915 ms**，中位数 **7.523 ms**，p95 **127.417 ms**，p99 **158.759 ms**，最大 **227.192 ms**。Fusion 发布代理→callback 退出的响应均值 29.808 ms；该量与 callback 自身耗时分列。

沿用本工作区“完整 Fusion 输出流下一份真实输出”更新边界策略，693 个可判定实例中 35 个超出边界，比例 **5.05%**。该策略是分析者选择的更新预算，**不是 Apollo 安全契约**；没有固定 100 ms Safety Deadline。signed slack 使用真实下一输出时间减 proc_exit；相等记 meet。

输入间隔用 proc_enter，表示 callback 入口节奏，不是 Reader 到达。输出间隔用 writer_done，表示 Write 返回节奏；692 个窗口内相邻间隔，最大 **562.835 ms**。20 个连续输出间隔的跨度共有 **673** 个完全在窗口内的结果。没有正式 I/theta，故不判稳定性违约。

物理源时间语义和跨主机同步未验证：Freshness 和严格 Consistency 均空值且有原因。源码显示 fusion_inputs.sensor_ts_ns 对每个父都复制触发帧的 in_message->lidar_timestamp_，不是该传感器独立观测时刻。复制值保存在 diagnostic_copied_frame_timestamp_ns，fusion_audit_copied_tag_span_ms 只审核复制标签；即使为 0，也不能证明多源一致性良好。

## P4：可复核的状态区间

保存 **193,853** 条 callback/Task 裁剪状态区间，其中原始 state、state_row_id、原始完整区间边界、utid、CPU、waker_utid 和裁剪后边界均可追溯。所有 693 个 callback 和 724 个 Task 的未知时长为 0，状态闭合误差为 0 ns。Sleeping(S/I) 与 Uninterruptible(D) 分列、不重复相加；R 与 R+ 都计 Runnable。

callback 线程累计量（ms）：{"running_ms": 3192.154525, "runnable_ms": 1460.710711, "sleeping_ms": 13999.443736, "uninterruptible_ms": 0.0, "unknown_ms": 0.0}。

Task 线程累计量（ms）：{"running_ms": 10980.818218, "runnable_ms": 6818.519424, "sleeping_ms": 23872.839847, "uninterruptible_ms": 0.153954, "unknown_ms": 0.0}。

这些是线程累计量，父 callback 与并行 worker 有重叠，不能相加为关键路径或根因贡献。Task 排队时间的精确 enqueue 点未观测，queue_wait_ms 留空，worker_tasks.csv 保留上下界。

critical_path_segments.csv 提供逐 callback 的已观察状态包络和关键帧候选 Task 分支。缺少精确 Future Ready、完整异步完成/通知链，**尚未认证完整关键路径**；critical_flag 留空。没有用某个最晚 task_end 或父线程全部睡眠来补造关键路径。

## P5：关键帧证据与结论边界

全量候选并集见 metadata/anomaly_candidate_union.csv：callback/stage/派发代理长尾、更新边界 miss、输出停顿及正常对照均有单独规则，共 112 个候选/对照实例。详细调用栈/唤醒/CPU 证据覆盖 **9** 个代表实例：7 个窗口 callback≥p99 实例及 2 个按障碍物/forward 次数匹配的对照。其他模型和历史状态的可比性未完全控制，不据此量化因果贡献。

详细样本保存 **9,834** 条栈记录、**4,405** 条唤醒/就绪记录、**15,503** 条 CPU 重叠记录。普通采样、switch 事件栈和 waking 事件栈分列。栈的部分展开和原始 unwind_error 原样保存。

cpu_competition 使用目标线程随后实际恢复 CPU 的可见占用，只证明重叠，不证明该 CPU 是唯一可运行 CPU，也不证明占用者造成了全部延迟。策略/亲和性/配额未完整记录时保持缺失。中断上下文的 waker TID 也不自动等于真正设备/锁的唤醒主体。

root_cause_evidence 仅为 E1 观测候选。当前未确认锁持有者、GPU kernel/stream、完整 Future 通知链或系统机制造成的可归因增量。工作区等级为 E0–E3，用户提到的 E4 没有既定定义，因此不生成 E4。无 GPU 时间线时，CUDA/驱动函数只能提供 CPU 端线索。

## P6：未实施优化实验

intervention_runs.csv 和 before_after_metrics.csv 均为有 schema 的空表。当前没有干预组、重复实验或优化后数据，不虚构 before/after 数值，不将相关性写成因果验证。

## F647—F680 甘特图

[34 帧总览 PNG](figures/prediction_F647_F680/prediction_gantt_F647_F680_overview.png)、[矢量 SVG](figures/prediction_F647_F680/prediction_gantt_F647_F680_overview.svg)及[6 页阶段详图和绘图数据](figures/prediction_F647_F680/README.md)。全部图使用 F647 callback 入口为统一零点，保留阶段嵌套与实际 TID；不将阶段累计耗时解释为关键路径。

## 复核与复现

### Prediction 实时性分析报告

按本轮方法形成的[完整报告](prediction_realtime_report/prediction_realtime_analysis_report.md)及[交付索引](prediction_realtime_report/README.md)已补充：693 个窗口实例、五层案例证据、4,415 个事件栈关联等待 episode、同签名对照与 6 张科研图。正文使用用户指定的 anti-defensive-writing Skill。交互版构建成功；浏览器安全策略阻止页面验视，此项单独保留为未通过的检查。

本机依赖仅 Python 标准库与 numpy。原始大型 Trace、应用 CSV 和 Record 仍位于 Orin，本机 inputs/ 是证据缓存且不加入 Git；仓库提交 24 个要求文件、分析脚本、源码快照、SQL、元数据、测试与本文。

```powershell
$py = 'C:\Users\22142\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $py -X utf8 D:\data\runtime-analysis-perfetto\scripts\build_tables.py application
# 重新提取时：将 metadata/extraction_request.json 及 scripts/extract_perfetto_remote.py
# 放到 /media/ssd/hjh/runtime_analysis_exports/pw_20261020_141904_5_105/，运行远端脚本。
# 再运行 scripts/extract_context_remote.py，并将两个导出目录复制回 inputs/。
& $py -X utf8 D:\data\runtime-analysis-perfetto\scripts\build_tables.py system
& $py -X utf8 D:\data\runtime-analysis-perfetto\scripts\build_tables.py final
& $py -X utf8 D:\data\runtime-analysis-perfetto\scripts\write_report.py
& $py -X utf8 D:\data\runtime-analysis-perfetto\scripts\validate_tables.py
```

按某一 frame_uid 查询所有关联表可用 scripts/query_frame.py；不要把窗口外实例纳入窗口统计。验收结果见 tests/validation_results.json；原始文件散列、产物 SHA-256 和数据字典在 metadata/ 下。

方法参考：本项目是离线证据整理与部分图恢复；不声称部署了 Kairos 编译器插桩或 BCOZ 因果实验。[Kairos §4](https://www.usenix.org/system/files/osdi24-li.pdf)提供时间标签和时序约束模型；[BCOZ](https://www.usenix.org/conference/osdi24/presentation/ahn)启发联合保存 On-/Off-CPU 证据，但其因果分析并未由这些观测表自动实现。[Perfetto SQL 文档](https://perfetto.dev/docs/analysis/sql-tables)用于核对 utid、thread_state 和 sched 的语义。
