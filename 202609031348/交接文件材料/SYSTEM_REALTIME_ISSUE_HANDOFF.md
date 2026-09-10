# Apollo Perception P4 实时性异常 OS 层交接

## 1. Case 范围

本交接关注 run `202608271537` 中 Apollo Perception P4 LidarDetection 的应用层实时性异常，以及已经完成的 perf sched / CPU stack 采集与对齐。交付目标是让 OS 同事能从 source frame 追到时间、实际 Host TID、capture-wide perf 原始数据和完整 stack samples；不重新寻找或判定根因。

S4 的应用层完整异常段为 F611–F649；当前具有明显 CPU Running inflation 且完成重点 CPU stack 整理的核心窗口为 F611–F623，两者不是同一概念。

## 2. 实时性问题发现步骤

Module deadline → P1–P7 node deadline → P4 LidarDetection → 连续异常段 → perf sched → CPU stack。详细定义、分母、指标和证据路径见 [`01_problem_discovery.md`](01_problem_discovery.md)。

## 3. 应用层已经观测到的问题

- Perception module：646 / 687 deadline miss（94.03%）；输入周期 P50 99.783 ms；response P50/P95/MAX = 114.198/211.882/447.912 ms。
- P4：224 / 687 miss（32.61%）；execution-driven 123，waiting-driven 53，no-service 48。
- 完整连续 miss 段 76 个；四个主要段为 S1 F17–F38、S2 F61–F95、S3 F308–F331、S4 F611–F649。
- P1–P3 未形成大量自身 miss；P5–P7 统计只覆盖实际从 P4 到达的实例。

## 4. 系统层已经观测到的问题

S4 的既有 perf 结果记录：F601–F610 参考帧 Infer 约 81–87 ms、CPU Running 约 29–34 ms；F611/F613/F616/F619/F623 的 Infer 为 276–327 ms、CPU Running 为 250–293 ms（约 89.7%–91.0%）。这些重点帧的 runnable waiting / max scheduler delay 见 [`02_system_observation.md`](02_system_observation.md) 逐帧表。

现有 perf 数据观察到严重异常窗口中 CPU Running 明显增加，而 runnable waiting / scheduler delay 相对较小。该观测不在本交接中继续扩展为 scheduler 或其他 OS 根因结论。

## 5. CPU Stack 已采集内容

F611、F613、F616、F619、F623 均保留严格 Infer 窗口内全部 stack samples，包含普通 Paddle/cuDNN 和已有 allocator/`cudaMalloc`/kernel path 样本，不去重、不只保留“有意义”样本。逐帧链接位于本文件第 9 节；已观察函数路径与证据边界见 [`02_system_observation.md`](02_system_observation.md)。

## 6. 重点异常窗口

- F601–F610：异常前参考（原审计角色 `candidate_control`）。
- F611–F623：严重 CPU Running inflation 核心窗口；有唯一 Infer 实例的重点帧为 F611/F613/F616/F619/F623。
- F624–F625：core 与 recovery 边界之间的 no-service 帧，不强行归入任一 phase。
- F626–F649：执行状态恢复参考；F640 无唯一 Infer 映射。

## 7. Frame / Timestamp / TID 对照

打开 [`frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。时钟、单位、校准和空值规则见 [`03_data_mapping.md`](03_data_mapping.md)。其中 `e2e_trace_v3 mono_ns` 与 Perception log `CP_INFER mono_ns` 分域保留，不直接相减；perf sched 只使用已经单独校准到 CP Infer/CPU stack 时间域的窗口。

## 8. perf 原始数据

- 原始二进制：[`perf.data`](raw/perf_sched/perf.data)
- 原始 sched script：[`perf_sched_script.txt`](raw/perf_sched/perf_sched_script.txt)
- 原始 timehist：[`perf_sched_timehist.txt`](raw/perf_sched/perf_sched_timehist.txt)
- 原始 map：[`perf_sched_map.txt`](raw/perf_sched/perf_sched_map.txt)
- collector 命令、stderr/stdout、latency 和状态文件：[`raw/perf_sched`](raw/perf_sched/)
- TID 映射证据：[`raw/tid_mapping`](raw/tid_mapping/)

## 9. CPU Stack 原始数据

- capture-wide 原始二进制：[`perf_cpu_stack.data`](raw/cpu_stack/perf_cpu_stack.data)
- capture-wide perf script：[`perf_cpu_stack_script.txt`](raw/cpu_stack/perf_cpu_stack_script.txt)
- 五帧逐帧完整 samples：

- F611: [perf raw-event extract](extracted/F611_F623_core/F611_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F611_cpu_stack_raw.txt)
- F613: [perf raw-event extract](extracted/F611_F623_core/F613_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F613_cpu_stack_raw.txt)
- F616: [perf raw-event extract](extracted/F611_F623_core/F616_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F616_cpu_stack_raw.txt)
- F619: [perf raw-event extract](extracted/F611_F623_core/F619_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F619_cpu_stack_raw.txt)
- F623: [perf raw-event extract](extracted/F611_F623_core/F623_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F623_cpu_stack_raw.txt)

## 10. OS 层后续分析入口

以下问题尚未在本交接材料中继续分析，可由 OS 层基于原始数据进一步判断。

- 从 F611 的 `frame_system_trace_index.csv` 行核对应用层/Infer/perf 三个时间域与 Host TID，再打开 F611 perf 与 stack 文件。
- 依次对照 F613、F616、F619、F623，必要时回到 capture-wide `perf_sched_script.txt`、`perf_cpu_stack_script.txt` 或两个 `.data` 文件重新生成视图。
- 使用 `raw/tid_mapping/` 中的 before/after `/proc` 与 Host `ps` 快照复核线程身份。
- 对尚未回答的问题保持为待分析项，不从本交接中的函数路径自动推导因果关系。
