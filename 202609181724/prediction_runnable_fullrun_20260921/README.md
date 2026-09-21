# Prediction NotifyTask Runnable 全量事件重建

## 范围

- 输入为当前 run 的 14,016 个 `cyber_taskmanager_notify_task` 实例。
- 打点边界先使用已验证的当前 run 仿射校准转换到 perf sched 时间域。
- `01_runnable_abnormal_episodes.csv` 仅保留与 NotifyTask 相交且单次时长不小于 1 ms 的 Runnable episode。
- 本目录只保存事件重建和描述性统计，不进行根因分类。

## 结果规模

- Runnable episode：104 个，分布在 101 帧。
- `start_type=wakeup`：90 个；其中 89 个在目标线程事件中能回溯到此前的非 Running switch-out，1 个未观察到此前 switch-out。
- `start_type=preempted`：14 个，起点为 `sched_switch prev_state=R`。
- episode 时长：最小 1.017 ms，中位数 4.233 ms，P75 5.601409 ms，P95 8.995655 ms，最大 19.457057 ms。
- 相关调度时间线：9,388 行；目标 CPU occupancy：860 行；estimated runqueue：9,672 行；CPU availability：104 行。

## 文件语义

- `01_runnable_abnormal_episodes.csv`：总索引。`start_type` 是观测事件类型，不是根因类型。
- `02_runnable_sched_timeline.csv`：episode 所在目标 CPU 与目标 TID 相关的 `sched_waking`、`sched_switch`、`sched_migrate_task` 原始字段化时间线。本次 base perf sched 文本未出现 `sched_wakeup` 或 `sched_wakeup_new`。
- `03_runnable_cpu_occupancy.csv`：目标当前 CPU 路径上的完整 residency，保留 `swapper/N`。
- `04_runnable_runqueue.csv`：有限预热窗口内由观测事件重建的 `estimated_runqueue` 下界。由于无法证明预热开始前没有已 Runnable 任务，所有行的 `coverage_ok=false`。
- `05_runnable_cpu_availability.csv`：CPU0–CPU11 的 idle overlap。仅在 12 个 CPU 的 episode 起点状态均能由预热窗口内 switch 事件建立时，`coverage_ok=true`。
- `06_irq_softirq_overlap.csv`、`07_cpu_idle_power_overlap.csv`：只有表头，因为本次 capture 没有对应 tracepoint。
- `perf_event_inventory.txt`：记录事件清单核查及限制。
- `manifest.json`、`quality_audit.json`：输入、口径、记录数和质量限制。

## 质量限制

- 当前 Windows 分析机没有兼容该 aarch64 `perf.data` 的 Linux perf，因此没有实际执行 `perf evlist`；事件清单退化为采集命令与已解码 perf script 的联合核查。
- 原始命令是 `perf sched record -a --clockid CLOCK_MONOTONIC`。已解码 base sched 文本只有调度事件，IRQ、softirq 和 `power:cpu_idle` 均不可从本次历史数据恢复。
- notify RCA 采集报告丢失 277,767 chunks，无法安全定位到具体 episode，因此 `rca_cause_coverage_ok=false`。
- base perf sched 报告 1 个 out-of-order event；目标 Runnable 链中有 1 个 episode 不完整，详见 `quality_audit.json`。
- `swapper/N`、空闲 CPU、优先级计数、迁核和被切出事件均为观测事实，不能由提取脚本自动解释成调度根因。
