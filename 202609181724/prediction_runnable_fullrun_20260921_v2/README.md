# Prediction NotifyTask Runnable 全量重建 v2

本目录只包含事件重建、描述性统计和质量审核，不包含最终根因分类。

## 统计口径

- 总 NotifyTask：14016
- 主样本：NotifyTask `runnable_ms >= 3.000 ms`
- 异常 NotifyTask：76
- 异常 Frame：73
- 异常 NotifyTask 内 Runnable episode：80
- wakeup episode：68
- preempted episode：12
- migration 相关 episode：33
- sched chain 完整：80；不完整：0
- sensitivity dataset：单 episode 不小于 1 ms，共 104 个

## 质量摘要

- Runnable reconstruction 绝对误差：max 0.000000000 ms，median 0.000000000 ms，P95 0.000000000 ms，超过 0.1 ms 的 NotifyTask 数 0。
- 01 到 02、03、04、05 的缺失数依次为：0、0、0、0。
- CPU occupancy 完整覆盖 episode：80 个，共 80 个。
- CPU availability `coverage_ok=true`：80 个；`coverage_ok=false`：0 个。
- estimated runqueue episode coverage level：{"fully_observed_since_state_init": 80, "partially_initialized": 0, "unknown_initial_state": 0, "trace_gap_affected": 0}。
- IRQ 与 softirq trace available：false。
- `power:cpu_idle` available：false。
- F317 scheduler chain repaired：true。
- Validation：PASS。

## 事件能力

`perf_event_inventory_orin.txt` 是本次事件能力的唯一依据。实际列出的 events：

- `sched:sched_switch`
- `sched:sched_stat_runtime`
- `sched:sched_process_fork`
- `sched:sched_wakeup_new`
- `sched:sched_migrate_task`
- `sched:sched_waking`
- `sched:sched_stat_wait`
- `sched:sched_stat_sleep`
- `sched:sched_stat_iowait`
- `dummy:HG`

IRQ、softirq 和 `power:cpu_idle` 不在清单中，因此 06 与 07 只有表头。03 与 05 仍保留 `swapper/N`、PID 0 和 `<idle>` 形式的 scheduler idle 证据，但不解释为具体 C-state。

## 覆盖边界

- `perf_sched_script.txt` 从第一条到最后一条 scheduler event 单遍流式回放，未使用 episode 局部 warm-up。
- `estimated_runqueue` 只代表可观察事件初始化后的重建状态，不是内核历史运行队列绝对真值。
- notify RCA 报告全局丢失 277,767 chunks；不能可靠定位到具体 episode，因此只记录 `global_trace_loss_present=true`、`episode_specific_trace_loss=unknown`。
- start type、occupant、migration、scheduler idle、priority 和 availability 均为观测字段，不自动解释为根因。
