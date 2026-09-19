# 202609181724 Prediction 全帧调度等待统计

## 结论

本 run 共重建 876 个完整 Prediction callback、14,016 个 NotifyTask 实例和 5,790 个 waiting episode。callback 状态闭合最大误差为 0.00031 ms，NotifyTask 状态闭合最大误差为 0。

全 run 的 callback 墙钟时间合计为 5,305.998 ms，其中 running 为 2,713.028 ms，占 51.13%；runnable 为 653.100 ms，占 12.31%；blocked 为 1,939.938 ms，占 36.56%。waiting 合计占 callback 墙钟时间的 48.87%。执行时间中位数为 4.195 ms，P95 为 12.844 ms，P99 为 17.520 ms；running 的 P99 只有 4.618 ms，而 runnable 和 blocked 的 P99 分别为 8.195 ms 与 11.834 ms。这个分布说明本 run 的长尾主要来自等待时间增长，而不是 callback 的 CPU 驻留时间同步增长。

在 Notify RCA 原因证据完整覆盖的 856 帧中，callback waiting 合计为 2,527.330 ms：

| 原因类别 | 帧数 | episode 数 | 总时长 | waiting 占比 |
| --- | ---: | ---: | ---: | ---: |
| futex 关联 blocked | 745 | 979 | 1,562.502 ms | 61.82% |
| 唤醒后 runnable | 841 | 1,098 | 476.396 ms | 18.85% |
| 直接 CPU 竞争 runnable | 157 | 301 | 156.291 ms | 6.18% |
| 原因未知 blocked | 170 | 205 | 329.852 ms | 13.05% |
| 原因未知 runnable | 14 | 21 | 2.289 ms | 0.09% |

这里的“直接 CPU 竞争 runnable”是互斥主分类，只覆盖以可运行状态切出且有占用者证据的 episode。若把唤醒后 runnable 也纳入 CPU 占用重叠审核，system-wide switch 序列覆盖了 366.425 ms，占全部可用 runnable 时间的 57.71%。这表示 runnable 延迟中有超过一半观察到了目标 CPU 被其他线程占用，但不能把同 CPU 重叠直接解释成单一线程的独立因果效应。

futex 关联 blocked 高度集中在两个相邻地址：`aaaaccc5ec3c` 为 792.029 ms，`aaaaccc5ec38` 为 742.461 ms；两者合计占 futex 关联 blocked 的 98.21%。这足以证明 callback 在本 run 中反复进入相同的一组 futex 同步等待路径。现有栈中出现 `futex_wait`、`__pthread_cond_timedwait`、`std::condition_variable::notify_one` 等符号，但 DWARF 解码存在失败且部分调用链方向不可信，因此不能把这两个地址直接命名为 `mtx_wq_`、某一把 mutex 或具体锁持有者。

没有观察到与 `sched_yield` 关联的 callback runnable episode。由于 Notify RCA 报告丢失 277,767 个 chunks，这一结果只能写成“未观察到”，不能证明整个 run 没有执行 `sched_yield`。观察到 435 个发生迁核的唯一 callback off-CPU 区间，涉及 384 帧；迁核与等待可以同时发生，不能单独作为根因。

## 时间对齐

本次只使用当前 run 的校准文件：

```text
t_sched = t_trace
        + 14.278004238280982
        + 1.290452120922855e-05 × (t_trace - 1826918.7012528323)
```

- 打点到 perf sched 使用 438 个训练同步点和 438 个留出同步点。
- 留出支持率为 100%，P95 绝对残差为 0.247 ms，最大绝对残差为 0.290 ms。
- CPU stack 使用 `CLOCK_MONOTONIC`；750 个留出样本的同 TID、同 CPU 支持率为 100%。
- callback 与 NotifyTask 的原始 `enter_ns`、`exit_ns` 和转换后的 sched 秒均保存在输出表中。
- 校准文件 SHA-256 为 `31fe4a09574691806fec29c6c9f2a8e50f3322c3535bccc4b4ae4f78126f3434`。
- 旧目录中的 `calibration_stop_report.md` 和 `quality_audit.json` 记录的是早期失败尝试，已经被后续带输入哈希的 verified 校准替代，不能作为当前状态。

## 全帧状态

| 指标 | 中位数 | P95 | P99 | 最大值 |
| --- | ---: | ---: | ---: | ---: |
| execution_ms | 4.195 | 12.844 | 17.520 | 23.713 |
| running_ms | 3.038 | 3.566 | 4.618 | 7.216 |
| runnable_ms | 0.069 | 5.018 | 8.195 | 19.511 |
| blocked_ms | 0.455 | 7.600 | 11.834 | 14.807 |
| waiting_ms | 1.006 | 9.493 | 13.859 | 19.656 |
| waiting_share | 24.15% | 76.14% | 81.96% | 85.16% |

waiting 最大的帧为 F617，execution 为 23.713 ms，waiting 为 19.656 ms，其中 runnable 为 19.511 ms。blocked 最大值出现在 F277，为 14.807 ms。逐帧完整数据见 `all_frames_system_states.csv`。

## NotifyTask 分布

每帧固定观察到 16 个 NotifyTask，按帧内 enter 时间编号。编号 16 的 waiting 总量最高，为 375.483 ms，占全部 NotifyTask waiting 的 15.35%；其中 66 次超过 1 ms，35 次超过 5 ms。编号 2 的 runnable 总量最高，为 163.832 ms。异常没有只集中在单一编号，因此当前证据更符合多个 NotifyTask 位置都可能遇到同步等待和调度干扰。

这些编号只是帧内时间顺序。现有 trace 没有 CRID 或 task name 字段，不能把编号 16 直接映射成 `/internal/taskN`。

## Runnable 证据

- 覆盖完整的 callback runnable 时间为 634.976 ms。
- system-wide CPU 占用重叠为 366.425 ms，占 57.71%。
- 最大累计单一占用者为 `kswapd0`，累计 21.920 ms；其后包括多个 `mainboard` worker、`kworker` 和采集用 `perf` 线程。
- `perf` 本身累计成为 12.344 ms runnable 区间的最大占用者，说明采集开销是不可忽略的干扰变量。
- 迁核发生在 435 个唯一 off-CPU 区间，涉及 384 帧。
- 未观察到 `sched_yield` 关联 episode。

竞争者汇总是“在目标预期 CPU 上运行的线程”证据，不表示该线程单独造成了全部 runnable 时长。唤醒后 runnable 与直接抢占 runnable 分开保存，避免把 blocked 结束后的 scheduler delay 并回 blocked。

## Blocked 证据

- 856 个 RCA 覆盖完整帧中，745 帧出现 futex 关联 blocked。
- 979 个 futex 关联 blocked episode 合计 1,562.502 ms。
- 前两个 futex 地址贡献 1,534.490 ms，占 futex 关联 blocked 的 98.21%。
- waker 分散在多个 `mainboard` worker TID；waker 只表示执行唤醒的线程，不等于锁持有者。
- callback 切出栈在 RCA 覆盖完整 episode 中的可用率只有 21.69%。可用栈主要出现 futex wait、pthread 条件变量和少量 pthread mutex 路径。

这些证据把主要 blocked 机制定位到 futex 支撑的同步等待，并显示两个稳定地址占主导。要把地址进一步映射到 Apollo C++ 对象，需要符号可靠的 off-CPU 采集、对象地址映射或源码级定向打点；当前结果不能越过这一证据边界。

## 数据质量与覆盖

- perf sched 状态覆盖全部 876 帧，状态 unknown 为 0。
- Notify RCA 原因证据完整覆盖 856 帧和 13,696 个 NotifyTask；F857 至 F876 超出 RCA 最后事件时间，只保留 sched 状态，原因字段标为覆盖不足。
- 5,790 个 waiting episode 中，5,555 个具有 RCA 时间覆盖，235 个覆盖不足。
- Notify RCA 解码报告处理 44,354,671 个事件并丢失 277,767 个 chunks。丢失单位是 chunk，不能直接换算为事件丢失比例。
- 线程身份只有采集前快照，没有采集后快照。当前 run 的同步点、TID 集合和 sched 运行区间一致，但仍保留生命周期风险。
- CPU stack 样本用于时间域和同 TID、同 CPU 验证，不用于替代 perf sched 计算 off-CPU 时间。

因此，状态总量可以作为全帧统计使用；具体 syscall、waker、竞争者和调用栈原因最高按中等置信度报告。缺少链路的 episode 保留 unknown。

## 输出文件

| 文件 | 内容 |
| --- | --- |
| `all_frames_system_states.csv` | 876 帧 callback 状态、episode 数、NotifyTask 汇总、路径标签和 RCA 覆盖 |
| `all_notifytask_system_states.csv` | 14,016 个 NotifyTask 的状态、帧内编号和 RCA 覆盖 |
| `all_waiting_episodes.csv` | 每个 runnable、blocked episode 的时间、CPU、waker、迁核、syscall、竞争者、栈、原因和置信度 |
| `waiting_cause_summary.csv` | callback 与 NotifyTask 两层的互斥主原因统计 |
| `notify_index_summary.csv` | 16 个帧内 NotifyTask 编号的 waiting 分布 |
| `futex_address_summary.csv` | callback blocked 的 futex 地址聚类 |
| `waker_summary.csv` | callback blocked 的 waker TID 聚类 |
| `runnable_competitor_summary.csv` | callback runnable 的最大 CPU 占用者聚类 |
| `blocking_stack_summary.csv` | callback blocked 切出栈签名聚类 |
| `waker_stack_summary.csv` | callback blocked 唤醒栈签名聚类 |
| `statistics_audit.json` | 分位数、总量、闭合与交叉校验 |
| `validation.json` | 校准、覆盖、输入哈希和数据质量结论 |

## 证据等级

- 全帧 callback 与 NotifyTask 的 running、runnable、blocked：E2，当前 run 校准和状态闭合已验证。
- futex 关联 blocked、唤醒后 runnable、system-wide CPU 占用者：E2，事件链可核查，但受丢 chunk 影响，正向分类最高为中等置信度。
- “两个 futex 地址对应某个具体 Apollo mutex”及“waker 就是锁持有者”：现有证据不支持。

## 来源记录

- 应用帧：`D:\data\202609181724\打点逐帧数据统计\prediction数据统计\lidar_prediction_reanalysis\prediction_frames.csv`
- 节点实例：`D:\data\202609181724\打点逐帧数据统计\prediction数据统计\lidar_prediction_reanalysis\prediction_node_instances.csv`
- 当前 run 校准：`D:\data\202609181724\prediction_system_rca_20260918\verified_clock_calibration\clock_calibration.json`
- sched 目标事件缓存：`D:\data\202609181724\prediction_system_rca_20260918\sched_relevant_verified_context.txt`
- system-wide Notify RCA 事件：`D:\data\202609181724\prediction_rca_01\notify_rca\notify_rca_script.stdout.txt`
- Notify RCA 状态与丢失告警：`D:\data\202609181724\prediction_rca_01\notify_rca\status.json`
- 线程身份：`prediction_proc_status_before.txt` 与 `prediction_ps_threads_before.txt`，位于 `prediction_rca_01`。
