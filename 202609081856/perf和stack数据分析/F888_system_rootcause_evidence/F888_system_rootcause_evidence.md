# F888 CruiseMLP worker 系统层根因证据

## 范围与证据口径

本次分析直接复用既有逐 worker 状态账本，不重新计算 Prediction 工作量、deadline、callback 状态统计，也不重新论证 F888 的同负载异常属性。应用打点通过当前 run 的校准参数转换到 perf sched 时间域。校准文件记录 trace marker 支持率为 100%，本报告的系统竞争与唤醒关系来自原始 system-wide sched 事件。

`blocked` 只表示线程从 CPU 切出到首次 sched_waking 的睡眠区间；`runnable` 只表示线程已可运行但尚未重新获得 CPU。二者不等同于具体锁等待、GPU 完成等待或其他设备完成等待。

## 关键事实

- F888 ThreadPoolRun 相对 F887 与 F889 均值增加 110.506320 ms。F888 关键 worker 是 TID 1646949。
- 关键 worker 的 blocked 增量为 74.082000 ms，占 ThreadPoolRun 增量的 67.04%。这是关键路径状态账本的时间归属，不是具体阻塞对象的因果证明。
- 关键 worker 的 runnable 增量为 32.241499 ms，占 ThreadPoolRun 增量的 29.18%。
- 关键 worker 的 running 增量为 4.156501 ms，占 ThreadPoolRun 增量的 3.76%。
- 三类状态增量合计 110.480000 ms，与 ThreadPoolRun 增量相差 0.026320 ms。该残差来自 ThreadPoolRun 边界与关键 worker span 边界的差异。

## 10.347 ms 应用空隙与 10.071 ms 调度等待

F888 TID 1646949 的模型 2 结束到模型 3 开始间隔为 10.347168 ms；对应 runnable 区间为 10.071000 ms，解释比例为 97.33%。两者边界差为 0.276168 ms。二者属于同一绝对时间窗口，CSV 可按 TID 1646949、模型序号 3 和 runnable 起止时间反查。剩余时间由模型标记边界与 scheduler 状态边界之间的小段执行和状态转换构成。

## 两段重点 Runnable Waiting 的直接解释

- TID 1646959 在 968509.388319000–968509.412974000 s 等待 24.655000 ms。它先在 CPU0 的可运行队列等待，968509.412062 s 迁移到 CPU8，968509.412974 s 获得 CPU。指定 CPU 在该段被其他线程占用 24.640000 ms，空闲仅 0.015000 ms；主要占用者是 `dds.udp.27457[1648118]` 7.940000 ms、`dds.udp.27417[1644778]` 4.900000 ms、`mainboard[1645550]` 2.737000 ms 和 `ksoftirqd/0[12]` 2.666000 ms。`irq/134-host_sy[174]` 占用 0.816000 ms，不是该段最大竞争者。
- TID 1646949 在 968509.546242000–968509.556313000 s 等待 10.071000 ms。它先在 CPU0 等待，968509.556033 s 迁移到 CPU1，968509.556313 s 获得 CPU。指定 CPU 在该段被其他线程占用 10.060000 ms，空闲仅 0.011000 ms；主要占用者是 `dds.udp.27457[1648118]` 6.131000 ms、`ksoftirqd/0[12]` 1.413000 ms 和 `irq/134-host_sy[174]` 1.007000 ms。

这两段都有完整的 assigned-CPU 占用闭合。直接原因是 worker 已可运行，但其当时所在 CPU 持续运行其他线程；在迁移到另一 CPU 后才重新获得执行。现有事件不包含 CFS vruntime、调度域负载均衡决策和 CPU affinity 的时点快照，因此无法进一步证明为何调度器没有更早迁移。

## 唤醒依赖链

原始 sched_waking 事件验证链条真实存在：TID 1646959 在 24.655 ms runnable waiting 后于 968509.412974000 s 重新获得 CPU，并在 968509.413060000 s 唤醒 TID 1646942，两者间隔 0.086000 ms；随后 TID 1646942 在 968509.413130000 s 唤醒 TID 1646949，前后唤醒事件间隔 0.070000 ms；TID 1646949 在 968509.413441000 s 唤醒 TID 1646953，前后唤醒事件间隔 0.311000 ms。该链证明唤醒先后与直接 waker，不证明睡眠期间等待的内核对象。

## irq/134-host_sy

原始 sched 事件确认完整线程名为 `irq/134-host_sy`，TID 为 174。它是内核 IRQ 线程，但现有数据没有 `/proc/interrupts`、IRQ action 和设备驱动映射，因此不能称为 GPU IRQ。

ThreadPoolRun 内运行时间：F887 为 2.093999 ms，F888 为 5.189000 ms，F889 为 2.709001 ms。F888 对 worker runnable 的时间重叠为 3.697000 ms。是否显著增强应以该三帧表中运行时间、段数与最大连续运行时间共同判断，不能只因它出现在异常窗口就认定为主因。

## 证据级回答

### A. Blocked 能解释多少 ThreadPoolRun 增长

按关键路径 worker 状态账本，blocked 增量为 74.082000 ms，占约 67.04%。该数值回答时间归属；具体 blocked 对象仍需同步原语或设备等待 trace 才能定因。

### B. Runnable Waiting 能解释多少 ThreadPoolRun 增长

按关键路径 worker 状态账本，runnable 增量为 32.241499 ms，占约 29.18%。

### C. Runnable Waiting 的主要 CPU 竞争线程

- `dds.udp.27457[1648118]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 16.357000 ms，sched 优先级数值为 120。
- `dds.udp.27417[1644778]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 8.232000 ms，sched 优先级数值为 120。
- `ksoftirqd/0[12]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 6.654000 ms，sched 优先级数值为 120。
- `mainboard[1645550]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 3.061000 ms，sched 优先级数值为 120。
- `mainboard[1645479]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 2.593000 ms，sched 优先级数值为 120。
- `irq/134-host_sy[174]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 1.850000 ms，sched 优先级数值为 49。
- `mainboard[1646161]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 1.613000 ms，sched 优先级数值为 120。
- `MainThread[1560836]` 在七段大于 2 ms 的 runnable 区间中累计覆盖 0.912000 ms，sched 优先级数值为 120。
Linux sched 优先级数值越小，调度优先级越高。四个 Prediction worker 的记录值为 120；`irq/134-host_sy[174]` 的记录值为 49，但它在七段长 runnable waiting 中的累计占用并非最大。逐段 CPU 路由、优先级和全部竞争线程见 `f888_runnable_delay_rootcause.csv`，独立明细见 `f888_runnable_cpu_occupancy.csv`。

### D. Blocked 的主要 waker 与依赖对象

waker 可以由 sched_waking 直接确定。按大于 2 ms 的 blocked 区间汇总如下；不同 worker 的 blocked 可以并行重叠，因此这些时长不能相加后与 ThreadPoolRun 直接比较。

- `mainboard[1646949]`：20 次直接唤醒，对应 blocked 区间累计 113.531000 ms。
- `mainboard[1646942]`：18 次直接唤醒，对应 blocked 区间累计 107.876000 ms。
- `irq/134-host_sy[174]`：27 次直接唤醒，对应 blocked 区间累计 99.545000 ms。
- `mainboard[1646959]`：11 次直接唤醒，对应 blocked 区间累计 71.760000 ms。
- `mainboard[1646953]`：8 次直接唤醒，对应 blocked 区间累计 34.982000 ms。

长 blocked 区间中存在 worker 间串行唤醒，且上述三段链已由原始事件验证。blocked 期间等待的具体对象，现有数据无法证明。`sched_waking` 只证明谁执行了唤醒，不包含 futex 地址、锁 owner、CUDA event、dma-fence 或设备队列标识。

### E. irq/134-host_sy 是否为 F888 特有或显著增强

它不是 F888 特有，因为 F887 与 F889 均有运行记录；F888 ThreadPoolRun 内运行时间高于两帧对照，分别是 F887 的 2.48 倍和 F889 的 1.92 倍。F888 运行段数不是三帧最高，但最大连续运行段为 0.434000 ms，高于 F887 的 0.099000 ms 和 F889 的 0.131000 ms。局部三帧可描述为运行时间与最长连续段增强，但样本不足以进行总体统计显著性判断。它与 worker runnable 的精确重叠量见对照 CSV；即使增多，也只能证明同期 CPU 竞争活动增强，不能证明它触发了 blocked。

### F. PI pthread mutex 是否已量化为主要根因

目前只能证明 PI pthread mutex 慢路径在 CPU stack 采样中存在。现有数据无法证明它覆盖了主要 blocked 时长，也无法量化为主要根因。单个或少量 CPU stack 样本不能外推整段 off-CPU 时间，perf sched 也没有用户态 mutex 地址与 owner。

## 下一次采集需要补充的证据

- 保留 `sched:sched_switch`、`sched:sched_waking`、`sched:sched_wakeup`、`sched:sched_migrate_task`，并同时记录 raw trace，避免文本转换丢失字段。
- 增加 `syscalls:sys_enter_futex` 与 `syscalls:sys_exit_futex`，或者使用 eBPF uprobe 跟踪 `pthread_mutex_lock`、`pthread_mutex_unlock`、futex wait 与 futex wake；必须保存 mutex 地址、owner TID、waiter TID、PI 标志和返回码。
- 若要区分 GPU 完成等待与其他设备完成等待，增加 CUDA CUPTI 或 Nsight Systems 事件，并采集平台可用的 `dma_fence`、`nvhost`、GPU submit、GPU complete tracepoint，保留 correlation id。
- 采集 `/proc/interrupts`、`/proc/irq/134/*`、`/proc/174/status`、`/proc/174/stack`、`/proc/174/sched`、`/proc/174/cgroup` 和 `/sys/kernel/irq/134/actions`，用于把 `irq/134-host_sy` 映射到确切设备与驱动。
- 在采集前后保存所有目标 TID 的 `/proc/<tid>/status`、`sched`、`cgroup`、`comm`、CPU affinity 与调度策略，避免优先级和绑核配置只能从 perf 事件间接推断。

## 文件索引

- `f888_system_sched_timeline.csv`：两个关键窗口的全 CPU sched_switch 运行区间。
- `f888_runnable_delay_rootcause.csv`：七段大于 2 ms runnable waiting 的 CPU 路由与竞争线程。
- `f888_runnable_cpu_occupancy.csv`：每段 runnable waiting 内每个实际 CPU occupant 的独立明细。
- `f888_worker_block_dependency.csv`：大于 2 ms blocked 区间及直接 waker。
- `f888_worker_dependency_timeline.png`：第一关键窗口的 worker 状态与唤醒链。
- `irq_host_sync_same_workload_comparison.csv`：F887、F888、F889 的 IRQ 线程对照。
- `model_system_state_alignment_887_888_889.csv`：每次 CruiseMLP 模型调用与系统状态对齐。
