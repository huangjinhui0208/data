# 3. Frame → Time → TID → Raw Data 映射说明

主索引：[`frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。索引覆盖 F601–F649，共 49 个 source frame；空值表示当前证据中不存在该字段，没有用相邻帧、均值或模型值补齐。

## 字段与时钟

- `perception_input_time`：应用层 `source_time_unix_s`，Unix wall-clock 秒。
- `p4_input_ready_time`：`e2e_trace_v3` 中的 `input_ready_proxy_mono_ns`；P4 使用上游 P3 `output_pub` 作为 input-ready 代理，不是严格 Reader 排队时间。
- `p4_proc_enter` / `p4_output_time`：`e2e_trace_v3` 事件的 `mono_ns`。
- `infer_start` / `infer_end`：Perception log 中唯一配对的 `CP_INFER_ENTER/EXIT mono_ns`；现有 validation 已验证该时间域与 `perf_cpu_stack` 的 `CLOCK_MONOTONIC` samples 对齐。
- `perf_window_start` / `perf_window_end`：与 Infer 窗口对齐后的 perf sched 秒。
- 时钟校准方法：`same-TID + same-CPU CPU-clock samples aligned to sched_switch reconstructed run intervals`。
- 校准值：`sched_minus_stack = -297346430 ns`；支持样本 667 / 676（98.67%）。
- `host_tid`：由原始 `/proc` task status、Host `ps` before/after 快照和现有 identity audit 给出的该帧实际 worker TID；不是固定的 LidarDetection 线程名。

`e2e_trace_v3 mono_ns` 与 Perception log 的 `CP_INFER mono_ns` 在当前文件中不是同一数值基准；例如 F611 的 P4 trace 时间与 CP Infer 时间不能直接相减。现有 Frame→Infer 关联来自 `centerpoint_internal_timing_per_source_frame.csv` 的 source-frame/sensor-time 映射和相邻日志行中的唯一 CP pair，而不是把这两个 `mono_ns` 字段直接对齐。当前交接未发现现成的 trace→CP 显式时钟 offset，因此保留两个原始时间字段并明确分域，不自行补算 offset。

## phase 边界

- F601–F610：`normal`（异常前参考；原审计角色为 `candidate_control`）。
- F611–F623：`core_abnormal`。
- F624–F625：位于 core 与 recovery 边界之间，按字段留空，避免混入任一概念。
- F626–F649：`recovery`。

## per-frame perf 文件边界

每个 `Fxxx_perf_sched_raw.txt` 由现有 `p4_sched_events_context.csv` 中、落在该帧严格 Infer perf 窗口内的 `raw_event` 行按原顺序导出；未去重、未改写行内容。该文件是目标 TID/kswapd0 等既有筛选上下文，不等同于窗口内全部 system-wide 事件。完整 system-wide 原文保留在 [`perf_sched_script.txt`](raw/perf_sched/perf_sched_script.txt)，原始二进制为 [`perf.data`](raw/perf_sched/perf.data)。
