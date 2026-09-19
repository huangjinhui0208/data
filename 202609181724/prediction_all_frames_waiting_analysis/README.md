# 输出说明

本目录保存 202609181724 run 的 Prediction 全帧调度等待统计。主结论见 `全帧Prediction调度等待统计报告.md`，质量门槛见 `validation.json`。

时间字段约定：

- `enter_ns`、`exit_ns`、`prediction_enter_ns`、`prediction_exit_ns`：Apollo trace 原始单调时钟纳秒。
- `aligned_sched_start_s`、`aligned_sched_end_s`、episode 的 `start_s`、`end_s`：应用当前 run 仿射校准后的 perf sched 秒。
- `rca_cause_coverage_ok=False`：状态统计仍可用，但 Notify RCA syscall、waker、竞争者和调用栈原因不可作为完整证据。
- `confidence=medium`：事件链存在，但源报告有全局丢 chunk 告警。
- `confidence=unknown`：原因链缺失或超出 RCA 覆盖。

复算入口为 `D:\data\reusable_scripts\prediction\analyze_prediction_all_waiting.py`。该脚本对 49 GB system-wide RCA 文本进行一次顺序扫描，运行时间较长；已有 CSV 可直接用于后续统计，不需要重复扫描原始文件。
