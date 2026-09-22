# Prediction同工作负载组perf与CPU stack提取

本目录沿用 `same_workload_10N_10M_26I` 的证据提取口径，对六个具体工作负载组合进行处理。`9，9，22` 与 `9，9，23` 作为一个相邻工作负载对照，因此对应五项分析任务、六个输出目录。

## 时间对齐与覆盖

- trace到perf sched使用当前run已经验证的偏移，参数保存在 `clock_calibration.json`。
- 本次重新核验的trace marker支持率为1.0，CPU stack同TID同CPU支持率为0.996237。
- 每帧提取窗口包含callback区间及前后各100 ms上下文。
- 只有完整落入已验证perf扫描区间的帧进入sched、CPU stack和工作线程甘特图。其余同工作负载帧保留在各组的 `matching_frames_coverage_audit.csv`，不作为完整系统状态样本。

## 组合与实际提取帧数

| 组合 | 全run匹配帧 | perf完整覆盖帧 |
| --- | ---: | ---: |
| 7，7，18 | 20 | 16 |
| 1，1，2 | 35 | 13 |
| 8，8，21 | 31 | 17 |
| 9，9，22 | 10 | 10 |
| 9，9，23 | 11 | 11 |
| 7，7，17 | 18 | 18 |

共提取85帧。完整帧列表见 `group_coverage_summary.csv` 和各组的 `selected_frames.csv`。

## 目录结构

- `Fxxxx`：逐帧原始提取证据，包括trace节点、sched上下文、CPU stack样本和提取窗口。
- `frame_summary.csv`：85帧callback的running、blocked、runnable和采样覆盖汇总。
- `group_*`：每个工作负载组合的选择表、覆盖审核、分组frame summary和工作线程分析。
- `group_*/worker_gantt/worker_model_scheduler_gantt.png`：模型调用、推理区间和线程调度状态叠加甘特图。
- `group_*/worker_gantt/worker_cpu_residency_heatmap.png`：工作线程在实际CPU上的驻留时间热力图。
- `group_*/worker_gantt/*.csv`：模型、worker、调度时间线、blocked区间、CPU驻留和stack样本明细。
- `output_manifest.csv`：输出文件大小和SHA-256清单。

## 证据边界

- 甘特图泳道表示软件工作线程，不表示固定CPU核心。
- 灰色blocked区间表示线程处于睡眠等待状态，不能仅凭该状态断言等待GPU完成。
- 橙色runnable区间表示线程可以运行但尚未获得CPU。
- CPU stack是采样证据，不等于精确CPU时间，也不能替代perf sched的off-CPU统计。
- 12帧的callback TID在callback窗口内没有采到CPU stack样本，对应的 `cpu_stack_callback_tid.txt` 为空；这些帧仍保留同帧其他实际TID样本，空文件不表示时间未对齐或callback未执行。
- 当前数据没有逻辑ThreadPool group标识，不能把worker TID直接解释为逻辑group。
