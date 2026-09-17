# Prediction 同路径 P99 系统状态基线

## 样本选择

- `p99_abnormal`：`jobs.csv` 中 `execution_ms_p99_tail=True` 的全部 12 帧。
- `normal`：上述 P99 帧在 `normal_controls.csv` 中已有正常对照的并集，去重后为 30 帧。
- 42 帧都具备当前 run 的 callback 调度状态和阶段调度状态。
- F91 缺少工作量记录，它的三个对照仅满足同 run、邻近、非候选和同观察路径条件，不能视为工作量严格匹配。
- F988 没有专属正常对照，因此本目录支持队列级比较，不支持 F988 的逐帧配对比较。

## 同路径定义

这里的“同路径”是可观察路径签名一致：

| 字段 | 42 帧共同取值 |
| --- | ---: |
| `async_schedule_calls` | 1 |
| `async_submit_calls` | 1 |
| `notify_tasks_calls` | 1 |
| `async_draw_calls` | 1 |
| `async_completion_status` | `ok` |

每帧还都包含一个 `semantic_base_async_draw` 实例和 16 个 `cyber_taskmanager_notify_task` 子实例。该定义证明已观测节点及调用次数一致，不等价于未插桩控制流完全一致。

## 文件说明

- `same_path_frame_selection.csv`：样本角色、路径计数、callback 边界、异步绘制边界及两者重叠时长。
- `same_path_callback_system_states.csv`：callback 的 running、runnable、blocked、unknown 及等待汇总。
- `same_path_notifytask_system_states.csv`：`NotifyTasks` 父阶段时长，以及持续时间最长的 `NotifyTask` 子实例状态。
- `same_path_cohort_summary.csv`：正常组和 P99 组各指标的描述统计。
- `same_path_validation.json`：路径一致性、状态闭合、关键中位数和等待集中度校验。

`waiting_ms` 定义为 `runnable_ms + blocked_ms`，`waiting_share` 为 `waiting_ms / execution_ms`。`async_overlap_ms` 是 callback 区间与异步绘制区间的交集长度。

## 验证结果

两组的可观察路径签名完全一致。正常组 callback 执行时间中位数为 3.217360 ms，P99 组为 16.316096 ms；对应等待时间中位数从 0.187500 ms 增至 13.376500 ms，等待占比中位数从 0.058484 增至 0.819142，而 running 时间中位数分别为 2.860404 ms 和 2.894312 ms。

P99 组中，持续时间最长的 `NotifyTask` 子实例所含等待占 callback 全部等待的中位比例为 0.993134，最小值为 0.602478。证据支持以下命题：在已观测路径一致的样本中，P99 callback 的额外墙钟时间主要来自等待增加，并且等待主要落在最长 `NotifyTask` 子实例区间内。

这些调度状态不能识别具体等待对象，不能据此断言锁竞争、GPU 等待或其他设备等待。

## 复现

运行：

```powershell
python D:\data\202609111649\打点数据分析\系统层实时性分析_20260915\prediction\build_same_path_p99_baseline.py
```
