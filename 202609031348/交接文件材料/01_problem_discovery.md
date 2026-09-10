# 1. 实时性问题发现过程

本文件只复述既有分析步骤、指标和证据边界，不新增根因判断。数据源 run 为 `202608271537`；交付目录所在的 `202609031348` 仅是本次材料的目标位置。

## Step 1：模块级 deadline

既有报告将 Perception freshness deadline 定义为：本帧 P1 `proc_enter` 到 P7 `output_pub` 必须不晚于下一帧 P1 `proc_enter`。约 100 ms 表示 P1 输入帧间隔，不是 sensor 产生到进入 Perception 的传输时延。

| 指标 | 既有结果 |
| --- | ---: |
| 可判帧 | 687 |
| deadline miss | 646 / 687 |
| miss rate | 94.03% |
| 输入周期 P50 | 99.783 ms |
| response time P50 | 114.198 ms |
| response time P95 | 211.882 ms |
| response time MAX | 447.912 ms |

来源：[`module_deadline_summary.csv`](raw/original_reports/application_analysis/data/module_deadline_summary.csv)、[`sections_1_1_1_2.md`](raw/original_reports/application_analysis/report/sections_1_1_1_2.md)。

## Step 2：节点级 deadline

| 节点 | 可判实例 | deadline miss | miss rate | 已有判断 |
| --- | ---: | ---: | ---: | --- |
| P1 PointCloudPreprocess | 687 | 0 | 0.00% | 未形成大量自身 deadline miss |
| P2 MapBasedROI | 687 | 0 | 0.00% | 未形成大量自身 deadline miss |
| P3 GroundDetection | 687 | 0 | 0.00% | 未形成大量自身 deadline miss |
| P4 LidarDetection | 687 | 224 | 32.61% | 主要异常节点；123 execution-driven / 53 waiting-driven / 48 no-service |
| P5 LidarDetectionFilter | 639 | 0 | 0.00% | 自身无新增 miss；仅统计真正从 P4 到达的实例 |
| P6 LidarTracking | 639 | 0 | 0.00% | 自身无新增 miss；仅统计真正从 P4 到达的实例 |
| P7 MultiSensorFusion | 639 | 0 | 0.00% | 自身无新增 miss；仅统计真正从 P4 到达的实例 |

P2–P7 的 input-ready 使用上一节点 `output_pub` 代理；严格 Reader receive/enqueue/drop 标记不可用。P5–P7 的 639 个实例只覆盖真正从 P4 到达的实例。

来源：[`perception_node_proxy_deadline_summary.csv`](raw/original_reports/application_analysis/data/perception_node_proxy_deadline_summary.csv)、[`perception_node_proxy_deadline_audit.csv`](raw/original_reports/application_analysis/data/perception_node_proxy_deadline_audit.csv)。

## Step 3：异常段划分

既有结果从 P4 deadline miss 逐帧数据中得到 76 个连续 miss 段，并提取 4 个主要异常段：

| 段 | 帧范围 | span | 既有起点说明 |
| --- | --- | ---: | --- |
| S1 | F17-F38 | 22 | F17 precursor miss; F18 execution spike; repeated at F22/F29 |
| S2 | F61-F95 | 35 | F61 execution spike |
| S3 | F308-F331 | 24 | F308 execution spike |
| S4 | F611-F649 | 39 | F611 execution spike |

来源：[`p4_contiguous_miss_runs.csv`](raw/original_reports/application_analysis/data/p4_contiguous_miss_runs.csv)、[`p4_major_abnormal_segments.csv`](raw/original_reports/application_analysis/data/p4_major_abnormal_segments.csv)。

## Step 4：为什么进一步使用 perf

应用层打点已经记录 P4 execution 长尾、deadline miss、waiting、no-service/frame skipping，以及异常连续维持多个 source frame；这些数据不能区分 execution 增长期间线程是在等待 CPU、sleep/block，还是获得 CPU 后的 CPU Running 时间增加。因此既有工作针对异常段继续检查 perf sched。本交接不进一步判断 OS 根因。

## Step 5：perf sched 得到了什么

既有 S4 分解将 F601–F610 作为异常前参考，将 F611/F613/F616/F619/F623 作为严重 CPU Running inflation 的有完整 Infer 实例重点帧，并记录 F626 起的恢复参考。完整逐帧表见 [`02_system_observation.md`](02_system_observation.md) 和各阶段 `infer_sched_frame_summary.csv`。

现有 perf 数据观察到：严重异常窗口中 CPU Running 明显增加，而 runnable waiting / scheduler delay 相对较小。该句是现有观测的整理，不扩展为 OS 根因判断。

## Step 6：为什么继续采集 CPU Stack

perf sched 给出线程状态时长，但不能直接说明 CPU Running 时运行在哪些函数。因此既有工作在 F611、F613、F616、F619、F623 的严格 Infer 时间窗口内保留 CPU stack samples。每个逐帧文件保留 sample timestamp、comm、原始数值 ID、CPU、`cpu-clock` event 和完整 user + kernel call chain，不去重、不按符号筛选。
