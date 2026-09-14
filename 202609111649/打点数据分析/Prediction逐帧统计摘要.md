# Prediction逐帧统计摘要

## 统计范围与口径

- run：`202609111649`
- 主输入关联：`fusion_inputs` 中 `is_main_sensor=1` 且 `sensor_kind=lidar`
- Prediction自身执行时间：`proc_enter → proc_exit` 的 `mono_ns` 墙钟差
- 统计异常候选：完整帧 `execution_ms` 严格大于全程P99；这只是统计尾部，不等同于已定位故障
- deadline：当前Fusion输出为release，完整Fusion输出流中的下一份真实输出为deadline；不是固定100 ms，也不是Apollo配置契约

## 主要结果

- Prediction trace帧：1193，完整帧：1193
- 执行耗时：中位数3.514240 ms，P95 10.879808 ms，P99 14.333795 ms，最大19.007168 ms
- 执行P99尾部候选：12帧：91、411、513、514、560、586、715、718、772、906、988、989
- 局部执行突增候选：16帧，详见 `lidar_prediction_reanalysis/local_execution_jumps.csv`
- Fusion关联：1192/1193；主LiDAR入口关联：1191/1193。缺失均保留为空值，不补造
- 下一真实Fusion deadline：1192个可判定job，0 miss，miss rate 0%
- signed slack：中位数93.992768 ms，P05 82.808730 ms
- 相邻完整callback退出间隔：中位数99.442512 ms，P95 116.898517 ms，最大621.161280 ms（帧938）。该字段不是严格writer发布时间差
- 最大输出间隔附近的Fusion实际输出间隔为621.328832 ms（seq 1808→1812）；seq 1808对应帧937的deadline因此也按621.328832 ms计算，没有套用固定周期

## 障碍物与工作量判断

record与trace可完整对齐1102帧；record未覆盖帧1–91，因此尾部帧91没有障碍物数量证据。其余11个执行P99尾部帧中：

- 9帧感知/Prediction障碍物均为0
- 帧772和988各为1个障碍物，且均为Ignore；没有Caution、Normal或Interactive
- 本run没有配对到CruiseMLP或Joint Prediction-Planning模型调用；VectorNet精确调用打点不可用
- ThreadPool总执行时间可用，但逻辑group身份和每group障碍物映射缺少打点证据，相关字段明确标记不可用

因此，这11个有record证据的异常候选并非由障碍物数量普遍增加所解释。帧91因record覆盖边界无法作同样判断。

## 数据质量与特殊事件

- TID不一致、负派发时间、内部节点未配对：均为0
- Perception与Prediction trace writer均报告overflow、allocation failure、truncated record和I/O error为0；Fusion输出序号虽有跳号，但现有健康记录不支持将其解释为trace writer丢记录
- 发现173个 `semantic_base_async_draw_error`。每次错误发生时，同TID唯一活动子阶段均为 `semantic_draw_roads`；逐实例表将错误事件作为该子阶段终止点，并保留 `completion_status=error` 与 `terminal_phase`
- Prediction帧1早于Fusion上下文采集起点，写入 `prediction_jobs_without_fusion_match.csv`，不参与deadline计算
- perf目录中的 `perf.data` 为0字节；本次任务只生成打点与record统计，未据此开展perf分析

## 输出索引

- `lidar_prediction_reanalysis/prediction_frames.csv`：全部Prediction逐帧时间及节点汇总
- `lidar_prediction_reanalysis/prediction_node_instances.csv`：内部节点逐实例时间与终止状态
- `lidar_prediction_reanalysis/execution_tail_frames.csv`：12个执行P99尾部候选
- `prediction_workload/prediction_frame_obstacle_counts.csv`：逐帧障碍物分类、模型调用和ThreadPool字段
- `prediction_workload/execution_tail_comparison.csv`：可与record对齐的11个尾部帧工作量对照
- `prediction_next_fusion_deadline/prediction_jobs_next_fusion_deadline.csv`：逐job下一真实Fusion deadline结果
- `prediction_next_fusion_deadline/validation_and_summary.json`：deadline汇总与验证结果

## 可复现命令

```powershell
python D:/data/reusable_scripts/prediction/analyze_prediction_lidar.py --run-dir D:/data/202609111649 --output-dir D:/data/202609111649/打点数据分析/lidar_prediction_reanalysis
python D:/data/reusable_scripts/prediction/analyze_prediction_obstacle_counts.py --run-dir D:/data/202609111649 --timings-dir D:/data/202609111649/打点数据分析/lidar_prediction_reanalysis --output-dir D:/data/202609111649/打点数据分析/prediction_workload
python D:/data/reusable_scripts/prediction/analyze_prediction_next_fusion_deadline.py --frames D:/data/202609111649/打点数据分析/lidar_prediction_reanalysis/prediction_frames.csv --fusion-context D:/data/202609111649/trace/message_context/perception.multi_sensor_fusion.3772789.csv --output-dir D:/data/202609111649/打点数据分析/prediction_next_fusion_deadline
```
