# 202609081856 Prediction 分析

本次有效口径是主 LiDAR 血缘关联 + Prediction `proc_enter → proc_exit` 墙钟耗时。未指定模块 deadline，不计算固定 100 ms deadline/miss rate。11帧是完整 callback 耗时超过全程 P99（155.3447872 ms）的统计尾部候选，不等于已证明计算瓶颈。

## 结果入口

- [逐帧统计与说明](打点数据分析/lidar_prediction_reanalysis/分析说明.md)
- [F800–F950 甘特图](打点数据分析/lidar_prediction_reanalysis/figures_800_950)：151个 Prediction callback 序号，不是 LiDAR 源帧序号。分5页；灰色为上游、橙色为派发代理、蓝色为 callback，细条为内部节点。
- [11帧 perf/stack 汇总](perf和stack数据分析/frame_summary.csv)：F809、824、827、847、851、852、854、887、888、889、925。
- [时钟校准](perf和stack数据分析/clock_calibration.json)及[逐样本验证](perf和stack数据分析/stack_alignment_validation.csv)。

每个 Fxxxx 文件夹提供帧身份、实际 callback/worker TID、trace节点实例、校准后时间窗、目标线程的 sched 原始事件（前后各100 ms），以及 callback 执行窗内的完整 CPU 调用栈。`active_trace_nodes` 为空的样本只是在同一 TID/时间窗，不保证属于该帧任务。

## 可复用脚本检查与修正

`generate_p4_perf_analysis.py --help` 和 `extract_perf_sched_frame_windows.py --help` 均可在本机运行。前者总入口绑定 P4/CP_INFER，后者的 timehist 模式不能使用本次空的 timehist 导出（采集报告显示不支持 show-prio 参数）。因此新增轻量 Prediction 适配脚本，直接导入通用脚本的 TID快照解析、时间范围读取、sched解析/分解和完整stack解析函数，未复制P4逻辑。

原始sched中的 `sample_tid=-1` 事件仍含有效 prev/next TID，旧正则会漏读。通用解析器已允许负采样TID，并新增回归测试；没有修改历史分析结果。源文件SHA-256已在本地 SOURCE_MANIFEST 更新，本次上传的文件校验值见 prediction_delivery_manifest.csv。

## 时钟与证据边界

通过同TID运行区间校准得到 `sched_s = trace_ns/1e9 + 0.5091642640763894`，所有所选帧节点入口/出口验证通过。

CPU stack 与 sched 不能直接共用时钟：采用含漂移的一次模型，`sched_s = stack_s + intercept + slope*(stack_s-reference)`。1412个样本交替分为706个拟合样本和706个验证样本；验证样本同TID、同CPU匹配率100%（2微秒文本舍入容差）。参数见校准JSON。这是窗口内经验校准，不应外推至其他run；仅有采集前TID快照，没有after快照。

逐帧分解满足 running + blocked + runnable + unknown = callback墙钟耗时，11帧 unknown 均为0。running仅表示CPU驻留，blocked包括等待工作线程等；`sched_waking` 作为等待分界代理，不能当严格 `sched_wakeup`。例如F888 callback耗时269.131 ms，但主callback线程CPU驻留仅13.577 ms、blocked为253.185 ms；因此不能把269.131 ms全称为自身CPU计算。工作线程的完整stack也已保存，进一步根因判断需要结合这些证据。

## 复现

在仓库根目录使用 Python 3（绘图额外需要 Pillow、当前使用Windows Arial字体）：

```powershell
python "202609081856/打点数据分析/scripts/analyze_prediction_lidar.py" --run-dir "202609081856"
python "202609081856/打点数据分析/scripts/plot_prediction_800_950.py"
python reusable_scripts/perf/test_sched_unknown_sample_tid.py
python "202609081856/打点数据分析/scripts/extract_prediction_perf.py"
```

perf提取最后一步需要本地 `prediction_run_010/perf_sched_script.txt`。GitHub包含分析所需Prediction原始打点/日志、主LiDAR关联输入、CPU stack原始文本与身份快照、校准及逐帧提取结果。完整约15 GB sched文本、约3 GB sched map、record内容、其他模块无关数据和已撤回的固定deadline旧结果未上传。全局目标TID校准缓存只保留本地，逐帧原始证据已上传。
