# t1-t2 逐帧固定阈值时延分析

`timing_anomaly_detector.py` 计算窗口内每个 Trace 帧的 Perception、Prediction、Planning 和内部 E2E 耗时，并与人工配置的固定阈值比较。

这是显式固定阈值超限检测，不是由车速、距离和制动能力推导的物理隐形 deadline。物理隐形 deadline 使用 `implicit_deadline_analyzer.py`。

独立运行：

```powershell
python D:\data\anlysis_case\timing_anomaly_detector.py `
  --case-dir D:\data\case `
  --out-dir D:\data\case\timing_output `
  --functional-result D:\data\case\analysis\classification_result.json
```

输出：

- `timing_frame_latencies.csv`：窗口内全部 Trace 帧；
- `timing_anomaly_frames.csv`：E2E 超过配置阈值的帧；
- `timing_analysis_result.json`：统计和归因；
- `timing_e2e_scatter.svg`：E2E 散点图。

结果新增准确字段 `explicit_deadline_threshold_miss`。旧字段 `implicit_deadline_miss` 暂时保留用于兼容旧调用方，但不应再把它解释为物理隐形 deadline。
