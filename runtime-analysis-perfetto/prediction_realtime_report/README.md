# Prediction 实时性分析

Run：`pw_20261020_141904`。系统证据窗口：Trace 起点后 `[5,105)` 秒。

- [完整分析报告](prediction_realtime_analysis_report.md)
- [全量窗口逐帧分析表](data/frame_analysis.csv)：693 个实例、原始事件、RAW/BOOTTIME 时间及工作量签名。
- [五层案例证据](data/five_layer_case_evidence.csv)：正常参考与重点异常的 L1—L5 证据和缺口。
- [应用阶段线程状态](data/major_stage_states.csv)：各阶段的实际 TID、转换后边界和状态分解。
- [等待 episode 与事件栈](data/waiting_episodes_with_event_stacks.csv)：4,415 条唯一关联、原始 state/sample ID、调用链、匹配时间差。
- [同签名对照](data/matched_comparisons.csv)：保留缺少正常对照的实例，基线差值与可归因增量分列。
- [任务依赖边界](data/case_task_dependencies.csv)：真实 group_end、task_end、Future Get，Ready 留空。
- [实际 Planning RunOnce 关联](data/downstream_runonce.csv)：690 个实例，发布时刻代理语义保留。
- [科研图集](figures/)：6 张图，各含 300 dpi PNG 和 SVG；此前 F647—F680 连续甘特图位于上级目录 `figures/prediction_F647_F680/`。
- [分析校验](validation_results.json)、[交付校验](artifact_validation.json)、[方法与写作技能记录](report_metadata.json)。

交互报告项目位于 `report_app/`。预览服务：`http://127.0.0.1:4177/`；最终编译文件为 `report_app/dist/index.html`，同目录 JSON sidecar 必须一并保存。浏览器安全策略阻止了本轮页面验视；构建、数据及静态科研图检查分别记录。

复现（在 `D:\data\runtime-analysis-perfetto` 运行，使用带 numpy 的 Python）：

```powershell
python scripts/analyze_realtime_report.py
python scripts/write_realtime_report.py
python scripts/render_realtime_figures.py --report-dir prediction_realtime_report
```

最后一步需要 matplotlib。本轮使用 Orin 上已有的 matplotlib 3.1.2 渲染，未安装新依赖。完整原始数据位于既有远程仿真/Perfetto 目录；本机 `inputs/` 是不纳入 Git 的读取缓存。部署源码、采集文件和仿真状态没有修改。

报告写作遵循用户指定的 [anti-defensive-writing Skill](https://github.com/Adkid-Zephyr/anti-defensive-writing-Skill/blob/main/skills/anti-defensive-writing/SKILL.md)。
