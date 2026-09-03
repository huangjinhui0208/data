# 可复用脚本索引入口

完整的扫描说明、筛选标准、脚本用途、参数、依赖、示例和排除原因见 [`README.md`](README.md)。

快捷入口：

- [Perception逐帧分析](perception/analyze_perception_realtime.py)：默认保存到 `<run-dir>/打点逐帧数据统计/perception数据统计`
- [Perception关键路径甘特图](perception/plot_perception_critical_path_gantt.py)
- [异常帧perf sched窗口提取](scheduler/extract_perf_sched_frame_windows.py)
- [CenterPoint Infer逐帧调度状态分解](scheduler/analyze_perf_sched_infer_frames.py)
- [P4逐帧sched与CPU stack审计数据生成](perf/generate_p4_perf_analysis.py)
- [P4 CPU stack完整调用链互斥分类](perf/classify_p4_cpu_stack_samples.py)
- [碰撞案例分类器](collision/collision_case_classifier.py)
- [实时碰撞分析核心库](collision/realtime_collision_core.py)
- [Apollo/Orin采集工具](collection/apollo_perf_sched_collector.py)
- [Apollo/Orin perf sched与CPU stack联合采集](collection/apollo_perf_sched_cpu_stack_collector.py)
- [DOCX结构检查](documents/inspect_docx.py)
- [来源与SHA-256清单](SOURCE_MANIFEST.csv)
- [依赖说明](REQUIREMENTS.md)
