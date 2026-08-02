# Apollo + CARLA 实时性碰撞自动分析

本目录仅保存分析程序、派生数据、图表和报告。四组原始数据目录保持只读。

## 复现命令

```powershell
& 'C:\Users\22142\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' `
  'D:\data\realtime_collision_analysis\analyze_realtime_collision.py' `
  --input 'D:\data' `
  --output 'D:\data\realtime_collision_analysis' `
  --config 'D:\data\realtime_collision_analysis\config\analysis_config.yaml'
```

程序会递归读取 baseline、100 ms、300 ms、400 ms 四组数据，生成输入哈希、数据质量报告、逐 run 时间线、物理指标、统计检验、PNG/SVG 图和中文 Markdown/HTML 报告。

主报告：`report/realtime_collision_experiment_report.md`

核心图：`figures/fig08_m_space_vs_actual_latency.png`
