# 第二次实验六层时序—物理安全分析

本目录由 `$autonomous-driving-temporal-safety-analysis` 工作流生成。原始目录 `/Users/huangjinhui/Desktop/萨卡班/data/第二次实验` 只读；所有新结果位于本目录。

## Reproduce

```bash
cd "/Users/huangjinhui/Desktop/萨卡班/data"
python3 report_workspace/scripts/analyze_second_experiment.py
python3 report_workspace/scripts/generate_report.py
python3 report_workspace/scripts/validate_outputs.py
python3 output/second_experiment_six_layer_analysis/scripts/build_six_layer_report.py
python3 /Users/huangjinhui/.codex/skills/autonomous-driving-temporal-safety-analysis/scripts/validate_analysis_outputs.py   --analysis-dir output/second_experiment_six_layer_analysis
```

主报告：`report/six_layer_analysis_report.md`。观测和模型结果分别位于 `tables/run_level_observed.csv` 与 `tables/run_level_model_predicted.csv`。
