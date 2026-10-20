# Prediction F647—F680 甘特图

共 34 帧。横轴统一以 F647 的 proc_enter 为 0 ms，原始 CLOCK_MONOTONIC_RAW 时间为 1032194602197120 ns。PNG 为 300 dpi，SVG 可无损放大。

- [34 帧总览 PNG](prediction_gantt_F647_F680_overview.png)
- [34 帧总览 SVG](prediction_gantt_F647_F680_overview.svg)

总览：空心边框为 callback 总区间，蓝色为 Evaluator，深蓝色细条为 ThreadPoolRun，紫色细条为 Semantic async draw。条旁数字为 callback 墙钟耗时（ms）。

阶段详图显示 Total、Container、Perception processing、Evaluator、SemanticMap sync、ThreadPoolRun、Predictor、Writer API、Semantic async draw；标签包括实际 TID 和阶段墙钟耗时。阶段为 inclusive，可以嵌套/并行，不能相加为总时间。纵向排列仅为展示分组。小阶段使用真实宽度，不人为放大。

| 帧范围 | PNG | SVG |
|---|---|---|
| F647—F652 | [PNG](prediction_gantt_F647_F652_stages.png) | [SVG](prediction_gantt_F647_F652_stages.svg) |
| F653—F658 | [PNG](prediction_gantt_F653_F658_stages.png) | [SVG](prediction_gantt_F653_F658_stages.svg) |
| F659—F664 | [PNG](prediction_gantt_F659_F664_stages.png) | [SVG](prediction_gantt_F659_F664_stages.svg) |
| F665—F670 | [PNG](prediction_gantt_F665_F670_stages.png) | [SVG](prediction_gantt_F665_F670_stages.svg) |
| F671—F676 | [PNG](prediction_gantt_F671_F676_stages.png) | [SVG](prediction_gantt_F671_F676_stages.svg) |
| F677—F680 | [PNG](prediction_gantt_F677_F680_stages.png) | [SVG](prediction_gantt_F677_F680_stages.svg) |

绘图数据：gantt_primary_stage_data.csv（306 行主要阶段）、gantt_all_stage_data.csv（1,552 行全部可靠阶段）、gantt_frame_data.csv（34 帧）。均保留 frame_uid、源事件 ID、原始时间与相对时间。plot_spec.json 保存图形输入，gantt_validation.json 保存校验和图片散列。

复现：本机执行 scripts/plot_prediction_application_gantt.py prepare；在已有 Matplotlib 环境执行同一脚本 render --output-dir <本目录>。不要求修改原始数据或安装新的采集工具。
