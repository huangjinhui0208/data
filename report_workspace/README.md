# 第二次实验分析工作区

本目录是对 `/Users/huangjinhui/Desktop/萨卡班/data/第二次实验/` 的只读分析产物。原始 run、日志和旧报告未被修改。

## 主交付件

- `report/group_meeting_report.md`：5010 个汉字的组会技术报告。
- `tables/run_level_metrics.csv`：12 个 run 的观测主指标，含数据源和缺失值。
- `tables/group_summary.csv`：主分析 11 个 run 的组统计。
- `tables/stage_latency_summary.csv`：Trace 阶段耗时和数据连续性统计。
- `tables/collision_case_comparison.csv`：`1131`/`1643` 与同设置安全 run `1211` 对照。
- `tables/evidence_matrix.csv`：A 配置、B 直接观测、C 模型/反事实的证据边界。
- `tables/target_identity_audit.csv`：Planning/Fusion/CARLA actor 身份对齐。
- `tables/counterfactual_model.csv`：单独保存的 C 类反事实结果。
- `figures/*.png`：7 张组级图与 8 张碰撞案例图。
- `validation/`：数据盘点、差异、排除样本、时钟对齐和最终验证记录。

DOCX 未生成：当前机器上没有 `pandoc`。Markdown 是主交付件；安装 pandoc 后重运行 `generate_report.py` 会自动生成 `report/group_meeting_report.docx`。

## 环境和依赖

- Python 3.9+
- `numpy`、`pandas`、`PyYAML`、`matplotlib`
- 原始解析器从当前 Git `HEAD` 机械提取至 `scripts/vendor/realtime_collision_analysis/`。仅将报告渲染所需的 `markdown` 和模型所需的 `scipy` 改为可选导入，未修改日志解析、t1/t2 检测或墙钟积分逻辑。
- 当前重算环境：Python 3.9.6，NumPy 2.0.2，pandas 2.3.1，PyYAML 6.0.3，matplotlib 3.9.4。

## 一键复现

```bash
cd "/Users/huangjinhui/Desktop/萨卡班/data"
python3 report_workspace/scripts/analyze_second_experiment.py
python3 report_workspace/scripts/generate_report.py
python3 report_workspace/scripts/validate_outputs.py
```

`analyze_second_experiment.py` 每次都从 `第二次实验/` 原始文件重新解析，不依赖旧报告表格。`generate_report.py` 只读取新生 CSV 生成报告。`validate_outputs.py` 独立检查必需文件、公式、NA 规则、图片和 Markdown 链接。

## 统一定义

- `t1`：目标首次连续 3 帧稳定 Fusion 序列中第一帧的源时刻。
- `t2`：首个满足连续 2 个 Localization 间隔减速度≥0.5 m/s²，且后续 0.3 s 速度下降≥0.3 m/s 的区间终点。
- `T_e2e=t2-t1`。
- `D1_clear=D1_center-5.3074 m`。
- `D_delay`：车速对 t1→t2 墙钟时间的梯形积分，不使用 CARLA 帧数、sim time 或 Localization 位移代替。
- `D2=D1-D_delay`。
- 完整停车 run 的主 `D_brake_data` 是 t2 到后续最小速度样本的 Localization 位移；近停、严格持续停车与速度积分距离另存为诊断列。

## 时钟、缺失值和排除规则

- 主结果全部使用 Apollo/Localization 墙钟 epoch 时间线。Trace 阶段时间使用 monotonic clock，由 trace anchor 对齐目标源时刻。
- 主分析中没有 actor history 的 9 个 run（另有被排除的 `1206`）不能独立拟合 CARLA sim time→wall time；这不影响墙钟主指标，但不用于计算 realtime factor。
- 碰撞 run 的完整制动距离、观测 0 m/6 m 余量和观测 deadline 必须为 NA，不以模型补齐。
- `202607271206` 保留 t1/t2 和距离诊断，但因“无碰撞事件”与“固定目标几何穿透”冲突，不进入主组统计或结局对照。
