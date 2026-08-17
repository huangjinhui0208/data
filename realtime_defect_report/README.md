# 实时性缺陷报告工作区

本目录是“第二次车速隐形deadline实验”的全新、可复算交付物。原始数据位于 `../第二次实验/`，分析脚本只读原始目录，所有新表、图和文档都写入本目录。

## 研究定位

发现、复现、量化和归因自动驾驶系统暴露出的实时性缺陷，以及实时性缺陷如何传播为车辆安全后果。主证据链为：受控时序故障 → 实测端到端变化 → 距离债务 → 制动位置后移 → 余量压缩 → 临界停车/碰撞 → 缺陷分类。

## 复现

依赖：Python 3、pandas、numpy、matplotlib、PyYAML。底层原始日志解析器及配置已保存在 `scripts/vendor/`。

```bash
cd /Users/huangjinhui/Desktop/萨卡班/data
python3 realtime_defect_report/scripts/analyze_realtime_defects.py
python3 realtime_defect_report/scripts/generate_realtime_defect_report.py
python3 realtime_defect_report/scripts/validate_realtime_defect_report.py
```

底层解析器单元测试：

```bash
PYTHONPATH=realtime_defect_report/scripts/vendor/realtime_collision_analysis/src \
python3 -m pytest -q realtime_defect_report/scripts/vendor/realtime_collision_analysis/tests/test_realtime_collision_core.py
```

分析脚本每次从原始日志重算 12 个 run，并重新生成表格和 11 张指定图。成文脚本只读取新生成的表格，生成主报告、讲稿、单页摘要和证据附录。验证脚本检查样本数、排除逻辑、观测/模型分离、关键数值、图像有效性、报告长度和交付清单。

## 主要入口

- `report/group_meeting_report.md`：5000–8000汉字主报告。
- `report/speech_10_12min.md`：10–12分钟讲稿。
- `report/one_page_summary.md`：单页摘要。
- `report/evidence_appendix.md`：字段定义、逐run证据和反事实边界。
- `tables/run_level_metrics.csv`：12个run的观测主表。
- `tables/realtime_defect_evidence_matrix.csv`：缺陷分类与归因边界。
- `validation/validation_results.md`：自动检查结果。

## 口径边界

- 主 `D_delay` 只用速度对墙钟时间的梯形积分。
- 碰撞run的完整制动距离、观测余量和观测deadline保持NA。
- `counterfactual_model.csv`属于模型/预测，不覆盖观测字段。
- `202607271206`保留时延诊断，但因结局证据冲突不进入11-run主结局分析。
- Guardian不在当前Bridge命令实际执行链中。
