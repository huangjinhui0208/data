# 自动验证结果

- 通过：41/41
- 失败：0/41

| 检查项 | 状态 | 详情 |
|---|---|---|
| 报告文件 group_meeting_report.md | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/report/group_meeting_report.md |
| 报告文件 speech_10_12min.md | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/report/speech_10_12min.md |
| 报告文件 one_page_summary.md | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/report/one_page_summary.md |
| 报告文件 evidence_appendix.md | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/report/evidence_appendix.md |
| 表格文件 run_inventory.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/run_inventory.csv |
| 表格文件 run_level_metrics.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/run_level_metrics.csv |
| 表格文件 group_summary.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/group_summary.csv |
| 表格文件 stage_latency_summary.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/stage_latency_summary.csv |
| 表格文件 collision_case_comparison.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/collision_case_comparison.csv |
| 表格文件 realtime_defect_evidence_matrix.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/realtime_defect_evidence_matrix.csv |
| 表格文件 target_identity_audit.csv | PASS | /Users/huangjinhui/Desktop/萨卡班/data/realtime_defect_report/tables/target_identity_audit.csv |
| 图像文件 realtime_fault_propagation_chain.png | PASS | 2358x585 |
| 图像文件 e2e_response_by_run.png | PASS | 1996x974 |
| 图像文件 latency_amplification.png | PASS | 1473x940 |
| 图像文件 distance_debt_by_run.png | PASS | 1997x975 |
| 图像文件 braking_position_by_run.png | PASS | 1996x975 |
| 图像文件 realtime_safety_cliff.png | PASS | 1626x1045 |
| 图像文件 case_1131_causal_chain.png | PASS | 2358x585 |
| 图像文件 case_1131_fusion_timeline.png | PASS | 1671x1171 |
| 图像文件 case_1643_causal_chain.png | PASS | 2358x585 |
| 图像文件 case_1643_data_freshness.png | PASS | 1692x1171 |
| 图像文件 outcome_timeline.png | PASS | 1961x1153 |
| 发现12个预期run | PASS | metrics=12, inventory=12 |
| 11个run进入主分析 | PASS | 11 |
| 1206按证据冲突排除 | PASS | uncertain_geometry_event_conflict |
| 碰撞run完整制动与余量保持NA | PASS | collision rows=2 |
| 观测与模型分表保存 | PASS | run_level=observed, counterfactual=model |
| baseline响应中位数 | PASS | 300.358057 |
| 300ms响应中位数 | PASS | 749.901414 |
| 距离债务中位差 | PASS | expected 7.431230 m |
| 制动起点净距中位差 | PASS | expected 7.142753 m |
| baseline最终净距均为正 | PASS | 7/7 |
| 12组首个稳定sensor→Fusion均低于500ms | PASS | range=208.315–319.285 ms |
| 1131连续性/新鲜度异常可复现 | PASS | gap=507.439, lifecycle=705.892 |
| 两套队列字段分离 | PASS | lidar trace queue vs SCB command queue |
| 碰撞/异常分类齐全 | PASS | {'202607271131': 'RT_DOMINATED_COLLISION', '202607271202': 'NONCOLLISION_REALTIME_MARGIN_EXHAUSTION', '202607271206': 'INDETERMINATE', '202607271211': 'NONCOLLISION_REALTIME_MARGIN_EXHAUSTION', '202607271643': 'MULTI_FACTOR_COLLISION'} |
| 主报告5000–8000汉字 | PASS | 5091 |
| 报告明确否定通用700ms阈值 | PASS | boundary statement present |
| 报告使用观测/模型边界 | PASS | boundary statement present |
| Markdown本地链接均可解析 | PASS | none |
| 原始目录文件盘点仍为454 | PASS | 454 |
