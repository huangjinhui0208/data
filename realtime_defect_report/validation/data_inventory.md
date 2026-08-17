# 实时性缺陷报告数据盘点

- 预期 run：12；发现：12；缺失：无；额外：无。
- 文件分类计数：`{"trace_events": 144, "trace_message_context": 144, "other": 30, "existing_figure": 24, "collection_window": 12, "localization_log": 12, "perception_log": 12, "planning_log": 12, "prediction_log": 12, "control_delay_evidence": 12, "trace_fusion_inputs": 12, "trace_anchor": 12, "existing_analysis_script": 9, "carla_collision_event": 4, "carla_actor_history": 2, "documentation": 1}`。
- 原始目录只读；新生结果均位于 `realtime_defect_report/`。
- 详细文件清单见 `../extracted/file_inventory.csv`，模式清单见 `../extracted/schema_inventory.json`。
- 优先级：原始日志/Trace/SCB/CollisionSensor/actor history > 本目录可复算脚本 > 旧CSV/JSON > 旧报告文字。
- 点云数量560000、队列长度1等属于实验设定，不作为逐run实测常量。该项来源于实验设定，当前归档中缺少独立配置文件快照。
