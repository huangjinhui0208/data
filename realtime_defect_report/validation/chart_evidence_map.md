# 图表—证据映射

| 图 | 主数据 | 支持的结论 | 不支持的结论 |
|---|---|---|---|
| realtime_fault_propagation_chain | group_summary.csv | 组级故障传播顺序 | 单一模块根因 |
| e2e_response_by_run | run_level_metrics.csv | 逐run响应差异 | 通用硬阈值 |
| latency_amplification | group_summary.csv + SCB列 | 闭环增量大于名义注入 | 剩余149.5 ms全部属于某一模块 |
| distance_debt_by_run | run_level_metrics.csv | 墙钟积分距离债务 | CARLA sim距离 |
| braking_position_by_run | run_level_metrics.csv | t2时可用净距 | 完整制动能力相同 |
| realtime_safety_cliff | run_level_metrics.csv | 当前样本结局转换区 | 普适700 ms阈值 |
| case_1131_causal_chain | collision_case_comparison.csv | 实时性主导候选 | 纯时延唯一致撞 |
| case_1131_fusion_timeline | Fusion目标序列 | 输出连续性/新鲜度异常 | 目标从未识别 |
| case_1643_causal_chain | collision_case_comparison.csv | 多因素碰撞 | 与1131相同Fusion gap机制 |
| case_1643_data_freshness | Fusion目标序列 | 持续输出下源数据老化 | 单帧处理超过500 ms |
| outcome_timeline | run_level_metrics.csv | t1→t2→端点顺序 | 碰撞run完整停车距离 |
