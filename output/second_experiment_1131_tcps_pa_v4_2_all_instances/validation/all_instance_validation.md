# 全实例分析验证

总体状态：**PASS**

- PASS — lidar_instance_count: {'lidar_detection_processing': 352, 'planning_runonce': 353, 'ground_detection_processing': 361}
- PASS — planning_instance_count: {'lidar_detection_processing': 352, 'planning_runonce': 353, 'ground_detection_processing': 361}
- PASS — ground_instance_count: {'lidar_detection_processing': 352, 'planning_runonce': 353, 'ground_detection_processing': 361}
- PASS — selected_extreme_values: {"lidar_detection_processing": 507.314848, "planning_runonce": 473.55728, "ground_detection_processing": 481.353664}
- PASS — one_multi_component_segment: count=1
- PASS — control_population: outputs=3636, traces=354
- PASS — source_fusion_all_instances: expected=353, available=352
- PASS — no_fabricated_physical_episode_distribution: log_all_delayed_commands=false; no record; no event-local payload/apply-to-feedback mapping
- PASS — required_artifacts_exist: /Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/all_instance_lineage_timing.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/all_instance_lineage_distribution.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/component_timing_all_instances.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/component_timing_distribution.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/research_anomaly_segments.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/control_trace_reuse.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/control_physical_episode_audit.csv|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/figures/component_timing_scatter_all_instances.png|/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/report/all_instance_reanalysis_report.md

## 必须保留的限制

- research阈值不是component contract；
- run内实例不是独立实验重复；
- Control→physical严格episode分布不可用。
