# 结果一致性检查

总体结果：**PASS**

- PASS：`run_count_matches_23`
- PASS：`group_counts_match`
- PASS：`metrics_row_count_matches_runs`
- PASS：`collision_count_matches_event_files`
- PASS：`rt_only_all_conditions_satisfied`
- PASS：`png_and_svg_present`
- PASS：`all_15_required_figures_present`
- PASS：`all_runs_classified_once`
- PASS：`nominal_and_actual_delay_fields_separate`
- PASS：`baseline_model_uses_all_six_runs`
- PASS：`baseline_braking_distance_matches_authoritative_handoff`
- PASS：`analyzed_runs_have_finite_space_margin`
- PASS：`analyzed_runs_have_both_margin_definitions`
- PASS：`dual_margins_differ_by_six_meters`
- PASS：`target_collision_counterfactual_avoids_contact`
- PASS：`target_collision_planning_fallback_evidence_present`
- PASS：`target_collisions_classified_as_realtime_induced`
- PASS：`invalid_attribution_excluded_from_latency`
- PASS：`collision_runs_do_not_emit_strict_stop_event`
- PASS：`report_has_no_nan_literal`
- PASS：`report_avoids_forbidden_contrast_phrase`
- PASS：`input_hashes_complete`

- CollisionSensor文件计数：3
- run_metrics碰撞计数：3
- PNG/SVG数量：17/17

## RT_ONLY_COLLISION必要条件复核

```json
{
  "202607191727": {
    "collision": true,
    "injected_delay_group": true,
    "all_baseline_runs_safe": true,
    "all_modules_pass": true
  },
  "202607201611": {
    "collision": true,
    "injected_delay_group": true,
    "all_baseline_runs_safe": true,
    "all_modules_pass": true
  }
}
```
