# Collision Case Classifier

这个脚本用于快速筛选 Apollo/Carla 碰撞案例。它只消费已经存在的日志、CSV、JSON、JSONL，不解析 cyber record，不依赖 Apollo 运行环境，也不使用 control / guardian 参与最终判定。

## 支持的新增日志

脚本会优先解析 Apollo 日志中的这些标签：

- `[FUSION_OBS_FRAME]`
- `[FUSION_OBS]`
- `[PLANNING_EGO_STATE]`
- `[LOCALIZATION_POSE]`

同时兼容已有解析结果，例如 `fusion_obs_aligned.csv`、`planning_stop_obs_st_bounds*.csv`、`planning_blocking_stop_bounds*.csv`、`planning_fallback*.csv`、Carla collision CSV/JSON/JSONL。

## 运行

```bash
python3 tools/collision_case_classifier.py \
  --case-dir /path/to/case \
  --out-dir /path/to/case/classifier_output \
  --verbose
```

可选参数：

```bash
python3 tools/collision_case_classifier.py \
  --case-dir /path/to/case \
  --out-dir /path/to/output \
  --config tools/collision_classifier_config.yaml \
  --target-id 320 \
  --collision-time 1782823249.9375129 \
  --max-window-sec 5.0 \
  --verbose
```

`--max-window-sec` 会覆盖 `analysis.pre_collision_window_sec`。

## 输出

输出目录只生成 JSON：

- `classification_result.json`
- `schema_inventory.json`
- `target_resolution_debug.json`

旧版本的 `evidence_report.md`、`module_verdicts.csv`、`target_timeline.csv` 不再生成；脚本运行时会清理这些已知旧输出文件。

## target_id 自动锁定

自动目标锁定优先使用 planning 认为影响自车行驶的强证据：

- `stop_id`
- `Blocking obstacle ID[...]`
- `print_STOP...obs_st_bounds`
- planning obstacle / perception id
- STOP / FOLLOW / YIELD / OVERTAKE 纵向约束

然后结合 fusion 障碍物和 ego state 计算相对位置：

- `rel_forward`
- `rel_left`
- `rel_distance`
- `closing_speed`
- `ttc`

ego state 优先使用 `[LOCALIZATION_POSE]`，其次使用 `[PLANNING_EGO_STATE]`。碰撞后 0.1 到 2.0 秒内稳定出现的同 ID 目标，或通过位置、速度、朝向连续性匹配到的同一物理目标，会作为后验校验。

`length` / `width` / `height` 只写入 debug，不作为同一障碍物主匹配条件。

## 分类

最终分类只会是：

- `PERCEPTION_ABNORMAL`
- `PREDICTION_ABNORMAL`
- `PLANNING_ABNORMAL`
- `FUNCTION_NORMAL_BUT_TOO_LATE`
- `PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING`
- `UNKNOWN_OR_DATA_INSUFFICIENT`

如果没有可靠碰撞时间或 target_id，输出 `UNKNOWN_OR_DATA_INSUFFICIENT`，并在 JSON 中说明缺失原因。
