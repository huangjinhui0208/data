# Collision Case Classifier

## Evidence-chain mode for added Apollo logs

The classifier now treats the case as a reviewable evidence chain:

```text
collision truth -> physical target chain -> perception -> prediction -> planning
```

In addition to the existing fusion / ego tags, it recognizes the minimal added Apollo logs:

- `[PREDICTION_INPUT_OBS]`
- `[PREDICTION_OUTPUT_OBS]`
- `[PLANNING_INPUT_PRED_FRAME]`
- `[PLANNING_INPUT_OBS]`
- `[PLANNING_DECISION]`
- `[PLANNING_ST_BOUNDARY]`
- `[PLANNING_OUTPUT]`

Prediction is not marked PASS only because a target id appears. A dynamic
target must show a non-empty prediction trajectory. A static target may legally
use an empty trajectory only when `pred_is_static=1` and
`pred_has_is_static!=0`. Both cases still require fresh target-chain output and
downstream planning consumption. Missing trajectory/static detail is
`UNKNOWN_OR_DATA_INSUFFICIENT`.

Planning is not marked PASS only because stop/blocking evidence exists. It must show the target-chain entering planning, a useful decision or ST boundary, a valid non-empty planning output trajectory, and timing that leaves enough reaction time. Missing decision/ST/output detail is `UNKNOWN_OR_DATA_INSUFFICIENT`.

## Target Id Resolution

When `carla_collision_actor_history_*.csv` is present, target selection is
CARLA-ground-truth-first:

```text
collision event other_actor_id
  -> select role=other history
  -> CARLA-to-Apollo coordinate conversion
  -> interpolate history at each FUSION_OBS obs_time
  -> hard multi-frame position gates
  -> weighted position/velocity/heading/type score
  -> link short Apollo id-switch segments
```

The coordinate transform is `x=x_carla`, `y=-y_carla`, `vx=vx_carla`,
`vy=-vy_carla`, and `heading=-yaw*pi/180`. Heading is ignored while the
physical target or Apollo observation is nearly stationary. Planning has only
the configurable `planning_aux_weight` (default 0.05) and cannot bypass the
hard geometry gates.

If CARLA history is missing or no candidate passes its confidence/margin
requirements, the previous fusion/planning heuristic remains as a fallback.
The full evidence and candidate ranking are written to
`target_resolution_debug.json.carla_history_match`.

To determine only target_id without writing classifier outputs:

```powershell
python D:\data\anlysis_case\determine_target_id.py `
  --case-dir D:\data\202607102138
```

The result contains both the first Apollo `target_id` and
`physical_target_id_chain`, for example `269` and `[269, 278]`.

## Perception Pass Rules

Perception correctness is evaluated only in `w1 = [t2 - 5s, t2]`. Post-collision fusion output may help select the physical collision target, but it does not prove pre-collision perception correctness.

`CONTINUOUS_SINGLE_ID` and `LATE_STABLE_DETECTION` are valid on both straight and turning scenes. In both cases, once the target first appears at `t1`, the same obstacle id must be continuously output until `t2` with no gap larger than `max_perception_gap_sec`.

`REACQUIRED_WITH_ID_SWITCH_NON_PERCEPTION_CAUSE` is only enabled when `turn_context.is_turn = true`. Turn context is detected from ego heading change in w1 using `classification.turn_context_heading_change_rad` (default `0.25 rad`). On straight scenes, id switch reacquisition is disabled; an interruption from first detection to collision is treated as `TARGET_CHAIN_BROKEN_UNEXPLAINED` or `TARGET_MISSING_WHEN_RELEVANT`.

When turn id-switch reacquisition is enabled, same-target scoring reduces the impact of heading and adds relative-motion continuity, because heading is less reliable during turning or very low-speed target motion.

这个脚本用于快速筛选 Apollo/Carla 碰撞案例。它只消费已经存在的日志、CSV、JSON、JSONL，不解析 cyber record，不依赖 Apollo 运行环境，也不使用 control / guardian 参与最终判定。

## 支持的日志标签

脚本会优先解析 Apollo 日志中的这些标签：

- `[FUSION_OBS_FRAME]`
- `[FUSION_OBS]`
- `[PLANNING_EGO_STATE]`
- `[LOCALIZATION_POSE]`

同时兼容已有解析结果，例如 `fusion_obs_aligned.csv`、planning stop/blocking/ST boundary 表、prediction 表、Carla collision CSV/JSON/JSONL。

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

`--max-window-sec` 只覆盖 `analysis.pre_collision_window_sec`。默认 w1 固定为碰撞前 5 秒：

```text
w1 = [t2 - 5s, t2]
```

`classification_result.json` 中的 `w1.start` 始终等于 `t2 - analysis.pre_collision_window_sec`，不会因为 target id 首次出现较晚而缩短。

解析日志时会先按相关时间窗口过滤：`[t2 - max(pre_collision_window_sec, planning_target_window_sec), t2 + post_collision_window_sec]`，并为 ego state 匹配额外保留 `analysis.max_time_match_diff_sec` 的边界余量。`schema_inventory.json` 里的 parsed counts 因此表示窗口内解析到的记录数。

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

ego state 优先使用 `[LOCALIZATION_POSE]`，其次使用 `[PLANNING_EGO_STATE]`。

## Perception 判定

Perception 判断对象是 planning 锁定的关键物理目标链 `physical_target_chain`，不是单一 obstacle id。也就是说，旧 id 中断后，如果新 id 能通过 position、velocity、theta、relative motion 证明是同一物理目标，并且稳定输出到 t2，不会仅因为 id switch 判为 perception abnormal。

如果 target_id 在 w1 内没有 fusion 行，脚本不会仅凭相对风险或 post-collision 单点观测轻易替换成其他 candidate。只有在原 target_id 没有强 planning 证据、候选 id 有强 planning 证据且稳定到 t2 时，才允许作为初始物理目标候选；否则按 `TARGET_NOT_IN_FUSION` 或 `TARGET_MISSING_WHEN_RELEVANT` 输出。

Perception 功能正常包含三类：

- `CONTINUOUS_SINGLE_ID`
- `LATE_STABLE_DETECTION`
- `REACQUIRED_WITH_ID_SWITCH_NON_PERCEPTION_CAUSE`

Perception abnormal 包含：

- `TARGET_NOT_IN_FUSION`
- `TARGET_CHAIN_BROKEN_UNEXPLAINED`
- `TARGET_MISSING_WHEN_RELEVANT`
- `FUSION_TARGET_TYPE_UNSTABLE`
- `FUSION_TARGET_POSITION_JUMP`

`length` / `width` / `height` 只保留到 debug 输出中，不作为同一障碍物或感知异常的硬判据。

Post-collision evidence 需要目标稳定出现，单帧出现不再计为 `stable_same_id`。

`analysis.planning_target_window_sec` 用于 target resolver 的 planning 候选窗口和 planning score；默认 w1 仍由 `analysis.pre_collision_window_sec` 控制。

Prediction / planning 检查会使用 perception 输出的 `id_chain`。例如 `id_chain=["320","347"]` 时，prediction 或 planning 命中 `320` 或 `347` 任一链内 id 都会被计入目标证据。

Prediction 的目标证据 reason code 使用 chain 语义：`PREDICTION_TARGET_CHAIN_PRESENT`、`PREDICTION_TARGET_CHAIN_MISSING`、`PREDICTION_TARGET_CHAIN_APPEARED_TOO_LATE`。

Fusion 类型会归一化 Apollo numeric type：`0=UNKNOWN`、`1=UNKNOWN_MOVABLE`、`2=UNKNOWN_UNMOVABLE`、`3=PEDESTRIAN`、`4=BICYCLE`、`5=VEHICLE`。

## 输出

输出目录只生成 JSON：

- `classification_result.json`
- `schema_inventory.json`
- `target_resolution_debug.json`

`classification_result.json` 的 `perception` 字段包含 physical target chain 详情：`id_chain`、`id_switch`、`segments`、`switches`、`first_seen_time`、`last_seen_time`、`max_chain_gap_sec`、`vehicle_type_ratio`、`stable_to_t2`。

`target_timeline` 会包含 chain 内所有 perception id 的观测行。例如 `id_chain=["320","347"]` 时，timeline 会同时包含 `320` 和 `347` 两段 perception 行。

旧版本的 `evidence_report.md`、`module_verdicts.csv`、`target_timeline.csv` 不再生成；脚本运行时会清理这些已知旧输出文件。

## 最终分类

最终分类只会是：

- `PERCEPTION_ABNORMAL`
- `PREDICTION_ABNORMAL`
- `PLANNING_ABNORMAL`
- `FUNCTION_NORMAL_BUT_TOO_LATE`
- `PLANNING_FUNCTION_NORMAL_COLLISION_AFTER_PLANNING`
- `UNKNOWN_OR_DATA_INSUFFICIENT`

如果 physical target chain 的 perception verdict 为 `FAIL`，最终分类为 `PERCEPTION_ABNORMAL`。如果 perception verdict 为 `PASS`，继续判断 prediction 和 planning。

如果没有可靠碰撞时间或 target_id，输出 `UNKNOWN_OR_DATA_INSUFFICIENT`，并在 JSON 中说明缺失原因。
