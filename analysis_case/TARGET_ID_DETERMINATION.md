# target_id 自动确定方法

## 输入

- `carla_collision_events_*.csv`：提供碰撞时刻、`other_actor_id` 和 history 文件名。
- `carla_collision_actor_history_*.csv`：提供碰撞对象在碰撞前的 CARLA 轨迹。
- perception 日志中的 `[FUSION_OBS]`：提供 Apollo `id`、`obs_time`、位置、速度、航向和类型。
- Planning 日志：只作为辅助加分，不作为目标身份的必要条件。

## 判定流程

1. 只从 collision events 读取第一条碰撞事件，history 第一帧不能作为碰撞时刻。
2. 使用 `other_actor_id` 选择 history 中 `role=other` 的轨迹。
3. 将 CARLA 轨迹转换为 Apollo 坐标：

   ```text
   x = x_carla
   y = -y_carla
   vx = vx_carla
   vy = -vy_carla
   heading = -yaw_carla * pi / 180
   ```

4. 对每条 Apollo `[FUSION_OBS]`，按其 `obs_time` 在线性插值后的 CARLA 轨迹上取得同一时刻真值。
5. 按 Apollo ID 聚合多帧位置、速度、航向和类型误差。静止/低速目标不使用航向分数。
6. 先应用位置硬门限，再计算身份分数。Planning 默认只占 5%，不能让位置不匹配的 ID 通过。
7. 主 ID 至少需要满足配置的多帧数；随后允许在短时间、短位置跳变条件下连接一次或多次 ID 切换。
8. 输出首次检测目标的 ID 为 `target_id`，同时输出完整的 `physical_target_id_chain`。

本场景输出为：

```text
CARLA collision actor_id = 214
target_id = 269
physical_target_id_chain = [269, 278]
```

## 只确定 target_id

```powershell
python D:\data\anlysis_case\determine_target_id.py `
  --case-dir D:\data\202607102138
```

该脚本只读取数据并向标准输出打印 JSON，不创建结果目录。

## 运行完整筛选

```powershell
python D:\data\anlysis_case\collision_case_classifier.py `
  --case-dir D:\data\202607102138 `
  --out-dir D:\data\202607102138\classifier_output `
  --config D:\data\anlysis_case\collision_classifier_config.yaml
```

主要调节项位于 `collision_classifier_config.yaml` 的 `carla_target_match`。默认分析窗口为碰撞前 15 秒。
