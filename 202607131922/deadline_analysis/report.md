# 202607131922 车速隐形 Deadline 分析

## 结论

最终碰撞的主因证据指向**感知主链路排队导致障碍物在碰撞前没有成为规划可用目标**。Bridge 直接消费 Control 输出，端到端时延终点按首个 Control 发布计算。

- 最终直道进入名义 50 m 雷达范围的代表帧为 `2026-07-13T19:28:59.647504`：车辆约 `53.91 km/h`，到出生点障碍中心约 `50.13 m`。
- 按规划日志中的最大硬停减速度 `4 m/s²` 和 `PathDecider distance_s=-6` 对应的 6 m 裕量，隐形 deadline 约 `1.075 s`（`10.75` 个 0.1 s bridge 步长）。
- 该帧到首个 Control 发布实际用了 `4.662 s`，超期 `3.588 s`（`35.9` 个 bridge 步长），发布时已经在碰撞后。
- 碰撞记录为 `19:29:02.347041+08:00`，碰撞前一帧速度 `66.47 km/h`。若直接用该速度和 50 m 计算，保守 deadline 只剩 `0.075 s`，不足一个 0.1 s 步长。
- 成功的 55.6 km/h 刹停段 E3：首个可触发停车的帧到 Control 为 `0.659 s`，当时 deadline `0.848 s`，仅余 `0.189 s`（约 `1.9` 个步长），已经很接近边界。

## 四段场景还原

|段|时间与结果|速度/位置证据|规划证据|
|---|---|---|---|
|E1|19:23:03 起步，19:23:17 首次静止|峰值 14.6 km/h，停在 y=124.50|起步前目标已在跟踪；19:23:03.411 出现 blocking-obstacle STOP|
|E2|障碍约移到 y=51.7，19:23:32 刹停|峰值 40.8 km/h，停在 y=74.23|id=2698；19:23:31.425 输出 `stop by 2698`|
|E3|障碍约移到 y=-48.3，19:24:41 刹停|峰值 55.6 km/h；峰值到停下 22.23 m；等效减速度 5.37 m/s²|id=3869；19:24:39.199 首次硬停轨迹，配置上限 4 m/s²|
|E4|19:25:28 开始绕场；19:29:02 碰撞|整圈最高 105.6 km/h；碰撞前 66.47 km/h|碰撞前没有目标障碍的 STOP decision、STOP ST boundary 或 `stop by <id>`|

定位轨迹没有大于 10 m/0.1 s 的跳变；数据支持的是**障碍物继续后移约 100 m**，并不支持“车辆瞬移后置 100 m”。从 E2 障碍约 y=51.7 到 E3 障碍约 y=-48.3，正好约 100 m。

## 关键链路对比

|案例|D1/速度|理论 deadline|实测 sensor→control|余量|
|---|---:|---:|---:|---:|
|E2 成功|46.66 m / 33.7 km/h|3.176 s|1.392 s|+1.784 s|
|E3 成功|44.98 m / 52.5 km/h|0.848 s|0.659 s|+0.189 s|
|E4 碰撞|50.13 m / 53.9 km/h|1.075 s|4.662 s|-3.588 s|

E4 的 50 m 代表帧中，`sensor→fusion` 已占 `4.623 s`；fusion 之后到首个 Control 只有 `39.3 ms`。关键车道目标对应的稍后帧（19:28:59.947）直到 19:29:04.631 才发布融合结果，也在碰撞后。

对 19:28:59.947 关键帧逐阶段检查：传感器到点云预处理入口 64.8 ms；预处理、ROI、地面检测在约 86.7 ms 完成；之后等待约 4.464 s，`lidar_detection` 才进入；检测计算约 121.6 ms；fusion 后到首个 Control 约 40.0 ms。因此主要瓶颈是 **ground-detection 输出到 lidar-detection 入口之间的队列/调度等待**。

## 雷达 50 m 的使用限制

`fusion_inputs` 明确将 `radar_front` 标为 `is_main_sensor=0`，而 `velodyne64` 为 `is_main_sensor=1`；perception 日志反复出现 `Fusion receive from radar_front. Skip because it is not the main sensor.`。这意味着雷达帧本身不能立即触发融合输出，必须等待激光雷达主帧到达。最终接近时激光雷达链路已积压 4–5 s，所以即使雷达在 50 m 内及时看到目标，也不能据现有架构形成及时的规划输入。

当前日志不能精确测出“原始雷达连续 N 帧稳定识别”的 D1：radar message-context 的 `object_count` 为 0，且没有目标级 radar 坐标/ID。因此本报告把 50 m 用作名义上界，把“首个能导致规划停车的融合帧”作为运行时 D1 代理。最终场景碰撞前不存在这样的 planner-visible D1；不能把名义 50 m 直接当成已实现的稳定感知距离。

## 时延统计与日志适用性

- 全时段 `sensor→control`：p50 `1549.8 ms`，p95 `4329.9 ms`，最大 `5818.1 ms`。
- 最终接近窗口 p50 `4071.4 ms`，p95 `5048.7 ms`；远大于约 1 s 的 deadline。
- Control 约 100 Hz；Bridge 和传感器主帧约 10 Hz。以 0.1 s 步长分析时，应至少预留 1 个完整步长作为离散采样裕量。
- 本场景 Bridge 直接消费 Control 指令，Control 首次发布时刻作为软件链路终点。CARLA applied-control 未采集，执行器施加时延仍需后续补充。
- `control.log` 没有 brake/throttle/applied-control 数值；现有分析用定位速度下降和规划硬停轨迹反推制动。下次应同步记录 `/apollo/control` 命令和 CARLA 实际施加的 throttle/brake。
- 采集在碰撞后仍继续到 19:31:52；本次场景因果分析在 19:29:02.347 截止，之后数据不纳入最终碰撞结论。

## 建议的下一轮采集字段

1. 原始 radar 目标级：时间戳、ID、位置、range/range-rate、confidence、连续跟踪帧数。
2. 每个队列的 enqueue/dequeue 时间与队列深度，尤其 ground-detection→lidar-detection。
3. `/apollo/control` 的 brake、throttle、acceleration、命令时间戳，以及 bridge/CARLA applied-control 时间戳。
4. 障碍物 ground-truth actor 位姿历史，不只保留碰撞前 18 s。
5. 把 D1 定义为“同一目标连续 3 帧满足位置/置信度阈值”，D2 以沿参考线的障碍物前缘到自车前缘距离计算，避免中心点距离口径混用。

## 输出文件

- `scenario_summary.csv`：四段场景与速度、停车、时延统计。
- `critical_path_comparison.csv`：E2/E3 成功与 E4 碰撞的 deadline/实测时延对比。
- `deadline_speed_table.csv`：不同车速、两种减速度口径下的 deadline。
- `e2e_latency_by_frame.csv`：每个主激光帧的端到端分阶段时延。
- `vehicle_state_10hz.csv`：定位轨迹、速度。
- `planning_relevant_events.csv`：与停车有关的规划证据。
- `critical_stage_timeline.csv`：E2/E3/E4 代表帧的逐阶段事件时间轴。
