# 时延注入实现审计

## 审计结论

- 注入位置：Bridge 的 `EgoVehicle.control_command_updated()` 接收 `/apollo/control` 后，将 ControlCommand 交给 `ControlDelayInjector.submit()`；worker 延后调用 `_apply_control_command()`，最终执行 CARLA `apply_control()`。
- 注入机制：Cyber 回调复制 protobuf 并压入按 `release_monotonic_ns` 排序的最小堆；独立 daemon worker 使用 `Condition.wait(timeout=...)` 等待释放。
- 时间基准：请求等待时间使用 `time.monotonic_ns()`；审计CSV同时记录 wall time、monotonic time、CARLA frame 和 simulation elapsed time。
- 消息行为：触发前沿用原始直接执行路径；触发后所有后续 ControlCommand 进入延迟队列并按释放时间和sequence保持顺序。
- 队列边界：`queue_max_messages` 达到上限时弹出最早待释放消息并记录 `DROPPED_QUEUE_FULL`。当前归档CSV仅记录首次有效制动，无法逐命令排除后续队列丢弃。
- 仿真推进：worker 的等待不阻塞 CARLA synchronous tick 线程；Bridge 主循环按0.1 s wall monotonic节拍推进，CPU/GIL竞争仍可能形成间接负载影响。
- 触发状态：现有源码采用速度锁存ARM与制动阈值触发，未实现障碍物生成后的显式ARM。多数SCB记录显示触发早于目标稳定感知。
- 配置含义：100/300/400 ms为Control链路上的附加墙钟等待请求值；实际闭环解释采用 `actual_e2e_latency_ms`。

## 证据文件

- `D:\data\carla_apollo10.0_bridge\carla_bridge\control_delay_injector.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\actor\ego_vehicle.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\main.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\config\settings.yaml`

## SCB归档覆盖

- 23个run中有 17 个保存SCB CSV。
- 注入组缺少SCB的run：202607181958, 202607182007, 202607182017, 202607182021, 202607182026, 202607182029。

| 名义时延 (ms) | 有SCB实测n | 墙钟实际时延中位数 (ms) | 范围 (ms) |
|---:|---:|---:|---:|
| 0 | 6 | 7.234 | 0.141–12.384 |
| 100 | 5 | 105.508 | 100.078–109.280 |
| 300 | 1 | 309.970 | 309.970–309.970 |
| 400 | 5 | 402.912 | 401.336–420.415 |

## 配置快照限制

工作区 `settings.yaml` 当前写有Town01、0 ms、activation 5 m/s和brake 1%；运行SCB记录显示另一套远端参数，并且CollisionSensor确认碰撞run运行于Town04。每个run未保存完整Bridge settings快照，报告使用SCB行作为实际加载参数证据，地图采用用户固定条件并由碰撞event交叉验证。
