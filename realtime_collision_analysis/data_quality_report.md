# 数据质量报告

## 数量核验

- baseline：6/6
- 100 ms：6/6
- 300 ms：6/6
- 400 ms：5/5
- 总数：23/23

## 关键缺失

- 注入组缺少SCB：202607181958, 202607182007, 202607182017, 202607182021, 202607182026, 202607182029
- CollisionSensor记录：202607191727, 202607191739, 202607201611
- 非碰撞run未保存actor history，统一时钟仅能使用Apollo/Localization epoch。
- 每个run未保存Bridge settings快照，SCB行承担注入参数证据；无SCB run的实际注入状态保持不确定。
- ControlCommand payload未归档，Control事件时间来自继承目标Trace的`/apollo/control`输出。

## 固定条件证据

- 地图：用户固定Town04；碰撞event直接记录`Carla/Maps/Town04`。
- 固定步长：用户固定0.1 s；SCB保存CARLA frame与simulation elapsed差。
- 点云：用户固定130万；run目录只保存处理后`num points before fusing`，缺少原始点云计数配置快照。

## 纳入原则

所有23个run保留在清单和原始散点中。速度越界、SCB缺失、t1前已进入持续减速、时间映射受限均通过质量字段分层，未执行静默删除。
