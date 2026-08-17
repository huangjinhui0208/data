# 1131 run 逐帧时延—自车行驶距离性能映射

## 结论

这次补充分析已经把“时延变大”映射成直接可观测的车辆性能量：从每个 LiDAR source 到该 trace 的首个 Control 输出，同时计算软件端到端时延和车速对墙钟时间的梯形积分距离。三个晚期异常帧在软件管线中分别驻留 **783.440 ms、721.321 ms、731.829 ms**，期间自车分别行驶 **9.519 m、8.520 m、8.234 m**。相比异常前 17 个目标帧的描述性 P50 **324.293 ms**，同一 observed 速度轨迹上的参考条件化额外暴露为 **5.246 m、4.408 m、4.273 m**。

但必须限定语义：这些距离是“帧在自动驾驶软件管线内流动期间，自车实际前进的距离暴露”，不是已证明由该帧时延独立造成的可避免距离或碰撞损失。

![1131逐帧时延与行驶距离](/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/figures/framewise_e2e_latency_distance.png)

## 分析人口与端点

- 分析起点是当前证据中“障碍物 11 首个稳定序列帧”的 source epoch，即 `t_sample=1785123121.945625`。这不等于未标定的 `t_observable` 或物理需求起点 `t_demand`。
- 从 `t_sample` 到碰撞前共有 **27** 个 LiDAR source 输入；其中 **22** 个在碰撞前完成到首个关联 Control，**3** 个被碰撞右删失，**2** 个没有 Fusion lineage。
- 软件端点是通过 trace lineage 配对的 `sensor source→Fusion→Prediction→Planning→first Control output`，有端点时为 Grade A。
- 距离严格按 `D=∫v(t)dt_wall` 计算：Localization 速度在端点线性插值，然后按墙钟时间梯形积分。没有使用 CARLA 仿真帧数或仿真时间替代主距离。

## 三个异常帧的性能影响

| 帧 | Fusion trace | source→Control (ms) | 处理期间距离 (m) | P50后额外暴露 (m) | 主导阶段 |
|---:|---|---:|---:|---:|---|
| 17 | `17293896665878496499` | 783.440 | 9.519 | 5.246 | Prediction→Planning |
| 18 | `17293896665878496500` | 721.321 | 8.520 | 4.408 | Source→Fusion |
| 19 | `17293896665878496501` | 731.829 | 8.234 | 4.273 | Source→Fusion |

这张表揭示了两种不同的性能损害路径：帧 F17 主要卡在 `Prediction→Planning`，该段单独持续约 **480.043 ms**，期间自车行驶约 **5.521 m**；F18/F19 主要卡在 `source→Fusion`，该段分别为 **699.268/705.980 ms**，期间自车行驶约 **8.302/8.001 m**。这说明同样的端到端变慢，可以来自不同 Apollo 模块，并在物理上表现为新感知或新规划结果到达前的更长行驶暴露。

## 不能把“首次位置变化”当作逐帧物理端点

自车在所有这些帧到来前已经连续运动，因此“Control 后下一个位置样本变了”只能说明采样时序，不能证明是该帧引起的物理作用。本 run 中 Control 以约 10 ms 重复发布并复用 trace，同时缺少事件局部 Control payload、Bridge receive/apply 和执行器反馈，所以不能为 27 帧各自建立独立 `Control→physical` episode。

因此，本补充分析对“时延影响自动驾驶性能”的可证明结论是：**时延尖峰显著增加了新闭环结果发布前的行驶距离，并降低了碰撞前可用刷新机会**。但它不单独证明“这三帧的额外距离造成了碰撞”。

## 右删失、缺失和不可相加性

- 后续 3 帧的首个 Control 发布在碰撞后，主结果只画到碰撞并标注右删失；碰撞后的 trace 完成时间只作诊断字段，不补成碰撞前完整结果。
- 两个输入没有对应 Fusion lineage，在图中使用红色 `X` 保留，不做均值填补。已知的最后前缀端点是：F20 停在 pointcloud_ground_detection （source后 504.257 ms，期间行驶 5.758 m）；F21 停在 pointcloud_map_based_roi （source后 196.869 ms，期间行驶 2.135 m）。它们只能说明当前证据链在哪里停止，不足以断言是 drop、覆盖还是调度根因。
- 同时在管线中的帧大量重叠。20 个目标帧逐帧距离之和是 **115.512 m**，但它们的时间区间去重后仅覆盖 **38.641 m**。前者重复计数，禁止当作“总损失距离”。

## 权威方法与 Apollo 10 实现核对

- Apollo 10 的 [`Header`](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/common_msgs/basic_msgs/header.proto) 明确区分消息发布时间与 `lidar_timestamp`，因此本分析使用 LiDAR source/trace anchor 作为帧起点，不用 Fusion 或 Planning 发布时间代替传感器采样时间。
- Apollo 10 源码显示 `lidar_timestamp` 沿 [Prediction](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/prediction/prediction_component.cc#L246-L285)、[Planning](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/planning/planning_component/planning_base.cc#L98-L108) 到 [Control](https://github.com/ApolloAuto/apollo/blob/v10.0.0/modules/control/control_component/control_component.cc#L452-L495) 传播；这是逐帧 source→first Control lineage 的实现依据。Control 是 timer component，反复读取 latest trajectory，所以同一 trace 的后续发布是 reuse，不是新帧或独立物理 episode。
- [ECRTS 2023](https://doi.org/10.4230/LIPIcs.ECRTS.2023.10) 要求 effect/actuation 必须确实“based on”对应样本；这支持不把运动车辆的下一个位置样本冒充为逐帧物理效果。
- [Yi 2021](https://arxiv.org/abs/2106.04508) 将 sensor→actuator 时限与处理期间可行驶距离联系起来；[Koopman 2019](https://arxiv.org/abs/1911.01207) 则明确把 response delay 纳入安全距离。本报告因此将时延与墙钟积分距离并列，但没有在缺少合格 dynamic contract 和反事实 replay 时把它宣布为 deadline debt 或碰撞因果。

## 数据产物

- 逐帧主表：`/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/framewise_sensor_to_control_performance.csv`
- 逐阶段长表：`/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/framewise_stage_performance.csv`
- 摘要表：`/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/tables/framewise_performance_summary.csv`
- 图：`/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/figures/framewise_e2e_latency_distance.png` / `/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/figures/framewise_e2e_latency_distance.svg`
- 复现脚本：`/Users/huangjinhui/Desktop/萨卡班/data/output/second_experiment_1131_tcps_pa_v4_2_all_instances/scripts/analyze_1131_framewise_performance_v4_2.py`

## 下一证据门槛

若要把软件端到端暴露提升为逐动作的物理响应时间或可归因安全损失，需要在同一时钟基础上保存 Control payload/sequence、Bridge receive/apply、制动或加速度反馈，再按“命令语义发生独立变化”切分 action episode，而不是按每条 Control 发布消息切分。
