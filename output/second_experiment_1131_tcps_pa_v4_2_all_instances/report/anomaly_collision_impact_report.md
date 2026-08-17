# 1131 run 三个晚期异常的 Apollo 系统影响与碰撞反事实

## 结论

这三个异常会在 Apollo 中造成**后期闭环刷新完整性退化**，但现有证据不支持把它们认定为首次制动过晚或最终碰撞的必要原因。

- 三者最早在 `t_phys` 后 **1189.275 ms** 才开始，故删除它们不会改变本run已经发生的首次物理制动端点。
- 异常前约 **77.510 ms**，Planning 已输出 `stop by 11`、`trajectory_type=3`、`max_abs_decel=4` 的非空降级轨迹。异常中的 Planning 输出仍是相同停车语义。
- 三异常覆盖期间，CARLA actor速度从 **12.544 m/s** 降到 **10.000 m/s**，按墙钟梯形积分前进 **5.816 m**。这证明制动没有在该段消失；该距离不能直接称为异常造成的安全损失。
- “没有这三个异常是否安全”的直接数据答案是 **NOT_IDENTIFIABLE**：本run没有无异常重放，Control payload与逐命令Bridge apply也缺失。
- 模型诊断给出更强的方向性答案：到最早异常开始时，车辆仅余 **6.729 m** 净距、速度 **12.544 m/s**，瞬时停车需约 **11.692 m/s²**；三套制动假设下的0 m接触余量均为负。因此只消除这三个晚期尖峰，**碰撞仍很可能发生**。

## 三个异常如何映射到 Apollo

Apollo 10 的 LiDAR DAG 显示，Ground Detection 的输出 `/perception/lidar/pointcloud_ground_detection` 是 Lidar Detection 的输入，检测结果再进入跟踪/融合链；Planning 则由 Prediction 触发并发布供 Control 执行的轨迹。参见 [Apollo LiDAR Fusion DAG](https://apollo.baidu.com/docs/apollo/10.x/lidar__fusion__output_8dag_source.html)、[Planning Component README](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_component/README_cn.md) 与 [Planning 架构输出说明](https://github.com/ApolloAuto/apollo/blob/master/docs/07_Prediction/Class_Architecture_Planning.md?plain=1)。

| 异常 | 本地实例 | 映射到Apollo后的直接影响 | 对碰撞的证据结论 |
|---|---:|---|---|
| Ground Detection | 481.354 ms | 阻塞同trace的Lidar Detection；对应下一条`source→Fusion=705.980 ms`、`source→Control=731.829 ms`，整轮感知到控制刷新后移。 | 支持晚期刷新退化；不支持它改变首次`t_phys`。 |
| Lidar Detection | 507.315 ms | 目标检测结果晚到，使对应`source→Fusion=699.268 ms`，并在相邻Fusion目标输出之间形成507.439 ms空洞。 | Prediction/Planning看到的目标状态变旧，但此前制动轨迹已经存在。 |
| Planning RunOnce | 473.557 ms | `Prediction→Planning=480.043 ms`，新轨迹到04.423057才发布，首个Control到04.429530。 | 延迟轨迹刷新；但异常前后都输出`stop by 11/max_abs_decel=4`，未观测到制动被取消。 |

这三者是相邻trace上的并发异常，不是同一帧：Planning尖峰属于Fusion trace `...6499`，Lidar尖峰属于父trace `...9851`并进入Fusion `...6500`，Ground尖峰属于父trace `...9852`并进入Fusion `...6501`。

## 物理状态与碰撞影响

### 直接观测（data/observed）

1. `t_phys=1785123122.745261`；最早异常开始 `1785123123.934536`；碰撞 `1785123124.852247`，事件先后关系明确。
2. 异常开始时的净距按本报告既有的目标11纵向端点和几何扣除口径计算为 **6.729 m**。它不是CARLA actor中心欧氏距离，避免与主`D1_clear/D2_clear`口径混列。
3. 上一帧Planning在异常前已经产生停车轨迹；三异常期间车辆速度持续下降。
4. 碰撞及 **7.988 m/s** 碰撞速度是直接观测；但“三异常造成多少物理安全损失”不可从单run分离，因为不存在同状态无异常轨迹。

所以，数据能够证明的是“约0.5 s的新鲜信息/轨迹刷新机会被推迟”，而不是“这0.5 s全部转化为额外制动起始延迟”或“造成了某个可直接相减的碰撞距离”。

### 模型诊断（model/predicted，不能覆盖实际结果）

| 状态/反事实 | central | conservative | 解释 |
|---|---:|---:|---|
| `t_phys`时0 m接触停车余量 | -5.195 m | -13.810 m | 均为负；模型认为三异常开始前已越界。 |
| 最早异常开始时0 m接触停车余量 | -8.691 m | -13.116 m | 均为负；规划降级4 m/s²余量为-12.939 m。 |
| 删除全部初始响应距离的0 m余量 | 12.919 m | 5.647 m | 均为正，但这是不同反事实。 |
| 删除全部初始响应距离的6 m余量 | 6.919 m | -0.353 m | central为正、conservative略负，不稳健。 |

模型的baseline与1131分离，但锁时无效且摩擦、坡度、曲率、载荷和制动建立时间域未验证，因此只能写作 `MODEL_SUPPORTED_ONLY`。它支持“仅删除晚期三异常不足以避免碰撞”的方向性判断，不能冒充无异常重放结果。

## 正确的反事实结论

- **仅删除这三个晚期异常**：直接结果不可识别；结合事件顺序、先前STOP轨迹和制动包络，碰撞仍很可能发生。
- **删除从`t_sample`到`t_phys`的全部初始响应延迟**：0 m接触边界下，两套模型均显示可能避免碰撞，但这与本次三个晚期尖峰不是同一个干预。
- **要求碰撞前还保留6 m余量**：即便删除全部初始响应距离，模型结论也对制动能力敏感，不能宣称稳健安全。

## 证据限制

- 无Apollo record、Control payload、逐命令Bridge receive/release/apply与Chassis反馈，无法判定每个Control动作episode及轨迹复用的精确物理效果。
- 无同初始状态“去掉三个异常”的回放，不能直接估计避免碰撞概率或异常可归因距离。
- `median+6×MAD`是research筛查阈值，不是Apollo architectural deadline或独立校准的物理deadline。
- 本报告只回答异常映射和碰撞影响，不判断三异常的共同根因。

## 复现与数据

- 映射表：`tables/anomaly_apollo_impact_mapping.csv`
- observed/model分离反事实：`tables/anomaly_collision_counterfactual.csv`
- 重绘散点图：`figures/component_timing_scatter_all_instances.png`
- 复现脚本：`scripts/analyze_1131_anomaly_collision_impact_v4_2.py`

Apollo模块关系也可从[官方核心模块架构说明](https://apollo.baidu.com/docs/apollo/10.x/md_docs_2_xE6_xA1_x86_xE6_x9E_xB6_xE8_xAE_xA1_2_xE6_xA0_xB8_xE5_xBF_x83_xE6_xA8_xA1_4c7232e9aaaaf30846a51b133b9b71bf.html)核对。
