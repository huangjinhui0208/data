# 指标与证据来源

| 项目 | 证据层 | 说明 |
|---|---|---|
| CARLA 0.9.15、Apollo 10.0.0、Bridge直接读取Control | A 配置/部署说明 | 来自工作区 AGENTS.md；Guardian Trace不作为车辆执行链输入 |
| 名义300 ms注入、约560k点云、配置队列长度1 | A 实验设定 | 该项来源于实验设定，当前归档中缺少独立配置文件快照。 |
| t1、t2、T_e2e、D1、D_delay、D2、停车/碰撞结局 | B 直接观测 | 从原始Fusion/Trace/Localization/SCB/CollisionSensor/actor history统一重算 |
| baseline恢复反事实、预测碰撞余量/冲击速度 | C 模型 | 单独保存在counterfactual_model.csv，不回填观测结果 |

主结果只以B类证据下结论；A类用于说明系统和实验边界；C类只回答局部“如果更早响应”问题。
