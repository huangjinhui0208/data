# 202609031348 Perception 实时性逐帧检查

## 结论

判定：`P4_OVERLOAD_CHAIN_REPRODUCED_RECOVERY_NOT_ESTABLISHED`。

- 选定窗口：2026-09-03T13:48:47.431665+08:00 至 2026-09-03T13:49:55.031707+08:00，677 个 lidar 源帧；窗口模式 `all_fusion_frames`。
- P4 proxy deadline：195/676 miss，execution/waiting/no-service = 117/51/27。
- P4 execution 相邻帧突增：有，标记 7 帧（[30, 36, 69, 77, 316, 413, 623]）；本 run 的稳健增量阈值为 30.000 ms，且要求相邻帧倍率不低于 1.5。
- 完整 Perception freshness：615/676 miss。
- P4 execution P50/P95/MAX：89.421/112.082/285.599 ms；waiting MAX 99.012 ms。
- CenterPoint inference P50/P95/MAX：79.421/97.984/276.118 ms。
- GroundDetection buffer overflow：24 条警告，drop_message 合计 27；全通道统计仅作旁证，不混入 P4 判定。

## 口径与证据边界

P2–P7 使用上一节点 `output_pub` 作为 input-ready proxy，P1 使用自身 callback start；strict Reader arrival/enqueue 不可用。完整 Perception deadline 定义为本帧 P1 `proc_enter` 到 P7 `output_pub` 必须不晚于下一帧 P1 `proc_enter`。本次按用户要求仅检查 Perception，Prediction、Planning、Control、bridge 与车辆动力学均不在范围内。

独立结构与算术检查：6/6 通过。
