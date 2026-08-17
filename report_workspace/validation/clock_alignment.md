# 时钟对齐说明

- 主分析只使用 Apollo/Localization 墙钟 epoch 时间线：`t1`、`t2`、`D_delay`与停车端点共用同一基准。
- `D_delay` 是速度对墙钟时间的梯形积分；不混用 CARLA 帧数、sim time 或 Localization 空间位移。
- 主分析中 9 个无 actor history run（另有被排除的 `1206` 也无 actor history）状态为 `LIMITED_NO_DUAL_CLOCK_HISTORY`；这不影响墙钟主指标，但不能用于估计 realtime factor。
- 两个碰撞 run 使用 actor history 拟合 CARLA sim time→wall time。其状态为 `ALIGNED` 和 `ALIGNED`，p95残差分别为 0.661 ms 和 0.720 ms。
- Trace 内部阶段耗时使用 monotonic clock，通过 `trace_anchor.ingress_ms` 和目标 trace ID 与源时刻对齐。
