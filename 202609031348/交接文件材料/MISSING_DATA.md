# MISSING_DATA

## 五个重点帧完整性结论

F611、F613、F616、F619、F623 均具备：应用层 P4 timing、唯一 Infer window、Host TID、perf sched summary、逐帧 perf raw-event extract 和未去重的 CPU stack raw samples。详见 [`integrity_audit.json`](index/integrity_audit.json)。

## 仍存在的证据缺口

- P2–P7 的严格 Reader receive/enqueue/drop 时间不可用；`p4_input_ready_time` 是上一节点 `output_pub` 代理。
- 当前资料没有提供 `e2e_trace_v3 mono_ns` 到 Perception log `CP_INFER mono_ns` 的显式校准 offset；两者按 source frame/sensor timestamp 建立关联，不能直接相减。perf sched 与 CP Infer/CPU stack 的 offset 已单独校准并保留。
- F612、F614、F615、F617、F618、F620、F621、F622、F624、F625、F640 没有唯一 `CP_INFER_ENTER/EXIT` 映射；对应 Infer、Host TID、perf window 和逐帧 stack 字段为空。应用层将这些帧标为 no-service；其中 F624–F625 不属于给定的 core phase。
- CPU stack 原始 sample header 保留 `comm`、数值 ID、CPU、timestamp 和 event；现有格式及审计将该数值 ID 用作 Host TID，但 sample 行没有同时展开独立的 PID 与 TID 两列。
- F624–F625 不属于给定的 `core_abnormal=F611–F623` 或 `recovery=F626–F649`，索引中的 `phase` 有意留空。

这些缺口未使用相邻帧、平均值或模型值补齐。
