# 字段与端点语义

本轮 `pw_20261020_141904`。源码快照在 `source_snapshots/`；它们是采集后读取的当前部署源码，尚未证明与采集时二进制逐字一致。数据枚举与端点时间同时用于交叉核对。

| 字段/端点 | 核实结果与来源 | 使用限制 |
|---|---|---|
| trace_id | Prediction 从 Perception Header 继承；输出 FillHeader 后重新复制。`prediction_component.cc:192–203,370–396` | 不是全局唯一帧 ID；所有 CSV 保留十进制字符串 |
| proc_id | 顶层 callback 为 0；模型及 inference 使用其他调用实例 ID；`prediction_trace_context.h` | 不能把所有非零 proc_id 丢弃，也不能把它们当独立 callback |
| frame_uid | run + 模块 + PID@starttime_ticks + trace_id + proc_id + callback event_id | 分析生成的稳定处理实例键；frame_index 仅用于浏览 |
| frame_instance_id | EvaluatorManager::Run 调用时用进程内原子 NextPredictionWorkerId 创建 | Evaluator 激活，不是传感器帧；通过显式 trace/context 关联 callback |
| task_id/group_id/pool_level | ForEach/Post 显式上下文，队列 lambda 恢复上下文；group_id 是逻辑分组 | TID 不能替代 group_id；本轮 Task ID 和激活 ID 来自进程内计数器 |
| primary_parent_trace_id | `trace_header_util.h:34–43` 返回 Header.parent_trace_ids(0) | 仅第一个父，不等于全部父 |
| parent_trace_count | Header.parent_trace_ids_size() | 不是运行时消费次数；本次使用 fusion_inputs 逐父行恢复父集合 |
| input_seq/output_seq | 来自各自 Channel 的 Header.sequence_num | 跨 Channel 的同序号不能合并；与 trace_id/data_ts/所属 run 联合匹配 |
| fusion_inputs.sensor_ts_ns | multi_sensor_fusion_component.cc:442–454 将同一触发帧 LiDAR 标签复制给全部父传感器 | 不是独立 sensor 时间，不能用于严格多源 Consistency |
| data_ts_ns | Prediction/Planning 边界传入 header.lidar_timestamp | 继承的 LiDAR 标签；不能替代全部融合输入，更不能假设属于本机 RAW |
| mono_ns | `perf_trace_writer.h:45–49` CLOCK_MONOTONIC_RAW | 只与同域事件直接相减；与 Perfetto 用本轮快照转换 |
| Prediction context in | PredictionEndToEndProc 参数处，同时写 proc_enter 和消息边界 | 是处理入口已持有的消息；不是 Reader 到达时刻 |
| context out / output_pub | Write 后回填 `publish_enter_ns`，与 writer_enter 同时 | 是 Writer 调用入口代理，不能写成发布完成 |
| writer_done | prediction_writer_->Write 返回后立即读取 RAW | Writer API 返回，不保证下游接收/实际消费完成 |
| task_submit_begin/end | 包围 queue.Enqueue，end 的 status 为 accepted/rejected | 精确入队时刻只在此区间内；queue_wait_ms 留空，保存上下界 |
| task_begin | lambda 恢复显式上下文后，调用 packaged_task 之前 | 实际 Worker TID 来自此事件；不由 callback TID 推断 |
| group_end / task_end | group_end 是执行函数作用域末端；task_end 在 packaged_task 返回之后 | Future Ready 未直接采集；task_end 是 Ready 上界，不能强接 task_end→future_get_begin |
| future_get_begin/end | 包围 future.get 的 RAII 作用域 | API 墙钟区间，包含 Running/Runnable/Sleeping；不是纯 blocked |
| Planning context in | PlanningComponent::Proc 收到 Prediction 指针，随后赋给 local_view | 先于 CheckInput；仅此钩子不能证明 RunOnce 执行。本次额外读取 RunOnce 事件核对 |
| record_time_ns | Recorder::ReaderCallback 调用 Time::Now 后传给 WriteMessage；Time::Now 使用 high_resolution_clock | Recorder 接收/写入入口标签，不是发布或传感器测量时刻。Linux epoch 解释用于覆盖审计，源物理 Age 不据此计算 |
| utid/host_tid | utid 为本份 Trace 内的线程生命周期键，host_tid 为 OS TID | 采集前 NSpid 与 Host PID/TID namespace 相同；结合 TP 所属进程和生命周期核对，不跨 run 复用 utid |

Execution 边的 `FRAME_ASSOCIATION` 仅表达显式身份成员关系，不是因果先后。SPAWN 使用 submit_begin→task_begin；JOIN 使用 group_end→future_get_end；SEQUENCE 使用同 Task 同 TID 的已结束调用→后续调用。RESUME 表示 future_get 返回的应用检查点，不表示内核唤醒事件。缺少真实事件的 SIGNAL、CRoutine 迁移和精确 Ready 不生成。

阶段耗时为 inclusive 墙钟。阶段可嵌套、重叠或在 callback 返回后继续；不累加为 CPU 时间或完整关键路径。`critical_path_segments.csv` 保存 callback 可观测包络和候选 Task 分支，`critical_flag` 留空，避免把未闭合路径写成已确认关键路径。
