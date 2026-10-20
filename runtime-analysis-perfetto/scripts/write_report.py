"""Generate provenance, data dictionary and concise scientific-use notes."""
import collections,csv,hashlib,json
from pathlib import Path
import numpy as np
from build_tables import ROOT as R, RUN, write, dump, read, stats

s=json.loads((R/'metadata/global_summary.json').read_text());clock=json.loads((R/'clock_alignment.json').read_text())
frames=read(R/'frame_instances.csv');metrics=read(R/'frame_timing_metrics.csv');tasks=read(R/'worker_tasks.csv');stages=read(R/'frame_stage_spans.csv')
inside=[r for r in metrics if r['window_status']=='inside'];sys=s['system'];cb=read(R/'metadata/callback_state_attribution.csv');ta=read(R/'task_state_attribution.csv')
selection=json.loads((R/'metadata/extraction_request.json').read_text());chosen={r['frame_uid']:r['selection_reason'] for r in selection['selected_frames']}
stagep99={}
for name in {r['stage_name'] for r in stages}:
    rr=[float(r['duration_ms']) for r in stages if r['stage_name']==name and r['window_status']=='inside']
    if rr:stagep99[name]=float(np.percentile(rr,99))
stagematches=collections.defaultdict(list)
for r in stages:
    if r['window_status']=='inside' and float(r['duration_ms'])>=stagep99.get(r['stage_name'],float('inf')):stagematches[r['frame_uid']].append(r['stage_name'])
dispatch={r['frame_uid']:(int(r['proc_enter_ns'])-int(r['release_ns']))/1e6 for r in inside if r['release_ns']};dp99=float(np.percentile(list(dispatch.values()),99))
gaps=[float(r['output_interval_ms']) for r in inside if r['interval_window_status']=='inside'];gp99=float(np.percentile(gaps,99));candidates=[]
for r in inside:
    reasons=[]
    if float(r['callback_duration_ms'])>=s['callback_statistics_window_ms']['p99']:reasons.append('callback_p99')
    if stagematches[r['frame_uid']]:reasons.append('stage_p99')
    if dispatch.get(r['frame_uid'],-1)>=dp99:reasons.append('dispatch_proxy_p99')
    if r['deadline_miss']=='True':reasons.append('analyst_next_fusion_miss')
    if r['interval_window_status']=='inside' and float(r['output_interval_ms'])>=gp99:reasons.append('writer_return_gap_p99')
    if r['frame_uid'] in chosen:reasons.append(chosen[r['frame_uid']])
    if reasons:candidates.append(dict(run_id=RUN,frame_uid=r['frame_uid'],trace_id=r['trace_id'],selection_reasons=reasons,p99_stages=stagematches[r['frame_uid']],dispatch_proxy_ms=dispatch.get(r['frame_uid']),callback_ms=r['callback_duration_ms'],slack_ms=r['slack_ms'],p5_detailed_evidence=r['frame_uid'] in chosen,missing_rules='workload residual model and physical source age not established'))
write('metadata/anomaly_candidate_union.csv',candidates)

requested=['run_manifest.json','trace_coverage.csv','trace_quality.csv','frame_instances.csv','worker_tasks.csv','frame_stage_spans.csv','identity_edges.csv','clock_snapshots.csv','clock_alignment.json','tdfg_nodes.csv','tdfg_edges.csv','execution_nodes.csv','execution_edges.csv','dynamic_path_segments.csv','frame_timing_metrics.csv','task_state_attribution.csv','thread_state_spans.csv','critical_path_segments.csv','callstack_samples.csv','wakeup_events.csv','cpu_competition.csv','root_cause_evidence.csv','intervention_runs.csv','before_after_metrics.csv']
manifest=[];dictionary=[]
for name in requested:
    p=R/name;count=None;fields=[]
    if p.suffix=='.csv':
        with p.open(encoding='utf-8',newline='') as f:
            rr=csv.DictReader(f);fields=rr.fieldnames;count=sum(1 for _ in rr)
        for col in fields:
            unit='ns' if col.endswith('_ns') or col in ['raw_ns','boottime_ns','monotonic_ns','realtime_ns'] else 'ms' if col.endswith('_ms') else None
            dictionary.append(dict(table=name,column=col,storage='CSV text; parse IDs and integer nanoseconds losslessly',unit=unit,meaning='See run_manifest.metric_definitions, metadata/field_semantics.md and per-row reason fields',null_policy='empty = unavailable/not applicable; never convert to zero',clock_rule='*_boot_ns = BOOTTIME; P1/application start_ns/end_ns = RAW; P4/P5 spans = BOOTTIME; consult timestamp_domain'))
    manifest.append(dict(file=name,rows=count,bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),availability='headers_only_no_intervention' if name in ['intervention_runs.csv','before_after_metrics.csv'] else 'partial_critical_path_not_certified' if name=='critical_path_segments.csv' else 'available_with_per_row_limits'))
write('metadata/deliverable_manifest.csv',manifest);write('metadata/data_dictionary.csv',dictionary)
dump(R/'metadata/analysis_scope.json',dict(run_id=RUN,callback_window=[r['frame_uid'] for r in inside],p5_selected_frames=selection['selected_frames'],candidate_union_count=len(candidates),p5_detailed_frame_count=len(chosen),p5_selection='Seven callback >= p99 frames and two controls matched by obstacle count and forward-call count. Other model/history properties not proven equivalent.',unknown_or_incomplete=['complete CRoutine path','exact Future Ready','certified complete critical path','GPU kernels/copies/streams','physical source clock','capture-time CPU eligibility constraints','intervention outcomes'],method_reuse='Reviewed reusable_scripts/INDEX.md and Prediction S1-S7 contracts. Existing system parsers expect perf.data/text, not Perfetto SQL. New adapter directly queries existing TP normalized tables; it does not fork historical per-run perf parsers.'))

semantics='''# 字段与端点语义

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
'''
(R/'metadata/field_semantics.md').write_text(semantics,encoding='utf-8')
rows='\n'.join(f'| {x["file"]} | {x["rows"] if x["rows"] is not None else "JSON"} | {x["availability"]} |' for x in manifest)
cbtotal={k:sum(float(r[k]) for r in cb) for k in ['running_ms','runnable_ms','sleeping_ms','uninterruptible_ms','unknown_ms']}
tatotal={k:sum(float(r[k]) for r in ta) for k in ['running_ms','runnable_ms','sleeping_ms','uninterruptible_ms','unknown_ms']}
report=f'''# 5—105 秒跨层数据交付

Run：`{RUN}`。结果目录：`D:\\data\\runtime-analysis-perfetto`。

窗口原点为 Trace Processor `trace_bounds.start_ts=1032146343301613` BOOTTIME ns。统计窗口 `[1032151343301613,1032251343301613)`，即 Trace 相对 5—105 秒；北京时间约 2026-10-20 14:19:10.582—14:20:50.582。它与命令启动、READY 或应用首事件的零点不同。

## 交付范围与数量

全部 **1,057** 个 callback 和 **1,929** 个已开始 Task 的身份账本均保留；完整落入窗口的 **693** 个 callback、**724** 个 Task 用于系统分解。窗口外账本不计入该窗口的性能统计。完整配对应用阶段 **{len(stages):,}** 个，保留嵌套、非零 proc_id 和异步阶段。

| 文件 | 行数 | 可用性 |
|---|---:|---|
{rows}

所有表以 run_id/frame_uid 关联；task_id 的全局作用域是本轮进程生命周期。trace_id 和所有大整数 ID 按字符串读取。CSV 空值不是 0；原始应用 RAW 时间与转换后的 *_boot_ns 同时保留。P4/P5 的原始输入是 Trace Processor 已归一化的 BOOTTIME，不能再次加应用时钟偏移。

## P0：采集质量与覆盖

原始采集 `FAILED_OR_INCOMPLETE` 保留。三个 perf 源触发 guardrail，均发生在选定窗口之后；这不消除窗口内部的 unwind 错误。5—105 秒有 system-wide sched **24,214,611** 条；CPU-clock/perf-switch/perf-waking 样本分别 **118,800 / 1,051,246 / 1,051,938** 条，对应 unwind error **1,596 / 45,807 / 95,407**。有样本不等于栈完整。

目标进程采集前身份快照记录 PID/Host PID 942621、starttime_ticks=103208480、同 PID namespace。采集结束后身份快照失败仍为限制；窗口内用 TP 线程所属进程、utid 及生命周期交叉核对。采集时 Apollo commit、功耗模式、完整 CARLA actor/weather 配置未知，未用当前运行配置补成历史事实。

初次 SQL 导出在前七个结果集完成后，中止了耗时的 CPU 关联语句；该作业返回码 1 原样保留。仅使用已完整输出并通过 gzip CRC/行数核验的七个结果集。CPU 上下文、窗口计数及质量计数由第二个独立作业成功完成（返回码 0），随后在本机按实际等待区间关联。见 metadata/system_summary.json；这与原始采集失败标记分开记录。

## P1/P2：身份与时钟

47 组快照支持 RAW→BOOTTIME 分段线性 offset 插值。偶数快照拟合、奇数快照留出验证；最终参数保留所有快照，禁止外推。选定窗口内留出最大残差 **{clock['window_max_abs_residual_ns']/1000:.3f} μs**。独立应用事件验证 **{clock['application_event_validation']['samples']}** 个，同 TID Running 支持率 100%，100 μs 容差下最大到运行区间距离 {clock['application_event_validation']['maximum_distance_ns']} ns。另有 **11,606** 个 perf 样本在同 TID、同 CPU 的 2 μs 容差内支持率 100%。支持率与时间误差分别报告。

时钟快照插值的误差不是严格上界，极短 Task/状态边界仍有微秒级归属不确定性。窗口外可生成时间映射，但未作为已通过本窗口系统审计的样本使用。

## P3：数据流、执行与时间指标

消息依赖按 Channel+trace_id+sequence+data_ts 匹配，并用源码语义确认；多父 Header 集合来自 fusion_inputs，而非只取 primary_parent；这些父节点来自 active sensor trace 缓存，实际算法逐输入消费未观测，故源传感器→Fusion 边保留为 unverified。窗口内主链完整关联 **{s['lineage_main_chain_complete_window']}/{s['callback_inside']} ({100*s['lineage_main_chain_complete_window']/s['callback_inside']:.2f}%)**。此“完整”仅限 Fusion 输出消息→Prediction→Planning RunOnce 主消息链。完整物理来源血缘为 0；全部实例 lineage_status 保留 partial。Localization、Storytelling、历史 Planning 等身份也未全恢复。

Record 显式匹配结果：Trace 输出 1,057，Record 输出 1,034，共同消息 **1,001**；Trace-only **56** 个均早于 Record 第一份 Prediction 输出序号；Record-only **33** 条。全量差值 23 不等于丢帧。覆盖详见 metadata/record_trace_matches.csv 和 metadata/record_only_outputs.json。

窗口 callback 墙钟：均值 **{s['callback_statistics_window_ms']['mean']:.3f} ms**，中位数 **{s['callback_statistics_window_ms']['median']:.3f} ms**，p95 **{s['callback_statistics_window_ms']['p95']:.3f} ms**，p99 **{s['callback_statistics_window_ms']['p99']:.3f} ms**，最大 **{s['callback_statistics_window_ms']['max']:.3f} ms**。Fusion 发布代理→callback 退出的响应均值 {s['release_response_window_ms']['mean']:.3f} ms；该量与 callback 自身耗时分列。

沿用本工作区“完整 Fusion 输出流下一份真实输出”更新边界策略，693 个可判定实例中 35 个超出边界，比例 **5.05%**。该策略是分析者选择的更新预算，**不是 Apollo 安全契约**；没有固定 100 ms Safety Deadline。signed slack 使用真实下一输出时间减 proc_exit；相等记 meet。

输入间隔用 proc_enter，表示 callback 入口节奏，不是 Reader 到达。输出间隔用 writer_done，表示 Write 返回节奏；692 个窗口内相邻间隔，最大 **{s['writer_return_interval_window_ms']['max']:.3f} ms**。20 个连续输出间隔的跨度共有 **673** 个完全在窗口内的结果。没有正式 I/theta，故不判稳定性违约。

物理源时间语义和跨主机同步未验证：Freshness 和严格 Consistency 均空值且有原因。源码显示 fusion_inputs.sensor_ts_ns 对每个父都复制触发帧的 in_message->lidar_timestamp_，不是该传感器独立观测时刻。复制值保存在 diagnostic_copied_frame_timestamp_ns，fusion_audit_copied_tag_span_ms 只审核复制标签；即使为 0，也不能证明多源一致性良好。

## P4：可复核的状态区间

保存 **{sys['state_spans']:,}** 条 callback/Task 裁剪状态区间，其中原始 state、state_row_id、原始完整区间边界、utid、CPU、waker_utid 和裁剪后边界均可追溯。所有 693 个 callback 和 724 个 Task 的未知时长为 0，状态闭合误差为 0 ns。Sleeping(S/I) 与 Uninterruptible(D) 分列、不重复相加；R 与 R+ 都计 Runnable。

callback 线程累计量（ms）：{json.dumps(cbtotal,ensure_ascii=False)}。

Task 线程累计量（ms）：{json.dumps(tatotal,ensure_ascii=False)}。

这些是线程累计量，父 callback 与并行 worker 有重叠，不能相加为关键路径或根因贡献。Task 排队时间的精确 enqueue 点未观测，queue_wait_ms 留空，worker_tasks.csv 保留上下界。

critical_path_segments.csv 提供逐 callback 的已观察状态包络和关键帧候选 Task 分支。缺少精确 Future Ready、完整异步完成/通知链，**尚未认证完整关键路径**；critical_flag 留空。没有用某个最晚 task_end 或父线程全部睡眠来补造关键路径。

## P5：关键帧证据与结论边界

全量候选并集见 metadata/anomaly_candidate_union.csv：callback/stage/派发代理长尾、更新边界 miss、输出停顿及正常对照均有单独规则，共 {len(candidates)} 个候选/对照实例。详细调用栈/唤醒/CPU 证据覆盖 **9** 个代表实例：7 个窗口 callback≥p99 实例及 2 个按障碍物/forward 次数匹配的对照。其他模型和历史状态的可比性未完全控制，不据此量化因果贡献。

详细样本保存 **{sys['stack_rows']:,}** 条栈记录、**{sys['wakeup_rows']:,}** 条唤醒/就绪记录、**{sys['competition_rows']:,}** 条 CPU 重叠记录。普通采样、switch 事件栈和 waking 事件栈分列。栈的部分展开和原始 unwind_error 原样保存。

cpu_competition 使用目标线程随后实际恢复 CPU 的可见占用，只证明重叠，不证明该 CPU 是唯一可运行 CPU，也不证明占用者造成了全部延迟。策略/亲和性/配额未完整记录时保持缺失。中断上下文的 waker TID 也不自动等于真正设备/锁的唤醒主体。

root_cause_evidence 仅为 E1 观测候选。当前未确认锁持有者、GPU kernel/stream、完整 Future 通知链或系统机制造成的可归因增量。工作区等级为 E0–E3，用户提到的 E4 没有既定定义，因此不生成 E4。无 GPU 时间线时，CUDA/驱动函数只能提供 CPU 端线索。

## P6：未实施优化实验

intervention_runs.csv 和 before_after_metrics.csv 均为有 schema 的空表。当前没有干预组、重复实验或优化后数据，不虚构 before/after 数值，不将相关性写成因果验证。

## F647—F680 甘特图

[34 帧总览 PNG](figures/prediction_F647_F680/prediction_gantt_F647_F680_overview.png)、[矢量 SVG](figures/prediction_F647_F680/prediction_gantt_F647_F680_overview.svg)及[6 页阶段详图和绘图数据](figures/prediction_F647_F680/README.md)。全部图使用 F647 callback 入口为统一零点，保留阶段嵌套与实际 TID；不将阶段累计耗时解释为关键路径。

## 复核与复现

本机依赖仅 Python 标准库与 numpy。原始大型 Trace、应用 CSV 和 Record 仍位于 Orin，本机 inputs/ 是证据缓存且不加入 Git；仓库提交 24 个要求文件、分析脚本、源码快照、SQL、元数据、测试与本文。

```powershell
$py = 'C:\\Users\\22142\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe'
& $py -X utf8 D:\\data\\runtime-analysis-perfetto\\scripts\\build_tables.py application
# 重新提取时：将 metadata/extraction_request.json 及 scripts/extract_perfetto_remote.py
# 放到 /media/ssd/hjh/runtime_analysis_exports/pw_20261020_141904_5_105/，运行远端脚本。
# 再运行 scripts/extract_context_remote.py，并将两个导出目录复制回 inputs/。
& $py -X utf8 D:\\data\\runtime-analysis-perfetto\\scripts\\build_tables.py system
& $py -X utf8 D:\\data\\runtime-analysis-perfetto\\scripts\\build_tables.py final
& $py -X utf8 D:\\data\\runtime-analysis-perfetto\\scripts\\write_report.py
& $py -X utf8 D:\\data\\runtime-analysis-perfetto\\scripts\\validate_tables.py
```

按某一 frame_uid 查询所有关联表可用 scripts/query_frame.py；不要把窗口外实例纳入窗口统计。验收结果见 tests/validation_results.json；原始文件散列、产物 SHA-256 和数据字典在 metadata/ 下。

方法参考：本项目是离线证据整理与部分图恢复；不声称部署了 Kairos 编译器插桩或 BCOZ 因果实验。[Kairos §4](https://www.usenix.org/system/files/osdi24-li.pdf)提供时间标签和时序约束模型；[BCOZ](https://www.usenix.org/conference/osdi24/presentation/ahn)启发联合保存 On-/Off-CPU 证据，但其因果分析并未由这些观测表自动实现。[Perfetto SQL 文档](https://perfetto.dev/docs/analysis/sql-tables)用于核对 utid、thread_state 和 sched 的语义。
'''
(R/'README.md').write_text(report,encoding='utf-8')
dump(R/'metadata/data_quality_report.json',dict(run_id=RUN,source_capture_status='FAILED_OR_INCOMPLETE',window_alignment_passed=clock['system_attribution_pass'],stack_alignment_passed=clock['stack_attribution_pass'],unknown_entities=sys['unknown_entities'],state_closure_max_error_ns=sys['maximum_closure_error_ns'],physical_source_age_available=False,certified_critical_path_available=False,causal_root_confirmed=False,interventions_available=False,record_trace_reconciliation=dict(trace=1057,record=1034,matched=1001,trace_only_before_record=56,record_only=33),deliverables=manifest))
print(json.dumps(dict(deliverables=len(manifest),candidate_union=len(candidates),report=str(R/'README.md'))))
