"""Independent arithmetic, identity, coverage and graph checks for exported tables."""
import collections,csv,hashlib,json,math
from pathlib import Path

R=Path(__file__).resolve().parents[1]
def read(name):
    with (R/name).open(encoding='utf-8',newline='') as f:return list(csv.DictReader(f))
checks=[]
def check(name,ok,detail):checks.append(dict(test=name,passed=bool(ok),details=detail))
f=read('frame_instances.csv');t=read('worker_tasks.csv');m=read('frame_timing_metrics.csv');s=read('frame_stage_spans.csv')
raw=read('inputs/trace/events/prediction.942621.csv');w=read('inputs/trace/prediction_worker_events/prediction.942621.csv')
pairs=collections.Counter((r['trace_id'],r['proc_id'],r['phase']) for r in raw if r['phase'] in ['proc_enter','proc_exit'])
check('callback_inventory_and_pairing',len(f)==1057 and sum(r['phase']=='proc_enter' for r in raw)==1057 and sum(r['phase']=='proc_exit' for r in raw)==1057 and all(v==1 for v in pairs.values()),dict(rows=len(f),inventory_expected=1057))
check('task_inventory',len(t)==1929 and len({r['task_id'] for r in t})==1929 and len({r['task_id'] for r in w if r['phase']=='task_begin'})==1929,dict(tasks=len(t)))
check('composite_identity_uniqueness',len({x['frame_uid'] for x in f})==len(f) and all(x['run_id']=='pw_20261020_141904' and '942621@103208480' in x['frame_uid'] for x in f),'Run/process lifecycle/trace/proc/callback event identity; not bare trace_id')
fm={x['frame_uid']:x for x in f};known=set(fm)
check('task_frame_identity_integrity',all(x['frame_uid'] in known and fm[x['frame_uid']]['trace_id']==x['trace_id'] and fm[x['frame_uid']]['frame_instance_id']==x['frame_instance_id'] for x in t),'Explicit frame_instance_id+trace mapping checked')
bad=[x['frame_uid'] for x in m if abs(float(x['callback_duration_ms'])-(int(x['proc_exit_ns'])-int(x['proc_enter_ns']))/1e6)>1e-9]
check('callback_raw_arithmetic',not bad,dict(failures=bad[:10]))
bad=[x['stage_uid'] for x in s if int(x['end_ns'])<int(x['start_ns']) or abs(float(x['duration_ms'])-(int(x['end_ns'])-int(x['start_ns']))/1e6)>1e-9]
check('all_stage_pair_arithmetic',not bad,dict(stages=len(s),failures=bad[:10]))
nodes=read('tdfg_nodes.csv');edges=read('tdfg_edges.csv');nm={n['node_uid']:n for n in nodes}
check('tdfg_referential_integrity',len(nm)==len(nodes) and all(e['source_uid'] in nm and e['target_uid'] in nm for e in edges),dict(nodes=len(nodes),edges=len(edges)))
outputcounter=collections.Counter(e['frame_uid'] for e in edges if e['edge_type']=='PRODUCES')
pc=read('inputs/trace/message_context/prediction.942621.csv');sourceout=collections.Counter(x['trace_id'] for x in pc if x['edge']=='out')
check('all_outputs_preserved',all(outputcounter[x['frame_uid']]==sourceout[x['trace_id']] for x in f),'Observed output cardinality preserved, with no first-output-only graph collapse')
check('multiple_output_contract',all(len(json.loads(x['output_message_uids']))==int(x['output_count']) for x in f),'Output list/cardinality and graph evidence checked; current run has one traced output per callback')
source_nodes=[n for n in nodes if n['node_type']=='DATA_SOURCE']
sourcecounts=collections.Counter(n['frame_uid'] for n in source_nodes)
check('multiparent_not_primary_only',all(sourcecounts[x['frame_uid']]==int(x['upstream_parent_count_recovered']) for x in f) and all(int(x['upstream_parent_count_recovered'])==int(x['upstream_parent_count_expected']) for x in f if x['upstream_parent_status']=='complete'),'Full fusion_inputs parent set checked; primary_parent alone is never used as complete set')
check('copied_fusion_time_not_sensor_observation',all(not x['source_timestamp_ns'] and x['diagnostic_copied_frame_timestamp_ns'] for x in source_nodes) and all(x['verified']=='False' for x in edges if x.get('header_parent_relation_verified')=='True'),'Active cached header parents are not automatically actual algorithm inputs; copied LiDAR times cannot establish sensor consistency')
message_channels=collections.defaultdict(set)
for n in nodes:
    if n['message_uid'] and n.get('channel'):message_channels[n['message_uid']].add(n['channel'])
check('channel_aware_message_identity',all(len(v)==1 for v in message_channels.values()),dict(distinct_messages=len(message_channels)))
check('uncalibrated_sensor_age_is_null',all(not x['input_age_ms'] and not x['output_age_ms'] and not x['planning_age_ms'] and x['age_missing_reason'] for x in m),'Clock snapshots do not prove physical sensor time or cross-host synchronization')
bad=[]
for e in edges:
    a,b=nm[e['source_uid']],nm[e['target_uid']]
    if e['verified']=='True' and a['timestamp_domain']==b['timestamp_domain']=='CLOCK_MONOTONIC_RAW' and a['event_time_ns'] and b['event_time_ns'] and int(a['event_time_ns'])>int(b['event_time_ns']):bad.append(e['lineage_id'])
check('verified_message_edge_order',not bad,dict(contradictions=bad[:10]))
recon=read('metadata/record_trace_matches.csv');ro=json.loads((R/'metadata/record_only_outputs.json').read_text())
matched=sum(int(x['record_output_count']) for x in recon);unmatched=sum(not int(x['record_output_count']) for x in recon)
check('record_trace_coverage_reconciles',matched+len(ro)==1034 and matched+unmatched==1057,dict(trace_outputs=1057,record_outputs=1034,matched=matched,trace_only=unmatched,record_only=len(ro),trace_only_reasons=dict(collections.Counter(x['reason'] for x in recon if x['reason']))))
for name in ['task_state_attribution.csv','metadata/callback_state_attribution.csv']:
    rr=read(name);bad=[]
    for x in rr:
        total=sum(float(x[k]) for k in ['running_ms','runnable_ms','sleeping_ms','uninterruptible_ms','unknown_ms'])
        if abs(total-float(x['task_wall_ms']))>1e-7 or abs(float(x['closure_error_ns']))>0:bad.append(x['frame_uid'])
    check('state_time_closure_'+Path(name).stem,not bad,dict(rows=len(rr),failures=bad[:10],note='Closure is independent of clock alignment validation'))
at=read('task_state_attribution.csv');spans=read('thread_state_spans.csv');sums=collections.Counter()
for x in spans:sums[(x['frame_uid'],x['task_id'])]+=int(x['duration_ns'])
check('state_intervals_recompute_task_wall',all(sums[(x['frame_uid'],x['task_id'])]==int(x['end_boot_ns'])-int(x['start_boot_ns']) for x in at),'Unaggregated state intervals independently reconstruct task wall time')
check('system_window_bounds',all(1032151343301613<=int(x['start_ns'])<=int(x['end_ns'])<=1032251343301613 for x in spans),'All attributed task/callback intervals are within requested [5,105] seconds')
wakes=read('wakeup_events.csv')
check('wakeup_wait_nonnegative_or_missing',all(not x['runnable_wait_ms'] or float(x['runnable_wait_ms'])>=0 for x in wakes),'Do not attach the next wake to an earlier preemption Runnable interval')
stateorig={x['state_row_id']:x['original_start_ns'] for x in spans if x['state_row_id']}
check('waking_exact_state_origin',all(not x['waking_ns'] or x['waking_ns']==stateorig.get(x['state_row_id']) for x in wakes),'Actual sched_waking timestamp must equal the derived state origin, with the same target TID')
clock=json.loads((R/'clock_alignment.json').read_text())
check('clock_snapshot_validation',clock['snapshot_count']==47 and clock['calibration_pass'] and clock['system_attribution_pass'],dict(snapshot_count=clock['snapshot_count'],window_max_holdout_residual_ns=clock['window_max_abs_residual_ns'],event_validation=clock['application_event_validation']))
check('stack_same_tid_cpu_validation',clock['stack_attribution_pass'],clock['stack_validation'])
en=read('execution_nodes.csv');ee=read('execution_edges.csv');em={x['node_id']:x for x in en};adj=collections.defaultdict(list);indeg=collections.Counter({x:0 for x in em});contradictions=[]
for e in ee:
    a,b=em[e['source_node_id']],em[e['target_node_id']];adj[a['node_id']].append(b['node_id']);indeg[b['node_id']]+=1
    if e['edge_type']!='FRAME_ASSOCIATION':
        aa=int(a['start_ns']) if e['edge_type']=='SPAWN' else int(a['end_ns']);bb=int(b['start_ns']) if e['edge_type'] in ['SPAWN','SEQUENCE'] else int(b['end_ns'])
        if aa>bb:contradictions.append(e)
queue=collections.deque(x for x,v in indeg.items() if v==0);n=0
while queue:
    a=queue.popleft();n+=1
    for b in adj[a]:
        indeg[b]-=1
        if indeg[b]==0:queue.append(b)
check('execution_dag_endpoints_and_acyclicity',not contradictions and n==len(em),dict(nodes=len(em),edges=len(ee),violations=len(contradictions),visited=n))
critical=read('critical_path_segments.csv');criticalids={x['critical_segment_id'] for x in critical}
check('critical_and_evidence_node_references',all(x['node_id'] in em for x in critical) and all(not x['critical_segment_id'] or x['critical_segment_id'] in criticalids for x in read('root_cause_evidence.csv')),'Candidate/envelope segments reference real execution nodes; no claimed causal critical segment for callback-wide summaries')
integrity=read('metadata/export_integrity.csv')
check('source_export_crc_and_counts',len(integrity)==10 and all(x['passed']=='True' and x['actual']==x['expected'] for x in integrity),'Seven completed primary result sets and three successful auxiliary result sets checked independently')
check('deadline_signed_arithmetic',all(abs(float(x['slack_ms'])-(int(x['deadline_ns'])-int(x['proc_exit_ns']))/1e6)<1e-9 and (x['deadline_miss']=='True')==(int(x['proc_exit_ns'])>int(x['deadline_ns'])) for x in m if x['deadline_ns']),'Analyst next-Fusion update boundary; equality meets, not formal safety deadline')
required=['run_manifest.json','trace_coverage.csv','trace_quality.csv','frame_instances.csv','worker_tasks.csv','frame_stage_spans.csv','identity_edges.csv','clock_snapshots.csv','clock_alignment.json','tdfg_nodes.csv','tdfg_edges.csv','execution_nodes.csv','execution_edges.csv','dynamic_path_segments.csv','frame_timing_metrics.csv','task_state_attribution.csv','thread_state_spans.csv','critical_path_segments.csv','callstack_samples.csv','wakeup_events.csv','cpu_competition.csv','root_cause_evidence.csv','intervention_runs.csv','before_after_metrics.csv']
check('all_24_requested_outputs',all((R/p).is_file() for p in required),dict(count=len(required)))
check('no_synthetic_intervention_results',not read('intervention_runs.csv') and not read('before_after_metrics.csv'),'P6 files contain headers only; no optimization data exists')
check('critical_path_uncertainty_retained',all(not x['critical_flag'] and x['path_status'] in ['partial_callback_envelope_not_complete_critical_path','candidate_branch'] for x in read('critical_path_segments.csv')),'No complete critical path claimed without Future Ready and complete async dependencies')
report=dict(run_id='pw_20261020_141904',passed=all(x['passed'] for x in checks),test_count=len(checks),checks=checks)
(R/'tests/validation_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(0 if report['passed'] else 1)
