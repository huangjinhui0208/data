"""Source-backed report evidence; reads existing exports without changing them."""
import bisect
import collections
import csv
import hashlib
import json
import pathlib
import statistics
import datetime

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / 'prediction_realtime_report'
OUT.mkdir(exist_ok=True)
DATA = OUT / 'data'
DATA.mkdir(exist_ok=True)
RUN = 'pw_20261020_141904'
ORIGIN = 1032146343301613


def read(name):
    with (ROOT / name).open(encoding='utf-8-sig', newline='') as h:
        return list(csv.DictReader(h))


def save(name, rows):
    rows = list(rows)
    if not rows:
        return
    for row in rows:
        row.setdefault('run_id',RUN)
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (DATA / name).open('w', encoding='utf-8', newline='') as h:
        w = csv.DictWriter(h, fields)
        w.writeheader()
        w.writerows(rows)


def stats(values):
    a = np.array(values, dtype=float)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), p99=float(np.quantile(a, .99)),
                max=float(a.max()), std_population=float(a.std())) if len(a) else dict(count=0)


frames = read('frame_instances.csv')
fm = {r['frame_uid']: r for r in frames}
metrics = {r['frame_uid']: r for r in read('frame_timing_metrics.csv')}
stages = read('frame_stage_spans.csv')
tasks = read('worker_tasks.csv')
states = read('thread_state_spans.csv')
cb = {r['frame_uid']: r for r in read('metadata/callback_state_attribution.csv')}
taskstates = {(r['frame_uid'], r['task_id']): r for r in read('task_state_attribution.csv')}
raw = read('inputs/trace/prediction_worker_events/prediction.942621.csv')
window = [r for r in frames if r['window_status'] == 'inside']
sig = collections.defaultdict(collections.Counter)
events = collections.defaultdict(list)
for r in raw:
    events[r['frame_instance_id']].append(r)
    if r['phase'] in ['group_member', 'forward_begin', 'model_cruise_mlp_enter',
                      'model_jointly_prediction_planning_enter', 'model_cyclist_keep_lane_enter']:
        sig[r['frame_instance_id']][(r['group_id'], r['phase'])] += 1
by_signature = collections.defaultdict(list)
for f in window:
    by_signature[tuple(sorted(sig[f['frame_instance_id']].items()))].append(f)
ledger = []
for f in window:
    uid = f['frame_uid']; m = metrics[uid]; c = cb[uid]
    peers = [x for x in by_signature[tuple(sorted(sig[f['frame_instance_id']].items()))] if x['frame_uid'] != uid]
    baseline = statistics.median(float(metrics[x['frame_uid']]['callback_duration_ms']) for x in peers) if len(peers) >= 4 else None
    e = events[f['frame_instance_id']]
    row = dict(run_id=RUN, frame_uid=uid, frame=int(f['frame_index']), trace_id=f['trace_id'],
               proc_id=f['proc_id'], pid=f['pid'], tid=f['callback_tid'],
               event_id_enter=f['event_id_enter'], event_id_exit=f['event_id_exit'],
               start_s=(int(f['proc_enter_boot_ns']) - ORIGIN) / 1e9,
               raw_start_ns=f['proc_enter_ns'], boot_start_ns=f['proc_enter_boot_ns'],
               callback_ms=float(m['callback_duration_ms']), response_ms=float(m['response_ms']),
               dispatch_proxy_ms=float(m['response_ms'])-float(m['callback_duration_ms']),
               slack_ms=float(m['slack_ms']), miss=m['deadline_miss']=='True',
               running_ms=float(c['running_ms']), runnable_ms=float(c['runnable_ms']),
               sleeping_ms=float(c['sleeping_ms']), uninterruptible_ms=float(c['uninterruptible_ms']),
               unknown_ms=float(c['unknown_ms']), task_count=sum(x['phase']=='task_begin' for x in e),
               obstacles=sum(x['phase']=='group_member' for x in e),
               forward_calls=sum(x['phase']=='forward_begin' for x in e),
               jointly_calls=sum(x['phase']=='model_jointly_prediction_planning_enter' for x in e),
               same_signature_peer_count=len(peers), baseline_same_signature_median_ms=baseline,
               baseline_difference_ms=float(m['callback_duration_ms'])-baseline if baseline is not None else None,
               signature=json.dumps(sorted((g,p,n) for (g,p),n in sig[f['frame_instance_id']].items())),
               input_message_uid=f['input_message_uid'], output_message_uid=f['output_message_uid'])
    ledger.append(row)
save('frame_analysis.csv', ledger)
bins = []
for a in range(5, 105, 5):
    rr = [r for r in ledger if a <= r['start_s'] < a+5]
    bins.append(dict(start_s=a,end_s=a+5,frames=len(rr),misses=sum(r['miss'] for r in rr),
                     **{k:v for k,v in stats([r['callback_ms'] for r in rr]).items() if k!='count'},
                     first_frame=min(r['frame'] for r in rr),last_frame=max(r['frame'] for r in rr),
                     frame_uids=json.dumps([r['frame_uid'] for r in rr])))
save('five_second_bins.csv', bins)
bursts=[]; active=[]
for r in ledger+[dict(miss=False)]:
    if r['miss']: active.append(r)
    elif active:
        bursts.append(dict(first_frame=active[0]['frame'],last_frame=active[-1]['frame'],
                           frames=len(active),start_s=active[0]['start_s'],
                           min_slack_ms=min(x['slack_ms'] for x in active),
                           frame_uids=json.dumps([x['frame_uid'] for x in active])))
        active=[]
save('consecutive_miss_bursts.csv',bursts)

# Event-triggered callchains bind to actual sleeping episode boundaries, not nearest frames.
samples=read('callstack_samples.csv')
ss=collections.defaultdict(list)
for s in samples:
    if s['source_type']=='sched_switch_event_stack':
        ss[(s['frame_uid'],s['task_id'],s['host_tid'])].append(s)
for key in ss: ss[key].sort(key=lambda x:int(x['timestamp_ns']))
episodes=[]
by_entity=collections.defaultdict(list)
for r in states:
    by_entity[(r['frame_uid'],r['task_id'],r['host_tid'])].append(r)
predecessor={}
for key, rr in by_entity.items():
    rr.sort(key=lambda x:int(x['start_ns']))
    for i,r in enumerate(rr):
        if i and rr[i-1]['state']=='Running' and rr[i-1]['end_ns']==r['start_ns']:
            predecessor[(key,r['state_row_id'])]=rr[i-1]
for r in states:
    if r['state'] not in ['S','D','I']: continue
    key=(r['frame_uid'],r['task_id'],r['host_tid']); seq=ss.get(key,[])
    if not seq: continue
    prev=predecessor.get((key,r['state_row_id']))
    if not prev: continue
    t=int(r['original_start_ns']); times=[int(s['timestamp_ns']) for s in seq]
    # PERF sched_switch stack is collected on-CPU before the tracepoint that
    # terminates residency. Bind only a unique event stack in that residency.
    candidates=seq[bisect.bisect_left(times,int(prev['start_ns'])):bisect.bisect_right(times,t)]
    if len(candidates)!=1: continue
    s=candidates[0]; funcs=[x['function'] for x in json.loads(s['callstack'])]; chain=';'.join(funcs)
    if 'futex_lock_pi' in chain: mechanism='futex_PI_mutex_path'
    elif 'nvhost_syncpt_wait_timeout' in chain: mechanism='nvhost_syncpoint_wait_path'
    elif 'CUDACachingAllocator' in chain and 'futex' in chain: mechanism='CUDA_allocator_mutex_path'
    elif 'cudnn' in chain.lower() and 'futex' in chain: mechanism='cuDNN_mutex_path'
    elif 'EvaluatorManager3Run' in chain and '_M_futex_wait' in chain: mechanism='Evaluator_future_futex_path'
    elif 'futex' in chain: mechanism='other_futex_path'
    else: mechanism='other_observed_path'
    episodes.append(dict(run_id=RUN,frame_uid=r['frame_uid'],frame=int(fm[r['frame_uid']]['frame_index']),
                         task_id=r['task_id'],host_tid=r['host_tid'],utid=r['utid'],state=r['state'],
                         start_boot_ns=r['start_ns'],end_boot_ns=r['end_ns'],original_start_boot_ns=r['original_start_ns'],
                         duration_ms=int(r['duration_ns'])/1e6,state_row_id=r['state_row_id'],
                         sample_id=s['sample_id'],sample_boot_ns=s['timestamp_ns'],
                         sample_boundary_delta_ns=int(s['timestamp_ns'])-t,
                         preceding_running_start_boot_ns=prev['start_ns'],
                         observed_path=mechanism,unwind_error=s['unwind_error'],
                         callstack=s['callstack'],workspace_evidence='E2',method_evidence='E2_with_mechanism_evidence',
                         wait_object_identity=None,attributable_excess_ms=None,
                         interpretation='observed blocking path; object/holder/completion causal chain unresolved'))
save('waiting_episodes_with_event_stacks.csv',episodes)
episode_summary=[]
for key in sorted(set((e['frame'],e['task_id'],e['observed_path']) for e in episodes)):
    rr=[e for e in episodes if (e['frame'],e['task_id'],e['observed_path'])==key]
    episode_summary.append(dict(frame=key[0],task_id=key[1],observed_path=key[2],episodes=len(rr),
                                sleeping_ms=sum(e['duration_ms'] for e in rr),
                                maximum_episode_ms=max(e['duration_ms'] for e in rr),
                                sample_ids=json.dumps([e['sample_id'] for e in rr]),frame_uid=rr[0]['frame_uid']))
save('waiting_path_summary.csv',episode_summary)

# Compare exact measured signatures. A meet does not by itself define a short normal callback.
comparisons=[]
for number in [648,653,659,675,677]:
    r=next(x for x in ledger if x['frame']==number)
    peers=[x for x in ledger if x['signature']==r['signature'] and x['frame']!=number]
    meets=[x for x in peers if not x['miss']]
    ref=min(meets,key=lambda x:abs(x['start_s']-r['start_s'])) if meets else None
    comparisons.append(dict(frame=number,frame_uid=r['frame_uid'],peer_count=len(peers),
                            peer_meet_count=len(meets),control_frame=ref['frame'] if ref else None,
                            control_frame_uid=ref['frame_uid'] if ref else None,
                            callback_ms=r['callback_ms'],control_callback_ms=ref['callback_ms'] if ref else None,
                            baseline_ms=r['baseline_same_signature_median_ms'],
                            baseline_difference_ms=r['baseline_difference_ms'],
                            control_scope='same observed group/model/forward counts; tensor values and scene differ' if ref else 'no exact measured-signature meet control',
                            attributable_latency_ms=None))
save('matched_comparisons.csv',comparisons)

# Normal reference: longest repeated nonempty worker signature with meets and low median.
eligible=[]
for signature, ff in by_signature.items():
    rr=[r for r in ledger if r['frame_uid'] in {x['frame_uid'] for x in ff}]
    if signature and len(rr)>=5 and all(not x['miss'] for x in rr):
        eligible.append((statistics.median(x['callback_ms'] for x in rr),rr))
normal_group=min(eligible,key=lambda x:x[0])[1]
normal=sorted(normal_group,key=lambda x:abs(x['callback_ms']-statistics.median(y['callback_ms'] for y in normal_group)))[0]
case_numbers=[normal['frame'],648,653,659,675,677]
case_rows=[r for r in ledger if r['frame'] in case_numbers]
save('case_callback_states.csv',case_rows)
case_tasks=[]
for t in tasks:
    uid=t['frame_uid']
    if int(fm[uid]['frame_index']) not in case_numbers or (uid,t['task_id']) not in taskstates: continue
    s=taskstates[(uid,t['task_id'])]
    case_tasks.append(dict(frame=int(fm[uid]['frame_index']),frame_uid=uid,task_id=t['task_id'],group_id=t['group_id'],
                           host_tid=t['host_tid'],obstacles=t['obstacle_count'],forward_calls=t['inference_count'],
                           **{k:float(s[k]) for k in ['task_wall_ms','running_ms','runnable_ms','sleeping_ms','uninterruptible_ms','unknown_ms']},
                           queue_lower_ms=float(t['queue_wait_lower_ms']),queue_upper_ms=float(t['queue_wait_upper_ms']),
                           task_begin_event_id=t['task_begin_event_id'],task_end_event_id=t['task_end_event_id']))
save('case_worker_states.csv',case_tasks)
save('F648_thread_state_spans.csv',[r for r in states if fm[r['frame_uid']]['frame_index']=='648'])
dependencies=[]
for t in tasks:
    if int(fm[t['frame_uid']]['frame_index']) not in case_numbers: continue
    ee=[e for e in events[t['frame_instance_id']] if e['task_id']==t['task_id'] and e['phase']=='group_end']
    dependencies.append(dict(frame=int(fm[t['frame_uid']]['frame_index']),frame_uid=t['frame_uid'],task_id=t['task_id'],
                             host_tid=t['host_tid'],submit_begin_raw_ns=t['submit_begin_ns'],task_begin_raw_ns=t['task_begin_ns'],
                             group_end_raw_ns=ee[0]['mono_ns'] if len(ee)==1 else None,
                             group_end_event_id=ee[0]['event_id'] if len(ee)==1 else None,
                             task_end_raw_ns=t['task_end_ns'],future_get_begin_raw_ns=t['future_get_begin_ns'],
                             future_get_end_raw_ns=t['future_get_end_ns'],future_ready_raw_ns=None,
                             get_wall_ms=(int(t['future_get_end_ns'])-int(t['future_get_begin_ns']))/1e6,
                             future_get_end_event_id=t['future_get_end_event_id'],
                             completion_semantics='group_end before packaged-task readiness; task_end after ready, may follow get-return'))
save('case_task_dependencies.csv',dependencies)
major=['proc','container','perception','evaluator','semantic_map_run','thread_pool_run','predictor','Writer']
stage_rows=[]
for s in stages:
    if s['window_status']=='inside' and s['stage_name'] in major:
        stage_rows.append(dict(**s,frame=int(fm[s['frame_uid']]['frame_index'])))
save('major_stage_timings.csv',stage_rows)
stage_states=[]
for r in stage_rows:
    entity=by_entity.get((r['frame_uid'],'',r['host_tid']),[])
    if not entity: continue
    start,end=int(r['start_boot_ns']),int(r['end_boot_ns'])
    sums=collections.Counter()
    for x in entity:
        overlap=max(0,min(end,int(x['end_ns']))-max(start,int(x['start_ns'])))
        if overlap:sums[x['state']]+=overlap
    covered=sum(sums.values())
    stage_states.append(dict(frame=r['frame'],frame_uid=r['frame_uid'],stage_uid=r['stage_uid'],stage=r['stage_name'],
                             host_tid=r['host_tid'],start_raw_ns=r['start_ns'],end_raw_ns=r['end_ns'],
                             start_boot_ns=r['start_boot_ns'],end_boot_ns=r['end_boot_ns'],
                             running_ms=sums['Running']/1e6,runnable_ms=(sums['R']+sums['R+'])/1e6,
                             sleeping_ms=(sums['S']+sums['I'])/1e6,uninterruptible_ms=sums['D']/1e6,
                             unknown_ms=(end-start-covered)/1e6,coverage_ratio=covered/(end-start),
                             event_id_begin=r['event_id_begin'],event_id_end=r['event_id_end']))
save('major_stage_states.csv',stage_states)
case_evidence=[]
for c in case_rows:
    uid=c['frame_uid'];eps=[e for e in episodes if e['frame_uid']==uid]
    paths=sorted(set(e['observed_path'] for e in eps))
    case_evidence.append(dict(run_id=RUN,frame_uid=uid,frame=c['frame'],
        L1_application=f"callback {c['callback_ms']:.6f} ms; dispatch proxy {c['dispatch_proxy_ms']:.6f} ms",
        L2_thread_states=f"running {c['running_ms']:.6f}; runnable {c['runnable_ms']:.6f}; sleeping {c['sleeping_ms']:.6f} ms",
        L3_scheduling='selected raw waking/wakeup and actual resume CPU overlap' if eps else 'not in selected detailed scheduler context export',
        L4_blocking_paths=json.dumps(paths),L4_missing='lock object/holder, exact Future Ready/notification and device completion chain',
        L5_observed_slack_ms=c['slack_ms'],L5_attributable_increment_ms=None,
        workspace_level='E2' if eps else 'E1',method_level='E2_with_mechanism_evidence' if eps else 'E2',
        control='F653 same measured signature meet (long-tail itself)' if c['frame'] in [659,677] else 'F398 different-workload descriptive normal reference',
        normal_control_l3_l4_available=False,clock='RAW duration / validated BOOTTIME states',
        source='data/frame_analysis.csv;data/major_stage_states.csv;data/waiting_episodes_with_event_stacks.csv'))
save('five_layer_case_evidence.csv',case_evidence)
competition=read('cpu_competition.csv'); top=[]
for frame in [648,653,659,675,677]:
    uid=next(r['frame_uid'] for r in ledger if r['frame']==frame)
    c=collections.Counter()
    for r in competition:
        if r['frame_uid']==uid:c[(r['competing_tid'],r['competing_comm'])]+=float(r['competing_running_ms'])
    for (tid,name),ms in c.most_common(8):
        top.append(dict(frame=frame,frame_uid=uid,competing_tid=tid,comm=name,
                        overlapping_thread_cumulative_ms=ms,interpretation='actual resume CPU overlap; parallel waits summed; not causal loss'))
save('cpu_overlap_candidates.csv',top)
downstream=[]
nodes=read('tdfg_nodes.csv')
for f in window:
    rr=[n for n in nodes if n['frame_uid']==f['frame_uid'] and n['module']=='planning' and n['node_type']=='TEMPORAL_CHECKPOINT']
    for n in rr:
        downstream.append(dict(frame=int(f['frame_index']),frame_uid=f['frame_uid'],message_uid=n['message_uid'],
                               planning_event_id=n['event_id'],planning_raw_ns=n['event_time_ns'],
                               writer_enter_raw_ns=f['writer_enter_ns'],writer_done_raw_ns=f['writer_done_ns'],
                               entry_proxy_to_runonce_ms=(int(n['event_time_ns'])-int(f['writer_enter_ns']))/1e6,
                               writer_return_to_runonce_ms=(int(n['event_time_ns'])-int(f['writer_done_ns']))/1e6,
                               actual_wire_publish_ns=None,physical_source_age_ms=None))
save('downstream_runonce.csv',downstream)
intervals=[]
for r in ledger:
    m=metrics[r['frame_uid']]
    intervals.append(dict(frame=r['frame'],frame_uid=r['frame_uid'],input_interval_ms=float(m['input_interval_ms']) if m['input_interval_ms'] and m['interval_window_status']=='inside' else None,
                           output_interval_ms=float(m['output_interval_ms']) if m['output_interval_ms'] and m['interval_window_status']=='inside' else None,
                           stability_span_ms=float(m['stability_span_ms']) if m['stability_span_ms'] and m['stability_window_status']=='inside' else None,
                           interval_next_frame_uid=m['interval_next_frame_uid']))
save('interval_stability.csv',intervals)
anomaly=[r for r in ledger if 90<=r['start_s']<95]
normal_early=[r for r in ledger if 5<=r['start_s']<70]
summary=dict(run_id=RUN,window=[5,105],frames=len(window),full_frames=len(frames),tasks_in_window=len(taskstates),
             callback=stats([r['callback_ms'] for r in ledger]),response=stats([r['response_ms'] for r in ledger]),
             misses=sum(r['miss'] for r in ledger),miss_rate=sum(r['miss'] for r in ledger)/len(ledger),
             anomaly=dict(frames=len(anomaly),misses=sum(r['miss'] for r in anomaly),
                          callback=stats([r['callback_ms'] for r in anomaly]),
                          fraction_of_all_misses=sum(r['miss'] for r in anomaly)/sum(r['miss'] for r in ledger)),
             normal_reference=normal,comparisons=comparisons,consecutive_miss_bursts=bursts,
             early_5_70_callback=stats([r['callback_ms'] for r in normal_early]),
             waiting_episodes=len(episodes),waiting_paths=collections.Counter(e['observed_path'] for e in episodes),
             exact_signature_normal_control_available=False,
             downstream_runonce_matched_frames=len({r['frame_uid'] for r in downstream}),
             downstream_entry_to_runonce_ms=stats([r['entry_proxy_to_runonce_ms'] for r in downstream]),
             intervals={k:stats([r[k] for r in intervals if r[k] is not None]) for k in ['input_interval_ms','output_interval_ms','stability_span_ms']},
             scientific_claim='Instance identity and calibrated thread states localize sustained latency to Evaluator/ThreadPool and observed blocking paths.',
             writing_skill='https://github.com/Adkid-Zephyr/anti-defensive-writing-Skill/blob/main/skills/anti-defensive-writing/SKILL.md')
(OUT/'analysis_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
checks=[dict(name='full_instance_count',passed=len(frames)==1057),dict(name='window_instance_count',passed=len(ledger)==693),
        dict(name='window_misses',passed=sum(r['miss'] for r in ledger)==35),
        dict(name='segment_misses',passed=len(anomaly)==34 and sum(r['miss'] for r in anomaly)==29),
        dict(name='episode_sample_unique_match',passed=len(episodes)>0 and all(int(e['preceding_running_start_boot_ns'])<=int(e['sample_boot_ns'])<=int(e['original_start_boot_ns']) for e in episodes)),
        dict(name='callback_RAW_recompute',passed=all(abs((int(f['proc_exit_ns'])-int(f['proc_enter_ns']))/1e6-float(metrics[f['frame_uid']]['callback_duration_ms']))<1e-8 for f in window)),
        dict(name='no_causal_attribution_fabricated',passed=all(e['attributable_excess_ms'] is None for e in episodes)),
        dict(name='source_ids_preserved',passed=all(isinstance(r['trace_id'],str) and r['event_id_enter'] for r in ledger))]
checks.append(dict(name='stage_state_complete',passed=all(abs(r['unknown_ms'])<1e-6 for r in stage_states)))
(OUT/'validation_results.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in checks),checks=checks),indent=2),encoding='utf-8')
queries={}
for q, rows, source in [('frames',ledger,'frame_instances.csv;frame_timing_metrics.csv;metadata/callback_state_attribution.csv'),
                        ('bins',bins,'data/five_second_bins.csv'),('cases',case_rows,'data/case_callback_states.csv'),
                        ('workers',case_tasks,'data/case_worker_states.csv'),('comparisons',comparisons,'data/matched_comparisons.csv'),
                        ('waits',episode_summary,'data/waiting_path_summary.csv'),('downstream',downstream,'data/downstream_runonce.csv'),
                        ('intervals',intervals,'data/interval_stability.csv')]:
    queries[q]=dict(rows=rows,source=dict(label=f'{RUN} audited {q}',files=[dict(name=source)],
                     filters=['CLOCK_BOOTTIME trace-relative [5,105) s; callbacks fully covered'],
                     caveats=['next Fusion update is an analyst boundary, not a safety contract','Physical sensor age uncalibrated; no GPU timeline'],
                     metricDefinitions=[dict(label='Time (ms)',definition='RAW durations; calibrated BOOTTIME thread-state intersections; strict miss c>d.',componentIds=['report-'+q])],
                     evidenceFlow=[dict(title='Reproducible analysis',detail='scripts/analyze_realtime_report.py reads immutable exports; per-row frame UID and original event IDs retained.')]),
                     methods=[dict(language='python',code='Run python scripts/analyze_realtime_report.py from the parent analysis project. Full transformations are in that source file.')])
snapshot=dict(surface='report',title='Prediction 持续长延迟集中在 Evaluator 的并行任务等待',status='observed',
              buildStatus='creating',generatedAt=datetime.datetime.now(datetime.timezone.utc).isoformat(),filters=[],
              report=dict(asOf='2026-10-20'),queries=queries)
(OUT/'reviewed_snapshot.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:summary[k] for k in ['frames','misses','anomaly','normal_reference','waiting_episodes','waiting_paths']},ensure_ascii=False,indent=2))
