"""Application ledger and windowed Perfetto evidence. IDs remain decimal strings."""
import argparse, bisect, collections, csv, gzip, hashlib, json, math, statistics
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RUN='pw_20261020_141904'
T0=1032146343301613
LO=T0+5_000_000_000
HI=T0+105_000_000_000
RAW='CLOCK_MONOTONIC_RAW'
BOOT='CLOCK_BOOTTIME'
APP=ROOT/'inputs'
PROCESS=f'{RUN}:prediction:942621@103208480'

def read(p):
    with Path(p).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def write(name,rows,fields=()):
    p=ROOT/name;p.parent.mkdir(parents=True,exist_ok=True)
    cols=list(fields)
    for r in rows:
        for k in r:
            if k not in cols:cols.append(k)
    with p.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader()
        for r in rows:w.writerow({k:json.dumps(v,ensure_ascii=False,separators=(',',':')) if isinstance(v,(list,dict)) else v for k,v in r.items()})
def ns(v):return int(v) if v not in (None,'','None') else None
def ms(a,b):return (b-a)/1e6 if a is not None and b is not None else None
def ev(r,worker=False):return f'prediction.942621:{r["event_id"]}'
def stats(v):
    x=np.asarray([a for a in v if a is not None],dtype=float)
    if not len(x):return dict(count=0)
    return dict(count=len(x),min=float(x.min()),mean=float(x.mean()),median=float(np.median(x)),p90=float(np.percentile(x,90)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),max=float(x.max()),std_population=float(x.std()))

class Clock:
    """Piecewise linear offset interpolation; no application or stack offset reuse."""
    def __init__(self):
        src=json.loads((ROOT/'metadata/inventory_clock_snapshots.json').read_text())
        g={}
        for r in src:g.setdefault(r['snapshot_id'],{})[r['clock_name']]=int(r['clock_value'])
        self.rows=[dict(run_id=RUN,snapshot_id=k,raw_ns=v.get('MONOTONIC_RAW'),monotonic_ns=v.get('MONOTONIC'),boottime_ns=v.get('BOOTTIME'),realtime_ns=v.get('REALTIME'),realtime_coarse_ns=v.get('REALTIME_COARSE'),monotonic_coarse_ns=v.get('MONOTONIC_COARSE')) for k,v in sorted(g.items())]
        self.x=[r['raw_ns'] for r in self.rows];self.offset=[r['boottime_ns']-r['raw_ns'] for r in self.rows]
        train=list(range(0,len(self.x),2)); hold=list(range(1,len(self.x)-1,2))
        residual=[]
        for i in hold:
            j=bisect.bisect_left([self.x[t] for t in train],self.x[i]);a,b=train[j-1],train[j]
            pred=self.offset[a]+(self.offset[b]-self.offset[a])*(self.x[i]-self.x[a])/(self.x[b]-self.x[a])
            residual.append(dict(snapshot_id=i,raw_ns=str(self.x[i]),residual_ns=self.offset[i]-pred))
        window_res=[r for r in residual if LO-1e9<=self.convert(int(r['raw_ns']))<=HI+1e9]
        maxerr=max(abs(r['residual_ns']) for r in window_res)
        self.report=dict(run_id=RUN,source_clock=RAW,target_clock=BOOT,method='piecewise_linear_offset_interpolation',formula='boot_ns = raw_ns + interpolate(raw_ns, knots_raw_ns, knots_boot_minus_raw_ns)',parameters=dict(knots_raw_ns=[str(x) for x in self.x],knots_boot_minus_raw_ns=[str(x) for x in self.offset]),snapshot_count=len(self.rows),fit_validation_split='even snapshot indices train; odd indices held out; final interpolation uses all 47 knots',training_samples=len(train),validation_samples=len(hold),validation_residuals=residual,window_validation_samples=len(window_res),window_max_abs_residual_ns=maxerr,global_max_abs_residual_ns=max(abs(r['residual_ns']) for r in residual),residual_tolerance_ns=100000,calibration_pass=maxerr<=100000,valid_raw_range_ns=[str(self.x[0]),str(self.x[-1])],analysis_window_boot_ns=[str(LO),str(HI)],no_extrapolation=True,uncertainty='Held-out residual is measured validation error, not a guaranteed bound; independent event checks required before attribution.',application_event_validation=None,stack_validation=None,source_timestamp_alignment='Not established: sensor header provenance/cross-host clock unknown',stack_conversion='Perfetto perf_sample.ts already normalized BOOTTIME by Trace Processor; no RAW offset applied to samples')
    def convert(self,t):
        if t is None or t<self.x[0] or t>self.x[-1]:return None
        i=min(max(bisect.bisect_right(self.x,t)-1,0),len(self.x)-2)
        a,b=self.x[i],self.x[i+1]
        return t+round(self.offset[i]+(self.offset[i+1]-self.offset[i])*(t-a)/(b-a))

def selected(a,b):
    if a is None or b is None:return 'unknown'
    if LO<=a<=b<=HI:return 'inside'
    if b>LO and a<HI:return 'partial'
    return 'outside'

def phase_pair(rows,a,b):
    aa=[r for r in rows if r['phase']==a];bb=[r for r in rows if r['phase']==b]
    if len(aa)==len(bb)==1 and int(bb[0]['mono_ns'])>=int(aa[0]['mono_ns']):return aa[0],bb[0]
    return None,None

def application():
    c=Clock();write('clock_snapshots.csv',c.rows);dump(ROOT/'clock_alignment.json',c.report)
    events=read(APP/'trace/events/prediction.942621.csv');workers=read(APP/'trace/prediction_worker_events/prediction.942621.csv')
    contexts={p.stem:read(p) for p in (APP/'trace/message_context').glob('*.csv')}
    groups=collections.defaultdict(list)
    for r in events:groups[r['trace_id']].append(r)
    pc=contexts['prediction.942621'];frames=[];stages=[];framebytrace={}; issues=[]
    for trace,rs in groups.items():
        proc='0'
        a,b=phase_pair(rs,'proc_enter','proc_exit')
        if not a or not b:
            issues.append(dict(trace_id=trace,proc_id=proc,reason='callback_missing_duplicate_or_reversed'));continue
        uid=f'{PROCESS}:trace:{trace}:proc:{proc}:event:{a["event_id"]}'
        f=dict(run_id=RUN,frame_uid=uid,processing_instance_uid=uid,trace_id=trace,proc_id=proc,process_instance=PROCESS,host_pid='942621',pid='942621',callback_tid=a['tid'],host_tid=a['tid'],event_id_enter=ev(a),event_id_exit=ev(b),timestamp_domain=RAW,aligned_clock_domain=BOOT)
        for phase in ['proc_enter','proc_exit','writer_enter','writer_done','output_pub']:
            rr=[r for r in rs if r['phase']==phase]
            val=int(rr[0]['mono_ns']) if len(rr)==1 else None
            f[phase+'_ns']=val;f[phase+'_boot_ns']=c.convert(val)
        f['window_status']=selected(f['proc_enter_boot_ns'],f['proc_exit_boot_ns'])
        f['input_message_uid']=None;f['output_message_uid']=None;f['quality_flag']='paired;process_before_snapshot_verified_after_missing'
        frames.append(f);framebytrace[(trace,proc)]=f
        # Pair by exact stage, actual TID, proc and trace. Preserve asynchronous spans.
        ss=collections.defaultdict(list)
        for r in rs:
            ph=r['phase']
            if ph!='writer_enter' and (ph.endswith('_enter') or ph.endswith('_exit')):
                ss[(ph.rsplit('_',1)[0],r['tid'],r['proc_id'])].append(r)
        ss[('Writer',a['tid'],proc)]=[r for r in rs if r['phase'] in ['writer_enter','writer_done']]
        for (name,tid,stageproc),rr in ss.items():
            startphase,endphase=('writer_enter','writer_done') if name=='Writer' else (name+'_enter',name+'_exit')
            stack=[]
            for r in sorted(rr,key=lambda x:(int(x['mono_ns']),int(x['event_id']))):
                if r['phase']==startphase:stack.append(r)
                elif r['phase']==endphase and stack:
                    q=stack.pop();s,e=int(q['mono_ns']),int(r['mono_ns'])
                    stages.append(dict(run_id=RUN,frame_uid=uid,stage_uid=f'{PROCESS}:span:{q["event_id"]}:{r["event_id"]}',stage_name=name,event_id_begin=ev(q),event_id_end=ev(r),start_ns=s,end_ns=e,duration_ms=ms(s,e),host_tid=tid,pid='942621',trace_id=trace,proc_id=stageproc,timestamp_domain=RAW,start_boot_ns=c.convert(s),end_boot_ns=c.convert(e),aligned_clock_domain=BOOT,window_status=selected(c.convert(s),c.convert(e)),quality_flag='reliable_pair'))
                else:issues.append(dict(trace_id=trace,stage=name,event_id=ev(r),reason='unpaired_end'))
            for q in stack:issues.append(dict(trace_id=trace,stage=name,event_id=ev(q),reason='unpaired_begin'))
    frames.sort(key=lambda f:f['proc_enter_ns'])
    for i,f in enumerate(frames,1):f['frame_index']=i
    bytask=collections.defaultdict(list);byactivation=collections.defaultdict(list)
    for r in workers:
        byactivation[r['frame_instance_id']].append(r)
        if r['task_id']!='0':bytask[r['task_id']].append(r)
    act={};identity=[]
    for aid,rr in byactivation.items():
        f=framebytrace.get((rr[0]['trace_id'],'0'))
        if f:
            f['frame_instance_id']=aid;act[aid]=f
    tasks=[]
    for task,rr in bytask.items():
        a,b=phase_pair(rr,'task_begin','task_end')
        if not a:continue
        f=act.get(a['frame_instance_id']);uid=f['frame_uid'] if f else None
        d=dict(run_id=RUN,frame_uid=uid,task_id=task,task_uid=f'{PROCESS}:task:{task}',frame_instance_id=a['frame_instance_id'],trace_id=a['trace_id'],group_id=a['group_id'],pool_level=a['pool_level'],host_tid=a['tid'],host_pid=a['pid'],timestamp_domain=RAW,aligned_clock_domain=BOOT)
        for prefix,ph in [('submit','task_submit'),('task','task'),('future_get','future_get')]:
            x,y=phase_pair(rr,ph+'_begin',ph+'_end')
            for suffix,v in [('begin',x),('end',y)]:
                t=int(v['mono_ns']) if v else None
                d[prefix+'_'+suffix+'_ns']=t;d[prefix+'_'+suffix+'_boot_ns']=c.convert(t);d[prefix+'_'+suffix+'_event_id']=ev(v) if v else None
        d['future_get_tid']=next((r['tid'] for r in rr if r['phase']=='future_get_begin'),None)
        d['obstacle_count']=len({r['obstacle_id'] for r in rr if r['phase']=='group_member'})
        d['inference_count']=sum(r['phase']=='forward_begin' for r in rr)
        d['window_status']=selected(d['task_begin_boot_ns'],d['task_end_boot_ns'])
        d['queue_wait_ms']=None;d['queue_wait_missing_reason']='enqueue instant not observed; bounded by submit interval'
        d['queue_wait_lower_ms']=max(0,d['task_begin_ns']-d['submit_end_ns'])/1e6 if d['submit_end_ns'] else None
        d['queue_wait_upper_ms']=ms(d['submit_begin_ns'],d['task_begin_ns'])
        d['task_end_semantics']='post-packaged_task observation; future ready upper bound, not exact ready'
        d['quality_flag']='paired;routine_id_unavailable';tasks.append(d)
        for src,tgt,rel,tid,aa,bb,evidence in [(uid,d['task_uid'],'FRAME_TASK',a['tid'],d['task_begin_ns'],d['task_end_ns'],d['task_begin_event_id']), (d['task_uid'],f'host:942621:tid:{a["tid"]}','TASK_TID',a['tid'],d['task_begin_ns'],d['task_end_ns'],d['task_begin_event_id']), (d['task_uid'],d['task_uid']+':future','TASK_FUTURE',d['future_get_tid'],d['future_get_begin_ns'],d['future_get_end_ns'],d['future_get_begin_event_id'])]:
            identity.append(dict(run_id=RUN,frame_uid=uid,task_id=task,source_uid=src,target_uid=tgt,relation_type=rel,host_tid=tid,start_ns=aa,end_ns=bb,start_boot_ns=c.convert(aa),end_boot_ns=c.convert(bb),timestamp_domain=RAW,evidence_event_id=evidence,match_method='explicit_frame_instance_task_context;source_restores_context',confidence='verified'))
    for f in frames:identity.append(dict(run_id=RUN,frame_uid=f['frame_uid'],source_uid=f['frame_uid'],target_uid=f'host:942621:tid:{f["callback_tid"]}',relation_type='FRAME_CALLBACK',start_ns=f['proc_enter_ns'],end_ns=f['proc_exit_ns'],start_boot_ns=f['proc_enter_boot_ns'],end_boot_ns=f['proc_exit_boot_ns'],timestamp_domain=RAW,evidence_event_id=f['event_id_enter'],match_method='callback_event_tid',confidence='verified'))
    write('frame_instances.csv',frames);write('worker_tasks.csv',tasks);write('frame_stage_spans.csv',stages);write('identity_edges.csv',identity)
    dump(ROOT/'metadata/application_pairing_issues.json',issues)
    dump(ROOT/'metadata/phase_enumerations.json',dict(application=dict(collections.Counter(r['phase'] for r in events)),worker=dict(collections.Counter(r['phase'] for r in workers)),message_edges={k:dict(collections.Counter(r['edge'] for r in v)) for k,v in contexts.items()}))
    inside=[f for f in frames if f['window_status']=='inside'];p99=np.percentile([ms(f['proc_enter_ns'],f['proc_exit_ns']) for f in inside],99)
    tails=[f for f in inside if ms(f['proc_enter_ns'],f['proc_exit_ns'])>=p99]
    med=statistics.median(ms(f['proc_enter_ns'],f['proc_exit_ns']) for f in inside)
    controls=[]
    for f in tails:
        peers=[q for q in inside if q['frame_uid']!=f['frame_uid'] and sum(t['obstacle_count'] for t in tasks if t['frame_uid']==q['frame_uid'])==sum(t['obstacle_count'] for t in tasks if t['frame_uid']==f['frame_uid']) and sum(t['inference_count'] for t in tasks if t['frame_uid']==q['frame_uid'])==sum(t['inference_count'] for t in tasks if t['frame_uid']==f['frame_uid']) and ms(q['proc_enter_ns'],q['proc_exit_ns'])<p99]
        if peers:controls.append(min(peers,key=lambda q:abs(ms(q['proc_enter_ns'],q['proc_exit_ns'])-med)))
    if not controls:controls=[min(inside,key=lambda f:abs(ms(f['proc_enter_ns'],f['proc_exit_ns'])-med))]
    chosen={f['frame_uid']:dict(f,selection_reason='callback_p99_tail' if f in tails else 'workload_matched_or_median_control') for f in tails+controls}
    windows=[]
    for f in chosen.values():
        windows.append(dict(frame_uid=f['frame_uid'],task_id='',host_tid=f['callback_tid'],start_ns=f['proc_enter_boot_ns'],end_ns=f['proc_exit_boot_ns'],role='callback'))
        for t in tasks:
            if t['frame_uid']==f['frame_uid'] and t['window_status']=='inside':windows.append(dict(frame_uid=f['frame_uid'],task_id=t['task_id'],host_tid=t['host_tid'],start_ns=t['task_begin_boot_ns'],end_ns=t['task_end_boot_ns'],role='task'))
    # Include a deterministic calibration marker sample from all event TIDs; separate from fitting snapshots.
    markers=[dict(host_tid=r['tid'],event_id=ev(r),raw_ns=r['mono_ns'],boot_ns=c.convert(int(r['mono_ns']))) for i,r in enumerate(events) if i%17==0 and c.convert(int(r['mono_ns'])) and LO<=c.convert(int(r['mono_ns']))<=HI]
    write('metadata/calibration_markers.csv',markers)
    req=dict(run_id=RUN,t0=T0,start_ns=LO,end_ns=HI,target_pid=942621,windows=windows,selected_frames=list(chosen.values()),p99_ms=float(p99),tids=sorted({r['tid'] for r in events}|{r['tid'] for r in workers}))
    dump(ROOT/'metadata/extraction_request.json',req)
    print(json.dumps(dict(frames=len(frames),tasks=len(tasks),stages=len(stages),frames_inside=len(inside),tasks_inside=sum(t['window_status']=='inside' for t in tasks),selected_frames=len(chosen),clock_window_max_holdout_error_ns=c.report['window_max_abs_residual_ns'])))

def system():
    folder=APP/'perfetto_export'
    def load(name,keep=None):
        out=[]
        with gzip.open(folder/(name+'.jsonl.gz'),'rt',encoding='utf-8') as f:
            for line in f:
                r=json.loads(line)
                if keep is None or keep(r):out.append(r)
        return out
    status=json.loads((folder/'extraction_status.json').read_text());assert not status['parse_errors']
    assert all(status['counts'][k]>0 for k in ['thread','thread_state','target_sched','perf_sample','stack_callsites','stack_validation','wakeup'])
    aux=APP/'perfetto_context';auxstatus=json.loads((aux/'status.json').read_text());assert auxstatus['returncode']==0 and not auxstatus['parse_errors']
    status['completed_sections_used']=['thread','thread_state','target_sched','perf_sample','stack_callsites','stack_validation','wakeup']
    status['incomplete_job_explanation']='First seven SELECT result sets completed before interrupting expensive CPU join. Original returncode retained. Replacement CPU context and quality/count audit completed independently.'
    status['auxiliary_export']=auxstatus
    threads=load('thread');tmap={r['utid']:r for r in threads};owned=collections.defaultdict(list)
    for t in threads:
        if t['pid']==942621:owned[str(t['tid'])].append(t)
    frames=read(ROOT/'frame_instances.csv');tasks=read(ROOT/'worker_tasks.csv')
    wanted={f['callback_tid'] for f in frames if f['window_status']=='inside'}|{t['host_tid'] for t in tasks if t['window_status']=='inside'}
    wantedutid={t['utid'] for t in threads if str(t['tid']) in wanted and t['pid']==942621}
    states=load('thread_state',lambda r:r['utid'] in wantedutid);sched=load('target_sched');byutid=collections.defaultdict(list);runs=collections.defaultdict(list)
    for r in states:byutid[r['utid']].append(r)
    for r in sched:runs[r['utid']].append(r)
    for rr in byutid.values():rr.sort(key=lambda r:r['ts'])
    for rr in runs.values():rr.sort(key=lambda r:r['ts'])
    runstarts={u:[r['ts'] for r in rr] for u,rr in runs.items()}
    def resume_after(utid,at):
        rr=runs.get(utid,[]);i=bisect.bisect_left(runstarts.get(utid,[]),at)
        return rr[i] if i<len(rr) else None
    starts={u:[r['ts'] for r in rr] for u,rr in byutid.items()}
    def identity(tid,a,b):
        tt=[t for t in owned[str(tid)] if (t['start_ts'] is None or t['start_ts']<=a) and (t['end_ts'] is None or t['end_ts']>=b)]
        return tt[0]['utid'] if len(tt)==1 else None
    clock=json.loads((ROOT/'clock_alignment.json').read_text());markers=read(ROOT/'metadata/calibration_markers.csv');checks=[]
    for m in markers:
        at=int(m['boot_ns']);u=identity(m['host_tid'],at,at)
        rr=runs.get(u,[]);rs=[r['ts'] for r in rr];ix=bisect.bisect_right(rs,at+100000)-1
        near=[r for r in rr[max(0,ix-2):ix+1] if r['dur']>=0]
        dist=min((max(r['ts']-at,at-r['ts']-r['dur'],0) for r in near),default=None)
        checks.append(dict(**m,utid=u,nearest_running_distance_ns=dist,supported_100us=dist is not None and dist<=100000))
    supported=sum(r['supported_100us'] for r in checks)/len(checks)
    sv=load('stack_validation');stacksummary=[]
    for sid in sorted({r['perf_session_id'] for r in sv}):
        rr=[r for r in sv if r['perf_session_id']==sid]
        stacksummary.append(dict(perf_session_id=sid,samples=len(rr),support_2us=sum(r['supported_2us'] for r in rr)/len(rr),support_100us=sum(r['supported_100us'] for r in rr)/len(rr),timestamp_transform='identity: TP normalized BOOTTIME; no application offset'))
    clock['application_event_validation']=dict(samples=len(checks),same_tid_running_support=supported,tolerance_ns=100000,required_support=.95,pass_=supported>=.95,failed_markers=sum(not r['supported_100us'] for r in checks),maximum_distance_ns=max(r['nearest_running_distance_ns'] or 0 for r in checks))
    clock['stack_validation']=stacksummary;clock['system_attribution_pass']=clock['calibration_pass'] and supported>=.95
    clock['stack_attribution_pass']=bool(stacksummary) and all(r['support_100us']>=.95 for r in stacksummary)
    dump(ROOT/'clock_alignment.json',clock);write('metadata/application_clock_validation.csv',checks);write('metadata/stack_clock_validation.csv',sv);write('metadata/linux_threads.csv',[dict(run_id=RUN,**r) for r in threads])
    spans=[];attributions=[];dynamic=[];entities=[]
    for f in frames:
        if f['window_status']=='inside':entities.append(dict(frame_uid=f['frame_uid'],task_id='',host_tid=f['callback_tid'],a=int(f['proc_enter_boot_ns']),b=int(f['proc_exit_boot_ns']),entity_kind='callback',source_a=f['proc_enter_ns'],source_b=f['proc_exit_ns'],node_id=f'{PROCESS}:execution:{f["event_id_enter"]}:{f["event_id_exit"]}:proc'))
    for t in tasks:
        if t['window_status']=='inside':entities.append(dict(frame_uid=t['frame_uid'],task_id=t['task_id'],host_tid=t['host_tid'],a=int(t['task_begin_boot_ns']),b=int(t['task_end_boot_ns']),entity_kind='task',source_a=t['task_begin_ns'],source_b=t['task_end_ns'],node_id=f'{PROCESS}:execution:{t["task_begin_event_id"]}:{t["task_end_event_id"]}:task'))
    def category(s):
        if s=='Running':return 'running_ms'
        if s in ('R','R+'):return 'runnable_ms'
        if s.startswith('D'):return 'uninterruptible_ms'
        if s in ('S','I'):return 'sleeping_ms'
        return 'unknown_ms'
    entity_states={}
    for e in entities:
        a,b=e['a'],e['b'];u=identity(e['host_tid'],a,b);rr=byutid.get(u,[]);ix=max(0,bisect.bisect_right(starts.get(u,[]),a)-1)
        cursor=a;out=[];total=collections.Counter()
        def span(x,y,state,r=None):
            if y<=x:return
            d=dict(run_id=RUN,frame_uid=e['frame_uid'],task_id=e['task_id'],entity_kind=e['entity_kind'],utid=u,host_pid='942621',host_tid=e['host_tid'],cpu_id=r['cpu'] if r else None,state=state,start_ns=x,end_ns=y,duration_ns=y-x,waker_utid=r['waker_utid'] if r else None,state_row_id=r['id'] if r else None,original_start_ns=r['ts'] if r else None,original_duration_ns=r['dur'] if r else None,irq_context=r['irq_context'] if r else None,timestamp_domain=BOOT,aligned_start_ns=x,aligned_end_ns=y,quality_flag='validated_clock_and_observed_state' if clock['system_attribution_pass'] else 'provisional_clock_support',evidence='Perfetto thread_state' if r else 'unobserved_or_unmapped_interval')
            out.append(d);spans.append(d);total[category(state)]+=y-x
        for r in rr[ix:]:
            if r['ts']>=b:break
            if r['dur']<0:continue
            x=max(a,r['ts'],cursor);y=min(b,r['ts']+r['dur'])
            if y<=x:continue
            if x>cursor:span(cursor,x,'Unknown')
            span(x,y,r['state'],r);cursor=y
        if cursor<b:span(cursor,b,'Unknown')
        entity_states[(e['frame_uid'],e['task_id'])]=out
        result=dict(run_id=RUN,frame_uid=e['frame_uid'],task_id=e['task_id'],host_tid=e['host_tid'],utid=u,entity_kind=e['entity_kind'],task_wall_ms=(b-a)/1e6,queue_wait_ms=None,queue_wait_missing_reason='exact enqueue point not recorded; see worker_tasks queue bounds',coverage_ratio=1-total['unknown_ms']/(b-a) if b>a else None,clock_validation_status='passed' if clock['system_attribution_pass'] else 'provisional',identity_status='same PID namespace; before snapshot + unique imported owning process/lifecycle',start_ns=int(e['source_a']),end_ns=int(e['source_b']),timestamp_domain=RAW,start_boot_ns=a,end_boot_ns=b)
        result.update({k:total[k]/1e6 for k in ['running_ms','runnable_ms','sleeping_ms','uninterruptible_ms','unknown_ms']});result['closure_error_ns']=(b-a)-sum(total.values())
        attributions.append(result)
        for s in out:dynamic.append(dict(run_id=RUN,frame_uid=e['frame_uid'],task_id=e['task_id'],routine_id=None,host_pid='942621',host_tid=e['host_tid'],utid=u,cpu_id=s['cpu_id'],start_ns=s['start_ns'],end_ns=s['end_ns'],relation_type='Frame_Task_TID_observed_state' if e['task_id'] else 'Frame_Callback_TID_observed_state',confidence='explicit_context+trace_thread_owner',timestamp_domain=BOOT,state=s['state'],routine_missing_reason='no CRoutine identity events in inputs',cpu_missing_reason=None if s['cpu_id'] is not None else 'CPU not assigned during off-CPU state',state_row_id=s['state_row_id']))
    write('task_state_attribution.csv',[r for r in attributions if r['entity_kind']=='task']);write('metadata/callback_state_attribution.csv',[r for r in attributions if r['entity_kind']=='callback']);write('thread_state_spans.csv',spans);write('dynamic_path_segments.csv',dynamic)
    request=json.loads((ROOT/'metadata/extraction_request.json').read_text());selectedframes={r['frame_uid'] for r in request['selected_frames']};windows=request['windows']
    samples=load('perf_sample');calls={r['id']:r for r in load('stack_callsites')};stackcache={}
    def callstack(cid):
        if cid in stackcache:return stackcache[cid]
        chain=[];seen=set();i=cid
        while i is not None and i in calls and i not in seen:
            seen.add(i);r=calls[i];chain.append(dict(function=r['name'],mapping=r['mapping_name'],rel_pc=r['rel_pc'],frame_id=r['frame_id']));i=r['parent_id']
        stackcache[cid]=chain;return chain
    stackrows=[]
    for s in samples:
        t=tmap.get(s['utid']);tid=str(t['tid']) if t else None
        for w in windows:
            if str(w['host_tid'])==tid and w['start_ns']<=s['ts']<=w['end_ns']:
                chain=callstack(s['callsite_id']);stackrows.append(dict(run_id=RUN,frame_uid=w['frame_uid'],task_id=w['task_id'],sample_id=s['id'],timestamp_ns=s['ts'],host_tid=tid,utid=s['utid'],cpu_id=s['cpu'],cpu_mode=s['cpu_mode'],callstack=chain,top_function=chain[0]['function'] if chain else None,unwind_error=s['unwind_error'],perf_session_id=s['perf_session_id'],source_type={0:'cpu_clock_sample',1:'sched_switch_event_stack',2:'sched_waking_event_stack'}.get(s['perf_session_id'],'unknown'),timestamp_domain=BOOT,aligned_timestamp_ns=s['ts'],quality_flag='validated_same_tid_cpu' if clock['stack_attribution_pass'] else 'provisional_stack_alignment'))
    write('callstack_samples.csv',stackrows,('run_id','frame_uid','task_id','sample_id','timestamp_ns','host_tid','utid','cpu_id','cpu_mode','callstack','top_function','unwind_error'))
    rawwake=load('wakeup');wakebytid=collections.defaultdict(list)
    for w in rawwake:wakebytid[str(w['target_tid'])].append(w)
    wake=[]
    for key,rr in entity_states.items():
        fid,task=key
        if fid not in selectedframes:continue
        for s in rr:
            if s['state'] not in ['R','R+']:continue
            original=ns(s['original_start_ns'])
            original_end=original+ns(s['original_duration_ns']) if original is not None else s['end_ns']
            ww=[w for w in wakebytid[s['host_tid']] if original is not None and original<=w['ts']<=original_end]
            waking=[w for w in ww if w['name']=='sched_waking' and w['ts']==original]
            wakeup=[w for w in ww if w['name'] in ['sched_wakeup','sched_wakeup_new'] and w['success'] in [None,1]]
            # A preemption R/R+ interval is not represented as a wakeup.
            if not waking and s['waker_utid'] is None:continue
            wr=waking[0] if len(waking)==1 else None;wu=wakeup[0] if len(wakeup)==1 else None
            uid=wr['waker_utid'] if wr else s['waker_utid'];t=tmap.get(uid,{})
            run=resume_after(s['utid'],original_end)
            rin=run['ts'] if run else None
            wake.append(dict(run_id=RUN,frame_uid=fid,task_id=task,target_tid=s['host_tid'],target_utid=s['utid'],waker_tid=t.get('tid'),waker_utid=uid,waking_ns=wr['ts'] if wr else None,wakeup_ns=wu['ts'] if wu else None,schedule_in_ns=rin,runnable_wait_ms=ms(wr['ts'],rin) if wr else None,cpu_id=run['cpu'] if run else None,timestamp_domain=BOOT,state_row_id=s['state_row_id'],waking_event_id=wr['id'] if wr else None,wakeup_event_id=wu['id'] if wu else None,irq_context=s['irq_context'],waker_semantics='event context TID; interrupt context may not be causal actor',missing_reason=None if wr and wu and rin else 'one or more raw wake boundaries missing or ambiguous; no synthetic timestamp'))
    write('wakeup_events.csv',wake,('run_id','frame_uid','target_tid','waker_tid','waking_ns','wakeup_ns','schedule_in_ns','runnable_wait_ms','cpu_id'))
    write('metadata/raw_wakeup_events.csv',[dict(run_id=RUN,timestamp_domain=BOOT,**r) for r in rawwake])
    competition=[];overlaps=[];context={}
    with gzip.open(aux/'sched_context.jsonl.gz','rt') as z:
        for line in z:
            r=json.loads(line);context[r['id']]=r
    bycpu=collections.defaultdict(list)
    for r in context.values():bycpu[r['cpu']].append(r)
    for rr in bycpu.values():rr.sort(key=lambda x:x['ts'])
    cpustarts={cpu:[r['ts'] for r in rr] for cpu,rr in bycpu.items()}
    waitingseen=set()
    for (fid,task),ss in entity_states.items():
        if fid not in selectedframes:continue
        for s in ss:
            if s['state'] not in ['R','R+'] or s['state_row_id'] in waitingseen:continue
            waitingseen.add(s['state_row_id']);a=ns(s['original_start_ns']);b=a+ns(s['original_duration_ns'])
            next_run=resume_after(s['utid'],b)
            cpu=next_run['cpu'] if next_run else None
            if cpu is None:continue
            rr=bycpu[cpu];ix=max(0,bisect.bisect_right(cpustarts[cpu],a)-1)
            for r in rr[ix:]:
                if r['ts']>=b:break
                x,y=max(a,r['ts']),min(b,r['ts']+r['dur'])
                if y>x and r['utid']!=s['utid']:overlaps.append(dict(state_id=s['state_row_id'],target_utid=s['utid'],waiting_start=a,waiting_end=b,cpu=cpu,sched_id=r['id'],competing_utid=r['utid'],priority=r['priority'],overlap_start=x,overlap_end=y))
    for r in overlaps:
        target=tmap.get(r['target_utid'],{});other=tmap.get(r['competing_utid'],{})
        for w in windows:
            if str(w['host_tid'])!=str(target.get('tid')):continue
            a=max(r['overlap_start'],w['start_ns']);b=min(r['overlap_end'],w['end_ns'])
            if b<=a:continue
            competition.append(dict(run_id=RUN,frame_uid=w['frame_uid'],task_id=w['task_id'],target_tid=target.get('tid'),target_utid=r['target_utid'],waiting_start_ns=r['waiting_start'],waiting_end_ns=r['waiting_end'],cpu_id=r['cpu'],competing_tid=other.get('tid'),competing_utid=r['competing_utid'],competing_comm=other.get('name'),competing_running_ms=(b-a)/1e6,scheduling_policy=None,priority=r['priority'],start_ns=a,end_ns=b,timestamp_domain=BOOT,state_row_id=r['state_id'],sched_row_id=r['sched_id'],confidence='observed_overlap_only',cpu_choice='actual subsequent resume CPU; not complete affinity-eligible CPU set',missing_reason='capture-time scheduling policy/affinity/quota not fully available; overlap does not establish causation'))
    write('cpu_competition.csv',competition,('run_id','frame_uid','target_tid','waiting_start_ns','waiting_end_ns','cpu_id','competing_tid','competing_comm','competing_running_ms','scheduling_policy','priority'))
    critical=[]
    for e in entities:
        if e['entity_kind']!='callback':continue
        for i,s in enumerate(entity_states[(e['frame_uid'],'')]):
            critical.append(dict(run_id=RUN,frame_uid=e['frame_uid'],path_id=e['frame_uid']+':callback_observed_envelope',segment_order=i,node_id=e['node_id'],state_row_id=s['state_row_id'],task_id='',host_tid=e['host_tid'],segment_type=s['state'],start_ns=s['start_ns'],end_ns=s['end_ns'],duration_ms=s['duration_ns']/1e6,critical_flag=None,dependency_evidence='callback state interval; Future Ready / complete async dependencies absent',timestamp_domain=BOOT,path_status='partial_callback_envelope_not_complete_critical_path',critical_segment_id=e['frame_uid']+':callback_state:'+str(i),missing_reason='full critical membership and quantitative causal contribution unverified'))
    # Preserve candidate task branches separately without summing them with parent waiting.
    for e in entities:
        if e['entity_kind']=='task' and e['frame_uid'] in selectedframes:
            critical.append(dict(run_id=RUN,frame_uid=e['frame_uid'],path_id=e['frame_uid']+':task_branch:'+e['task_id'],segment_order=0,node_id=e['node_id'],task_id=e['task_id'],host_tid=e['host_tid'],segment_type='TASK_BRANCH',start_ns=e['a'],end_ns=e['b'],duration_ms=(e['b']-e['a'])/1e6,critical_flag=None,dependency_evidence='explicit task context plus corresponding future_get; exact ready absent',timestamp_domain=BOOT,path_status='candidate_branch',critical_segment_id=e['frame_uid']+':task_branch:'+e['task_id'],missing_reason='not added to parent wall time; critical status unverified'))
    write('critical_path_segments.csv',critical)
    evidence=[]
    for r in attributions:
        if r['frame_uid'] not in selectedframes:continue
        fid,task=r['frame_uid'],r['task_id'];stk=[s for s in stackrows if s['frame_uid']==fid and s['task_id']==task]
        causes=[]
        if r['runnable_ms']>0:causes.append('observed_runnable_wait;constraint_or_cause_unresolved')
        if r['sleeping_ms']+r['uninterruptible_ms']>0:causes.append('observed_sleep;wait_object_unresolved')
        if r['running_ms']>0:causes.append('observed_cpu_residency;useful_compute_unquantified')
        evidence.append(dict(run_id=RUN,frame_uid=fid,task_id=task,critical_segment_id=fid+':task_branch:'+task if task else None,candidate_path_id=fid+(':task_branch:'+task if task else ':callback_observed_envelope'),suspected_mechanism=causes,running_ms=r['running_ms'],runnable_ms=r['runnable_ms'],blocked_ms=r['sleeping_ms']+r['uninterruptible_ms'],callstack_evidence=[s['sample_id'] for s in stk],wakeup_evidence=[w['state_row_id'] for w in wake if w['frame_uid']==fid and w['task_id']==task],competing_thread_evidence=[x['sched_row_id'] for x in competition if x['frame_uid']==fid and x['task_id']==task],alternative_explanations='normal task synchronization; workload; CPU eligibility; allocator/driver/device; collection overhead; waiting object unknown',evidence_level='E1' if clock['system_attribution_pass'] else 'E0',confidence='observation_only;causal_root_not_confirmed',attributable_latency_ms=None,gpu_timeline_available=False))
    write('root_cause_evidence.csv',evidence)
    dump(ROOT/'metadata/system_summary.json',dict(entities=len(entities),task_rows=sum(e['entity_kind']=='task' for e in entities),state_spans=len(spans),stack_rows=len(stackrows),wakeup_rows=len(wake),competition_rows=len(competition),unknown_entities=sum(r['unknown_ms']>0 for r in attributions),maximum_closure_error_ns=max(abs(r['closure_error_ns']) for r in attributions),application_validation=clock['application_event_validation'],stack_validation=stacksummary,source_extraction=status))
    print(json.dumps(json.loads((ROOT/'metadata/system_summary.json').read_text())))

def graphs_and_metrics():
    c=Clock();frames=read(ROOT/'frame_instances.csv');tasks=read(ROOT/'worker_tasks.csv');stages=read(ROOT/'frame_stage_spans.csv')
    events=read(APP/'trace/events/prediction.942621.csv');workers=read(APP/'trace/prediction_worker_events/prediction.942621.csv')
    contexts={p.stem:read(p) for p in (APP/'trace/message_context').glob('*.csv')}
    pc=contexts['prediction.942621'];fusion=contexts['perception.multi_sensor_fusion.942650'];planning=contexts['planning.942625']
    fi=read(next((APP/'trace/fusion_inputs').glob('*.csv')))
    record={name:[json.loads(line) for line in (APP/f'record_{name}_headers.jsonl').read_text().splitlines()] for name in ['prediction','perception']}
    parents=collections.defaultdict(list)
    for r in fi:parents[r['fusion_trace_id']].append(r)
    def key(r):return (r['channel'],r['trace_id'],r['output_seq'] if r['edge']=='out' else r['input_seq'],r['data_ts_ns'])
    def msguid(k):return RUN+':message:'+':'.join(k)
    recordindex=collections.defaultdict(list)
    for name,rr in record.items():
        for r in rr:
            h=r['header'];k=(r['channel'],str(h.get('trace_id','')),str(h.get('sequence_num','')),str(h.get('lidar_timestamp','0')))
            recordindex[k].append(r)
    upstream=collections.defaultdict(list);output=collections.defaultdict(list);inputindex=collections.defaultdict(list);plans=collections.defaultdict(list)
    for r in fusion:
        if r['edge']=='out':upstream[key(r)].append(r)
    for r in pc:(output if r['edge']=='out' else inputindex)[(r['trace_id'],r['proc_id'])].append(r)
    for r in planning:
        if r['edge']=='in':plans[key(r)].append(r)
    pe=read(APP/'trace/events/planning.942625.csv');runonce=collections.defaultdict(list)
    for r in pe:
        if r['phase']=='runonce_enter':runonce[r['trace_id']].append(r)
    nodes=[];edges=[];metrics=[];recordaudit=[];timingissues=[];nodesseen=set();messages=[]
    def node(uid,f,typ,module,event=None,message=None,**extra):
        if uid in nodesseen:return
        nodesseen.add(uid);r=dict(run_id=RUN,node_uid=uid,frame_uid=f['frame_uid'],node_type=typ,module=module,message_uid=message,source_timestamp_ns=None,event_time_ns=None,event_time_boot_ns=None,timestamp_domain=RAW,quality_flag='verified',pid=None,tid=None,trace_id=f['trace_id'],proc_id=f['proc_id'],event_id=None)
        if event:r.update(event_time_ns=int(event['mono_ns']),event_time_boot_ns=c.convert(int(event['mono_ns'])),pid=event['pid'],tid=event['tid'],event_id=f'{module}.{event["pid"]}:{event["event_id"]}')
        r.update(extra);nodes.append(r)
    def edge(f,a,b,typ,evidence,verified=True,**extra):
        r=dict(run_id=RUN,frame_uid=f['frame_uid'],source_uid=a,target_uid=b,edge_type=typ,message_uid=None,parent_trace_id=None,match_method='explicit_header_trace_seq_data_timestamp',confidence='verified' if verified else 'unverified',verified=verified,evidence_event_id=evidence,lineage_id=f'{f["frame_uid"]}:lineage:{len(edges)}');r.update(extra);edges.append(r)
    fusionout=sorted([r for r in fusion if r['edge']=='out'],key=lambda r:int(r['mono_ns']));fusionpos={r['event_id']:i for i,r in enumerate(fusionout)}
    for f in frames:
        fid=f['frame_uid'];ins=inputindex[(f['trace_id'],f['proc_id'])];outs=output[(f['trace_id'],f['proc_id'])]
        f['input_message_uid']=msguid(key(ins[0])) if len(ins)==1 else None
        f['output_message_uid']=msguid(key(outs[0])) if len(outs)==1 else None
        f['output_message_uids']=[msguid(key(x)) for x in outs];f['output_count']=len(outs)
        node(fid,f,'PROCESSING_INSTANCE','prediction',event_time_ns=ns(f['proc_enter_ns']),event_time_boot_ns=ns(f['proc_enter_boot_ns']),end_ns=ns(f['proc_exit_ns']),end_boot_ns=ns(f['proc_exit_boot_ns']),pid=f['pid'],tid=f['callback_tid'],event_id=f['event_id_enter'])
        parentrows=parents[f['trace_id']];parentids={r['parent_trace_id'] for r in parentrows};expected=ns(ins[0]['parent_trace_count']) if len(ins)==1 else None
        fullparents=bool(expected is not None and len(parentids)==expected)
        f['upstream_parent_count_expected']=expected;f['upstream_parent_count_recovered']=len(parentids)
        f['upstream_parent_status']='complete' if fullparents else 'partial'
        release=None;deadline=None;matchup=[]
        for r in ins:
            k=key(r);m=msguid(k);n=fid+':input:'+r['event_id'];matchup=upstream[k]
            node(n,f,'MESSAGE','perception',r,m,source_timestamp_ns=r['data_ts_ns'],source_clock_domain='SENSOR_HEADER_UNVERIFIED',channel=r['channel'],parent_trace_count=r['parent_trace_count'],parent_trace_ids=sorted(parentids),parent_set_status=f['upstream_parent_status'])
            edge(f,n,fid,'CONSUMES',f['event_id_enter'],message_uid=m,consumption_scope='PredictionEndToEndProc argument; not Reader arrival')
            if len(matchup)==1:
                u=matchup[0];un=fid+':perception_output:'+u['event_id'];node(un,f,'TEMPORAL_CHECKPOINT','perception.multi_sensor_fusion',u,m,source_timestamp_ns=r['data_ts_ns'],channel=u['channel'])
                edge(f,un,n,'DERIVES_FROM',f'perception.multi_sensor_fusion.942650:{u["event_id"]}',message_uid=m)
                release=int(u['mono_ns']);ix=fusionpos[u['event_id']]
                if ix+1<len(fusionout):deadline=int(fusionout[ix+1]['mono_ns'])
                for j,p in enumerate(parentrows):
                    pn=fid+':sensor:'+p['parent_trace_id'];node(pn,f,'DATA_SOURCE',p['sensor'],source_timestamp_ns=None,timestamp_domain='SENSOR_PHYSICAL_TIME_UNAVAILABLE',quality_flag='active_trace_header_parent_only;actual_algorithm_use_unverified',diagnostic_copied_frame_timestamp_ns=p['sensor_ts_ns'],source_time_missing_reason='fusion producer copies trigger lidar_timestamp into every sensor_ts_ns row',parent_trace_id=p['parent_trace_id'],sensor_kind=p['sensor_kind'],is_main_sensor=p['is_main_sensor'])
                    edge(f,pn,un,'DERIVES_FROM','fusion_inputs:trace:'+f['trace_id']+':parent:'+p['parent_trace_id'],verified=False,parent_trace_id=p['parent_trace_id'],match_method='explicit_header_parent_set_from_active_sensor_cache;not_algorithm_consumption',header_parent_relation_verified=True)
        matchedrecords=[];consumers=0;actualrunonce=0
        for r in outs:
            k=key(r);m=msguid(k);n=fid+':output:'+r['event_id'];records=recordindex[k];matchedrecords.extend(records)
            node(n,f,'MESSAGE','prediction',r,m,source_timestamp_ns=r['data_ts_ns'],channel=r['channel'],quality_flag='writer_entry_proxy',record_match_count=len(records))
            edge(f,fid,n,'PRODUCES',f'prediction.942621:{r["event_id"]}',message_uid=m,publication_semantics='Write entry captured retrospectively, not publication completion')
            for pi,p in enumerate(plans[k]):
                pn=fid+':planning_input:'+p['event_id'];node(pn,f,'PROCESSING_INSTANCE','planning',p,m,quality_flag='callback_argument_verified')
                edge(f,n,pn,'CONSUMES',f'planning.942625:{p["event_id"]}',message_uid=m,consumption_scope='PlanningComponent::Proc argument; RunOnce verified separately')
                consumers+=1
                rr=[e for e in runonce[f['trace_id']] if int(e['mono_ns'])>=int(p['mono_ns'])]
                if len(rr)==1:
                    actualrunonce+=1;rn=fid+':planning_runonce:'+rr[0]['event_id'];node(rn,f,'TEMPORAL_CHECKPOINT','planning',rr[0],m)
                    edge(f,pn,rn,'DERIVES_FROM',f'planning.942625:{rr[0]["event_id"]}',consumption_scope='same trace_id unique RunOnce after callback argument; local_view pointer assigned in source')
                if int(p['mono_ns'])<int(r['mono_ns']):timingissues.append(dict(frame_uid=fid,edge='prediction_to_planning',reason='consumer_before_writer_entry',source_event_id=r['event_id'],target_event_id=p['event_id']))
        f['planning_callback_matches']=consumers;f['planning_runonce_matches']=actualrunonce;f['record_output_matches']=len(matchedrecords)
        f['main_message_chain_status']='complete' if len(ins)==1 and len(outs)>=1 and len(matchup)==1 and fullparents and actualrunonce else 'partial'
        f['lineage_status']='partial'
        f['lineage_scope']='main message chain separately audited; active parent cache is not actual fusion input usage; side-input identities unobserved'
        recordaudit.append(dict(run_id=RUN,frame_uid=fid,trace_id=f['trace_id'],window_status=f['window_status'],trace_output_count=len(outs),record_output_count=len(matchedrecords),output_sequence_numbers=[r['output_seq'] for r in outs],record_match_status='matched' if matchedrecords else 'unmatched',reason=None if matchedrecords else 'record_coverage_or_unmatched_identity_pending_audit'))
        agewhy='physical sensor timestamp/cross-host synchronization unverified; fusion audit sensor_ts repeats trigger lidar_timestamp'
        m=dict(run_id=RUN,frame_uid=fid,trace_id=f['trace_id'],proc_id=f['proc_id'],pid=f['pid'],tid=f['callback_tid'],window_status=f['window_status'],response_ms=ms(release,ns(f['proc_exit_ns'])),response_definition='Fusion output_pub proxy to proc_exit',callback_duration_ms=ms(ns(f['proc_enter_ns']),ns(f['proc_exit_ns'])),response_writer_enter_ms=ms(ns(f['proc_enter_ns']),ns(f['writer_enter_ns'])),response_writer_return_ms=ms(ns(f['proc_enter_ns']),ns(f['writer_done_ns'])),input_age_ms=None,output_age_ms=None,planning_age_ms=None,age_missing_reason=agewhy,deadline_ms=ms(release,deadline),deadline_source='analyst_next_real_fusion_output_not_safety_contract' if deadline else None,release_ns=release,deadline_ns=deadline,slack_ms=ms(ns(f['proc_exit_ns']),deadline),deadline_miss=ns(f['proc_exit_ns'])>deadline if deadline else None,deadline_boundary_rule='completion > deadline; equality meets',consistency_span_ms=None,consistency_missing_reason='multiple source labels retained but physical clock semantics unverified',observed_sensor_tag_span_ms=(max(int(p['sensor_ts_ns']) for p in parentrows)-min(int(p['sensor_ts_ns']) for p in parentrows))/1e6 if len(parentids)>1 else None,input_interval_ms=None,output_interval_ms=None,stability_span_ms=None,lineage_status=f['lineage_status'],quality_flag=f['quality_flag'],timestamp_domain=RAW,aligned_clock_domain=BOOT,input_message_uid=f['input_message_uid'],output_message_uid=f['output_message_uid'],event_id_begin=f['event_id_enter'],event_id_end=f['event_id_exit'])
        m['main_message_chain_status']=f['main_message_chain_status']
        m['consistency_missing_reason']='fusion sensor_ts_ns copies a single trigger lidar timestamp; independent actually-used source labels unavailable'
        m['fusion_audit_copied_tag_span_ms']=m.pop('observed_sensor_tag_span_ms')
        for field in ['proc_enter_ns','proc_exit_ns','writer_enter_ns','writer_done_ns','output_pub_ns','proc_enter_boot_ns','proc_exit_boot_ns','writer_enter_boot_ns','writer_done_boot_ns','output_pub_boot_ns']:m[field]=f[field]
        metrics.append(m)
    for i,m in enumerate(metrics):
        if i+1<len(metrics):
            n=metrics[i+1];m['input_interval_ms']=ms(ns(m['proc_enter_ns']),ns(n['proc_enter_ns']));m['output_interval_ms']=ms(ns(m['writer_done_ns']),ns(n['writer_done_ns']))
            m['interval_next_frame_uid']=n['frame_uid'];m['interval_window_status']='inside' if m['window_status']==n['window_status']=='inside' else 'outside_or_boundary'
        vals=[q['output_interval_ms'] for q in metrics[max(0,i-19):i+1] if q.get('output_interval_ms') is not None]
        # Full windows only, 20 intervals = 21 output events. Evaluate in a second pass below.
    for i,m in enumerate(metrics):
        win=metrics[max(0,i-19):i+1];vals=[q['output_interval_ms'] for q in win if q['output_interval_ms'] is not None]
        if len(vals)==20:
            m['stability_span_ms']=max(vals)-min(vals);m['stability_window_intervals']=20
            m['stability_window_status']='inside' if all(q.get('interval_window_status')=='inside' for q in win) else 'outside_or_boundary'
        else:m['stability_missing_reason']='requires 20 consecutive observed output intervals'
    # Reconcile by explicit message identity, not by subtracting aggregate counts.
    predrecordtimes=[int(r['record_time_ns']) for r in record['prediction']]
    seqs=[int(r['header']['sequence_num']) for r in record['prediction']]
    unmatched=[]
    matchedkeys={key(r) for r in pc if r['edge']=='out' and recordindex[key(r)]}
    for a,f in zip(recordaudit,frames):
        if not a['record_output_count']:
            sq=a['output_sequence_numbers']
            a['reason']='before_first_recorded_prediction_sequence' if sq and all(int(s)<min(seqs) for s in sq) else 'after_last_recorded_prediction_sequence' if sq and all(int(s)>max(seqs) for s in sq) else 'inside_record_sequence_extent_unmatched'
    for r in record['prediction']:
        h=r['header'];k=(r['channel'],str(h.get('trace_id','')),str(h.get('sequence_num','')),str(h.get('lidar_timestamp','0')))
        if k not in matchedkeys:unmatched.append(dict(record_time_ns=r['record_time_ns'],trace_id=h.get('trace_id'),sequence=h.get('sequence_num'),reason='record_output_not_in_trace_identity_set'))
    write('frame_instances.csv',frames);write('frame_timing_metrics.csv',metrics);write('tdfg_nodes.csv',nodes);write('tdfg_edges.csv',edges)
    write('metadata/record_trace_matches.csv',recordaudit);dump(ROOT/'metadata/record_only_outputs.json',unmatched);dump(ROOT/'metadata/temporal_order_issues.json',timingissues)
    # Execution graph: interval nodes carry containment; causal edges explicitly identify their endpoint.
    enodes=[];eedges=[];nodebyevent={};validframes={f['frame_uid']:f for f in frames if f['window_status']=='inside'}
    taskmap={t['task_id']:t for t in tasks};activation={f.get('frame_instance_id'):f for f in frames}
    def exnode(f,task,stage,a,b,tid,ea,eb,typ,**extra):
        nid=f'{PROCESS}:execution:{ea}:{eb}:{stage}'
        d=dict(run_id=RUN,node_id=nid,frame_uid=f['frame_uid'],task_id=task,node_type=typ,stage_name=stage,start_ns=a,end_ns=b,duration_ms=ms(a,b),host_tid=tid,start_boot_ns=c.convert(a),end_boot_ns=c.convert(b),timestamp_domain=RAW,aligned_clock_domain=BOOT,event_id_begin=ea,event_id_end=eb,quality_flag='verified_span',**extra)
        enodes.append(d);nodebyevent[(ea,stage)]=nid;return nid
    for s in stages:
        f=validframes.get(s['frame_uid'])
        if f and s['window_status']=='inside':exnode(f,'',s['stage_name'],ns(s['start_ns']),ns(s['end_ns']),s['host_tid'],s['event_id_begin'],s['event_id_end'],'CALLBACK' if s['stage_name']=='proc' else 'PUBLISH_API' if s['stage_name']=='Writer' else 'APPLICATION_STAGE')
    wgroups=collections.defaultdict(list)
    for r in workers:
        f=activation.get(r['frame_instance_id'])
        if not f or f['frame_uid'] not in validframes:continue
        ph=r['phase'];base=ph.rsplit('_',1)[0]
        if ph.endswith(('_begin','_end','_enter','_exit')):
            wgroups[(r['frame_instance_id'],r['task_id'],r['tid'],base,r['model_call_id'],r['inference_id'],r['obstacle_id'],r['lane_sequence_index'])].append(r)
    workerpair=[]
    for k,rr in wgroups.items():
        f=activation[k[0]];base=k[3];endings=('_begin','_end') if any(r['phase']==base+'_begin' for r in rr) else ('_enter','_exit')
        a,b=phase_pair(rr,base+endings[0],base+endings[1])
        if not a:continue
        aa,bb=int(a['mono_ns']),int(b['mono_ns'])
        if selected(c.convert(aa),c.convert(bb))!='inside':continue
        typ='TASK' if base=='task' else 'FUTURE_WAIT_API' if base=='future_get' else 'MODEL_CALL' if base.startswith('model_') else 'INFERENCE_API' if 'inference' in base else 'WORKER_PHASE'
        nid=exnode(f,k[1],base,aa,bb,k[2],ev(a),ev(b),typ,model_call_id=k[4],inference_id=k[5],obstacle_id=k[6])
        workerpair.append((nid,k,aa,bb,a,b))
    tasknodes={};futurenodes={};submitnodes={};groupnodes={}
    for nid,k,a,b,ea,eb in workerpair:
        if k[3]=='task':tasknodes[k[1]]=nid
        elif k[3]=='future_get':futurenodes[k[1]]=nid
        elif k[3]=='task_submit':submitnodes[k[1]]=nid
        elif k[3]=='group':groupnodes[k[1]]=nid
    for t in tasks:
        task=t['task_id'];fid=t['frame_uid']
        if fid not in validframes:continue
        for a,b,typ,endpoint in [(submitnodes.get(task),tasknodes.get(task),'SPAWN','submit_begin -> task_begin; enqueue instant bracketed'),(groupnodes.get(task),futurenodes.get(task),'JOIN','group_end -> future_get_end; NOT task_end -> future_get_begin')]:
            if a and b:eedges.append(dict(run_id=RUN,frame_uid=fid,task_id=task,source_node_id=a,target_node_id=b,edge_type=typ,evidence_event_id=t['task_begin_event_id'] if typ=='SPAWN' else t['future_get_end_event_id'],confidence='source_supported',verified=True,endpoint_semantics=endpoint))
    # Proven same-thread serial order only between disjoint top-level task model calls.
    grouped=collections.defaultdict(list)
    for n in enodes:
        if n['node_type']=='MODEL_CALL':grouped[(n['frame_uid'],n['task_id'],n['host_tid'])].append(n)
    for rr in grouped.values():
        rr.sort(key=lambda n:n['start_ns'])
        for a,b in zip(rr,rr[1:]):
            if a['end_ns']<=b['start_ns']:eedges.append(dict(run_id=RUN,frame_uid=a['frame_uid'],task_id=a['task_id'],source_node_id=a['node_id'],target_node_id=b['node_id'],edge_type='SEQUENCE',evidence_event_id=[a['event_id_end'],b['event_id_begin']],confidence='same_task_same_tid_program_order',verified=True,endpoint_semantics='end -> begin'))
    # Explicit identity association is kept distinct from causal precedence.
    callbacknodes={n['frame_uid']:n for n in enodes if n['node_type']=='CALLBACK'}
    for n in enodes:
        cb=callbacknodes.get(n['frame_uid'])
        if cb and n['node_id']!=cb['node_id']:
            eedges.append(dict(run_id=RUN,frame_uid=n['frame_uid'],task_id=n['task_id'],source_node_id=cb['node_id'],target_node_id=n['node_id'],edge_type='FRAME_ASSOCIATION',evidence_event_id=n['event_id_begin'],confidence='explicit_trace_or_frame_instance_context',verified=True,endpoint_semantics='identity membership only; not causal precedence',causal_precedence=False))
    for task,nid in futurenodes.items():
        n=next(n for n in enodes if n['node_id']==nid)
        resume=dict(n,node_id=nid+':return_checkpoint',node_type='RESUME_CHECKPOINT',stage_name='future_get_return',start_ns=n['end_ns'],start_boot_ns=n['end_boot_ns'],duration_ms=0,event_id_begin=n['event_id_end'])
        enodes.append(resume)
        eedges.append(dict(run_id=RUN,frame_uid=n['frame_uid'],task_id=task,source_node_id=nid,target_node_id=resume['node_id'],edge_type='RESUME',evidence_event_id=n['event_id_end'],confidence='observed_future_get_scope_return',verified=True,endpoint_semantics='future_get_end -> scope-return checkpoint; not kernel wakeup'))
    write('execution_nodes.csv',enodes);write('execution_edges.csv',eedges)
    return frames,tasks,metrics

def finalize():
    frames,tasks,metrics=graphs_and_metrics()
    inventory_path=ROOT.parent/'analysis_inventory/apollo_prediction_data_inventory.json'
    inv=json.loads(inventory_path.read_text(encoding='utf-8'));clock=json.loads((ROOT/'clock_alignment.json').read_text());env=inv['environment']
    inside=[r for r in metrics if r['window_status']=='inside'];allstats=stats(r['callback_duration_ms'] for r in metrics);instats=stats(r['callback_duration_ms'] for r in inside)
    evaluable=[r for r in inside if r['slack_ms'] is not None]
    summary=dict(run_id=RUN,trace_origin_boot_ns=str(T0),analysis_window_relative_s=[5,105],analysis_window_boot_ns=[str(LO),str(HI)],callback_total=len(frames),task_total=len(tasks),callback_inside=len(inside),task_inside=sum(t['window_status']=='inside' for t in tasks),callback_statistics_window_ms=instats,callback_statistics_full_capture_ms=allstats,release_response_window_ms=stats(r['response_ms'] for r in inside),writer_return_window_ms=stats(r['response_writer_return_ms'] for r in inside),input_interval_window_ms=stats(r['input_interval_ms'] for r in inside if r.get('interval_window_status')=='inside'),writer_return_interval_window_ms=stats(r['output_interval_ms'] for r in inside if r.get('interval_window_status')=='inside'),output_stability_20_intervals_ms=stats(r['stability_span_ms'] for r in inside if r.get('stability_window_status')=='inside'),lineage_main_chain_complete_window=sum(r['main_message_chain_status']=='complete' for r in inside),full_physical_lineage_complete_window=0,freshness_computable_window=0,consistency_computable_window=0,stability_computable_window=sum(r['stability_span_ms'] is not None and r.get('stability_window_status')=='inside' for r in inside),deadline_policy='analyst_next_real_fusion_output_not_safety_contract',deadline_evaluable_window=len(evaluable),deadline_miss_window=sum(r['deadline_miss'] for r in evaluable),deadline_miss_rate_window=sum(r['deadline_miss'] for r in evaluable)/len(evaluable) if evaluable else None,formal_safety_deadline=None,root_cause_status='observational candidates only; no causal attribution or intervention performed')
    if (ROOT/'metadata/system_summary.json').exists():summary['system']=json.loads((ROOT/'metadata/system_summary.json').read_text())
    dump(ROOT/'metadata/global_summary.json',summary)
    provenance=dict(inventory_path=str(inventory_path),inventory_sha256=hashlib.sha256(inventory_path.read_bytes()).hexdigest(),environment=env,original_trace_quality=inv['perfetto_data']['trace_quality'],original_trace_range=inv['perfetto_data']['trace_time_range'],original_scheduling_summary=inv['perfetto_data']['scheduling_events']['sched_summary'])
    dump(ROOT/'metadata/inventory_evidence_excerpt.json',provenance)
    session=json.loads((APP/'session/session.json').read_text());manifest=dict(run_id=RUN,apollo_version=env['apollo_version'],apollo_capture_commit=None,apollo_source_snapshot_status='current deployed files read after capture; capture-time binary/source equivalence not proven',carla_version='0.9.15 (user environment)',carla_scene=env['carla_map'],carla_actors_weather=None,prediction_pid='942621',process_starttime_ticks='103208480',process_uid=PROCESS,host='nivadia-desktop',kernel=env['kernel_at_capture'],jetson_power_mode=None,power_mode_missing_reason='not recorded at capture; current setting would not prove historical setting',capture_tool_version=env['capture_tool_metadata'],collection_status=session['status'],perfetto_status='FAILED_OR_INCOMPLETE',experiment_start_raw_ns=str(session['joint_ready_raw_ns']),experiment_stop_requested_raw_ns=str(session['stop_requested_raw_ns']),analysis_window=dict(origin='Trace Processor trace_bounds.start_ts',origin_boot_ns=str(T0),relative_seconds=[5,105],start_boot_ns=str(LO),end_boot_ns=str(HI),wall_time_beijing_approx=['2026-10-20T14:19:10.582+08:00','2026-10-20T14:20:50.582+08:00'],interval_convention='start inclusive, end exclusive; complete entities require begin>=start and end<=end',raw_to_boot_calibration='clock_alignment.json'),original_application_path='/media/ssd/apollo10/data/scb_data/'+RUN,original_perfetto_path='/media/ssd/hjh/prediction_worker_trace/sessions/'+RUN,inventory=str(inventory_path),source_manifest='inputs/source_manifest.json (local evidence cache; excluded from Git)',retention_policy='All 1057 callback and 1929 started-task identity rows retained. System attribution/execution graph restricted to complete window entities. Selected-frame P5 evidence is explicitly sampled.',metric_definitions=dict(callback_duration_ms='proc_exit - proc_enter, native RAW',response_ms='proc_exit - matching full Fusion output_pub proxy, native RAW',deadline_ms='next actual Fusion output_pub - current Fusion output_pub, analyst-selected update boundary',slack_ms='next actual Fusion output_pub - proc_exit; negative means miss; equality meets',output_pub='writer entry proxy; not writer return or actual consumer receipt',output_interval_ms='next writer_done - current writer_done, RAW; next event must also be window-complete for window statistics',stability_span_ms='max(output intervals)-min(output intervals) over 20 consecutive intervals; no formal I or theta; a constant I cancels in the span',sleeping_ms='interruptible S or idle I; excludes D',uninterruptible_ms='D and D flags; disjoint from sleeping',unknown_ms='event gaps, unmapped TID, open states or unsupported states',running_ms='CPU residency, not proof of useful computation',queue_wait_ms='unavailable exact point; lower/upper bounds retained separately'),evidence_scale=dict(E0='coverage/alignment insufficient',E1='observed fact',E2='mechanism supported but chain incomplete',E3='closed causal chain or controlled intervention',E4='not defined in workspace scale; never emitted'),no_original_data_modified=True,no_interventions=True)
    dump(ROOT/'run_manifest.json',manifest)
    coverage=[];quality=[]
    for name,path,field,domain in [('application_prediction',APP/'trace/events/prediction.942621.csv','mono_ns',RAW),('worker_events',APP/'trace/prediction_worker_events/prediction.942621.csv','mono_ns',RAW),('record_message_index',APP/'record_message_index.csv','record_time_ns','CLOCK_REALTIME_recorder_host')]:
        rr=read(path);tt=[int(r[field]) for r in rr if r.get(field)]
        # Record recorder-host epoch conversion uses explicit REALTIME/BOOTTIME snapshots.
        if domain==RAW:
            cc=Clock();aligned=[cc.convert(t) for t in tt];count=sum(x is not None and LO<=x<HI for x in aligned)
        else:
            cc=Clock();real=[r['realtime_ns'] for r in cc.rows];offs=[r['boottime_ns']-r['realtime_ns'] for r in cc.rows]
            def r2b(t):
                if t<real[0] or t>real[-1]:return None
                i=min(max(bisect.bisect_right(real,t)-1,0),len(real)-2)
                return t+round(offs[i]+(offs[i+1]-offs[i])*(t-real[i])/(real[i+1]-real[i]))
            aligned=[r2b(t) for t in tt];count=sum(x is not None and LO<=x<HI for x in aligned)
        valid=[t for t in aligned if t is not None]
        coverage.append(dict(run_id=RUN,source=name,start_ns=min(tt),end_ns=max(tt),timestamp_domain=domain,start_boot_ns=min(valid),end_boot_ns=max(valid),effective_window_start_boot_ns=max(LO,min(valid)),effective_window_end_boot_ns=min(HI,max(valid)),event_count_total=len(rr),event_count_window=count,interruption_known=None,validity='window_observed;per_entity_checks_required',quality_note='Record receiver timestamp, not sensor age' if domain!=RAW else 'Native events calibrated with current-run snapshots'))
        quality.append(dict(run_id=RUN,source=name,lost_events=None,unclosed_events=0 if name=='application_prediction' else None,begin_end_pairs=len(frames) if name=='application_prediction' else len(tasks) if name=='worker_events' else None,stack_unwind_errors=None,parse_errors=0,reason='Application health originally unverified; pairing and current-run clock checks retained independently' if domain==RAW else 'Record starts later; missing matches not interpreted as loss',quality_level='E1'))
    ps=inv['perfetto_data'];sched=ps['scheduling_events'];rawsys=APP/'perfetto_export'
    for name,lo,hi,num in [('Perfetto_sched_systemwide',int(sched['sched_summary']['first_ts_ns']),int(sched['sched_summary']['last_closed_end_ns']),sched['sched_summary']['records'])]+[(f'perf_stack_source_{r["perf_session_id"]}',int(r['first_ts_ns']),int(r['last_ts_ns']),r['samples']) for r in ps['cpu_stack_availability']['sources']]:
        coverage.append(dict(run_id=RUN,source=name,start_ns=lo,end_ns=hi,timestamp_domain=BOOT,start_boot_ns=lo,end_boot_ns=hi,effective_window_start_boot_ns=max(LO,lo),effective_window_end_boot_ns=min(HI,hi),event_count_total=num,event_count_window=None,interruption_known='perf_guardrail_stop' if 'stack' in name else None,validity='selected_window_within_global_observed_range',quality_note='Global first/last observations do not prove every thread/event is covered; window counts in SQL audit when available'))
        quality.append(dict(run_id=RUN,source=name,lost_events=None,unclosed_events=sched['sched_summary']['open'] if 'sched_systemwide' in name else None,begin_end_pairs=None,stack_unwind_errors=next((r['unwind_errors'] for r in ps['cpu_stack_availability']['sources'] if name.endswith('_'+str(r['perf_session_id']))),None),parse_errors=0,reason='Global capture FAILED_OR_INCOMPLETE retained; perf guardrails occur after selected window; unwind errors can occur inside window',quality_level='E1_window_conditional'))
    with gzip.open(APP/'perfetto_context/counts.jsonl.gz','rt') as z:windowcounts={r['source']:r for r in (json.loads(line) for line in z)}
    for r in coverage:
        key='sched_systemwide' if r['source']=='Perfetto_sched_systemwide' else r['source'].replace('perf_stack_source_','perf_source_')
        if key in windowcounts:
            k=windowcounts[key];r['event_count_window']=k['count'];r['window_first_event_boot_ns']=k['first_ts'];r['window_last_event_boot_ns']=k['last_ts']
    for r in quality:
        key=r['source'].replace('perf_stack_source_','perf_source_')
        if key in windowcounts:r['stack_unwind_errors_window']=windowcounts[key].get('unwind_errors')
    write('trace_coverage.csv',coverage);write('trace_quality.csv',quality)
    write('intervention_runs.csv',[],['run_id','experiment_group','intervention_type','modified_parameters','cpu_affinity','scheduling_policy','priority','worker_count','workload_config','power_mode','replicate_id','missing_reason'])
    write('before_after_metrics.csv',[],['run_id','experiment_group','frame_count','mean_response_ms','p95_ms','p99_ms','miss_rate','critical_runnable_ms','critical_blocked_ms','output_age_ms','drop_rate','cpu_usage','gpu_usage','missing_reason'])
    dump(ROOT/'metadata/p6_availability.json',dict(status='not_performed',reason='No optimization intervention or repeated experiment in current data. Both requested P6 CSVs intentionally contain headers only; no synthetic baseline/after rows.'))
    (ROOT/'.gitignore').write_text('inputs/\ninputs_export.tar.gz\n**/__pycache__/\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='system'},ensure_ascii=False))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('step',choices=['application','system','final']);args=ap.parse_args()
    if args.step=='application':application()
    elif args.step=='system':system()
    else:finalize()
