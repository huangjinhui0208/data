"""Prediction adapter reusing reusable_scripts/perf parser and window reader."""
import importlib.util
import csv
import json
import bisect
import statistics
import hashlib
from pathlib import Path
from collections import defaultdict, Counter

ROOT=Path(__file__).resolve().parents[3]
RUN=Path(__file__).resolve().parents[2]
OUT=RUN/'perf和stack数据分析'
PERF=RUN/'prediction_run_010'
DATA=RUN/'打点数据分析/lidar_prediction_reanalysis'
spec=importlib.util.spec_from_file_location('shared_perf',ROOT/'reusable_scripts/perf/generate_p4_perf_analysis.py')
p=importlib.util.module_from_spec(spec); spec.loader.exec_module(p)

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))

def write(path,rows):
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader();w.writerows(rows)

def main():
    OUT.mkdir(exist_ok=True)
    frames=read(DATA/'execution_tail_frames.csv')
    instances=read(DATA/'prediction_node_instances.csv')
    frameids={r['frame_index'] for r in frames}
    target_instances=[r for r in instances if r['frame_index'] in frameids]
    tids={int(r['tid']) for r in target_instances}
    mapping=p.parse_status_snapshot(PERF/'prediction_proc_status_before.txt')
    lwps=p.parse_ps_lwps(PERF/'prediction_ps_threads_before.txt')
    assert tids<=lwps and tids<=mapping.keys()
    write(OUT/'tid_mapping.csv',[{'tid':tid,'snapshot':str(mapping[tid]),'host_ps_verified':True,'evidence':'before snapshot only; no after snapshot available'} for tid in sorted(tids)])
    raw=PERF/'perf_sched_script.txt'
    start=min(int(r['prediction_enter_ns']) for r in frames)/1e9-2
    end=max(int(r['prediction_exit_ns']) for r in frames)/1e9+13
    compact=OUT/'sched_relevant_calibration_context.txt'
    # Reuse time-seeking reader; retain all target thread switches, wakes and migrations.
    counts=Counter(); previous=None; reversals=0
    print('Reading scheduler range',start,end,flush=True)
    cache_meta=OUT/'sched_cache_provenance.json'
    signature={'source':str(raw),'size':raw.stat().st_size,'mtime_ns':raw.stat().st_mtime_ns,'parser_sha256':hashlib.sha256(Path(p.__file__).read_bytes()).hexdigest()}
    if not compact.exists() or not cache_meta.exists() or json.loads(cache_meta.read_text())!=signature:
      with compact.open('w',encoding='utf-8') as f:
        for ev in p.iter_sched_range(raw,start,end):
            if previous is not None and ev['time']<previous: reversals+=1
            previous=ev['time']
            if ev.get('prev_pid') in tids or ev.get('next_pid') in tids or ev.get('pid') in tids:
                f.write(ev['raw_event']+'\n'); counts[ev['event']]+=1
        cache_meta.write_text(json.dumps(signature,indent=2),encoding='utf-8')
    print('Filtered',dict(counts),'reversals',reversals,flush=True)
    # Use original reader for reconstruction, but point its iterator at retained events.
    retained=[p.parse_sched_line(line) for line in compact.read_text(encoding='utf-8').splitlines()]
    assert all(retained)
    reversals=sum(a['time']>b['time'] for a,b in zip(retained,retained[1:]))
    assert reversals==0
    original_iterator=p.iter_sched_range
    p.iter_sched_range=lambda path,a,b:(ev for ev in retained if a<=ev['time']<=b)
    sched=p.scan_scheduler(compact,tids,start,end)
    p.iter_sched_range=original_iterator
    runs=sched['target_runs']
    assert all(a['end']<=b['start'] for rr in runs.values() for a,b in zip(rr,rr[1:])), 'Overlapping scheduler runs: inspect missing switch events'
    # Trace timestamps must independently land in same-TID CPU-resident intervals.
    def fit(points,limit):
        endpoints=[]
        for i,(tid,t,cpu) in enumerate(points):
            for run in runs.get(tid,[]):
                if cpu is not None and run['cpu']!=cpu:continue
                lo=max(-limit,run['start']-t);hi=min(limit,run['end']-t)
                if lo<=hi:endpoints.extend([(lo,1,i),(hi,-1,i)])
        active=Counter();n=0;best=-1;offset=0
        for v,delta,i in sorted(endpoints,key=lambda x:(x[0],-x[1])):
            before=active[i];active[i]+=delta
            n+=int(before==0 and active[i]>0)-int(before>0 and active[i]==0)
            if n>best:best=n;offset=v
        return offset,best/len(points)
    points=[(int(r['tid']),int(r[k])/1e9,None) for r in target_instances for k in ('enter_ns','exit_ns')]
    trace_offset,trace_support=fit(points,1.)
    print('Trace best alignment',trace_offset,trace_support,flush=True)
    markers=[]
    for r in target_instances:
        for key in ('enter_ns','exit_ns'):
            t=int(r[key])/1e9+trace_offset; tid=int(r['tid'])
            ok=any(x['start']-2e-6<=t<=x['end']+2e-6 for x in runs.get(tid,[]))
            markers.append({'frame_index':r['frame_index'],'node':r['node'],'tid':tid,'marker':key,'trace_ns':r[key],'on_cpu_after_alignment':ok})
    write(OUT/'trace_sched_alignment.csv',markers)
    trace_support=sum(r['on_cpu_after_alignment'] for r in markers)/len(markers)
    print('Trace same-TID support after alignment',trace_support,flush=True)
    samples=p.parse_stack_samples(PERF/'perf_cpu_stack_script.txt')
    command=(PERF/'perf_cpu_stack_record_command.txt').read_text()
    assert '--clockid CLOCK_MONOTONIC' in command
    # Find feasible sched-minus-stack offsets using same TID AND same CPU.
    calibration=[s for s in samples if s['tid'] in tids and start+12<s['time_ns']/1e9<end-.5]
    endpoints=[]
    for i,s in enumerate(calibration):
        t=s['time_ns']/1e9
        for r in runs.get(s['tid'],[]):
            if r['cpu']!=s['cpu']:continue
            lo=max(-12.,r['start']-t);hi=min(12.,r['end']-t)
            if lo<=hi: endpoints.extend([(lo,1,i),(hi,-1,i)])
    active=Counter();n=0;best=-1;offset=None
    for v,delta,i in sorted(endpoints,key=lambda x:(x[0],-x[1])):
        before=active[i];active[i]+=delta
        n+=int(before==0 and active[i]>0)-int(before>0 and active[i]==0)
        if n>best:best=n;offset=v
    assert calibration and offset is not None
    support=best/len(calibration)
    bins=[]
    training=calibration[::2]
    for second in sorted({int(s['time_ns']/1e9) for s in training}):
        ss=[s for s in training if int(s['time_ns']/1e9)==second]
        off,sup=fit([(s['tid'],s['time_ns']/1e9,s['cpu']) for s in ss],12)
        bins.append({'time':statistics.mean(s['time_ns']/1e9 for s in ss),'offset':off,'support':sup,'n':len(ss)})
    print('Stack offset by second',bins,flush=True)
    ref=statistics.mean(b['time'] for b in bins)
    meanoff=statistics.mean(b['offset'] for b in bins)
    slope=sum((b['time']-ref)*(b['offset']-meanoff) for b in bins)/sum((b['time']-ref)**2 for b in bins)
    intercept,train_support=fit([(s['tid'],s['time_ns']/1e9+slope*(s['time_ns']/1e9-ref),s['cpu']) for s in training],12)
    def stack_aligned(s):
        t=s['time_ns']/1e9
        return t+intercept+slope*(t-ref)
    heldout=calibration[1::2]
    support=sum(any(r['cpu']==s['cpu'] and r['start']-2e-6<=stack_aligned(s)<=r['end']+2e-6 for r in runs[s['tid']]) for s in heldout)/len(heldout)
    evidence={'trace_to_sched_offset_s':trace_offset,'trace_marker_support':trace_support,
              'constant_offset_candidate_s_not_used':offset,'stack_calibration_samples':len(calibration),
              'same_tid_cpu_heldout_support':support,'training_support':train_support,'stack_offset_intercept_s':intercept,'stack_offset_slope':slope,'stack_reference_s':ref,
              'method':'time-varying affine stack offset: fit using alternate samples, validate remaining samples on same TID and CPU; trace markers independently checked on CPU',
              'scan_start_s':start,'scan_end_s':end,'time_reversals':reversals,
              'limitation':'Empirical alignment; 2us tolerance for trace-vs-sched text rounding. Before-only TID identity.'}
    (OUT/'clock_calibration.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    write(OUT/'stack_alignment_validation.csv',[{'stack_time_ns':s['time_ns'],'tid':s['tid'],'cpu':s['cpu'],'aligned_sched_s':stack_aligned(s),'split':'training' if i%2==0 else 'heldout','same_tid_cpu_match':any(r['cpu']==s['cpu'] and r['start']-2e-6<=stack_aligned(s)<=r['end']+2e-6 for r in runs[s['tid']])} for i,s in enumerate(calibration)])
    print('Calibration',evidence,flush=True)
    assert trace_support>=.95 and support>=.95, 'Clock alignment did not pass: do not extract as verified'
    summaries=[]
    for f in frames:
        dest=OUT/('F'+f['frame_index'].zfill(4));dest.mkdir(exist_ok=True)
        a=int(f['prediction_enter_ns'])/1e9+trace_offset;b=int(f['prediction_exit_ns'])/1e9+trace_offset
        callback=int(f['callback_tid'])
        fi=[r for r in target_instances if r['frame_index']==f['frame_index']]
        ftids={int(r['tid']) for r in fi}
        write(dest/'trace_node_instances.csv',fi)
        write(dest/'frame.csv',[f])
        (dest/'extract_window.json').write_text(json.dumps({'sched_start_s':a,'sched_end_s':b,'sched_context_padding_s':.1,'frame_tids':sorted(ftids),'trace_to_sched_offset_s':trace_offset},indent=2),encoding='utf-8')
        # Save all target-TID events with 100ms pre/post context, in source order.
        relevant=[e for e in retained if a-.1<=e['time']<=b+.1 and (e.get('prev_pid') in ftids or e.get('next_pid') in ftids or e.get('pid') in ftids)]
        (dest/'sched_all_frame_tids_context.txt').write_text('\n'.join(e['raw_event'] for e in relevant)+'\n',encoding='utf-8')
        callback_events=[e for e in relevant if callback in (e.get('prev_pid'),e.get('next_pid'),e.get('pid'))]
        (dest/'sched_callback_tid_context.txt').write_text('\n'.join(e['raw_event'] for e in callback_events)+'\n',encoding='utf-8')
        matched=[]
        for s in samples:
            t=stack_aligned(s)
            if not (a<=t<=b and s['tid'] in ftids):continue
            nodes=sorted({r['node'] for r in fi if int(r['tid'])==s['tid'] and int(r['enter_ns'])/1e9+trace_offset<=t<=int(r['exit_ns'])/1e9+trace_offset})
            matched.append((s,t,nodes))
        (dest/'cpu_stack_all_frame_tids.txt').write_text(''.join(s['raw'] for s,t,n in matched),encoding='utf-8')
        (dest/'cpu_stack_callback_tid.txt').write_text(''.join(s['raw'] for s,t,n in matched if s['tid']==callback),encoding='utf-8')
        write(dest/'cpu_stack_samples.csv',[{'tid':s['tid'],'cpu':s['cpu'],'stack_time_ns':s['time_ns'],'aligned_sched_s':t,'active_trace_nodes':';'.join(n),'leaf':p.normalized_leaf(s)[0],'dso':p.normalized_leaf(s)[1],'callchain':p.folded_callchain(s)} for s,t,n in matched])
        result=p.classify_target_window(callback,a,b,sched)
        assert all(result[k]>=-1e-9 for k in ('running_s','blocked_s','runnable_s','unknown_s'))
        assert abs(sum(result[k] for k in ('running_s','blocked_s','runnable_s','unknown_s'))*1000-float(f['execution_ms']))<.001
        summaries.append({'frame_index':f['frame_index'],'input_seq':f['input_perception_seq'],'callback_tid':callback,
                          'execution_ms':f['execution_ms'],'callback_running_ms':result['running_s']*1000,
                          'callback_blocked_ms':result['blocked_s']*1000,'callback_runnable_ms':result['runnable_s']*1000,
                          'unknown_ms':result['unknown_s']*1000,'stack_samples_all_frame_tids':len(matched),
                          'stack_samples_callback':sum(s['tid']==callback for s,t,n in matched),'sched_context_events':len(relevant)})
    write(OUT/'frame_summary.csv',summaries)
    (OUT/'README.md').write_text('Prediction 11帧证据提取。复用 generate_p4_perf_analysis.py 的解析、快照、调度窗口与状态分解函数。\n每帧目录包含 callback 与所有该帧实际worker的sched事件（前后100ms）及CPU完整调用栈，后者只取callback执行窗。active_trace_nodes为空的样本只表示同TID同时间，不能归因到该帧工作。\nCPU采样不等于精确CPU时间；blocked可能含等待工作线程完成，不能仅凭callback stack判定根因。调度waking用于等待分界是代理，精确唤醒分界需wakeup事件。\n时钟见clock_calibration.json，映射仅before快照。\n',encoding='utf-8')
    print(json.dumps(summaries,indent=2),flush=True)

if __name__=='__main__':main()
