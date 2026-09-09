"""Prediction timing using explicit main-LiDAR lineage; no assumed deadline."""
import csv
import json
import re
import statistics as st
from collections import defaultdict
from pathlib import Path
import argparse
import hashlib

def read(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))

def write(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)

def one(root, pattern):
    paths = list(root.glob(pattern))
    assert len(paths) == 1, (pattern, paths)
    return paths[0]

def percentile(values, p):
    a = sorted(values)
    pos = (len(a)-1)*p
    lo = int(pos)
    return a[lo] + (a[min(lo+1,len(a)-1)]-a[lo])*(pos-lo)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', type=Path, required=True)
    args = ap.parse_args()
    root = args.run_dir
    out = root/'打点数据分析'/'lidar_prediction_reanalysis'
    out.mkdir(parents=True, exist_ok=True)
    events = defaultdict(list)
    malformed = []
    for line, r in enumerate(read(one(root/'trace/events','prediction.*.csv')),2):
        if not all(r.get(k) for k in ['trace_id','mono_ns','tid','phase','event_id']):
            malformed.append({'line':line,'row':str(r)})
            continue
        for k in ['mono_ns','tid','event_id']:
            r[k] = int(r[k])
        events[r['trace_id']].append(r)
    context = defaultdict(dict)
    for r in read(one(root/'trace/message_context','prediction.*.csv')):
        context[r['trace_id']][r['edge']] = r
    fusion = {r['trace_id']:r for r in read(one(root/'trace/message_context','perception.multi_sensor_fusion.*.csv')) if r['edge']=='out'}
    parents = defaultdict(list)
    for r in read(one(root/'trace/fusion_inputs','perception.multi_sensor_fusion.*.csv')):
        if r['sensor_kind']=='lidar' and r['is_main_sensor']=='1':
            parents[r['fusion_trace_id']].append(r['parent_trace_id'])
    lidar = {r['trace_id']:r for r in read(one(root/'trace/message_context','perception.pointcloud_preprocess.*.csv')) if r['edge']=='in'}
    lidar_order = {t:i+1 for i,t in enumerate(sorted(lidar, key=lambda t:int(lidar[t]['mono_ns'])))}
    log_tids = {}
    for line in one(root/'log','prediction.log.INFO.*').read_text(encoding='utf-8',errors='replace').splitlines():
        m = re.search(r'\[PRED_TID\] input_perception_seq=(\d+) tid=(\d+)',line)
        if m:
            log_tids[m[1]] = int(m[2])
    frames, instances, issues = [], [], []
    for trace, ev in sorted(events.items(),key=lambda item:min(r['mono_ns'] for r in item[1])):
        phases = defaultdict(list)
        for r in ev:
            phases[r['phase']].append(r)
        if len(phases['proc_enter'])!=1:
            issues.append({'trace_id':trace,'issue':'missing/nonunique proc_enter'})
            continue
        start = phases['proc_enter'][0]
        end = phases['proc_exit'][0] if len(phases['proc_exit'])==1 else None
        ctx = context[trace]
        upstream = fusion.get(trace)
        parent = parents[trace][0] if len(parents[trace])==1 else None
        source = lidar.get(parent)
        seq = ctx.get('in',{}).get('input_seq','')
        upstream_ns = int(upstream['mono_ns']) if upstream else None
        source_ns = int(source['mono_ns']) if source else None
        row = {'frame_index':len(frames)+1,'trace_id':trace,'input_perception_seq':seq,
               'prediction_output_seq':ctx.get('out',{}).get('output_seq',''),
               'lidar_trace_id':parent,'lidar_frame_index':lidar_order.get(parent),
               'lidar_data_ts_ns':source['data_ts_ns'] if source else None,
               'callback_tid':start['tid'],'log_tid':log_tids.get(seq),
               'lidar_enter_ns':source_ns,'perception_output_ns':upstream_ns,
               'prediction_enter_ns':start['mono_ns'],'prediction_exit_ns':end['mono_ns'] if end else None,
               'complete':end is not None,
               'execution_ms':(end['mono_ns']-start['mono_ns'])/1e6 if end else None,
               'upstream_perception_ms':(upstream_ns-source_ns)/1e6 if source_ns and upstream_ns else None,
               'dispatch_proxy_ms':(start['mono_ns']-upstream_ns)/1e6 if upstream_ns else None,
               'lidar_to_prediction_complete_ms':(end['mono_ns']-source_ns)/1e6 if end and source_ns else None,
               'perception_to_prediction_complete_ms':(end['mono_ns']-upstream_ns)/1e6 if end and upstream_ns else None}
        pending = defaultdict(list)
        local = []
        for r in sorted(ev,key=lambda x:x['event_id']):
            phase=r['phase']
            if phase.endswith('_enter'):
                pending[(phase[:-6],r['tid'])].append(r)
            elif phase.endswith('_exit') or phase=='writer_done':
                node = 'writer' if phase=='writer_done' else phase[:-5]
                key=(node,r['tid'])
                if not pending[key]:
                    issues.append({'trace_id':trace,'issue':'unpaired exit','phase':phase,'tid':r['tid']})
                    continue
                s=pending[key].pop()
                assert r['mono_ns']>=s['mono_ns']
                local.append({'frame_index':row['frame_index'],'trace_id':trace,'node':node,'tid':r['tid'],
                              'enter_ns':s['mono_ns'],'exit_ns':r['mono_ns'],'duration_ms':(r['mono_ns']-s['mono_ns'])/1e6})
        for (node,tid), remaining in pending.items():
            for s in remaining:
                issues.append({'trace_id':trace,'issue':'unpaired enter','phase':node,'tid':tid})
        for node in sorted({r['node'] for r in local}):
            spans=[r['duration_ms'] for r in local if r['node']==node]
            row[node+'_calls']=len(spans)
            row[node+'_sum_ms']=sum(spans)
            row[node+'_max_ms']=max(spans)
        row['all_traced_span_ms']=(max(r['mono_ns'] for r in ev)-start['mono_ns'])/1e6 if end and not any(pending.values()) else None
        instances.extend(local)
        frames.append(row)
    metrics=['execution_ms','dispatch_proxy_ms','upstream_perception_ms','lidar_to_prediction_complete_ms','perception_to_prediction_complete_ms','all_traced_span_ms']
    summaries=[]
    for metric in metrics:
        vals=[r[metric] for r in frames if r['complete'] and r[metric] is not None]
        med=st.median(vals); mad=st.median(abs(v-med) for v in vals)
        threshold=percentile(vals,.99)
        summaries.append({'metric':metric,'n':len(vals),'median_ms':med,'p95_ms':percentile(vals,.95),'p99_ms':threshold,'max_ms':max(vals),'median_plus_6MAD_ms':med+6*1.4826*mad})
        for r in frames:
            r[metric+'_p99_tail']=bool(r['complete'] and r[metric] is not None and r[metric]>threshold)
    complete=[r for r in frames if r['complete']]
    for i,r in enumerate(frames):
        prior=frames[max(0,i-20):i]
        vals=[p['execution_ms'] for p in prior if p['complete']]
        baseline=st.median(vals) if len(vals)>=10 else None
        r['previous20_execution_median_ms']=baseline
        r['local_execution_jump']=bool(r['complete'] and baseline is not None and r['execution_ms']>2*baseline and r['execution_ms']-baseline>10)
        r['anomaly_categories']=';'.join(k for k in metrics if r[k+'_p99_tail'])
    spikes=sorted([r for r in complete if r['execution_ms_p99_tail']],key=lambda r:r['execution_ms'],reverse=True)
    write(out/'prediction_frames.csv',frames)
    write(out/'prediction_node_instances.csv',instances)
    write(out/'timing_summary.csv',summaries)
    write(out/'execution_tail_frames.csv',spikes)
    write(out/'local_execution_jumps.csv',[r for r in complete if r['local_execution_jump']])
    write(out/'all_timing_tail_frames.csv',[r for r in complete if r['anomaly_categories']])
    write(out/'pairing_issues.csv',issues)
    checks={'raw_frames':len(frames),'complete_frames':len(complete),'malformed_rows':malformed,
            'main_lidar_parent_linked':sum(r['lidar_trace_id'] is not None for r in frames),
            'lidar_ingress_linked':sum(r['lidar_enter_ns'] is not None for r in frames),
            'fusion_linked':sum(r['perception_output_ns'] is not None for r in frames),
            'tid_mismatches':sum(r['log_tid'] is not None and r['log_tid']!=r['callback_tid'] for r in frames),
            'pairing_issues':len(issues),'execution_tail_frames':len(spikes),
            'deadline':'not defined: no independent response budget supplied'}
    checks['complete_frame_pairing_issues'] = sum(i['trace_id'] in {r['trace_id'] for r in complete} for i in issues)
    checks['negative_dispatch_proxy'] = sum(r['dispatch_proxy_ms'] is not None and r['dispatch_proxy_ms']<0 for r in frames)
    checks['max_chain_arithmetic_error_ms'] = max(abs(r['lidar_to_prediction_complete_ms']-r['upstream_perception_ms']-r['dispatch_proxy_ms']-r['execution_ms']) for r in complete if r['lidar_to_prediction_complete_ms'] is not None)
    assert checks['complete_frame_pairing_issues']==0
    assert checks['tid_mismatches']==0
    assert checks['negative_dispatch_proxy']==0
    assert checks['max_chain_arithmetic_error_ms']<1e-9
    (out/'validation.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# LiDAR → Prediction 逐帧重新统计','',
           '以 fusion_inputs 中 is_main_sensor=1 且 sensor_kind=lidar 的 parent_trace_id 关联 LiDAR。时间差全部使用 trace mono_ns。',
           'execution_ms 为 proc_enter→proc_exit；dispatch_proxy_ms 为对应 Fusion output→proc_enter，包含传输与调度，不能直接称为 Reader 排队。',
           '100 ms 不再作为 deadline；下一次 proc_enter 不作为输入到达或 deadline。',
           '重复与可选节点全部按 trace、节点、TID 逐实例配对；并行节点 sum_ms 可超过 callback 时长，不代表关键路径或 CPU 时间。',
           '异常筛选：各指标 > 全程 P99 标记尾部候选；局部突增须超过前20帧中位数2倍且增加超过10ms（至少10个前序完整帧）。这些是统计候选，不等于故障。','',
           '## 执行尾部帧','', '| 帧 | LiDAR帧 | 输入序号 | TID | 执行ms | 上游ms | 派发代理ms |','|---:|---:|---:|---:|---:|---:|---:|']
    for r in spikes:
        lines.append('| '+' | '.join(str(round(r[k],3)) if isinstance(r[k],float) else str(r[k]) for k in ['frame_index','lidar_frame_index','input_perception_seq','callback_tid','execution_ms','upstream_perception_ms','dispatch_proxy_ms'])+' |')
    lines+=['','完整性与关联数量见 validation.json；不完整帧保留在 prediction_frames.csv，未纳入时延分位数。','此前目录中的固定deadline结果已经撤回，请以本目录为准。']
    (out/'分析说明.md').write_text('\n'.join(lines),encoding='utf-8')
    write(out/'manifest.csv',[{'file':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(out.glob('*')) if p.is_file() and p.name!='manifest.csv'])
    print(json.dumps(checks,ensure_ascii=False))
    print(json.dumps(summaries,ensure_ascii=False))

if __name__=='__main__':
    main()
