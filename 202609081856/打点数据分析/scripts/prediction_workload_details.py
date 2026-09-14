"""Extend frame counts with observed model spans; never equate workers to groups."""
import csv
import json
import re
from collections import defaultdict

MODELS={'model_cruise_mlp':'CRUISE_MLP_EVALUATOR',
        'model_jointly_prediction_planning':'JOINTLY_PREDICTION_PLANNING_EVALUATOR'}

def extend_workload(run, out, rows, frames):
    def save(name, data, fields):
        with (out/name).open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(data)
    nodes=defaultdict(list)
    with (run/'打点数据分析/lidar_prediction_reanalysis/prediction_node_instances.csv').open(encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):nodes[r['frame_index']].append(r)
    logs=defaultdict(list);seq=None
    log=next((run/'log').glob('prediction.log.INFO.*'))
    for line_no,line in enumerate(log.read_text(encoding='utf-8').splitlines(),1):
        marker=re.search(r'\[PRED_TID\] input_perception_seq=(\d+)',line)
        if marker:seq=marker[1]
        m=re.search(r'\s(\d+) evaluator_manager\.cc:\d+\] (\w+) Obstacle: id=(\d+) type=(\w+) evaluator=(\w+)',line)
        if m and seq:
            tid,priority,oid,kind,model=m.groups()
            logs[(seq,tid,model)].append({'obstacle_id':oid,'priority':priority,'type':kind,'log_line':line_no})
    workers=[];calls=[];status=[];matched=0;unmatched=0
    for row,frame in zip(rows,frames):
        assert row['frame_index']==frame['frame_index']
        ns=nodes[row['frame_index']];complete=frame['complete']=='True'
        for field,node in [('cruise_mlp_call_count','model_cruise_mlp'),('joint_prediction_planning_call_count','model_jointly_prediction_planning'),('cruise_mlp_inference_call_count','cruise_mlp_inference'),('joint_prediction_planning_inference_call_count','jointly_prediction_planning_inference')]:
            row[field]=sum(n['node']==node for n in ns) if complete else None
        row['model_call_count_status']='observed_paired_trace_spans' if complete else 'incomplete_frame'
        row['vectornet_call_count']=None
        row['vectornet_call_count_status']='unavailable_no_dedicated_trace_marker'
        row['vectornet_evaluator_log_mentions']=sum(len(v) for (s,t,m),v in logs.items() if s==row['input_perception_seq'] and 'VECTOR' in m)
        row['threadpool_group_count']=None
        row['threadpool_group_obstacle_counts']=None
        row['threadpool_group_execution_ms']=None
        row['threadpool_group_status']='unavailable_no_group_id_or_group_enter_exit'
        row['threadpool_total_execution_ms']=frame.get('thread_pool_run_sum_ms') or None
        spans=defaultdict(list)
        for n in ns:
            if n['node'] in MODELS:spans[(n['tid'],n['node'])].append(n)
        bytid=defaultdict(list)
        for (tid,node),vv in spans.items():
            vv.sort(key=lambda n:int(n['enter_ns']))
            ll=logs[(row['input_perception_seq'],tid,MODELS[node])]
            valid=len(vv)==len(ll)
            matched+=len(vv) if valid else 0;unmatched+=0 if valid else len(vv)
            for i,n in enumerate(vv):
                item={'frame_index':row['frame_index'],'input_perception_seq':row['input_perception_seq'],'trace_id':row['trace_id'],'worker_tid':tid,'model_node':node,'call_index_in_tid_model':i+1,'enter_ns':n['enter_ns'],'exit_ns':n['exit_ns'],'execution_ms':n['duration_ms'],
                      'obstacle_id':ll[i]['obstacle_id'] if valid else None,'log_line':ll[i]['log_line'] if valid else None,
                      'obstacle_mapping_status':'seq_log_block+tid+model+ordered_count_match' if valid else 'unavailable_log_trace_count_mismatch'}
                calls.append(item);bytid[tid].append(item)
        row['observed_model_worker_count']=len(bytid)
        for tid,vv in bytid.items():
            ordered=sorted(vv,key=lambda v:int(v['enter_ns']))
            assert all(int(a['exit_ns'])<=int(b['enter_ns']) for a,b in zip(ordered,ordered[1:]))
            ids=[v['obstacle_id'] for v in vv]
            workers.append({'frame_index':row['frame_index'],'worker_tid':tid,'group_id':None,'group_obstacle_count':None,'group_execution_ms':None,
                            'observed_evaluated_obstacle_count':len(set(ids)) if all(v is not None for v in ids) else None,
                            'observed_obstacle_ids':';'.join(sorted(set(ids),key=int)) if all(v is not None for v in ids) else None,
                            'model_call_count':len(vv),'model_execution_sum_ms':sum(float(v['execution_ms']) for v in vv),
                            'first_model_enter_to_last_exit_ms':(max(int(v['exit_ns']) for v in vv)-min(int(v['enter_ns']) for v in vv))/1e6,
                            'status':'worker_model_spans_only_not_logical_group'})
        status.append({k:row[k] for k in ['frame_index','threadpool_group_count','threadpool_group_obstacle_counts','threadpool_group_execution_ms','threadpool_group_status','observed_model_worker_count','threadpool_total_execution_ms']})
        if row['prediction_count'] is not None:
            assert sum(row[k] for k in ['caution_count','normal_count','ignore_count','priority_unknown_count'])==row['prediction_count']
            assert row['interactive_count']<=row['prediction_count']
    save('prediction_model_call_instances.csv',calls,list(calls[0]))
    save('prediction_worker_model_workload.csv',workers,list(workers[0]))
    save('prediction_threadpool_group_availability.csv',status,list(status[0]))
    (out/'workload_validation.json').write_text(json.dumps({'frames':len(rows),'paired_model_calls':len(calls),'calls_with_log_obstacle_mapping':matched,'unmapped_model_calls':unmatched,'priority_count_conservation':'passed','worker_model_spans_nonoverlap':'passed','group_identity_available':False,'vectornet_exact_calls_available':False},indent=2),encoding='utf-8')
    (out/'新增工作量字段说明.md').write_text('''# 逐帧工作量字段

四类count来自Prediction输出record标签。Caution/Normal/Ignore是互斥优先级；Interactive来自interactive_tag=INTERACTION，是独立维度，不能与前三项相加。未知标签单列，缺失record输出留空。

cruise_mlp_call_count与joint_prediction_planning_call_count分别统计model_cruise_mlp与model_jointly_prediction_planning完整入口/出口对。另列内部inference调用次数，两种调用层级不能混用，也不表示GPU kernel次数。不完整帧留空。VectorNet无专用打点，精确调用次数不可确定；日志匹配为0仅表示未观测到该名称。

ThreadPool逻辑group的ID、障碍物分配和group入口/出口没有记录，相关字段留空。threadpool_total_execution_ms仅为整个thread_pool_run墙钟区间。

prediction_worker_model_workload.csv提供实际worker TID的可观测模型调用数、模型耗时之和及首个模型入口到最后模型出口的跨度。这不是group execution，也不是scheduler CPU时间。observed_evaluated_obstacle_count只计成功关联到模型日志的障碍物，不能包含未进入模型的障碍物或恢复真实group分配。

prediction_model_call_instances.csv保存逐调用trace时间。障碍物ID按PRED_TID日志块的输入序号、worker TID、模型名及同TID调用顺序关联，要求日志/trace数量一致。这是间接关联，trace没有直接存obstacle_id；日志异步乱序仍可能影响ID，调用次数和耗时直接取trace，不依赖ID关联。

若要获得严格group数据，后续采集应记录frame trace_id、group_id、group内obstacle_id列表，以及group_execute_enter/exit与执行TID；VectorNet需要独立Evaluate/Inference入口出口打点。当前run不能补采已发生的事件。
''',encoding='utf-8')
