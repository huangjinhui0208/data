"""Count complete record messages and join by sequence AND trace identity."""
import csv
import json
import statistics as st
import argparse
from collections import defaultdict
from pathlib import Path

RUN=Path(__file__).resolve().parents[2]
OUT=RUN/'打点数据分析/prediction_obstacle_counts'
DATA=RUN/'record/04_prediction_perception'
csv.field_size_limit(100000000)

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f:
        yield from csv.DictReader(f)

def write(name,rows):
    with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader();w.writerows(rows)

def messages(module):
    result={}
    for row in read(DATA/(module+'_raw.jsonl')):
        obj=json.loads(row['raw_json']); h=obj['header']; seq=str(h['sequence_num'])
        assert seq==row['header_sequence_num'] and seq not in result
        obs=obj.get(module+'_obstacle',[])
        r={'seq':seq,'trace_id':str(h.get('trace_id','')),'count':len(obs),
           'lidar_timestamp':str(h.get('lidar_timestamp','')),
           'ids':';'.join(str(o.get('perception_obstacle',o).get('id')) for o in obs)}
        if module=='prediction':
            for label in ('CAUTION','NORMAL','IGNORE'):
                r[label.lower()+'_count']=sum(o.get('priority',{}).get('priority')==label for o in obs)
            r['interactive_count']=sum(o.get('interactive_tag',{}).get('interactive_tag')=='INTERACTION' for o in obs)
            r['priority_unknown_count']=sum(o.get('priority',{}).get('priority') not in ('CAUTION','NORMAL','IGNORE') for o in obs)
            r['interactive_unknown_count']=sum(o.get('interactive_tag',{}).get('interactive_tag') not in ('INTERACTION','NONINTERACTION') for o in obs)
            r.update(with_trajectory=sum(bool(o.get('trajectory')) for o in obs),
                     trajectories=sum(len(o.get('trajectory',[])) for o in obs),
                     trajectory_points=sum(len(t.get('trajectory_point',[])) for o in obs for t in o.get('trajectory',[])),
                     static=sum(o.get('is_static') is True for o in obs))
        result[seq]=r
    return result

def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if args.output_dir:
        OUT=args.output_dir
    OUT.mkdir(exist_ok=True)
    pred=messages('prediction');perc=messages('perception')
    write('prediction_record_message_counts.csv',list(pred.values()))
    frames=list(read(RUN/'打点数据分析/lidar_prediction_reanalysis/prediction_frames.csv'))
    rows=[]
    for f in frames:
        p=pred.get(f['prediction_output_seq']);q=perc.get(f['input_perception_seq'])
        for m in (p,q):
            if m:assert m['trace_id']==f['trace_id'], (f['frame_index'],m)
        if p and q:assert p['lidar_timestamp']==q['lidar_timestamp']
        r={k:f[k] for k in ['frame_index','trace_id','input_perception_seq','prediction_output_seq','execution_ms','thread_pool_run_sum_ms','execution_ms_p99_tail']}
        r.update(perception_count=q['count'] if q else None,prediction_count=p['count'] if p else None,
                 prediction_with_trajectory=p['with_trajectory'] if p else None,trajectory_count=p['trajectories'] if p else None,
                 trajectory_points=p['trajectory_points'] if p else None,static_count=p['static'] if p else None,
                 perception_ids=q['ids'] if q else None,prediction_ids=p['ids'] if p else None,
                 output_record_matched=p is not None,input_record_matched=q is not None)
        for key in ('caution_count','normal_count','ignore_count','interactive_count','priority_unknown_count','interactive_unknown_count'):
            r[key]=p[key] if p else None
        rows.append(r)
    from prediction_workload_details import extend_workload
    extend_workload(RUN, OUT, rows, frames)
    good=[r for r in rows if r['execution_ms'] and r['prediction_count'] is not None and r['perception_count'] is not None]
    tails=[r for r in good if r['execution_ms_p99_tail']=='True']
    assert len(tails)==11
    for r in rows:
        neighbors=[v for v in good if 0<abs(int(v['frame_index'])-int(r['frame_index']))<=20 and v['execution_ms_p99_tail']!='True']
        for k in ['perception_count','prediction_count','prediction_with_trajectory','trajectory_points','execution_ms']:
            r['neighbor20_normal_median_'+k]=st.median(float(v[k]) for v in neighbors) if neighbors else None
        previous=rows[int(r['frame_index'])-2] if int(r['frame_index'])>1 else None
        for k in ['perception_count','prediction_count']:
            r['delta_previous_'+k]=r[k]-previous[k] if previous and r[k] is not None and previous[k] is not None else None
        same=[v for v in neighbors if v['perception_count']==r['perception_count'] and v['prediction_count']==r['prediction_count']]
        r['same_count_neighbor_n']=len(same)
        r['same_count_neighbor_execution_median_ms']=st.median(float(v['execution_ms']) for v in same) if same else None
    write('prediction_frame_obstacle_counts.csv',rows)
    write('eleven_anomaly_comparison.csv',tails)
    groups=defaultdict(list)
    for r in good:groups[(r['perception_count'],r['prediction_count'])].append(r)
    grouped=[]
    for (a,b),vv in sorted(groups.items()):
        grouped.append({'perception_count':a,'prediction_count':b,'n':len(vv),'execution_median_ms':st.median(float(v['execution_ms']) for v in vv),'min_ms':min(float(v['execution_ms']) for v in vv),'max_ms':max(float(v['execution_ms']) for v in vv),'tail_n':sum(v['execution_ms_p99_tail']=='True' for v in vv)})
    write('count_group_execution.csv',grouped)
    check={'prediction_record_messages':len(pred),'perception_record_messages':len(perc),'trace_frames':len(rows),'matched_inputs':sum(r['input_record_matched'] for r in rows),'matched_outputs':sum(r['output_record_matched'] for r in rows),'complete_matched':len(good),'anomaly_frames':len(tails),'sequence_trace_lidar_checks':'passed','missing_output_frames':[r['frame_index'] for r in rows if not r['output_record_matched']]}
    (OUT/'validation.json').write_text(json.dumps(check,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# Prediction障碍物数量与耗时分析','',
           '数量增加与整体耗时升高有关，但不能单独解释11帧的额外尖峰；这是观测关联，不是已确认的因果关系。',
           '', '完整record原始导出虽命名为raw.jsonl，实际按CSV解析其中raw_json字段。逐消息计数保留零障碍物消息，按输出/输入sequence关联trace，并核对trace_id和LiDAR时间戳。',
           f"record含{len(pred)}条Prediction消息；1042个trace帧中1038帧输入输出均关联成功。F1039–F1042没有匹配的record输出，数量留空而非填0；11个异常帧均完整匹配。",
           '在1038个匹配帧中，Perception输入条目数均等于Prediction输出条目数。带轨迹障碍物另计，轨迹条数、轨迹点数与障碍物条目数不能混用。',
           '', '| Prediction帧 | 输入/输出障碍物数 | 带轨迹数 | 较上一帧数量变化 | 执行ms | 邻近同数量非尾部帧中位ms | 对照帧数 |',
           '|---:|---:|---:|---:|---:|---:|---:|']
    assert all(r['perception_count']==r['prediction_count'] for r in good)
    for r in tails:
        lines.append(f"| {r['frame_index']} | {r['prediction_count']} | {r['prediction_with_trajectory']} | {r['delta_previous_perception_count']:+d} | {float(r['execution_ms']):.3f} | {r['same_count_neighbor_execution_median_ms']:.3f} | {r['same_count_neighbor_n']} |")
    lines+=['', '对照定义：当前帧前后各20帧内，输入/输出障碍物数相同、未超过全程执行P99的完整匹配帧。该选择用于描述局部背景，不是随机对照；剔除尾部帧会使对照耗时偏低，不能据此估算因果贡献。',
            '', '全程按数量分组：0个障碍物362帧，耗时中位3.191ms；8个89帧，中位114.034ms；9个77帧，中位133.770ms；10个25帧，中位139.828ms；11个11帧，中位141.704ms。数量与时间段、场景和运动状态共同变化，分组差异不等于数量的独立效应。',
            '', '11帧中8帧数量较前帧不变、2帧减少、1帧增加。F824由7增至8，可能有工作量增加的贡献；其余尖峰没有总数量突增依据。F851总数不变，但带轨迹数由10升至11，说明仅看总数会遗漏任务组成变化。',
            '', 'F886–F890均为10个障碍物、10个带轨迹障碍物，耗时依次138.638、159.257、269.131、161.418、149.527ms；F888 thread_pool_run达253.362ms。F853与F854均为11条输出、10个带轨迹障碍物，耗时分别124.148与216.826ms。两组都显示数量相同仍有显著耗时差异。',
            '', '结合此前perf结果，F888 callback线程CPU驻留13.577ms、blocked253.185ms，不能把269.131ms全部当作callback自身计算。等待工作线程完成也会形成blocked，因此仍需检查工作线程计算、锁/同步、调度与内存行为；当前数据尚不能确定这些因素各自的贡献。',
            '', '文件：prediction_frame_obstacle_counts.csv保存全部trace帧；prediction_record_message_counts.csv保存全部record输出消息；eleven_anomaly_comparison.csv保存11帧对比；count_group_execution.csv保存按数量分组的统计；validation.json保存关联完整性。']
    (OUT/'分析结论.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(check))
    print(json.dumps([{k:r[k] for k in ['frame_index','execution_ms','perception_count','prediction_count','prediction_with_trajectory','trajectory_points','delta_previous_perception_count','neighbor20_normal_median_perception_count','same_count_neighbor_n','same_count_neighbor_execution_median_ms']} for r in tails],indent=2))
    print(json.dumps(grouped,indent=2))

if __name__=='__main__':main()
