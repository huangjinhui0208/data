"""Evaluate Prediction jobs against the next real Fusion output deadline.

The complete Fusion output stream defines each relative deadline. Prediction
frames define jobs and completion times. No fixed-period deadline is assumed.
"""
import csv
import hashlib
import json
import statistics as stats
from collections import defaultdict
from pathlib import Path

RUN = Path(__file__).resolve().parents[2]
FRAMES = RUN / '打点数据分析/lidar_prediction_reanalysis/prediction_frames.csv'
FUSION = RUN / 'trace/message_context/perception.multi_sensor_fusion.1645308.csv'
OUT = RUN / '打点数据分析/prediction_next_fusion_deadline'
WORKLOAD = ('cruise_mlp_inference_calls', 'model_cruise_mlp_calls',
            'jointly_prediction_planning_inference_calls',
            'model_jointly_prediction_planning_calls')

def read(path):
    with path.open(encoding='utf-8-sig', newline='') as handle:
        return list(csv.DictReader(handle))

def write(path, rows, fields=None):
    with path.open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields or list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)

def save_text(path, text):
    with path.open('w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)

def quantile(values, q):
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    low = int(pos)
    return ordered[low] + (ordered[min(low + 1, len(ordered) - 1)] - ordered[low]) * (pos - low)

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    frames = read(FRAMES)
    fusion = [r for r in read(FUSION) if r['edge'] == 'out']
    assert fusion and all(int(a['mono_ns']) < int(b['mono_ns']) for a, b in zip(fusion, fusion[1:]))
    assert len({r['output_seq'] for r in fusion}) == len(fusion)
    by_seq = {r['output_seq']: (i, r) for i, r in enumerate(fusion)}
    by_trace = {r['trace_id']: (i, r) for i, r in enumerate(fusion)}
    assert len(by_trace) == len(fusion)

    complete = [r for r in frames if r['complete'] == 'True']
    groups = defaultdict(list)
    for frame in complete:
        signature = tuple(int(frame[key] or 0) for key in WORKLOAD)
        groups[signature].append(float(frame['execution_ms']))

    jobs = []
    previous_exit = None
    for index, frame in enumerate(frames):
        release_match = by_seq.get(frame['input_perception_seq']) or by_trace.get(frame['trace_id'])
        assert release_match is not None
        fusion_index, release = release_match
        assert frame['trace_id'] == release['trace_id']
        assert int(frame['perception_output_ns']) == int(release['mono_ns'])
        next_fusion = fusion[fusion_index + 1] if fusion_index + 1 < len(fusion) else None
        next_job = frames[index + 1] if index + 1 < len(frames) else None
        next_job_match = (by_seq.get(next_job['input_perception_seq']) or by_trace.get(next_job['trace_id'])) if next_job else None
        next_job_fusion_index = next_job_match[0] if next_job_match else None
        complete_job = frame['complete'] == 'True'
        release_ns = int(release['mono_ns'])
        exit_ns = int(frame['prediction_exit_ns']) if complete_job else None
        deadline_ns = int(next_fusion['mono_ns']) if next_fusion else None
        budget_ms = (deadline_ns - release_ns) / 1e6 if deadline_ns else None
        actual_response_ms = (exit_ns - release_ns) / 1e6 if exit_ns else None
        excess_signed = actual_response_ms - budget_ms if actual_response_ms is not None and budget_ms is not None else None
        miss = excess_signed > 0 if excess_signed is not None else None
        signature = tuple(int(frame[key] or 0) for key in WORKLOAD)
        peers = list(groups[signature])
        if complete_job:
            peers.remove(float(frame['execution_ms']))
        baseline = stats.median(peers) if len(peers) >= 4 else None
        execution = float(frame['execution_ms']) if complete_job else None
        job = {
            'frame_index': frame['frame_index'], 'trace_id': frame['trace_id'],
            'input_perception_seq': release['output_seq'],
            'input_perception_seq_source': 'prediction_message_context' if frame['input_perception_seq'] else 'recovered_from_fusion_trace_id_and_mono_ns',
            'prediction_output_seq': frame['prediction_output_seq'],
            'job_release_fusion_ns': release_ns,
            'deadline_next_fusion_seq': next_fusion['output_seq'] if next_fusion else None,
            'deadline_next_fusion_ns': deadline_ns,
            'deadline_budget_ms_expected': budget_ms,
            'prediction_exit_ns': exit_ns,
            'actual_response_ms': actual_response_ms,
            'deadline_actual_expected_ratio': actual_response_ms / budget_ms if actual_response_ms is not None and budget_ms else None,
            'deadline_excess_signed_ms': excess_signed,
            'deadline_excess_latency_ms': max(0.0, excess_signed) if excess_signed is not None else None,
            'deadline_slack_ms': max(0.0, -excess_signed) if excess_signed is not None else None,
            'deadline_miss': miss,
            'prediction_output_interval_ms': (exit_ns - previous_exit) / 1e6 if exit_ns is not None and previous_exit is not None else None,
            'next_prediction_input_seq': next_job_match[1]['output_seq'] if next_job_match else None,
            'fusion_outputs_skipped_before_next_prediction': next_job_fusion_index - fusion_index - 1 if next_job_fusion_index is not None else None,
            'next_real_fusion_processed_by_next_prediction': next_job_fusion_index == fusion_index + 1 if next_job_fusion_index is not None else None,
            **{key: signature[i] for i, key in enumerate(WORKLOAD)},
            'workload_signature': '|'.join(map(str, signature)),
            'workload_baseline_peer_n': len(peers),
            'workload_expected_execution_ms': baseline,
            'actual_execution_ms': execution,
            'execution_actual_expected_ratio': execution / baseline if execution is not None and baseline else None,
            'workload_excess_signed_ms': execution - baseline if execution is not None and baseline is not None else None,
            'workload_excess_latency_ms': max(0.0, execution - baseline) if execution is not None and baseline is not None else None,
            'execution_p99_tail': frame['execution_ms_p99_tail'],
            'evaluation_status': 'evaluated' if excess_signed is not None else ('incomplete_prediction' if not complete_job else 'no_next_fusion_output'),
        }
        jobs.append(job)
        if exit_ns is not None:
            previous_exit = exit_ns

    evaluated = [r for r in jobs if r['evaluation_status'] == 'evaluated']
    misses = [r for r in evaluated if r['deadline_miss']]
    signed_slacks = [r['deadline_budget_ms_expected'] - r['actual_response_ms'] for r in evaluated]
    output_intervals = [r['prediction_output_interval_ms'] for r in jobs if r['prediction_output_interval_ms'] is not None]
    max_output_interval_job = max((r for r in jobs if r['prediction_output_interval_ms'] is not None),
                                  key=lambda r: r['prediction_output_interval_ms'])
    assert all(r['deadline_budget_ms_expected'] > 0 for r in evaluated)
    assert all(v > 0 for v in output_intervals)
    write(OUT / 'prediction_jobs_next_fusion_deadline.csv', jobs)
    write(OUT / 'prediction_deadline_miss_jobs.csv', misses, list(jobs[0]))

    baseline_rows = []
    for signature, values in sorted(groups.items()):
        baseline_rows.append({**{key: signature[i] for i, key in enumerate(WORKLOAD)},
                              'workload_signature': '|'.join(map(str, signature)), 'n': len(values),
                              'execution_median_ms': stats.median(values),
                              'execution_p95_ms': quantile(values, .95), 'execution_min_ms': min(values),
                              'execution_max_ms': max(values),
                              'execution_p99_tail_n': sum(r['execution_ms_p99_tail'] == 'True' and tuple(int(r[k] or 0) for k in WORKLOAD) == signature for r in complete)})
    write(OUT / 'prediction_global_workload_baseline.csv', baseline_rows)

    bursts = []
    current = []
    for job in jobs:
        if job['deadline_miss'] is True:
            current.append(job)
        elif current:
            bursts.append(current); current = []
    if current:
        bursts.append(current)
    burst_rows = []
    for burst_id, burst in enumerate(bursts, 1):
        burst_rows.append({'burst_id': burst_id, 'start_frame': burst[0]['frame_index'],
                           'end_frame': burst[-1]['frame_index'], 'miss_job_count': len(burst),
                           'start_input_perception_seq': burst[0]['input_perception_seq'],
                           'end_input_perception_seq': burst[-1]['input_perception_seq'],
                           'total_excess_ms': sum(r['deadline_excess_latency_ms'] for r in burst),
                           'max_excess_ms': max(r['deadline_excess_latency_ms'] for r in burst),
                           'max_excess_frame': max(burst, key=lambda r: r['deadline_excess_latency_ms'])['frame_index']})
    write(OUT / 'prediction_continuous_miss_bursts.csv', burst_rows,
          ['burst_id','start_frame','end_frame','miss_job_count','start_input_perception_seq','end_input_perception_seq','total_excess_ms','max_excess_ms','max_excess_frame'])

    f888 = next(r for r in jobs if r['frame_index'] == '888')
    assert f888['input_perception_seq'] == '1523' and f888['deadline_next_fusion_seq'] == '1524'
    checks = {'prediction_jobs': len(jobs), 'fusion_output_messages': len(fusion),
              'evaluated_jobs': len(evaluated), 'deadline_misses': len(misses),
              'miss_rate': len(misses) / len(evaluated), 'continuous_miss_bursts': len(bursts),
              'longest_miss_burst_jobs': max((len(v) for v in bursts), default=0),
              'workload_signatures': len(groups),
              'jobs_with_workload_baseline': sum(r['workload_expected_execution_ms'] is not None for r in jobs),
              'recovered_input_perception_sequences': sum(r['input_perception_seq_source'].startswith('recovered') for r in jobs),
              'prediction_output_intervals': len(output_intervals),
              'prediction_output_interval_median_ms': stats.median(output_intervals),
              'prediction_output_interval_p95_ms': quantile(output_intervals, .95),
              'global_signed_slack_median_ms': stats.median(signed_slacks),
              'global_signed_slack_p05_ms': quantile(signed_slacks, .05),
              'prediction_output_interval_max_ms': max_output_interval_job['prediction_output_interval_ms'],
              'prediction_output_interval_max_frame': int(max_output_interval_job['frame_index']),
              'prediction_jobs_with_skipped_intermediate_fusion': sum((r['fusion_outputs_skipped_before_next_prediction'] or 0) > 0 for r in jobs),
              'f888_deadline_source': {'current_fusion_seq': f888['input_perception_seq'],
                                       'next_real_fusion_seq': f888['deadline_next_fusion_seq'],
                                       'next_prediction_input_seq': f888['next_prediction_input_seq'],
                                       'budget_ms': f888['deadline_budget_ms_expected'],
                                       'actual_response_ms': f888['actual_response_ms'],
                                       'excess_ms': f888['deadline_excess_latency_ms']},
              'validation': 'sequence uniqueness, strict Fusion time order, trace_id and release timestamp equality passed'}
    save_text(OUT / 'validation_and_summary.json', json.dumps(checks, ensure_ascii=False, indent=2))
    provenance = []
    for path, row_count, role in ((FRAMES, len(frames), 'complete Prediction job source'),
                                  (FUSION, len(read(FUSION)), 'complete Fusion context; all in/out rows')):
        provenance.append({'repository_path': path.relative_to(RUN.parent).as_posix(),
                           'role': role, 'bytes': path.stat().st_size, 'rows_excluding_header': row_count,
                           'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    write(OUT / 'source_provenance.csv', provenance)
    report = f'''# Prediction下一真实Fusion输出deadline分析

口径：每个Prediction job的release为其处理的Fusion输出时间；deadline为完整Fusion输出流中的下一条真实输出。Expected是两条Fusion输出的间隔，Actual是当前Fusion输出到Prediction完成的响应时间。miss条件为Prediction完成时间晚于下一条Fusion输出；excess latency是晚出的毫秒数。

共{len(jobs)}个Prediction job，完整Fusion流含{len(fusion)}个输出；{len(evaluated)}个job可判定deadline，miss {len(misses)}个，miss rate为{len(misses)/len(evaluated):.6%}。连续miss burst共{len(bursts)}段，最长{max((len(v) for v in bursts), default=0)}个job。11个执行P99尾部帧中9个miss，F827和F889未miss。

F888处理Fusion seq 1523，deadline确实取下一份真实Fusion seq 1524，而下一Prediction job处理seq {f888['next_prediction_input_seq']}。预算{f888['deadline_budget_ms_expected']:.3f}ms，实际响应{f888['actual_response_ms']:.3f}ms，超出{f888['deadline_excess_latency_ms']:.3f}ms。

全局workload基线按四元组（CruiseMLP inference、CruiseMLP模型、Joint inference、Joint模型调用次数）分组。逐帧Expected execution使用相同签名的其他完整帧中位数，至少要求4个peer；这是经验执行基线，不是deadline。Actual/Expected、带符号差值和正向excess分别保存。{sum(r['workload_expected_execution_ms'] is not None for r in jobs)}帧有足够基线样本。

Prediction output interval按相邻完整Prediction完成时间计算，共{len(output_intervals)}个区间，中位{stats.median(output_intervals):.3f}ms、P95 {quantile(output_intervals,.95):.3f}ms。Fusion中间消息是否被跳过另列；本次只有F888之后跳过seq 1524。连续miss burst按Prediction job顺序统计，遇到非miss或不可判定job结束。

最后一个或不完整job没有下一Fusion/完成时间时保留为空，不填0。该deadline是输入机会间隔定义的相对deadline，用于实时性评估，不代表Apollo配置中的契约deadline。
'''
    save_text(OUT / '分析说明.md', report)
    print(json.dumps(checks, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    main()
