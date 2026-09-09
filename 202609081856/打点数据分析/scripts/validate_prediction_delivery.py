"""Verify extracts and hash scoped deliverables; does not touch raw sources."""
import csv
import hashlib
from pathlib import Path

RUN=Path(__file__).resolve().parents[2]
def read(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

data=RUN/'打点数据分析/lidar_prediction_reanalysis'
perf=RUN/'perf和stack数据分析'
frames=read(perf/'frame_summary.csv')
assert {int(r['frame_index']) for r in frames}=={809,824,827,847,851,852,854,887,888,889,925}
for r in frames:
    total=sum(float(r[k]) for k in ('callback_running_ms','callback_blocked_ms','callback_runnable_ms','unknown_ms'))
    assert abs(total-float(r['execution_ms']))<.001
    assert all(float(r[k])>=-1e-6 for k in ('callback_running_ms','callback_blocked_ms','callback_runnable_ms','unknown_ms'))
    rows=read(perf/f"F{int(r['frame_index']):04d}"/'cpu_stack_samples.csv')
    assert len(rows)==int(r['stack_samples_all_frame_tids'])
gantt=read(data/'figures_800_950/gantt_frame_data.csv')
assert [int(r['frame_index']) for r in gantt]==list(range(800,951))
assert len(list((data/'figures_800_950').glob('*.png')))==5
paths=[*data.rglob('*'),*perf.rglob('*')]
paths += [Path(__file__).parent/n for n in ('analyze_prediction_lidar.py','extract_prediction_perf.py','plot_prediction_800_950.py','validate_prediction_delivery.py')]
paths += [RUN/'README_prediction.md',RUN.parent/'reusable_scripts/perf/generate_p4_perf_analysis.py',RUN.parent/'reusable_scripts/perf/test_sched_unknown_sample_tid.py']
excluded={'sched_relevant_calibration_context.txt','sched_cache_provenance.json'}
paths=sorted({p for p in paths if p.is_file() and '__pycache__' not in p.parts and p.name not in excluded and p.name not in {'analyze_prediction.py'}})
with (RUN/'prediction_delivery_manifest.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['repository_path','bytes','sha256'])
    for p in paths:w.writerow([p.relative_to(RUN.parent).as_posix(),p.stat().st_size,hashlib.sha256(p.read_bytes()).hexdigest()])
print('PASS: 11 frame extracts; 151 Gantt frames; 5 images; duration conservation; sample counts.')
