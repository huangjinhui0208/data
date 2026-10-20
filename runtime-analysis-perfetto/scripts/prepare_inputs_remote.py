"""Read originals; write an auditable, compact export outside source directories."""
import csv, json, hashlib, tarfile
from pathlib import Path
csv.field_size_limit(2**30)

RUN = 'pw_20261020_141904'
APP = Path('/media/ssd/apollo10/data/scb_data') / RUN
SESSION = Path('/media/ssd/hjh/prediction_worker_trace/sessions') / RUN
OUT = Path('/media/ssd/hjh/runtime_analysis_exports') / (RUN + '_5_105')
OUT.mkdir(parents=True, exist_ok=True)
manifest = []

def copy(src, rel):
    if not src.exists():
        return
    data = src.read_bytes()
    dest = OUT / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(data)
    manifest.append(dict(source=str(src), exported=str(rel), bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))

for rel in ['trace/events/prediction.942621.csv', 'trace/prediction_worker_events/prediction.942621.csv', 'record/extraction_summary.json']:
    copy(APP / rel, Path('inputs') / rel)
for p in (APP / 'trace/message_context').glob('*.csv'):
    copy(p, Path('inputs/trace/message_context') / p.name)
for p in (APP / 'trace/fusion_inputs').glob('*.csv'):
    copy(p, Path('inputs/trace/fusion_inputs') / p.name)
for rel in ['session.json', 'identity_before.json', 'perfetto/results/validation.json', 'perfetto/capture.pbtx', 'perfetto/tracebox.log']:
    copy(SESSION / rel, Path('inputs/session') / rel)
for p in APP.glob('**/*meta*.json'):
    if p.stat().st_size < 100000:
        copy(p, Path('inputs/application_metadata') / p.relative_to(APP))
sources = ['modules/prediction/prediction_component.cc', 'modules/prediction/common/message_process.cc',
 'modules/prediction/common/prediction_thread_pool.h', 'modules/prediction/common/prediction_trace_context.h',
 'modules/prediction/evaluator/evaluator_manager.cc', 'modules/common/util/perf_trace_writer.h',
 'modules/common/util/perf_trace_writer.cc', 'modules/planning/planning_component.cc',
 'modules/planning/planning_component.h', 'cyber/tools/cyber_recorder/recorder.cc', 'cyber/time/time.cc']
for rel in sources:
    copy(Path('/media/ssd/apollo10') / rel, Path('source_snapshots') / rel)
for p in Path('/media/ssd/apollo10/modules/planning').glob('**/*component.cc'):
    if 'APOLLO_TRACE_INPUT' in p.read_text(errors='replace'):
        copy(p, Path('source_snapshots') / p.relative_to('/media/ssd/apollo10'))
for rel in ['prediction/prediction_gflags.cc', 'perception/multi_sensor_fusion/multi_sensor_fusion_component.cc']:
    copy(Path('/media/ssd/apollo10/modules') / rel, Path('source_snapshots/modules') / rel)
for name in ['prediction', 'perception']:
    src = APP / f'record/04_prediction_perception/{name}_raw.jsonl'
    h = hashlib.sha256()
    with src.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024), b''):h.update(block)
    dest = OUT / f'inputs/record_{name}_headers.jsonl'
    with src.open() as f, dest.open('w') as w:
        for rowno,r in enumerate(csv.DictReader(f), 2):
            data = json.loads(r.pop('raw_json'))
            r.update(source_file=str(src), source_csv_row=rowno, header=data.get('header', {}),
                     obstacle_count=len(data.get('prediction_obstacle', data.get('perception_obstacle', []))))
            w.write(json.dumps(r, ensure_ascii=True) + '\n')
    manifest.append(dict(source=str(src),exported=str(dest.relative_to(OUT)),sha256=h.hexdigest(), bytes=src.stat().st_size,transformation='CSV raw_json decoded; retain complete header and obstacle count; omit trajectories'))
src = APP / 'record/00_meta/record_message_index.csv'
copy(src, Path('inputs/record_message_index.csv'))
src = APP / 'record/04_prediction_perception/prediction_obstacles.csv'
counts={}
with src.open() as f:
    reader=csv.DictReader(f)
    for r in reader:
        key=(r.get('header_sequence_num'),r.get('header_lidar_timestamp'))
        counts[key]=counts.get(key,0)+1
(OUT/'inputs/prediction_obstacle_table_audit.json').write_text(json.dumps({'source':str(src),'rows':sum(counts.values()),'message_keys':len(counts),'fields':reader.fieldnames},indent=2))
(OUT/'inputs/source_manifest.json').write_text(json.dumps(manifest,indent=2))
archive=OUT.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as t:
    for p in OUT.rglob('*'):
        if p.is_file():t.add(p,arcname=str(p.relative_to(OUT)))
print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,files=len(manifest))))
