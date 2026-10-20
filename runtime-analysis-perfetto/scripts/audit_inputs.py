"""Verify completed export result counts/CRC and preserve source hashes."""
import csv,gzip,hashlib,json
from pathlib import Path
from build_tables import ROOT as R, write, dump

status=json.loads((R/'inputs/perfetto_export/extraction_status.json').read_text());rows=[]
completed=['thread','thread_state','target_sched','perf_sample','stack_callsites','stack_validation','wakeup']
for name in completed:
    p=R/'inputs/perfetto_export'/(name+'.jsonl.gz')
    with gzip.open(p,'rt') as f:count=sum(1 for _ in f)
    rows.append(dict(section=name,expected=status['counts'][name],actual=count,gzip_crc_pass=True,passed=count==status['counts'][name],sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
aux=json.loads((R/'inputs/perfetto_context/status.json').read_text())
for name,count in aux['counts'].items():
    p=R/'inputs/perfetto_context'/(name+'.jsonl.gz')
    with gzip.open(p,'rt') as f:n=sum(1 for _ in f)
    rows.append(dict(section='aux:'+name,expected=count,actual=n,gzip_crc_pass=True,passed=n==count,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
write('metadata/export_integrity.csv',rows)
assert all(r['passed'] for r in rows)
src=[]
for p in (R/'source_snapshots').rglob('*'):
    if p.is_file():src.append(dict(snapshot=str(p.relative_to(R)),source='/media/ssd/apollo10/'+p.relative_to(R/'source_snapshots').as_posix(),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),capture_binary_equivalence='not_proven'))
write('metadata/source_snapshot_manifest.csv',src)
planning=R/'inputs/trace/events/planning.942625.csv'
dump(R/'metadata/additional_input_hash.json',dict(source='/media/ssd/apollo10/data/scb_data/pw_20261020_141904/trace/events/planning.942625.csv',sha256=hashlib.sha256(planning.read_bytes()).hexdigest(),bytes=planning.stat().st_size))
print(json.dumps(dict(checked_exports=len(rows),passed=True,source_snapshots=len(src))))
