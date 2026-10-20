"""Export all requested CSV records belonging to an exact composite frame UID."""
import argparse,csv,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--frame-uid',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
result={}
for p in sorted(root.glob('*.csv')):
    with p.open(encoding='utf-8',newline='') as f:
        rr=csv.DictReader(f)
        if 'frame_uid' in (rr.fieldnames or []):result[p.name]=[r for r in rr if r['frame_uid']==a.frame_uid]
if not result.get('frame_instances.csv'):raise SystemExit('Unknown frame_uid: use exact frame_instances.csv value')
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(a.output)
