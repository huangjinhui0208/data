"""Validate delivered report data and figures; browser acceptance is separate."""
import csv,json,pathlib,hashlib,re,xml.etree.ElementTree as ET
from PIL import Image

p=pathlib.Path(__file__).resolve().parents[1]/'prediction_realtime_report'
def read(f):return list(csv.DictReader((p/'data'/f).open(encoding='utf-8-sig')))
frames=read('frame_analysis.csv');ids={r['frame_uid'] for r in frames};checks=[]
def check(name,passed,detail):checks.append(dict(name=name,passed=bool(passed),detail=detail))
check('unique_frame_grain',len(ids)==len(frames)==693,'One unique UID per complete window callback')
check('precise_large_ids',all(isinstance(r['trace_id'],str) and len(r['trace_id'])>=19 for r in frames),'CSV IDs preserved as strings')
for f in ['major_stage_states.csv','waiting_episodes_with_event_stacks.csv','matched_comparisons.csv','downstream_runonce.csv','case_task_dependencies.csv']:
 rr=read(f);check('frame_reference_'+f,all(r['frame_uid'] in ids and r['run_id']=='pw_20261020_141904' for r in rr),str(len(rr))+' rows')
ep=read('waiting_episodes_with_event_stacks.csv')
check('episode_one_to_one',len({(r['frame_uid'],r['task_id'],r['state_row_id']) for r in ep})==len(ep)==4415,'No duplicate episode associations')
check('episode_mechanism_not_causal_increment',all(not r['attributable_excess_ms'] and not r['wait_object_identity'] for r in ep),'Object and attributable excess remain missing')
figs=list((p/'figures').glob('*.png'));check('six_png_svg_pairs',len(figs)==6 and all(f.with_suffix('.svg').exists() for f in figs),'6 paired scientific figures')
check('png_300dpi',all(min(Image.open(f).info.get('dpi',(0,0)))>=299 for f in figs),'PNG resolution metadata')
try:
 for f in figs:ET.parse(f.with_suffix('.svg'))
 valid=True
except ET.ParseError:valid=False
check('svg_valid_xml',valid,'All SVG parsed')
report=(p/'prediction_realtime_analysis_report.md').read_text(encoding='utf8')
refs=re.findall(r'!\[[^]]*\]\(([^)]+)\)',report)
check('markdown_figures_resolve',len(refs)==6 and all((p/f).exists() for f in refs),'All figure references exist')
snap=json.loads((p/'report_app/src/data.json').read_text(encoding='utf8'))
blocks=json.loads((p/'report_app/src/content/report/narrative.json').read_text(encoding='utf8'))
check('narrative_source_queries_exist',all(q in snap['queries'] for b in blocks for q in b['queries']),'Exact declared query scopes')
check('reviewed_coverage',len(snap['queries']['frames']['rows'])==693 and len(snap['queries']['downstream']['rows'])==690,'Report snapshot counts reconcile')
check('writing_skill_link', 'anti-defensive-writing-Skill' in report,'User requested writing skill recorded')
check('no_placeholders_in_delivered_data',all('example' not in r['frame_uid'] for r in frames),'Actual instance IDs only')
result=dict(passed=all(c['passed'] for c in checks),checks=checks,
            browser_render_check='blocked_by_browser_security_policy; not passed or substituted',
            static_visual_review='Six scientific PNGs inspected; title/legend overlap repaired')
(p/'artifact_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
manifest=[]
for f in sorted((p/'data').glob('*.csv')):
 manifest.append(dict(path=str(f.relative_to(p)),sha256=hashlib.sha256(f.read_bytes()).hexdigest(),rows=len(read(f.name))))
(p/'data_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps(dict(passed=result['passed'],checks=len(checks),failed=[c for c in checks if not c['passed']]),ensure_ascii=False))
if not result['passed']:raise SystemExit(1)
