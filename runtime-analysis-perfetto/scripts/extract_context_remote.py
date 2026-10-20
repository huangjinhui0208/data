"""Finish narrow CPU context and count audit independently of expensive joins."""
import gzip,json,pathlib,subprocess,time
ROOT=pathlib.Path('/media/ssd/hjh/runtime_analysis_exports/pw_20261020_141904_5_105')
req=json.loads((ROOT/'extraction_request.json').read_text());lo,hi=req['start_ns'],req['end_ns'];out=ROOT/'perfetto_context';out.mkdir(exist_ok=True)
windows=sorted((w['start_ns'],w['end_ns']) for w in req['windows']);merged=[]
for a,b in windows:
    if merged and a<=merged[-1][1]:merged[-1][1]=max(b,merged[-1][1])
    else:merged.append([a,b])
query=[]
for a,b in merged:
    query.append(f"SELECT json_object('section','sched_context','row',json_object('id',id,'ts',ts,'dur',dur,'cpu',cpu,'utid',utid,'priority',priority,'end_state',end_state)) AS payload FROM sched WHERE ts<{b} AND ts+dur>{a} AND dur>=0;")
query.append(f"SELECT json_object('section','counts','row',json_object('source','sched_systemwide','count',COUNT(*),'first_ts',MIN(ts),'last_ts',MAX(ts))) AS payload FROM sched WHERE ts>={lo} AND ts<{hi};")
query.append(f"SELECT json_object('section','counts','row',json_object('source','perf_source_'||perf_session_id,'count',COUNT(*),'unwind_errors',SUM(unwind_error IS NOT NULL),'with_stack',COUNT(callsite_id),'first_ts',MIN(ts),'last_ts',MAX(ts))) AS payload FROM perf_sample WHERE ts>={lo} AND ts<{hi} GROUP BY perf_session_id;")
query.append("SELECT json_object('section','stats','row',json_object('name',name,'idx',idx,'severity',severity,'source',source,'value',value)) AS payload FROM stats WHERE severity IN ('error','data_loss') OR name GLOB '*lost*' OR name GLOB '*overrun*' OR name GLOB '*dropped*';")
sql='\n'.join(query);(out/'extract_context.sql').write_text(sql)
handles={k:gzip.open(out/(k+'.jsonl.gz'),'wt') for k in ['sched_context','counts','stats']};counts={k:0 for k in handles};errors=[];begin=time.monotonic()
with (out/'processor.log').open('w') as err:
    p=subprocess.Popen(['/media/ssd/hjh/perf_tools/perfetto/bin/trace_processor_shell','query','-f',str(out/'extract_context.sql'),'/media/ssd/hjh/prediction_worker_trace/sessions/pw_20261020_141904/perfetto/system.perfetto-trace'],stdout=subprocess.PIPE,stderr=err,text=True,bufsize=1)
    for line in p.stdout:
        s=line.strip()
        if s.startswith('"{') and s.endswith('}"'):
            try:
                r=json.loads(s[1:-1]);k=r['section'];handles[k].write(json.dumps(r['row'])+'\n');counts[k]+=1
            except Exception as e:errors.append(str(e))
    rc=p.wait()
for h in handles.values():h.close()
report=dict(returncode=rc,counts=counts,parse_errors=errors,elapsed_s=time.monotonic()-begin,merged_windows=merged)
(out/'status.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
