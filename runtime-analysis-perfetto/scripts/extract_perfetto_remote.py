"""One read-only Trace Processor ingestion; export only requested evidence."""
import argparse, gzip, json, pathlib, subprocess, time, hashlib

ap=argparse.ArgumentParser();ap.add_argument('request');args=ap.parse_args()
request=json.loads(pathlib.Path(args.request).read_text())
out=pathlib.Path(args.request).parent/'perfetto_export';out.mkdir(exist_ok=True)
tp='/media/ssd/hjh/perf_tools/perfetto/bin/trace_processor_shell'
trace='/media/ssd/hjh/prediction_worker_trace/sessions/pw_20261020_141904/perfetto/system.perfetto-trace'
lo,hi=request['start_ns'],request['end_ns'];sql=[];sections=[]
def emit(name,query,columns):
    obj=','.join("'%s',%s"%(x,x) for x in columns)
    sql.append("SELECT json_object('section','%s','row',json_object(%s)) AS payload FROM (%s);"%(name,obj,query))
    sections.append(name)
sql.append('CREATE PERFETTO TABLE target_threads AS SELECT t.*,p.pid FROM thread t JOIN process p USING(upid) WHERE p.pid=942621;')
vals=','.join("('%s','%s',%s,%d,%d)"%(w['frame_uid'],w['task_id'],w['host_tid'],w['start_ns'],w['end_ns']) for w in request['windows'])
sql.append('CREATE PERFETTO TABLE selected_windows AS WITH v(frame_uid,task_id,tid,a,b) AS (VALUES '+vals+') SELECT * FROM v;')
emit('thread', 'SELECT t.utid,t.tid,t.name,t.upid,p.pid,p.name AS process_name,t.start_ts,t.end_ts FROM thread t LEFT JOIN process p USING(upid)', ['utid','tid','name','upid','pid','process_name','start_ts','end_ts'])
sql.append(f'CREATE PERFETTO TABLE target_states AS SELECT s.* FROM thread_state s JOIN target_threads t USING(utid) WHERE s.ts<{hi+500000000} AND (s.dur<0 OR s.ts+s.dur>{lo-500000000});')
emit('thread_state','SELECT id,ts,dur,utid,state,cpu,waker_utid,waker_id,irq_context,io_wait,blocked_function FROM target_states ORDER BY utid,ts',['id','ts','dur','utid','state','cpu','waker_utid','waker_id','irq_context','io_wait','blocked_function'])
sql.append(f'CREATE PERFETTO TABLE target_sched AS SELECT s.* FROM sched s JOIN target_threads t USING(utid) WHERE s.ts<{hi+500000000} AND (s.dur<0 OR s.ts+s.dur>{lo-500000000});')
emit('target_sched','SELECT id,ts,dur,utid,cpu,priority,end_state FROM target_sched ORDER BY utid,ts',['id','ts','dur','utid','cpu','priority','end_state'])
sql.append('CREATE PERFETTO TABLE selected_samples AS SELECT s.* FROM perf_sample s JOIN target_threads t USING(utid) WHERE EXISTS(SELECT 1 FROM selected_windows w WHERE t.tid=w.tid AND s.ts>=w.a-100000 AND s.ts<=w.b+100000);')
emit('perf_sample','SELECT id,ts,utid,cpu,cpu_mode,callsite_id,unwind_error,perf_session_id FROM selected_samples ORDER BY ts',['id','ts','utid','cpu','cpu_mode','callsite_id','unwind_error','perf_session_id'])
emit('stack_callsites', 'WITH RECURSIVE c(id,parent_id,frame_id,depth) AS (SELECT s.id,s.parent_id,s.frame_id,s.depth FROM stack_profile_callsite s WHERE s.id IN (SELECT DISTINCT callsite_id FROM selected_samples) UNION SELECT p.id,p.parent_id,p.frame_id,p.depth FROM stack_profile_callsite p JOIN c ON c.parent_id=p.id) SELECT c.id,c.parent_id,c.frame_id,c.depth,f.name,m.name AS mapping_name,f.rel_pc FROM c JOIN stack_profile_frame f ON c.frame_id=f.id LEFT JOIN stack_profile_mapping m ON f.mapping=m.id', ['id','parent_id','frame_id','depth','name','mapping_name','rel_pc'])
sql.append(f'CREATE PERFETTO TABLE validation_samples AS SELECT s.* FROM perf_sample s JOIN target_threads t USING(utid) WHERE s.id%97=0 AND s.ts BETWEEN {lo} AND {hi};')
emit('stack_validation', 'SELECT s.id,s.ts,s.utid,s.cpu,s.perf_session_id,EXISTS(SELECT 1 FROM target_sched r WHERE r.utid=s.utid AND r.cpu=s.cpu AND r.dur>=0 AND r.ts-100000<=s.ts AND r.ts+r.dur+100000>=s.ts) AS supported_100us,EXISTS(SELECT 1 FROM target_sched r WHERE r.utid=s.utid AND r.cpu=s.cpu AND r.dur>=0 AND r.ts-2000<=s.ts AND r.ts+r.dur+2000>=s.ts) AS supported_2us FROM validation_samples s', ['id','ts','utid','cpu','perf_session_id','supported_100us','supported_2us'])
emit('wakeup', f"SELECT e.id,e.ts,e.name,e.utid AS waker_utid,EXTRACT_ARG(e.arg_set_id,'pid') AS target_tid,EXTRACT_ARG(e.arg_set_id,'target_cpu') AS target_cpu,EXTRACT_ARG(e.arg_set_id,'success') AS success FROM ftrace_event e WHERE e.name IN ('sched_waking','sched_wakeup','sched_wakeup_new') AND e.ts BETWEEN {lo} AND {hi} AND EXISTS(SELECT 1 FROM selected_windows w WHERE e.ts BETWEEN w.a-100000 AND w.b+100000) AND EXTRACT_ARG(e.arg_set_id,'pid') IN (SELECT tid FROM target_threads) ORDER BY e.ts", ['id','ts','name','waker_utid','target_tid','target_cpu','success'])
# CPU context is exported separately by extract_context_remote.py and joined
# locally after reconstructing the actual Runnable intervals. This avoids a
# correlated scan of the full scheduler table for every waiting episode.
emit('window_stats', f"SELECT name,idx,severity,source,value FROM stats WHERE value!=0 AND (severity IN ('error','data_loss') OR name GLOB '*lost*' OR name GLOB '*overrun*' OR name GLOB '*dropped*')", ['name','idx','severity','source','value'])
query='\n'.join(sql);(out/'extract.sql').write_text(query)
handles={name:gzip.open(out/(name+'.jsonl.gz'),'wt',encoding='utf-8') for name in sections};counts={name:0 for name in sections}
start=time.monotonic();errors=[]
with (out/'processor.log').open('w') as log:
    proc=subprocess.Popen([tp,'query','-f',str(out/'extract.sql'),trace],stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
    for line in proc.stdout:
        line=line.strip()
        if line.startswith('"{') and line.endswith('}"'):
            try:
                data=json.loads(line[1:-1]);name=data['section'];handles[name].write(json.dumps(data['row'],ensure_ascii=True,separators=(',',':'))+'\n');counts[name]+=1
            except Exception as e:errors.append(str(e)+':'+line[:200])
    rc=proc.wait()
for f in handles.values():f.close()
report=dict(returncode=rc,counts=counts,parse_errors=errors,elapsed_s=time.monotonic()-start,trace=trace,trace_modified=False,sql_sha256=hashlib.sha256(query.encode()).hexdigest())
(out/'extraction_status.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if rc or errors:raise SystemExit(1)
