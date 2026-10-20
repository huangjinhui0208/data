"""Run with matplotlib; input/output default to a report evidence directory."""
import argparse,csv,pathlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--report-dir',required=True);a=p.parse_args()
root=pathlib.Path(a.report_dir);out=root/'figures';out.mkdir(exist_ok=True)
def read(name):return list(csv.DictReader((root/'data'/name).open(encoding='utf-8-sig')))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none','figure.facecolor':'white','axes.facecolor':'white'})
manifest=[]
def save(fig,name):
 fig.tight_layout(rect=(0,0,1,.94) if fig._suptitle else (0,0,1,1))
 for ext in ['png','svg']:fig.savefig(str(out/(name+'.'+ext)),dpi=300,bbox_inches='tight')
 plt.close(fig);manifest.append(name)
r=read('frame_analysis.csv');x=[float(z['start_s']) for z in r];y=[float(z['callback_ms']) for z in r]
fig,ax=plt.subplots(2,1,figsize=(11,6),sharex=True)
ax[0].plot(x,y,'.',color='.25',ms=2.8)
for q,ls in [(.95,'--'),(.99,':')]:ax[0].axhline(np.quantile(y,q),color='.5',ls=ls,label='p%d = %.3f ms'%(q*100,np.quantile(y,q)))
ax[0].set_ylabel('Callback (ms)');ax[0].legend(loc='upper left',fontsize=8)
sl=[float(z['slack_ms']) for z in r];ax[1].scatter(x,sl,c=['#b44035' if v<0 else '.4' for v in sl],s=6);ax[1].axhline(0,c='black',lw=.8)
ax[1].set_ylabel('Next-Fusion slack (ms)');ax[1].set_xlabel('Trace-relative BOOTTIME (s)')
for aa in ax:aa.axvspan(90,95,color='.8',alpha=.25);aa.grid(axis='y',color='.9')
fig.suptitle('Prediction: 693 callbacks; next-Fusion update boundary, not a safety deadline')
save(fig,'fig01_response_slack')
st=read('major_stage_timings.csv');names=['proc','evaluator','thread_pool_run','semantic_map_run','predictor','Writer'];fr=[648,653,659,675,677]
fig,ax=plt.subplots(figsize=(10,4.5));width=.12
for j,f in enumerate(fr):
 vals=[sum(float(z['duration_ms']) for z in st if z['frame']==str(f) and z['stage_name']==n) for n in names]
 ax.bar(np.arange(len(names))+j*width,vals,width,label='F%d'%f,color=str(.15+j*.16))
ax.set_xticks(np.arange(len(names))+2*width);ax.set_xticklabels(names,rotation=15);ax.set_ylabel('Inclusive wall time (ms)');ax.legend(ncol=5,fontsize=8);ax.set_title('Nested inclusive stages; bars must not be summed');save(fig,'fig02_major_stages')
ca=read('case_callback_states.csv');fig,ax=plt.subplots(figsize=(9,4));bottom=np.zeros(len(ca))
for field,label,color in [('running_ms','Running','.15'),('runnable_ms','Runnable','.5'),('sleeping_ms','Sleeping','.8')]:
 vals=np.array([float(z[field]) for z in ca]);ax.barh(np.arange(len(ca)),vals,left=bottom,label=label,color=color);bottom+=vals
ax.set_yticks(range(len(ca)));ax.set_yticklabels(['F'+z['frame'] for z in ca]);ax.set_xlabel('Per-thread BOOTTIME duration (ms)');ax.legend(ncol=3);ax.set_title('F398: descriptive normal reference with different workload');save(fig,'fig03_callback_states')
ep=read('waiting_path_summary.csv');keys=[(648,'1826'),(648,'1828'),(648,'1830'),(648,'1832'),(675,'2360'),(675,'2361'),(675,'2362'),(675,'2363'),(675,'2366')]
fig,ax=plt.subplots(figsize=(10,4.8));bottom=np.zeros(len(keys))
for path,label,color in [('futex_PI_mutex_path','PI futex','.25'),('nvhost_syncpoint_wait_path','nvhost syncpoint','.65'),('CUDA_allocator_mutex_path','CUDA allocator mutex','.85')]:
 vals=np.array([sum(float(z['sleeping_ms']) for z in ep if z['frame']==str(f) and z['task_id']==t and z['observed_path']==path) for f,t in keys]);ax.barh(range(len(keys)),vals,left=bottom,label=label,color=color);bottom+=vals
ax.set_yticks(range(len(keys)));ax.set_yticklabels(['F%d / T%s'%k for k in keys]);ax.set_xlabel('Associated Sleeping episode time (ms)');ax.set_title('Observed pre-switch paths; exact lock/device object unresolved');ax.legend(fontsize=8,ncol=3);save(fig,'fig04_waiting_paths')
it=read('interval_stability.csv');fig,ax=plt.subplots(2,1,figsize=(11,5.7),sharex=True)
for k,label,c in [('input_interval_ms','Callback input','.2'),('output_interval_ms','Writer return','.6')]:ax[0].plot([int(z['frame']) for z in it],[float(z[k]) if z[k] else np.nan for z in it],label=label,color=c,lw=.9)
ax[0].set_ylabel('Interval (ms)');ax[0].legend();ax[1].plot([int(z['frame']) for z in it],[float(z['stability_span_ms']) if z['stability_span_ms'] else np.nan for z in it],c='.25');ax[1].set_ylabel('20-interval span (ms)');ax[1].set_xlabel('Prediction frame index');ax[1].set_title('Measured span; no formal stability threshold')
save(fig,'fig05_intervals')
sp=read('F648_thread_state_spans.csv');stage=[z for z in st if z['frame']=='648' and z['stage_name'] in ['proc','evaluator','thread_pool_run','Writer']]
zero=min(int(z['start_boot_ns']) for z in stage);fig,ax=plt.subplots(figsize=(11,5))
labels=[];ypos=0
for z in stage:
 ax.broken_barh([((int(z['start_boot_ns'])-zero)/1e6,(int(z['end_boot_ns'])-int(z['start_boot_ns']))/1e6)],(ypos-.3,.6),facecolors='.35');labels.append(z['stage_name']);ypos+=1
entities=sorted(set((z['task_id'],z['host_tid']) for z in sp))
colors={'Running':'.12','R':'.5','R+':'.5','S':'.83','D':'#b44035'}
for task,tid in entities:
 for z in sp:
  if (z['task_id'],z['host_tid'])==(task,tid):
   ax.broken_barh([((int(z['start_ns'])-zero)/1e6,int(z['duration_ns'])/1e6)],(ypos-.3,.6),facecolors=colors.get(z['state'],'white'),linewidth=0)
 labels.append(('Callback' if not task else 'Task '+task)+' / TID '+tid);ypos+=1
ax.set_yticks(range(len(labels)));ax.set_yticklabels(labels);ax.invert_yaxis();ax.set_xlabel('Aligned BOOTTIME relative to F648 callback entry (ms)')
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color='.12',label='Running'),Patch(color='.5',label='Runnable'),Patch(color='.83',label='Sleeping')],ncol=3,loc='upper center',bbox_to_anchor=(.55,-.16));ax.set_title('F648: application spans and actual thread states on one calibrated axis')
save(fig,'fig06_crosslayer_F648')
(out/'figure_manifest.json').write_text(json.dumps(dict(figures=manifest,dpi=300,formats=['png','svg'],source='data/*.csv; scripts/render_realtime_figures.py'),indent=2))
print(manifest)
