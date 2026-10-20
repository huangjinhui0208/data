"""Render verified application spans; requires matplotlib on the render host.

prepare: local Python/numpy runtime, no plotting dependency.
render: python3 script render --output-dir PATH (Matplotlib >= 3.1).
"""
import argparse,csv,hashlib,json,math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
STAGES=[('proc','Prediction.total','#383838'),('container','Container','#9a9a9a'),
 ('perception','Perception processing','#a8c2d7'),('evaluator','Evaluator','#577f9e'),
 ('semantic_map_run','SemanticMap sync','#94a48c'),('thread_pool_run','ThreadPoolRun','#204965'),
 ('predictor','Predictor','#b4a188'),('Writer','Writer API','#a07a50'),
 ('semantic_base_async_draw','Semantic async draw','#8d7e9c')]

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def write(path,rows):
    with path.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def prepare(args):
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    frames=sorted([f for f in read(ROOT/'frame_instances.csv') if args.first_frame<=int(f['frame_index'])<=args.last_frame],key=lambda f:int(f['frame_index']))
    assert [int(f['frame_index']) for f in frames]==list(range(args.first_frame,args.last_frame+1))
    index={f['frame_uid']:f for f in frames};origin=int(frames[0]['proc_enter_ns'])
    source=read(ROOT/'frame_stage_spans.csv');allspans=[];primary=[];checks=[]
    for s in source:
        f=index.get(s['frame_uid'])
        if not f:continue
        a,b=int(s['start_ns']),int(s['end_ns']);assert b>=a
        assert abs((b-a)/1e6-float(s['duration_ms']))<1e-9
        row=dict(s,frame_index=int(f['frame_index']),callback_tid=f['callback_tid'],origin_frame=args.first_frame,origin_raw_ns=str(origin),start_relative_ms=(a-origin)/1e6,end_relative_ms=(b-origin)/1e6,duration_ms=(b-a)/1e6,displayed=s['stage_name'] in dict((k,v) for k,v,_ in STAGES))
        allspans.append(row)
        if row['displayed']:primary.append(row)
    for f in frames:
        rows=[s for s in primary if s['frame_uid']==f['frame_uid']]
        for key,_,_ in STAGES:
            ss=[s for s in rows if s['stage_name']==key]
            assert len(ss)==1,(f['frame_index'],key,len(ss))
            if key=='proc':assert int(ss[0]['start_ns'])==int(f['proc_enter_ns']) and int(ss[0]['end_ns'])==int(f['proc_exit_ns']) and ss[0]['host_tid']==f['callback_tid']
        checks.append(dict(frame_index=f['frame_index'],frame_uid=f['frame_uid'],phase_pairs=len(rows),callback_duration_ms=(int(f['proc_exit_ns'])-int(f['proc_enter_ns']))/1e6,passed=True))
    write(out/'gantt_primary_stage_data.csv',primary);write(out/'gantt_all_stage_data.csv',allspans)
    write(out/'gantt_frame_data.csv',frames)
    spec=dict(run_id=frames[0]['run_id'],first_frame=args.first_frame,last_frame=args.last_frame,origin_frame=args.first_frame,origin_raw_ns=str(origin),clock_domain='CLOCK_MONOTONIC_RAW',frames=frames,spans=primary,frames_per_page=6,stages=STAGES,validation=checks,
              source_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'frame_instances.csv',ROOT/'frame_stage_spans.csv']},
              semantics='Inclusive application wall-clock spans. Asynchronous spans preserve their own TID. Vertical order is a display grouping, not a synthesized dependency or additive decomposition.')
    (out/'plot_spec.json').write_text(json.dumps(spec,ensure_ascii=True,indent=2),encoding='utf-8')
    print(json.dumps(dict(frames=len(frames),primary_spans=len(primary),all_spans=len(allspans),origin_raw_ns=str(origin),output=str(out))))

def render(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch,Rectangle
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MultipleLocator
    out=args.output_dir;spec=json.loads((out/'plot_spec.json').read_text());frames=spec['frames'];spans=spec['spans'];first,last=spec['first_frame'],spec['last_frame']
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#666666','axes.labelcolor':'#333333','text.color':'#252525','xtick.color':'#444444','ytick.color':'#333333','axes.axisbelow':True,'savefig.facecolor':'white'})
    stagecolors={a:c for a,b,c in spec['stages']};index={(r['frame_uid'],r['stage_name']):r for r in spans};outputs=[]
    def save(fig,stem):
        for ext in ['png','svg']:
            path=out/(stem+'.'+ext);fig.savefig(str(path),dpi=300,facecolor='white');outputs.append(path.name)
        plt.close(fig)
    # Overview: shared global x coordinates, one frame group per row.
    fig,ax=plt.subplots(figsize=(16,14.2));fig.subplots_adjust(left=.19,right=.955,top=.89,bottom=.095)
    maxend=max(r['end_relative_ms'] for r in spans);ax.set_xlim(-15,math.ceil(maxend/250)*250+80)
    labels=[]
    for i,f in enumerate(frames):
        total=index[(f['frame_uid'],'proc')];a,b=total['start_relative_ms'],total['end_relative_ms']
        ax.add_patch(Rectangle((a,i-.33),b-a,.42,fill=False,edgecolor='#393939',linewidth=.8))
        e=index[(f['frame_uid'],'evaluator')];ax.broken_barh([(e['start_relative_ms'],e['duration_ms'])],(i-.29,.34),facecolors=stagecolors['evaluator'],edgecolors='none')
        for phase,yy,hh in [('thread_pool_run',i+.15,.12),('semantic_base_async_draw',i+.32,.10)]:
            r=index[(f['frame_uid'],phase)];ax.broken_barh([(r['start_relative_ms'],r['duration_ms'])],(yy,hh),facecolors=stagecolors[phase],edgecolors='none')
        ax.text(b+18,i-.10,'{:.1f}'.format(total['duration_ms']),va='center',fontsize=8.5,color='#333333')
        labels.append('F{}   TID {}'.format(f['frame_index'],f['callback_tid']))
        if i%2==0:ax.axhspan(i-.49,i+.49,color='#f5f5f5',zorder=-2)
    ax.set_ylim(len(frames)-.4,-.75);ax.set_yticks(range(len(frames)));ax.set_yticklabels(labels,fontsize=9)
    ax.tick_params(axis='y',length=0,pad=10);ax.xaxis.set_major_locator(MultipleLocator(500));ax.xaxis.set_minor_locator(MultipleLocator(100));ax.grid(axis='x',which='major',color='#dedede',linewidth=.7)
    ax.set_xlabel('Time from F{} callback entry (ms)'.format(first),labelpad=10)
    fig.text(.035,.96,'Prediction application Gantt | F{}-F{}'.format(first,last),fontsize=19,weight='bold')
    fig.text(.035,.932,'{} | 0 ms = F{} proc_enter | CLOCK_MONOTONIC_RAW'.format(spec['run_id'],first),fontsize=10.5,color='#555555')
    handles=[Patch(facecolor='white',edgecolor='#393939',label='Prediction.total'),Patch(facecolor=stagecolors['evaluator'],label='Evaluator'),Patch(facecolor=stagecolors['thread_pool_run'],label='ThreadPoolRun'),Patch(facecolor=stagecolors['semantic_base_async_draw'],label='Semantic async draw')]
    fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.185,.918),ncol=4,frameon=False,fontsize=10)
    fig.text(.035,.029,'Numbers beside total spans: callback wall time (ms). Nested spans are inclusive; do not add them. No scheduler states are shown.',fontsize=9,color='#555555')
    save(fig,'prediction_gantt_F{}_F{}_overview'.format(first,last))
    # Details: page-local zoom; all pages still use F647 as the common zero.
    n=spec['frames_per_page'];stages=spec['stages']
    for p in range(0,len(frames),n):
        page=frames[p:p+n];N=len(page);height=2.3+N*2.15
        fig,ax=plt.subplots(figsize=(16,height));fig.subplots_adjust(left=.285,right=.965,top=1-1.55/height,bottom=.8/height)
        data=[r for r in spans if r['frame_uid'] in {f['frame_uid'] for f in page}]
        lo=min(r['start_relative_ms'] for r in data);hi=max(r['end_relative_ms'] for r in data)
        ax.set_xlim(lo-5,hi+15);ticks=[];labels=[]
        for j,f in enumerate(page):
            base=j*10
            ax.axhspan(base-.5,base+8.5,color='#f5f5f5' if j%2==0 else 'white',zorder=-2)
            for k,(phase,name,col) in enumerate(stages):
                r=index[(f['frame_uid'],phase)];y=base+k;ticks.append(y)
                label=('F{}  '.format(f['frame_index']) if k==0 else '')+name+'  | '+r['host_tid']+'  | {:.3f} ms'.format(r['duration_ms']);labels.append(label)
                # Preserve actual width, including sub-millisecond stages.
                ax.broken_barh([(r['start_relative_ms'],r['duration_ms'])],(y-.29,.58),facecolors=col,edgecolors='#333333' if phase=='proc' else 'none',linewidth=.5)
                if r['duration_ms']>40:ax.text((r['start_relative_ms']+r['end_relative_ms'])/2,y,'{:.1f}'.format(r['duration_ms']),color='white' if phase in ['proc','evaluator','thread_pool_run','Writer'] else '#252525',ha='center',va='center',fontsize=8)
            ax.axhline(base+9,color='#dddddd',linewidth=.6)
        ax.set_ylim(N*10-.6,-.8);ax.set_yticks(ticks);ax.set_yticklabels(labels,fontsize=8.4);ax.tick_params(axis='y',length=0,pad=8)
        ax.xaxis.set_major_locator(MultipleLocator(100));ax.xaxis.set_minor_locator(MultipleLocator(20));ax.grid(axis='x',color='#dedede',linewidth=.65)
        ax.set_xlabel('Time from F{} callback entry (ms); page-local zoom'.format(first),labelpad=9)
        fig.text(.025,1-.40/height,'Prediction stages | F{}-F{}'.format(page[0]['frame_index'],page[-1]['frame_index']),fontsize=18,weight='bold')
        fig.text(.025,1-.77/height,'Label columns: stage | actual TID | inclusive wall time. {}. All pages share F{} = 0 ms.'.format(spec['run_id'],first),fontsize=9.5,color='#555555')
        fig.text(.025,1-1.10/height,'Semantic async draw uses its own thread and may overlap or extend beyond the callback. Vertical order is a display grouping.',fontsize=9,color='#555555')
        fig.text(.025,.17/height,'RAW origin = {} ns. Bar widths use measured endpoints; small stages are not enlarged.'.format(spec['origin_raw_ns']),fontsize=9,color='#555555')
        save(fig,'prediction_gantt_F{}_F{}_stages'.format(page[0]['frame_index'],page[-1]['frame_index']))
    (out/'render_manifest.json').write_text(json.dumps(dict(matplotlib_version=matplotlib.__version__,png_dpi=300,files=outputs,frames=len(frames),phase_spans=len(spans)),indent=2))
    print(json.dumps(dict(rendered=len(outputs),files=outputs)))

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('mode',choices=['prepare','render']);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--first-frame',type=int,default=647);ap.add_argument('--last-frame',type=int,default=680);a=ap.parse_args()
    (prepare if a.mode=='prepare' else render)(a)
