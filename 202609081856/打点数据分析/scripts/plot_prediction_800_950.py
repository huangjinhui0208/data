"""Absolute monotonic timeline, 151 callbacks split into readable 31-row pages."""
from pathlib import Path
import csv
from PIL import Image,ImageDraw,ImageFont
RUN=Path(__file__).resolve().parents[2]
DATA=RUN/'打点数据分析/lidar_prediction_reanalysis'
OUT=DATA/'figures_800_950'
OUT.mkdir(exist_ok=True)
def read(p):
    with p.open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
rows=[r for r in read(DATA/'prediction_frames.csv') if 800<=int(r['frame_index'])<=950]
nodes=read(DATA/'prediction_node_instances.csv')
font=lambda n:ImageFont.truetype('C:/Windows/Fonts/arial.ttf',n)
colors={'upstream':'#CBD5E1','dispatch':'#F59E0B','proc':'#2563EB','thread_pool_run':'#E45756','semantic_base_async_draw':'#9B59B6'}
for begin in range(0,len(rows),31):
    page=rows[begin:begin+31]; low=int(page[0]['frame_index']);high=int(page[-1]['frame_index'])
    origin=min(int(r['lidar_enter_ns']) for r in page if r['lidar_enter_ns'])
    end=max(int(r['prediction_exit_ns']) for r in page)
    W=2400;H=220+len(page)*54;L=430;R=2200;T=130
    im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
    d.text((40,25),f'Prediction F{low}-F{high} | main LiDAR to callback completion',font=font(31),fill='#172B4D')
    d.text((40,70),f'Absolute timeline: origin mono_ns={origin}; x-axis: milliseconds from origin; no assumed deadline',font=font(22),fill='#52606D')
    x=lambda ns:L+(int(ns)-origin)/(end-origin)*(R-L)
    for tick in range(0,int((end-origin)/1e6)+1,500):
        xx=x(origin+tick*1000000);d.line((xx,T,xx,H-75),fill='#E2E8F0');d.text((xx-15,H-66),str(tick),font=font(18),fill='#52606D')
    for i,r in enumerate(page):
        y=T+i*54
        label=f"F{r['frame_index']}  in {r['input_perception_seq']}  TID {r['callback_tid']}"
        d.text((20,y+3),label,font=font(18),fill='#172B4D')
        for a,b,c in [('lidar_enter_ns','perception_output_ns','upstream'),('perception_output_ns','prediction_enter_ns','dispatch'),('prediction_enter_ns','prediction_exit_ns','proc')]:
            if r[a] and r[b]:d.rectangle((x(r[a]),y+2,max(x(r[a])+1,x(r[b])),y+19),fill=colors[c])
        for n in nodes:
            if n['frame_index']!=r['frame_index'] or n['node'] not in ('thread_pool_run','semantic_base_async_draw'):continue
            yy=y+23 if n['node']=='thread_pool_run' else y+34
            d.rectangle((x(n['enter_ns']),yy,max(x(n['enter_ns'])+1,x(n['exit_ns'])),yy+8),fill=colors[n['node']])
        d.text((x(r['prediction_exit_ns'])+6,y),f"{float(r['execution_ms']):.1f}ms",font=font(17),fill='#172B4D')
    xx=430
    for name,color in colors.items():
        d.rectangle((xx,H-28,xx+20,H-10),fill=color);d.text((xx+27,H-30),name,font=font(18),fill='#172B4D');xx+=330
    im.save(OUT/f'prediction_gantt_F{low}_F{high}.png')
with (OUT/'gantt_frame_data.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print('Rendered',len(rows),'frames across 5 pages')
