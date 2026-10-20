import React, {useState} from 'react';
import {EvidenceChart, DataComponent, DataTable, ReportSection, RichNarrative, useDataApp} from '../../data-app-public.jsx';
import blocks from './narrative.json';
const previews={
 'https://www.usenix.org/system/files/osdi24-ahn.pdf':{title:'BCOZ · OSDI 2024',summary:'论文联合 On/Off-CPU 观察与因果性能分析。本报告使用事件栈观察，不包含虚拟加速实验。',approvedForReport:true},
 'https://www.usenix.org/system/files/osdi24-li.pdf':{title:'Kairos · OSDI 2024',summary:'论文以时间标签传播表达数据流新鲜度、一致性及稳定性约束。本报告在已有日志上离线重建应用消息链。',approvedForReport:true},
 'https://github.com/Adkid-Zephyr/anti-defensive-writing-Skill/blob/main/skills/anti-defensive-writing/SKILL.md':{title:'用户指定的学术写作 Skill',summary:'围绕有证据支持的主要价值组织论文，明确主张与论证职责，避免按执行过程堆叠材料。',approvedForReport:true},
};
const cols=fields=>fields.map(([key,label])=>({key,label}));
export function ReportContent(){
 const {reviewedRows,appTitle,canEdit,mode,setAppTitle}=useDataApp();
 const [frame,setFrame]=useState('648');
 const rows=q=>reviewedRows(q);
 const frameRows=rows('frames');
 const selected=frameRows.filter(r=>String(r.frame)===frame);
 const workerRows=rows('workers').filter(r=>String(r.frame)===frame);
 const clean=text=>text.replace(/!\[[^\]]*\]\([^)]*\)/g,'').replace(/\[F647—F680 图集\]\([^)]*\)/g,'F647—F680 图集（配套文件目录）');
 function chart(q,title,spec,height=320){return <EvidenceChart id={'chart-'+q} queryId={q} title={title} spec={spec} rows={rows(q)} sourceRows={rows(q)} height={height}/>;}
 return <article className="report-content prediction-report" aria-label="Prediction 实时性分析报告">
  <header className="report-hero"><h1 data-data-app-title contentEditable={canEdit&&mode==='edit'} suppressContentEditableWarning onBlur={canEdit&&mode==='edit'?(e)=>setAppTitle(e.currentTarget.textContent.trim()||appTitle):undefined} onKeyDown={canEdit&&mode==='edit'?(e)=>{if(e.key==='Enter'){e.preventDefault();e.currentTarget.blur();}}:undefined}>{appTitle}</h1><RichNarrative id="report-run" value="Run `pw_20261020_141904` · 应用全量账本与 Trace 5—105 秒系统取证" className="report-deck"/></header>
  {blocks.map((b,i)=><React.Fragment key={b.id}>
   <ReportSection id={b.id} title={b.text.split('\n')[0].replace(/^#+ /,'')} queryId={b.queries[0]} queryIds={b.queries} sourceRowsByQuery={Object.fromEntries(b.queries.map(q=>[q,rows(q)]))} showHeading={false}>
    <RichNarrative id={b.id+':body'} value={clean(b.text)} sourcePreviews={previews}/>
   </ReportSection>
   {i===3&&<>
    {chart('frames','693 次 callback 的墙钟与更新 Slack（ms）',{type:'line',x:'start_s',y:'callback_ms',fields:['callback_ms','slack_ms'],stackable:false,xLabel:'Trace-relative time (s)',yLabel:'ms',valueDecimals:3})}
    {chart('bins','每 5 秒 callback 均值及最大值（ms）',{type:'line',x:'start_s',y:'mean',fields:['mean','max'],stackable:false,xLabel:'Interval start (s)',yLabel:'ms'})}
   </>}
   {i===5&&<>
    {chart('cases','每个 callback 的状态时间；各行独立闭合',{type:'horizontalStackedBar',x:'frame',y:'running_ms',fields:['running_ms','runnable_ms','sleeping_ms'],xLabel:'Frame',yLabel:'ms',valueDecimals:3},360)}
    <DataComponent id="table-frames" queryId="frames" kind="table" title="逐帧查询：保留身份和原始事件" sourceRows={selected} displayRows={selected}>
     <label className="frame-picker">Frame <select aria-label="选择 Prediction 帧" value={frame} onChange={e=>setFrame(e.target.value)}>{frameRows.map(r=><option key={r.frame_uid} value={r.frame}>F{r.frame}</option>)}</select></label>
     <DataTable rows={selected} columns={cols([['frame','Frame'],['callback_ms','Callback ms'],['response_ms','Response ms'],['slack_ms','Slack ms'],['tid','Host TID'],['trace_id','Trace ID'],['event_id_enter','Enter event']])} searchable={false} label="选定实例"/>
    </DataComponent>
    <DataComponent id="table-workers" queryId="workers" kind="table" title="选定实例的 Worker 状态" sourceRows={workerRows} displayRows={workerRows}>
     {workerRows.length?<DataTable rows={workerRows} columns={cols([['task_id','Task'],['host_tid','TID'],['task_wall_ms','Wall ms'],['running_ms','Running ms'],['runnable_ms','Runnable ms'],['sleeping_ms','Sleeping ms']])} searchable={false} label="任务状态"/>:<RichNarrative id="worker-scope-note" value="该实例不在报告精选 Worker 表中；完整 Worker 数据见项目根目录的 worker_tasks.csv 和 task_state_attribution.csv。"/>}
    </DataComponent>
   </>}
   {i===7&&<DataComponent id="table-waits" queryId="waits" kind="table" title="事件栈关联等待路径（逐 Task，非关键路径总和）" sourceRows={rows('waits')}><DataTable rows={rows('waits')} columns={cols([['frame','Frame'],['task_id','Task'],['observed_path','Observed path'],['episodes','Episodes'],['sleeping_ms','Sleeping ms'],['maximum_episode_ms','Maximum ms']])} label="等待路径证据"/></DataComponent>}
   {i===9&&<>
    {chart('intervals','输入与 Writer 返回间隔（两个端点均在窗口内）',{type:'line',x:'frame',y:'input_interval_ms',fields:['input_interval_ms','output_interval_ms'],stackable:false,xLabel:'Frame',yLabel:'ms'})}
    <DataComponent id="table-comparisons" queryId="comparisons" kind="table" title="精确观测签名匹配结果" sourceRows={rows('comparisons')}><DataTable rows={rows('comparisons')} columns={cols([['frame','Frame'],['peer_count','Peers'],['peer_meet_count','Meet peers'],['control_frame','Meet reference'],['baseline_ms','Median baseline ms'],['baseline_difference_ms','Difference ms']])} searchable={false} label="对照匹配"/></DataComponent>
   </>}
  </React.Fragment>)}
 </article>;
}
