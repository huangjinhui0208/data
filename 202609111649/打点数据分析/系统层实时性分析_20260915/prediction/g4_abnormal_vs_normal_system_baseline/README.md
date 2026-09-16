# G4 abnormal-vs-normal system baseline

本批次补充提取 P99 之外的 local-jump-only 帧及匹配正常帧，用于建立实际系统状态基线。

- 异常：F184、F204、F311、F755。唯一选中原因是 `callback_previous20_median_x2_and_plus10ms`，未命中任何 `strict_p99` 规则。
- 正常：每个异常帧选择3个同run、同异步状态、相同已观测工作量字段、30帧内、非候选且 callback 不超过全run P75（6.577184 ms）的对照，共12个唯一帧。
- 本目录保存完整S1行、匹配边和选择审计；perf状态、逐帧原始窗口和比较结果保存在 `perf和stack数据分析` 下的同名目录。

正常组是为异常案例选择的低延迟匹配基线，适合回答“额外墙钟时间落在哪种系统状态”，不代表全体正常帧的无偏随机分布，也不自动形成因果结论。

主要文件：

- `selected_frames.csv`：供S5直接消费的16帧完整S1 schema。
- `selection_audit.csv/json`：角色、配对、规则、工作量摘要和校准覆盖。
- `matched_control_edges.csv`：4×3异常—正常匹配边。
- `reproduce.ps1`：选帧、独立perf提取与汇总命令。

系统结果入口：`D:\data\202609111649\perf和stack数据分析\系统层实时性分析_20260915\prediction\g4_abnormal_vs_normal_system_baseline\README.md`。
