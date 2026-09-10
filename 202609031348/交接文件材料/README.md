# Apollo Perception P4 实时性异常 OS 层交接材料

## OS 层建议首先查看的数据

- 主入口：[`SYSTEM_REALTIME_ISSUE_HANDOFF.md`](SYSTEM_REALTIME_ISSUE_HANDOFF.md)
- 重点异常段：S4 F611–F649
- 异常前参考：F601–F610
- CPU Running 明显异常窗口：F611–F623；不要与完整应用层异常段 F611–F649 混为一谈
- 重点异常帧：F611、F613、F616、F619、F623
- 恢复参考：F626 及之后
- 核心五帧实际 Host TID：1016506（已由现有 identity audit 与原始 before/after TID 快照核对）
- CPU stack 已观察到的路径：Paddle allocator → `cudaMalloc` → `nvmap` → Linux page allocator / `clear_page`，以及 `nvgpu`/GMMU mapping；这里只作为已观察路径导航，不作为根因结论

- F611: [perf raw-event extract](extracted/F611_F623_core/F611_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F611_cpu_stack_raw.txt)
- F613: [perf raw-event extract](extracted/F611_F623_core/F613_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F613_cpu_stack_raw.txt)
- F616: [perf raw-event extract](extracted/F611_F623_core/F616_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F616_cpu_stack_raw.txt)
- F619: [perf raw-event extract](extracted/F611_F623_core/F619_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F619_cpu_stack_raw.txt)
- F623: [perf raw-event extract](extracted/F611_F623_core/F623_perf_sched_raw.txt) · [CPU stack raw samples](extracted/F611_F623_core/F623_cpu_stack_raw.txt)

逐帧总索引：[`index/frame_system_trace_index.csv`](index/frame_system_trace_index.csv)。缺失项与证据边界：[`MISSING_DATA.md`](MISSING_DATA.md)。

## 数据源与目录边界

本交接的数据源是 `D:\data\202608271537` 及其两个既有输出目录；`D:\data\202609031348\交接文件材料` 只是用户指定的本次交付位置。未使用同级 `202609031348` run 的 timing/perf/stack 数据。

目录说明：

- `raw/perf_sched/`：capture-wide perf sched 二进制与原始文本，逐字节复制。
- `raw/cpu_stack/`：capture-wide CPU stack 二进制、perf script/report 文本，逐字节复制。
- `raw/frame_timing/`：Perception log 与 P1–P7 原始 trace。
- `raw/tid_mapping/`：采集时钟、进程、`/proc`、Host `ps`、调度策略与 affinity 快照。
- `raw/original_reports/`：既有应用层、perf、CPU stack 分析结果与相关脚本的未修改副本。
- `extracted/`：按 normal/core/recovery 划分的 Case 窗口与逐帧导航文件。
- `index/`：Frame→Time→TID→Raw Data 索引、原始文件清单、完整性审核和本构建脚本。

## 建议阅读顺序

1. [`SYSTEM_REALTIME_ISSUE_HANDOFF.md`](SYSTEM_REALTIME_ISSUE_HANDOFF.md)
2. [`01_problem_discovery.md`](01_problem_discovery.md)
3. [`02_system_observation.md`](02_system_observation.md)
4. [`index/frame_system_trace_index.csv`](index/frame_system_trace_index.csv)
5. 对应帧的 perf/stack raw extract，再回到 capture-wide `raw/` 原始文件复核

## 原始文件完整性

本交接共复制 129 个 raw/ 文件，合计 16.62 GiB。每个文件的原绝对路径、交付相对路径、大小、mtime 和 SHA-256 见 [`raw_data_inventory.csv`](index/raw_data_inventory.csv)。复制过程不修改、格式化、删除或重排 raw/ 文件内容。

主要大文件：

- `perf.data` — 4.84 GiB; SHA256 `1c366b895dd5751cb8d93c942cbec99573e60c883ff5f995c2c470959e77b9c5`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf.data`; handoff: `raw/perf_sched/perf.data`
- `perf_sched_script.txt` — 6.44 GiB; SHA256 `61b749bf46226165b12093aa9ba20f91ec9de4393250d15c759ddf99e46f9641`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_sched_script.txt`; handoff: `raw/perf_sched/perf_sched_script.txt`
- `perf_sched_timehist.txt` — 3.97 GiB; SHA256 `9dbf02bbbb67866dea0e66d4dca2a394d590a45a475f3f28c9d34f84a3fe9696`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_sched_timehist.txt`; handoff: `raw/perf_sched/perf_sched_timehist.txt`
- `perf_sched_map.txt` — 1.31 GiB; SHA256 `966875ef50c0e03aa12863f7e3d51d99419e53964262740f0e99253974981167`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_sched_map.txt`; handoff: `raw/perf_sched/perf_sched_map.txt`
- `perf_cpu_stack.data` — 30.98 MiB; SHA256 `74fb4dd451f884fd79f6a759c4b0ef314e5c178e1b2cdf1439eed65bcc1ec900`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_cpu_stack.data`; handoff: `raw/cpu_stack/perf_cpu_stack.data`
- `perf_cpu_stack_script.txt` — 17.19 MiB; SHA256 `0b3981aee131f7122d8f419ed47886a7b311ca27367b26ed2d4ce3a7a191c636`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_cpu_stack_script.txt`; handoff: `raw/cpu_stack/perf_cpu_stack_script.txt`
- `perf_cpu_stack_report.txt` — 1.32 MiB; SHA256 `dedc54b02415616a4b2867c841edfa4a3a9551e9550b33bc050cfc1da78c2789`; original: `D:\data\202608271537\perf_run_cpu_stack_001\perf_cpu_stack_report.txt`; handoff: `raw/cpu_stack/perf_cpu_stack_report.txt`

## 关键证据边界

- P4 input-ready 是上游输出代理，不是严格 Reader queue 时间。
- `e2e_trace_v3 mono_ns` 与 Perception log `CP_INFER mono_ns` 是分开的数值基准；本交接按 source frame/sensor timestamp 关联，不直接相减，也不静默套用 perf 的 offset。
- `Running` 仅表示 CPU-resident，不能单独解释为有效计算或硬件等待。
- per-frame perf extract 是现有审计筛选后的 raw-event 行；完整 system-wide 数据须查看 capture-wide `perf_sched_script.txt` / `perf.data`。
- CPU stack sample percentage 是样本比例，不是精确 wall time 或 CPU ms。
- 本材料不重新判定根因，不把 association/correlation/observed path 改写为 causal conclusion。

## GitHub 交付方式

GitHub 仓库中保留本目录的说明、索引和 `extracted/` 结果。由于 `raw/` 合计 16.62 GiB，仓库通过 Git LFS 保存其 gzip tar 分卷，不直接提交原始目录。分卷清单、SHA-256 和恢复脚本见 [`github_bundle/README.md`](github_bundle/README.md)。恢复后的 `raw/` 内容仍以 [`index/raw_data_inventory.csv`](index/raw_data_inventory.csv) 为逐文件完整性依据。
