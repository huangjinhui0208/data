# D:\data 可复用脚本索引

本目录是对 `D:\data` 工作区脚本的集中归档。整理方式为**复制，不移动**：原始分析、报告和采集目录保持不变，避免破坏已有相对路径与历史可追溯性。

## 扫描与筛选范围

- 初次扫描日期：2026-08-25；CPU stack归档更新：2026-09-03
- 扫描根目录：`D:\data`
- 共发现：54 个 `.py`、2 个 `.sh`、3 个 `.ipynb`，合计 59 个脚本/笔记本
- 当前归档：15 个脚本；另复制 3 个配置/说明配套文件
- 本次新增3个CPU stack相关入口：联合采集、逐帧审计数据生成和完整调用链互斥分类

纳入标准：至少满足下列一项，并且没有被更通用版本替代。

1. 输入、输出或帧区间可通过命令行参数传入；
2. 是可被多条分析流水线导入的通用库；
3. 是可重复部署到 Apollo/CARLA 环境的采集工具；
4. 对一类文件执行稳定、与单个 run/章节无关的转换。

排除标准：写死单个 run、固定报告章节、固定历史输出目录或旧 macOS 路径；仅作为一次性验证/渲染器；被相同 SHA-256 的副本或更通用脚本替代。

## 目录结构

```text
reusable_scripts/
├── collection/
│   ├── apollo_perf_sched_collector.py
│   ├── apollo_perf_sched_cpu_stack_collector.py
│   ├── start_collect.sh
│   └── copy_carla_log.sh
├── perception/
│   ├── analyze_perception_realtime.py
│   └── plot_perception_critical_path_gantt.py
├── scheduler/
│   ├── extract_perf_sched_frame_windows.py
│   └── analyze_perf_sched_infer_frames.py
├── perf/
│   ├── generate_p4_perf_analysis.py
│   └── classify_p4_cpu_stack_samples.py
├── collision/
│   ├── collision_case_classifier.py
│   ├── collision_classifier_config.yaml
│   ├── README_collision_case_classifier.md
│   ├── realtime_collision_core.py
│   ├── config/analysis_config.yaml
│   └── tests/test_realtime_collision_core.py
├── documents/
│   ├── inspect_docx.py
│   └── make_contact_sheets.py
├── REQUIREMENTS.md
├── SOURCE_MANIFEST.csv
├── INDEX.md
└── README.md
```

## 脚本索引

### Perception逐帧分析

| 脚本 | 状态 | 用途 | 主要入口 | 依赖 |
| --- | --- | --- | --- | --- |
| [`perception/analyze_perception_realtime.py`](perception/analyze_perception_realtime.py) | 可直接复用 | 从 Perception trace/log 生成P1–P7逐帧时间、P4 waiting/execution、deadline、CenterPoint及overflow结果 | `run_id`、`--run-dir`、`--output-dir`、`--window` | numpy、pandas |
| [`perception/plot_perception_critical_path_gantt.py`](perception/plot_perception_critical_path_gantt.py) | 可直接复用 | 按任意 source-frame 区间绘制P1入口→Ground输出→P4→Fusion甘特图并导出图表数据 | `--timings`、`--ranges`、`--output-dir`；可选 source/diagnostics | numpy、pandas、Pillow、微软雅黑字体 |

分析示例：

```powershell
python perception/analyze_perception_realtime.py 202608251619 `
  --run-dir D:\data\202608251619 `
  --window all
```

未传 `--output-dir` 时，结果默认按模块保存到
`D:\data\202608251619\打点逐帧数据统计\perception数据统计`；需要兼容旧流水线时仍可显式传入
`--output-dir` 覆盖默认位置。

绘图示例：

```powershell
python perception/plot_perception_critical_path_gantt.py `
  --timings D:\data\202608251619\打点逐帧数据统计\perception数据统计\data\perception_node_frame_timings.csv `
  --source-frames D:\data\202608251619\打点逐帧数据统计\perception数据统计\data\selected_source_frames.csv `
  --p4-diagnostics D:\data\202608251619\打点逐帧数据统计\perception数据统计\data\p4_framewise_change_diagnostics.csv `
  --ranges 1-40 575-600 `
  --output-dir D:\data\202608251619\打点逐帧数据统计\perception数据统计\figures `
  --run-id 202608251619
```

Perception分析的证据边界：P2–P7缺少严格Reader receive/enqueue时间，脚本使用上一节点 `output_pub` 作为input-ready proxy；P4 execution仍是直接的 `proc_enter → output_pub`。

### Linux scheduler异常帧窗口

| 脚本 | 状态 | 用途 | 主要入口 | 依赖 |
| --- | --- | --- | --- | --- |
| [`scheduler/extract_perf_sched_frame_windows.py`](scheduler/extract_perf_sched_frame_windows.py) | 可直接复用，Linux主模式 | 根据Apollo逐帧 `mono_ns`，用 `perf sched timehist --time` 直接从 `perf.data` 提取异常帧前后小窗口；自动合并重叠窗口，并输出system-wide、目标P4 TID及全部Perception TID三种视图 | `--frame-csv`、`--perf-run-dir`、`--run-dir`、`--output-dir`；可选 `--frames`、`--before-ms`、`--after-ms`、`--clock-offset-ns` | Python 3.8标准库；直接读取perf.data需Linux、匹配的perf；已有timehist裁剪模式可在Windows运行 |
| [`scheduler/analyze_perf_sched_infer_frames.py`](scheduler/analyze_perf_sched_infer_frames.py) | 可直接复用 | 根据逐帧CenterPoint CSV与原始日志自动锁定每帧实际Infer worker TID和精确CP窗口，计算running、blocked、runnable、最大连续调度等待、迁核及 `kswapd0/kcompactd0` CPU时间 | `--centerpoint-csv`、`--perception-log`、`--raw-sched`、`--ranges`、`--clock-offset-ns`、`--output-dir`；可选真实 `--capture-start-s/--capture-end-s` 和 `--windows-only` | Python 3.8标准库；raw过滤结果必须保留目标TID switch/wakeup/migrate和内存回收线程switch事件且保持时序；最大调度等待同时计入wakeup后等待和R/R+抢占等待；CP日志与trace时钟若不同必须单独校准到perf |

Linux/Orin直接读取 `perf.data`（省略 `--frames` 时，自动选取 `execution_sudden_increase_flag=True` 的全部帧）：

```bash
python3 scheduler/extract_perf_sched_frame_windows.py \
  --perf-run-dir /path/to/perf_run_01 \
  --run-dir /path/to/202608251619 \
  --frame-csv /path/to/p4_framewise_change_diagnostics.csv \
  --dictionary /path/to/apollo_perf_sched_data_dictionary.json \
  --output-dir /path/to/perf_sched_frame_windows \
  --before-ms 100 --after-ms 100 --sudo --strict-alignment
```

若大 `perf_sched_timehist.txt` 已经存在，也可只扫描一次并在Windows生成相同的小窗口目录：

```powershell
python scheduler/extract_perf_sched_frame_windows.py `
  --perf-run-dir D:\data\202608251619\perf_run_01 `
  --existing-timehist D:\data\202608251619\perf_run_01\perf_sched_timehist.txt `
  --run-dir D:\data\202608251619 `
  --frame-csv D:\data\output\202608251619_perception_p4_framewise\data\p4_framewise_change_diagnostics.csv `
  --output-dir D:\data\output\202608251619_perf_sched_frame_windows
```

时间对齐边界：脚本使用 `perf_time_ns = Apollo_mono_ns + clock_offset_ns`。不能用采集器的Python `monotonic_ns` 自动替代Apollo trace的 `mono_ns`；默认偏移0只应在同一TID事件或其他同步点验证后使用。输出中的 `timehist_target_tid.txt` 是该帧trace记录的实际P4 worker TID，但不表示该TID永久专属于LidarDetection。

为避免重复占用磁盘，合并解码的cluster中间 `timehist_all.txt` 在拆分到 `frames/Fxxxx/` 后默认删除；调试perf解码或需要保留合并窗口时传 `--keep-cluster-files`。

逐帧分解已有原始sched报告：

```powershell
python scheduler/analyze_perf_sched_infer_frames.py `
  --centerpoint-csv D:\data\output\RUN_perception_p4_framewise\data\centerpoint_internal_timing_per_source_frame.csv `
  --perception-log D:\data\RUN\log\perception.log.INFO.TIMESTAMP.PID `
  --raw-sched D:\data\RUN\perf_run_XX\perf_sched_script.txt `
  --ranges 10-15 66-100 `
  --clock-offset-ns VERIFIED_OFFSET `
  --capture-start-s PERF_FIRST_EVENT_TIME `
  --capture-end-s PERF_LAST_EVENT_TIME `
  --output-dir D:\data\output\RUN_perf_sched_infer_analysis
```

`running`表示调度器观测到线程驻留在CPU上，其中仍可能包含GPU同步忙轮询、驱动执行或内存停顿；该脚本不能单独把running拆为有效计算和硬件等待。若raw报告先经过筛选，必须保留所有目标TID的`sched_switch/waking/wakeup/migrate`以及`kswapd0/kcompactd0`的`sched_switch`，且不能改变事件顺序。对perf采集起点之外或只被部分覆盖的帧，应保留覆盖状态而不能填0。

### 碰撞分类与实时性分析库

| 脚本 | 状态 | 用途 | 主要入口 | 依赖 |
| --- | --- | --- | --- | --- |
| [`collision/collision_case_classifier.py`](collision/collision_case_classifier.py) | 可直接复用 | 快速筛选Apollo/CARLA碰撞案例，仅消费已有日志、CSV、JSON/JSONL | `--case-dir`、`--out-dir`；可选config/target/time | numpy、pandas、PyYAML |
| [`collision/realtime_collision_core.py`](collision/realtime_collision_core.py) | 通用库 | 跨run原始日志解析、墙钟速度积分、安全与实时性分析、图表及报告底层函数 | 由其他脚本 `import`，不是独立CLI | numpy、pandas、PyYAML、matplotlib；scipy/markdown可选 |
| [`collision/tests/test_realtime_collision_core.py`](collision/tests/test_realtime_collision_core.py) | 配套测试 | 对核心库的时间解析、距离积分和分析不变量执行回归测试 | pytest | pytest及核心库依赖 |

分类器示例：

```powershell
python collision/collision_case_classifier.py `
  --case-dir D:\data\某个run `
  --out-dir D:\data\某个run\classifier_output `
  --config collision/collision_classifier_config.yaml `
  --verbose
```

配套文件：[`collision/collision_classifier_config.yaml`](collision/collision_classifier_config.yaml) 和 [`collision/README_collision_case_classifier.md`](collision/README_collision_case_classifier.md)。

核心库测试还需要 [`collision/config/analysis_config.yaml`](collision/config/analysis_config.yaml)。建议从 `collision` 目录运行测试，使 `realtime_collision_core.py` 可被直接导入：`python -m unittest discover tests`。

### 采集工具

| 脚本 | 状态 | 用途 | 运行环境/限制 |
| --- | --- | --- | --- |
| [`collection/apollo_perf_sched_collector.py`](collection/apollo_perf_sched_collector.py) | 环境绑定，可复用 | 在Orin Host采集Perception进程静态调度画像与system-wide `perf sched` | Linux、Apollo 10、`perf`、sudo；必须传 `--output-dir` |
| [`collection/start_collect.sh`](collection/start_collect.sh) | 环境绑定，非自包含 | 通过 `.collect_active` 启停 `e2e_trace_v3` 采集并整理日志 | Apollo Linux环境；当前工作区未找到其调用的 `copy_apollo_logs_by_time.sh`，部署前必须补齐 |
| [`collection/copy_carla_log.sh`](collection/copy_carla_log.sh) | 环境绑定，可复用 | 通过SSH从CARLA主机复制SCB控制时延和碰撞日志 | 默认 `scb@10.0.0.10`；路径按部署位置推导，建议放回项目 `tools/` 目录运行 |

`start_collect.sh` 和 `copy_carla_log.sh` 是部署快照，不建议直接在本Windows归档目录运行。

### 文档与图像检查

| 脚本 | 状态 | 用途 | 入口/限制 | 依赖 |
| --- | --- | --- | --- | --- |
| [`documents/inspect_docx.py`](documents/inspect_docx.py) | 可直接复用 | 将DOCX段落、表格、样式、图片关系和公式存在性导出为JSON | `python inspect_docx.py INPUT.docx OUTPUT.json` | python-docx |
| [`documents/make_contact_sheets.py`](documents/make_contact_sheets.py) | 模板型复用 | 将脚本同目录的 `page-*.png` 按2×5排版为联系表 | 当前无CLI；复制到页面PNG目录后运行 | Pillow |

## 未归档内容及原因

以下文件仍保留在原位置，并未删除或修改：

- `第二次实验/analysis_*.py`、`diagnose_*.py`：固定章节、固定run集合或固定碰撞案例。
- `output/202608171524_e2e_deadline_analysis/` 与 `output/202608241701_e2e_deadline_analysis/`：旧run专用流水线；通用Perception部分已由 `analyze_perception_realtime.py` 覆盖。
- `output/second_experiment_1131_*`、`1211_*`、`1643_*`：写死单run，部分仍包含旧macOS工作区路径。
- `report_workspace/`、`realtime_defect_report/` 顶层分析/生成/校验脚本：绑定固定run清单和具体报告产物；仅通用核心库及测试被归档。
- `output/*report*`、`single_run_realtime_diagnostic_framework_*`：与特定Markdown、模板、图片或目标文件名配套的构建脚本。
- `convert_docx_to_dingtalk_md.py`：图片映射写死DOCX block序号，不适用于任意DOCX；只归档了通用的 `inspect_docx.py`。
- `deadline_analysis/`：标题、run和证据窗口固定。
- 3个 `.ipynb`：属于分析执行记录，不作为可复用脚本归档。
- `report_workspace` 与 `realtime_defect_report` 中两份 `realtime_collision_core.py` SHA-256完全相同，仅归档一份。

## 工作区分析口径

使用归档脚本进行后续分析时，继续遵守 `D:\data\AGENTS.md`：

- 主 `D_delay` 必须是车辆速度对墙钟时间的梯形积分；不得与CARLA帧数、仿真时间或Localization位移混用。
- data/observed与model/predicted必须分开保存；实际输入缺失时标记不可用，不能用模型值补成实际结果。
- 不同run比较必须采用一致的时间基准、积分方法和端点定义。

## 同步与维护

- [`SOURCE_MANIFEST.csv`](SOURCE_MANIFEST.csv) 保存归档脚本的原路径和SHA-256。
- 原有复制归档的10个脚本源文件与归档文件哈希全部一致；scheduler窗口提取器是在通用目录中新建的规范版本。
- 从现在起建议把 `reusable_scripts` 作为通用版本入口；若仍修改历史目录中的源副本，应重新复制并更新manifest，避免两个版本静默分叉。
