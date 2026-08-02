# Codex 任务交接、踩坑与当前进度（始建于2026-07-15，更新至2026-07-27）

> 下一个 Codex 窗口开始工作前必须先完整阅读本文件。当前任务横跨 Apollo 10 日志分类、CARLA 碰撞目标匹配、Trace 显式时延分析、车速隐形 Deadline 实验、Bridge 时延注入及日志采集。不要根据旧聊天印象重新实现，也不要在未验证 Orin 证据链前开始批量实验。

> **2026-07-27 当前权威口径：** 本文件前面的80 km/h示例、202607151759部署排障过程、第15章的5 m主裕度和早期baseline计划均保留为历史记录。通用计算、字段命名、实际数据结果与模型结果的使用方式统一以第16章为准；第二次实验报告的写法、实际数据口径和碰撞配对分析统一以第17章为准。第16章或第17章与前面章节冲突时，以对应的最新章节为准。

## 1. 环境与目录

- Windows 工作区：`D:\data`
- Bridge 源码：`D:\data\carla_apollo10.0_bridge`
- 分析脚本：`D:\data\analysis_case`
  - 注意：目录已经是 **`analysis_case`**，不是旧文档中的拼写错误 `anlysis_case`。
- 最新案例：`D:\data\202607151759`
- Orin/Apollo 容器中的实际 Bridge 路径（用户截图确认）：
  - `/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/main.py`
  - `/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/control_delay_injector.py`
  - `/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/actor/ego_vehicle.py`
  - `/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/config/settings.yaml`
- 仿真条件：CARLA 同步模式，固定步长 `0.1 s`；用户最终计划 80 km/h 实验；传感器范围固定 50 m。
- 从真实 Trace `202607131922`测得 `/apollo/control`约 **100 Hz**，不是用户最初以为的 10 Hz。Planning 可约 10 Hz，但 ControlCommand 输出约 100 Hz。

## 2. 用户最终目标

构造并分析车速条件下的物理隐形 Deadline 实验：

1. 固定车速、地图、步长、传感器范围和障碍物场景。
2. baseline 不注入时延；另做目标总响应约 200 ms、300 ms 的固定时延组。
3. 离线证明：
   - 目标在 `t1` 达到稳定感知；
   - Apollo 功能链证据可用；
   - 第一条有效制动命令到达 Bridge、延迟释放并调用 CARLA；
   - Localization 出现持续有效减速；
   - 物理响应是否超过由车速、D1、减速度推导的安全/碰撞 Deadline；
   - 注入组相对 baseline 的碰撞率和响应分布是否显著恶化。

## 3. 已完成：功能分类与 target_id

主要文件：

- `D:\data\analysis_case\collision_case_classifier.py`
- `D:\data\analysis_case\collision_classifier_config.yaml`
- `D:\data\analysis_case\determine_target_id.py`
- `D:\data\analysis_case\TARGET_ID_DETERMINATION.md`

已完成内容：

- 碰撞时刻只取 collision event，不再把 history 第一帧当碰撞时刻。
- target_id 优先使用 CARLA 碰撞对象 `other_actor_id` 的历史轨迹与 Apollo `[FUSION_OBS]` 多帧匹配。
- 坐标转换采用：
  - `x_apollo = x_carla`
  - `y_apollo = -y_carla`
  - `vx_apollo = vx_carla`
  - `vy_apollo = -vy_carla`
  - `heading_apollo = -yaw_carla * pi / 180`
- 同时比较位置、速度、航向、类型；静止/低速目标不强依赖航向。
- Planning 证据只辅助加分，不是 target_id 必要条件，也不能掩盖位置不匹配。
- 修过一个严重时间语义问题：碰撞后的 perception 输出不能反过来作为碰撞前首次检测 target_id 的证据。
- 支持同一物理目标发生 Apollo ID 切换，输出 `physical_target_id_chain`。
- 文档中的已验证示例：`202607102138`中 CARLA actor 214 对应 Apollo target 269，ID chain `[269, 278]`。
- Prediction 已增加静态目标合法性判断，使用日志中的 `is_static/is_status` 与 `has_is_static` 语义；无轨迹但静态状态合法可以通过，显式静态状态无效会报 `PREDICTION_STATIC_STATUS_INVALID`。

重要限制：如果以后真的关闭 CARLA actor history，target_id 将失去最强的多帧真值匹配证据。当前本地 `settings.yaml`为 `collision_history_enabled: false`，但最新远端案例仍生成了 history，说明本地与实际部署配置目前不一致。

## 4. 已完成：显式固定阈值时延检测

主要文件：

- `D:\data\analysis_case\timing_anomaly_detector.py`
- `D:\data\analysis_case\timing_threshold_config.yaml`
- `D:\data\analysis_case\TIMING_ANALYSIS.md`

功能：

- 在 `t1-t2` 窗口逐帧计算 Perception、Prediction、Planning、内部 E2E。
- 先用 E2E 固定阈值找异常帧，再比较三个模块阈值做归因。
- 输出全部帧、异常帧、JSON 摘要及 E2E 散点图，超阈值点红圈标记。
- 这是 **显式固定阈值超限**，不是车速/距离推导的物理隐形 Deadline。不要混用术语。

## 5. 已完成：隐形 Deadline 分析工具

主要文件：

- `D:\data\analysis_case\implicit_deadline_analyzer.py`
- `D:\data\analysis_case\implicit_deadline_config.yaml`
- `D:\data\analysis_case\scb_calibrate.py`
- `D:\data\analysis_case\scb_group_summary.py`
- `D:\data\analysis_case\scb_experiment.example.yaml`
- `D:\data\analysis_case\main.py`
- `D:\data\analysis_case\SCB_MULTI_GROUP_EXPERIMENT.md`

### 5.1 公式和关键结论

```text
v = speed_kmh / 3.6
D_brake = v^2 / (2a)
D2_safety = D_brake + v * actuator_delay_s + safety_margin_m
D2_collision = D_brake + v * actuator_delay_s
D1_safety = D2_safety + v * target_deadline_s
D1_collision = D2_collision + v * target_deadline_s
```

80 km/h、`a=10 m/s²`、安全裕度 5 m、目标 Deadline 200 ms：

- `D_brake = 24.6914 m`
- `D1_safety = 34.1358 m`
- `D1_collision = 29.1358 m`

重要：34.1 m对应“200 ms 后仍保留 5 m安全裕度”，不是“200 ms必然碰撞边界”。在理想模型中，34.1 m对应的碰撞 Deadline 约 425 ms。若要构造约 200 ms碰撞边界，应接近 29.1 m，或保留34.1 m但让物理响应超过约425 ms。

### 5.2 分析端点

脚本严格区分：

1. Apollo Trace内部E2E；
2. Bridge收到/释放命令及CARLA API返回；
3. Localization速度序列首次出现持续有效减速（物理制动起点）。

物理隐形 Deadline 使用第3项。CARLA `apply_control()`返回不等于车辆已经产生制动力。

已修复的分析逻辑问题：

- 不再用障碍物50 m生成距离覆盖期望稳定感知D1。
- CLI `--speed-kmh/--deadline-ms/--decel`等覆盖值会真正用于案例分析，不只用于 `--calculate-only`。
- 40 km/h等低速实验的目标搜索速度门限按配置车速比例推导，不再固定要求20 m/s。
- t1之前的旧制动记录不能被错误复用为t1后的响应。
- 无碰撞安全案例的解析窗口优先参考分析结束/制动证据，不再错误选障碍物spawn时刻。
- 物理制动起点从Localization持续减速识别，默认要求至少2个连续减速间隔、最小0.5 m/s²。
- 分组汇总不会再因“每组10个空目录”误判PASS。
- 碰撞率以所有结局已知的选中实验为分母，不只统计成功分析样本。

### 5.3 D1参考系仍未完成

当前配置：

```yaml
d1_distance_reference: actor_center_to_actor_center
d1_reference_verified_for_physical_model: false
longitudinal_distance_offset_m: 0.0
```

Apollo `rel_forward`通常是actor中心距，制动公式应使用选定的碰撞净空/保险杠距离。必须实测车辆和障碍物几何修正后填写offset，并把verified设为true。在此之前脚本可输出诊断，但不会给出强因果候选，校准也会拒绝当作正式有效样本。

## 6. 已完成：Bridge碰撞记录

主要文件：

- `D:\data\carla_apollo10.0_bridge\carla_bridge\collision_sensor_logger.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\main.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\core\actor_factory.py`

实现过：

- 碰撞event CSV/JSONL。
- history环形缓冲，发生碰撞后一次性写出ego与other actor轨迹。
- history增加`wall_time_iso`等时间字段。
- 仅记录第一次碰撞的方案（`collision_first_event_only: true`）。
- history固定内存缓冲、无额外Apollo网络流量。

用户后来明确表示暂时不需要history，因此本地配置目前是：

```yaml
collision_history_enabled: false
```

但有两个不一致需要后续决定：

1. 最新案例`202607151759`仍有`carla_collision_actor_history_*.csv`，说明远端实际配置可能仍为true或未部署本地settings。
2. `D:\data\copy_carla_log.sh`原逻辑仍把actor history当必需文件；如果远端真正关闭history，拷贝会失败。

## 7. 已完成：Bridge固定控制时延注入与SCB CSV

主要文件：

- `D:\data\carla_apollo10.0_bridge\carla_bridge\control_delay_injector.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\actor\ego_vehicle.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\main.py`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\config\settings.yaml`
- `D:\data\carla_apollo10.0_bridge\carla_bridge\tools\test_control_delay_injector.py`

### 7.1 当前本地配置

```yaml
fixed_delta_seconds: 0.1
collision_history_enabled: false
control_delay_injection:
  enabled: true
  delay_ms: 0.0
  activation_speed_mps: 5.0
  brake_threshold_percentage: 1.0
  queue_max_messages: 64
  log_all_delayed_commands: false
  log_dir: "/apollo/data/log"
  log_csv: "scb_control_delay_{wall_time_iso}.csv"
```

### 7.2 触发和记录语义

- `enabled=true`：主入口读取配置后创建CSV，和车速/制动无关。
- `BRIDGE_CONFIG_LOADED`：证明main/settings/injector路径、PID、cwd和配置。
- `INITIALIZED`：只有更新后的`EgoVehicle`创建`ControlDelayInjector`后才追加。
- 车辆曾达到`activation_speed_mps`后锁存ARMED；后续第一条`brake >= brake_threshold_percentage`触发注入。
- 第一条有效制动无论`log_all_delayed_commands`是否为false，都必须写`APPLIED, first_effective_brake=1`。
- `log_all_delayed_commands=false`时不逐条写后续正常命令，但队列溢出/执行失败仍记录。
- 触发后所有后续ControlCommand按固定延迟、保持顺序，由daemon worker执行；Cyber回调不sleep。
- `delay_ms=0`触发后仍会经过复制、队列和worker，因此baseline有很小但非零的代码路径开销。
- 100 Hz下200 ms约20条、300 ms约30条；队列上限64，内存有界。

CSV当前schema为`scb_control_delay_v3`。关键字段：receive、release、CARLA API call start/end、requested/actual delay、brake%、ego speed、CARLA frame/time、源码路径和状态。

### 7.3 main.py不可再踩的坑

用户明确要求：**不改动/删除/移动原始main内容，只允许添加。**

当前已与Git原版自动对比：`MAIN_ORIGINAL_LINES_REMOVED=0`。原始以下语句和位置已恢复：

```python
carla_bridge = CarlaCyberBridge()
carla_parameters = parameters["carla"]
self.carla_parameters = params["carla"]
```

SCB通过新增 `_prepare_scb_control_delay_evidence(...)`旁路调用接入，不能再用替换原赋值或移动构造顺序的方式修改。

曾发生过真实部署事故：只更新了新版`main.py`，远端`control_delay_injector.py`较旧，启动报：

```text
AttributeError: type object 'ControlDelayInjector' has no attribute
'normalize_bridge_parameters'
```

当前main已经不再调用该方法，并对缺少`prepare_startup_evidence`的旧注入器打印版本不匹配，不再因该属性崩溃。但是正式实验仍必须把main、injector、ego_vehicle、settings作为同一版本整体部署。

## 8. 最新真实案例：202607151759（最重要的当前状态）

目录：`D:\data\202607151759`

已经成功收集：Apollo日志、Trace、碰撞event、actor history和：

```text
D:\data\202607151759\log\scb_control_delay_20260715175803_413384.csv
```

SCB文件已经证明启动落盘和复制脚本工作，内容为schema v3，路径指向：

```text
bridge_entry_file=/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/main.py
settings_source_file=/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/config/settings.yaml
injector_source_file=/apollo_workspace/modules/carla_apollo10.0_bridge/carla_bridge/control_delay_injector.py
activation_speed_mps=5.0
brake_threshold_percentage=1.0
requested_delay_ms=0.0
status=BRIDGE_CONFIG_LOADED
```

但该文件 **只有这一行数据**，没有：

- `INITIALIZED`
- `APPLIED, first_effective_brake=1`

当前严格分析函数实测返回：

```python
{'status': 'MISSING',
 'reason': 'SCB_CONTROL_DELAY_EVIDENCE_NOT_FOUND',
 'source_file': None}
```

这意味着：

- CSV“启动即创建”问题已经解决；
- 第一条有效制动证据链仍未打通；
- `202607151759`目前不能作为正式隐形Deadline样本。

结合该案例成功生成collision history（该logger只在识别EgoVehicle后挂载），最可能原因是远端`actor/ego_vehicle.py`仍是旧版，没有实例化`ControlDelayInjector`，或者没有与当前injector同步部署。下一步必须先在Orin检查：

```bash
cd /apollo_workspace/modules/carla_apollo10.0_bridge
grep -n "ControlDelayInjector" carla_bridge/actor/ego_vehicle.py
grep -n "def prepare_startup_evidence" carla_bridge/control_delay_injector.py
grep -n "normalize_bridge_parameters" carla_bridge/main.py
```

预期：

- `ego_vehicle.py`能找到import、构造、submit和destroy调用；
- injector能找到`prepare_startup_evidence`；
- main中不应再出现对`normalize_bridge_parameters()`的调用。

然后彻底停止旧Bridge进程，整体覆盖4个文件并重启。不要只复制main或settings。

### 8.1 新增已验证案例：202607171107 的全生命周期时延

目录：`D:\data\202607171107`

该案例已经出现完整SCB证据：`BRIDGE_CONFIG_LOADED`、`INITIALIZED`和第一条`APPLIED`。第一条有效制动命令为`brake=29.6084%`，Bridge接收时车速`19.3945 m/s`，配置注入时延`0 ms`，实际worker/API完成开销约`7.249 ms`。因此该案例可以用于验证Apollo Trace、Bridge执行和Localization物理响应三类端点。

#### 8.1.1 统计口径和异常值清洗

- 全生命周期采集窗口：`2026-07-17 11:07:53`至`11:08:16`。
- 完整E2E口径：原始点云`data_ts_ns`到第一条继承相同Trace ID的`/apollo/control`输出。
- Apollo内部E2E口径：PointCloud Preprocess的`proc_enter`到Planning的`output_pub`。
- 对每项指标独立删除高于`Q3 + 1.5 * IQR`的异常极大值；低值不删除。
- 共208个Fusion帧，其中199帧完整关联到Planning和Control；9帧为不完整Trace，不参与完整E2E统计。

#### 8.1.2 全生命周期E2E平均值

| 链路口径 | 样本数 | 原始平均 | 剔除数 | 清洗后平均 |
|---|---:|---:|---:|---:|
| 点云时间戳 -> Fusion输出 | 201 | 528.188 ms | 0 | 528.188 ms |
| 点云时间戳 -> Planning输出 | 199 | 560.896 ms | 0 | 560.896 ms |
| 点云时间戳 -> 第一条对应Control输出 | 199 | 567.415 ms | 0 | **567.415 ms** |
| Preprocess开始 -> Planning输出（Apollo内部E2E） | 199 | 424.254 ms | 0 | 424.254 ms |

完整点云到Control E2E最大值为`871.290 ms`，IQR高异常阈值为`1113.199 ms`，因此该指标没有样本被判定为孤立异常极大值。较高时延主要来自持续性的感知排队，而不是少量尖峰。

#### 8.1.3 Apollo主要模块平均值

| 模块/阶段 | 原始平均 | 剔除数 | 清洗后平均 |
|---|---:|---:|---:|
| 点云时间戳 -> Preprocess入口（传输/入口等待） | 136.655 ms | 6/201 | **135.858 ms** |
| Perception完整内部链路 | 391.533 ms | 0/201 | **391.533 ms** |
| Prediction本地处理 | 4.610 ms | 12/207 | **4.238 ms** |
| Planning本地处理 | 24.780 ms | 10/206 | **21.164 ms** |
| Control本地处理 | 1.212 ms | 53/2224 | **1.179 ms** |

Guardian不计入该因果链，因为当前Bridge直接读取Control命令。

#### 8.1.4 Perception子模块清洗后平均值

| 子模块 | 原始平均 | 剔除数 | 清洗后平均 |
|---|---:|---:|---:|
| PointCloud Preprocess | 5.711 ms | 12/202 | 5.220 ms |
| Map Based ROI | 7.445 ms | 14/202 | 6.901 ms |
| Ground Detection | 13.936 ms | 14/203 | 13.201 ms |
| Lidar Detection纯计算 | 103.880 ms | 7/207 | **100.198 ms** |
| Detection Filter | 0.271 ms | 23/208 | 0.226 ms |
| Lidar Tracking | 0.543 ms | 19/208 | 0.384 ms |
| Multi Sensor Fusion | 2.774 ms | 9/208 | 2.520 ms |

感知子模块清洗后的本地计算合计约`128.65 ms`，而Perception完整内部链路平均为`391.53 ms`；两者相差约`262.88 ms`，主要是队列等待、线程调度和模块交接，不能把`Lidar Detection约100 ms`直接当成完整感知E2E。

#### 8.1.5 碰撞关键帧的单次因果时延（不得与全生命周期平均混用）

目标ID 133的首次稳定观测点云时间为`11:08:08.363017`：

| 端点 | 相对首次观测的单次累计时延 |
|---|---:|
| Fusion输出 | 641.974 ms |
| Planning首次停车输出 | 662.824 ms |
| Control制动命令header | 672.249 ms |
| Bridge收到命令 | 678.140 ms |
| Bridge完成`apply_control()` | 685.404 ms |
| Localization确认持续减速 | **841.753 ms** |

该关键帧的Lidar Detection由`queue_wait=530.3 ms`和`proc=108.5 ms`组成，总计`638.8 ms`。车辆真正持续减速时车速约`19.070 m/s`、碰撞净空约`25.36 m`；实测等效减速度约`4.08 m/s²`，最终以`12.522 m/s`发生碰撞。全生命周期点云到Control平均`567.415 ms`描述整次run的总体性能，而关键帧点云到物理减速`841.753 ms`才是该次碰撞物理Deadline分析使用的响应值。

## 9. 已完成：SCB文件拷贝脚本

文件：`D:\data\copy_carla_log.sh`

最小修改内容：

- 保留原三份碰撞文件复制逻辑和目标目录。
- 新增查找`scb_control_delay_*.csv`。
- 因SCB文件名是Bridge启动时刻、碰撞文件名是碰撞时刻，选择“不晚于碰撞时刻的最新SCB文件”。
- 加入存在性检查，并与三份碰撞文件一起SCP到同一`${dst_dir}`。
- 最新案例证明SCB文件已成功被拷到案例`log`目录。

仍存在的风险：

- 如果本次run没有生成SCB，但远端残留上一次文件，当前“碰撞前最新”规则可能选到旧run。正式使用前应进一步用session start/end、PID或CSV内部时间验证同一次run。
- 当前脚本只检查文件存在，不检查是否含`INITIALIZED/APPLIED`。最新案例正好说明“复制成功不等于证据完整”。
- 当前脚本仍强制要求actor history；若真正关闭history会失败。
- Windows工作区没有可用Bash/WSL，修改后只做了UTF-8和静态结构检查；真实SSH/SCP语法由Linux运行环境最终验证。最新案例已间接证明当前版本至少成功复制了SCB文件，但仍需确认它是否就是刚修改的脚本版本。

## 10. 测试状态

使用Codex工作区Python（系统`D:\python\python.exe`缺NumPy，不能用于完整分析测试）：

```text
C:\Users\22142\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe
```

最近通过：

- `analysis_case`：18/18测试通过。
- Bridge injector：6/6测试通过。
- Bridge/analysis关键Python文件`py_compile`通过。
- `main.py`相对Git原版删除行数：0。

Bridge单测命令：

```powershell
$env:PYTHONPATH='D:\data\carla_apollo10.0_bridge'
& 'C:\Users\22142\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' `
  -m unittest carla_bridge.tools.test_control_delay_injector -v
```

分析测试命令：

```powershell
cd D:\data\analysis_case
& 'C:\Users\22142\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' `
  -m unittest discover -s tests -p 'test_*.py' -v
```

测试边界：本地没有Apollo Cyber/CARLA运行时，只能做注入器单测、语法和离线分析测试；完整Bridge入口必须在Orin容器验证。

## 11. 多组实验计划（尚未开始正式执行）

证据链修好后：

1. 先做3次pilot：0 ms、已知100 ms、再次0 ms，验证可逆性和每次都有完整SCB三类记录。
2. 初始校准10次：`enabled=true, delay_ms=0`，初始障碍物50 m生成。
3. 每个案例保存`scb_experiment.yaml`、实际settings快照、bridge.log、Apollo日志、Trace、SCB CSV和碰撞结局。
4. 使用`scb_calibrate.py`汇总检测滞后、baseline物理响应、保守减速度和推荐D1。
5. 测量D1中心距到净空距离修正，固定正式D1。
6. 正式baseline至少10次。
7. 200 ms/300 ms目标组先各3次pilot，只按组调整固定`delay_ms`，不能逐帧自适应。
8. 每个正式组至少10次，再用`scb_group_summary.py`比较响应、deadline miss和碰撞率。

不要简单使用`目标总E2E - baseline内部Trace E2E`当作Bridge注入值。实验关注的是t1到物理持续减速的总响应，应通过pilot测得固定Bridge delay与实际物理响应的关系。

## 12. 未完成任务（按优先级）

1. **最高优先级：修通`INITIALIZED/APPLIED`。** 核对并整体部署更新后的`ego_vehicle.py`，重启Bridge，只跑一次短pilot；SCB必须至少有`BRIDGE_CONFIG_LOADED`、`INITIALIZED`、第一条`APPLIED`。
2. 验证`APPLIED`中的`requested_delay_ms/actual_delay_ms/brake_percentage/ego_speed`和Localization持续减速能被`implicit_deadline_analyzer.py`识别。
3. 决定是否真的关闭collision history；同时统一Bridge settings、target_id方法和`copy_carla_log.sh`对history的要求。
4. 加固拷贝脚本的同run匹配，并考虑复制后检查CSV含`APPLIED`。
5. 修正文档中的旧目录`D:\data\anlysis_case`为`D:\data\analysis_case`、旧schema v2为v3、Bridge示例速度7.5为当前或正式实验值。
6. 测量并配置D1距离参考/offset，将`d1_reference_verified_for_physical_model`设为true。
7. 为每次run生成实际`bridge_settings.yaml`和`bridge.log`快照；当前案例目录尚未包含这两项。
8. 证据链验证后才开始10次校准、baseline、200 ms、300 ms正式实验。
9. 尚未在真实Orin环境做CPU/内存profiling；目前只有结构分析：启动CSV只写少量行，`log_all=false`，队列64有界，不会100 Hz持续落盘，但delay=0触发后仍有worker开销。

## 13. 工作区安全提醒

- Bridge Git工作区非常脏，包含大量用户已有修改、生成日志、pycache、地图/传感器文件和未跟踪文件。
- 不要执行`git reset --hard`、`git checkout --`或清理未跟踪文件。
- 不要把全仓`git diff --check`失败误认为本任务代码失败；无关`objects.json/lidar.py/radar.py`已有尾随空格。
- 只检查和修改当前任务明确涉及的文件。
- 用户特别在意token浪费和反复承诺；下一窗口应先读取真实最新文件/运行证据，再下结论，不要再把“没有SCB”归因于车速阈值。

## 14. 下一窗口建议的第一步

不要立刻改分析脚本。先让用户在Orin执行并返回以下输出，或读取其下一次同步到`D:\data`的新案例：

```bash
cd /apollo_workspace/modules/carla_apollo10.0_bridge

grep -n "ControlDelayInjector" carla_bridge/actor/ego_vehicle.py
grep -n "def prepare_startup_evidence" carla_bridge/control_delay_injector.py
grep -n "normalize_bridge_parameters" carla_bridge/main.py

ps -ef | grep '[c]arla_bridge/main.py'
```

整体部署后，启动终端应显示SCB配置/证据路径。短场景结束后检查：

```bash
head -n 5 /apollo/data/log/scb_control_delay_*.csv
grep -n 'INITIALIZED\|APPLIED' /apollo/data/log/scb_control_delay_*.csv
```

只有看到`APPLIED, first_effective_brake=1`，才能继续隐形Deadline脚本与批量实验。

## 15. 2026-07-18 当前车速隐形 Deadline 计算与数据使用规范（权威）

本章用于指导后续窗口直接分析 baseline 和时延注入实验。当前目标不是测量真实车辆 CANBus/制动执行器延迟，而是在固定 CARLA–Apollo–Bridge 实验平台中，计算从障碍物稳定感知到车辆出现可观测持续减速的闭环响应，并利用制动距离推导安全 Deadline 和碰撞 Deadline。

当前固定实验条件：

```text
CARLA版本：0.9.15，服务器侧
Apollo版本：10.0.0，Orin侧
Bridge位置：服务器侧
Guardian：未接入，Bridge直接读取/apollo/control
CARLA synchronous_mode：true
fixed_delta_seconds：0.1 s，保持不变
当前baseline实际t1车速：约15.6 m/s
当前稳定感知中心距离：约45.3 m
当前稳定感知车头净距：约40.0 m
安全余量：5 m
```

完整六组 baseline 报告及图表位于：

```text
D:\data\baseline\VEHICLE_SPEED_HIDDEN_DEADLINE_BASELINE.md
D:\data\baseline\figures\
```

六组数据目录：

```text
D:\data\baseline\202607171703
D:\data\baseline\202607171706
D:\data\baseline\202607171721
D:\data\baseline\202607171726
D:\data\baseline\202607171735
D:\data\baseline\202607171738
```

### 15.1 三类时间端点必须分开

#### 15.1.1 稳定感知时刻 t1

`t1` 定义为目标障碍物连续至少 3 帧稳定出现时，第一帧的源观测时间或点云时间。

要求：

- 使用 source observation/point-cloud timestamp；
- 不使用 Fusion 发布时刻代替 `t1`；
- 不使用 CARLA 障碍物 spawn 时刻代替 `t1`；
- 必须先确认三帧属于同一物理障碍物；若 Apollo ID 切换，使用 `physical_target_id_chain`；
- `t1` 对应的 D1 和车速必须从同一时刻或最近 Localization 样本取得。

#### 15.1.2 Control 有效制动命令时间

Control 时间定义为：Apollo Control 首次发布由当前目标障碍物引起的持续有效制动命令的时间。

它用于分析 Apollo 软件链路：

$$
T_{t_1\rightarrow\mathrm{Control}}
=t_{\mathrm{Control}}-t_1
$$

不能简单采用全场景第一条 `brake>0` 的命令，因为正常速度调节可能在障碍物出现前产生制动。应结合以下证据选择：

```text
t1已经出现
目标障碍物连续稳定存在
Planning对该目标产生STOP/制动约束
Control随后连续输出明显制动
```

Control 时间只用于链路诊断，不替代当前实验的闭环物理响应时间。原因是时延注入发生在 Control 发布之后，只看 `t1→Control` 无法反映 Bridge 注入对车辆结果的影响。

#### 15.1.3 Localization 可观测持续减速时间 t2

`t2` 定义为：在 Localization 速度序列中，首次观测到车辆连续出现有效减速度的时间。

速度为：

$$
v_i=\sqrt{v_{x,i}^2+v_{y,i}^2+v_{z,i}^2}
$$

相邻区间的制动减速度取正值：

$$
a_i=\frac{v_{i-1}-v_i}{t_i-t_{i-1}}
$$

默认判据：

$$
a_i\ge0.5\ \mathrm{m/s^2}
$$

并且连续至少 2 个相邻区间满足该条件。第二个区间只负责确认减速具有持续性，`t2` 回记为第一个有效减速区间的结束时间。

当前实验的闭环响应时间为：

$$
T_{\mathrm{response,observed}}=t_2-t_1
$$

该端点应称为“Localization 可观测持续减速起点”或“实验平台闭环物理响应”，不要描述成真实车辆制动器内部的毫秒级起点。

### 15.2 为什么当前实验继续使用 t1 到 Localization 持续减速

该估计对于当前车速隐形 Deadline 仿真实验是合理的，原因如下：

1. 时延注入发生在 Control 之后，`t1→Control` 不会随注入值变化，不能单独评价注入效果。
2. 碰撞、停车距离和最小净距都发生在同一个 CARLA 平台中；Bridge/CARLA 固定执行因素同时作用于 baseline 和注入组。
3. `t1→Localization持续减速`能够反映从稳定感知、Apollo处理、Bridge注入到车辆真正出现运动变化的总闭环效果。
4. Control 时间、Bridge release/apply 时间仍应保存，用于定位时延来自哪个阶段，但不替代闭环响应端点。
5. 当前不需要为了 Deadline 主实验增加高频逐 tick 磁盘日志；保持平台负载一致比追求虚假的毫秒级物理起点更重要。

必须同时承认分辨率限制：六组 baseline 的 Localization 实际约为 `8.7–9.2 Hz`，典型采样间隔约 `110–112 ms`。因此单次 `t2` 是约一个 Localization 周期分辨率下的观测值。

建议报告：

```text
平均观测闭环响应约344 ms
Localization典型时间分辨率约110 ms
```

不要把 `343.657 ms`解释成精确到 `0.001 ms` 的真实物理时间。三位小数只用于脚本内部计算和复核。

### 15.3 各类数据的用途

| 数据/日志 | 主要用途 | 不应用来做什么 |
|---|---|---|
| Perception/Fusion日志 | 确定目标ID、连续稳定帧、t1、目标位置和D1 | 不用Fusion发布时间代替源观测t1 |
| PointCloud/Trace | 获得源点云时间、Trace ID、感知排队和模块处理时延 | 不把全生命周期平均E2E直接当碰撞关键帧响应 |
| Prediction日志 | 验证目标状态和静态障碍物语义 | 静态目标没有预测轨迹时不能直接判异常 |
| Planning日志 | 证明STOP/制动决策针对当前目标，排除普通速度调节制动 | 不用Planning时刻代替车辆持续减速时刻 |
| Control日志/Trace | 确定首次相关有效制动命令、计算t1到Control | 不单独用于评价Bridge注入后的闭环结果 |
| scb_control_delay CSV | 验证配置、触发、实际release/apply时延、队列和状态 | `apply_control()`返回不等于车辆物理减速 |
| Localization日志 | 取得t1车速、车辆轨迹、t2、停车位置和制动距离 | 不宣称具有高于约110 ms的物理时间精度 |
| CARLA碰撞event | 判定是否碰撞、碰撞对象、碰撞帧和碰撞速度 | 不能用history第一帧当碰撞时刻 |
| CARLA actor history（若有） | 校准目标身份、中心距到净距、碰撞前轨迹 | 当前关闭时不能假设目录一定存在 |
| bridge/settings快照 | 证明每组fixed_delta、delay、队列和触发条件一致 | 缺少快照时不能把不同run强行当单变量对照 |

### 15.4 单个场景的完整计算步骤

#### 步骤1：验证实验配置和结局

记录并核对：

```text
场景目录
Bridge/settings版本
fixed_delta_seconds=0.1
请求注入时延
SCB实际注入时延
点云配置与点数状态
是否碰撞
碰撞对象与碰撞速度
```

若配置缺失、时延注入状态不明、目标身份不明或结局未知，应标记为数据不足，不能强行纳入正式统计。

#### 步骤2：确定目标障碍物

优先级：

```text
CARLA碰撞对象/history多帧匹配
→ Apollo Fusion多帧位置/速度/类型匹配
→ Planning stop by目标ID辅助验证
```

记录目标ID和可能的 `physical_target_id_chain`。

#### 步骤3：确定 t1

找到目标连续至少 3 帧稳定存在的第一帧，取该帧源观测时间：

$$
t_1=t_{\mathrm{source,first\ stable\ frame}}
$$

同时保存：

```text
v1：t1时刻自车速度
D1_center：自车中心到障碍物中心的纵向距离
D1_clear：自车前端到障碍物近端的净距
```

#### 步骤4：统一 D1 距离参考系

当前六组 baseline 暂用同一车辆碰撞场景标定的几何偏移：

$$
L_{\mathrm{offset}}=5.3074\ \mathrm{m}
$$

$$
D_{1,\mathrm{clear}}
=D_{1,\mathrm{center}}-L_{\mathrm{offset}}
$$

该 offset 仍属于基于现有碰撞场景的暂定值。若以后补充准确 actor bounding box/contact history，应重新校准，不能同时混用中心距和净距。

#### 步骤5：确定 Control 有效制动命令

在 `t1` 之后结合 Planning 目标约束，找到 Control 首次持续有效制动命令，计算：

$$
T_{t_1\rightarrow\mathrm{Control}}
=t_{\mathrm{Control}}-t_1
$$

该指标用于拆分 Apollo 软件响应，不作为主 Deadline 结局。

#### 步骤6：确定 Localization 持续减速 t2

按第 15.1.3 节的速度差分和连续 2 区间判据寻找 `t2`，计算：

$$
T_{\mathrm{response,observed}}=t_2-t_1
$$

并计算 Control 发布后的剩余闭环时间：

$$
T_{\mathrm{Control\rightarrow physical}}
=t_2-t_{\mathrm{Control}}
$$

该差值包含 Bridge、CARLA tick、车辆动力学以及 Localization 采样相位，不能单独命名为“Bridge时延”或“执行器时延”。六组 baseline 的均值约为 `77.028 ms`，小于一个 Localization 周期，因此不得对其做过度的毫秒级物理解读。

#### 步骤7：计算实际制动距离和等效减速度

对于能够停车的场景，从 `t2` 开始到第一次接近静止位置，计算车辆沿行驶方向的实际制动距离：

$$
D_{\mathrm{brake,actual}}
=s_{\mathrm{first\ stop}}-s(t_2)
$$

以 `t2` 附近的制动初速度 `v2` 反推等效减速度：

$$
a_{\mathrm{eq}}
=\frac{v_2^2}{2D_{\mathrm{brake,actual}}}
$$

如果场景发生碰撞而没有完整停车过程，不能用碰撞后的不完整距离反推等效减速度，应使用同配置 baseline 的保守减速度或中位数。

#### 步骤8：计算理论制动距离

隐形 Deadline 模型假设车辆在等待期间近似保持 `t1` 速度 `v1`，开始制动后使用等效减速度：

$$
D_{\mathrm{brake}}
=\frac{v_1^2}{2a_{\mathrm{eq}}}
$$

#### 步骤9：计算安全和碰撞 Deadline

当前安全余量：

$$
D_{\mathrm{margin}}=5\ \mathrm{m}
$$

带安全余量的最晚制动距离：

$$
D_{2,\mathrm{safe}}
=D_{\mathrm{brake}}+D_{\mathrm{margin}}
$$

带安全余量的总隐形 Deadline：

$$
T_{\mathrm{deadline,safe}}
=\frac{D_{1,\mathrm{clear}}-D_{\mathrm{brake}}-D_{\mathrm{margin}}}{v_1}
$$

只以刚好不碰撞为边界：

$$
T_{\mathrm{deadline,collision}}
=\frac{D_{1,\mathrm{clear}}-D_{\mathrm{brake}}}{v_1}
$$

这里不再额外加入 actuator delay，因为当前主响应 `t1→Localization持续减速` 已经包含从软件输出到平台出现物理减速的闭环时间；重复加入会双重计算。

#### 步骤10：计算可继续注入的时延

带 5 m 安全余量：

$$
\Delta T_{\mathrm{inject,safe}}
=T_{\mathrm{deadline,safe}}
-T_{\mathrm{response,baseline}}
$$

只以碰撞为边界：

$$
\Delta T_{\mathrm{inject,collision}}
=T_{\mathrm{deadline,collision}}
-T_{\mathrm{response,baseline}}
$$

若结果小于或等于 0，表示 baseline 本身已经越过对应边界，不应继续增加时延。

#### 步骤11：注入组重新计算而不是机械相加

固定注入值只用于实验设置。每个注入 run 仍必须重新从日志测量：

```text
t1
v1
D1_clear
Control有效制动时间
Localization持续减速t2
实际t1到t2
最小净距
是否碰撞
碰撞速度
SCB实际注入时延
```

线性估计：

$$
T_{\mathrm{response,predicted}}
\approx T_{\mathrm{response,baseline}}
+\Delta T_{\mathrm{inject}}
$$

只用于设计时延档位，不能替代注入后的实测值。CARLA 0.1 s步长会带来 tick 相位量化，实际物理响应不一定严格增加相同毫秒数。

### 15.5 六组 baseline 的当前结果

六组零注入 baseline 汇总：

| 指标 | 当前结果 |
|---|---:|
| t1实际车速均值 | 15.689 m/s（约56.5 km/h） |
| t1实际车速中位数 | 15.596 m/s |
| D1中心距离均值 | 45.283 m |
| D1净距均值 | 39.975 m |
| t1到Fusion均值 | 236.519 ms |
| t1到Control均值 | 266.629 ms |
| t1到Localization持续减速均值 | 343.657 ms（报告时约344 ms） |
| t1到Localization持续减速P90 | 359.821 ms |
| Control到Localization持续减速均值 | 77.028 ms |
| 等效减速度均值 | 4.957 m/s² |
| 实际制动距离均值 | 24.876 m |
| 最终净距估计均值 | 10.480 m |

使用六组参数均值代入公式：

$$
v_1=15.6885\ \mathrm{m/s}
$$

$$
D_{1,\mathrm{clear}}=39.975\ \mathrm{m}
$$

$$
a_{\mathrm{eq}}=4.9567\ \mathrm{m/s^2}
$$

$$
D_{\mathrm{brake}}
=\frac{15.6885^2}{2\times4.9567}
\approx24.828\ \mathrm{m}
$$

$$
T_{\mathrm{deadline,safe}}
=\frac{39.975-24.828-5}{15.6885}
\approx646.796\ \mathrm{ms}
$$

$$
T_{\mathrm{deadline,collision}}
=\frac{39.975-24.828}{15.6885}
\approx965.500\ \mathrm{ms}
$$

$$
\Delta T_{\mathrm{inject,safe}}
=646.796-343.657
=303.139\ \mathrm{ms}
$$

$$
\Delta T_{\mathrm{inject,collision}}
=965.500-343.657
=621.843\ \mathrm{ms}
$$

注意两种汇总方法略有差异：

```text
先逐场景计算再取平均：安全Deadline 639.648 ms，可注入安全时延295.991 ms
先取参数均值再代入公式：安全Deadline 646.796 ms，可注入安全时延303.139 ms
```

正式统计应优先“逐场景计算再汇总”；均值参数公式主要用于解释和选择下一组时延。

### 15.6 六组逐场景的可注入时延

| 场景 | 可注入安全时延 (ms) | 可注入至碰撞边界 (ms) | 备注 |
|---|---:|---:|---|
| 1703 | 468.134 | 788.963 | 正常baseline |
| 1706 | 54.675 | 360.255 | t1车速16.362 m/s，速度条件异常且净距最低 |
| 1721 | 319.081 | 637.602 | 接近300 ms安全临界 |
| 1726 | 479.539 | 802.255 | 安全余量较大 |
| 1735 | 261.447 | 581.799 | 300 ms预计越过5 m余量 |
| 1738 | 193.072 | 518.092 | 300 ms预计越过5 m余量 |

由此必须区分两个实验目标：

```text
若要求六组都保留5 m余量：注入值不应超过约50 ms
若目标是寻找平均安全临界点：第一组选择约300 ms
若目标是继续定位碰撞边界：在确认400 ms结果后再逐步增加
```

### 15.7 下一步时延注入的当前方案

在修正 SCB 触发时机后，推荐先运行：

$$
0\ \mathrm{ms}
\rightarrow300\ \mathrm{ms}
\rightarrow0\ \mathrm{ms}
$$

目的：

- 第一个 0 ms 验证当次 baseline；
- 300 ms 接近均值参数推导的安全临界点；
- 第二个 0 ms 验证队列、系统负载和状态能够恢复。

随后可采用：

```text
0、200、300、400 ms
```

每档至少重复 5 次。当前 `fixed_delta_seconds=0.1`，粗搜索优先采用 100 ms 步长；50 ms细化可能受到 tick 相位影响，必须增加重复次数并以实际 `t1→t2`、最小净距和碰撞结果判断。

按六组 baseline 线性外推：

| 注入时延 | 预计平均总响应 | 预计低于5 m安全余量 | 预计碰撞 |
|---:|---:|---:|---:|
| 100 ms | 444 ms | 1/6 | 0/6 |
| 200 ms | 544 ms | 2/6 | 0/6 |
| 300 ms | 644 ms | 3/6 | 0/6 |
| 400 ms | 744 ms | 4/6 | 约1/6 |
| 500 ms | 844 ms | 6/6 | 约1/6 |
| 600 ms | 944 ms | 6/6 | 约3/6 |

这些只是档位设计预测，不是最终实验结论。

### 15.8 SCB 触发问题必须先解决

六组 baseline 的 SCB 请求注入时延均为 0，因此可以用于零注入分析。但六组 `APPLIED` 都发生在障碍物稳定识别 `t1` 之前约 `33.7–40.2 s`，平均提前约 `36.1 s`。

原因是车辆在正常行驶和速度控制过程中已经满足：

```text
activation_speed_mps
brake_threshold_percentage
```

导致注入器过早触发。若直接把 `delay_ms` 改为非零，Bridge 会在障碍物出现前就延迟后续 Control，改变接近速度和 D1，实验将不再是单变量对照。

正式注入前必须增加显式 ARM，推荐任选一种：

```text
障碍物生成时ARM
自车距离目标约50–55 m时ARM
场景脚本发送独立ARM信号
```

ARM 之后才允许制动阈值触发注入；每次实验结束必须清空队列并复位状态。

### 15.9 场景纳入、异常和统计规则

当前建议的实际车速接收窗口：

$$
15.6\pm0.3\ \mathrm{m/s}
$$

`202607171706` 的 `t1` 车速为 `16.362 m/s`，超出窗口。它应保留在六组总体结果中作为保守样本，但正式重复实验时应标记为条件异常，不能与速度合格样本无区别地合并。

统计规则：

1. 物理结局不因“不利”而删除；碰撞和小净距是实验结果，不是异常值。
2. 只有配置不一致、目标身份不明、速度越界、日志缺失或结局未知时，才能按预先规则排除或分层。
3. 模块本地处理时延可以使用 `Q3 + 1.5×IQR` 删除孤立异常极大值，但必须报告原始样本数和剔除数。
4. Deadline 必须逐场景计算，再汇总均值、中位数、P90和范围。
5. 每个注入档位报告碰撞次数/总次数，不能只统计成功解析的案例。
6. 同时报告最小净距、碰撞速度和 `t1→t2`；不能只给“碰撞/未碰撞”二值结果。

### 15.10 不同E2E数据不能混用

当前工作区存在多个不同案例、不同口径的时延数字：

```text
202607171107全生命周期点云到Control平均：567.415 ms
202607171107碰撞关键帧t1到Localization持续减速：841.753 ms
六组baseline的t1到Control平均：266.629 ms
六组baseline的t1到Localization持续减速平均：343.657 ms
```

用途分别为：

- `567.415 ms`：描述 `202607171107` 整次 run 的点云到 Control 总体性能；
- `841.753 ms`：解释 `202607171107` 碰撞关键帧的单次因果响应；
- `266.629 ms`：描述六组当前 baseline 的 Apollo 软件命令响应；
- `343.657 ms`：用于六组当前 baseline 的闭环隐形 Deadline 和注入预算。

禁止用 `202607171107` 的全生命周期平均替换六组 baseline 响应，也禁止把模块本地处理时延相加后直接当作物理闭环响应。

### 15.11 每个run建议输出的最终记录

```text
场景编号
配置快照与Bridge版本
请求注入时延
SCB实际注入时延
目标ID/physical_target_id_chain
t1及时间来源
t1时刻实际车速v1
D1中心距离
D1净距
Control首次相关有效制动时间
t1到Control时间
Localization持续减速t2
t1到t2观测闭环响应
Control到t2剩余闭环时间
Localization采样间隔/频率
等效减速度
实际制动距离
理论安全Deadline
理论碰撞Deadline
是否低于5 m安全余量
最终最小净距
是否碰撞
碰撞速度
数据质量标记与排除原因
```

最后用以下三项共同判断：

```text
t1到t2是否超过安全Deadline
最小净距是否小于5 m
是否发生碰撞
```

三种结果解释：

```text
未超过安全Deadline且净距>=5 m：满足当前安全余量
超过安全Deadline但未碰撞：安全余量失效，尚未越过碰撞边界
发生接触：越过当前平台下的碰撞边界
```

## 16. 2026-07-22 四组实验复核与隐形 Deadline 可复用规范（当前权威）

本章汇总2026-07-18至2026-07-22对四组正式实验的复核结论、对话中发现的关键问题，以及后续可以直接复用的计算流程。它覆盖第15章中已经变化的分析口径。

### 16.1 当前实验、数据与输出位置

固定实验条件：

- CARLA 0.9.15部署在服务器侧，Apollo 10.0.0部署在Orin侧，Bridge位于服务器侧。
- Guardian未接入，Bridge直接读取 /apollo/control。
- 地图为Town04。
- CARLA同步模式固定步长为0.1 s。
- 点云配置数量为130万；run目录没有归档原始点数计数，因此报告只能写“配置为130万”。
- 当前场景为前方静止障碍物。
- 当前稳定感知车头净距约40 m，名义传感器范围约50 m。
- 主安全裕度为6 m；0、5、6、8、10 m进入敏感性分析。

输入目录：

- D:\data\baseline
- D:\data\100ms
- D:\data\300ms
- D:\data\400ms

分析输出：

- D:\data\realtime_collision_analysis
- 主报告：D:\data\realtime_collision_analysis\report\realtime_collision_experiment_report.md
- 当前共识别23个run：baseline 6个、100 ms组6个、300 ms组6个、400 ms组5个。

当前报告和CSV是在“模型余量作为逐run主余量”的旧实现下生成的。后续脚本和报告必须按本章把实际数据计算结果与模型预测结果拆开；完成重算前，不得把旧字段 M_collision_0m 和 M_safety_6m直接称为实际观测余量。

### 16.2 时间端点与主响应时间

稳定感知时刻 \(t_1\)：

1. 目标障碍物连续至少3帧稳定存在。
2. \(t_1\)取稳定序列第一帧的传感器源时间，不取Fusion输出时间。
3. 同时保存目标ID、物理目标ID链、\(v_1\)、中心距和车头净距。

有效物理减速时刻 \(t_2\)：

1. 从Localization计算速度 \(v_i\)。
2. 相邻区间减速度幅值定义为

\[
b_i=\frac{v_{i-1}-v_i}{t_i-t_{i-1}}.
\]

3. 主判据要求连续两个区间满足 \(b_i\ge0.5\ \mathrm{m/s^2}\)。
4. 随后0.3 s累计降速至少0.3 m/s，用于确认持续性。
5. 确认通过后，\(t_2\)回记为第一段合格减速区间结束时刻。
6. 三点中值平滑以及0.3、0.5、1.0 m/s²阈值只进入敏感性分析。

实际端到端响应采用墙钟时间：

\[
T_{\mathrm{e2e,wall}}=t_2-t_1,
\qquad
L_{\mathrm{e2e,wall}}=1000T_{\mathrm{e2e,wall}}\ \mathrm{ms}.
\]

Planning、Control、Bridge release/apply时间继续保存，用于拆分功能链。它们不能替代 \(t_2\)，CARLA apply_control返回也不能代表车辆已经产生有效物理减速。

### 16.3 D_delay必须统一采用墙钟速度积分

主响应距离定义为车辆速度对墙钟时间的梯形积分：

\[
D_{\mathrm{delay,wall}}
=
\sum_i
\frac{v_i+v_{i+1}}{2}
\left(t_{i+1,\mathrm{wall}}-t_{i,\mathrm{wall}}\right),
\qquad t_i\in[t_1,t_2].
\]

所有run的主字段 D_delay 必须使用该公式。CARLA推进帧数、仿真时间和Localization坐标位移可能因实时因子变化而不同，必须作为独立诊断量保存：

- D_delay_wall_integral_m：主墙钟速度积分距离。
- D_delay_sim_observed_m：CARLA空间轨迹在 \(t_1\) 到 \(t_2\) 的纵向位移。
- e2e_sim_frames：区间内推进的仿真帧数。
- e2e_sim_ms：仿真帧数乘以0.1 s。
- realtime_factor：仿真推进时间除以墙钟时间。

禁止以下做法：

1. 一部分run使用墙钟速度积分，另一部分run使用CARLA空间位移。
2. 只修正单个run后，将结果放回原表与其他旧口径run比较。
3. 因为CARLA空间位移更接近直接碰撞结果，就覆盖主墙钟 D_delay。
4. 默认认为0.1 s固定步长等于0.1 s墙钟时间。

建议执行两个一致性检查：

\[
D_{\mathrm{delay,constant}}
\approx v_1T_{\mathrm{e2e,wall}},
\]

\[
RTF
=
\frac{T_{\mathrm{e2e,sim}}}{T_{\mathrm{e2e,wall}}}.
\]

第一项用于检查积分和速度变化，第二项用于识别CARLA时间膨胀。二者都是诊断，不替代主墙钟积分。

### 16.4 必须同时保存实际数据结果和模型预测结果

后续分析必须形成两套字段。字段名、图题和表头应明确包含 data/observed 或 model/predicted。

#### 16.4.1 实际数据计算结果

实际数据结果使用该run真实记录的输入：

- 实测 \(D_1\)车头净距。
- 实测墙钟 \(D_{\mathrm{delay,wall}}\)。
- 实测制动过程和制动位移。
- 实测最终净距、碰撞event和碰撞速度。

当前兼容口径下，能够达到 \(v<0.1\ \mathrm{m/s}\) 的run使用 \(t_2\) 之后最低速度样本作为经验制动完成端点：

\[
D_{\mathrm{brake,data}}
=
\left\|
\mathbf p_{\mathrm{min\ speed}}
-\mathbf p(t_2)
\right\|_2.
\]

实际数据公式余量：

\[
M_{\mathrm{collision,0m,data}}
=
D_1-D_{\mathrm{delay,wall}}-D_{\mathrm{brake,data}},
\]

\[
M_{\mathrm{safety,6m,data}}
=
D_1-D_{\mathrm{delay,wall}}-D_{\mathrm{brake,data}}-6.
\]

两者必须满足：

\[
M_{\mathrm{collision,0m,data}}
=
M_{\mathrm{safety,6m,data}}+6.
\]

碰撞run没有完整停车过程，碰撞前制动距离属于右截断观测。此时 D_brake_data、M_collision_0m_data和M_safety_6m_data应标记为不可用，主结果使用碰撞event、碰撞速度、碰撞前最小净距和已经观测到的制动过程。禁止用经验模型填充后再标成“实际数据结果”。

CARLA直接记录的最终净距或最小净距应单列为 clearance_observed_m。由于墙钟积分与CARLA同步仿真空间推进可能存在时间膨胀，clearance_observed_m不保证等于上述墙钟物理预算公式余量；这种差值应作为仿真实时因子问题报告。

#### 16.4.2 模型预测结果

当前经验制动模型为：

\[
D_{\mathrm{brake,model}}(v_2)
=
k_{\mathrm{median}}v_2^2,
\]

\[
k_{\mathrm{median}}
=0.1019684762\ \mathrm{s^2/m}.
\]

模型余量：

\[
M_{\mathrm{collision,0m,model}}
=
D_1-D_{\mathrm{delay,wall}}-D_{\mathrm{brake,model}},
\]

\[
M_{\mathrm{safety,6m,model}}
=
D_1-D_{\mathrm{delay,wall}}-D_{\mathrm{brake,model}}-6.
\]

模型结果用于：

- 验证经验模型精度。
- 计算模型误差和适用范围。
- 设计下一档速度、距离和总响应时间。
- 计算未发生条件的预测值和反事实值。

模型结果不能覆盖实际数据结果。模型精度至少报告：

\[
e_D
=D_{\mathrm{brake,model}}-D_{\mathrm{brake,data}},
\]

\[
e_M
=M_{\mathrm{collision,0m,model}}
-M_{\mathrm{collision,0m,data}},
\]

\[
e_{\mathrm{rel}}
=\frac{D_{\mathrm{brake,model}}-D_{\mathrm{brake,data}}}
{D_{\mathrm{brake,data}}}.
\]

实验结果分析报告的主表、摘要和结论使用实际数据计算结果与CARLA直接结局；模型值放在独立的模型验证、预测和反事实章节。

### 16.5 经验制动模型表达的含义

该模型把从 \(t_2\) 到近零速度的完整变减速过程压缩为总制动位移。Apollo实际减速度可以随Planning轨迹、jerk约束、fallback、Control跟踪和车辆动力学不断变化。模型不会复现瞬时减速度曲线。

等效减速度定义为：

\[
a_{\mathrm{eff}}
=
\frac{v_2^2}{2D_{\mathrm{brake,data}}}.
\]

它表示产生相同总制动距离的等效能力，不代表Apollo持续输出固定减速度。

当前模型信息：

- 拟合样本：6条无碰撞baseline近停样本。
- \(v_2\)范围：15.378至16.276 m/s。
- \(k_{\mathrm{median}}=0.1019684762\ \mathrm{s^2/m}\)。
- \(k\)的bootstrap 95%区间：0.09265057至0.10964053 s²/m。
- 等效减速度中位数：4.90446 m/s²。
- 模型适用范围：当前Town04、当前车辆和控制配置、静止障碍物、约15至17 m/s。

202607201555的减速度P10、中位数、P90和峰值分别约为0.568、2.568、8.545和12.422 m/s²，说明实际过程明显变减速。该run模型预测制动距离25.827 m，实际经验制动位移22.980 m，模型多估计2.847 m，约12.4%。这类误差必须在模型验证章节公开。

Planning速度求解不可行后进入fallback属于Apollo设计内功能。当前用户确认fallback减速度约4 m/s²，但run归档没有保存对应配置快照。约4 m/s²可以作为并行保守边界，不能冒充归档中直接测得的配置值。

### 16.6 制动端点与停车后蠕行

当前baseline经验模型沿用“\(t_2\)之后最低速度样本”的端点，以保持既有六条样本可比。该端点可能落在首次严格停车之后，并包含停车后蠕行。

202607201555中：

- 首次严格停车时间约为1784534205.046 s。
- 最低速度代理端点约为1784534208.274 s。
- 两者相差约3.23 s。
- 期间车辆向前蠕行约0.90 m。

后续每个run建议同时保存：

- t_near_stop：首个 \(v<0.1\ \mathrm{m/s}\) 样本。
- t_stop_strict：\(v<0.1\ \mathrm{m/s}\) 持续至少0.5 s的起点。
- t_minimum_speed：\(t_2\)之后最低速度样本。
- D_brake_to_strict_stop：制动到首次严格停车的位移。
- D_creep_after_stop：严格停车后到最小净距或分析端点的继续接近距离。
- D_brake_legacy_min_speed：用于复现当前baseline模型的旧端点位移。

若未来改用严格停车端点重建模型，必须对全部baseline统一重算并更换模型版本，不能只替换个别run。

### 16.7 隐形 Deadline 的可复用计算方法

隐形Deadline分析应优先使用空间预算，因为实际run允许速度变化。

#### 16.7.1 单个run的数据计算流程

1. 验证地图、步长、点云配置、Bridge版本、时延配置和碰撞结局。
2. 确定物理目标、Apollo目标ID和目标ID链。
3. 确定 \(t_1\)、\(v_1\)、\(D_{1,\mathrm{center}}\) 和 \(D_1\)车头净距。
4. 使用组合几何偏移5.3074 m完成中心距到净距转换；几何不确定性约为正负0.52 m。
5. 确定Planning STOP、Control相关制动命令和 \(t_2\)。
6. 计算墙钟总响应 \(T_{\mathrm{e2e,wall}}\)。
7. 对墙钟时间做速度梯形积分，得到 \(D_{\mathrm{delay,wall}}\)。
8. 有完整停车证据时计算 \(D_{\mathrm{brake,data}}\) 和实际数据余量。
9. 使用经验模型计算 \(D_{\mathrm{brake,model}}\) 和模型余量。
10. 保存CARLA仿真帧、仿真时间、空间位移和实时因子诊断。
11. 保存碰撞、碰撞速度、最终净距和最小净距，作为直接结局。
12. 分开输出实际数据表和模型验证表。

#### 16.7.2 设计实验时的显式时间Deadline

在设计阶段采用恒速近似：

\[
D_{\mathrm{delay}}\approx vT_{\mathrm{total}}.
\]

0 m接触边界：

\[
T_{\mathrm{deadline,0m}}
=
\frac{D_1-D_{\mathrm{brake}}(v)}{v}.
\]

6 m安全边界：

\[
T_{\mathrm{deadline,6m}}
=
\frac{D_1-D_{\mathrm{brake}}(v)-6}{v}.
\]

给定总响应时间时，所需车头净距为：

\[
D_{1,\mathrm{required}}
=
vT_{\mathrm{total}}
+D_{\mathrm{brake}}(v)
+D_{\mathrm{margin}}.
\]

障碍物与车辆中心距离为：

\[
D_{\mathrm{center,required}}
=D_{1,\mathrm{required}}+5.3074.
\]

该时间Deadline用于解释和选点。逐run正式结果仍使用实测墙钟速度积分，避免恒速近似掩盖实际速度变化。

#### 16.7.3 可继续注入的时延

\[
\Delta T_{\mathrm{inject}}
=
T_{\mathrm{deadline}}
-T_{\mathrm{response,baseline}}.
\]

这个结果只用于选择下一档注入值。每个注入run都必须重新测量 \(t_1\)、\(t_2\)、实际总响应、墙钟 D_delay、速度、D1和结局，不能将名义注入值机械加到baseline。

### 16.8 0 m余量、6 m余量和反事实

0 m碰撞余量回答：按照当前数据或模型口径，车辆是否有足够距离在接触前完成停车。

6 m安全余量回答：完成停车后能否继续保留6 m距离。

每一套口径内部必须满足：

\[
M_{\mathrm{collision,0m}}
=M_{\mathrm{safety,6m}}+6.
\]

反事实回答：如果当前run的额外墙钟响应恢复到匹配baseline水平，模型预计能够节省多少距离。令：

\[
\Delta T
=T_{\mathrm{e2e,current}}
-T_{\mathrm{e2e,baseline}},
\]

\[
D_{\mathrm{saved}}
=
\int_{t_2-\Delta T}^{t_2}v(t)\,\mathrm{d}t_{\mathrm{wall}}.
\]

\[
M_{\mathrm{counterfactual,model}}
=M_{\mathrm{observed\ input,model}}+D_{\mathrm{saved}}.
\]

反事实从未真实发生，必须标为model/counterfactual。它不能命名为实际余量，也不能单独证明碰撞由实时性引发。

RT_ONLY_COLLISION需要联合证据：

1. 目标碰撞event成立。
2. 时延注入和Bridge执行证据完整。
3. Perception、Prediction、Planning、Control和Bridge功能链完整。
4. 同条件baseline能够安全停车。
5. 目标身份、D1和时间链可复核。

Planning进入设计内fallback仍可判为功能PASS。模型余量属于支持性诊断，不应成为“功能正常且注入导致碰撞”样本的唯一否决条件。伴随感知时序退化的样本应使用独立子类，例如TIMING_INDUCED_FUNCTIONAL_DEGRADATION。

### 16.9 两个400 ms run暴露的时间基准陷阱

202607191727和202607201555的实际数据：

| 指标 | 202607191727 | 202607201555 |
|---|---:|---:|
| 墙钟总响应 | 809.268 ms | 892.358 ms |
| 主墙钟D_delay | 13.520 m | 14.385 m |
| CARLA空间位移诊断 | 11.696 m | 8.062 m |
| \(t_1\)到\(t_2\)仿真间隔 | 7 | 5 |
| 推算仿真推进时间 | 0.7 s | 0.5 s |
| Localization墙钟间隔中位数 | 113.343 ms | 185.071 ms |
| SCB实际仿真延迟 | 300 ms / 3帧 | 200 ms / 2帧 |
| \(v_2\) | 17.021 m/s | 15.915 m/s |
| 结局 | 碰撞，4.026 m/s | 未碰撞 |

曾经出现的错误表格将202607191727的墙钟积分13.520 m和202607201555的CARLA空间位移8.062 m放在同一列，造成“响应更长、D_delay反而更小”的假象。

统一墙钟口径后：

- 202607191727：D_delay为13.520 m。
- 202607201555：D_delay为14.385 m。

主D_delay随两组墙钟响应保持同方向变化。CARLA空间位移出现11.696 m和8.062 m的反向关系，原因是202607201555的仿真实时因子更低，892 ms墙钟内只推进5个仿真步。

必须保留的结论：

1. 主报告使用墙钟速度积分。
2. CARLA空间位移仅作为sim_observed诊断。
3. 同为400 ms名义配置时，实际跨越的CARLA帧数可能不同。
4. 若希望碰撞结局直接模拟真实道路的墙钟时延效应，需要控制实时因子接近1，或增加按仿真帧/仿真时间控制的并行实验。

### 16.10 202607201555的数据结果与模型结果示例

该run的共同输入：

\[
D_1=39.571654\ \mathrm{m},
\]

\[
D_{\mathrm{delay,wall}}=14.385018\ \mathrm{m}.
\]

实际经验制动位移：

\[
D_{\mathrm{brake,data}}=22.980107\ \mathrm{m}.
\]

因此实际数据公式结果：

\[
M_{\mathrm{collision,0m,data}}
=39.571654-14.385018-22.980107
=2.206529\ \mathrm{m},
\]

\[
M_{\mathrm{safety,6m,data}}
=-3.793471\ \mathrm{m}.
\]

模型预测制动距离：

\[
D_{\mathrm{brake,model}}=25.826723\ \mathrm{m}.
\]

模型结果：

\[
M_{\mathrm{collision,0m,model}}
=-0.640086\ \mathrm{m},
\]

\[
M_{\mathrm{safety,6m,model}}
=-6.640086\ \mathrm{m}.
\]

CARLA最低速度代理端点的直接净距为8.532414 m。三组数值含义不同：

- 2.207 m：实测墙钟D_delay与实测制动位移代入空间预算得到的数据结果。
- -0.640 m：实测墙钟D_delay与baseline经验模型代入得到的模型预测。
- 8.532 m：CARLA空间轨迹直接测得的仿真结局。

模型负余量与未碰撞并存，主要说明模型预测误差和CARLA时间膨胀。模型负值不能覆盖碰撞传感器与最终净距，也不能改写为“实际发生空间不足”。

### 16.11 30 m/s、总响应300 ms的复用示例

当前经验模型仅可用于粗略试验设计，30 m/s明显超出15.378至16.276 m/s的样本范围。

总响应时间为300 ms时：

\[
D_{\mathrm{delay}}=30\times0.3=9.0\ \mathrm{m},
\]

\[
D_{\mathrm{brake,model}}
=0.1019684762\times30^2
=91.772\ \mathrm{m}.
\]

经验模型给出：

- 0 m边界车头净距：100.772 m。
- 6 m边界车头净距：106.772 m。
- 0 m边界中心距：106.079 m。
- 6 m边界中心距：112.079 m。

若按4 m/s²并行保守边界：

\[
D_{\mathrm{brake,4m/s^2}}
=\frac{30^2}{2\times4}
=112.5\ \mathrm{m}.
\]

- 0 m边界车头净距：121.5 m。
- 6 m边界车头净距：127.5 m。
- 0 m边界中心距：126.807 m。
- 6 m边界中心距：132.807 m。

当前稳定感知距离约40 m、名义范围约50 m，无法提供100至133 m的有效停车预算。把障碍物生成得更远只有在Apollo能够从该距离稳定感知时才有意义。正式开展30 m/s实验前，需要扩大可靠感知距离，并在26、28、30、32 m/s附近重新采集baseline、拟合和留出验证。

### 16.12 静止障碍物与运动前车的适用边界

当前整套碰撞余量模型只在前方静止障碍物场景得到验证。经验制动子模型可以作为当前车辆完整停车能力的初始描述，但运动前车还需要相对运动。

前车保持匀速时，可用于试验设计的简化关系为：

\[
\Delta v=v_e-v_f,
\]

\[
D_{\mathrm{relative,delay}}
=\Delta vT_{\mathrm{e2e}},
\]

\[
D_{\mathrm{relative,brake}}
\approx k(\Delta v)^2.
\]

前车制动、切入、横穿或对向运动时，应计算：

\[
D_{\mathrm{relative}}(t)
=
\int_{t_1}^{t}
\left[v_e(\tau)-v_f(\tau)\right]\mathrm{d}\tau,
\]

并取整个过程最大的累计接近距离。运动目标还会引入跟踪与Prediction链路差异，需要新的baseline验证，不能直接把静止障碍物模型当作已验证结论。

### 16.13 已踩过的坑与复核清单

1. t1取稳定序列第一帧源时间，Fusion发布时间单独保存。
2. t2取Localization持续有效减速起点，Planning、Control和apply_control均不替代。
3. D_delay统一使用墙钟速度梯形积分。
4. CARLA固定步长不代表固定墙钟帧率。
5. 同一列不得混入wall_integral和sim_observed。
6. 任何计算口径修改必须对全部run重算。
7. 实际数据结果、模型结果、反事实结果和CARLA直接结局分别命名。
8. 碰撞run缺少完整停车距离，不能用截断距离拟合制动模型。
9. Apollo变减速过程可以用实际总制动位移描述；等效减速度不代表瞬时恒减速。
10. 最低速度端点可能包含停车后蠕行，应同时保存严格停车端点。
11. 模型负余量但未碰撞首先检查模型误差、时间基准、几何偏移和端点。
12. D1使用车头到障碍物近端的纵向净距；中心距需要减去5.3074 m。
13. 几何偏移当前约有正负0.52 m不确定性。
14. 0 m碰撞边界和6 m安全边界必须同时保存，二者严格相差6 m。
15. 名义100、300、400 ms只描述配置；每个run仍以实际墙钟总响应为准。
16. SCB的首条APPLIED日志可能早于目标t1；触发后后续ControlCommand仍持续进入延迟队列，不能只看第一条日志时间下结论。
17. Planning fallback属于设计内功能，功能链判定要结合日志语义。
18. 反事实属于模型推断，不能写成已经观测到的物理结果。
19. 30 m/s等超出样本速度范围的结果只能用于试验设计。
20. 障碍物生成距离、理论传感器范围和稳定感知距离必须分开。
21. 实验报告只维护一份Markdown文档。块公式统一使用与既有报告相同的`$$...$$`，行内变量使用`$...$`；不要使用会在钉钉中显示为原始文本的`\[...\]`或`\(...\)`。表格内使用简洁指标名或字段名，长公式放在表格外单独书写，不再额外生成“钉钉版”报告。

### 16.14 推荐字段和自动校验

实际数据与直接结局字段：

- t_sensor_origin_s
- t_brake_effective_s
- actual_e2e_latency_wall_ms
- D1_clear_m
- D_delay_wall_integral_m
- D_brake_data_m
- M_collision_0m_data_m
- M_safety_6m_data_m
- clearance_observed_m
- collision
- impact_speed_mps

模型与反事实字段：

- braking_model_version
- k_median
- D_brake_model_m
- M_collision_0m_model_m
- M_safety_6m_model_m
- T_deadline_0m_model_ms
- T_deadline_6m_model_ms
- D_saved_counterfactual_wall_m
- M_collision_0m_counterfactual_model_m
- M_safety_6m_counterfactual_model_m
- braking_distance_model_error_m
- braking_distance_model_relative_error

CARLA诊断字段：

- D_delay_sim_observed_m
- e2e_sim_frames
- e2e_sim_ms
- realtime_factor
- localization_interval_wall_ms
- scb_actual_wall_delay_ms
- scb_actual_sim_delay_ms
- scb_actual_frame_delay

自动校验：

1. data和model两套结果分别满足 \(M_{\mathrm{collision,0m}}=M_{\mathrm{safety,6m}}+6\)。
2. D_delay_wall_integral与速度、墙钟响应量级一致。
3. 修改公式后全部23个run同步重算。
4. 碰撞run的完整D_brake_data保持缺失，除非存在可信的碰撞后完整停车反事实数据源。
5. 表头、图例和正文不得把model字段缩写成observed。
6. 反事实字段必须包含counterfactual和model标记。
7. 最终结论以碰撞event、实际数据结果和功能链证据为主，模型只承担预测与验证作用。

### 16.15 当前待办

1. 修改realtime_collision_analysis脚本，将实际数据结果和模型结果分开生成。
2. 保留主墙钟D_delay，撤销任何只对202607201555使用8.062 m的局部替换。
3. 为全部run新增CARLA仿真帧、仿真时间和实时因子诊断。
4. 对无碰撞完整停车run计算D_brake_data、data余量和模型误差。
5. 对碰撞run保持完整实际制动距离和data余量缺失，主报碰撞event、碰撞速度和截断制动证据。
6. 重新生成CSV、图、Markdown和HTML报告。
7. 重新审核当前RT_ONLY_COLLISION和TIMING_INDUCED_FUNCTIONAL_DEGRADATION分类，确认分类没有把模型余量当作实际观测硬门槛。
8. 增加单元测试，覆盖墙钟/仿真口径分离、data/model字段分离、6 m恒等关系和局部混用检测。

## 17. 2026-07-27 第二次车速隐形Deadline实验报告写作规范（最新）

本章只约束第二次实验的结果分析报告。与前面章节中的通用报告建议冲突时，第二次实验报告以本章和用户最新要求为准。

### 17.1 报告文件与格式

- 当前报告文件只有一个：

  ```text
  D:\data\第二次实验\第二次车速隐形Deadline实验分析报告.md
  ```

- 不创建纯文本版、钉钉版或第二份内容相同的报告。
- 排版格式参考：

  ```text
  D:\data\realtime_collision_analysis\report\realtime_collision_experiment_report.md
  ```

- 只参考该文件的Markdown组织方式，包括标题层级、正文、列表、表格、图注和公式写法；不得引用或复用其中的实验数值、结论、样本分类和分析结果。
- 块公式使用：

  ```text
  $$
  LaTeX公式
  $$
  ```

- 行内变量使用`$...$`。
- 禁止在报告中使用`\[...\]`和`\(...\)`作为公式定界符。
- 表格内优先使用简洁中文指标名或字段名，不在单元格中放置过长的LaTeX表达式；需要解释的公式在表格前后使用块公式单独给出。

### 17.2 当前报告内容范围

当前报告已经完成：

1. 第一章“实验设置”。
2. 第二章“指标定义与解释”。
3. 第三章“时延对制动位置和距离债务的影响”。
4. 第四章“安全余量计算”。
5. 第五章“202607271131与202607271206碰撞差异分析”。
6. 第六章“202607271643与202607271211碰撞差异分析”。

第二章保持精简，不增加以下独立章节：

- 墙钟指标与CARLA仿真诊断；
- SCB实际时延指标；
- CARLA直接结局；
- 模型预测结果；
- 分组汇总与数据质量规则；
- 推荐输出字段；
- 基于模型制动距离的时间Deadline。

上述信息中确实需要支撑实际结果或碰撞归因的部分，可以放入对应实验结果或碰撞案例分析中，但不再扩展成第二章的独立指标小节。

### 17.3 只使用实际数据形成报告结果

第二次实验结果分析报告的主表、图、摘要、逐run结果、碰撞分析和结论全部使用该run实际记录的数据，包括：

- 实际目标稳定感知源时间；
- 实际Localization速度和位置；
- 实际闭环响应；
- 实际墙钟速度积分距离；
- 实际完整停车制动位移；
- 实际最终净距和最小净距；
- 实际碰撞event和碰撞速度；
- 实际感知、Planning、Control和Bridge执行证据。

当前第二次实验报告不编写经验制动模型预测、模型余量、反事实余量或模型预测Deadline，不使用其他run或旧实验的模型值填补本run的实际结果。

碰撞run没有完整停车过程时：

- `D_brake_data_m`保持不可用；
- `M_collision_0m_data_m`保持不可用；
- `M_safety_6m_data_m`保持不可用；
- 只报告碰撞event、碰撞速度、碰撞前最小净距、碰撞前降速和截断制动过程。

### 17.4 统一口径要求

baseline组和300 ms组的全部run必须统一使用：

- 同一目标身份确认规则；
- 同一$t_1$定义；
- 同一$t_2$判据；
- 同一Localization速度计算与插值方法；
- 同一墙钟时间基准；
- 同一$D_{\mathrm{delay}}$梯形积分方法；
- 同一$D_1$几何转换；
- 同一停车端点；
- 同一实际制动距离公式；
- 同一0 m碰撞余量和6 m安全余量公式；
- 同一碰撞run缺失值处理方法；
- 同一组内统计方法。

计算口径发生变化时，必须对第二次实验全部12个run统一重新计算，禁止只修改部分run后继续与其他旧结果同列比较。

主`D_delay`仍统一使用墙钟速度梯形积分：

$$
D_{\mathrm{delay,wall}}
=
\int_{t_1}^{t_2}v(t)\,\mathrm{d}t_{\mathrm{wall}}.
$$

CARLA空间位移、仿真帧数和实时因子仅在确有必要解释碰撞差异时作为诊断证据，不得替代主`D_delay`。

### 17.5 不进行敏感性分析

第二次实验报告不包含：

- 替代减速阈值；
- 替代速度平滑方法；
- 替代稳定感知帧数；
- 5 m、8 m、10 m等替代安全余量；
- 敏感性图表或敏感性结论。

所有run使用唯一主判据：连续两个Localization区间的制动减速度幅值不小于0.5 m/s²，并由随后0.3 s累计降速不少于0.3 m/s确认持续性；确认后$t_2$回记为第一段合格区间的结束时刻。

### 17.6 碰撞run单独分析

每个碰撞run必须单独成节，并使用两层实际数据对照：

1. 与点云56万、`lidar_detection` DAG队列1、相同请求时延、相同地图、车辆和Bridge设置下的全部未碰撞run比较。
2. 在相同设置未碰撞run中根据当前问题选择一个主配对run：若问题是“显式响应相近为何结局不同”，优先选择$T_{\mathrm{e2e,wall}}$最接近者；若问题是隔离初始条件影响，优先选择$t_1$车速和$D_{1,\mathrm{clear}}$最接近者；用户明确指定配对时按指定组合分析。无论采用哪种选择理由，都必须公开$t_1$车速、初始净距和实际响应的原始差值，不能把不接近的量描述成“已控制一致”。

碰撞对比至少覆盖：

- 初始车速和初始净距；
- 点云入口等待；
- Lidar Detection排队、纯计算、总驻留和Trace完成率；
- Fusion输出连续性；
- $t_1$到Fusion、Prediction、Planning和Control的实际阶段响应；
- Bridge请求时延、实际执行时延和执行状态；
- $t_1$到$t_2$实际闭环响应；
- 墙钟$D_{\mathrm{delay}}$；
- 碰撞前实际降速和截断制动位移；
- 未碰撞对照run的完整实际制动距离；
- 实际最小净距、碰撞event和碰撞速度。

碰撞归因必须由目标身份、功能链、Bridge执行、实际运动和同设置对照共同支持。不得仅凭响应时间较长或单个派生指标给出强因果结论。

### 17.7 碰撞与未碰撞场景的逐项配对方法

碰撞与未碰撞场景的比较不是只列两个run的总响应和结局，而是要按照“可比性确认、显式时延、$t_2$状态、后续运动、感知连续性、最终结局”的顺序建立证据链。

#### 17.7.1 配对前的可比性确认

只有以下条件一致的run才进入主对照：

- 点云数量；
- `lidar_detection` DAG队列长度；
- 名义Bridge请求时延；
- 地图、路线、车辆类型和静止障碍物场景；
- Bridge、Apollo和分析脚本版本；
- $t_1$、$t_2$、目标身份和距离口径；
- 碰撞记录、Localization、Perception、Planning、Control和SCB证据完整性。

先将碰撞run与同设置下全部未碰撞run做描述性比较，再选一个主配对run做逐项时间链和空间链分析。当前第二次实验已经使用的主配对为：

- 202607271131与202607271206：两组实际闭环响应均约800 ms，用于回答“显式响应接近为何结局不同”；
- 202607271643与202607271211：按用户指定组合分析，用于说明更长显式响应、更小初始净距和后续数据新鲜度如何共同形成碰撞。

这两个组合是当前报告问题下的配对，不是永久固定的自动选择规则。新增run或改变研究问题后必须重新说明配对理由。

#### 17.7.2 差值方向必须固定

所有配对差值统一采用：

$$
\Delta X
=
X_{\mathrm{collision}}
-X_{\mathrm{noncollision}}.
$$

对于时延、速度、响应阶段行驶距离和数据年龄，正值通常表示碰撞run更不利。对于初始净距和$t_2$剩余净距，建议另行报告“未碰撞run减碰撞run”：

$$
\Delta D_{\mathrm{available}}
=
D_{\mathrm{noncollision}}
-D_{\mathrm{collision}},
$$

使正值直接表示碰撞run少了多少可用距离。表格标题和正文必须写清差值方向，禁止同一表中交替使用两种方向。

#### 17.7.3 第一步：比较显式时延

显式时延至少拆分为：

- 目标源时刻至Fusion输出；
- Fusion至Prediction；
- Prediction至Planning STOP；
- Planning STOP至Control制动命令；
- $t_1$至Control制动命令；
- Control制动命令至$t_2$；
- $t_1$至$t_2$实际闭环响应；
- SCB请求时延、实际墙钟执行时延、仿真帧数和首条制动比例。

SCB实际执行时延包含在Control至$t_2$时间中，不能重复相加。必要时可以计算：

$$
T_{\mathrm{control}\to t_2,\mathrm{res}}
=
T_{\mathrm{control}\to t_2}
-T_{\mathrm{SCB}},
$$

但该剩余量同时包含命令作用、车辆动力学响应和Localization采样确认，不能直接命名为某个Apollo模块时延。

分析顺序：

1. 先判断两组总显式响应是否真的接近。
2. 若总响应接近，检查前段和后段差异是否相互抵消。
3. 若碰撞run总响应明显更长，明确指出额外时间来自哪些实际阶段。
4. SCB实际执行接近时，不能把总响应差异归因于Bridge固定配置。

#### 17.7.4 第二步：比较$t_2$时的车辆状态和隐形Deadline

主距离口径：

$$
D_{2,r}
=
D_{1,r}
-D_{\mathrm{delay,wall},r}.
$$

其中：

$$
D_{\mathrm{delay,wall},r}
=
\int_{t_{1,r}}^{t_{2,r}}
v_r(t)\,\mathrm{d}t_{\mathrm{wall}}.
$$

必须同时列出：

- $v_1$；
- $D_1$初始净距；
- $t_1$至$t_2$速度增量；
- 响应阶段平均速度；
- $D_{\mathrm{delay,wall}}$；
- $v_2$；
- $D_2$剩余净距。

将$t_2$剩余净距按当时速度换算为等效时间预算：

$$
T_{\mathrm{budget},2,r}
=
\frac{D_{2,r}}{v_{2,r}}.
$$

该值只用于比较“当前空间按当前速度还对应多少时间”，不是对真实碰撞时刻的恒速预测，也不是新的软件模块时延。

碰撞run在$t_2$时的空间劣势可以拆成：

$$
D_{2,\mathrm{noncollision}}
-D_{2,\mathrm{collision}}
=
\left(
D_{1,\mathrm{noncollision}}
-D_{1,\mathrm{collision}}
\right)
+
\left(
D_{\mathrm{delay,collision}}
-D_{\mathrm{delay,noncollision}}
\right).
$$

该分解可以区分空间差来自初始净距，还是来自响应阶段额外行驶。

#### 17.7.5 第三步：比较$t_2$后的实际减速形成

达到实际速度阈值$u$的时间定义为：

$$
T_{\leq u,r}
=
t_r\!\left(v\leq u\right)
-t_{2,r}.
$$

阈值必须对两组使用相同值。碰撞前未达到的阈值写“碰撞前未达到”，不能外推时间。

还应在相同$t_2$后墙钟时刻比较：

- 实际车速；
- 从$t_2$开始的累计墙钟速度积分距离；
- 两组累计行驶距离差；
- 碰撞run发生碰撞时，未碰撞run在同一相对时刻的速度、位置和剩余制动过程。

设碰撞run为$C$、未碰撞run为$N$，在$t_2$后相同时间$\tau$的相对空间预算劣势为：

$$
\Delta D_{\mathrm{relative}}(\tau)
=
\left(
D_{2,N}
-D_{2,C}
\right)
+
\left[
D_C(\tau)
-D_N(\tau)
\right],
$$

其中：

$$
D_r(\tau)
=
\int_{t_{2,r}}^{t_{2,r}+\tau}
v_r(t)\,\mathrm{d}t_{\mathrm{wall}}.
$$

正值表示碰撞run相对于未碰撞run少了更多空间预算。该值是两组相对比较量，不是任一run的CARLA包围盒绝对净距。

#### 17.7.6 第四步：区分Fusion输出间断和数据变旧

Fusion相关时序必须至少区分三个概念：

1. 相邻目标Fusion输出间隔；
2. 单条目标输出的生命周期；
3. 当前时刻最近可用目标源观测的年龄。

单条目标生命周期：

$$
T_{\mathrm{fusion,life}}
=
T_{\mathrm{Fusion,out}}
-T_{\mathrm{sensor,obs}}.
$$

在分析时刻$t$，最近源观测年龄：

$$
A_{\mathrm{source}}(t)
=
t
-
\max
\left\{
T_{\mathrm{sensor,obs},i}
\mid
T_{\mathrm{Fusion,out},i}\leq t
\right\}.
$$

比较碰撞run和未碰撞run时，应使用相同相对时间窗，通常以碰撞run的碰撞时刻作为两组共同终点。需要分别报告：

- 目标ID切换次数；
- 相同时间窗最大连续输出间隔；
- 全run异常目标输出间断次数；
- 目标生命周期中位数、P90和最大值；
- 时间窗末最近输出年龄；
- 时间窗末最近源观测年龄；
- 最近目标输出后车辆实际行驶距离。

没有后续输出闭合确认时，“最后一次输出到碰撞”的时间只能称为末次输出年龄或末段无更新时间，不能冒充相邻输出间隔。目标输出连续但生命周期更长时，应描述为“数据持续更旧”，不能写成“Fusion断流”。

#### 17.7.7 第五步：形成有层次的碰撞结论

推荐按以下顺序写结论：

1. 两组直接结局：碰撞event、碰撞速度、未碰撞run最低速度或停车端点。
2. Bridge配置和实际执行是否一致。
3. 显式闭环响应是否接近，或碰撞run具体多出多少时间。
4. 初始净距和响应阶段行驶如何形成$t_2$空间差。
5. $t_2$速度和等效时间预算差。
6. 后续达到相同速度阈值的时间差和累计行驶差。
7. Fusion输出连续性、生命周期和数据新鲜度是首要原因、放大因素，还是可排除因素。
8. 同一相对时刻未碰撞run为何仍有足够空间继续停车。

碰撞run没有完整停车过程，因此不能计算完整实际制动距离或实际0 m、6 m停车余量。碰撞前截断制动距离必须明确标记为“到碰撞为止”，不能与未碰撞run的完整停车制动距离放在同一列当作同类指标。

Planning fallback次数只用于辅助排除或发现功能链差异。未碰撞run fallback更多但仍停车时，不能把碰撞run的较少fallback次数写成碰撞原因。不得使用Mann–Whitney检验、Cliff's delta、Holm校正或敏感性分析替代上述实际证据链；第二次实验当前报告不包含这些内容。

### 17.8 统一空间ST图的构造与解析方法

#### 17.8.1 ST图的目的

不同run的CARLA绝对坐标、稳定目标位置和$t_1$墙钟时刻不同，不能直接把绝对$x$坐标与绝对时间叠加。ST图需要先统一空间原点和时间原点，再比较车辆在同一分析空间中的轨迹。

当前静止障碍物场景统一采用：

$$
S_{\mathrm{obs}}=0,
$$

即把每个run根据稳定Fusion目标和固定组合几何偏移得到的障碍物分析参考边界平移到同一位置。每个run以自己的$t_1$为时间零点：

$$
\tau_r
=
t-t_{1,r}.
$$

车辆在统一空间中的位置定义为：

$$
S_r(t)
=
-D_{1,r}
+
\int_{t_{1,r}}^{t}
v_r(\lambda)\,\mathrm{d}\lambda_{\mathrm{wall}}.
$$

在$t_1$：

$$
S_r(t_{1,r})=-D_{1,r}.
$$

在$t_2$：

$$
S_r(t_{2,r})
=
-D_{1,r}
+D_{\mathrm{delay,wall},r}
=
-D_{2,r}.
$$

#### 17.8.2 数据计算规则

1. 使用Localization实际墙钟时间和实际速度。
2. 在$t_1$与终点处做统一插值。
3. 相邻速度样本使用梯形积分累积$S$。
4. 碰撞run终点取CARLA collision event时刻，速度标注使用碰撞前实际碰撞速度，不使用碰撞后的Localization速度。
5. 未碰撞run终点使用报告统一的最低速度代理或严格停车端点，并保存终点实际速度。
6. 两组使用相同积分函数、端点处理和绘图尺度。
7. 不使用CARLA仿真帧数、仿真时间或局部空间位移替换墙钟速度积分。

建议保存：

- `st_time_rel_t1_s`；
- `st_position_wall_integral_m`；
- `st_t2_time_rel_t1_s`；
- `st_t2_position_m`；
- `st_endpoint_time_rel_t1_s`；
- `st_endpoint_position_m`；
- `st_endpoint_type`；
- `st_endpoint_speed_mps`。

#### 17.8.3 图中必须标记的点

一张配对ST图至少标记：

- 两组$t_1$起点；
- 两组$t_2$位置；
- 统一分析参考边界$S=0$；
- 碰撞run的碰撞端点和碰撞速度；
- 未碰撞run在碰撞run相同相对时刻的位置和速度；
- 未碰撞run最终停车端点和最低速度；
- 碰撞与未碰撞图例。

两组比较时，碰撞run的共同相对时刻定义为：

$$
\tau_{\mathrm{collision}}
=
t_{\mathrm{collision},C}
-t_{1,C}.
$$

未碰撞run在相同相对时刻的位置为：

$$
S_N
\left(
t_{1,N}
+\tau_{\mathrm{collision}}
\right).
$$

由此可以直接计算碰撞时刻两组统一空间位置差：

$$
\Delta S_{\mathrm{matched}}
=
S_C
\left(
t_{\mathrm{collision},C}
\right)
-
S_N
\left(
t_{1,N}
+\tau_{\mathrm{collision}}
\right).
$$

#### 17.8.4 ST图的读取顺序

1. **起点纵向间隔**：两条曲线在$\tau=0$的差值表示$t_1$初始净距差。
2. **到$t_2$的变化**：两组$t_2$标记的纵向差表示显式响应结束时的空间预算差。
3. **曲线斜率**：

   $$
   \frac{\mathrm{d}S_r}{\mathrm{d}t}
   =
   v_r(t).
   $$

   曲线越陡表示车速越高。
4. **曲率和减速**：

   $$
   \frac{\mathrm{d}^2S_r}{\mathrm{d}t^2}
   =
   a_r(t).
   $$

   斜率逐渐减小、曲线变平表示车辆正在减速。
5. **相同相对时刻的垂直间隔**：表示两组按同一ST口径形成的相对空间差。
6. **终点形态**：碰撞run在非零速度处截断；未碰撞run应继续延伸并逐渐变平至近零速。

ST图优先回答“哪一组更早消耗空间、到$t_2$时相差多少、制动后空间差是否扩大”，不单独回答CARLA是否物理碰撞。

#### 17.8.5 $S=0$不是CARLA精确碰撞平面

当前$S=0$由Apollo Fusion目标中心、纵向投影和固定5.3074 m组合几何偏移构造。CARLA碰撞由实际物理包围盒接触决定，两者存在目标中心映射、几何偏移和路径/纵向投影误差。

因此可能出现：

- 碰撞run在collision event时显示$S>0$；
- 未碰撞run停车端点也显示$S>0$；
- ST终点位置与第四章0 m余量不完全相同。

这些现象不应通过平移单个run或修改个别几何偏移强行消除。必须保留原始统一口径，并明确解释：

1. $S>0$只表示越过分析参考边界，不等于CARLA已经碰撞。
2. collision event和碰撞速度是碰撞直接证据。
3. 未碰撞run的近零速度、停车端点和缺少collision event是未碰撞直接证据。
4. ST图使用墙钟速度路径积分；实际制动余量可能使用Localization纵向位移，二者必须分别命名。
5. ST图中最可靠的是两组的相对位置差、斜率差和随时间的变化，不是$S=0$的绝对接触判定。

当前第五章202607271131与202607271206的ST图中，两组最终均越过$S=0$分析边界，但只有202607271131发生碰撞；这正是“分析参考边界不能替代CARLA collision event”的实例。第六章202607271643与202607271211的ST图中，碰撞run越过参考边界，而未碰撞run停在边界前，视觉结局与直接事件一致。两类图都必须保留真实计算结果，不能为了让图形看起来符合预期而改变坐标口径。

#### 17.8.6 推荐报告写法

ST图前先给出统一坐标公式和符号含义，图后按以下顺序解释：

1. $t_1$初始位置差；
2. $t_2$位置差和速度差；
3. 制动阶段曲线斜率变化；
4. 碰撞run碰撞时刻两组位置和速度；
5. 未碰撞run如何继续减速至停车；
6. $S=0$分析边界与CARLA物理碰撞判据的区别。

表格内使用“t1位置”“t2位置”“碰撞同一相对时刻”“最终端点”等简洁中文，不把长LaTeX公式放入单元格。公式统一放在表格外使用`$$...$$`。如果ST图采用其他位置口径，必须在图名、坐标轴和正文中明确标注，不能仍写成`st_position_wall_integral_m`。
