# 车速条件隐形 Deadline 多组实验

## 1. 当前证据边界

脚本区分三种时间：

1. Apollo Trace 内部 E2E：Perception 开始到 Planning 输出。
2. Bridge API 响应：稳定观测到 `carla_actor.apply_control()` 返回。
3. 物理响应：稳定观测到 Localization 速度序列首次出现持续有效减速。

物理隐形 Deadline 只与第 3 项比较。Bridge API 返回时间不能直接称为车辆开始制动。

## 2. 公式与两种 Deadline

```text
v = speed_kmh / 3.6
D_brake = v^2 / (2a)
D2_safety = D_brake + v * actuator_delay_s + safety_margin_m
D2_collision = D_brake + v * actuator_delay_s

D1_safety = D2_safety + v * target_deadline_s
D1_collision = D2_collision + v * target_deadline_s
```

80 km/h、10 m/s²、5 m裕度、200 ms时：

```text
D1_safety    = 34.1358 m
D1_collision = 29.1358 m
```

因此34.1 m对应的是“200 ms后仍保留5 m裕度”，真正的理想碰撞Deadline约为425 ms。若要构造200 ms碰撞边界，应使用约29.1 m，或者保留34.1 m并把碰撞组响应提高到425 ms以上。

## 3. Bridge配置

配置文件：`carla_apollo10.0_bridge/carla_bridge/config/settings.yaml`

```yaml
control_delay_injection:
  enabled: True
  delay_ms: 0.0
  activation_speed_mps: 7.5
  brake_threshold_percentage: 1.0
  queue_max_messages: 64
  log_all_delayed_commands: False
  log_dir: "/apollo/data/log"
  log_csv: "scb_control_delay_{wall_time_iso}.csv"
```

- `enabled=True`：Bridge启动后立即创建CSV并写入`INITIALIZED`配置行。
- `delay_ms=0`：不加入人为等待，但保持与注入组相同代码路径。
- `activation_speed_mps`：车辆首次达到该速度后锁存为`ARMED`。
- `brake_threshold_percentage`：ARMED后第一条达到该值的制动命令触发注入。
- `queue_max_messages`：队列硬上限。Apollo `/apollo/control`实测约100 Hz，300 ms延迟约占30条。
- `log_all_delayed_commands=False`：正式实验必须保持关闭，只记录启动、首次制动和异常。

每次实验必须完全重启Bridge，以清除ARMED、TRIGGERED和队列状态。

## 4. CSV v2字段语义

`scb_control_delay_v2`分别记录：

- `receive_*`：Bridge收到ControlCommand；
- `release_*`：配置等待结束；
- `apply_call_start_*`：调用CARLA API之前；
- `apply_call_end_*`：CARLA API返回之后；
- `actual_delay_ms`：接收到延迟释放的实际等待；
- `api_completion_delay_ms`：接收到API返回；
- `api_call_duration_ms`：CARLA API调用本身耗时。

CSV中的API时间不是物理制动力开始时间。离线分析使用Localization持续减速检测物理终点。

## 5. 每个案例必须保存

```text
D:\data\scb_data\<time>\
  log\
    perception.log.INFO.*
    prediction.log.INFO.*
    planning.log.INFO.*
    localization.log.INFO.*
    control.log.INFO.*
    scb_control_delay_*.csv
    carla_collision_events_*.csv/jsonl   # 仅碰撞案例
  trace\
    events\...
    message_context\...
    trace_anchor\...
  bridge.log
  bridge_settings.yaml                  # 本次运行实际配置快照
  scb_experiment.yaml
```

`scb_experiment.yaml`至少填写：

```yaml
group: calibration
repeat_index: 1
configured_bridge_delay_ms: 0.0
target_total_response_ms: null
obstacle_spawn_wall_time_unix_ns: null
analysis_end_wall_time_unix_ns: null
obstacle_spawn_distance_m: 50.0
apollo_target_id: null
notes: ""
```

没有`bridge.log`和实际配置快照时，无法证明运行进程加载了哪一份Bridge源码。

## 6. D1距离参考必须校准

Apollo `rel_forward`通常是actor中心到中心，而制动公式通常使用保险杠净距离。必须在`implicit_deadline_config.yaml`中填写经测量的修正：

```yaml
d1_distance_reference: bumper_clearance
d1_reference_verified_for_physical_model: true
longitudinal_distance_offset_m: <中心距离转换为净距离的修正，通常为负数>
```

在该项仍为`false`时，脚本可以输出诊断数据，但不会给出“碰撞由deadline miss解释”的强因果候选。

## 7. 单案例分析

独立运行，不要求功能分类先通过：

```powershell
python D:\data\anlysis_case\implicit_deadline_analyzer.py `
  --case-dir D:\data\scb_data\<time> `
  --out-dir D:\data\scb_data\<time>\scb_analysis
```

40 km/h调试案例可以临时覆盖速度：

```powershell
python D:\data\anlysis_case\implicit_deadline_analyzer.py `
  --case-dir D:\data\scb_data\<time> `
  --out-dir D:\data\scb_data\<time>\scb_analysis `
  --speed-kmh 40
```

输出：

- `implicit_deadline_result.json`
- `implicit_deadline_trace_frames.csv`

只有检测到t1之后的有效制动、CARLA API证据和Localization持续减速，结果才会是`ANALYZED`。

## 8. 正式实验顺序

1. 先做3次pilot：0 ms、已知100 ms、再次0 ms，验证可逆性和证据链。
2. 初始校准10次：Bridge `enabled=True, delay_ms=0`，障碍物先在50 m生成。
3. 校准汇总：

```powershell
python D:\data\anlysis_case\scb_calibrate.py `
  --cases-root D:\data\scb_data `
  --group calibration `
  --out-dir D:\data\scb_data\calibration
```

4. 根据检测滞后和实测减速度固定正式场景D1。
5. 正式baseline组至少10次，`delay_ms=0`。
6. e2e200和e2e300组先各做3次pilot，根据实测物理响应调整固定Bridge延迟；正式组中不得逐帧自适应。
7. 三组完成后汇总：

```powershell
python D:\data\anlysis_case\scb_group_summary.py `
  --cases-root D:\data\scb_data `
  --out-dir D:\data\scb_data\summary
```

汇总只有在每组至少10次、至少8次`ANALYZED`、所有碰撞结局已知且元数据延迟与Bridge证据一致时才返回`PASS`。

