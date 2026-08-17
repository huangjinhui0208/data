# 证据附录

## A. 数据和脚本

- 原始目录：`../../第二次实验/`，454个文件。
- 全量文件清单：`../extracted/file_inventory.csv`；模式清单：`../extracted/schema_inventory.json`。
- 逐run清单：`../tables/run_inventory.csv`；观测主表：`../tables/run_level_metrics.csv`。
- 组统计：`../tables/group_summary.csv`；阶段统计：`../tables/stage_latency_summary.csv`。
- 案例对照：`../tables/collision_case_comparison.csv`；缺陷分类：`../tables/realtime_defect_evidence_matrix.csv`。
- 身份审计：`../tables/target_identity_audit.csv`；模型反事实：`../tables/counterfactual_model.csv`。
- 复算脚本：`../scripts/analyze_realtime_defects.py`；成文脚本：`../scripts/generate_realtime_defect_report.py`。

证据优先级为原始数据 > 可复算结果 > 旧CSV/JSON > 旧报告。主分析从原始数据重算；旧产物仅做差异核验。

## B. 事件、距离和端点定义

1. `t1`：首次连续3个稳定Fusion周期中第一帧目标的source time。
2. `t2`：首个满足连续两个间隔减速度≥0.5 m/s²且后0.3 s速度下降≥0.3 m/s的区间终点。
3. `D1_clear=D1_center−5.3074 m`。
4. `D_delay=∫v(t)dt_wall`，只用墙钟Localization速度梯形积分。
5. `D2=D1_clear−D_delay`。
6. 完整停车run的`D_brake,data`取统一低速最小速度端点；碰撞run只保留截断距离/时间。
7. `M0=D2−D_brake,data`；`M6=M0−6 m`。只对完整停车run计算观测deadline。
8. sensor→Fusion、Fusion→Prediction、Prediction→Planning STOP、Planning STOP→Control使用目标trace和单调时钟；Control→t2跨到墙钟物理端点。

## C. 逐run观测主表

| run | 组别 | 主分析 | T_e2e/ms | D1/m | D_delay/m | D2/m | D_brake,data/m | M0/m | 结局 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| 202607271031 | baseline | 是 | 299.655 | 39.891 | 4.673 | 35.218 | 29.880 | 5.338 | 停车/无碰撞事件 |
| 202607271048 | baseline | 是 | 300.358 | 38.744 | 5.254 | 33.490 | 30.019 | 3.471 | 停车/无碰撞事件 |
| 202607271054 | baseline | 是 | 299.977 | 39.632 | 5.367 | 34.264 | 31.525 | 2.740 | 停车/无碰撞事件 |
| 202607271059 | baseline | 是 | 399.750 | 40.155 | 7.006 | 33.149 | 30.403 | 2.746 | 停车/无碰撞事件 |
| 202607271104 | baseline | 是 | 399.646 | 38.784 | 6.936 | 31.848 | 27.880 | 3.968 | 停车/无碰撞事件 |
| 202607271108 | baseline | 是 | 300.641 | 38.759 | 5.284 | 33.475 | 29.963 | 3.512 | 停车/无碰撞事件 |
| 202607271113 | baseline | 是 | 299.796 | 38.817 | 5.231 | 33.586 | 30.236 | 3.351 | 停车/无碰撞事件 |
| 202607271131 | 300 ms | 是 | 799.636 | 38.258 | 13.432 | 24.826 | NA | NA | 碰撞，撞击前7.988 m/s |
| 202607271202 | 300 ms | 是 | 699.155 | 39.616 | 11.749 | 27.867 | 27.319 | 0.548 | 停车/无碰撞事件 |
| 202607271206 | 300 ms | 否 | 800.753 | 39.790 | 13.741 | 26.049 | 27.685 | -1.636 | 结局不确定 |
| 202607271211 | 300 ms | 是 | 700.167 | 40.202 | 11.999 | 28.203 | 27.185 | 1.018 | 停车/无碰撞事件 |
| 202607271643 | 300 ms | 是 | 893.870 | 36.651 | 15.451 | 21.201 | NA | NA | 碰撞，撞击前11.728 m/s |

## D. 7个baseline最终停车净距

| baseline run | 最终投影净距/m | 0 m碰撞余量M0/m | 说明 |
|---|---:|---:|---|
| 202607271031 | 5.331 | 5.338 | 完整停车、无碰撞事件 |
| 202607271048 | 3.471 | 3.471 | 完整停车、无碰撞事件 |
| 202607271054 | 2.738 | 2.740 | 完整停车、无碰撞事件 |
| 202607271059 | 2.735 | 2.746 | 完整停车、无碰撞事件 |
| 202607271104 | 3.955 | 3.968 | 完整停车、无碰撞事件 |
| 202607271108 | 3.520 | 3.512 | 完整停车、无碰撞事件 |
| 202607271113 | 3.341 | 3.351 | 完整停车、无碰撞事件 |

这里的最终投影净距来自稳定目标几何与统一停车端点；M0来自`D1−D_delay−D_brake,data`。两者用途相近但计算链不同，厘米级差异用于一致性检查。

## E. 12个run的感知连续性和新鲜度

| run | sensor→Fusion/ms | Fusion最大输出间隔/ms | lifecycle最大值/ms | 结局端源数据年龄/ms | 判断 |
|---|---:|---:|---:|---:|---|
| 202607271031 | 208.315 | 117.223 | 228.392 | 299.965 | 未见首个稳定Fusion响应超过500 ms |
| 202607271048 | 234.979 | 112.502 | 247.550 | 2799.843 | 未见首个稳定Fusion响应超过500 ms |
| 202607271054 | 227.212 | 116.793 | 264.961 | 500.017 | 未见首个稳定Fusion响应超过500 ms |
| 202607271059 | 268.629 | 117.523 | 268.629 | 399.791 | 未见首个稳定Fusion响应超过500 ms |
| 202607271104 | 251.648 | 297.602 | 257.022 | 399.615 | 未见首个稳定Fusion响应超过500 ms |
| 202607271108 | 235.998 | 113.378 | 279.834 | 300.107 | 未见首个稳定Fusion响应超过500 ms |
| 202607271113 | 227.746 | 285.799 | 242.370 | 299.697 | 未见首个稳定Fusion响应超过500 ms |
| 202607271131 | 292.885 | 507.439 | 705.892 | 1006.892 | 连续性与新鲜度异常 |
| 202607271202 | 263.520 | 188.028 | 263.520 | 898.791 | 未见首个稳定Fusion响应超过500 ms |
| 202607271206 | 308.372 | 112.789 | 323.988 | 2399.347 | 端点年龄异常但结局不确定 |
| 202607271211 | 261.787 | 188.254 | 262.484 | 699.891 | 未见首个稳定Fusion响应超过500 ms |
| 202607271643 | 319.285 | 115.906 | 322.269 | 599.668 | 单帧/间隔未超阈，碰撞端源数据偏老 |

注意：安全停车后很久不再输出目标会使“结局端源数据年龄”变大，例如`1048`；这不等于行驶关键阶段发生处理超时。因此结局端年龄必须与端点时刻、输出间隔和生命周期一起解释。`1206`的2399.347 ms同样受长端点与不确定结局影响，不能单独归类为感知处理缺陷。

## F. 两起碰撞的截断观测

| run | t2→碰撞/s | 截断制动距离/m | 撞击前速度/(m/s) | 碰撞端几何投影诊断/m | 完整D_brake/M0/deadline |
|---|---:|---:|---:|---:|---|
| 1131 | 2.107 | 27.148 | 7.988 | -2.322 | NA |
| 1643 | 1.599 | 23.378 | 11.728 | -2.177 | NA |

负的碰撞端投影是固定组合偏移下的穿透方向诊断，不替代CollisionSensor事件，也不用于回填完整制动余量。

## G. 反事实模型（C类，不是观测）

| run | 恢复到baseline中位响应 | 回收距离/m | 预测余量/m | 预测结局 | 预测撞击速度/(m/s) |
|---|---:|---:|---:|---|---:|
| 1131 | 300.358 ms | 8.565 | 5.495 | 避碰 | 0.000 |
| 1643 | 300.358 ms | 10.519 | -2.000 | 仍碰撞 | 3.980 |

模型保持各碰撞run从实际t2到接触的能量等效减速度，只提前响应时机；它不重放控制器，也不模拟新的闭环反馈。

## H. 缺陷分类矩阵

| run | 分类 | 主要缺陷 | 置信度 | 结论边界 |
|---|---|---|---|---|
| 202607271131 | RT_DOMINATED_COLLISION | RESPONSE_TOO_LONG + DATA_FRESHNESS/OUTPUT_CONTINUITY_DEGRADATION | MEDIUM_HIGH | Observed chain is direct; baseline-restoration avoidance is model/predicted only |
| 202607271202 | NONCOLLISION_REALTIME_MARGIN_EXHAUSTION | RESPONSE_TOO_LONG | HIGH_FOR_MARGIN; MEDIUM_FOR_PHYSICAL_OUTCOME | Does not establish a universal millisecond threshold |
| 202607271206 | INDETERMINATE | OUTCOME_EVIDENCE_CONFLICT | HIGH_FOR_INDETERMINATE | Excluded from the 11-run main outcome analysis before interpreting direction |
| 202607271211 | NONCOLLISION_REALTIME_MARGIN_EXHAUSTION | RESPONSE_TOO_LONG | HIGH_FOR_MARGIN | 1206 has closer latency to 1131 but uncertain outcome, so it is not used as safe control |
| 202607271643 | MULTI_FACTOR_COLLISION | RESPONSE_TOO_LONG with smaller initial clearance and observed braking differences | HIGH | Baseline-restoration model still predicts contact at reduced impact speed |


## I. 时钟、身份与物理真值边界

主`t1/t2/D_delay/停车端点`使用Apollo/Localization墙钟epoch。Trace内部阶段使用monotonic clock，经trace anchor关联目标trace。只有1131和1643有actor history，可拟合CARLA sim time到wall time；其余run不能估计realtime factor。两个碰撞run的目标身份由Planning STOP ID、Fusion轨迹与CARLA actor history共同支持；无碰撞run没有同等物理真值。

## J. 方法借鉴边界

DriveFI、R-TOD、FADE、D3只作为任务说明中给出的研究组织概念：分别对应受控注入、端到端时序、时间质量到功能退化、时间到距离预算。报告未重新核验这些论文全文与正式书目信息，因此没有把任何具体论文结论作为本地数据结论的来源。
