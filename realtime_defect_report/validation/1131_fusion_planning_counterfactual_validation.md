# 1131 Fusion/Planning 时延插入反事实校验

## Overall Assessment: Share with caveats

问题：在1131原有 baseline 恢复反事实中，将实际日志里的 Fusion 与 Planning 异常超额时延移入首次响应链，是否会重新造成碰撞。

数据截至：2026-07-27实验日志；复核日期：2026-08-06；时区：Asia/Shanghai。

## Methodology Review

- 复用 `analyze_realtime_defects.py::counterfactual` 的物理模型，不更改碰撞几何修正或1131等效制动能力。
- `D_delay` 使用 t1 到候选 t2 的 Localization 墙钟速度梯形积分。
- baseline恢复时延为7个baseline主分析run的中位数300.358057 ms。
- Fusion新增量为507.438898−99.783659=407.655239 ms。
- Planning新增量为472.595000−14.098600=458.496400 ms。
- 主联合情景按两个异常超额区间的实际时间并集481.400251 ms；串行相加866.151639 ms仅作保守上界。

## Calculation Spot-Checks

- 原反事实复现：D_delay=4.867182 m、余量=+5.494593 m、预测不碰撞，与 `counterfactual_model.csv` 完全一致。
- 观测参考复现：候选时延设为799.635887 ms时，D_delay=13.432028 m、D2=24.826178 m、预测撞击速度=7.988017 m/s，与观测值一致。
- Fusion原始日志：seq1322与seq1323的header_time差为507.438898 ms；相邻obs_time差为99.783659 ms。
- Planning原始日志：seq690 total=472.595 ms；PIECEWISE_JERK_SPEED=460.138 ms；窗口中位数=14.0986 ms。
- 距离恒等式：每个候选情景均满足 D2=D1−D_delay。
- 碰撞翻转边界：相对300.358057 ms基线增加约195.691466 ms时，模型余量穿过0 m。
- 正常参考敏感性：Fusion用gap p90、Planning用p90或100 ms预算作为正常参考时，新增量仍大于翻转边界且都预测碰撞。

## Required Caveats

- 真实1131中这两个异常发生在实际t2之后；把它们移入首次响应链是用户指定的假设性压力情景，不是实际事件重放。
- Fusion和Planning异常高度重叠，不能把两项超额机械相加作为主结果。
- 主结果保持1131实际等效减速度4.466860 m/s²。各插入情景若要避免碰撞，需要整个剩余制动阶段的持续等效减速度提高约21%至25%。
- 模型不重跑Apollo闭环、车辆动力学或障碍物交互，只回答在固定制动能力和观测速度轨迹条件下的局部结果。

## Reproducible Artifact

- `realtime_defect_report/notebooks/1131_fusion_planning_delay_counterfactual.ipynb`
