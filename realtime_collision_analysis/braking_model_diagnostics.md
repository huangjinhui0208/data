# Baseline经验制动模型诊断

- 样本数：6
- run：202607171703, 202607171706, 202607171721, 202607171726, 202607171735, 202607171738
- 模型：`D_brake_required(v)=k_median×v²`
- k中位数：0.10196848 s²/m
- k bootstrap 95%区间：0.09265057–0.10964053 s²/m
- 等效减速度中位数：4.9045 m/s²
- 经验制动位移均值：24.8755 m
- 距离口径：3-D Localization displacement from effective brake onset to the post-t2 minimum-speed sample, conditioned on reaching v<0.1 m/s
- 适用范围：当前Town04静止障碍物、约15–17 m/s实验。

碰撞run的碰撞前行驶距离未用于拟合近停制动位移。模型通过baseline近停样本估计碰撞run所需制动距离。严格停车保持状态与近停代理状态已在`run_metrics.csv`分字段记录。
