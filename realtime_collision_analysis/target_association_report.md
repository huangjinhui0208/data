# 目标关联报告

| run | CARLA actor | Apollo ID链 | 位置误差中位数/m | 匹配帧数 | 置信度 | 结论 |
|---|---|---|---|---|---|---|
| 202607171703 | — | ['250'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607171706 | — | ['17'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607171721 | — | ['13'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607171726 | — | ['12'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607171735 | — | ['15'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607171738 | — | ['15'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181815 | — | ['18'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181818 | — | ['12'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181844 | — | ['14'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181854 | — | ['11'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181955 | — | ['13'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607181958 | — | ['14'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182007 | — | ['40'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182012 | — | ['14'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182017 | — | ['19'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182021 | — | ['34'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182026 | — | ['19'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607182029 | — | ['21'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607191727 | 155 | ['55'] | 1.466 | 23 | HIGH | CARLA碰撞目标history经y轴转换后与Apollo Fusion目标多帧匹配 |
| 202607191734 | — | ['39'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607191739 | 155 | ['3'] | 1.612 | 19 | HIGH | CARLA碰撞目标history经y轴转换后与Apollo Fusion目标多帧匹配 |
| 202607201555 | — | ['29'] | — | — | MEDIUM_HIGH | Planning STOP目标与静态连续Fusion目标一致 |
| 202607201611 | 155 | ['44'] | 1.456 | 22 | HIGH | CARLA碰撞目标history经y轴转换后与Apollo Fusion目标多帧匹配 |

碰撞run执行CARLA y轴到Apollo y轴的符号转换并进行多帧插值匹配。安全run以Planning STOP目标、Prediction静态语义和Fusion连续轨迹联合确定。
