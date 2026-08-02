# 202607182017事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784377066.961 | 0.000 | 16.487 | — | first Fusion target source observation |
| t_perception_first | 1784377067.203 | 242.101 | 16.220 | 36.241 | first Fusion target output |
| t_sensor_origin | 1784377066.961 | 0.000 | 16.487 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784377067.203 | 242.101 | 16.220 | 36.241 | output time of stable sequence first frame |
| t_prediction_first | 1784377067.208 | 246.650 | 16.220 | 36.168 | target prediction output |
| t_prediction_static | 1784377067.208 | 246.650 | 16.220 | 36.168 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784377067.218 | 256.395 | 16.220 | 36.010 | first STOP decision for target |
| t_planning_decel | 1784377067.221 | 259.925 | 16.220 | 35.953 | first target stop trajectory output |
| t_control_brake_command | 1784377067.230 | 269.261 | 16.157 | 35.802 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784377067.603 | 641.849 | 15.995 | 29.793 | two consecutive deceleration intervals |
| t_stop | 1784377070.081 | 3120.049 | 0.002 | 4.943 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784377077.000 | 10038.868 | 5.366 | -4.227 | collection end_log |
