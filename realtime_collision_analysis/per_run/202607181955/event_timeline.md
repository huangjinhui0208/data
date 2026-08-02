# 202607181955事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784375746.638 | 0.000 | 16.589 | — | first Fusion target source observation |
| t_perception_first | 1784375746.880 | 241.874 | 16.657 | 34.900 | first Fusion target output |
| t_sensor_origin | 1784375746.638 | 0.000 | 16.589 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784375746.880 | 241.874 | 16.657 | 34.900 | output time of stable sequence first frame |
| t_prediction_first | 1784375746.885 | 246.013 | 16.657 | 34.831 | target prediction output |
| t_prediction_static | 1784375746.885 | 246.013 | 16.657 | 34.831 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784375746.902 | 263.676 | 16.657 | 34.537 | first STOP decision for target |
| t_planning_decel | 1784375746.908 | 269.429 | 16.657 | 34.441 | first target stop trajectory output |
| t_control_brake_command | 1784375746.920 | 281.732 | 16.657 | 34.236 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784375747.105 | 466.492 | 16.510 | 31.165 | two consecutive deceleration intervals |
| t_stop | 1784375751.386 | 4747.814 | 0.000 | 3.489 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784375755.000 | 8361.501 | 0.610 | 2.572 | collection end_log |
