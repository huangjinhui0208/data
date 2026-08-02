# 202607182021事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784377326.789 | 0.000 | 16.538 | — | first Fusion target source observation |
| t_perception_first | 1784377327.027 | 238.003 | 16.556 | 36.375 | first Fusion target output |
| t_sensor_origin | 1784377326.789 | 0.000 | 16.538 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784377327.027 | 238.003 | 16.556 | 36.375 | output time of stable sequence first frame |
| t_prediction_first | 1784377327.031 | 241.831 | 16.556 | 36.312 | target prediction output |
| t_prediction_static | 1784377327.031 | 241.831 | 16.556 | 36.312 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784377327.055 | 266.525 | 16.556 | 35.903 | first STOP decision for target |
| t_planning_decel | 1784377327.061 | 272.218 | 16.556 | 35.809 | first target stop trajectory output |
| t_control_brake_command | 1784377327.073 | 284.258 | 16.559 | 35.609 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784377327.453 | 663.760 | 16.414 | 29.332 | two consecutive deceleration intervals |
| t_stop | 1784377330.010 | 3220.663 | 0.002 | 2.429 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784377337.000 | 10211.107 | 0.044 | 1.885 | collection end_log |
