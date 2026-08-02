# 202607201611事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784535189.159 | 0.000 | 15.993 | — | first Fusion target source observation |
| t_perception_first | 1784535189.431 | 271.929 | 16.348 | 35.330 | first Fusion target output |
| t_sensor_origin | 1784535189.159 | 0.000 | 15.993 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784535189.431 | 271.929 | 16.348 | 35.330 | output time of stable sequence first frame |
| t_prediction_first | 1784535189.435 | 275.874 | 16.348 | 35.265 | target prediction output |
| t_prediction_static | 1784535189.435 | 275.874 | 16.348 | 35.265 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784535189.452 | 292.646 | 16.668 | 34.988 | first STOP decision for target |
| t_planning_decel | 1784535189.457 | 297.324 | 16.668 | 34.910 | first target stop trajectory output |
| t_control_brake_command | 1784535189.462 | 302.589 | 16.668 | 34.823 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784535190.070 | 910.625 | 17.976 | 24.169 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | 1784535192.629 | 3469.448 | 0.239 | -9.116 | first CollisionSensor event |
| t_end | 1784535199.000 | 9840.528 | 0.088 | -10.012 | collection end_log |
