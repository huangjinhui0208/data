# 202607181854事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784372086.891 | 0.000 | 15.760 | — | first Fusion target source observation |
| t_perception_first | 1784372087.161 | 270.185 | 15.787 | 35.740 | first Fusion target output |
| t_sensor_origin | 1784372086.891 | 0.000 | 15.760 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784372087.161 | 270.185 | 15.787 | 35.740 | output time of stable sequence first frame |
| t_prediction_first | 1784372087.166 | 274.584 | 15.787 | 35.670 | target prediction output |
| t_prediction_static | 1784372087.166 | 274.584 | 15.787 | 35.670 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784372087.178 | 286.925 | 15.787 | 35.475 | first STOP decision for target |
| t_planning_decel | 1784372087.182 | 290.437 | 15.787 | 35.420 | first target stop trajectory output |
| t_control_brake_command | 1784372087.186 | 294.712 | 15.787 | 35.352 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784372087.378 | 486.686 | 15.693 | 32.323 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784372096.000 | 9108.705 | 7.713 | -7.541 | collection end_log |
