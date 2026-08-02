# 202607181818事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784369939.144 | 0.000 | 15.546 | — | first Fusion target source observation |
| t_perception_first | 1784369939.399 | 255.273 | 15.583 | 36.548 | first Fusion target output |
| t_sensor_origin | 1784369939.144 | 0.000 | 15.546 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784369939.399 | 255.273 | 15.583 | 36.548 | output time of stable sequence first frame |
| t_prediction_first | 1784369939.404 | 259.765 | 15.583 | 36.478 | target prediction output |
| t_prediction_static | 1784369939.404 | 259.765 | 15.583 | 36.478 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784369939.429 | 285.020 | 15.587 | 36.084 | first STOP decision for target |
| t_planning_decel | 1784369939.436 | 291.857 | 15.587 | 35.977 | first target stop trajectory output |
| t_control_brake_command | 1784369939.442 | 297.835 | 15.587 | 35.884 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784369939.602 | 457.735 | 15.409 | 33.402 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784369949.000 | 9856.076 | 8.167 | -6.972 | collection end_log |
