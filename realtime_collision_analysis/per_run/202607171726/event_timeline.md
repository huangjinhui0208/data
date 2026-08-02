# 202607171726事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784280441.275 | 0.000 | 15.494 | — | first Fusion target source observation |
| t_perception_first | 1784280441.517 | 241.912 | 15.564 | 36.367 | first Fusion target output |
| t_sensor_origin | 1784280441.275 | 0.000 | 15.494 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784280441.517 | 241.912 | 15.564 | 36.367 | output time of stable sequence first frame |
| t_prediction_first | 1784280441.521 | 245.670 | 15.564 | 36.308 | target prediction output |
| t_prediction_static | 1784280441.521 | 245.670 | 15.564 | 36.308 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784280441.530 | 254.571 | 15.564 | 36.170 | first STOP decision for target |
| t_planning_decel | 1784280441.533 | 257.815 | 15.564 | 36.120 | first target stop trajectory output |
| t_control_brake_command | 1784280441.548 | 272.668 | 15.564 | 35.890 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784280441.609 | 333.253 | 15.407 | 34.954 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784280451.000 | 9724.711 | 8.951 | -6.069 | collection end_log |
