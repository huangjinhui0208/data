# 202607181958事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784375911.134 | 0.000 | 15.391 | — | first Fusion target source observation |
| t_perception_first | 1784375911.393 | 259.425 | 15.458 | 36.031 | first Fusion target output |
| t_sensor_origin | 1784375911.134 | 0.000 | 15.391 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784375911.393 | 259.425 | 15.458 | 36.031 | output time of stable sequence first frame |
| t_prediction_first | 1784375911.397 | 263.597 | 15.458 | 35.967 | target prediction output |
| t_prediction_static | 1784375911.397 | 263.597 | 15.458 | 35.967 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784375911.409 | 275.527 | 15.458 | 35.782 | first STOP decision for target |
| t_planning_decel | 1784375911.413 | 278.971 | 15.458 | 35.729 | first target stop trajectory output |
| t_control_brake_command | 1784375911.426 | 292.223 | 15.499 | 35.524 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784375911.584 | 450.687 | 15.351 | 33.077 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784375920.000 | 8866.190 | 0.791 | — | collection end_log |
