# 202607171735事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784280947.596 | 0.000 | 15.608 | — | first Fusion target source observation |
| t_perception_first | 1784280947.833 | 237.196 | 15.677 | 36.528 | first Fusion target output |
| t_sensor_origin | 1784280947.596 | 0.000 | 15.608 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784280947.833 | 237.196 | 15.677 | 36.528 | output time of stable sequence first frame |
| t_prediction_first | 1784280947.836 | 240.535 | 15.677 | 36.476 | target prediction output |
| t_prediction_static | 1784280947.836 | 240.535 | 15.677 | 36.476 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784280947.848 | 252.056 | 15.677 | 36.295 | first STOP decision for target |
| t_planning_decel | 1784280947.851 | 255.134 | 15.677 | 36.247 | first target stop trajectory output |
| t_control_brake_command | 1784280947.863 | 267.496 | 15.677 | 36.054 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784280947.947 | 351.037 | 15.526 | 34.752 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784280955.000 | 7404.366 | 0.890 | 4.678 | collection end_log |
