# 202607191734事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784453761.129 | 0.000 | 15.164 | — | first Fusion target source observation |
| t_perception_first | 1784453761.393 | 263.722 | 13.886 | 36.244 | first Fusion target output |
| t_sensor_origin | 1784453761.129 | 0.000 | 15.164 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784453761.393 | 263.722 | 13.886 | 36.244 | output time of stable sequence first frame |
| t_prediction_first | 1784453761.397 | 267.625 | 13.886 | 36.190 | target prediction output |
| t_prediction_static | 1784453761.397 | 267.625 | 13.886 | 36.190 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784453761.404 | 274.699 | 13.886 | 36.090 | first STOP decision for target |
| t_planning_decel | 1784453761.407 | 277.566 | 13.886 | 36.050 | first target stop trajectory output |
| t_control_brake_command | 1784453761.418 | 289.160 | 13.886 | 35.888 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | — | — | — | — | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784453772.000 | 10870.714 | 1.539 | 2.828 | collection end_log |
