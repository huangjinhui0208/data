# 202607182026事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784377589.369 | 0.000 | 14.769 | — | first Fusion target source observation |
| t_perception_first | 1784377589.642 | 273.052 | 15.720 | 36.291 | first Fusion target output |
| t_sensor_origin | 1784377589.369 | 0.000 | 14.769 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784377589.642 | 273.052 | 15.720 | 36.291 | output time of stable sequence first frame |
| t_prediction_first | 1784377589.649 | 280.468 | 15.720 | 36.173 | target prediction output |
| t_prediction_static | 1784377589.649 | 280.468 | 15.720 | 36.173 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784377589.661 | 291.846 | 16.088 | 35.992 | first STOP decision for target |
| t_planning_decel | 1784377589.665 | 295.877 | 16.088 | 35.928 | first target stop trajectory output |
| t_control_brake_command | 1784377589.670 | 301.190 | 16.088 | 35.843 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784377590.051 | 682.265 | 16.378 | 29.624 | two consecutive deceleration intervals |
| t_stop | 1784377592.765 | 3396.448 | 0.000 | 0.481 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784377599.000 | 9631.252 | 3.948 | -3.555 | collection end_log |
