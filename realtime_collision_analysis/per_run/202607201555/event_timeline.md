# 202607201555事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784534200.091 | 0.000 | 16.138 | — | first Fusion target source observation |
| t_perception_first | 1784534200.439 | 347.317 | 16.154 | 33.962 | first Fusion target output |
| t_sensor_origin | 1784534200.091 | 0.000 | 16.138 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784534200.439 | 347.317 | 16.154 | 33.962 | output time of stable sequence first frame |
| t_prediction_first | 1784534200.444 | 352.071 | 16.154 | 33.886 | target prediction output |
| t_prediction_static | 1784534200.444 | 352.071 | 16.154 | 33.886 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784534200.451 | 359.355 | 16.154 | 33.768 | first STOP decision for target |
| t_planning_decel | 1784534200.454 | 362.301 | 16.154 | 33.720 | first target stop trajectory output |
| t_control_brake_command | 1784534200.467 | 375.599 | 16.154 | 33.506 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784534200.984 | 892.358 | 15.915 | 25.187 | two consecutive deceleration intervals |
| t_stop | 1784534205.046 | 4954.771 | 0.002 | -15.827 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784534211.000 | 10908.523 | 4.113 | -23.598 | collection end_log |
