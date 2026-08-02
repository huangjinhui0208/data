# 202607182012事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784376808.314 | 0.000 | 15.634 | — | first Fusion target source observation |
| t_perception_first | 1784376808.575 | 260.690 | 15.124 | 36.619 | first Fusion target output |
| t_sensor_origin | 1784376808.314 | 0.000 | 15.634 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784376808.575 | 260.690 | 15.124 | 36.619 | output time of stable sequence first frame |
| t_prediction_first | 1784376808.581 | 266.689 | 15.124 | 36.529 | target prediction output |
| t_prediction_static | 1784376808.581 | 266.689 | 15.124 | 36.529 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784376808.589 | 274.952 | 15.124 | 36.405 | first STOP decision for target |
| t_planning_decel | 1784376808.592 | 277.906 | 15.124 | 36.361 | first target stop trajectory output |
| t_control_brake_command | 1784376808.601 | 286.664 | 14.810 | 36.230 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | — | — | — | — | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784376818.000 | 9685.565 | 10.770 | -7.947 | collection end_log |
