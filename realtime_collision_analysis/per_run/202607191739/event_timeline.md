# 202607191739事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784454000.708 | 0.000 | 17.336 | — | first Fusion target source observation |
| t_perception_first | 1784454001.522 | 814.085 | 17.768 | 24.514 | first Fusion target output |
| t_sensor_origin | 1784454000.708 | 0.000 | 17.336 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784454001.522 | 814.085 | 17.768 | 24.514 | output time of stable sequence first frame |
| t_prediction_first | 1784454001.525 | 817.495 | 17.768 | 24.453 | target prediction output |
| t_prediction_static | 1784454001.525 | 817.495 | 17.768 | 24.453 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784454001.532 | 824.091 | 17.768 | 24.336 | first STOP decision for target |
| t_planning_decel | 1784454001.535 | 827.356 | 17.768 | 24.278 | first target stop trajectory output |
| t_control_brake_command | 1784454001.544 | 836.209 | 17.768 | 24.120 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784454002.041 | 1332.801 | 17.768 | 15.256 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | 1784454003.439 | 2731.482 | 3.270 | -5.645 | first CollisionSensor event |
| t_end | 1784454011.000 | 10292.272 | 3.096 | -8.335 | collection end_log |
