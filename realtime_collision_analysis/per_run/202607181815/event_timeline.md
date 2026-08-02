# 202607181815事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784369761.488 | 0.000 | 15.222 | — | first Fusion target source observation |
| t_perception_first | 1784369761.783 | 294.165 | 15.300 | 37.089 | first Fusion target output |
| t_sensor_origin | 1784369761.488 | 0.000 | 15.222 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784369761.783 | 294.165 | 15.300 | 37.089 | output time of stable sequence first frame |
| t_prediction_first | 1784369761.787 | 298.356 | 15.300 | 37.025 | target prediction output |
| t_prediction_static | 1784369761.787 | 298.356 | 15.300 | 37.025 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784369761.803 | 314.531 | 15.353 | 36.777 | first STOP decision for target |
| t_planning_decel | 1784369761.808 | 319.550 | 15.353 | 36.700 | first target stop trajectory output |
| t_control_brake_command | 1784369761.815 | 326.908 | 15.353 | 36.587 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784369761.968 | 479.986 | 15.226 | 34.244 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784369770.000 | 8511.554 | 1.270 | 6.158 | collection end_log |
