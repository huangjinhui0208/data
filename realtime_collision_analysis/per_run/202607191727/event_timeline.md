# 202607191727事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784453385.099 | 0.000 | 15.665 | — | first Fusion target source observation |
| t_perception_first | 1784453385.383 | 283.272 | 16.785 | 34.858 | first Fusion target output |
| t_sensor_origin | 1784453385.099 | 0.000 | 15.665 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784453385.383 | 283.272 | 16.785 | 34.858 | output time of stable sequence first frame |
| t_prediction_first | 1784453385.386 | 287.069 | 16.785 | 34.795 | target prediction output |
| t_prediction_static | 1784453385.386 | 287.069 | 16.785 | 34.795 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784453385.398 | 298.165 | 16.785 | 34.610 | first STOP decision for target |
| t_planning_decel | 1784453385.402 | 302.312 | 16.785 | 34.541 | first target stop trajectory output |
| t_control_brake_command | 1784453385.409 | 309.824 | 16.785 | 34.415 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784453385.909 | 809.268 | 17.021 | 25.914 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | 1784453391.180 | 6080.715 | 0.315 | -9.076 | first CollisionSensor event |
| t_end | 1784453398.000 | 12900.616 | 7.077 | -32.722 | collection end_log |
