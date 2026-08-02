# 202607171738事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784281122.817 | 0.000 | 15.384 | — | first Fusion target source observation |
| t_perception_first | 1784281123.065 | 248.413 | 15.501 | 36.769 | first Fusion target output |
| t_sensor_origin | 1784281122.817 | 0.000 | 15.384 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784281123.065 | 248.413 | 15.501 | 36.769 | output time of stable sequence first frame |
| t_prediction_first | 1784281123.069 | 252.575 | 15.501 | 36.705 | target prediction output |
| t_prediction_static | 1784281123.069 | 252.575 | 15.501 | 36.705 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784281123.078 | 261.503 | 15.501 | 36.566 | first STOP decision for target |
| t_planning_decel | 1784281123.081 | 264.406 | 15.501 | 36.521 | first target stop trajectory output |
| t_control_brake_command | 1784281123.087 | 270.483 | 15.501 | 36.427 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784281123.184 | 367.443 | 15.378 | 34.931 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784281130.000 | 7183.478 | 0.313 | 3.857 | collection end_log |
