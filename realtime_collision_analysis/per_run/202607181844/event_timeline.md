# 202607181844事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784371518.867 | 0.000 | 15.548 | — | first Fusion target source observation |
| t_perception_first | 1784371519.107 | 240.063 | 15.603 | 36.405 | first Fusion target output |
| t_sensor_origin | 1784371518.867 | 0.000 | 15.548 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784371519.107 | 240.063 | 15.603 | 36.405 | output time of stable sequence first frame |
| t_prediction_first | 1784371519.113 | 245.232 | 15.603 | 36.324 | target prediction output |
| t_prediction_static | 1784371519.113 | 245.232 | 15.603 | 36.324 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784371519.122 | 254.852 | 15.603 | 36.174 | first STOP decision for target |
| t_planning_decel | 1784371519.126 | 258.487 | 15.603 | 36.117 | first target stop trajectory output |
| t_control_brake_command | 1784371519.133 | 265.654 | 15.627 | 36.005 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784371519.307 | 439.977 | 15.465 | 33.291 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784371528.000 | 9132.666 | 5.025 | -0.714 | collection end_log |
