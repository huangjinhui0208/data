# 202607171721事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784280146.854 | 0.000 | 15.698 | — | first Fusion target source observation |
| t_perception_first | 1784280147.082 | 228.506 | 15.742 | 36.417 | first Fusion target output |
| t_sensor_origin | 1784280146.854 | 0.000 | 15.698 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784280147.082 | 228.506 | 15.742 | 36.417 | output time of stable sequence first frame |
| t_prediction_first | 1784280147.090 | 236.712 | 15.742 | 36.288 | target prediction output |
| t_prediction_static | 1784280147.090 | 236.712 | 15.742 | 36.288 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784280147.104 | 250.796 | 15.742 | 36.067 | first STOP decision for target |
| t_planning_decel | 1784280147.107 | 253.826 | 15.742 | 36.019 | first target stop trajectory output |
| t_control_brake_command | 1784280147.121 | 267.356 | 15.742 | 35.807 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784280147.184 | 329.872 | 15.581 | 34.830 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784280154.000 | 7146.371 | 1.160 | 5.813 | collection end_log |
