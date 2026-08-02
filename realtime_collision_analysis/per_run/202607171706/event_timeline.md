# 202607171706事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784279265.799 | 0.000 | 16.362 | — | first Fusion target source observation |
| t_perception_first | 1784279266.035 | 235.850 | 16.430 | 35.493 | first Fusion target output |
| t_sensor_origin | 1784279265.799 | 0.000 | 16.362 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784279266.035 | 235.850 | 16.430 | 35.493 | output time of stable sequence first frame |
| t_prediction_first | 1784279266.042 | 243.647 | 16.430 | 35.365 | target prediction output |
| t_prediction_static | 1784279266.042 | 243.647 | 16.430 | 35.365 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784279266.053 | 254.507 | 16.430 | 35.187 | first STOP decision for target |
| t_planning_decel | 1784279266.057 | 258.226 | 16.430 | 35.126 | first target stop trajectory output |
| t_control_brake_command | 1784279266.061 | 262.261 | 16.430 | 35.060 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784279266.151 | 352.199 | 16.276 | 33.591 | two consecutive deceleration intervals |
| t_stop | — | — | — | — | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784279275.000 | 9201.292 | 4.289 | -4.383 | collection end_log |
