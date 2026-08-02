# 202607182007事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784376582.412 | 0.000 | 16.697 | — | first Fusion target source observation |
| t_perception_first | 1784376582.705 | 292.655 | 17.068 | 34.262 | first Fusion target output |
| t_sensor_origin | 1784376582.412 | 0.000 | 16.697 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784376582.705 | 292.655 | 17.068 | 34.262 | output time of stable sequence first frame |
| t_prediction_first | 1784376582.709 | 297.185 | 17.068 | 34.184 | target prediction output |
| t_prediction_static | 1784376582.709 | 297.185 | 17.068 | 34.184 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784376582.723 | 310.425 | 17.068 | 33.958 | first STOP decision for target |
| t_planning_decel | 1784376582.728 | 315.891 | 17.068 | 33.864 | first target stop trajectory output |
| t_control_brake_command | 1784376582.743 | 330.412 | 17.068 | 33.616 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784376583.063 | 650.257 | 17.147 | 28.121 | two consecutive deceleration intervals |
| t_stop | 1784376586.159 | 3746.491 | 0.002 | -6.353 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784376597.000 | 14587.699 | 5.326 | -15.342 | collection end_log |
