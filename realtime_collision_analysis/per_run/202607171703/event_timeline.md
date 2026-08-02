# 202607171703事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784279046.475 | 0.000 | 15.585 | — | first Fusion target source observation |
| t_perception_first | 1784279046.703 | 227.240 | 15.866 | 35.885 | first Fusion target output |
| t_sensor_origin | 1784279046.475 | 0.000 | 15.585 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784279046.703 | 227.240 | 15.866 | 35.885 | output time of stable sequence first frame |
| t_prediction_first | 1784279046.710 | 234.732 | 15.866 | 35.766 | target prediction output |
| t_prediction_static | 1784279046.710 | 234.732 | 15.866 | 35.766 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784279046.719 | 243.499 | 15.866 | 35.627 | first STOP decision for target |
| t_planning_decel | 1784279046.722 | 246.631 | 15.866 | 35.578 | first target stop trajectory output |
| t_control_brake_command | 1784279046.735 | 259.521 | 15.866 | 35.373 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784279046.803 | 328.138 | 15.805 | 34.288 | two consecutive deceleration intervals |
| t_stop | 1784279049.221 | 2745.583 | 0.002 | 9.951 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784279056.000 | 9524.639 | 7.430 | -3.242 | collection end_log |
