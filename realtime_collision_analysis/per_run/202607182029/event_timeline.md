# 202607182029事件时间线

| 事件 | 统一时间/s | 相对t1/ms | 速度/m/s | 净距/m | 判据 |
|---|---|---|---|---|---|
| t_spawn | — | — | — | — | not archived |
| t_sensor_first_visible | 1784377800.016 | 0.000 | 15.249 | — | first Fusion target source observation |
| t_perception_first | 1784377800.253 | 236.732 | 15.960 | 37.259 | first Fusion target output |
| t_sensor_origin | 1784377800.016 | 0.000 | 15.249 | — | first source frame in 3-frame stable sequence |
| t_perception_stable | 1784377800.253 | 236.732 | 15.960 | 37.259 | output time of stable sequence first frame |
| t_prediction_first | 1784377800.256 | 240.088 | 15.960 | 37.205 | target prediction output |
| t_prediction_static | 1784377800.256 | 240.088 | 15.960 | 37.205 | pred_has_is_static=1 and pred_is_static=1 |
| t_planning_stop | 1784377800.266 | 250.085 | 15.960 | 37.046 | first STOP decision for target |
| t_planning_decel | 1784377800.269 | 252.963 | 15.960 | 36.999 | first target stop trajectory output |
| t_control_brake_command | 1784377800.282 | 266.111 | 15.960 | 36.789 | first /apollo/control output inheriting target trace; payload unavailable |
| t_brake_effective | 1784377800.673 | 656.455 | 16.282 | 30.435 | two consecutive deceleration intervals |
| t_stop | 1784377804.585 | 4568.460 | 0.000 | 3.577 | speed <0.1 m/s for 0.5 s |
| t_collision | — | — | — | — | first CollisionSensor event |
| t_end | 1784377808.000 | 7983.805 | 0.019 | 3.167 | collection end_log |
