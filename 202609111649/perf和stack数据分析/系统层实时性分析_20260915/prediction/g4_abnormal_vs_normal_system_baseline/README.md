# G4 local-jump-only vs matched-normal system baseline

本目录只比较未命中任何严格P99规则的4个local jump与12个匹配正常帧。应用选帧及完整S1行在打点数据分析的同名目录；本目录保存独立S5原始窗口、callback状态、阶段状态、匹配差值和校验。

| 指标 | local-jump-only中位数 ms | 正常对照中位数 ms |
| --- | ---: | ---: |
| execution_ms | 13.483440 | 3.279552 |
| running_ms | 3.035724 | 2.846592 |
| runnable_ms | 0.036000 | 0.014500 |
| blocked_ms | 10.411500 | 0.122500 |
| max_sched_delay_ms | 0.017000 | 0.012000 |

逐案例结果见 `per_case_system_baseline.csv`；每个异常帧与三个控制帧的直接差值见 `matched_pair_system_deltas.csv`。这些差值是匹配描述，不是独立因果效应。blocked只表示睡眠/等待状态，具体锁、future、worker或设备对象仍需对象级打点。

12个异常—对照配对中，execution差值为正 12/12，blocked差值为正 12/12；配对execution差值中位数 10.297632 ms，blocked差值中位数 9.990500 ms。四个案例的blocked增量占墙钟增量90.5%–98.9%。最长NotifyTask实例的blocked中位数为9.829500 ms（正常0.108500 ms）；最大连续调度等待中位数仅0.017000 ms（正常0.012000 ms）。因此这组local jump表现为NotifyTask路径内的睡眠/等待延长，不是明显的CPU就绪排队。等待对象仍未知。

CPU stack覆盖较稀：异常组全部帧TID样本6个、callback样本1个；正常组分别4个和3个。调用栈不足以形成组间路径基线，本结论依赖完整sched状态分解。阶段表从此前同一校准与同一规范化sched缓存生成的全量选帧结果中严格过滤这16帧。

状态闭合最大误差：callback 1.585e-07 ms，阶段 2.274e-07 ms；16个callback的unknown合计为0。
