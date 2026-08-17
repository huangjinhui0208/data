# Data quality audit

## Scope and immutability

- Scope: `/Users/huangjinhui/Desktop/萨卡班/data/第二次实验/300ms/202607271211` only.
- Raw files: 32; SHA-256 saved for every file.
- Raw files were not modified.

## Time and endpoint quality

- t_world is left-censored no later than t_sample.
- t_demand and t_observable are unavailable.
- t_sample is the source epoch of the first frame in a retrospectively identified three-frame stable sequence.
- Same-host source-to-Control trace is available; event-local cross-host Bridge apply is unavailable.
- t_phys 0.5/1.0 m/s^2 and all median3 sensitivities agree at 700.167 ms; raw 0.3 m/s^2 gives 601.118 ms.

## Physical outcome quality

- Canonical D_response is the wall-clock speed trapezoid: 11.999340611265 m.
- Minimum-speed proxy is used; strict stop-hold is absent.
- No CARLA CollisionSensor or actor-history file exists, so direct noncollision/physical identity is unavailable.
- No parsed record exists.

## Model boundary

- Four dynamic residual-budget rows use seven disjoint baseline runs, but calibration was not locked by the evaluated run and the domain is unvalidated.
- Model crossings do not establish a primary deadline or guarantee-loss time.
