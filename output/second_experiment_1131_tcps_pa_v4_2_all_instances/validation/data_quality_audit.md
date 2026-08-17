# Data quality audit

## Scope and immutability

- Scope: only `/Users/huangjinhui/Desktop/萨卡班/data/第二次实验/300ms/202607271131`.
- Raw-file count: 35; SHA-256 recorded for every file.
- No file was written beneath the raw run directory.

## Clocks and endpoints

- Clock alignment: `ALIGNED`; p95 absolute residual 0.661433 ms; median 0.270724 ms.
- t_world: left-censored, known no later than t_sample from the selected target evidence.
- t_demand: unavailable because no independent demand predicate onset is archived.
- t_observable: unavailable because no qualified observer/FOV/occlusion model exists.
- t_sample: source timestamp of the first frame in the first qualifying 3-frame stable target-11 sequence.
- t_phys: first raw-Localization sample satisfying the declared sustained-deceleration detector.
- t2 sensitivity: raw thresholds 0.3/0.5/1.0 m/s² agree; median-3 shifts the endpoint by one 100 ms sample.
- Collision endpoint: direct CARLA collision event, never actor-history first row.

## Distance and physical data

- Sample-relative D_response: wall-clock speed trapezoid, 13.432027958525 m.
- Demand-relative D_response is unavailable and remains blank.
- Collision truncates full stopping; full D_brake and M0 remain blank.
- Fusion clearance and CARLA collision truth use different measurement sources; identity/geometry uncertainty is preserved.

## Missing data and claim ceilings

- No parsed record or Control payload archive.
- Event-local Bridge apply rows are suppressed after first APPLIED row.
- No qualified demand-origin physical deadline or WCRT/suffix bound.
- Seven disjoint baseline runs are imported only into model sensitivity; the artifact was generated post-run and lacks micro-ODD validation.
