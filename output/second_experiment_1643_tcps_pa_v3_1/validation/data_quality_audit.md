# Data quality audit

## Scope and immutability

- Scope: only `/Users/huangjinhui/Desktop/萨卡班/data/第二次实验/300ms/202607271643`.
- Raw-file count: 35; SHA-256 recorded for every file.
- No file was written beneath the raw run directory.

## Clocks and endpoints

- Clock alignment: `ALIGNED`; p95 absolute residual 0.720203 ms; median 0.258327 ms.
- t1: source timestamp of the first frame in the first qualifying 3-frame stable target-6 sequence.
- t2: first raw-Localization sample satisfying the declared sustained-deceleration detector.
- t2 sensitivity: raw thresholds 0.3/0.5/1.0 m/s² agree; median-3 shifts the endpoint by one 100 ms sample.
- Collision endpoint: direct CARLA collision event, never actor-history first row.

## Distance and physical data

- Canonical D_response: wall-clock speed trapezoid, 15.450549919780 m.
- Collision truncates full stopping; full D_brake and M0 remain blank.
- Fusion clearance and CARLA collision truth use different measurement sources; identity/geometry uncertainty is preserved.

## Missing data and claim ceilings

- No parsed record or Control payload archive.
- Event-local Bridge apply rows are suppressed after first APPLIED row.
- No qualified prospective physical deadline or WCRT/suffix bound.
- No cross-run value enters data/observed. Seven baseline runs enter only separate UNVALIDATED_MODEL deadline diagnostics; calibration/evaluation IDs are disjoint.

## Baseline model validation boundary

- 1643 response-window observed positive-acceleration peak: 4.505839 m/s².
- Baseline conservative candidate: 2.668280 m/s².
- The evaluation observation exceeds the candidate, so it is not a valid response-acceleration upper envelope.
- Collision right-censoring prevents a complete evaluation-run braking-envelope validation.
