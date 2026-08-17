# Data quality audit

## Automated consistency checks

- Expected/found runs: 12/12.
- Main-analysis runs: 11; excluded outcome-conflict run: `202607271206`.
- `T_R=(t2-t1)` maximum absolute residual: `0 ms`.
- `D2=D1-D_response` maximum absolute residual: `7.1054273576e-15 m`.
- Required Localization/Perception/SCB source paths missing: `0`.
- Observed/model separation: `run_level_observed.csv` contains no model/predicted columns; model results are stored separately.
- Collision endpoint rule: 2/2 collision runs keep full observed `D_brake`, full-stop margins and data-derived deadline unavailable.
- Main response distance uses wall-clock speed trapezoidal integration for every run.
- Record association: 0/12 same-run parsed record exports; record-only metrics remain unavailable.

## Known evidence limits

1. `202607271206` has a reproducible timing/distance chain but conflicting outcome evidence; it is retained for diagnostics and excluded from outcome aggregates.
2. Collision runs are trajectory-truncated; any full stopping distance, deadline, margin or restored outcome is model-only.
3. Actor history is archived only for two collision runs. Most noncollision runs lack dual-clock CARLA history; this does not alter wall-clock main metrics but limits realtime-factor and actor-truth claims.
4. Full Control payload is not archived. Control Trace establishes timing, while the deployment description establishes that Bridge reads Control directly. Guardian is not used as the executed command source.
5. The baseline braking model uses the same seven baseline full-stop runs for descriptive calibration and comparison; it is not cross-validated and is explicitly not an observed result.
6. The 6 m engineering safety boundary is distinct from the 0 m contact boundary. Both are reported; neither is selected after seeing collision labels.

## Distance semantics

- `D_delay_wall_integral_data_observed_m` = total response-stage distance from `t1` to `t2`.
- `D_distance_debt_*` = incremental distance after an independently derived deadline and before `t2`.
- Localization displacement/path and CARLA sim quantities are diagnostics only and do not replace the wall-clock main field.
