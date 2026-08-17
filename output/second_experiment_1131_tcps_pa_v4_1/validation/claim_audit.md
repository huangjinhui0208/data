# TCPS-PA v2 claim audit

## CLAIM: P_CLOCK.1131

**PROPOSITION:** The event timing domains are sufficiently aligned for this event-chain ordering and millisecond intervals.

**PREREQUISITES:** None

**SUPPORT:** EV.CLOCK.1131 (OBSERVED_DERIVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** p95 alignment residual is a bound, not zero error.

**ALLOWED LANGUAGE:** Clock alignment supports event ordering and interval measurement.

**FORBIDDEN LANGUAGE:** All timestamps are exact and error-free.

## CLAIM: P_PHASE.1131

**PROPOSITION:** A phase effect has been actively tested and isolated.

**PREREQUISITES:** None

**SUPPORT:** None

**CHALLENGES:** EV.PHASE.1131

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** No active phase scan or matched repeats.

**ALLOWED LANGUAGE:** Tick/phase is an unresolved alternative.

**FORBIDDEN LANGUAGE:** Phase is the root cause.

## CLAIM: P_OBSERVABILITY.1131

**PROPOSITION:** The demand observability boundary and first causal sample are independently qualified.

**PREREQUISITES:** P_CLOCK.1131

**SUPPORT:** EV.TIMESEM.1131 (OBSERVED_DERIVED), EV.CHAIN.1131 (TRACE_LINEAGE)

**CHALLENGES:** EV.OBSERVABILITY.1131

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** LOW / LOW

**WHY NOT STRONGER:** t_sample is observed, but the observer/FOV/occlusion model and t_observable are absent.

**ALLOWED LANGUAGE:** Only sample-relative chain timing is qualified.

**FORBIDDEN LANGUAGE:** t_sample equals t_demand.|Demand-to-sample is software latency.

## CLAIM: P_TARGET.1131

**PROPOSITION:** Fusion/Prediction/Planning target 11 corresponds to CARLA collision actor 155.

**PREREQUISITES:** None

**SUPPORT:** EV.TARGET.1131 (TRACE_LINEAGE)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** CARLA other-actor history starts 399.215 ms after t_sample.

**ALLOWED LANGUAGE:** Target identity is strongly supported across 20 matched frames.

**FORBIDDEN LANGUAGE:** Target identity is mathematically exact at t_sample.

## CLAIM: P_FUNC.1131

**PROPOSITION:** Relevant functional behavior is fully qualified and does not independently explain the outcome.

**PREREQUISITES:** P_TARGET.1131

**SUPPORT:** EV.FUNC.1131 (OBSERVED_DERIVED), EV.CHAIN.1131 (TRACE_LINEAGE)

**CHALLENGES:** EV.FUNC.1131

**DEFEATERS:** D_FUNC.P_FUNC.1131=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL

**CONFIDENCE / CEILING:** MEDIUM / MEDIUM

**WHY NOT STRONGER:** Planning fallback and absent Control/Bridge payload continuity keep functionality unresolved.

**ALLOWED LANGUAGE:** The STOP chain exists, but functional correctness remains partial.

**FORBIDDEN LANGUAGE:** functionally correct, temporally wrong|功能正确但时间错误

## CLAIM: P_DEADLINE.1131

**PROPOSITION:** An independently qualified prospective dynamic physical deadline is available.

**PREREQUISITES:** P_TARGET.1131

**SUPPORT:** EV.NODEADLINE.1131 (MISSING)

**CHALLENGES:** EV.NODEADLINE.1131

**DEFEATERS:** D_DEADLINE.P_DEADLINE.1131=OPEN: INVALIDATES

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** LOW / LOW

**WHY NOT STRONGER:** t_demand is left-censored/unavailable; post-run calibrated parameters are not locked and their domain is unvalidated.

**ALLOWED LANGUAGE:** Primary demand-origin tau_req is unavailable; sample-origin model sensitivity is reported separately.

**FORBIDDEN LANGUAGE:** The run missed its physical safety deadline.

## CLAIM: C1.1131

**PROPOSITION:** A 300 ms Bridge fixed-delay stressor was actually applied before the selected event.

**PREREQUISITES:** None

**SUPPORT:** EV.FAULT.1131 (DIRECT_OBSERVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** Only the first applied command is logged; persistence follows deployed implementation semantics.

**ALLOWED LANGUAGE:** The external Bridge temporal stressor entered the system.

**FORBIDDEN LANGUAGE:** Apollo has an intrinsic real-time defect.

## CLAIM: C2.1131

**PROPOSITION:** The configured Bridge delay manifested as a measured local delay event.

**PREREQUISITES:** C1.1131

**SUPPORT:** EV.FAULT.1131 (DIRECT_OBSERVED), EV.FRESH.1131 (OBSERVED_DERIVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** Freshness quantities have no qualified local requirement; the Bridge delay itself has an explicit 300 ms reference.

**ALLOWED LANGUAGE:** A measured local Bridge delay manifestation is established.

**FORBIDDEN LANGUAGE:** All observed age/gap values are requirement violations.

## CLAIM: C3.1131

**PROPOSITION:** The target instance propagates from source through Control and is temporally associated with physical t2.

**PREREQUISITES:** C1.1131, C2.1131, P_CLOCK.1131, P_TARGET.1131

**SUPPORT:** EV.CHAIN.1131 (TRACE_LINEAGE), EV.REACTION.1131 (OBSERVED_DERIVED), EV.FRESH.1131 (OBSERVED_DERIVED)

**CHALLENGES:** None

**DEFEATERS:** D_PAYLOAD.C3.1131=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL_PASS

**CONFIDENCE / CEILING:** MEDIUM / MEDIUM

**WHY NOT STRONGER:** Grade A lineage stops at Control output; Control-to-physical link is Grade C.

**ALLOWED LANGUAGE:** Strict software propagation and system-level physical association are supported.

**FORBIDDEN LANGUAGE:** The exact event-local command payload caused t2.

## CLAIM: C4.1131

**PROPOSITION:** Observed physical reaction time violated a qualified dynamic temporal contract.

**PREREQUISITES:** C3.1131, P_DEADLINE.1131

**SUPPORT:** EV.REACTION.1131 (OBSERVED_DERIVED), EV.NODEADLINE.1131 (MISSING), EV.MODELDEADLINE.1131 (UNVALIDATED_MODEL)

**CHALLENGES:** EV.NODEADLINE.1131, EV.MODELDEADLINE.1131

**DEFEATERS:** D_DEADLINE.C4.1131=OPEN: INVALIDATES

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** LOW / LOW

**WHY NOT STRONGER:** Only sample-relative T_R is observed; all four center model budgets miss, with one geometry/effect boundary overlap.

**ALLOWED LANGUAGE:** Primary demand-origin contract is not testable; a sample-origin model-supported miss is established across the declared sensitivity set.

**FORBIDDEN LANGUAGE:** A collision proves a deadline miss.|The 799.636 ms response is a deadline violation.

## CLAIM: C5.1131

**PROPOSITION:** A requirement-constrained deadline-excess distance debt is established.

**PREREQUISITES:** C4.1131

**SUPPORT:** EV.DRESPONSE.1131 (OBSERVED_DERIVED)

**CHALLENGES:** EV.NODEADLINE.1131

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** LOW / LOW

**WHY NOT STRONGER:** D_response is observed; primary D_debt requires qualified tau_req and is unavailable.

**ALLOWED LANGUAGE:** 13.432 m is observed response distance, not deadline-excess debt.

**FORBIDDEN LANGUAGE:** 13.432 m is timing-caused safety loss.|13.432 m is deadline debt.

## CLAIM: C6.1131

**PROPOSITION:** A direct physical collision outcome occurred with actor 155.

**PREREQUISITES:** None

**SUPPORT:** EV.COLLISION.1131 (DIRECT_OBSERVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** Collision establishes outcome but not timing attribution.

**ALLOWED LANGUAGE:** Collision and impact severity are directly observed.

**FORBIDDEN LANGUAGE:** The collision proves a temporal contract miss.

## CLAIM: C7.1131

**PROPOSITION:** The observed timing behavior is established as the cause of the collision and quantified physical loss.

**PREREQUISITES:** C4.1131, C5.1131, C6.1131, P_FUNC.1131

**SUPPORT:** EV.FAULT.1131 (DIRECT_OBSERVED), EV.CHAIN.1131 (TRACE_LINEAGE), EV.COLLISION.1131 (DIRECT_OBSERVED)

**CHALLENGES:** EV.FUNC.1131, EV.FRESH.1131, EV.POSTGAP.1131, EV.NODEADLINE.1131, EV.PHASE.1131

**DEFEATERS:** D_INITIAL_CLEARANCE.C7.1131=OPEN: CAPS_AT_PARTIAL; D_INITIAL_SPEED.C7.1131=OPEN: CAPS_AT_PARTIAL; D_BRAKING_CAPABILITY.C7.1131=OPEN: CAPS_AT_PARTIAL; D_FUNCTIONAL_FAILURE.C7.1131=OPEN: CAPS_AT_PARTIAL; D_TARGET_MISMATCH.C7.1131=RESOLVED: BOUNDED; D_DATA_FRESHNESS.C7.1131=OPEN: CAPS_AT_PARTIAL; D_UPDATE_GAP.C7.1131=OPEN: CAPS_AT_PARTIAL; D_SOLVER_FALLBACK.C7.1131=OPEN: CAPS_AT_PARTIAL; D_CLOCK.C7.1131=BOUNDED: BOUNDED; D_PHASE.C7.1131=OPEN: CAPS_AT_PARTIAL; D_PREHAZARD_STATE.C7.1131=OPEN: CAPS_AT_PARTIAL; D_GEOMETRY.C7.1131=OPEN: CAPS_AT_PARTIAL; D_OUTCOME_CONFLICT.C7.1131=RESOLVED: BOUNDED

**VERDICT:** UNCERTAIN

**CONFIDENCE / CEILING:** LOW / LOW

**WHY NOT STRONGER:** Unqualified deadline, partial functionality, event-local Bridge payload gap, phase, braking, and geometry alternatives remain.

**ALLOWED LANGUAGE:** An injected temporal stressor, delayed physical response, later freshness degradation, and collision co-occur in one chain; causal share is unresolved.

**FORBIDDEN LANGUAGE:** The delay caused the collision.|Timing was the sole cause.|The injected delay caused 13.432 m of safety loss.
