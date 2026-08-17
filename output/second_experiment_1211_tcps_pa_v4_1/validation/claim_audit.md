# TCPS-PA v2 claim audit

## CLAIM: P_CLOCK.1211

**PROPOSITION:** Cross-host timestamps are qualified for event-local subtraction.

**PREREQUISITES:** None

**SUPPORT:** EV.CLOCK.1211 (MISSING)

**CHALLENGES:** EV.CLOCK.1211

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** No dual-clock history/event-local apply mapping.

**ALLOWED LANGUAGE:** Same-host trace ordering is usable; cross-host suffix timing is unqualified.

**FORBIDDEN LANGUAGE:** Cross-host timestamps are exact.

## CLAIM: P_PHASE.1211

**PROPOSITION:** Periodic phase sensitivity is tested and isolated.

**PREREQUISITES:** None

**SUPPORT:** EV.PHASE.1211 (MISSING)

**CHALLENGES:** EV.PHASE.1211

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** No phase scan or matched repeats.

**ALLOWED LANGUAGE:** Phase remains an unresolved alternative.

**FORBIDDEN LANGUAGE:** Phase is the root cause.

## CLAIM: P_OBSERVABILITY.1211

**PROPOSITION:** Demand observability and the first causal sample are independently qualified.

**PREREQUISITES:** P_CLOCK.1211

**SUPPORT:** EV.TIMESEM.1211 (OBSERVED_DERIVED), EV.CHAIN.1211 (TRACE_LINEAGE)

**CHALLENGES:** EV.OBSERVABILITY.1211

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** t_sample exists; t_demand/t_observable do not.

**ALLOWED LANGUAGE:** Only sample-relative timing is qualified.

**FORBIDDEN LANGUAGE:** t_sample equals t_demand.

## CLAIM: P_TARGET.1211

**PROPOSITION:** The relevant Apollo target identity is continuous and physically anchored.

**PREREQUISITES:** None

**SUPPORT:** EV.TARGET.1211 (TRACE_LINEAGE)

**CHALLENGES:** EV.TARGET.1211

**DEFEATERS:** D_TARGET.P_TARGET.1211=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL

**CONFIDENCE / CEILING:** MEDIUM / MEDIUM

**WHY NOT STRONGER:** Apollo target 12 is continuous; CARLA actor truth is absent.

**ALLOWED LANGUAGE:** Target 12 is lineage-consistent inside Apollo only.

**FORBIDDEN LANGUAGE:** Target 12 is proven identical to a CARLA actor.

## CLAIM: P_FUNC.1211

**PROPOSITION:** Relevant functional behavior is fully qualified and does not independently explain the low margin.

**PREREQUISITES:** P_TARGET.1211

**SUPPORT:** EV.FUNC.1211 (OBSERVED_DERIVED), EV.CHAIN.1211 (TRACE_LINEAGE)

**CHALLENGES:** EV.FUNC.1211, EV.AGEGAP.1211

**DEFEATERS:** D_FUNC.P_FUNC.1211=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL

**CONFIDENCE / CEILING:** MEDIUM / MEDIUM

**WHY NOT STRONGER:** Speed optimization fallback and payload continuity gaps remain.

**ALLOWED LANGUAGE:** STOP exists, but functional correctness is partial.

**FORBIDDEN LANGUAGE:** Functionally correct, temporally wrong.

## CLAIM: P_DEADLINE.1211

**PROPOSITION:** An independently qualified prospective dynamic physical deadline is available.

**PREREQUISITES:** P_TARGET.1211

**SUPPORT:** EV.NODEADLINE.1211 (MISSING)

**CHALLENGES:** EV.NODEADLINE.1211, EV.TIMESEM.1211, EV.MODELDEADLINE.1211

**DEFEATERS:** D_DEADLINE.P_DEADLINE.1211=OPEN: INVALIDATES

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** No t_demand and model parameters were not prospectively locked/validated.

**ALLOWED LANGUAGE:** Primary deadline is unavailable; sample model sensitivity is separate.

**FORBIDDEN LANGUAGE:** The run violated a qualified physical deadline.

## CLAIM: C1.1211

**PROPOSITION:** A configured 300 ms Bridge fixed-delay stressor entered the command path.

**PREREQUISITES:** None

**SUPPORT:** EV.FAULT.1211 (DIRECT_OBSERVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** Only one APPLIED command is individually archived.

**ALLOWED LANGUAGE:** The external Bridge stressor entered the deployed path.

**FORBIDDEN LANGUAGE:** Apollo has an intrinsic real-time defect.

## CLAIM: C2.1211

**PROPOSITION:** The configured Bridge delay manifested as a measured local delay event.

**PREREQUISITES:** C1.1211

**SUPPORT:** EV.FAULT.1211 (DIRECT_OBSERVED)

**CHALLENGES:** None

**DEFEATERS:** None

**VERDICT:** PASS

**CONFIDENCE / CEILING:** HIGH / HIGH

**WHY NOT STRONGER:** A/G have no architectural threshold; research anomalies are separate.

**ALLOWED LANGUAGE:** Local Bridge delay fidelity is established.

**FORBIDDEN LANGUAGE:** All observed gaps are requirement violations.

## CLAIM: C3.1211

**PROPOSITION:** The selected target propagates through Control and is temporally associated with physical response.

**PREREQUISITES:** C1.1211, C2.1211, P_CLOCK.1211, P_TARGET.1211

**SUPPORT:** EV.CHAIN.1211 (TRACE_LINEAGE), EV.REACTION.1211 (OBSERVED_DERIVED), EV.LOCALWAIT.1211 (TRACE_LINEAGE), EV.TARGET.1211 (TRACE_LINEAGE), EV.COHERENCE.1211 (TRACE_LINEAGE)

**CHALLENGES:** EV.CLOCK.1211

**DEFEATERS:** D_PAYLOAD.C3.1211=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL_PASS

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** Grade A software prefix; cross-host/event-local suffix remains Grade C/unqualified.

**ALLOWED LANGUAGE:** Software propagation and sample-relative physical association are supported.

**FORBIDDEN LANGUAGE:** The exact Control command caused t_phys.

## CLAIM: C4.1211

**PROPOSITION:** Observed physical response exceeded a qualified dynamic temporal contract.

**PREREQUISITES:** C3.1211, P_DEADLINE.1211

**SUPPORT:** EV.REACTION.1211 (OBSERVED_DERIVED), EV.NODEADLINE.1211 (MISSING), EV.MODELDEADLINE.1211 (UNVALIDATED_MODEL)

**CHALLENGES:** EV.NODEADLINE.1211, EV.TIMESEM.1211, EV.OBSERVABILITY.1211, EV.MODELDEADLINE.1211

**DEFEATERS:** D_DEADLINE.C4.1211=OPEN: INVALIDATES

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** Only sample-relative model crossings exist.

**ALLOWED LANGUAGE:** All declared sample-origin model scenarios miss, but primary C4 is not testable.

**FORBIDDEN LANGUAGE:** The 700.167 ms response is a qualified deadline violation.

## CLAIM: C5.1211

**PROPOSITION:** A requirement-constrained deadline-excess distance debt is established.

**PREREQUISITES:** C4.1211

**SUPPORT:** EV.DRESPONSE.1211 (OBSERVED_DERIVED)

**CHALLENGES:** EV.NODEADLINE.1211, EV.TIMESEM.1211

**DEFEATERS:** None

**VERDICT:** NOT_TESTABLE

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** Observed response distance exists; primary deadline debt does not.

**ALLOWED LANGUAGE:** 12.0 m is response distance, not primary debt.

**FORBIDDEN LANGUAGE:** 12.0 m is timing-caused safety loss.

## CLAIM: C6.1211

**PROPOSITION:** The run reached a low-clearance near-stop physical outcome.

**PREREQUISITES:** None

**SUPPORT:** EV.OUTCOME.1211 (OBSERVED_DERIVED)

**CHALLENGES:** EV.OUTCOME.1211, EV.TARGET.1211

**DEFEATERS:** D_OUTCOME.C6.1211=OPEN: CAPS_AT_PARTIAL

**VERDICT:** PARTIAL_PASS

**CONFIDENCE / CEILING:** MEDIUM / MEDIUM

**WHY NOT STRONGER:** Near-zero speed and projected clearance are observed-derived; direct collision/noncollision sensor truth is absent.

**ALLOWED LANGUAGE:** A near-stop with about 1 m projected 0 m margin is supported.

**FORBIDDEN LANGUAGE:** A direct safe-stop outcome is proven by absence of a file.

## CLAIM: C7.1211

**PROPOSITION:** Observed timing anomalies are established as the unique cause and quantified loss of the low-margin outcome.

**PREREQUISITES:** C4.1211, C5.1211, C6.1211, P_FUNC.1211

**SUPPORT:** EV.FAULT.1211 (DIRECT_OBSERVED), EV.CHAIN.1211 (TRACE_LINEAGE), EV.OUTCOME.1211 (OBSERVED_DERIVED)

**CHALLENGES:** EV.CLOCK.1211, EV.PHASE.1211, EV.FUNC.1211, EV.AGEGAP.1211, EV.NODEADLINE.1211, EV.MODELDEADLINE.1211, EV.COHERENCE.1211

**DEFEATERS:** D_INITIAL_CLEARANCE.C7.1211=OPEN: CAPS_AT_PARTIAL; D_INITIAL_SPEED.C7.1211=OPEN: CAPS_AT_PARTIAL; D_BRAKING_CAPABILITY.C7.1211=OPEN: CAPS_AT_PARTIAL; D_FUNCTIONAL_FAILURE.C7.1211=OPEN: CAPS_AT_PARTIAL; D_TARGET_MISMATCH.C7.1211=OPEN: CAPS_AT_PARTIAL; D_DATA_FRESHNESS.C7.1211=OPEN: CAPS_AT_PARTIAL; D_UPDATE_GAP.C7.1211=OPEN: CAPS_AT_PARTIAL; D_SOLVER_FALLBACK.C7.1211=OPEN: CAPS_AT_PARTIAL; D_CLOCK.C7.1211=OPEN: CAPS_AT_PARTIAL; D_PHASE.C7.1211=OPEN: CAPS_AT_PARTIAL; D_PREHAZARD_STATE.C7.1211=OPEN: CAPS_AT_PARTIAL; D_GEOMETRY.C7.1211=OPEN: CAPS_AT_PARTIAL; D_OUTCOME_CONFLICT.C7.1211=OPEN: CAPS_AT_PARTIAL

**VERDICT:** UNCERTAIN

**CONFIDENCE / CEILING:** NONE / NONE

**WHY NOT STRONGER:** Deadline, cross-host lineage, functionality, phase, geometry and direct outcome truth remain open.

**ALLOWED LANGUAGE:** Timing/functional/physical candidates coexist; unique causation is unresolved.

**FORBIDDEN LANGUAGE:** Timing uniquely caused the low margin.
