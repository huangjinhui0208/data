# Legacy report v1 -> TCPS-PA v2 claim reassessment

## Six-Layer Inference Status Matrix

| Layer | v2 verdict | Evidence ceiling | Main reason |
|---|---|---|---|
| L1 / C1 | PASS | HIGH | Bridge/SCB disturbance is directly observed. |
| L2 / C2 | PARTIAL_PASS | MEDIUM | T_R is observed; gap reference/distribution and phase scan are incomplete. |
| L3 / C3 | PARTIAL_PASS | MEDIUM | Causal lineage grade C and clocks are partial. |
| L4 / C4 | NOT_TESTABLE | LOW | No independently qualified deadline. |
| L5 / C5 | MODEL_SUPPORTED_ONLY | LOW | D_response is observed; deadline-excess debt is model-tainted. |
| L6 / C6 | PASS | MEDIUM | Physical outcomes are directly observed; attribution remains separate. |
| Attribution / C7 | PARTIAL_PASS (Level 2 max) | LOW | Weak C4/C5 and open functional/pre-hazard defeaters cap attribution. |

## Claim-by-claim decisions

### 1. ‘300 ms在闭环物理响应端被放大’

**REFRAMED** — Only one non-zero intervention level exists. Use an observed incremental response ratio, not amplification/gain.

### 2. ‘300 ms → 449.543 ms’

**UNCHANGED** — The descriptive observed delta may remain if endpoint and uncertainty are stated; it is not a stable transfer coefficient.

### 3. L2 Fusion degradation

**REFRAMED** — A Fusion maximum is case-level characterization. No reference distribution/requirement supports group degradation.

### 4. L3 Cause-Effect Timing

**DOWNGRADED** — Lineage is grade C temporal alignment, not explicit trace/provenance lineage.

### 5. tau_data_derived evidence qualification

**DOWNGRADED** — Reclassified as tau_retro: useful for reconstruction, ineligible as primary independent deadline.

### 6. 1202/1211 model deadline miss

**MODEL_ONLY** — The deadline and miss are unvalidated-model outputs, not observed temporal failures.

### 7. 1131 Distance Debt 5.018 m

**MODEL_ONLY** — Debt inherits the model deadline taint; D_response remains separately observed.

### 8. 1643 Distance Debt 10.309 m

**MODEL_ONLY** — Debt inherits the model deadline taint; it cannot enter the observed primary chain.

### 9. 1131 timing-dominated candidate

**DOWNGRADED** — At most a candidate: C4 is not testable, P_FUNC is partial, and critical defeaters remain open.

### 10. 1643 multi-factor

**REFRAMED** — Multi-factor is the conservative class, but individual contributions are not quantified.

### 11. ‘functionally correct, temporally wrong’

**NOT_TESTABLE** — P_FUNC is not QUALIFIED_PASS and C4 is not established.

### 12. Current six-layer overall claim

**DOWNGRADED** — L1 PASS, L2 PARTIAL, L3 PARTIAL, L4 NOT_TESTABLE, L5 MODEL_SUPPORTED_ONLY, L6 PASS. Strong ends do not close weak middle claims.

## Diagnostic conclusion

The experiment directly establishes the injected disturbance and physical outcomes. It supports temporal association and an unvalidated-model propagation mechanism, but it does not establish a primary temporal deadline failure or qualified observed Distance Debt. The most important missing evidence is an independent scenario deadline and requirement-constrained L5 debt, followed by end-to-end lineage, cross-host clock qualification, functional-chain qualification, and pre-hazard state history.
