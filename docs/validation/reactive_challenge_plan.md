# Reactive 3D challenge implementation plan

Existing branch feature/m09-visualization at bf71fdb; preservation hashes in
reactive_challenge_baseline.json include all pre-existing M02/M09 files.

Reuse SimulationFactory, CompositeAccelerationField, saturated FightFlightDynamics,
RK4/TimeStepper, authoritative FK, M07 manifest selection and M08 recording/lifecycle.
No alternative controller, dynamics engine, estimator or actuator.

1. Baseline: preserve hashes/status; full functional tests/build; bounded nominal
   live run using existing M08 lifecycle and full M02 physics trace.
2. Pure math, test first: generic cubic phase schedule (durations, transition at
   start of each subsequent phase; clamp before/after, reset follows source time).
   Contextual base guidance a=-kr(rho-rhod)er-kh(h-hd)ez+aphi(ez cross er).
   A typed physical-base sample validates provenance/frame/age and causality.
   Ordinary position/time fields retain their interface; composite dispatches
   contextual fields explicitly. No added damping or total acceleration cap.
3. Authoritative FK feasibility sweep, nominal/15/30 wrist, camera +X/R unchanged,
   to select candidate low/high/ring parameters; no collision feasibility claim.
4. Simulation owns contextual sample buffer and field factory, source timestamps
   and physical feedback integration. Challenge alone selects physical odometry
   for camera TF; nominal remains wheel-odometry TF. No future samples used at
   integration step start; zero-order hold over bounded RK4 integration interval.
5. YAML A-E: inherited system/scenario/strategy settings, complete description
   variants through existing M07 references; verify every physical consumer and
   initial posture. Flatten authoritative description if inheritance cannot be
   consumed consistently; never duplicate a limit table in another package.
6. M08 study inputs/telemetry plus full M02 physical trace; per-phase gap-aware
   joint metrics, actual base-relative geometry/heading, FK reach, tracking,
   safety and reproducible plots. A/B then C then D/E; primary seeds 42/917.
7. Full functional/architecture tests, ten-package builds, copied installation,
   compute-only zero-command check, independent frame/config/metrics review.

Owners: simulation fields/scheduling/physical context; description wrist variants;
control experimental cost YAML only; bringup selection/composition;
experiments phase metrics/configuration and reporting. No nominal manifest edits.

Risks: overhead pose may be infeasible at R=1; guidance equilibrium differs from
schedule under Fight/Flight; rear-relative orbit can fail if base follows heading;
reactive trajectories differ by controller even for equal seeds; soft physics
motors are not a universal speed guarantee. Missing/gapped telemetry is not zero
motion and cannot certify safety. Failures remain explicit CONDITIONAL/FAIL.
User has authorized implementation without additional permission gates, and no
staging/commit/push. Execute inline with independent feasibility/allocation audits.
