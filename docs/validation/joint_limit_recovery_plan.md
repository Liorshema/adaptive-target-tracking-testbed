# Joint-limit recovery causal investigation (2026-10-21)

Baseline SHA/status/hashes: joint_limit_recovery_baseline.json; current workspace
has 1,564 pre-existing nonignored files and no running Gazebo/actuation processes.
Preserve all M02/M09/challenge work, limits, PID/governor defaults and raw results.
No production correction until a real-physics failing reproducer discriminates causes.

| Hypothesis | Required observation | Falsifier | Smallest discriminating experiment |
|---|---|---|---|
| H1: hard-stop constraint interaction | sustained inward native demand at hard stop with no physical motion; change only constraint condition | same load/target resumes with unchanged constraints but repaired transport | direct native position targets lower/upper then interior, full-step trace |
| H2: insufficient effort/backend behavior | actual backend identified; loaded stall correlated with effort cap/backend law | unladen wrist stalls equally with ample configured effort and explicit inward demand | existing robot shoulder under gravity vs wrist, unchanged limits/effort; inspect actual backend code |
| H3: governor/target pathology | accepted native reference/demand prevents inward acceleration | native internal target and inward demand persist over a contiguous window | trace pre/post q,v, target, previous/current demand at physics rate |
| H4: recovery-authority deadlock | missing/revoked validated heartbeat or inward ref before crossing release margin | validated fresh heartbeat/reference persists through physical stall | drive same hard stop via real SafeVelocityCommand + adapter, capture guard/authority/output |
| H5: compounded mechanisms | direct physical stall and independent state-machine inability to release/recover | resolving one proven cause restores every causal link | compare direct native experiment with normal adapter/control path |

1. Analyze historical C/D/E windows as durations/contiguous segments; inspect actual
   compiled plugin identity/backend/SDF and authority code independently.
2. Build bounded isolated native-motor real-physics probe using existing M08
   lifecycle/robot builder/traces, generic named joint configuration and valid
   command ownership. Cover both hard stops, loaded shoulder, nominal/15/30 wrist.
3. Archive pre-fix source/plugin/config identities; obtain RED physical evidence.
4. Implement only demonstrated generic corrections with red/green regressions.
   Never reset physical q/v, enlarge bounds, weaken tolerances or add writers.
5. Verify release/ordinary base+arm motion, authority/replay/timeouts/serious faults,
   two independent starts, full-step safety, reset and zero-command compute-only.
6. Unchanged C/D/E sufficiently long runs; compare contiguous stalled/recovery
   durations independently from tracking accuracy. Build normal/copied installs,
   regression, exact task inventory and root-cause report. Missing evidence stays
   CONDITIONAL; a repeated reproduced deadlock or safety regression is FAIL.
