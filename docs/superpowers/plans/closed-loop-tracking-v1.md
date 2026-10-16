# Closed-loop Tracking v1

Reuse DLS, SafetyPipeline, the permanent world builder and the existing position
PIDs. Do not change robot geometry. Compute Only remains the default.

1. Add named, stamped safe velocities and diagnostic messages to interfaces.
2. Test and implement optional safe publication in Control, including invalidation
   and clock-epoch identity. Expose existing reference/FK diagnostics.
3. Test and implement a bounded measured-initialized position reference adapter
   and independent actuator watchdog in Simulation. Gate command bridges.
4. Compose the existing packages in Bringup, derive TF from deployment spawn,
   and add a reproducible bounded moving target configuration.
5. Build, run functional regressions, verify physical actuator response, compare
   measured-FK tracking with hold-only, and exercise failures and recovery.
6. Review and document results, commands, parameters and remaining limitations.

No commit or push. User authorized direct workspace implementation.
