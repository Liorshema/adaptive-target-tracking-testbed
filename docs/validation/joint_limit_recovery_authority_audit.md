# Joint-limit recovery authority audit

Read-only diagnosis, 2026-10-21. This audit changes no production code or tests and launches no simulation. Its scope is the ROS position guard, command authority, recovery heartbeat and release transitions. It does not establish the physical solver root cause or substitute for the required real-physics red/green experiment.

## Finding

The inspected terminal C/D/E seed42 freezes sustain a legal inward native target and nonzero inward velocity demand throughout approximately 18–19 seconds of essentially zero physical velocity. The guard consequently cannot cross its measured release margin. Fresh safe commands continue, and recovery reference slewing reaches the configured full 0.06 rad inset. The available counters show no new watchdog, command rejection or feedback rejection during these terminal recoveries. A recovery-authority starvation mechanism is therefore not supported as the cause of these recorded freezes.

The frozen `accepted_stamp` is expected historical lineage: it identifies the last command accepted for ordinary active integration. Commands validated during recovery refresh a separate private heartbeat. They intentionally do not update accepted identity. Existing telemetry does not directly expose that heartbeat, so its exact local wall age cannot be reconstructed from saved telemetry alone.

An additional in-memory callback experiment supplied 50,000 fresh active commands over 1,000 simulation seconds while holding feedback infinitesimally below a configured lower limit. Recovery remained authorized, the legal reference reached its inward inset, and accepted identity remained unchanged. Supplying an interior measurement beyond the release margin immediately restored `normal` and accepted ordinary command authority. This demonstrates orchestration behavior under controlled inputs, not physical motion.

## Source authority and transitions

Paths below are relative to `/home/user/ros2_ws`.

| Transition or invariant | Exact source |
|---|---|
| Numerical excursions are distinguished from recoverable excursions; excursions above the recoverable threshold latch a serious fault. | `src/robot_simulation/robot_simulation/actuation/position_guard.py:41-73` |
| Existing recovery side clears only when measured position reaches lower + release margin or upper − release margin. This happens before deciding whether a new excursion exists and does not depend on command acceptance. | `position_guard.py:52-67` |
| A fresh inactive/recovery heartbeat can authorize a narrowly bounded inward release even when measured feedback rounds to the legal side of a hard stop. | `position_guard.py:56-62`; `src/robot_simulation/robot_simulation/nodes/velocity_actuation.py:322-328` |
| Recovery reference is legal, slews at configured bounded speed, affects only recovering joints and preserves an already safer inward reference. | `position_guard.py:75-87` |
| Every incoming recovery-authorizing command passes timestamp, identity/frame, velocity, retired-session, publisher ownership and sequence ordering checks. Active commands also require fresh joint and odometry feedback. | `velocity_actuation.py:203-256` |
| Valid inactive commands call `_halt` then restore `_recovery_heartbeat`; valid active commands in recovery do the same. | `velocity_actuation.py:237-257` |
| Only accepted ordinary active commands update `_accepted_identity` and `_accepted_commands`. | `velocity_actuation.py:258-265` |
| `_halt` clears ordinary and recovery authority, zeroes base output and preserves/publishes the current arm reference. It does not reset `_last_recovery_tick`. | `velocity_actuation.py:272-281`; `403-415` |
| Independent steady-clock watchdog checks command/recovery authority, both feedback sources and output ownership. | `velocity_actuation.py:114-118`; `143-150`; `302-312` |
| Recovery tick transfers an existing ordinary command into recovery authority; it advances the inward reference only with fresh authority, feedback, owned outputs and a bounded positive tick interval. | `velocity_actuation.py:314-348` |
| Resumed active acceptance retains reference continuity; it does not reinitialize a populated reference from the hard stop. | `velocity_actuation.py:258-262` |
| Backward clock transition retires transport state and old reference, but preserves a serious physical fault. | `velocity_actuation.py:283-300` |
| Diagnostics retain historical accepted identity through inactive heartbeats, holds and rejected commands. | `velocity_actuation.py:367-401`; `src/robot_interfaces/msg/ActuationTelemetry.msg:25-31` |

The authority chain has no cyclic requirement that ordinary control first become active in order to move inward. Recovery can advance while `active=false`, then measured inward displacement clears the guard, then the next valid active command resumes ordinary integration. With lower-stop feedback fixed at the limit, the intended hysteresis requires recovery to remain latched indefinitely.

The reference for a lower recovery side is `max(existing_reference, lower + inset)` with reference slew capped by `min(joint_velocity_limit, recovery_speed)`. Recovery release requires measured `q >= lower + release_margin`. A legal target alone does not satisfy that measured-position condition.

## Historical continuous physical windows

All paths share this run root:

`results/reactive_challenge_final/reactive_manipulation_20261019T155152_099000Z_nmxd3n7j/runs/`

The analysis streamed only the three named approximately 21 MB native CSVs and their small telemetry files. It did not read orphan results or launch Gazebo. For each wrist sample it selected a position within `1e-6` rad of the configured physical stop, velocity demand directed into the interval with magnitude `>0.05 rad/s`, and physical `|v_post| <0.001 rad/s`. It grouped adjacent selected samples with source-time gaps below `0.00201 s` (native cadence `0.002 s`), and chose the longest group.

| Variant | Continuous inward-demand stall [s] | Samples | Duration, last − first [s] | Inward demand range [rad/s] | Maximum physical speed [rad/s] |
|---|---:|---:|---:|---:|---:|
| C_seed42 | 22.874–41.610 | 9,369 | 18.736 | +0.056 to +0.600 | 1.785e-12 |
| D_seed42 | 23.568–42.062 | 9,248 | 18.494 | +0.056 to +0.600 | 1.177e-12 |
| E_seed42 | 22.588–41.748 | 9,581 | 19.160 | +0.0500000000066 to +0.600 | 7.905e-13 |

These longest windows all occur at the lower wrist stop. D's total filter count of 9,908 also includes other windows; it is not the duration of one contiguous stall. CSV `invalid_commands` stays zero and selected rows have `valid_step=1`, `epoch=0`. Effort measurements are unavailable (`effort_available=0`); these data do not measure torque or prove effort saturation.

Exact CSV evidence endpoints (header counted as line 1):

| File | First / last CSV line | Measured q range [rad] |
|---|---:|---:|
| `C_seed42/physics/actuator.csv` | 45,749 / 83,221 | −0.261799387800115 to −0.261799387799938 |
| `D_seed42/physics/actuator.csv` | 47,137 / 84,125 | −0.261799387800156 to −0.261799387800039 |
| `E_seed42/physics/actuator.csv` | 45,177 / 83,497 | −0.523598775598950 to −0.523598775598871 |

Terminal recovery telemetry corroborates actual receipt and advancement of inward references:

| Variant | Terminal recovery telemetry [s] | Full inward inset first observed [s] | Full wrist reference [rad] | Active commands / all commands in telemetry interval |
|---|---:|---:|---:|---:|
| C_seed42 | 22.900–41.500 | 23.400 | −0.2017993877991494 | 371 / 373 |
| D_seed42 | 23.610–42.000 | 24.102 | −0.2017993877991494 | 367 / 367 |
| E_seed42 | 22.604–41.702 | 23.204 | −0.4635987755982988 | 382 / 382 |

Exact `telemetry/actuation.jsonl` first / inset-reached / final lines: C 215 / 220 / 401; D 221 / 226 / 405; E 226 / 232 / 417. `telemetry/command.jsonl` supplies the safe-command observations. C's two inactive commands occur at 36.600 and 36.652 seconds and can themselves authorize bounded recovery after validation.

Across the terminal recovery intervals:

- Every sampled `feedback_valid` is true; sampled joint feedback source age is at most 0.020 s.
- `accepted_commands` is constant at C 417, D 391, E 416. Last accepted stamps are C 22.752, D 23.500, E 22.500 s.
- `rejected_commands` is constant at C 0, D 1, E 0; the D rejection predates this window.
- `timeout_events` is constant at C 52, D 0, E 0; C startup timeouts predate this window.
- `feedback_rejections` is zero throughout all three windows.
- Maximum successive recorded safe-command source gaps are C 0.066, D 0.058, E 0.060 s, below the configured 0.25 s source-age allowance.

Recorded command arrival gaps at the experiment observer reach C 0.614, D 0.401, E 0.408 wall seconds. Observer receipt timing is not actuation-node callback timing. In particular, C's gap must not be asserted to prove an actuator-local 0.5 s watchdog expiration: its actuator timeout counter does not advance. Telemetry at 10 Hz also cannot rule out every transient between observations. These limits do not explain away a full sustained inward native demand with no corresponding motion.

## Verification performed

Existing guard and adapter tests were run against source with bytecode and pytest cache writes disabled:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/user/ros2_ws/src/robot_simulation:$PYTHONPATH \
python3 -m pytest -q -p no:cacheprovider \
  src/robot_simulation/test/test_position_guard.py \
  src/robot_simulation/test/test_velocity_actuation.py
```

Result: **50 passed in 0.75 s**.

The additional callback experiment reused `test_velocity_actuation.py`'s existing `actuator.__wrapped__()` fixture without adding/editing tests. After one ordinary acceptance established historical identity, it initialized a legal lower-bound reference, repeatedly refreshed both feedback timestamps and a monotonically increasing, active, bounded command every 0.02 simulation seconds, and held measured feedback at `lower − 1e-13`. It called the production command callback and tick for 50,000 iterations. Observed output:

```json
{"duration_sim_s":1000.0,"mode":"numerical_recovery","reference":[-0.96,0.0],"accepted_identity_unchanged":true,"timeouts":0,"rejections":0,"heartbeat_fresh":true}
{"experiment":"feedback clears existing release margin","mode":"normal","normal_authority":true,"sequence":50002}
```

The fixture uses a 0.04 rad recovery inset. The release measurement was `lower + 0.006` rad, beyond the unchanged 0.005 rad margin. A preliminary exploratory assertion expected a pre-existing reference already far inside the interval to move back toward the recovery inset; it failed because the guard correctly preserves that safer reference. The completed experiment explicitly initialized the boundary reference to exercise inward slewing.

Relevant existing coverage includes hysteretic release and faults (`test_position_guard.py:45-62`), inactive legal-side boundary release (`78-91`), inactive heartbeat recovery and ordinary resumption (`test_velocity_actuation.py:208-222`), freshness/ownership revocation (`225-245`), reset-persistent faults (`248-259`), wall-clock revocation without reference reset (`262-276`), and historical accepted identity (`331-380`). These tests do not model actual Gazebo physical integration.

Audit source SHA-256 identities:

```text
7531967c8b404fd20c9e1bb860a88adcec34d365bf58313e26129dfa0e5d70ac  position_guard.py
137a36d541c5b34079593b986fed0670b295ffdb2cb25050df4008b958f107fa  nodes/velocity_actuation.py
6131ff26ea70e1dcf2b826ecab49b532633a899118b63b717d9351e3cb7d7dab  test_position_guard.py
4225925175843393caee4787d4d5abb233f231c1d01c6cd2fe372b92bcddcdd9  test_velocity_actuation.py
```

## Proposals and remaining questions

1. No behavioral change to guard release, command freshness or recovery heartbeat is justified by this evidence. First resolve why real inward native demands cannot produce physical inward motion. The main investigation owns that reproduction and any production correction.
2. A future regression can retain the long-lived active-heartbeat scenario above, with accepted lineage explicitly unchanged during recovery and ordinary authority restored after real measured release. Physical validation still needs real native actuation and both configured boundaries.
3. If further live diagnosis requires it, expose the already-existing recovery heartbeat timestamp/freshness in diagnostic telemetry. This is an observability proposal, not a demonstrated behavioral fix; current historical acceptance fields must retain their documented meaning.
4. The static load assertion in `test_position_guard.py:94-118` and the comments in `config/actuation.yaml:14-18` justify the inset through PID proportional effort. The selected backend is `velocity_servo` (`config/gazebo/joint_position_hold.yaml:5`), where those PID gains are unused (same file `26-28`; native dispatch `gazebo/native/actuator_system.cc:247-259`). That test establishes a legacy effort-PID condition, not native velocity-servo lifting ability. Make its backend scope explicit in a follow-up after the physical mechanism is established; it does not support changing the nominal inset.
5. `_halt` revokes reference integration authority while continuing to publish the held legal arm reference. Thus a watchdog timeout may pause further reference advancement, but does not undo an already established inward target. That distinction matters when interpreting a later physical freeze and is existing safety policy, not a newly introduced mechanism.

Historical traces do not establish that a future condition with stale feedback, missing odometry or callback starvation can never stop recovery. The independent watchdog deliberately does so. They establish that the inspected C/D/E terminal freezes already possess sustained inward native demand and cannot be attributed to a missing ordinary accepted command alone. The separate reported live shoulder freeze needs its own matched native/ROS evidence before assigning it the same physical cause.
