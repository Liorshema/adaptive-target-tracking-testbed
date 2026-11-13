# Predictor comparison setup

Configured the default Dashboard catalog at
`/home/user/.local/share/adaptive_tracking/catalog.yaml`.

**Verdict: PASS for configuration, preservation and Dashboard setup.** No execution
acceptance is claimed because the user requested no Runs.

## Saved structure

One active Session, **Predictors Comparison** (`db81ce48-56b7-45d2-b39d-06e95277f993`),
contains these explicit immutable Experiment references:

| Experiment | Catalog ID | Revision | Predictor |
|---|---|---:|---|
| IMM | EXP-PREDICTORS-IMM | 1 | imm |
| Gaussian Process | EXP-PREDICTORS-GAUSSIAN-PROCESS | 1 | gaussian_process |
| Koopman EDMD | EXP-PREDICTORS-KOOPMAN-EDMD | 1 | koopman_edmd |

All three passed Author/Factory/Registry validation. Their protected comparison
fingerprint is identical:
`0ac572900cfd981ec88b074bf6d876f51c7979914a208ded939b98ad91b90b5d`.
The declaration permits only `components.predictor` to vary. Resolved scenario,
robot, estimation, controller, sensors, seed consumers, runtime and the complete
prediction owner document match; only Predictor selection/type and its provenance
vary. Adaptation and fusion are `none` for all three.

## Common experimental conditions

- **3D Attractor Dynamics**: default-library `DYN-ATTRACTOR`, revision 1, imported
  through DynamicsLibrary from the existing saved definition at revision 8 in
  `results/dashboard_refinement/workspace/dynamics_library.yaml`.
- Source content SHA256: `86540063153330056c99211a3aaec426ac9e0ec907b81486383d8c80639357dd`.
- Canonical model SHA256: `1203214fbc94c5abdd6ceddd310879750faaa94ed8c5f9e52b08659ffb9483f5`.
- Model parameters: m=1 kg, k=1 kg/s², c=0.25 kg/s,
  g=[0,0,1] m, omega=[0,0,0.4] 1/s. Declared state initial positions [1,1,1] m
  and velocities [0.2,0.2,0.2] m/s are pinned identically.
- Robot Response **OFF**, explicitly frozen in each Experiment.
- Existing `environment_none` resource: no external acceleration fields or
  stochastic disturbances. The default catalog's inherited restoring field is
  incompatible with this mapped equation model; no-field selection is explicit.
- Same mobile manipulator, robot EKF, target Kalman filter and weighted-least-squares
  Distance + Pointing controller, robot description, initial conditions and simulation
  integration settings. No physics or algorithm changes.
- Seed **42**, evaluation duration **30 simulation seconds**, settling **3 seconds**.
  All retain the existing readiness-relative evaluation-window policy, max_skew
  and deadlines. Future absolute clock origins are established at Runner readiness.
- Target dynamics have no active robot-dependent influence, environmental field
  or stochastic disturbance. The autonomous model, initial state and fixed-step
  integration configuration match, preparing the same target trajectory at matched
  simulation times across Predictor choices.

**No Runs or Jobs were started.** Runtime trajectories and realized evaluation
origins have therefore not been measured. Existing runtime eligibility gates and
scientific comparison checks remain authoritative when these Experiments are run.
Dashboard physical actuation remains an explicit opt-in.

## Minimal Session extension

Schema-1 Sessions now support an optional `experiments` list of
`experiment_id`, `revision`, `sha256` references. Existing Sessions without this
field remain valid. `add_experiment_to_session` requires an explicit positive
revision and current compare-and-swap token, uses existing locks and deduplicates
retries. `list_session_experiments` resolves saved names through the Catalog.
The Dashboard shows these names beside the selected Session. Run references and
result storage retain their existing ownership. Explicit Scenario purge prunes
matching pins in its existing metadata transaction; soft archive preserves pins.

## Cleanup and preservation

Seven old default-catalog Experiments were **soft archived**. All their original
revision records remain unchanged and readable. The default workspace originally
had zero Sessions; it now has exactly one. Other development catalogs and Sessions
remain untouched. The pre-change default metadata is backed up under
`results/predictors_comparison_setup/default_before/`.

**11,459 historical scientific data/configuration files**
were checked by SHA256; **zero changed or disappeared**. No Run/Result purge API was
used. Evidence manifest and audit:
`results/predictors_comparison_setup/history_before.json` and `verification.json`.

## Verification

- **164 tests passed** across backend, Session management, transactional deletion
  and Dashboard execution. This includes explicit revisions, legacy Sessions,
  duplicate pins, stale tokens, malformed/forged pins, archiving and purge consistency.
- `colcon build --packages-select robot_experiments robot_dashboard`: PASS.
  Installed backend/deletion/Dashboard sources match current source.
- Focused lint comparison found **zero findings on added lines**. There are 42
  existing file-level style findings; they were not broadly reformatted.
- Fresh read-only review found no critical or important issues. Minor test coverage
  suggestions deferred: selected legacy Session with no pins in the view helper,
  and explicit assertion that a pin remains revision 1 after a later save.
- Installed Chromium: PASS. Exactly one Session and three Experiment options; all
  three loaded Predictor labels, duration 30, seed 42, Attractor model and Robot
  Response OFF verified. No page errors. At 1920×1080, page scroll dimensions
  equal the viewport. Report: `results/predictors_comparison_setup/browser.json`.
  Screenshots: `catalog_options.png`, `imm.png`, `gaussian_process.png`,
  `koopman_edmd.png` in the same evidence directory. The initial browser selector
  attempt is preserved separately; it needed scoping to the model selector because
  the GP card also contains kernel/normalization selectors.

## Dashboard launch

Use the existing Dashboard environment, which includes Dash:

```bash
cd /home/user/ros2_ws
source install/setup.bash
build/dashboard_venv/bin/python -c 'from robot_dashboard.cli import main; main(["--port", "8060", "--data-dir", "/home/user/.local/share/adaptive_tracking"])'
```

Open `http://localhost:8060/`. Select **Predictors Comparison**; the header lists
its three members and the Experiment selector contains exactly the three names.
No Run is required to inspect or load these saved configurations.

## Changed files for this task

- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/robot_experiments/deletion.py`
- `src/robot_experiments/test/test_management.py`
- `src/robot_experiments/test/test_deletion.py`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/test/test_execution.py`
- `docs/dashboard.md`
- `docs/validation/predictors_comparison_setup.md`

Active metadata: default `catalog.yaml`, `dynamics_library.yaml` and the single
Session JSON under `/home/user/.local/share/adaptive_tracking/`. Generated backups,
rehearsal, configuration scripts and validation evidence stay under ignored
`results/predictors_comparison_setup/`. The unsuccessful rehearsal with the inherited
field is retained. Prior GP/EDMD/MPC edits are preserved. No commit or push.
