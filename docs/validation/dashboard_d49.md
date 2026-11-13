# D4.9 — Predictor None and Dashboard improvements

## Implemented behavior

1. **None**, ID `none`, is `PositionHoldPredictor`: repeat the latest estimated
   position, set every future velocity coordinate to zero, preserve the standard
   future timestamps and model identity. It uses registered owner discovery and
   has no numerical parameters. Arbitrary even position/velocity dimensions are
   supported; finite state, time, positive step and strictly advancing timestamps
   are validated. No extrapolation of the estimated velocity is performed.
   The owner declaration requires Adaptation/Fusion `none`; the Factory supplies
   their actual selections in generic requirement context. This prevents residual
   corrections from changing the meaning of None. Input freshness/frame/safety
   handling and the prediction pipeline remain in their existing owners.
2. Comparison labels omit a redundant single-variant suffix such as `· imm`.
   Actual multi-variant studies retain distinctions, including when one variant
   failed. Existing per-variant Catalog labels are preserved without double suffixes
   or case-dependent `IMM · imm` duplication. Stored Run identities are unchanged.
3. Failed execution rows are filtered only from the RMSE comparison presentation.
   `readiness_timeout` evidence remains readable from Results and diagnostics.
   Completed Runs with failed safety acceptance are not hidden. No metric calculation,
   ranking, underlying comparison data or immutable recording is changed.
4. Normal Predictor/Adaptation/Fusion choices hide undeployable/incompatible entries.
   An explicitly selected historical unsupported entry is retained as disabled and
   labelled `unavailable (saved selection)`. Full preflight remains authoritative.
   No backend registry inventory or validation rule is weakened.
5. Windows Start/Stop launchers use the existing WSL installation/default Catalog.
   Start opens localhost:8060 after readiness, serializes repeated starts using
   flock, and reuses a healthy owned instance. Stop checks interpreter invocation,
   full serve command, UID, PID birth and expected port before signalling the owned
   process group. Process metadata/logs are cache data, separate from experiments.
   An occupied unowned port produces an error; unrelated processes are never adopted.

## Windows usage

Double-click `tools/dashboard/Start Dashboard.cmd`. For convenient desktop access,
create a Windows shortcut to that file rather than copying it away from its helpers.
Double-click `Stop Dashboard.cmd` to close the owned server. The WSL repository path
is derived from the launcher's location; the distribution is taken from its WSL UNC
path. The existing workspace setup and Dashboard virtual environment are required.
No Docker is used. The default WSL user must own the intended installation/Catalog.

WSL equivalents, from `/home/user/ros2_ws`:

```bash
bash tools/dashboard/dashboard.sh start
bash tools/dashboard/dashboard.sh stop
```

The generic CLI defaults remain unchanged; these launchers select port 8060.
Dashboard cache: `~/.cache/adaptive_tracking/dashboard-8060/` (XDG-aware), with
`server.log`, `server.json`, and an owner lock. Stop affects the Dashboard group;
existing Runner workers retain their separate lifecycle ownership.

## Verification and evidence

- Mathematical/Factory/ROS tests verify exact position hold, zero future velocity,
  multiple dimensions, new-estimate updates, timestamps, invalid input rejection,
  required disabled corrections and ROS model/frame identity.
- Actual Author validation, immutable Save/materialization, Prediction Factory and
  safe control compute were exercised for **Jacobian**, **WLS**, and **MPC**.
  Jacobian uses its existing Cartesian task; WLS/MPC use Distance + Pointing.
  All three integration cases passed; no experiment was executed.
- **202 functional tests passed** in the final suite, including the real retained
  completed GP Run and original readiness-timeout Run. Earlier eight optional
  evidence tests were skipped until their real artifact paths were explicitly supplied;
  the final suite has no skips. Existing Dash DataTable deprecation warnings remain.
- Additional focused Builder option filtering and variant/ROS tests passed. New
  production/test files pass Flake8 at 99 columns. No broad style cleanup was done.
- Build: `colcon build --packages-select robot_prediction robot_dashboard` passed.
- Windows/WSL interop was available: the actual Windows Start `.cmd` successfully
  requested opening the browser URL; repeated starts reused one server. Windows
  `Invoke-RestMethod` reached `/dashboard-health` and verified the default Catalog.
  The actual Stop `.cmd` stopped the owned server and closed port 8060.
- A controlled occupied socket was rejected with a clear error and no owned server
  created. A review reproduced an unrelated Python process carrying module-looking
  positional arguments: an independent RED→GREEN regression now rejects it. A
  different-port ownership record is also rejected. Exact owner checks were verified
  against the running server.
- Browser verification loads the existing comparison Experiment, inspects available
  dropdowns and an **unsaved** None draft. The existing comparison declaration
  explicitly restricts its model variable to the original three Predictors, so
  selecting None in that comparison correctly fails its policy check. An independent
  None draft (comparison disabled explicitly in the detached specification) validates
  and previews successfully through the existing backend. It does not save a revision or
  start a Run. Evidence: `results/dashboard_d49/browser.json`, `none_dashboard.png`.
- Preservation audit: `results/dashboard_d49/preservation.json` checks all 242
  pre-existing files under the default Catalog workspace by SHA256. No changed or
  missing files: Predictors Comparison, its three immutable Experiments, Runs,
  recorded Results and diagnostic files remain intact. No new experiment Run was
  created by this task.

Artifacts and logs remain under ignored `results/dashboard_d49/`, including TDD
failures, final regression, build, Windows start/stop/health and browser evidence.
Historical scientific data is not moved or deleted. No commit or push.

## Scope and remaining limitations

- The preserved Predictors Comparison policy authorizes only its three original
  model IDs. This task does not add a fourth Experiment or silently expand that
  declaration. For a new None baseline, author an independent Experiment with
  `components: {predictor: none, adaptation: none, fusion: none}` and
  `comparison: null`, or explicitly declare a comparison policy permitting `none`.
  Existing immutable revisions and Session references remain unchanged.

- Launcher readiness is a local Dashboard service check, not ROS/simulation readiness.
  The existing experiment deadlines and safety gates are unchanged.
- An older manually started server occupying 8060 must be stopped by its owner before
  using these launchers. They intentionally refuse to kill or adopt it.
- Windows shell execution, URL-open request and HTTP access were verified. No screenshot
  of the native Windows browser window is asserted; the visual capture uses installed
  Chromium against the same WSL server.
- None is a configured estimate-hold baseline. It requires corrections disabled and
  does not imply a stationary physical target or a tracking-performance improvement.
- No Gazebo Run was needed or executed for this task. The prior scientific and
  physical-safety acceptance limitations remain unchanged.

## Exact changed files for D4.9

- `src/robot_prediction/robot_prediction/baseline/position_hold.py`
- `src/robot_prediction/config/prediction.yaml` (new empty None owner parameters)
- `src/robot_prediction/robot_prediction/factory/system_factory.py`
- `src/robot_prediction/test/test_position_hold.py`
- `src/robot_prediction/test/test_prediction_components.py`
- `src/robot_prediction/test/test_stateless_model.py`
- `src/robot_prediction/test/test_prediction_node.py`
- `src/robot_experiments/test/test_position_hold_integration.py`
- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/results.py`
- `src/robot_dashboard/robot_dashboard/cli.py`
- `src/robot_dashboard/robot_dashboard/launcher.py`
- `src/robot_dashboard/test/test_builder.py`
- `src/robot_dashboard/test/test_results.py`
- `src/robot_dashboard/test/test_launcher.py`
- `tools/dashboard/Dashboard.ps1`
- `tools/dashboard/dashboard.sh`
- `tools/dashboard/Start Dashboard.cmd`
- `tools/dashboard/Stop Dashboard.cmd`
- `docs/dashboard.md`
- `docs/validation/dashboard_d49.md`

The working tree already contained MPC, EDMD, GP and Session-association changes.
Those remain preserved and are not reattributed to this task. Two accidental empty
launcher draft files were removed before delivery; no existing file was deleted.
