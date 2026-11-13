# Disabled external influence compatibility

## Verified root cause

The saved Halvorsen definition has three dimensionless states, the supplied rate
and alpha parameters, and physical output `scale*[x,y,z]`. Its ordinary mapped
position/velocity output validates for world deployment without influences. It
does not expose independent world velocity states into which an acceleration
profile can be injected unchanged. That restriction is correct when ON.

`EquationDynamics.__init__` passed every nonempty frozen influence list to
`compose_influences`. The latter called `_binding(base, profile)` before checking
`profile.enabled`. It even compiled the influenced candidate before the final
conditional assignment. Thus an OFF profile did not change the returned equations,
but its irrelevant model-binding requirements still blocked construction.

The trace through public authoring/UI interfaces is:

1. A library model is pinned by `draft_with_dynamics` / `choose_target_model`.
   That replaces target parameters with the selected exact model and its source pin.
2. Saved profiles arrive in effective target parameters. `hydrate_target_pin`
   preserves those frozen profiles and asks `describe_robot_response` for UI state.
3. `prepare_robot_response` preserves existing frozen profile data, defaults a new
   profile to OFF, and delegates validation to `EquationDynamics`.
4. Author preview/preflight and the simulation factory instantiate that same
   equation owner. The unconditional `_binding` raises
   `Unsupported influence projection: position must be direct world states`.

Both the direct owner regression and the real Dashboard-path regression failed
with that exact message before production changes. The failure was not caused by
Halvorsen's equations, library validation, environmental acceleration or ROS timing.

## Minimal correction and ownership

The simulation equation owner now treats an OFF profile as an inert frozen
configuration. It still validates required fields, finite data, profile version,
boolean enablement, world acceleration output, inputs/context, symbol names,
bindings structure, parameter bounds, expression safety, shape and physical units.
For the remaining mathematical checks it reuses `EquationDynamics` on an isolated
profile contract: the declared acceleration is the derivative of a temporary
velocity quantity, and profile inputs retain their declared context units. This
validation quantity is never inserted into the selected model, its state, output,
or runtime context. Its fresh name also avoids all expression references, ensuring
an undeclared symbol cannot accidentally bind to a validation-only quantity.

After independent validation the OFF branch continues without `_binding` or
candidate injection. Enabled profiles retain the original `_binding` and full
candidate validation. There is no Halvorsen branch, new integration algorithm,
projection approximation or disabled-profile deletion.

The backend separately probes the same profile with explicit ON to report whether
the Robot Response checkbox can actually be enabled. Valid OFF plus incompatible
ON yields `available=false`, `enabled=false`, its unchanged frozen profile/hash,
and the compatibility reason. It does not advertise a false capability. Compatible
six-state models keep their existing checkbox and ON/OFF behavior. No Dashboard
composition or validation pipeline was duplicated or bypassed.

## RED → GREEN evidence

Evidence is preserved in ignored `results/disabled_influence_compatibility/`:

- `red_unit.log`: **1 failed**, exact reported projection exception, before any
  production edit; 38 other cases deselected.
- `red_dashboard.log`: **1 failed**, same exception through library selection and
  Author preview in the Lab regression; 4 other cases deselected.
- `red_symbol.log`: an additional contract-isolation check initially exposed an
  undeclared-symbol collision with the validation quantity. The corrected fresh
  allocation excludes all referenced names; no AST/units validation was weakened.
- `regression_simulation.log`: **150 passed**, 5.88 s, including equation units,
  mapped deployment, Cartesian composition and influence mathematics.
- `regression_authoring.log`: **10 passed**, 108.02 s, covering public authoring,
  immutable profiles/Library revisions, environment and live-context gates.
- `green_dashboard.log`: **5 passed**, 175.48 s, including actual Dash callbacks
  and the new three-state workflow. Only 18 existing DataTable deprecation warnings.
- **Total: 165 passing tests.** No final functional test failure remains.
- `build.log`: **3 packages succeeded**, 1 min 13 s (`robot_simulation`,
  `robot_experiments`, `robot_dashboard`). The only build warning was the existing
  colcon warning about overriding the already-sourced robot_simulation underlay.

The new mathematical regression checks no-influence Halvorsen deployment, then a
disabled incompatible profile with the identical model. All derivatives, states,
physical positions and physical velocities are exactly equal across **200 RK4
steps of 0.002 s** with identical initial conditions. The effective definition is
unchanged, and no tracking context is required while OFF. Explicit ON still fails
the original projection check. The fixture copies the user's mathematics, initial
conditions, parameters and output exactly; only its test identifier is descriptive.

Disabled malformed version, enablement, frame, units, expression, parameter bound,
binding and undeclared-symbol cases remain rejected. Existing six-state scalar
and vector bindings, active malformed profiles, context safety, independent
acceleration references and exact OFF integration tests are retained unchanged.

The Lab regression uses the actual Dash HTTP Validate and Save callbacks. It
switches from a compatible saved Cartesian model to the scaled three-state model,
retains the existing OFF profile, hydrates the pin, checks the unavailable unchecked
Robot Response control, validates, saves and reloads the real immutable revision.
It checks the actual effective model/profile, rejects ON through the public backend,
and verifies unchanged prior source/Library/Catalog snapshots and revision count.

## Reproduction commands

From the existing workspace and Dashboard virtual environment:

```bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_simulation:$PWD/src/robot_dashboard:$PWD/src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_simulation/test/test_derivative_influences.py \
  src/robot_simulation/test/test_equation_dynamics.py \
  src/robot_simulation/test/test_position_output.py \
  src/robot_simulation/test/test_motion_composition.py \
  src/robot_simulation/test/test_mapped_deployment.py
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_experiments/test/test_influence_authoring.py \
  src/robot_experiments/test/test_equation_configuration.py
build/dashboard_venv/bin/python -m pytest -q src/robot_dashboard/test/test_influence_ui.py
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_simulation robot_experiments robot_dashboard
```

Separate package invocations avoid the existing ROS launch-testing duplicate-module
collection problem. All Catalog/Library writes use isolated temporary fixtures;
no real experiment Run or host user data is changed by these tests.

## Exact changed files

- `src/robot_simulation/robot_simulation/dynamics/derivative_influences.py` — separate
  inert profile validation from active model binding, reusing the equation owner.
- `src/robot_experiments/robot_experiments/backend.py` — accurately distinguish valid
  OFF configuration from capability to enable Robot Response.
- `src/robot_simulation/test/model_fixtures/halvorsen.yaml` — reusable immutable
  reproduction fixture, not a production model or template.
- `src/robot_simulation/test/test_derivative_influences.py` — exact numerical and
  contract-validation regressions.
- `src/robot_dashboard/test/test_influence_ui.py` — actual switching, hydration,
  preflight, persistence, reload, compatibility and history checks.
- `docs/validation/disabled_influence_compatibility.md` — this investigation.

No production Halvorsen definition, initial condition, mapping, historical revision,
threshold, controller, solver, ROS lifecycle, physics or Docker file is changed.
The original frozen profile remains stored even when OFF, retaining provenance.

## Verdict and limits

**PASS for the focused compatibility fix and its mathematical/authoring/UI scope.**
The installed-package smoke check (without source PYTHONPATH overrides) also
confirmed a deployable three-state OFF model, unchanged effective definition,
no required tracking context, preserved frozen profile, and unavailable ON status;
see `installed.json`. Python compilation and `git diff --check` completed cleanly.
Docker-related files were SHA256-compared with their pre-task contents; none changed
(`docker_preservation.json`). The task started on
`fix/disabled-influence-compatibility` with a clean working tree; no branch switch,
commit, merge or push was performed.

Live ROS/Gazebo integration is **UNVERIFIED in this task**: the tests exercise
mathematics, configuration/Factory preflight and real Dash callbacks, without
launching Gazebo or verifying live TF/robot response. There is no new measured
tracking-performance or physical-safety claim. Existing environmental acceleration
and enabled-profile deployment constraints remain; unsupported ON mappings still
require an appropriate mathematical projection rather than a bypass.
