# Independent experiment creation hotfix

## Root cause

The New management callback selects `create` mode, retaining the currently selected
Scenario as a configuration template. The load callback calls `begin_draft`, which
called `ExperimentBackend.new_draft`. That general-purpose API intentionally copies
the base experiment's explicit comparison declaration. Author also inherits the
base study's declaration unless explicitly overridden. Consequently, simply
omitting the field would not fix independent creation.

Selecting None was accepted by the registered Predictor/Factory, but preview's
comparison projection then rejected it against the inherited IMM/GP/EDMD allowlist:
`comparison variable predictor: effective model not permitted`. The actual Dash
Validate callback runs both validation and preview, so owner validation alone was
insufficient to reproduce the UI failure. The active Session is not an input to
draft construction; it did not cause the restriction.

## Correction

`begin_draft` explicitly sets `experiment.comparison: null` for `create` mode,
using Author's existing opt-out semantics. New copies, including Duplicate, inherit
the selected configuration and pinned base revision, while starting independently
of its comparison contract. Revise mode and the general-purpose backend draft API
retain their existing comparison behavior. Explicitly declared comparison policies
continue to enforce their allowed models.

Reset now retains the draft's comparison declaration, including explicit null.
Otherwise Reset would silently restore the base restriction after New. No policies
or stored revisions are migrated, rewritten, broadened or deleted.

## Verification

The new isolated regression first failed at the real Dash HTTP Validate callback
with the exact reported error; see
`results/dashboard_independent_creation/red.log` (1 failed, 67.11 s).

The regression exercises the New management callback, selected restricted IMM
Scenario → Load → Predictor/Adaptation/Fusion None → Validate → Create experiment,
then reloads the saved immutable revision. It checks all three actual None model
IDs, preservation of the inherited controller, absence of a comparison restriction
in the independent saved configuration, unchanged source revision and Session
metadata, Reset behavior, and rejection when an explicit restrictive policy is
reapplied. Revising the original comparison also continues to reject None.

- Corrected end-to-end callback regression: **1 passed**, 23 deselected,
  127.92 s (`results/dashboard_independent_creation/green.log`).
- Relevant regressions: **24 passed**, 309.25 s
  (`results/dashboard_independent_creation/regression.log`): immutable revisions,
  form edits, validation/save fingerprint enforcement, pending selection handling,
  revision refresh, duration invalidation, explicit comparison form validation,
  reserved identities, backend save/conflict/Reset, None mathematical/Factory
  validation, and Author/Factory integration with Jacobian, WLS and MPC.
- Build: **2 packages succeeded**, `robot_experiments` and `robot_dashboard`,
  15.0 s (`results/dashboard_independent_creation/build.log`).
- Python compilation and scoped `git diff --check` completed without errors.
- The test suites emitted only existing Dash DataTable deprecation warnings.

**Verdict: PASS** for this focused authoring hotfix. The failure was reproduced
before the production change and the requested creation path passes afterward;
explicit comparison membership remains restrictive. The already running Dashboard
must be restarted to load the updated modules; existing unsaved drafts should be
loaded again through New. No running user server was stopped by these tests.

Reproduce from the workspace with the existing Dashboard virtual environment:

```bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_prediction:$PWD/src/robot_dashboard:$PWD/src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest -q src/robot_dashboard/test/test_builder.py -k new_independent_none
build/dashboard_venv/bin/python /usr/bin/colcon build --packages-select robot_experiments robot_dashboard
```

Dashboard workflow after restarting its server to load the updated Python modules:
select a saved Scenario, click New, enter a display name, Load configuration, choose
None for Predictor, Adaptation and Fusion, Validate, then Create experiment.
Comparison restrictions remain active when editing existing comparison members.

All mutation checks use pytest-owned temporary Catalogs and Sessions. No Gazebo
Runs were executed, and the existing user Catalog, Session records, immutable
revisions and Results were not modified. Verification uses the real Dash HTTP
callbacks and owner services; no browser screenshot or new browser-layout claim is
part of this focused hotfix.

## Exact files changed for this hotfix

- `src/robot_dashboard/robot_dashboard/forms.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_dashboard/test/test_builder.py`
- `docs/dashboard.md`
- `docs/validation/dashboard_independent_creation.md`

Pre-existing D4.9 and earlier working-tree edits are preserved. No commit or push.
