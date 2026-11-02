# Dashboard D1 — Experiment Builder verification

Date: 2026-10-29. **Verdict: PASS for D1's stated authoring scope.**
Independent interactive Gazebo setup remains explicitly unsupported and disabled,
as permitted by the task. This verdict does not upgrade earlier runtime,
localization, tracking-quality or physical-safety acceptance verdicts.

## Delivered behavior

A separate ament_python `robot_dashboard` package serves one responsive Plotly Dash
screen. It consumes `robot_experiments.backend.ExperimentBackend`; the existing
Author, Catalog, owner schemas and M07 preflight remain authoritative.

- Select any saved experiment/revision and inspect its complete backend explanation.
- Create from a supported base/condition or edit the latest revision. Bases are
  pinned. Schema-2 comparisons remain inspectable, but cannot be one-file Author
  bases; the backend's explanation is surfaced when loading one is attempted.
- Discover all ten currently supported Author roles: robot/target estimator,
  predictor, adaptation, fusion, controller, task, reference policy/geometry and
  target dynamics. Model IDs/options/defaults are not maintained in the UI.
- Edit explicit local overrides using schema-derived numeric, boolean, string,
  enum and JSON array/object controls. Bounds, types, units and owner defaults
  are displayed. Missing metadata is labeled rather than invented.
- Inspect effective configuration, implementation identities and available
  provenance. Exact parameter ancestry that the backend cannot prove is labeled
  unknown. Preview snapshots are marked stale after form edits.
- Preview, validate and explicitly create/revise through existing APIs. Save
  revalidates and checks the captured expected revision. Edits invalidate prior
  approval. Conflicts preserve the draft; no historical revision is overwritten.
- Reset unsaved selections/parameters to the pinned base without changing saved
  definitions or defaults. Component redraws retain unrelated unpreviewed edits.
- Show physical robot/world source, entity, physics, deployment/holding/runtime
  settings and configuration preflight. These are resolved configuration details,
  not a live simulator status.

The default workspace Catalog is initialized once by the existing
`ExperimentCatalog.initialize()`. `--catalog` shares an existing authoritative
writable Catalog with headless clients. Browser draft state is detached and
memory-only. No separate result database, engine, registry, validator, numerical
calculation, Gazebo manager or Run/Stop interface was added.

## Simulation capability audit

`ExperimentBackend.gazebo_setup_capabilities()` reports independent interactive
setup and live reconfiguration unsupported, with a controlled restart required.
`robot_simulation/launch/simulation.launch.py` requires explicit deployment
configuration and exposes explicit actuation controls. The bringup closed-loop
launch consumes a validated composition. Neither supplies the independently
owned setup/reconfiguration/cleanup lifecycle needed for a safe dashboard action.
Existing scored Runner deployments already own isolated ROS/Gazebo lifecycles.

D1 therefore shows **Open Simulation disabled**, with the actual backend reason.
There is no launch callback, live robot/world mutation or actuation toggle. No
Gazebo experiment was run for this UI task; claiming live setup works would be
incorrect. Physical configuration, constraints and numerical behavior were not
changed.

## Measured verification

Evidence directory: `results/dashboard_builder/`. Counts below refer to distinct
suites; repeated focused reruns overlap and are not additional tests.

| Check | Actual result | Evidence |
|---|---|---|
| Dashboard functional tests, including real Dash HTTP callbacks | **11 passed**, 154.98 s | `dashboard_verified_tests.log` |
| Final renderer/provenance rerun after final renderer edit | **2 passed**, 17.79 s; subset of dashboard tests | `final_render_tests.log` |
| Public backend regression | **24 passed**, 62.53 s | `backend_regression.log` |
| Architecture/dependency/tree regression | **53 passed**, 46.91 s | `architecture_regression.log` |
| Dashboard scoped flake8 | **PASS**, no findings | `lint_verified.log`; package's 99-column configuration |
| Normal installed package build | **PASS**, 8.23 s | `build_installed.log` |
| Separate copied-prefix dashboard build | **PASS**, 2.97 s | `build_isolated.log` |
| Installed console entry / help | **PASS** | `installed_help.txt` |
| Separate-prefix import/index/CSS/source-byte identity | **PASS**, HTTP 200, copied module/asset bytes match source | `isolated_install.json` |
| ROS package discovery | **11 packages**, including dashboard | `packages.txt` |
| Real Chromium browser acceptance | **10 checks passed, zero page/console errors** | `browser_results.json`, `browser_verified.log` |
| Headless dependency isolation | **PASS**, system Python has no Dash and can use backend discovery; blocked-UI-import subprocess also passes | `test_headless_backend_does_not_import_dashboard_dependency`, installed smoke |

The copied dashboard installation uses the existing built ten-package ROS stack
as its underlay. Colcon correctly warns that these unchanged dependencies come
from the existing workspace; this was not a fresh rebuild of the whole stack.
Dash 3.4.0 / Plotly 7.1.0 ran under Python 3.12 / ROS 2 Jazzy. Playwright 1.64.1
and downloaded Chromium were verification tools only. Runtime dependency ownership
is confined to the new package (`dash>=3,<4` and the ROS
[`python3-dash-pip` key](https://index.ros.org/d/python3-dash-pip/)). The rest of the
repository's broad functional/style suites and Gazebo were intentionally not
rerun. Existing repository lint issues were not suppressed or reclassified.

The browser workflow exercised:

1. Installed page, real Catalog choices and disabled setup.
2. All ten discovered roles.
3. Negative damping rejection with a field-specific error and Save disabled.
4. Entering `0.23`, switching predictor to `constant_velocity` before Preview,
   and retaining the unrelated damping value and enabled override.
5. Effective dynamics **`saturated_fight_flight`**, damping **0.23 s^-1**, owner
   default **0.5 s^-1**, inherited selection and explicit parameter provenance.
6. Explicit new experiment creation as immutable r1.
7. Editing to 0.31 invalidating validation, then explicit CAS save as r2.
8. Unsaved edit/reset returning to pinned nominal damping 0.5, not modifying history.
9. A 390-pixel mobile layout without horizontal overflow.
10. Selecting that saved experiment, revising to 0.35 as r3 and seeing the revision
    selector refresh to r3 without switching experiments or reloading the page.

The validation Catalog retains `EXP-DASH-BUILDER` r1/r2/r3 for inspection, separate
from normal user workspace metadata. The final audit reads their real effective
configurations and records revision hashes in `change_manifest.json`.

Initial negative-input and reset browser checks had harness assumptions corrected:
the bounded numeric widget rejects out-of-range values immediately, and reset
must finish its backend preview before values are asserted. Their failed logs
are retained. Independent review also found two actual UI defects (stale values
on card redraw, stale revision choices); both were reproduced by failing tests,
fixed and verified in the final browser workflow. Final review found both issues
resolved and no remaining important issue in those fixes.

## Screenshots

These are captured from the installed running application, using real backend
configuration and persisted revisions, rather than mockups.

- [Validated desktop builder](../../results/dashboard_builder/builder_validated.png)
- [Saved revision](../../results/dashboard_builder/builder_saved.png)
- [Responsive mobile builder](../../results/dashboard_builder/builder_mobile.png)

## Preservation and scope

The task-start baseline captured 735 existing source/documentation files and
15,411 historical artifact size/mtime records. The audit compares every captured
source/documentation SHA-256, explicitly allowing only the dependency-map addition
listed below. It checks all captured artifact metadata, compares copied shipped
Catalog revisions with the authoritative builtin records, and explains the new
browser-created revisions through the public backend.

The existing architecture ownership map now admits only `robot_experiments` as a
dashboard project dependency, protecting headless packages from importing the UI.
The existing tree tool knows the dashboard/assets responsibility so engineers
can inspect the new package through its established command.

No owner YAML, physical limit, algorithm, estimator, predictor, target dynamics,
metric, backend implementation or historical validation file was changed. No
files were deleted, and no commit or push was performed. Generated development
and verification outputs stay in ignored build/install/results directories.
The final audit found **734 protected source/documentation files unchanged**, all
**15,411 captured historical artifacts unchanged**, and all **21 shipped Catalog
revisions unchanged**. The user-supplied Hebrew blueprint and its metadata
attachment were preserved.

## Exact task-relative files

This list is relative to the dirty D1 task-start snapshot, not Git HEAD.

```text
M src/robot_bringup/test/test_architecture.py
M tools/show_architecture.py
A docs/dashboard.md
A docs/validation/dashboard_builder.md
A src/robot_dashboard/package.xml
A src/robot_dashboard/resource/robot_dashboard
A src/robot_dashboard/setup.cfg
A src/robot_dashboard/setup.py
A src/robot_dashboard/robot_dashboard/__init__.py
A src/robot_dashboard/robot_dashboard/app.py
A src/robot_dashboard/robot_dashboard/cli.py
A src/robot_dashboard/robot_dashboard/forms.py
A src/robot_dashboard/robot_dashboard/assets/dashboard.css
A src/robot_dashboard/test/test_builder.py
```

## Reproduction

Installation steps and normal workflow are in [the dashboard guide](../dashboard.md).
After the verified build:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --data-dir results/dashboard_workspace
```

The installed application was left running at **http://127.0.0.1:8050** with the
clean `results/dashboard_workspace` Catalog, separate from acceptance-test data.
A final Chromium visit returned HTTP 200 with zero page errors (`ready_browser.json`).
Host, port, data directory and optional existing
Catalog path are the only new CLI configuration entry points. No new YAML profile
or competing parameter authority was introduced.

```bash
# Tests exercise source UI, installed ROS dependencies and temporary real Catalogs.
source install/setup.bash
PYTHONPATH=src/robot_dashboard:$PYTHONPATH build/dashboard_venv/bin/python -m pytest \
  src/robot_dashboard/test/test_builder.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_backend.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py -q
build/dashboard_venv/bin/python -m flake8 \
  --config src/robot_dashboard/setup.cfg src/robot_dashboard
python3 tools/show_architecture.py --package robot_dashboard --depth 2
```

`results/dashboard_builder/browser_workflow.py` documents the actual browser
acceptance sequence against port 8050 and the validation Catalog. Its fixed new
ID is intentionally single-use: do not rerun it against retained evidence without
choosing a fresh ID or a separate temporary Catalog. No evidence needs overwriting
to reproduce the behavior manually.

## Remaining limits and D2 handoff

- No standalone interactive Gazebo setup; a future authoritative lifecycle service
  would be needed before enabling that action.
- Drafts are lost on browser refresh. Conflicts require deliberate reload/rebase;
  D1 performs no automatic merging of scientific configurations.
- Schema-2 comparisons are inspectable but not one-file editable bases. Editing
  advanced bindings/resources or comparison contracts remains with existing APIs.
- Arrays/nested mathematical objects use JSON parameter controls. Full per-leaf
  provenance remains unavailable where the backend cannot provide it.
- The server targets a local engineering workspace, not authenticated multi-user
  production hosting. Existing scientific/runtime acceptance limits remain.

D2 should add explicit Run/Cancel using the existing backend jobs API, truthful
readiness/cleanup/error states, recorded results and current Session comparison.
Reuse the saved Experiment/revision identity and existing Run-owned metric/artifact
references. Do not add an execution engine, new metric calculations or duplicated
results. Historical analytics remain D3. Work stops at D1.
