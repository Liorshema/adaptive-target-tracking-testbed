# D2.8a — Safe deletion, Scenario rename and expanded trajectories

Date: 2026-11-01. **Session deletion: PASS. Scenario deletion: PASS. Scenario
rename: PASS. Expanded trajectory viewer: PASS**, within the managed-workspace
ownership contract and explicit blocked cases below. No scored Gazebo Run,
scientific algorithm change, commit, push or D3 work.

## Ownership and cascading behavior

The existing `ExperimentBackend` remains the UI boundary. Its `ResourceDeletion`
service coordinates the existing Catalog, Jobs, Session JSON and Run directories;
it is not an execution engine or a second result store.

- `preview_deletion(kind, identifier)` returns actual Run/shared-Run/affected-Session
  counts and a scope token. `delete_resources(kind, identifier, expected_token=...)`
  rechecks that exact scope under locks before performing any destructive action.
  The UI has no client-selected filesystem delete path or unrestricted purge API.
- Writable ownership defaults to `DATA_DIR/runs`. Repeatable
  `--managed-result-root PATH` explicitly configures writable boundaries. Existing
  `--result-root` registrations and discovered Job outputs grant **viewing**, not
  deletion authority. Registering historical evidence does not authorize its removal.
  All references are inventoried in the backend's managed Session workspace.
- A Session deletion removes its document and exclusively referenced managed Runs.
  A second Session referencing a Run protects it. Other Session documents and
  unrelated Runs remain intact. Shared study summaries remain protected when the
  study contains retained Runs; the confirmation explains this case.
- A Scenario is the existing Catalog Experiment. Runs are matched by immutable
  experiment ID/revision/hash, not names or directory labels. Run status, metadata
  and study identity must agree. Scenario deletion removes its owned studies,
  their results/metrics/telemetry, and Catalog revisions; it prunes only matching
  Run references in every affected Session. Empty and mixed Sessions survive.
  The Experiment ID remains retired and cannot be silently reused.
- Fully selected, verified studies include their standard summary/CSV/configuration
  artifacts in deletion. An owned empty study is also included. Missing Run status
  cannot hide a directory: inventory uses independent study/metadata anchors.
  Unowned files or an incomplete/shared/unmanaged study block Scenario deletion
  rather than leaving associated scientific summaries behind and claiming success.
- Symlinks, lexical path aliases, special files, nested cross-filesystem mounts,
  duplicate/conflicting identities, missing ownership or unverified Run cleanup
  block destructive operations. Scope tokens include all Session references,
  Catalog state, managed roots and target-tree file identities/sizes/mtime.
  Concurrent reference, Catalog or artifact changes invalidate confirmation.

### Jobs, locking and partial failure

Lock order is backend metadata → existing Jobs preview lock → Catalog → Session
locks. Session create/append/rename, accepted scored starts and experiment saves
use the same backend metadata boundary. Catalog writers retain their existing
owner lock. The normal Runner, worker, Gazebo and stop lifecycle remain unchanged.

Active, unknown, lost-worker or incompletely cleaned associated work blocks
removal. A legacy unassigned active Job conservatively blocks Session deletion.
For terminal legacy Jobs, retained Run statuses and empty cleanup survivor lists
supply actual cleanup evidence. A top-level `cleanup_complete=true` cannot override
contradictory retained Run evidence. Missing cleanup proof still blocks the action.

Deletion writes a durable undo journal, stages data by same-filesystem rename,
then commits metadata. Precommit failure restores references/Catalog/data.
Postcommit cleanup failure is an error and can resume only the same confirmed
scope. Pending transactions block supported metadata mutations and scored starts.
Completed/rolled-back journals are scrubbed of Session and Catalog preimages;
completed receipts do not retain deleted definitions. Small terminal-Job cleanup
proof hashes prevent deletion itself causing a permanent legacy block when the
original cleaned Run artifacts are intentionally removed. Job control/audit
metadata and shared infrastructure are retained.

If a browser is refreshed after an interrupted deletion, recover through the
backend before further writes. Find its pending receipt in
`DATA_DIR/.deletions/*.json` and use the exact recorded kind/ID/token:

```python
import json
from pathlib import Path
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

workspace = Path('/absolute/path/to/owned/workspace')
api = ExperimentBackend(workspace, catalog=ExperimentCatalog(workspace / 'catalog.yaml'))
# Supply the same result_roots and managed_result_roots used by the dashboard if customized.
for path in (workspace / '.deletions').glob('*.json'):
    receipt = json.loads(path.read_text())
    if receipt['phase'] not in {'completed', 'rolled_back'}:
        scope = receipt['impact']
        result = api.delete_resources(scope['kind'], scope['id'], expected_token=scope['token'])
        assert result['completed']
```

No arbitrary path is deleted by that call: recovery validates its managed targets
again. Do not manually edit the Catalog/Session files while recovery is pending.
Out-of-band modifications or occupied recovery paths fail closed and require
owner inspection; they are never overwritten to force success.

## Rename and presentation

Catalog entries gain an optional mutable `display_name`. `rename_display()` checks
both expected latest revision and prior display name under the Catalog lock.
`list()` shows the alias; `get()` and experiment preparation retain the original
recorded name and immutable revision definitions/hashes. No separate naming store
or frozen-definition rewriting is introduced. Current dashboard labels use the
alias while historical scientific metadata stays interpretable.

A live browser test found a false rename rejection: the source revision selector
finished refreshing between opening and confirming the dialog. Since the alias
belongs to the Experiment ID, its operation now checks stable ID and authoritative
Catalog metadata, independently of the viewed historical revision. Actual ID or
concurrent Catalog name/revision changes remain protected.

Native browser confirmations are replaced by compact Dash dialogs with calculated
counts and explicit Cancel/Delete. Successful deletion clears stale Session/Run
selection and notifies; cached Job rows cannot resurrect removed Run options.
Errors remain short, with exact owner diagnostics under Advanced.

The existing trajectory panel gains an Expand icon. Its overlay uses most of the
viewport, supports original 2D/3D controls, native zoom/pan/reset and 3D orbit,
responds to resize and closes without changing selected Run. It reuses the bounded
reader and figure builder: up to 200 independent records per stream, final genuine
prediction horizon, original XYZ/source epochs/frame/units, equal XY physical
scale and data-proportioned 3D axes. No alignment, interpolation or fabricated data.
Optional panel resizing is deferred to preserve the simple fixed four-panel grid.

## Verification

| Gate | Actual result |
| --- | --- |
| Core deletion | **36 passed**, `deletion_delivery.log`; includes invalid/scalar/filesystem-root authority rejection, exact staging namespace and recovery |
| Namespace and recovery | **4 passed**, `namespace_recovery_acceptance.log`; ordinary user names remain discoverable, orphaned staging blocks purge, and pre/postcommit interruption resumes the same scope |
| Deletion + management compatibility | **70 passed**, `deletion_management_verified.log` (earlier 26 deletion + 44 existing management cases) |
| Final safety + modal API integration | **43 passed**, `final_safety_ui.log` (28 deletion + 15 modal cases; later extra safety cases above passed separately) |
| Catalog + composition | **37 passed**, including 13 display-alias cases; final alias recheck **13 passed**, `catalog_alias_verified.log` |
| Run/Cancel/preview + trajectories | **39 passed**, `workflow_trajectory_verified.log` |
| Results regression | **22 passed**, `results_verified.log`; includes actual row-selection callback and hidden overlay grid contract |
| Focused Builder | **4 passed**, `builder_remaining.log`; duration/revision and existing presentation checks |
| Build | `robot_experiments` and `robot_dashboard` built successfully; `build_complete.log` |
| Lint | Changed dashboard modules/tests and new deletion module/tests pass configured flake8; baseline-relative backend/Catalog findings in `lint_audit.json` |
| Browser destruction | Actual installed Chromium: rename persisted into a fresh browser; Session deletion removed **1** exclusive Run and preserved **1** shared Run byte-for-byte; Scenario deletion removed remaining **2** Runs and pruned **2** Sessions while preserving unrelated evidence; `browser_report.json`, `browser_metadata_finish.log` |
| Visuals | Both tabs and expanded 2D/3D at **1920×1080, 1600×900, 1650×790**, 100% zoom, 14px body text, no page overflow or clipped controls. Four KPIs and equal result panels retained. `layout_report.json` |
| Geometry | Expanded XY pixel/metre scales matched on both axes: approximately **541.418**, **433.012**, **365.257** at the three viewport sizes. Truth Z exactly matches retained telemetry. Live resize, native orbit and camera reset verified. |
| Preservation | **109 original files unchanged by SHA-256**: original Catalog, eight original Sessions, 100 files across two retained Runs. Installed modules/CSS match final source; Jobs engine and scientific comparison renderer unchanged. `preservation_audit.json` |

The broad dashboard attempt was interrupted (exit 143); no full-project pass is
claimed. Scoped suites above completed. Two old test adapters initially expected
the old callback input count and exact child list; they were updated for the
management refresh input and hidden overlay, then all 22 Results checks passed.
Browser callback waits were adjusted in verification scripts, without changing
Runner/readiness or lifecycle timeouts. Initial failures and diagnostic logs remain.

### Real-data and destructive-test separation

Original visualization evidence uses Runs
`1a0712b4-6226-47f3-a2cf-5a11a2cb3a2a` and
`823fa1b3-36cf-4b36-9cc0-6288c02b4213`, registered read-only. Test metrics/telemetry
were not substituted for their charts. The existing comparison still reports
**Rank = Unavailable** where comparability is rejected.

All destructive browser actions targeted `results/dashboard_cleanup/workspace`,
a disposable Catalog copy and three explicitly generated test Run fixtures.
`setup_browser.py` refuses to overwrite an existing fixture. Its Scenario was
renamed and then deleted; its ID is retired. No original Catalog, Session, Run or
historical validation artifact was a destructive-test target. No Job or scored
simulation was started. The before snapshot, manifests, diagnostics and generated
screenshots live in `results/dashboard_cleanup/`.

### Screenshots

| View | 1920×1080 | 1600×900 | 1650×790 |
| --- | --- | --- | --- |
| Experiment Lab | [image](../../results/dashboard_cleanup/final_lab_1920x1080.png) | [image](../../results/dashboard_cleanup/final_lab_1600x900.png) | [image](../../results/dashboard_cleanup/final_lab_1650x790.png) |
| Results | [image](../../results/dashboard_cleanup/final_results_1920x1080.png) | [image](../../results/dashboard_cleanup/final_results_1600x900.png) | [image](../../results/dashboard_cleanup/final_results_1650x790.png) |
| Expanded 2D | [image](../../results/dashboard_cleanup/final_expanded_2d_1920x1080.png) | [image](../../results/dashboard_cleanup/final_expanded_2d_1600x900.png) | [image](../../results/dashboard_cleanup/final_expanded_2d_1650x790.png) |
| Expanded 3D | [image](../../results/dashboard_cleanup/final_expanded_3d_1920x1080.png) | [image](../../results/dashboard_cleanup/final_expanded_3d_1600x900.png) | [image](../../results/dashboard_cleanup/final_expanded_3d_1650x790.png) |
| Rename | [image](../../results/dashboard_cleanup/rename_1920x1080.png) | [image](../../results/dashboard_cleanup/rename_1600x900.png) | [image](../../results/dashboard_cleanup/rename_1650x790.png) |
| Delete Session | [image](../../results/dashboard_cleanup/delete_session_1920x1080.png) | [image](../../results/dashboard_cleanup/delete_session_1600x900.png) | [image](../../results/dashboard_cleanup/delete_session_1650x790.png) |
| Delete Scenario | [image](../../results/dashboard_cleanup/delete_scenario_1920x1080.png) | [image](../../results/dashboard_cleanup/delete_scenario_1600x900.png) | [image](../../results/dashboard_cleanup/delete_scenario_1650x790.png) |

## Reproduction and limitations

```bash
cd /home/user/ros2_ws
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest -q \
  src/robot_experiments/test/test_deletion.py \
  src/robot_dashboard/test/test_management_ui.py \
  src/robot_dashboard/test/test_results.py \
  src/robot_dashboard/test/test_trajectory.py
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
```

Normal dashboard with existing evidence (use a free port or stop the existing
owned listener before launching a replacement):

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/runner_readiness/workspace \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel
```

`http://127.0.0.1:8060`. `--managed-result-root` is unnecessary for the normal owned
`DATA_DIR/runs`. Do not grant it to unrelated baseline/history roots merely to
make a blocked deletion pass. Ownership/reference guarantees cover this backend
workspace; independent workspaces' unknown references cannot be discovered without
a global registry, which this task deliberately does not introduce. Configure only
roots exclusively managed by this workspace. External or ambiguous historical
ownership is unsupported and blocks deletion.

Other deliberately blocked cases include read-only Catalogs, uncertain cleanup,
missing/conflicting identity, orphaned staging, unowned Scenario study files, cross-filesystem nested
mounts and out-of-band recovery conflicts. Shared study summaries remain unchanged
when deleting a subset through a Session. Pending cleanup after browser restart
requires the backend recovery call above. Plotly 3D requires WebGL; validation used
Chromium software WebGL and makes no Gazebo rendering or new physical-safety claim.

## Exact task-relative changed files

- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/robot_dashboard/cli.py`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/robot_dashboard/management.py`
- `src/robot_dashboard/robot_dashboard/results.py`
- `src/robot_dashboard/test/test_management_ui.py`
- `src/robot_dashboard/test/test_results.py`
- `src/robot_dashboard/test/test_trajectory.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/robot_experiments/catalog.py`
- `src/robot_experiments/robot_experiments/deletion.py` (new; coordinated owned deletion)
- `src/robot_experiments/test/test_catalog.py`
- `src/robot_experiments/test/test_deletion.py` (new; disposable destructive tests)
- `src/robot_experiments/test/test_management.py` (tightened legacy cleanup expectations)
- `docs/dashboard.md`
- `docs/validation/dashboard_cleanup.md` (new)

No production files were removed or relocated. The old metadata-only Session API
and soft archive API remain compatible for existing headless callers; the dashboard
uses only the explicitly confirmed cascade API. Earlier untracked work, profiles,
experiments and validation evidence were preserved. No commit or push.
