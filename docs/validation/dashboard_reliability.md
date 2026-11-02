# D2.8b — Reliable deletion and complete trajectory visualization

Completed 2026-11-02. **Deletion: PASS. Trajectory coverage/history: PASS.**
Desktop layout and real-browser verification passed at all three requested sizes.
These verdicts cover the managed-workspace contract and limitations below, not
physical robot safety or new scientific tracking-quality acceptance.

## Scope and implementation plan

Preserve the existing backend, immutable Runs, transaction/recovery protocol,
scientific metrics and four equal Results panels. No simulation or scored execution,
algorithm, physics, readiness, ranking or safety changes. Destructive tests use
only disposable managed workspaces.

1. Reproduce ROS `ros-log/latest` rejection; inspect real link destinations.
   Extend only the verified owned-Run tree inventory to recognize ordinary ROS
   log links, fingerprint their raw destinations, and remove entries without
   traversing links. Test Session/Scenario, stale-token, malicious link and recovery
   cases against the existing deletion suite.
2. Inventory genuine target streams and capture before screenshots. Add an
   efficient bounded-page iterator to the existing telemetry reader. Read the
   selected interval to EOF with bounded display sampling and endpoint retention;
   report exact available/plotted counts. Select one actual forecast by message
   index, label its origin/horizon, never join different forecasts. Test multi-page
   tails, validity/frame handling, downsampling and normal/expanded consistency.
3. Tighten existing Plotly margins and union-of-trace padded bounds while retaining
   physical equal XY scale and data-based 3D aspect. Keep a compact forecast selector
   and visible coverage summary in the existing panel. Verify installed Chromium at
   1920×1080, 1600×900 and 1650×790, both tabs without page scrolling.
4. Run focused functional/regression tests and two-package build, preserve original
   evidence, record exact changes and supported limitations here. No commit/push.

Evidence is stored separately in `results/dashboard_reliability/`. The earlier
D2.8a report and artifacts remain unchanged. A fresh source snapshot and manifest
were captured before this task.

## Investigation

The old `_tree` rejected all symlinks including ordinary ROS log links. The old
trajectory reader fetched one 200-message page at the selected raw offset for
each independent stream; the figure also sliced records to 200 and selected only
the last prediction on that page. The existing union-based ranges already used
8% padding and equal XY scale: pagination and forecast selection, as well as
avoidable plotting margins, require correction.

## Corrections and ownership

**Deletion:** only the exact `ros-log/latest` entry inside an exclusively selected,
identity-verified managed Run can be allowed. All 92 observed retained Run aliases
are absolute links to existing sibling ROS timestamp/host/PID directories. A
single-basename relative alias is also supported. The destination must be a direct
sibling with the standard ROS directory name, on the same filesystem, without
symlink ancestors or chains. Misplaced links, external or cross-Run targets,
lexical escapes, dangling initial aliases, special files and ambiguous ownership
still block deletion. Job artifacts and registered historical roots gain no
additional deletion authority.

The tree inventory uses `lstat`/`readlink`, never walks through an alias, and includes
its raw destination in the confirmation token. The existing journal retains the
verified Run paths and exact raw destinations. During staged cleanup, absolute
original paths are mapped lexically into the staged tree; the old location is
never traversed. The existing Linux fd-based `shutil.rmtree` removes the owned
tree and unlinks aliases rather than following them. The sibling log directory
is ordinary owned content independently of the alias; it is never reached through
`latest`.

Review exposed one additional interrupted-cleanup case: `rmtree` could remove the
sibling directory and fail before unlinking `latest`. Recovery now tolerates that
already-removed destination only in reserved staging with the exact persisted
confirmation. Initial discovery remains strict. Replacement files/symlinks and
retargeting remain blocked. Precommit rollback, postcommit retry, scope tokens,
shared-Run protection, cleanup verification and receipt scrubbing remain intact.
The existing compact UI confirmations, short notices and Advanced diagnostics
are reused.

**Telemetry:** `ExperimentBackend.iter_telemetry` extends the existing authoritative
reader with single-pass, at-most-1000-message pages. `read_telemetry` preserves its
existing single-page interface. There is no new store or scientific computation.
The dashboard scans the selected recorded tail to EOF. Default offset zero means
the whole available recording, including startup and settling, independently for
each stream; it does not imply synchronized epochs or a scored-window selection.

Truth/Estimate paths retain every valid sample up to the 4000-point display budget.
Longer recordings use disclosed display sampling with actual first/last points
retained. Server aggregation and browser traces stay bounded; long scans still
require time proportional to recording length. Sampling is only for presentation,
not a change to recorded evidence or metrics. Frame/unit/epoch checks cover the
whole scan, including unsampled points; a failed scan is unavailable rather than
a valid-looking complete prefix. Invalid coordinates are counted, not fabricated.

Forecasts are independent messages with their own origin and future epochs.
The latest recorded origin is the default. The **Forecast** number selects any
recorded message; both normal and expanded views use the same pipeline, and an
expanded selection updates the normal view. A single selected horizon is plotted,
never a connected history of different forecasts. Recorded/declared horizon counts,
partial/unknown states and display sampling are explicit. History before/after
recording cannot be inferred. No interpolation, frame transform, metric change,
false synchronization or manufactured Z values is introduced.

The figure uses union-based 8% padded ranges, compact margins/legend, fewer horizontal
XY ticks, equal physical XY scale and 3D data aspect. Native zoom/pan/reset/orbit
remain available. The compact forecast control overrides the generic full-width
input style. A narrow physical XY extent still produces a narrow drawing: widening
it independently would distort geometry. The normal panel includes honest coverage
controls; expanded mode provides substantially more viewing area. Four equal panels
and the existing responsive grid remain unchanged.

## Genuine recorded coverage

All numbers below were checked against original JSONL records, coordinates and
source epochs, in both XY and XYZ. These Runs are registered read-only; no new
scored experiment was run. Full evidence: `coverage_verified.json` in the artifact
folder.

| Run | Stream | Available / plotted | Source or forecast epochs [s] |
| --- | --- | --- | --- |
| `1a0712b4-6226-47f3-a2cf-5a11a2cb3a2a` | Truth | 1264 / 1264 | 0.068–23.266 |
| same | Estimate | 368 / 368 | 0.658–23.200 |
| same | Prediction | 368 independent origins; selected 20 / 20 states | origin 23.200; forecast 23.300–25.200 |
| `823fa1b3-36cf-4b36-9cc0-6288c02b4213` | Truth | 588 / 588 | 0.728–10.000 |
| same | Estimate | 161 / 161 | 0.728–10.016 |
| same | Prediction | 160 independent origins; selected 20 / 20 states | origin 9.956; forecast 10.056–11.956 |

Both use recorded `world`, position in metres and source/forecast epochs in seconds.
There were zero invalid Truth/Estimate samples in these recordings. First, middle
and last origins were checked separately: indices 0/184/367 and 0/80/159. Predicted
future epochs beyond the last Truth epoch are the actual recorded forecast, not
additional future Truth.

For the longer Run, the old view showed only Truth 0.068–4.122 (200/1264), Estimate
0.658–12.982 (200/368), and the forecast from origin 12.982 with epochs
13.082–14.982. Thus the truncation was primarily pagination and origin selection,
not clipping or a frame transform. Before/after charts have intentionally different
spatial ranges because all genuine points and the selected complete horizon now
participate in the fit. Forecast paths can still be short because a horizon is
finite; no missing samples are invented to make it longer.

## Functional verification

- Final combined suite: **112 passed in 91.50 s** — all 77 deletion cases and all
  35 trajectory cases, after the partial-cleanup correction. Source-based tests
  use disposable temporary workspaces and genuine retained telemetry as appropriate.
  Log: `final_safety_trajectory.log`.
- Essential workflow regression: **84 passed in 121.22 s** — Results, trajectory,
  management dialogs, execution UI, simulation UI and backend paging/results.
  These overlap the focused tests; counts must not be added as unique tests.
  Log: `regression_final.log`. The final focused suite additionally includes the
  large-horizon and exact-display-budget boundary tests.
- Two-package installed build: **robot_experiments and robot_dashboard succeeded**
  in 11.5 s (`build_delivery.log`). Installed Results/backend/deletion modules and
  stylesheet match source (`delivery_preservation.json`).
- Task-relative lint: **zero introduced findings**, with 44 existing findings
  preserved in changed baseline files (`source_audit_final.log`). Changed plotting
  source/tests and deletion source/tests also passed focused 99-column flake8.
  Task-scoped `git diff --check` passed.
- The fresh preservation manifest covers **825 original files** from the actual
  workflow workspace and retained evidence root; none changed. Previous D2.8a
  reports, manifests, screenshots and generated experiments were not overwritten.

Deletion coverage includes Session whole-study/Run-only cases; absolute and relative
ROS aliases; Scenario cascades; shared evidence; 20 unexpected/malicious alias
shapes; stale confirmation after retargeting; precommit rollback and fresh-backend
postcommit recovery; reoccupied original paths; external/Job-discovered read-only
roots; active/uncertain Job protection; and partial `rmtree` interruption with
missing versus replaced targets. TDD red evidence and review correction logs are
retained under `deletion_symlinks_20261101T173817180494Z/`.

Real Chromium also executed the compact Session and Scenario confirmation flows
against `results/dashboard_reliability/workspace` only. Both fixtures contained
realistic ROS `latest` links. Session deletion removed its exclusive Run/alias,
preserved the shared Run byte-for-byte, updated the dropdown and showed success.
Scenario deletion then removed the remaining owned fixture Run, pruned its mixed
Session and preserved that Session's genuine original Run reference. No original
experiment was a destructive target; no Job was started. See
`browser_deletion_report.json` and the two `final_delete_*_1650x790.png` images.
The generated fixture definitions are explicitly labeled as fixtures and carry no
fabricated scientific metrics. Final scientific chart captures use a separate
read-only metadata fixture (`viewer_workspace`) referencing the original Runs.

## Browser verification and screenshots

Actual installed Chromium at device scale 1 / 100% zoom verified 1920×1080,
1600×900 and 1650×790. Both tabs fit without page-level horizontal/vertical scroll;
Results retains four equal panels. The 12 accepted normal/expanded XY/XYZ views
use exactly the same 1264 Truth, 368 Estimate and 20 selected forecast points.
Source epochs, `world`/metre metadata and physical equal XY scale were checked in
rendered Plotly figures; XYZ uses `aspectmode=data`. All points are included in
padded axis bounds. Coverage tests additionally checked exact original XYZ arrays.

`browser_accepted.log` / `accepted_browser.json` include full layout, matching data,
first/middle-origin interactions, expanded-to-normal forecast synchronization,
native 3D orbit/reset, live resize and Close retaining the selected Run. A final
visual review restored the pre-existing camera distance after finding a clipped
3D axis title with a closer camera. The six affected 3D views were recaptured and
verified separately (`browser_framing.log`, `framing_browser.json`); 35 trajectory
tests passed again in 4.90 s and the final two-package build completed in 10.2 s.
The smaller viewport's compact 3D view necessarily has less inspection room than
the expanded view; geometry is not stretched to fill it.

Earlier unsuccessful capture attempts and probes are retained. Some selected
through synthetic client updates during initial rendering or were interrupted by
a Results rebuild. The accepted check uses the actual dropdown option and a
separate read-only Session fixture; no production readiness/lifecycle behavior or
scientific criterion was changed to make browser checks pass. Explicit Run changes
or Results refresh rebuild the viewer, as before.

| Final view | 1920×1080 | 1600×900 | 1650×790 |
| --- | --- | --- | --- |
| Experiment Lab | [image](../../results/dashboard_reliability/verified_lab_1920x1080.png) | [image](../../results/dashboard_reliability/verified_lab_1600x900.png) | [image](../../results/dashboard_reliability/verified_lab_1650x790.png) |
| Normal 2D | [image](../../results/dashboard_reliability/accepted_normal_2d_1920x1080.png) | [image](../../results/dashboard_reliability/accepted_normal_2d_1600x900.png) | [image](../../results/dashboard_reliability/accepted_normal_2d_1650x790.png) |
| Expanded 2D | [image](../../results/dashboard_reliability/accepted_expanded_2d_1920x1080.png) | [image](../../results/dashboard_reliability/accepted_expanded_2d_1600x900.png) | [image](../../results/dashboard_reliability/accepted_expanded_2d_1650x790.png) |
| Normal 3D | [image](../../results/dashboard_reliability/framing_normal_3d_1920x1080.png) | [image](../../results/dashboard_reliability/framing_normal_3d_1600x900.png) | [image](../../results/dashboard_reliability/framing_normal_3d_1650x790.png) |
| Expanded 3D | [image](../../results/dashboard_reliability/framing_expanded_3d_1920x1080.png) | [image](../../results/dashboard_reliability/framing_expanded_3d_1600x900.png) | [image](../../results/dashboard_reliability/framing_expanded_3d_1650x790.png) |

For matched before/after comparison, baseline screenshots are
`before_stable_normal_{2d,3d}_<size>.png` and
`before_stable_expanded_{2d,3d}_<size>.png` in the same evidence folder. The baseline
uses the old 200-message pages and page-final forecast; final ranges cover the
whole recording and the latest selected horizon. Compact margins and horizontal
XY ticks reduce wasted annotation space. The coverage/origin control consumes a
small part of the panel; it is not claimed that every final compact trace is
larger in pixels than the incomplete baseline. Complete spatial bounds and
physical equal scale take priority over apparent visual size. Expanded mode shows
the same complete data at larger scale, with the final 3D axes fully visible.

## Verdict and remaining limitations

- **Deletion — PASS:** ordinary verified ROS aliases work for Session/Scenario
  deletion, including interrupted cleanup, without following links or granting
  external/Job deletion authority. Unsafe or ambiguous cases remain blocked.
- **Trajectories — PASS:** full available recorded interval coverage, bounded
  display memory, source provenance, independent forecast history, matching
  normal/expanded data and desktop layout were demonstrated with retained evidence.
- Paths beyond 4000 displayed points are explicitly sampled with endpoints retained;
  unsampled detail is available in the original artifacts. Scanning very long Runs
  takes linear time and is not a persistent cache or database index.
- A complete recorded forecast is finite. Missing/unrecorded history and future
  Truth are unavailable; the UI does not infer them. Frame-incompatible or malformed
  recordings remain unavailable. Plotly 3D needs WebGL; validation used software WebGL.
- Managed-workspace reference/ownership limits from D2.8a remain: unrelated historical
  roots and uncertain active/cleanup states are protected. No new scored Run,
  Gazebo/physical-safety acceptance, comparison ranking or algorithm claim is made.

## Reproduction

```bash
cd /home/user/ros2_ws
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest -q \
  src/robot_experiments/test/test_deletion.py \
  src/robot_dashboard/test/test_trajectory.py
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
```

Serve existing real evidence on a free port (do not launch a second listener on an
occupied port):

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/runner_readiness/workspace \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel
```

In Results & History select a genuine recorded Run. Keep Advanced paging offset
zero for the whole recording. Use Forecast 1 through the displayed count to inspect
individual origins; use 2D/3D and Expand, then Close. No actuation or simulation
starts from viewing results. For destructive reproduction use the pytest fixtures,
not the actual user workspace. Browser destruction scripts are one-shot evidence:
their fixture Scenario ID is retired after successful deletion.

## Exact task-relative changed files

- `src/robot_experiments/robot_experiments/deletion.py` — narrow ROS alias inventory
  and staging/recovery verification.
- `src/robot_experiments/robot_experiments/backend.py` — authoritative bounded-page
  telemetry iterator and compatible single-page access.
- `src/robot_dashboard/robot_dashboard/results.py` — full-interval bounded plotting,
  coverage/provenance, independent forecast selection and shared expanded view.
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css` — compact forecast controls.
- `src/robot_experiments/test/test_deletion.py` — disposable symlink/recovery cases.
- `src/robot_experiments/test/test_backend.py` — pagination/endpoint contract.
- `src/robot_dashboard/test/test_trajectory.py` — coverage, history, validity, bounded
  sampling and actual callback regressions.
- `src/robot_dashboard/test/test_results.py` — existing test adapter recognizes the
  authoritative iterator.
- `docs/dashboard.md` — operator-facing ownership, history and coverage semantics.
- `docs/validation/dashboard_reliability.md` — this evidence report.

No production files were removed or relocated. Generated scripts, logs, manifests,
fixture metadata and screenshots live separately under `results/dashboard_reliability`.
No scientific algorithm, Runner, Recorder, Jobs engine, physics, controller, lifecycle,
readiness deadline, safety gate, metric or ranking policy was changed. No commit/push.
