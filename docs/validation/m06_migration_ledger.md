# M06 migration ledger

Scope: approved focused compatibility migration; no algorithm or configuration-value changes.

- Preflight: source/active-validation snapshots and full dirty/untracked Git inventory saved in m06_migration_before.
- Contract RED: missing canonical loader and 10 canonical imports; resource backup/cache exclusion failed (11 contract failures).
- Implementation: 10 canonical relocations with module-identity aliases; existing implementation bodies verified identical after import-path normalization.
- Packaging: direct dependencies/buildtool metadata, model version and implemented-package documentation corrected.
- Resource cleanup: three ignored backups copied, hash-verified and removed from source; recursive resource installation excludes backup/cache artifacts.
- Scope ruling: use the existing workspace and preserve snapshots instead of a clean worktree/commits, as explicitly required by the user.
- Scope ruling: keep historical nested validation captures and archived before-source fixtures byte-for-byte; update only active top-level utilities and current source imports.
- No M07/M08 selection, algorithm, target, PID, limit or timing-policy changes.
- Build preflight correction: added COLCON_IGNORE to the rollback snapshot; otherwise colcon recursively discovers its preserved package.xml files as duplicate packages. No product change.
- GREEN focused: 113 tests passed in 2.79 s (canonical/legacy identity, loader semantics, node/adapter/model groups, resource exclusion).
- Build: normal symlink workspace 9 packages passed in 14.7 s; isolated copied install 9 packages passed in 32.7 s.
- Full functional regression: 871 passed, 18 existing style/copyright tests deselected, 15.10 s. One full run after migration.
- Real probe checks: current and archived estimator/actuator implementations share canonical module globals in four isolated subprocesses.
- Additional confirmed packaging correction: description display launch directly imports launch/launch_ros; declared these dependencies too.
- Final independent bounded review: no critical/important findings; reviewer verified body equivalence, backup hashes, compatibility globals, packaging and probe imports. Primary verification owns full-suite, copied-install and headless checks, which reviewer deliberately did not rerun.
- Isolated copied-install check exposed pre-existing missing control factory/pipeline initializers. New setuptools-discovery tests failed twice before correction. Added docstring-only __init__.py markers; no numerical code changed. Rebuild and installed check repeated for this confirmed packaging defect.
- Short 16 s headless nominal run completed through production composition with read-only trace prefixes; all chain/position/watchdog/command gates worked, measured physical speed gate remains failed as in M02. No second simulation run planned.
- Final GREEN: copied install resolves 7 canonical entry points, 10 legacy identities, 65 installed assets and generated Xacro limits from /tmp without workspace imports. Final focused tests 115 passed in 1.60 s.
- Recorded before/after replay matches saved M03 counts/errors to 1e-12 without overwriting original evidence.
- Final checks: implementation equivalence, AST parse, document links/fences and git diff whitespace clean. No ROS/Gazebo smoke processes remain. Report, 450-file actual source tree and 695-path task inventory complete. No commit/push; stop at M06.
