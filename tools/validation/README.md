# Validation utilities

Run these checkout tools after sourcing `install/setup.bash`. They are offline
analyses, read-only probes, installation checks and historical reproductions;
they are not runtime ROS packages. Sibling YAML files are reusable fixtures.
Production components and configuration remain in their owning `src/` packages.

Examples from the workspace root:

```bash
python3 -m pytest -q -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
python3 tools/validation/m06_verify_instrumentation.py --output results/instrumentation
python3 tools/validation/m02_verify_installation.py --prefix /path/to/copied/install \
  --output results/installation.json
python3 tools/validation/v2_tracking_gazebo.py --help
```

Root `pytest.ini` scopes collection to active source and validation tests, excluding
captured historical source trees. Run style checks separately; the functional
command above deliberately excludes them.

Write new captures to a unique directory under `results/`. Historical capture
defaults use `results/validation/`; explicit `--output` paths are preferred.
Archived estimator/actuator imports resolve there and translate old imports in
memory without editing captured source. Existing reports in `docs/validation/`
link to the corresponding tools and evidence. Preservation/report-population
utilities are milestone-specific historical checks, not current acceptance gates;
their original hashes and milestones must not be updated to manufacture a pass.

Large archived studies/copied workspaces require extraction using the adjacent
`results/archives/*.json` manifest's restore command. The root README describes
artifact ownership and restore paths. Local results, archives and rollback backups
are ignored and must be transferred separately with scientific evidence.
