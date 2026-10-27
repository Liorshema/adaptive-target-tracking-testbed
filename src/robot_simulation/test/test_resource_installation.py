"""Installed resource trees must contain assets without local backups/caches."""

from pathlib import Path
import runpy

import setuptools


def test_resource_installation_excludes_backups_and_bytecode(tmp_path, monkeypatch):
    """Use a synthetic resource tree so cleanup alone cannot make the check pass."""
    setup = Path(__file__).resolve().parents[1] / 'setup.py'
    assets = ['config/sensors/nested/active.yaml', 'models/target/model.sdf',
              'worlds/nested/world.sdf.xacro']
    debris = ['config/sensors/nested/active.yaml.bak', 'models/target/model.sdf.orig',
              'worlds/nested/__pycache__/cached.pyc', 'models/.hidden/secret.sdf']
    for name in assets + debris:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('fixture')
    captured = {}
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(tmp_path)
    runpy.run_path(str(setup))
    installed = {p for _, files in captured['data_files'] for p in files}
    assert set(assets) <= installed
    assert not set(debris) & installed
