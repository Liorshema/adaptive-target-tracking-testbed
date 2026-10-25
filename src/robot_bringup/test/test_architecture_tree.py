"""The architecture view reflects real files and one semantic ordering policy."""

import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def renderer():
    path = Path(__file__).resolve().parents[3] / 'tools/show_architecture.py'
    spec = importlib.util.spec_from_file_location('architecture_view', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_structural_then_domain_directories_then_files(tmp_path, renderer):
    for name in ['tracking', 'nodes', 'interfaces', 'factory', 'adapters', 'controllers', 'safety', 'pipeline']:
        (tmp_path / name).mkdir()
    for name in ['z.py', '__init__.py', 'a.py']:
        (tmp_path / name).touch()
    names = [p.name for p in renderer.ordered_children(tmp_path, 'robot_control')]
    assert names == ['nodes', 'interfaces', 'factory', 'adapters', 'tracking',
                     'controllers', 'safety', 'pipeline', 'a.py', 'z.py', '__init__.py']


def test_real_tree_excludes_generated_and_keeps_unknown_domain(tmp_path, renderer):
    for name in ['nodes', 'new_domain', '__pycache__', '.pytest_cache', 'build']:
        (tmp_path / name).mkdir()
    (tmp_path / 'nodes/active.py').write_text('pass\n')
    (tmp_path / 'nodes/active.pyc').write_bytes(b'cache')
    view = renderer.render_tree(tmp_path, 'robot_control')
    assert 'active.py' in view and 'new_domain/' in view
    assert '__pycache__' not in view and '.pytest_cache' not in view
    assert 'build/' not in view and '.pyc' not in view
    assert view.index('nodes/') < view.index('new_domain/')
    assert 'factory/' not in view


def test_repository_render_is_actual_and_ordered(renderer):
    root = Path(__file__).resolve().parents[2]
    package = root / 'robot_control/robot_control'
    view = renderer.render_tree(package, 'robot_control')
    assert 'controllers/' in view and 'optimization/' not in view
    assert view.index('nodes/') < view.index('interfaces/') < view.index('factory/')
    assert view.index('factory/') < view.index('adapters/') < view.index('controllers/')


def test_archived_loader_translates_imports_without_editing_evidence():
    path = Path(__file__).resolve().parents[3] / 'tools/validation/archived_estimation_imports.py'
    spec = importlib.util.spec_from_file_location('archived_import_translation', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = ("from robot_estimation.common.state import StateEstimate\n"
              "note = 'robot_estimation.common.state'\n"
              "# from robot_estimation.common.measurement import Measurement\n"
              "result = 2 * 3\n")
    translated = module.canonicalize_imports(source)
    assert translated == source.replace('from robot_estimation.common.state import',
                                        'from robot_estimation.state import')


def test_experiment_domain_order_and_tenth_package(renderer, tmp_path):
    for name in ['reporting','metrics','recording','execution','configuration','benchmarking']:
        (tmp_path / name).mkdir()
    assert [p.name for p in renderer.ordered_children(tmp_path,'robot_experiments')] == [
        'configuration','execution','recording','metrics','benchmarking','reporting']
    view=renderer.render_workspace(Path(__file__).resolve().parents[2], depth=1)
    assert 'robot_experiments/' in view
