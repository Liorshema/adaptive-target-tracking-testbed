"""Catalog resources materialize into ordinary owner documents without ROS."""

from copy import deepcopy
from pathlib import Path

import pytest
from robot_configuration.yaml_loader import load_yaml
import yaml


@pytest.fixture
def resources(tmp_path):
    def write(package, name, value):
        path = tmp_path / package / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(value))
        return path

    owner = write(
        'robot_description',
        'robot.yaml',
        {'robot': {'joints': {'wrist': {'lower': -1.57, 'upper': 1.57}}, 'velocity': [1.0, 1.5]}},
    )
    catalog = write(
        'robot_experiments',
        'recipes.yaml',
        {
            'physical': {
                'base': {'package': 'robot_description', 'path': 'robot.yaml'},
                'overrides': {'robot': {'joints': {'wrist': {'lower': -0.25, 'upper': 0.25}}}},
            },
            'again': {
                'base': {
                    'package': 'robot_experiments',
                    'path': 'recipes.yaml',
                    'group': 'physical',
                },
                'overrides': {'robot': {'velocity': [1.0, 1.2]}},
            },
            'first': {'value': {'a': 1, 'b': 2}},
            'second': {'value': {'b': 3}},
        },
    )
    return tmp_path, write, owner, catalog


def api():
    from robot_experiments.composition import materialize_resource
    from robot_experiments.composition import resource_sources

    return materialize_resource, resource_sources


def ref(group):
    return {'package': 'robot_experiments', 'path': 'recipes.yaml', 'group': group}


def test_plain_resources_keep_existing_resolution(resources):
    materialize, sources = api()
    root, _, owner, _ = resources
    assert (
        materialize(
            {'package': 'robot_description', 'path': 'robot.yaml'}, root, lambda p: root / p
        )
        == owner
    )
    assert materialize(str(owner), root, lambda p: root / p) == owner
    assert sources(owner) == [{'path': str(owner)}]


def test_recipe_overrides_owner_without_modifying_sources_and_tracks_ancestry(resources):
    materialize, sources = api()
    root, _, owner, catalog = resources
    original = owner.read_bytes(), catalog.read_bytes()
    generated = materialize(ref('again'), root, lambda p: root / p)
    assert not generated.is_relative_to(root)
    assert load_yaml(generated) == {
        'robot': {'joints': {'wrist': {'lower': -0.25, 'upper': 0.25}}, 'velocity': [1.0, 1.2]}
    }
    assert (owner.read_bytes(), catalog.read_bytes()) == original
    assert sources(generated) == [
        {'path': str(catalog), 'group': 'again'},
        {'path': str(catalog), 'group': 'physical'},
        {'path': str(owner)},
    ]
    detached = sources(generated)
    detached.clear()
    assert sources(generated)


def test_group_lists_use_existing_ordered_merge(resources):
    materialize, _ = api()
    root, _, _, _ = resources
    result = materialize(ref(['first', 'second']), root, lambda p: root / p)
    assert load_yaml(result) == {'value': {'a': 1, 'b': 3}}


def test_nested_deployment_references_become_plain_absolute_paths(resources):
    materialize, sources = api()
    root, write, _, _ = resources
    write(
        'robot_experiments',
        'deployment.yaml',
        {
            'robot': {'description': ref('physical')},
            'sensor': {'package': 'robot_description', 'path': 'robot.yaml'},
        },
    )
    result = materialize(
        {'package': 'robot_experiments', 'path': 'deployment.yaml', 'group': 'robot'},
        root,
        lambda p: root / p,
    )
    description = Path(load_yaml(result)['description'])
    assert description.is_absolute() and description.is_file()
    assert load_yaml(description)['robot']['joints']['wrist']['upper'] == 0.25
    assert any(record.get('group') == 'physical' for record in sources(result))


def test_system_preserves_owner_groups_but_expands_experiment_groups(resources):
    materialize, _ = api()
    root, write, _, _ = resources
    owner = {'package': 'robot_estimation', 'path': 'estimation.yaml', 'group': ['robot', 'pose']}
    document = {
        'schema_version': 2,
        'robot': {'description': ref('physical')},
        'estimation': {'robot': {'config': owner}},
    }
    original = deepcopy(document)
    write('robot_experiments', 'systems.yaml', {'nominal': document})
    result = materialize(
        {'package': 'robot_experiments', 'path': 'systems.yaml', 'group': 'nominal'},
        root,
        lambda p: root / p,
    )
    actual = load_yaml(result)
    assert actual['estimation']['robot']['config'] == owner
    assert Path(actual['robot']['description']).is_file()
    assert document == original


@pytest.mark.parametrize(
    'recipe,match',
    [
        ({'base': ref('physical'), 'overrides': {}, 'ignored': 1}, 'recipe'),
        ({'overrides': {}}, 'recipe'),
        ({'base': ref('physical')}, 'recipe'),
        ({'base': ref('physical'), 'overrides': {'robot': {'unknown': 1}}}, 'unknown'),
        ({'base': ref('physical'), 'overrides': {'robot': {'velocity': float('nan')}}}, 'invalid'),
        (
            {
                'base': ref('physical'),
                'overrides': {'robot': {'joints': {'wrist': {'upper': True}}}},
            },
            'invalid',
        ),
    ],
)
def test_recipe_rejects_ignored_or_invalid_overrides(resources, recipe, match):
    materialize, _ = api()
    root, write, _, _ = resources
    write('robot_experiments', 'invalid.yaml', {'bad': recipe})
    with pytest.raises(ValueError, match=match):
        materialize(
            {'package': 'robot_experiments', 'path': 'invalid.yaml', 'group': 'bad'},
            root,
            lambda p: root / p,
        )


def test_resource_recipe_cycles_fail_without_recursion_error(resources):
    materialize, _ = api()
    root, write, _, _ = resources
    write(
        'robot_experiments',
        'cycle.yaml',
        {
            'a': {
                'base': {'package': 'robot_experiments', 'path': 'cycle.yaml', 'group': 'b'},
                'overrides': {},
            },
            'b': {
                'base': {'package': 'robot_experiments', 'path': 'cycle.yaml', 'group': 'a'},
                'overrides': {},
            },
        },
    )
    with pytest.raises(ValueError, match='cycle'):
        materialize(
            {'package': 'robot_experiments', 'path': 'cycle.yaml', 'group': 'a'},
            root,
            lambda p: root / p,
        )


def test_resource_rejects_unknown_reference_keys(resources):
    materialize, _ = api()
    root, _, _, _ = resources
    with pytest.raises(ValueError, match='Resource|resource'):
        materialize({**ref('physical'), 'import': 'arbitrary.code'}, root, lambda p: root / p)


def test_owner_group_reference_materializes_without_an_experiment_recipe(resources):
    materialize, sources = api()
    root, write, _, _ = resources
    owner = write(
        'robot_simulation',
        'sensors.yaml',
        {'baseline': {'localization_sensor': {'white_noise_std': [0.04, 0.04, 0.03]}}},
    )
    selected = materialize(
        {'package': 'robot_simulation', 'path': 'sensors.yaml', 'group': 'baseline'},
        root,
        lambda p: root / p,
    )
    assert load_yaml(selected) == {'localization_sensor': {'white_noise_std': [0.04, 0.04, 0.03]}}
    assert sources(selected) == [{'path': str(owner), 'group': 'baseline'}]


def test_deployment_expands_plain_explicit_resources(resources):
    materialize, sources = api()
    root, write, owner, _ = resources
    write(
        'robot_experiments',
        'deployment.yaml',
        {'selected': {'description': {'package': 'robot_description', 'path': 'robot.yaml'}}},
    )
    selected = materialize(
        {'package': 'robot_experiments', 'path': 'deployment.yaml', 'group': 'selected'},
        root,
        lambda p: root / p,
    )
    assert load_yaml(selected)['description'] == str(owner)
    assert {'path': str(owner)} in sources(selected)


def test_recipe_resource_overrides_expand_before_strict_leaf_override(resources):
    materialize, sources = api()
    root, write, _, _ = resources
    write('robot_simulation', 'deployment.yaml', {'robot': {'config': 'nominal.yaml'}})
    write(
        'robot_experiments',
        'deployment.yaml',
        {
            'selected': {
                'base': {'package': 'robot_simulation', 'path': 'deployment.yaml'},
                'overrides': {'robot': {'config': ref('physical')}},
            }
        },
    )
    result = materialize(
        {'package': 'robot_experiments', 'path': 'deployment.yaml', 'group': 'selected'},
        root,
        lambda p: root / p,
    )
    physical = Path(load_yaml(result)['robot']['config'])
    assert load_yaml(physical)['robot']['joints']['wrist']['upper'] == 0.25
    assert any(record.get('group') == 'physical' for record in sources(result))


@pytest.mark.parametrize('current', [{}, {'type': 'old', 'parameters': {'old_gain': 1.0}}])
def test_recipe_selection_replaces_typed_slot_atomically_then_applies_overrides(
    resources, current
):
    materialize, _ = api()
    root, write, _, _ = resources
    write('robot_simulation', 'target.yaml', {'target': {'dynamics': current}})
    write(
        'robot_experiments',
        'selection.yaml',
        {
            'selected': {
                'base': {'package': 'robot_simulation', 'path': 'target.yaml'},
                'selection': {'target.dynamics': {'type': 'new', 'parameters': {'new_gain': 2.0}}},
                'overrides': {'target': {'dynamics': {'parameters': {'new_gain': 3.0}}}},
            }
        },
    )
    selected = materialize(
        {'package': 'robot_experiments', 'path': 'selection.yaml', 'group': 'selected'},
        root,
        lambda p: root / p,
    )
    assert load_yaml(selected) == {
        'target': {'dynamics': {'type': 'new', 'parameters': {'new_gain': 3.0}}}
    }


@pytest.mark.parametrize(
    'selection',
    [
        {'target.unknown': {'type': 'new', 'parameters': {}}},
        {'unknown.dynamics': {'type': 'new', 'parameters': {}}},
        {'target..dynamics': {'type': 'new', 'parameters': {}}},
        {'target.dynamics': {'type': '', 'parameters': {}}},
        {'target.dynamics': {'type': 'new', 'parameters': []}},
        {'target.dynamics': {'type': 'new', 'parameters': {}, 'extra': 1}},
        {'target.untyped': {'type': 'new', 'parameters': {}}},
        {'target.scalar': {'type': 'new', 'parameters': {}}},
        [],
    ],
)
def test_recipe_rejects_invalid_or_unknown_component_selection(resources, selection):
    materialize, _ = api()
    root, write, _, _ = resources
    write(
        'robot_simulation',
        'target.yaml',
        {'target': {'dynamics': {}, 'untyped': {'lower': -1.0}, 'scalar': 3.0}},
    )
    write(
        'robot_experiments',
        'selection.yaml',
        {
            'selected': {
                'base': {'package': 'robot_simulation', 'path': 'target.yaml'},
                'selection': selection,
                'overrides': {},
            }
        },
    )
    with pytest.raises(ValueError, match='selection'):
        materialize(
            {'package': 'robot_experiments', 'path': 'selection.yaml', 'group': 'selected'},
            root,
            lambda p: root / p,
        )


@pytest.mark.parametrize('omitted', [['optional'], ['nested.optional']])
def test_recipe_omits_only_explicit_existing_optional_fields(resources, omitted):
    materialize, _ = api()
    root, write, _, _ = resources
    write('robot_simulation', 'deployment.yaml', {'optional': 1, 'nested': {'optional': 2}})
    write(
        'robot_experiments',
        'omit.yaml',
        {
            'selected': {
                'base': {'package': 'robot_simulation', 'path': 'deployment.yaml'},
                'overrides': {},
                'omit': omitted,
            }
        },
    )
    result = load_yaml(
        materialize(
            {'package': 'robot_experiments', 'path': 'omit.yaml', 'group': 'selected'},
            root,
            lambda p: root / p,
        )
    )
    if omitted == ['optional']:
        assert result == {'nested': {'optional': 2}}
    else:
        assert result == {'optional': 1, 'nested': {}}


@pytest.mark.parametrize(
    'omitted', [['unknown'], ['nested.unknown'], ['optional', 'optional'], [True], 'optional']
)
def test_recipe_rejects_unknown_or_invalid_omitted_fields(resources, omitted):
    materialize, _ = api()
    root, write, _, _ = resources
    write('robot_simulation', 'deployment.yaml', {'optional': 1, 'nested': {}})
    write(
        'robot_experiments',
        'omit.yaml',
        {
            'selected': {
                'base': {'package': 'robot_simulation', 'path': 'deployment.yaml'},
                'overrides': {},
                'omit': omitted,
            }
        },
    )
    with pytest.raises(ValueError, match='omit'):
        materialize(
            {'package': 'robot_experiments', 'path': 'omit.yaml', 'group': 'selected'},
            root,
            lambda p: root / p,
        )


def test_inline_system_recipe_preserves_base_and_override_reference_origins(tmp_path):
    from robot_experiments.composition import materialize_resource

    parent = tmp_path / 'owner'
    caller = tmp_path / 'caller'
    parent.mkdir()
    caller.mkdir()
    (parent / 'prediction.yaml').write_text('prediction: {}\n')
    (caller / 'control.yaml').write_text('control: {}\n')
    source = parent / 'system.yaml'
    source.write_text(
        yaml.safe_dump({'schema_version': 2, 'prediction': {'config': 'prediction.yaml'}})
    )
    result = materialize_resource(
        {'base': str(source), 'overrides': {'control': {'config': 'control.yaml'}}},
        caller,
        lambda p: tmp_path / p,
    )
    actual = load_yaml(result)
    assert actual['prediction']['config'] == str(parent / 'prediction.yaml')
    assert actual['control']['config'] == str(caller / 'control.yaml')


def test_generated_resource_rejects_changed_source_before_snapshotting(resources):
    from robot_experiments.composition import materialize_resource
    from robot_experiments.composition import resource_sources

    root, _, owner, _ = resources
    result = materialize_resource(ref('physical'), root, lambda p: root / p)
    owner.write_text(owner.read_text() + '\n# edited after preparation\n')
    with pytest.raises(ValueError, match='changed since preparation'):
        resource_sources(result)


def test_inherited_group_preserves_reference_origin_and_parent_fingerprint(tmp_path):
    from robot_experiments.composition import materialize_resource
    from robot_experiments.composition import resource_sources

    parent = tmp_path / 'parent'
    child = tmp_path / 'child'
    parent.mkdir()
    child.mkdir()
    (parent / 'prediction.yaml').write_text('prediction: {}\n')
    source = parent / 'catalog.yaml'
    source.write_text(
        yaml.safe_dump(
            {'selected': {'schema_version': 2, 'prediction': {'config': 'prediction.yaml'}}}
        )
    )
    catalog = child / 'catalog.yaml'
    catalog.write_text('extends: ../parent/catalog.yaml\n')
    result = materialize_resource(
        {'package': 'child', 'path': 'catalog.yaml', 'group': 'selected'},
        tmp_path,
        lambda p: tmp_path / p,
    )
    assert load_yaml(result)['prediction']['config'] == str(parent / 'prediction.yaml')
    assert any(r['path'] == str(source) for r in resource_sources(result))
    source.write_text(source.read_text() + '\n# changed parent\n')
    with pytest.raises(ValueError, match='changed since preparation'):
        resource_sources(result)


def test_inline_owner_recipe_captures_inherited_parent_fingerprint(tmp_path):
    from robot_experiments.composition import materialize_resource, resource_sources
    parent = tmp_path / 'parent.yaml'
    parent.write_text('value: 1\n')
    child = tmp_path / 'child.yaml'
    child.write_text('extends: parent.yaml\n')
    result = materialize_resource({'base': str(child), 'overrides': {'value': 2}},
                                  tmp_path, lambda p: tmp_path / p)
    parent.write_text('value: 3\n')
    with pytest.raises(ValueError, match='changed since preparation'):
        resource_sources(result)


def test_inherited_recipe_base_uses_its_declaring_directory(tmp_path):
    from robot_experiments.composition import materialize_resource
    parent = tmp_path / 'parent'
    parent.mkdir()
    (parent / 'owner.yaml').write_text('value: 1\n')
    (parent / 'groups.yaml').write_text(yaml.safe_dump({'selected': {
        'base': 'owner.yaml', 'overrides': {'value': 2}}}))
    child = tmp_path / 'child'
    child.mkdir()
    (child / 'groups.yaml').write_text('extends: ../parent/groups.yaml\n')
    result = materialize_resource({'package': 'child', 'path': 'groups.yaml',
                                   'group': 'selected'}, tmp_path, lambda p: tmp_path / p)
    assert load_yaml(result) == {'value': 2}


def test_inline_system_owner_replacement_clears_inherited_group(tmp_path):
    from robot_experiments.composition import materialize_resource
    owner = tmp_path / 'standalone.yaml'
    owner.write_text('estimation_node: {ros__parameters: {}}\n')
    source = tmp_path / 'system.yaml'
    source.write_text(yaml.safe_dump({'schema_version': 2, 'estimation': {'robot': {
        'config': {'package': 'test', 'path': 'grouped.yaml', 'group': 'robot_ekf'}}}}))
    result = materialize_resource({'base': str(source), 'overrides': {
        'estimation': {'robot': {'config': {'package': 'test', 'path': 'standalone.yaml'}}}}},
        tmp_path, lambda p: tmp_path)
    actual = load_yaml(result)['estimation']['robot']
    assert actual['config'] == str(owner)
    assert actual['config_group'] is None
