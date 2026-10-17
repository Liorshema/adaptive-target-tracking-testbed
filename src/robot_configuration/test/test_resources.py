"""Resource paths and inherited document transforms remain generic and portable."""

from pathlib import Path

import pytest

from robot_configuration.yaml_loader import load_yaml


def test_transform_resolves_inherited_values_at_declaring_document(tmp_path):
    (tmp_path / 'parent').mkdir()
    (tmp_path / 'parent/base.yaml').write_text('resource: data.yaml\nvalue: 1\n')
    child = tmp_path / 'child.yaml'
    child.write_text('extends: parent/base.yaml\nvalue: 2\n')

    def transform(data, source):
        if 'resource' in data:
            data['resource'] = str(source.parent / data['resource'])
        return data

    result = load_yaml(child, transform=transform)
    assert result == {'resource': str(tmp_path / 'parent/data.yaml'), 'value': 2}


def test_resource_resolver_handles_relative_and_package_files(tmp_path):
    from robot_configuration.resources import resolve_resource
    (tmp_path / 'local.yaml').touch()
    assert resolve_resource('local.yaml', tmp_path, lambda _: None) == tmp_path / 'local.yaml'
    assert resolve_resource({'package': 'example', 'path': 'local.yaml'}, tmp_path,
                            lambda _: tmp_path) == tmp_path / 'local.yaml'


def test_package_resources_support_colcon_symlink_install(tmp_path):
    from robot_configuration.resources import resolve_resource
    source = tmp_path / 'source'; source.mkdir()
    (source / 'config.yaml').write_text('value: 1\n')
    share = tmp_path / 'installed_share'; share.mkdir()
    (share / 'config.yaml').symlink_to(source / 'config.yaml')
    assert resolve_resource({'package': 'example', 'path': 'config.yaml'}, tmp_path,
                            lambda _: share) == source / 'config.yaml'


@pytest.mark.parametrize('reference', [
    {'package': 'example', 'path': '../outside.yaml'},
    {'package': 'example', 'path': '/tmp/outside.yaml'},
    {'package': 'example', 'patth': 'file.yaml'}, '', [], True,
])
def test_resource_resolver_rejects_ambiguous_or_escaping_references(tmp_path, reference):
    from robot_configuration.resources import resolve_resource
    with pytest.raises(ValueError):
        resolve_resource(reference, tmp_path, lambda _: tmp_path)
