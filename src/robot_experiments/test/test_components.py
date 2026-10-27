"""A headless discovery client consumes owner contracts without algorithm branches."""
import json


def test_component_inventory_and_detached_description():
    from robot_experiments.components import list_components, describe_component, validate_component
    inventory = list_components()
    assert {'dynamics', 'robot_estimator', 'target_estimator', 'predictor', 'adaptation',
            'fusion', 'controller', 'reference_policy', 'reference_geometry', 'task'} <= inventory.keys()
    imm = describe_component('predictor', 'imm')
    assert imm['capabilities']['runtime_validation'] == 'conditional'
    assert describe_component('predictor', 'coordinated_turn')['capabilities']['deployable'] is False
    description = describe_component('dynamics', 'fight_flight')
    description['defaults']['alpha'] = 90.
    assert describe_component('dynamics', 'fight_flight')['defaults']['alpha'] == 1.
    check = validate_component('controller', 'weighted_least_squares', {'damping': -1.})
    assert check['valid'] is False and check['errors']
    assert validate_component('predictor', 'imm', context={
        'state_layout': 'position_velocity', 'spatial_dimension': 1})['valid'] is False


def test_component_cli_returns_structured_metadata(capsys):
    from robot_experiments.cli import main
    assert main(['components', 'show', 'predictor', 'constant_velocity']) == 0
    metadata = json.loads(capsys.readouterr().out)
    assert metadata['id'] == 'constant_velocity'
    assert metadata['schema']['type'] == 'object'
