"""Explicit owner registrations distinguish library capabilities from deployment."""

from copy import deepcopy

from robot_configuration.components import ComponentRegistry
from robot_configuration.components import schema_for
from robot_configuration.resources import package_config_path
from robot_prediction.baseline.constant_acceleration import ConstantAccelerationPredictor
from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
from robot_prediction.baseline.coordinated_turn import CoordinatedTurnPredictor
from robot_prediction.baseline.imm import IMMPredictor
from robot_prediction.configuration.prediction_config import PARAMETER_SCHEMAS
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.fusion.baseline_fusion import BaselineFusion
from robot_prediction.fusion.weighted_correction import WeightedCorrection
from robot_prediction.online_adaptation.classical.residual_adapter import ResidualAdapter
from robot_prediction.online_adaptation.classical.rls_adapter import RLSAdapter


def create_prediction_registries(config=None, configuration_path=None):
    """Build fresh trusted registries from the authoritative owner document."""
    config = config or PredictionConfig(
        configuration_path or package_config_path(__file__, 'robot_prediction', 'prediction.yaml')
    )
    data = config.data
    result = {key: ComponentRegistry() for key in ['predictor', 'adaptation', 'fusion']}

    def imm_builder(**parameters):
        return IMMPredictor(**parameters, **config.get('coordinated_turn'))

    declarations = [
        (
            'predictor',
            'constant_velocity',
            ConstantVelocityPredictor,
            True,
            {'state_layout': 'position_velocity'},
            {'input_layout': 'position_velocity'},
        ),
        (
            'predictor',
            'imm',
            imm_builder,
            True,
            {
                'state_layout': 'position_velocity',
                'spatial_dimension': {'type': 'integer', 'minimum': 2},
            },
            {
                'ensemble_order': ['cv', 'ca', 'ct'],
                'runtime_validation': 'conditional',
                'parameter_dependencies': {
                    'turn_rate_epsilon': 'predictors.coordinated_turn.parameters'
                },
            },
        ),
        (
            'predictor',
            'constant_acceleration',
            ConstantAccelerationPredictor,
            False,
            {'state_layout': 'position_velocity_acceleration', 'deployment': 'library'},
            {'input_layout': 'position_velocity_acceleration'},
        ),
        (
            'predictor',
            'coordinated_turn',
            CoordinatedTurnPredictor,
            False,
            {'state_layout': 'position_velocity_turn_rate', 'deployment': 'library'},
            {'input_layout': 'position_velocity_turn_rate'},
        ),
        (
            'adaptation',
            'residual',
            ResidualAdapter,
            True,
            {},
            {'correction_representation': 'state_bias', 'requires': 'baseline_residual_history'},
        ),
        (
            'adaptation',
            'rls',
            RLSAdapter,
            False,
            {'deployment': 'library'},
            {
                'correction_representation': 'feature_regression',
                'requires': 'feature_and_target_adapter',
            },
        ),
        (
            'fusion',
            'weighted_correction',
            WeightedCorrection,
            True,
            {'correction_representation': 'state_bias'},
            {'output_layout': 'position_velocity'},
        ),
        (
            'fusion',
            'baseline',
            lambda **_: BaselineFusion(),
            False,
            {'deployment': 'library'},
            {
                'input_representation': 'multiple_trajectories',
                'parameter_usage': (
                    'default_weights are caller-supplied fuse() weights, '
                    'not correction weights'
                ),
            },
        ),
    ]
    families = {'predictor': 'predictors', 'adaptation': 'adaptations', 'fusion': 'fusions'}
    for family, name, builder, deployed, requirements, capabilities in declarations:
        defaults = data[families[family]][name]['parameters']
        units = {key: '1' for key in defaults}
        if 'likelihood_variance' in units:
            units['likelihood_variance'] = 'state residual squared norm'
        if 'initial_covariance' in units:
            units['initial_covariance'] = 'feature-dependent covariance'
        if 'turn_rate_epsilon' in units:
            units['turn_rate_epsilon'] = 'rad/s'
        properties = deepcopy(PARAMETER_SCHEMAS[(families[family], name)])
        for key, unit in units.items():
            properties[key]['unit'] = unit
        schema = {
            'type': 'object',
            'properties': properties,
            'required': list(properties),
            'additionalProperties': False,
        }
        result[family].register(
            name,
            builder,
            description=name.replace('_', ' '),
            defaults=defaults,
            schema=schema,
            requirements=requirements,
            capabilities={
                'deployable': deployed,
                'runtime_validation': 'validated_contract' if deployed else 'library_only',
                'output_layout': 'position_velocity',
                'coordinate_frames': 'same fixed frame as input',
                'sampling': {
                    'defaults': config.get('prediction'),
                    'schema': PARAMETER_SCHEMAS[('prediction',)],
                },
                **capabilities,
            },
            dependencies=('input_dimension', 'output_dimension') if name == 'rls' else (),
        )
    for family, requirements in [
        ('adaptation', {}),
        ('fusion', {'correction_representation': 'none'}),
    ]:
        result[family].register(
            'none',
            lambda: None,
            description='Disabled explicitly',
            defaults={},
            schema=schema_for({}),
            requirements=requirements,
            capabilities={'deployable': True, 'correction_representation': 'none'},
        )
    return result
