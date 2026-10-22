"""Check copied M02 native actuator and additive interfaces outside the checkout."""
import argparse
import hashlib
import json
from pathlib import Path

from m09_verify_installation import run as verify_m09


def main():
    """Require the installed binary, owner configuration and historical interfaces."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    result = verify_m09(prefix)
    from ament_index_python.packages import get_package_share_directory
    from robot_configuration.yaml_loader import load_yaml
    from robot_interfaces.msg import ActuationTelemetry
    from robot_simulation.gazebo.robot_builder import actuator_library
    from robot_experiments.metrics import actuator_trace
    binary = Path(actuator_library())
    assert binary.is_relative_to(prefix)
    assert Path(actuator_trace.__file__).resolve().is_relative_to(prefix)
    holding = load_yaml(Path(get_package_share_directory('robot_simulation'))/
                        'config/gazebo/joint_position_hold.yaml')['joint_position_hold']
    assert holding['backend'] == 'velocity_servo'
    fields = ActuationTelemetry.get_fields_and_field_types()
    for key in ['commanded_velocity','measured_positions','accepted_command_valid',
                'accepted_stamp','measured_velocities','position_errors','feedback_valid']:
        assert key in fields
    result['m02_actuator'] = {'binary':str(binary),'sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
                              'backend':holding['backend'],'servo':holding['servo'],
                              'diagnostic_fields':list(fields)}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('Ten copied packages, native motor library, M07/M08/M09 contracts and M02 interfaces verified.')


if __name__ == '__main__':
    main()
