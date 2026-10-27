"""Run a declared experiment case continuously; use the CLI for scored runs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from robot_bringup.deployment.launching import configure
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_experiments.configuration import resolve_system_resource
from robot_experiments.launching import launch_profile


def select(context):
    """Resolve experiment selection before handing the validated plan to bringup."""
    def share(package):
        return Path(get_package_share_directory(package))

    names = ['gui', 'enable_actuation', 'visualize_tracking', 'publish_ground_truth',
             'publish_commands', 'lookahead', 'feedforward', 'strategy_config']
    overrides = {name: LaunchConfiguration(name).perform(context) for name in names}
    case = LaunchConfiguration('case').perform(context)
    manifest = LaunchConfiguration('system_config').perform(context)
    if manifest and LaunchConfiguration('controller').perform(context):
        raise ValueError(
            'Select controller in the explicit manifest or choose a case/controller pair')
    if manifest:
        path = Path(manifest)
        if not path.is_absolute():
            path = share('robot_experiments') / path
        plan = load_system_configuration(
            resolve_system_resource(path, path.parent, share), share, overrides)
    else:
        plan = launch_profile(case, share,
                              controller=LaunchConfiguration('controller').perform(context),
                              overrides=overrides)
    context.launch_configurations['config'] = ''
    return configure(context, plan=plan)


def generate_launch_description():
    """Keep actuation opt-in and all component choices owned by experiments."""
    return LaunchDescription([
        DeclareLaunchArgument('case', default_value='closed_loop'),
        DeclareLaunchArgument('controller', default_value=''),
        *[DeclareLaunchArgument(name, default_value='') for name in [
            'system_config', 'gui', 'enable_actuation', 'visualize_tracking',
            'visualization_config', 'publish_ground_truth', 'publish_commands',
            'strategy_config', 'lookahead', 'feedforward']],
        OpaqueFunction(function=select),
    ])
