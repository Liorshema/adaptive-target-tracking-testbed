"""Launch an explicitly selected, validated tracking composition."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from robot_bringup.deployment.launching import configure


def generate_launch_description():
    """Keep Compute Only and the separate GUI workflow as launch defaults."""
    return LaunchDescription([
        DeclareLaunchArgument('system_config', default_value='',
                              description='Required explicit composition manifest'),
        DeclareLaunchArgument('config', default_value='',
                              description='External schema-1 profile override only'),
        DeclareLaunchArgument('gui', default_value='', description='Default false'),
        DeclareLaunchArgument('enable_actuation', default_value='', description='Default false'),
        DeclareLaunchArgument('visualize_tracking', default_value='',
                              description='Publish measured frame axes and optical/target arrows'),
        DeclareLaunchArgument('visualization_config', default_value='',
                              description='Optional read-only visualization YAML; empty uses installed default'),
        DeclareLaunchArgument('publish_ground_truth', default_value='',
                              description=(
                                  'Optional physical pose output independent of RViz markers')),
        DeclareLaunchArgument('publish_commands', default_value='',
                              description='Optional disable override for watchdog experiments'),
        DeclareLaunchArgument('strategy_config', default_value='',
                              description='Control strategy YAML; empty retains v1 DLS'),
        DeclareLaunchArgument('lookahead', default_value='',
                              description='Override strategy lookahead in simulation seconds'),
        DeclareLaunchArgument('feedforward', default_value='',
                              description='Override strategy target feedforward: true or false'),
        OpaqueFunction(function=configure),
    ])
