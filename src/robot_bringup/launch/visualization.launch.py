"""Read-only spatial display around the existing closed-loop deployment."""
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_bringup.factory.system_configuration import load_system_configuration
from robot_bringup.visualization import validate_configuration
from robot_bringup.visualization.rviz import configured_rviz
from robot_configuration.resources import resolve_resource
from robot_configuration.yaml_loader import load_yaml


def has_display(environment=None):
    """Use the caller's display configuration; never fabricate WSL settings."""
    environment=os.environ if environment is None else environment
    return bool(environment.get('DISPLAY') or environment.get('WAYLAND_DISPLAY'))


def configure(context):
    """Preflight resources, then include exactly one physics/control deployment."""
    share=lambda package:Path(get_package_share_directory(package))
    values={key:LaunchConfiguration(key).perform(context) for key in
            ['system_config','visualization_config','gui','rviz','enable_actuation']}
    for key in ['gui','rviz','enable_actuation']:
        if values[key] not in ['true','false']:
            raise ValueError(f'{key} requires true or false')
    system=resolve_resource(values['system_config'],share('robot_bringup'),share)
    visual=resolve_resource(values['visualization_config'],share('robot_bringup'),share)
    config=validate_configuration(load_yaml(visual))
    plan=load_system_configuration(system,share,overrides={
        'enable_actuation':values['enable_actuation'],'publish_ground_truth':'true',
        'visualize_tracking':'true','gui':'false'})
    allowed={plan['estimation_parameters']['robot_state_frame_id']}
    simulation=load_yaml(plan['profile']['simulation_config'])
    resource_base=(share('robot_simulation') if simulation['paths']['base']=='package'
                   else Path(plan['profile']['simulation_config']).parent)
    robot=load_yaml(resource_base/simulation['robot']['config'])['robot']
    allowed.add(simulation['tracking']['reference_frame'])
    if config['frame_id'] not in allowed:
        raise ValueError(f'visualization.frame_id={config["frame_id"]!r}: supported static frames {sorted(allowed)}')
    robot_package=share(robot['description']['package'])
    xml=subprocess.run(['xacro',str(robot_package/robot['description']['xacro']),
                         f'config_file:={plan["description_path"]}'],check=True,
                        capture_output=True,text=True).stdout
    actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share('robot_bringup')/'launch/closed_loop.launch.py')),
        launch_arguments={'system_config':str(system),'visualization_config':str(visual),
          'gui':'false','enable_actuation':values['enable_actuation'],
          'publish_ground_truth':'true','visualize_tracking':'true'}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             name='visual_robot_state_publisher',namespace='visualization',
             parameters=[{'use_sim_time':True,'robot_description':xml,
                          'frame_prefix':config['model_frame_prefix']}],
             remappings=[('robot_description','/visualization/robot_description'),
                         ('joint_states',robot['joint_states']['topic'])])]
    if values['gui']=='true':
        if has_display():
            # GUI-only client: its failure cannot stop the independent server.
            actions.append(ExecuteProcess(cmd=['gz','sim','-g'],output='screen'))
        else:
            actions.append(LogInfo(msg='Gazebo GUI requested but no DISPLAY/WAYLAND_DISPLAY is configured; server remains headless.'))
    if values['rviz']=='true':
        if not has_display():
            actions.append(LogInfo(msg='RViz requested but no display is configured; marker/path publication remains active.'))
        else:
            try:
                share('rviz2')
            except LookupError:
                actions.append(LogInfo(msg='RViz package is unavailable; marker/path publication remains active.'))
            else:
                rviz_configuration=configured_rviz(load_yaml(
                    share('robot_bringup')/'config/tracking_scene.rviz'),config)
                rviz_path=Path(tempfile.mkdtemp(prefix='robot_visualization_'))/'scene.rviz'
                rviz_path.write_text(yaml.safe_dump(rviz_configuration,sort_keys=False))
                actions.append(Node(package='rviz2',executable='rviz2',
                  arguments=['-d',str(rviz_path)],
                  parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    """Offer GUI clients separately from explicit physical actuation."""
    return LaunchDescription([
        DeclareLaunchArgument('system_config',default_value='',description='Required explicit system manifest'),
        DeclareLaunchArgument('visualization_config',default_value='config/tracking_visualization.yaml'),
        DeclareLaunchArgument('gui',default_value='false',description='Optional Gazebo GUI-only client'),
        DeclareLaunchArgument('rviz',default_value='false',description='Optional RViz client'),
        DeclareLaunchArgument('enable_actuation',default_value='false',description='Explicit opt-in physical commands'),
        OpaqueFunction(function=configure)])
