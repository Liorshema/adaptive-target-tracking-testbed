"""Test-only scoped ROS input remaps around the unchanged production launch."""
import importlib.util
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from robot_configuration.yaml_loader import load_yaml

def generate_launch_description():
    path=Path(get_package_share_directory('robot_bringup'))/'launch/closed_loop.launch.py'
    spec=importlib.util.spec_from_file_location('diagnostic_production_bringup',path)
    production=importlib.util.module_from_spec(spec);spec.loader.exec_module(production)
    original=production.configure
    def configure(context):
        diagnostic=LaunchConfiguration('diagnostic_config').perform(context);cfg=load_yaml(diagnostic)
        assert cfg['diagnostic_only'] is True
        nodes=original(context);result=[]
        for node in nodes:
            if isinstance(node,Node) and node.node_package=='robot_control':
                remaps=[]
                if cfg['mode']['robot_truth']:remaps.append(SetRemap(src=cfg['context']['topics']['robot_estimate'],dst=cfg['robot_output']))
                if cfg['mode']['target_truth']:remaps.append(SetRemap(src=cfg['context']['topics']['prediction'],dst=cfg['prediction_output']))
                result.append(GroupAction([*remaps,node]))
            else:result.append(node)
        result.append(ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name('tracking_loss_oracle.py')),'--config',diagnostic],output='screen'))
        return result
    production.configure=configure
    description=production.generate_launch_description()
    description=LaunchDescription([DeclareLaunchArgument('diagnostic_config'),*description.entities])
    # The argument is supplied on the command line and is available when the
    # normal OpaqueFunction runs; no production action or owner YAML is edited.
    return description
