"""Check copied M09 resources and launch construction outside the source checkout."""
import argparse
from importlib import import_module,util
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from m08_verify_installation import run as verify_m08
from robot_configuration.yaml_loader import load_yaml


def run(prefix):
    result=verify_m08(prefix)
    share=lambda package:Path(get_package_share_directory(package))
    for name in ['configuration','geometry','scene','rviz']:
        module=import_module('robot_bringup.visualization.'+name)
        assert Path(module.__file__).resolve().is_relative_to(prefix)
    for name in ['joint_diagnostics','target_diagnostics']:
        module=import_module('robot_experiments.metrics.'+name)
        assert Path(module.__file__).resolve().is_relative_to(prefix)
    from robot_bringup.visualization import validate_configuration
    from robot_bringup.visualization.rviz import configured_rviz
    root=share('robot_bringup')
    config=validate_configuration(load_yaml(root/'config/tracking_visualization.yaml'))
    rviz=configured_rviz(load_yaml(root/'config/tracking_scene.rviz'),config)
    robot=next(d for d in rviz['Visualization Manager']['Displays'] if d['Class'].endswith('/RobotModel'))
    assert robot['TF Prefix']==config['model_frame_prefix'].rstrip('/')
    displays=rviz['Visualization Manager']['Displays']
    assert next(d for d in displays if d['Class'].endswith('/TF'))['Enabled'] is False
    assert 5 <= config['history_seconds'] <= 10
    assert config['layers']['camera_axis'] and config['layers']['target_truth_current']
    assert config['layers']['target_estimate_current']
    for display in displays:
        if display['Class'].endswith('/Path'):
            layer=display['Name'].lower().replace(' ','_')
            assert display['Line Style']=='Billboards'
            assert display['Line Width']==config['path_widths'][layer]
            assert display['Enabled']==config['layers'][layer]
    path=root/'launch/visualization.launch.py'
    spec=util.spec_from_file_location('installed_visualization_launch',path)
    module=util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext();context.launch_configurations.update(system_config='config/system_nominal.yaml',visualization_config='config/tracking_visualization.yaml',gui='false',rviz='false',enable_actuation='false')
    actions=module.configure(context)
    assert sum(isinstance(action,IncludeLaunchDescription) for action in actions)==1
    assert sum(isinstance(action,Node) for action in actions)==1
    result['visualization']={'launch':str(path),'model_tf_prefix':robot['TF Prefix'],
      'paths':5,'frame':config['frame_id'],'modules_from_copied_install':True,
      'full_tf_display_default':False,'history_seconds':config['history_seconds'],
      'secondary_histories_default':False,'metric_billboard_widths':True,
      'single_existing_closed_loop':True,'resources':[str(root/'config/tracking_visualization.yaml'),str(root/'config/tracking_scene.rviz')]}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prefix',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=run(args.prefix.resolve());args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('Verified ten copied packages, eight console entries, M07/M08 contracts, M09 modules/resources and one read-only visualization launch wrapper.')
