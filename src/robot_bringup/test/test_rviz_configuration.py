"""RViz follows the read-only adapter's configured frames and topics."""
from pathlib import Path
from robot_configuration.yaml_loader import load_yaml
from robot_bringup.visualization.rviz import configured_rviz


def test_model_tf_prefix_and_custom_topics_match_display_publication():
 root=Path(__file__).resolve().parents[1]
 template=load_yaml(root/'config/tracking_scene.rviz')
 result=configured_rviz(template,{'frame_id':'odom','model_frame_prefix':'inspection/',
 'topic':'/inspection/markers','paths_topic':'/inspection/paths','enabled':True,
 'layers':{'target_prediction':False}})
 manager=result['Visualization Manager'];displays=manager['Displays']
 model=next(d for d in displays if d['Class'].endswith('/RobotModel'))
 assert model['TF Prefix']=='inspection'
 assert manager['Global Options']['Fixed Frame']=='odom'
 assert next(d for d in displays if d['Class'].endswith('/MarkerArray'))['Marker Topic']['Value']=='/inspection/markers'
 prediction=next(d for d in displays if d['Name']=='Target Prediction')
 assert prediction['Topic']['Value']=='/inspection/paths/target_prediction'
 assert prediction['Enabled'] is False
 assert next(d for d in template['Visualization Manager']['Displays'] if d['Class'].endswith('/RobotModel'))['TF Prefix']=='visual'


def test_default_and_configured_rviz_hierarchy_widths_and_view():
 root=Path(__file__).resolve().parents[1]
 config=load_yaml(root/'config/tracking_visualization.yaml')
 template=load_yaml(root/'config/tracking_scene.rviz')
 result=configured_rviz(template,config)
 displays=result['Visualization Manager']['Displays']
 assert next(d for d in displays if d['Class'].endswith('/TF'))['Enabled'] is False
 assert next(d for d in displays if d['Name']=='Robot Truth')['Enabled'] is False
 assert next(d for d in displays if d['Name']=='Robot Estimate')['Enabled'] is False
 for d in displays:
  if d['Class'].endswith('/Path'):
   key=d['Name'].lower().replace(' ','_')
   assert d['Line Style']=='Billboards'
   assert d['Line Width']==config['path_widths'][key]
 assert 5<=config['history_seconds']<=10
 custom=configured_rviz(template,{'show_all_tf':True,'view':{'distance':2.,'focal_point':[3.,4.,5.]}})
 assert next(d for d in custom['Visualization Manager']['Displays'] if d['Class'].endswith('/TF'))['Enabled'] is True
 assert custom['Visualization Manager']['Views']['Current']['Distance']==2.
 assert custom['Visualization Manager']['Views']['Current']['Focal Point']==dict(X=3.,Y=4.,Z=5.)
