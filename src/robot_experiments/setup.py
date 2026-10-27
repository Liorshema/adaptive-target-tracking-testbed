from glob import glob
from setuptools import find_packages, setup

setup(name='robot_experiments', version='0.1.0', packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/robot_experiments']),
 ('share/robot_experiments',['package.xml']),('share/robot_experiments/config',glob('config/*.yaml')),
 ('share/robot_experiments/config/calibration',glob('config/calibration/*.yaml')),
 ('share/robot_experiments/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='user',maintainer_email='liorshema@gmail.com',
 description='Reproducible experiment orchestration, physical metrics and benchmarking.',license='Apache-2.0',
 entry_points={'console_scripts':['run = robot_experiments.cli:main']})
