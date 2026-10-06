from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_prediction'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
        'numpy',
        'pyyaml',
    ],

    zip_safe=True,

    maintainer='user',
    maintainer_email='liorshema@gmail.com',

    description=(
        'Target-motion prediction package for the '
        'Adaptive Target Tracking Testbed.'
    ),

    license='MIT',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'prediction_node = robot_prediction.prediction_node:main',
        ],
    },
)
