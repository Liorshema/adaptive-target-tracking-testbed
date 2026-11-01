"""Install the optional Dash client independently of the headless experiment stack."""

from setuptools import find_packages
from setuptools import setup

package_name = 'robot_dashboard'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['assets/*.css', 'assets/*.js']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'dash>=3,<4', 'plotly>=5'],
    zip_safe=False,
    maintainer='user',
    maintainer_email='liorshema@gmail.com',
    description='Experiment authoring, execution and recorded-evidence UI.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['dashboard = robot_dashboard.cli:main']},
)
