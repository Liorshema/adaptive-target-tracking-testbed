from setuptools import find_packages, setup

setup(
    name='robot_configuration', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/robot_configuration']),
                ('share/robot_configuration', ['package.xml'])],
    install_requires=['setuptools', 'PyYAML'],
    maintainer='user', maintainer_email='liorshema@gmail.com',
    description='Shared YAML mapping and inheritance loader.', license='Apache-2.0',
    extras_require={'test': ['pytest']}, zip_safe=True,
)
