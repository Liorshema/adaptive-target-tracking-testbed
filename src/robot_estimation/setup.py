from setuptools import find_packages, setup


package_name = "robot_estimation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/robot_estimator.yaml",
                "config/robot_localized.yaml",
                "config/target_estimator.yaml",
            ],
        ),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "PyYAML",
    ],
    zip_safe=True,
    maintainer="Lior",
    maintainer_email="lior@example.com",
    description=(
        "State estimation package for the Adaptive Target Tracking Testbed."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
    "console_scripts": [
        "estimation_node = robot_estimation.nodes.estimation_node:main",
    ],
},
)