
"""ROS 2 package setup for the simulation subsystem."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "robot_simulation"


def install_files(directory: str, pattern: str) -> list:
    """Register matching files for installation."""
    files = sorted(
        path for path in glob(pattern)
        if os.path.isfile(path)
    )

    if not files:
        return []

    return [
        (
            os.path.join("share", package_name, directory),
            files,
        )
    ]


def install_directory_tree(source_directory: str) -> list:
    """Recursively install files, preserving the directory structure."""
    data_files = []

    for root, directories, filenames in os.walk(source_directory):
        directories[:] = sorted(
            directory for directory in directories
            if not directory.startswith(".")
        )

        files = [
            os.path.join(root, filename)
            for filename in sorted(filenames)
            if not filename.startswith(".")
        ]

        if not files:
            continue

        destination = os.path.join(
            "share",
            package_name,
            root,
        )

        data_files.append((destination, files))

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "test.*"]),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),

        # ROS 2 launch files
        *install_files(
            "launch",
            "launch/*.launch.py",
        ),

        # Gazebo world definitions
        *install_directory_tree("worlds"),

        # Gazebo models, including nested model directories
        *install_directory_tree("models"),

        # Simulation configuration
        *install_files(
            "config",
            "config/*.yaml",
        ),

        # Legacy configuration — retained temporarily
        *install_files(
            "config/comparison",
            "config/comparison/*.yaml",
        ),
        *install_files(
            "config/experiments",
            "config/experiments/*.yaml",
        ),
        *install_files(
            "config/terrain",
            "config/terrain/*.yaml",
        ),
    ],

    install_requires=[
        "setuptools",
        "numpy",
        "PyYAML",
    ],

    zip_safe=True,

    maintainer="user",
    maintainer_email="liorshema@gmail.com",

    description=(
        "Modular 3D target simulation, "
        "behavioral dynamics, disturbances, "
        "synthetic measurements and Gazebo integration."
    ),

    license="Apache-2.0",

    extras_require={
        "test": [
            "pytest",
        ],
    },

    entry_points={
        "console_scripts": [
            "simulation_node = "
            "robot_simulation.nodes.simulation_node:main",
        ],
    },
)
