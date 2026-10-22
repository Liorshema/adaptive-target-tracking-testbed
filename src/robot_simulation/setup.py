
"""ROS 2 package setup for the simulation subsystem."""

import os
from pathlib import Path
import subprocess
from glob import glob

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext


package_name = "robot_simulation"


class GazeboBuild(build_ext):
    """Build a native Gazebo System inside the existing ament_python distribution."""

    def build_extension(self, extension):
        """Use vendor CMake targets; retain Python resources and console entry points."""
        output = Path(self.get_ext_fullpath(extension.name)).resolve()
        recovery = extension.name.endswith('._dartsim_recovery')
        build = Path(self.build_temp).resolve() / ('gazebo_recovery' if recovery
                                                  else 'gazebo_actuator')
        source = Path(__file__).resolve().parent / package_name / 'gazebo/native'
        prefix = 'RECOVERY' if recovery else 'ACTUATOR'
        subprocess.run(['cmake', '-S', str(source), '-B', str(build),
                        '-DCMAKE_BUILD_TYPE=RelWithDebInfo',
                        f'-D{prefix}_OUTPUT_DIRECTORY={output.parent}',
                        f'-D{prefix}_OUTPUT_NAME={output.name}'], check=True)
        target = 'robot_dartsim_recovery' if recovery else 'robot_actuator'
        subprocess.run(['cmake', '--build', str(build), '--target', target,
                        '--parallel', '2'], check=True)


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
            if not directory.startswith(".") and directory != "__pycache__"
        )

        files = [
            os.path.join(root, filename)
            for filename in sorted(filenames)
            if not filename.startswith(".")
            and not filename.endswith((".bak", ".backup", ".orig", ".rej", ".pyc", ".pyo"))
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
    ext_modules=[Extension('robot_simulation.gazebo._actuator', sources=[]),
                 Extension('robot_simulation.gazebo._dartsim_recovery', sources=[])],
    cmdclass={'build_ext': GazeboBuild},

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "third_party", "dartsim_recovery"),
            [os.path.join(package_name, "gazebo", "native", "dartsim_recovery", name)
             for name in ("LICENSE", "UPSTREAM.json", "README.md")],
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
        *install_directory_tree("config/gazebo"),
        *install_directory_tree("config/scenarios"),
        *install_directory_tree("config/sensors"),
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
            "localization_sensor = robot_simulation.nodes.localization_sensor:main",
            "velocity_actuation = robot_simulation.nodes.velocity_actuation:main",
        ],
    },
)
