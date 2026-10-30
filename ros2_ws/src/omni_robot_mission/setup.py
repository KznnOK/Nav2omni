import os
from glob import glob

from setuptools import find_packages, setup


package_name = "omni_robot_mission"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
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
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Mode manager and sakura round-trip mission",
    license="TODO",
    entry_points={
        "console_scripts": [
            "mode_manager = omni_robot_mission.mode_manager:main",
            "sakura_mission = omni_robot_mission.mission_node:main",
        ],
    },
)
