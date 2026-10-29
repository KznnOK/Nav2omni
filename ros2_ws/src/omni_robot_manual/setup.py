import os
from glob import glob

from setuptools import find_packages, setup


package_name = "omni_robot_manual"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Manual PS4 and keyboard control for the omni robot",
    license="TODO",
    entry_points={
        "console_scripts": [
            "joy_manual_control = omni_robot_manual.joy_manual_control:main",
            "keyboard_manual_control = omni_robot_manual.keyboard_manual_control:main",
            "keyboard_gui_control = omni_robot_manual.keyboard_gui_control:main",
            "manual_cmd_mux = omni_robot_manual.manual_cmd_mux:main",
            "manual_recorder = omni_robot_manual.manual_recorder:main",
        ],
    },
)
