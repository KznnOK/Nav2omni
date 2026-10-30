import os
from glob import glob

from setuptools import find_packages, setup


package_name = "omni_robot_gazebo"


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
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "models",
                "harurobo_field",
            ),
            glob("models/harurobo_field/*.*"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "models",
                "harurobo_field",
                "meshes",
            ),
            glob("models/harurobo_field/meshes/*"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "models",
                "kansai_field_2027",
            ),
            glob("models/kansai_field_2027/*.*"),
        ),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Gazebo simulation and drive control for the omni robot",
    license="TODO",
    entry_points={
    "console_scripts": [
        "omni_drive_controller = "
        "omni_robot_gazebo.omni_drive_controller:main",

        "omni_odometry = "
        "omni_robot_gazebo.omni_odometry:main",

        "imu_covariance = "
        "omni_robot_gazebo.imu_covariance:main",

        "generate_field = "
        "omni_robot_gazebo.field_layout:main",

        "scan_visibility_check = "
        "omni_robot_gazebo.scan_visibility_check:main",
        ],
    },
)
