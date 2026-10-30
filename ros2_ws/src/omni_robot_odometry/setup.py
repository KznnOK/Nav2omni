from setuptools import find_packages, setup


package_name = "omni_robot_odometry"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Odometry from passive tracking wheels and the IMU yaw rate",
    license="TODO",
    entry_points={
        "console_scripts": [
            "tracking_odometry = "
            "omni_robot_odometry.tracking_odometry:main",
        ],
    },
)
