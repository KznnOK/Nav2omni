from setuptools import find_packages, setup


package_name = "omni_robot_bringup"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/omni_robot_bringup"],
        ),
        (
            "share/omni_robot_bringup",
            ["package.xml"],
        ),
        (
            "share/omni_robot_bringup/launch",
            [
                "launch/simulation.launch.py",
                "launch/experiment.launch.py",
                "launch/sim_check.launch.py",
                "launch/sim_mission.launch.py",
            ],
        ),
        (
            "share/omni_robot_bringup/rviz",
            ["rviz/sim_check.rviz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Bringup package for the omni robot simulation",
    license="TODO",
    entry_points={
        "console_scripts": [],
    },
)
