from setuptools import find_packages, setup


package_name = "omni_robot_nav2"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/omni_robot_nav2",
            ],
        ),
        (
            "share/omni_robot_nav2",
            [
                "package.xml",
            ],
        ),
        (
            "share/omni_robot_nav2/launch",
            [
                "launch/map_server.launch.py",
                "launch/localization.launch.py",
                "launch/navigation.launch.py",
                "launch/navigate_to_goal.launch.py",
            ],
        ),
        (
            "share/omni_robot_nav2/maps",
            [
                "maps/field_map.png",
                "maps/field_map.yaml",
            ],
        ),
        (
            "share/omni_robot_nav2/config",
            [
                "config/amcl.yaml",
                "config/nav2_params.yaml",
            ],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Nav2 configuration for the four-wheel omni robot",
    license="TODO",
    entry_points={
        "console_scripts": [
            "navigate_to_goal = "
            "omni_robot_nav2.navigate_to_goal:main",
        ],
    },
)
