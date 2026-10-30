from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

from omni_robot_description.geometry import footprint_string, load_geometry


def generate_launch_description():
    nav2_package_share = Path(
        get_package_share_directory("omni_robot_nav2")
    )
    nav2_bringup_share = Path(
        get_package_share_directory("nav2_bringup")
    )

    params_file = nav2_package_share / "config" / "nav2_params.yaml"
    # Costmap footprint follows the maximum outline in robot_geometry.yaml.
    configured_params = RewrittenYaml(
        source_file=str(params_file),
        param_rewrites={"footprint": footprint_string(load_geometry())},
        convert_types=True,
    )
    navigation_launch_file = (
        nav2_bringup_share / "launch" / "navigation_launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(navigation_launch_file)
            ),
            launch_arguments={
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": configured_params,
                "use_composition": "False",
                "container_name": "nav2_container",
            }.items(),
        ),
    ])
