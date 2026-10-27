import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lego_loam_bor")
    config_file = os.path.join(
        package_share, "config", "x30_mid360_mapping.yaml"
    )
    rviz_file = os.path.join(package_share, "rviz", "lego_loam.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    mapping_dir = LaunchConfiguration("mapping_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "mapping_dir",
            default_value="/tmp/",
            description=(
                "Parent directory for saved mapping timestamp folders. "
                "The directory must already exist."
            ),
        ),
        SetEnvironmentVariable("DDDMR_MAPPING_DIR", mapping_dir),
        Node(
            package="lego_loam_bor",
            executable="lego_loam",
            output="screen",
            respawn=False,
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/lslidar_point_cloud", "/lidar_points"),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            respawn=False,
            arguments=["-d", rviz_file],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
