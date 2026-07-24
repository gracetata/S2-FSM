"""Launch the locomotion controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("locomotion_controller")
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "locomotion_controller.yaml",
                    ]
                ),
            ),
            Node(
                package="locomotion_controller",
                executable="locomotion_controller_node",
                name="locomotion_controller",
                parameters=[{"config_file": config_file}],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
