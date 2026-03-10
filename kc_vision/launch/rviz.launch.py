from launch import LaunchDescription
from launch.substitutions import FileContent
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("kc_vision")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d", os.path.join(
                    pkg_share,
                    "config",
                    "rviz_config.rviz"
                )
            ]
        )
    ])
