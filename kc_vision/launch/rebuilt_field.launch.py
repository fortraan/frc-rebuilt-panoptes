from launch import LaunchDescription
from launch.substitutions import FileContent
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("kc_vision")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="field_publisher",
            parameters=[{
                "robot_description": FileContent(os.path.join(
                    pkg_share,
                    "urdf",
                    "rebuilt_field.urdf"
                )),
                "publish_frequency": 0.2 # publish field transforms every 5 seconds
            }],
            remappings=[
                # this node isn't publishing the description of the actual robot, so remap its topics
                ("/robot_description", "/field_description"),
                ("/joint_states", "/field_joint_states")
            ]
        ),
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
