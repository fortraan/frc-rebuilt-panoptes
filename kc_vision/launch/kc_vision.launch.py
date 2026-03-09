from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os

from math import pi


def get_config_path(name):
    return os.path.join(
        get_package_share_directory("kc_vision"),
        "config",
        name
    )

def camera_nodes(namespace, config, camera_id):
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id", "robot",
                "--child-frame-id", "front_camera",
                #     "--x", 0.5,
                #     "--y", 0,
                #     "--z", 0.5,
                #     "--roll", -0.5 * pi,
                #     "--pitch", 0,
                #     "--yaw", -0.5 * pi
            ]
        ),
        ComposableNodeContainer(
            name="node_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                # usb_cam publishes frames read from the camera
                ComposableNode(
                    package="usb_cam",
                    plugin="usb_cam::UsbCamNode",
                    name="usb_cam",
                    namespace=namespace,
                    parameters=[
                        config,
                        {
                            "video_device": os.path.realpath(camera_id)
                        }
                    ],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify',
                    namespace=namespace,
                    remappings=[
                        ("image", "image_raw")
                    ],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                ),
                ComposableNode(
                    package="apriltag_ros",
                    plugin="AprilTagNode",
                    name="apriltag",
                    namespace=namespace,
                    parameters=[config],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                )
            ]
        ),
        # solve_pnp computes camera poses from apriltag detections and publishes them to the frame tree
        Node(
            package="kc_vision",
            executable="solve_pnp",
            namespace=namespace,
            parameters=[config]
        ),
    ]

def generate_launch_description():
    base_params = get_config_path("base_params.yaml")

    nodes = [
        # tag_consensus takes the detections provided by the apriltag nodes and computes the camera pose
        # for each tag. it considers all solutions of solvePnP. obvious outliers are rejected, and the
        # remaining solutions are processed with RANSAC to produce a single pose estimate.
        # Node(
        #     package="kc_vision",
        #     executable="tag_consensus",
        #     parameters=[base_params]
        # ),
        # ros_nt_bridge connects ROS to NetworkTables. it sends the pose estimate from tag_consensus to
        # the Rio via NetworkTables. additionally, it listens for the fused pose estimate computed by the
        # Rio and publishes it as a TF2 frame.
        # Node(
        #     package="kc_vision",
        #     executabe="ros_nt_bridge",
        #     parameters=[base_params]
        # ),
    ]

    nodes.extend(camera_nodes(
        "front_camera", get_config_path("front_camera_params.yaml"),
        "/dev/v4l/by-id/usb-046d_081b_64AF26A0-video-index0"
    ))

    return LaunchDescription(nodes)