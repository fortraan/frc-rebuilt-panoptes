from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import FileContent

from ament_index_python.packages import get_package_share_directory
import os


def get_config_path(name):
    return os.path.join(
        get_package_share_directory("kc_vision"),
        "config",
        name
    )

def camera_nodes(namespace, config, camera_id):
    return [
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
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify",
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
                ),
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::ResizeNode",
                    name="preview",
                    namespace=namespace,
                    remappings=[
                        ("image/image_raw", "image_rect"),
                        ("image/camera_info", "camera_info"),
                        ("resized/image_raw", "preview/image_rect"),
                        ("resized/camera_info", "preview/camera_info")
                    ],
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
    pkg_share = get_package_share_directory("kc_vision")
    base_params = get_config_path("base_params.yaml")
    intake_camera_ns = "intake_camera"
    intake_camera_path = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9782_USB_Camera_UC852-video-index0"

    nodes = [
        # this node publishes a model and description of the robot
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                base_params,
                {
                    "robot_description": FileContent(os.path.join(
                        pkg_share,
                        "urdf",
                        "mk1_robot.urdf"
                    )),
                }
            ]
        ),
        # publishes a transform between the fixed frame and the robot frame. only for testing.
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="debug_robot_pos_broadcaster",
        #     parameters=[base_params],
        #     arguments=[
        #         "--frame-id", "field",
        #         "--child-frame-id", "robot",
        #         "--x", "8.259",
        #         "--y", "4.0215"
        #     ]
        # ),
        # this node publishes a model and description of the field
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
        # tag_consensus takes the detections provided by the apriltag nodes and computes the camera pose
        # for each tag. it considers all solutions of solvePnP. obvious outliers are rejected, and the
        # remaining solutions are processed with RANSAC to produce a single pose estimate.
        Node(
            package="kc_vision",
            executable="tag_consensus",
            name="tag_consensus",
            parameters=[base_params],
            arguments=["--ros-args", "--log-level", "tag_consensus:=DEBUG"]
        ),
        # ros_nt_bridge connects ROS to NetworkTables. it sends the pose estimate from tag_consensus to
        # the Rio via NetworkTables. additionally, it listens for the fused pose estimate computed by the
        # Rio and publishes it as a TF2 frame.
        Node(
            package="kc_vision",
            executable="ros_nt_bridge",
            name="ros_nt_bridge",
            parameters=[base_params]
        ),

        # monocular intake camera nodes
        ComposableNodeContainer(
            name="container",
            namespace=intake_camera_ns,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="usb_cam",
                    plugin="usb_cam::UsbCamNode",
                    name="usb_cam",
                    namespace=intake_camera_ns,
                    parameters=[
                        base_params,
                        {
                            "video_device": os.path.realpath(intake_camera_path)
                        }
                    ],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                ),
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify",
                    namespace=intake_camera_ns,
                    remappings=[
                        ("image", "image_raw")
                    ],
                    parameters=[base_params],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                ),
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::ResizeNode",
                    name="resize",
                    namespace=intake_camera_ns,
                    remappings=[
                        ("image/image_raw", "image_rect"),
                        ("image/camera_info", "camera_info"),
                        ("resized/image_raw", "preview/image_rect"),
                        ("resized/camera_info", "preview/camera_info")
                    ],
                    extra_arguments=[{
                        "use_intra_process_comms": True
                    }]
                )
            ]
        ),

        # intake camera fuel detection
        Node(
            package="kc_vision",
            executable="grid_fuel_detector",
            name="fuel_detector",
            namespace=intake_camera_ns,
            parameters=[base_params]
        ),
        Node(
            package="kc_vision",
            executable="grid_fuel_visualizer",
            name="fuel_visualizer",
            namespace=intake_camera_ns,
            parameters=[base_params]
        ),

        # OAK-D nodes
        # ComposableNodeContainer(
        #     name="container",
        #     package="rclcpp_components",
        #     namespace=intake_camera_ns,
        #     executable="component_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package="depthai_ros_driver_v3",
        #             plugin="depthai_ros_driver::Driver",
        #             name=intake_camera_ns, # the driver namespaces everything under its own name
        #             parameters=[base_params],
        #             remappings=[
        #                 ("/robot_description", "description")
        #             ]
        #         ),
        #         ComposableNode(
        #             package="image_proc",
        #             plugin="image_proc::RectifyNode",
        #             name="rectify",
        #             namespace=intake_camera_ns + "/rgb",
        #             parameters=[base_params],
        #             remappings=[
        #                 ("image", "image_raw")
        #             ]
        #         ),
        #         ComposableNode(
        #             package="image_proc",
        #             plugin="image_proc::RectifyNode",
        #             name="rectify",
        #             namespace=intake_camera_ns + "/stereo",
        #             parameters=[base_params],
        #             remappings=[
        #                 ("image", "image_raw")
        #             ]
        #         ),
        #         ComposableNode(
        #             package="depth_image_proc",
        #             plugin="depth_image_proc::PointCloudXyzrgbNode",
        #             name="point_cloud",
        #             namespace=intake_camera_ns,
        #             parameters=[base_params],
        #             remappings=[
        #                 ("depth_registered/image_rect", "stereo/image_rect"), # the OAK registers the depth image for us
        #                 ("rgb/image_rect_color", "rgb/image_rect")
        #             ]
        #         )
        #     ]
        # )

        # system diagnostics
        Node(
            package="diagnostic_common_diagnostics",
            executable="cpu_monitor.py",
            name="cpu_monitor"
        ),
        Node(
            package="diagnostic_common_diagnostics",
            executable="ram_monitor.py",
            name="ram_monitor"
        ),
        Node(
            package="diagnostic_common_diagnostics",
            executable="sensors_monitor.py",
            name="orin_sensors_monitor"
        ),
        # Node(
        #     package="diagnostic_aggregator",
        #     executable="aggregator_node",
        #     name="diag_aggregator"
        # ),
    ]

    nodes.extend(camera_nodes(
        "front_camera", get_config_path("front_camera_params.yaml"),
        "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
    ))

    return LaunchDescription(nodes)