#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <ranges>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <fmt/format.h>

#include "utilities.h"

namespace {
    constexpr std::array DISTORTION_COEFFICIENTS_ALLOWED_SIZES { 0, 4, 5, 8, 12, 14 };

    constexpr double TAG_SIZE = 0.165; // meters

    const std::vector TAG_3D_POINTS {
        cv::Point3d { -TAG_SIZE / 2,  TAG_SIZE / 2, 0 },
        cv::Point3d {  TAG_SIZE / 2,  TAG_SIZE / 2, 0 },
        cv::Point3d {  TAG_SIZE / 2, -TAG_SIZE / 2, 0 },
        cv::Point3d { -TAG_SIZE / 2, -TAG_SIZE / 2, 0}
    };

    tf2::Quaternion rodriguesToQuaternion(const cv::Matx31d& vector) {
        const double theta = cv::norm(vector);
        return {
            vector(0) * std::sin(theta / 2) / theta,
            vector(1) * std::sin(theta / 2) / theta,
            vector(2) * std::sin(theta / 2) / theta,
            std::cos(theta / 2),
        };
    }
}

class SolvePnP : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
    std::shared_ptr<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>> detectionsSubscription;

    std::string cameraFrameId;
    std::string posePrefix;

    bool hasTransform;
    tf2::Transform robotToCamera;

    std::mutex intrinsicsMutex;
    cv::Matx33d cameraMatrix;
    cv::Mat distortionCoefficients;

    void onReceiveCameraInfo(const sensor_msgs::msg::CameraInfo& cameraInfo) {
        std::lock_guard lock(intrinsicsMutex);

        KC_DEBUG_ASSERT_ROS(get_logger(), cameraInfo.k.size() == 9, "Got wrong size of camera matrix!");
        std::ranges::copy(cameraInfo.k, cameraMatrix.val);

        const auto numDistortionCoefficients = static_cast<int>(cameraInfo.d.size());
        KC_DEBUG_ASSERT_ROS(
            get_logger(), std::ranges::find(
                DISTORTION_COEFFICIENTS_ALLOWED_SIZES, numDistortionCoefficients
            ) != DISTORTION_COEFFICIENTS_ALLOWED_SIZES.end(),
            "Invalid number of distortion coefficients!"
        );
        if (distortionCoefficients.empty() || distortionCoefficients.rows != numDistortionCoefficients) {
            distortionCoefficients = cv::Mat(numDistortionCoefficients, 1, CV_64FC1);
        }
        std::ranges::copy(cameraInfo.d, reinterpret_cast<double*>(distortionCoefficients.data));

        RCLCPP_INFO_ONCE(get_logger(), "Received camera intrinsics");
    }

    void onReceiveDetections(const apriltag_msgs::msg::AprilTagDetectionArray& detections) {
        std::lock_guard lock(intrinsicsMutex);

        if (!hasTransform) {
            // lazy init because waiting for the transform in the constructor is a bit of a chore -
            // tf2_ros::Buffer::waitForTransform can't be called until the node is fully initialized
            RCLCPP_INFO(get_logger(), "Looking up transform from robot to %s", cameraFrameId.c_str());

            using namespace std::chrono_literals;
            const geometry_msgs::msg::TransformStamped robotToCameraMsg = buffer.lookupTransform(
                cameraFrameId, "robot", tf2::TimePointZero, 100ms
            );
            tf2::fromMsg(robotToCameraMsg.transform, robotToCamera);
            hasTransform = true;
        }

        const rclcpp::Time detectionTime = detections.header.stamp;
        for (const auto& detection : detections.detections) {
            cv::Matx<double, 4, 2> imgPoints;

            KC_DEBUG_ASSERT_ROS(
                get_logger(),
                static_cast<int>(detection.corners.size()) == 4,
                "Incorrect number of detection corners!"
            );
            // todo check this with apriltag_ros
            // detections returned by apriltag are in the following order:
            // bottom left, bottom right, top right, top left
            // OpenCV expects the corners in the following order:
            // top left, top right, bottom right, bottom left
            const auto assignFromApriltag = [&imgPoints, &detection](const int cvIdx, const int aprIndex) {
                imgPoints(cvIdx, 0) = detection.corners[aprIndex].x;
                imgPoints(cvIdx, 1) = detection.corners[aprIndex].y;
            };
            assignFromApriltag(0, 3);
            assignFromApriltag(1, 2);
            assignFromApriltag(2, 1);
            assignFromApriltag(3, 0);

            // compute possible poses
            std::vector<cv::Matx31d> translations, rotations;
            const int numPoses = cv::solvePnPGeneric(
                TAG_3D_POINTS, imgPoints, cameraMatrix, distortionCoefficients,
                rotations, translations, false, cv::SOLVEPNP_IPPE_SQUARE
            );

            KC_DEBUG_ASSERT_ROS(
                get_logger(),
                static_cast<int>(translations.size()) == numPoses,
                "Incorrect number of translations!"
            );
            KC_DEBUG_ASSERT_ROS(
                get_logger(),
                static_cast<int>(rotations.size()) == numPoses,
                "Incorrect number of rotations!"
            );

            for (int i = 0; i < numPoses; i++) {
                tf2::Transform cameraToTag {
                    rodriguesToQuaternion(rotations[i]),
                    { translations[i].val[0], translations[i].val[1], translations[i].val[2] }
                };
                tf2::Transform tagToRobot = (robotToCamera * cameraToTag).inverse();

                // publish transform
                geometry_msgs::msg::TransformStamped transformMsg;

                // static frame of the tag on the field
                transformMsg.header.frame_id = fmt::format("apriltag_{}", detection.id);
                // dynamic frame of the estimate of the camera's pose
                transformMsg.child_frame_id = fmt::format(
                    "{}robot_pose_estimate_{}:{}", posePrefix, detection.id, i
                );
                // tf2 will automatically handle interpolating between times when calculating transforms, so the
                // timestamp of the transform needs to reflect when the measurement was taken.
                transformMsg.header.stamp = detectionTime;

                transformMsg.transform = tf2::toMsg(tagToRobot);

                // send it!
                broadcaster.sendTransform(transformMsg);
            }
        }
    }

public:
    SolvePnP() : Node("solve_pnp"), buffer(get_clock()), listener(buffer, this), broadcaster(this),
        hasTransform(false)
    {
        cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(1),
            [this](const sensor_msgs::msg::CameraInfo& cameraInfo) {
                onReceiveCameraInfo(cameraInfo);
            }
        );
        detectionsSubscription = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "detections", rclcpp::SensorDataQoS(),
            [this](const apriltag_msgs::msg::AprilTagDetectionArray& detections) {
                onReceiveDetections(detections);
            }
        );

        cameraFrameId = declare_parameter<std::string>("camera_frame_id", "");
        posePrefix = declare_parameter<std::string>("pose_prefix", "");

        if (cameraFrameId.empty()) {
            constexpr auto msg = "camera_frame_id must be specified!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }

        RCLCPP_INFO(
            get_logger(), "Reading detections from %s and publishing poses under the prefix %s",
            detectionsSubscription->get_topic_name(), posePrefix.c_str()
        );

        RCLCPP_INFO(get_logger(), "Initialization complete");
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    try {
        const auto node = std::make_shared<SolvePnP>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error! " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}