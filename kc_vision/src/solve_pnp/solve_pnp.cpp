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

#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <kc_vision_msgs/msg/observation.hpp>

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

    void convert(const geometry_msgs::msg::Transform& transform, geometry_msgs::msg::Pose& pose) {
        pose.position.x = transform.translation.x;
        pose.position.y = transform.translation.y;
        pose.position.z = transform.translation.z;
        pose.orientation = transform.rotation;
    }
}

class SolvePnP : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
    std::shared_ptr<rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>> detectionsSubscription;

    std::shared_ptr<rclcpp::Publisher<kc_vision_msgs::msg::Observation>> observationPublisher;

    std::string cameraFrameId;
    std::string posePrefix;

    bool hasTransform;
    tf2::Transform robotToCamera;

    std::mutex intrinsicsMutex;
    cv::Matx33d cameraMatrix;
    cv::Mat distortionCoefficients;

    geometry_msgs::msg::Transform cvToRosTform(const cv::Matx31d& translation, const cv::Matx31d& rotation) const {
        const tf2::Transform cameraToTag {
            rodriguesToQuaternion(rotation),
            { translation.val[0], translation.val[1], translation.val[2] }
        };
        // todo this doesn't seem to be working quite right
        const tf2::Transform tagToRobot = (robotToCamera * cameraToTag).inverse();
        return tf2::toMsg(tagToRobot);
    }

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

    void onReceiveDetections(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray& detections) {
        std::lock_guard lock(intrinsicsMutex);

        if (!hasTransform) {
            // lazy init because waiting for the transform in the constructor is a bit of a chore -
            // tf2_ros::Buffer::waitForTransform can't be called until the node is fully initialized
            RCLCPP_INFO(get_logger(), "Looking up transform from robot to %s", cameraFrameId.c_str());

            using namespace std::chrono_literals;
            const geometry_msgs::msg::TransformStamped robotToCameraMsg = buffer.lookupTransform(
                "robot", cameraFrameId, tf2::TimePointZero, 100ms
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
            // detections returned by isaac_ros_apriltag are in the following order:
            /// top left, top right, bottom right, bottom left
            // OpenCV expects the corners in the following order:
            // top left, top right, bottom right, bottom left
            const auto assignFromApriltag = [&imgPoints, &detection](const int cvIdx, const int aprIndex) {
                imgPoints(cvIdx, 0) = detection.corners[aprIndex].x;
                imgPoints(cvIdx, 1) = detection.corners[aprIndex].y;
            };
            assignFromApriltag(0, 0);
            assignFromApriltag(1, 1);
            assignFromApriltag(2, 2);
            assignFromApriltag(3, 3);

            // compute possible poses
            std::vector<cv::Matx31d> translations, rotations;
            std::vector<double> reprojectionErrors;
            const int numPoses = cv::solvePnPGeneric(
                TAG_3D_POINTS, imgPoints, cameraMatrix, distortionCoefficients,
                rotations, translations, false, cv::SOLVEPNP_IPPE_SQUARE,
                cv::noArray(), cv::noArray(), reprojectionErrors
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
            KC_DEBUG_ASSERT_ROS(
                get_logger(),
                static_cast<int>(reprojectionErrors.size()) == numPoses,
                "Incorrect number of reprojection errors!"
            );
            KC_DEBUG_ASSERT_ROS(get_logger(), numPoses == 2, "Incorrect number of returns from IPPE-Square!");

            const bool firstIsBetter = reprojectionErrors[0] < reprojectionErrors[1];
            const auto primaryIndex = firstIsBetter ? 0 : 1;
            const auto secondaryIndex = firstIsBetter ? 1 : 0;
            const auto primary = cvToRosTform(translations[primaryIndex], rotations[primaryIndex]);
            const auto secondary = cvToRosTform(translations[secondaryIndex], rotations[secondaryIndex]);

            kc_vision_msgs::msg::Observation observation;
            observation.header.frame_id = fmt::format("apriltag_{}", detection.id);
            observation.header.stamp = detectionTime;
            observation.id = fmt::format("{}{}", posePrefix, observation.header.frame_id);
            convert(primary, observation.primary.pose);
            observation.primary.reprojection_error = reprojectionErrors[primaryIndex];
            observation.primary.range = cv::norm(translations[primaryIndex]);
            convert(secondary, observation.secondary.pose);
            observation.secondary.reprojection_error = reprojectionErrors[secondaryIndex];
            observation.secondary.range = cv::norm(translations[secondaryIndex]);
            observationPublisher->publish(observation);

            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header = observation.header;

            transformStamped.child_frame_id = fmt::format(
                "{}/primary", observation.id
            );
            transformStamped.transform = primary;
            broadcaster.sendTransform(transformStamped);

            transformStamped.child_frame_id = fmt::format(
                "{}/secondary", observation.id
            );
            transformStamped.transform = secondary;
            broadcaster.sendTransform(transformStamped);
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
        detectionsSubscription = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            "detections", rclcpp::SensorDataQoS(),
            [this](const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray& detections) {
                onReceiveDetections(detections);
            }
        );
        // note that this is in the global namespace! we don't need separate observation topics
        // for each camera, and it's easier to have them all publish to the same topic anyways.
        observationPublisher = create_publisher<kc_vision_msgs::msg::Observation>(
            "/observations", rclcpp::SensorDataQoS()
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