#include <memory>
#include <optional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include "GridFuelDetector.h"

#define VALIDATE_PARAM(condition, msg) do { \
    if (!(condition)) { \
        RCLCPP_FATAL(get_logger(), msg); \
        throw std::runtime_error(msg); \
    } \
} while (false)

class GridFuelDetectorNode : public rclcpp::Node {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;

    std::optional<GridFuelDetector> detector;

    std::string gridFrameId;
    int gridWidth;
    int gridHeight;
    double gridResolution;

    double occupancyThreshold;
    cv::Scalar hsvLow, hsvHigh;

    void onCameraInfoReceived(const std::shared_ptr<const sensor_msgs::msg::CameraInfo>& msg) {
        // make sure we haven't already initialized the detector
        if (detector) return;

        // lookup transform from grid frame to camera optical frame
        const auto tformMsg = tfBuffer.lookupTransform(msg->header.frame_id, gridFrameId, tf2::TimePointZero);
        Eigen::Isometry3d tform = tf2::transformToEigen(tformMsg);

        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> p;
        std::ranges::copy(msg->p, p.data());
        Eigen::Matrix3d intrinsicMatrix = p(Eigen::all, Eigen::seq(0, 2));

        detector.emplace(
            tform, cv::Size2i(gridWidth, gridHeight), gridResolution,
            cv::Size2i(msg->width, msg->height), intrinsicMatrix,
            0, cv::Scalar(0), cv::Scalar(0)
        );
    }

public:
    GridFuelDetectorNode() : Node("grid_fuel_detector"), tfBuffer(get_clock()), tfListener(tfBuffer, this) {
        cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(1),
            [this](std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg) {
                onCameraInfoReceived(msg);
            }
        );

        gridFrameId = declare_parameter<std::string>("grid_frame_id", "");
        VALIDATE_PARAM(!gridFrameId.empty(), "grid_frame_id cannot be empty!");
        gridWidth = declare_parameter<int>("grid_cols", 10);
        VALIDATE_PARAM(gridWidth > 0, "grid_cols must be greater than zero!");
        gridHeight = declare_parameter<int>("grid_rows", 10);
        VALIDATE_PARAM(gridHeight > 0, "grid_rows must be greater than zero!");
        gridResolution = declare_parameter<double>("grid_cell_size", 0.1);
        VALIDATE_PARAM(gridResolution > 0, "grid_cell_size must be greater than zero!");
        occupancyThreshold = declare_parameter("occupancy_threshold", 0.5);
        hsvLow = {

        };
    }
};