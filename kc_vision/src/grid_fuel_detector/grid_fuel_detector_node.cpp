#include <memory>
#include <optional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

// hack to support compiling on different ROS distros
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/highgui.hpp>

#include "GridFuelDetector.h"
#include "utilities.h"

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
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> gridPublisher;

    std::optional<GridFuelDetector> detector;
    std::optional<image_transport::ImageTransport> imageTransport;
    std::optional<image_transport::Subscriber> imageSubscriber;

    std::string gridFrameId;
    int gridWidth;
    int gridHeight;
    float gridResolution;
    bool showDebugDisplays;

    rclcpp::Parameter occupancyThreshold;
    rclcpp::Parameter hLow, sLow, vLow;
    rclcpp::Parameter hHigh, sHigh, vHigh;

    void onCameraInfoReceived(const std::shared_ptr<const sensor_msgs::msg::CameraInfo>& msg) {
        // make sure we haven't already initialized the detector
        if (detector) return;

        // lookup transform from grid frame to camera optical frame
        using namespace std::chrono_literals;
        const auto tformMsg = tfBuffer.lookupTransform(
            msg->header.frame_id, gridFrameId, tf2::TimePointZero, 5s
        );
        Eigen::Isometry3d tform = tf2::transformToEigen(tformMsg);

        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> p;
        std::ranges::copy(msg->p, p.data());
        Eigen::Matrix3d intrinsicMatrix = p(Eigen::all, Eigen::seq(0, 2));

        detector.emplace(
            tform, cv::Size2i(gridWidth, gridHeight), gridResolution,
            cv::Size2i(msg->width, msg->height), intrinsicMatrix,
            showDebugDisplays
        );
        imageTransport.emplace(shared_from_this());
        imageSubscriber.emplace(imageTransport->subscribe(
            "image_rect", 1, &GridFuelDetectorNode::onFrameReceived, this
        ));
    }

    void onFrameReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
        KC_DEBUG_ASSERT_ROS(get_logger(), detector.has_value(), "detector is empty!");
        const auto start = get_clock()->now();
        detector->occupancyThreshold = occupancyThreshold.as_double();
        detector->hsvLow = cv::Scalar(
            static_cast<double>(hLow.as_int()), static_cast<double>(sLow.as_int()), static_cast<double>(vLow.as_int())
        );
        detector->hsvHigh = cv::Scalar(
            static_cast<double>(hHigh.as_int()), static_cast<double>(sHigh.as_int()), static_cast<double>(vHigh.as_int())
        );
        const auto cvFrame = cv_bridge::toCvShare(msg);
        KC_DEBUG_ASSERT_ROS(
            get_logger(), cvFrame->encoding == sensor_msgs::image_encodings::RGB8, "image has wrong encoding!"
        );
        const auto results = detector->processFrame(cvFrame->image);
        const auto processingTime = get_clock()->now() - start;
        const double fps = 1.0 / processingTime.seconds();
        if (fps < 30) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Processing is running slow (%.1f FPS)!", fps);
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000, "Processing took %ld ms (%.1f FPS)",
            processingTime.to_chrono<std::chrono::milliseconds>().count(), fps
        );

        nav_msgs::msg::OccupancyGrid occupancyGrid;
        occupancyGrid.header.frame_id = gridFrameId;
        occupancyGrid.header.stamp = msg->header.stamp;
        occupancyGrid.info.width = gridWidth;
        occupancyGrid.info.height = gridHeight;
        occupancyGrid.info.resolution = gridResolution;
        occupancyGrid.data.resize(results.occupancy.size().area());
        cv::Mat dataView(cv::Size(gridWidth, gridHeight), CV_8SC1, occupancyGrid.data.data());
        // by convention, ROS uses values 0-100 for occupancy grids
        results.occupancy.convertTo(dataView, CV_8SC1, 100.0 / 255.0);

        gridPublisher->publish(occupancyGrid);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Found %lu clumps", results.clumps.size());
    }

public:
    GridFuelDetectorNode() : Node("grid_fuel_detector"), tfBuffer(get_clock()), tfListener(tfBuffer, this) {
        cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(1),
            [this](std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg) {
                onCameraInfoReceived(msg);
            }
        );
        gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("grid", rclcpp::SensorDataQoS());

        // todo switch to using descriptors
        gridFrameId = declare_parameter<std::string>("grid.frame_id", "front_grid");
        VALIDATE_PARAM(!gridFrameId.empty(), "grid.frame_id cannot be empty!");
        gridWidth = static_cast<int>(declare_parameter<int64_t>("grid.cols", 0));
        VALIDATE_PARAM(gridWidth > 0, "grid.cols must be greater than zero!");
        gridHeight = static_cast<int>(declare_parameter<int64_t>("grid.rows", 0));
        VALIDATE_PARAM(gridHeight > 0, "grid.rows must be greater than zero!");
        gridResolution = static_cast<float>(declare_parameter<double>("grid.cell_size", 0));
        VALIDATE_PARAM(gridResolution > 0, "grid.cell_size must be greater than zero!");
        showDebugDisplays = declare_parameter<bool>("show_debug_displays", false);

        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = "Threshold at which to consider a grid cell occupied";
            rcl_interfaces::msg::FloatingPointRange range;
            range.from_value = 0;
            range.to_value = 1;
            descriptor.floating_point_range.push_back(range);
            declare_parameter<double>("occupancy_threshold", 0.5, descriptor);
            occupancyThreshold = get_parameter("occupancy_threshold");
        }

        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.integer_range.emplace_back();

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 180;
            declare_parameter<int>("low.h", 0, descriptor);
            hLow = get_parameter("low.h");

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 255;
            declare_parameter<int>("low.s", 170, descriptor);
            sLow = get_parameter("low.s");

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 255;
            declare_parameter<int>("low.v", 115, descriptor);
            vLow = get_parameter("low.v");

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 180;
            declare_parameter<int>("high.h", 88, descriptor);
            hHigh = get_parameter("high.h");

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 255;
            declare_parameter<int>("high.s", 255, descriptor);
            sHigh = get_parameter("high.s");

            descriptor.integer_range[0].from_value = 0;
            descriptor.integer_range[0].to_value = 255;
            declare_parameter<int>("high.v", 255, descriptor);
            vHigh = get_parameter("high.v");
        }
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<GridFuelDetectorNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}