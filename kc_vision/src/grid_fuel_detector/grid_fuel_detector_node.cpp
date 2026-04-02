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

    std::optional<GridFuelDetector> detector;
    std::optional<image_transport::ImageTransport> imageTransport;
    std::optional<image_transport::Subscriber> imageSubscriber;

    std::string gridFrameId;
    int gridWidth;
    int gridHeight;
    float gridResolution;

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
            occupancyThreshold.as_double(), cv::Scalar(
                hLow.as_int(), sLow.as_int(), vLow.as_int()
            ), cv::Scalar(
                hHigh.as_int(), sHigh.as_int(), vHigh.as_int()
            )
        );
        imageTransport.emplace(shared_from_this());
        imageSubscriber.emplace(imageTransport->subscribe(
            "image_rect", 1, &GridFuelDetectorNode::onFrameReceived, this
        ));
    }

    void onFrameReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
        KC_DEBUG_ASSERT_ROS(get_logger(), detector.has_value(), "detector is empty!");
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
        // todo publish results

        cv::Mat drawBuffer;
        cvFrame->image.copyTo(drawBuffer);
        detector->drawGrid(drawBuffer);
        cv::imshow("Grid", drawBuffer);

        nav_msgs::msg::OccupancyGrid occupancyGrid;
        occupancyGrid.header.frame_id = gridFrameId;
        occupancyGrid.header.stamp = msg->header.stamp;
        occupancyGrid.info.width = gridWidth;
        occupancyGrid.info.height = gridHeight;
        occupancyGrid.info.resolution = gridResolution;
        occupancyGrid.data.resize(results.occupancy.size().area());
        cv::Mat dataView(cv::Size(gridWidth, gridHeight), CV_8SC1, occupancyGrid.data.data());
        // by convention, ROS uses values 0-100
        results.occupancy.convertTo(dataView, CV_8SC1, 100.0 / 255.0);
    }

public:
    GridFuelDetectorNode() : Node("grid_fuel_detector"), tfBuffer(get_clock()), tfListener(tfBuffer, this) {
        cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(1),
            [this](std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg) {
                onCameraInfoReceived(msg);
            }
        );

        // todo switch to using descriptors
        gridFrameId = declare_parameter<std::string>("grid_frame_id", "front_grid");
        VALIDATE_PARAM(!gridFrameId.empty(), "grid_frame_id cannot be empty!");
        gridWidth = declare_parameter<int>("grid_cols", 10);
        VALIDATE_PARAM(gridWidth > 0, "grid_cols must be greater than zero!");
        gridHeight = declare_parameter<int>("grid_rows", 10);
        VALIDATE_PARAM(gridHeight > 0, "grid_rows must be greater than zero!");
        gridResolution = declare_parameter<float>("grid_cell_size", 0.1);
        VALIDATE_PARAM(gridResolution > 0, "grid_cell_size must be greater than zero!");

        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "Threshold at which to consider a grid cell occupied";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0;
        range.to_value = 1;
        descriptor.floating_point_range.push_back(range);
        declare_parameter<double>("occupancy_threshold", 0.5, descriptor);
        occupancyThreshold = get_parameter("occupancy_threshold");

        // todo range constraints
        declare_parameter<int>("h_low", 13);
        hLow = get_parameter("h_low");
        declare_parameter<int>("s_low", 133);
        sLow = get_parameter("s_low");
        declare_parameter<int>("v_low", 161);
        vLow = get_parameter("v_low");

        declare_parameter<int>("h_high", 30);
        hHigh = get_parameter("h_high");
        declare_parameter<int>("s_high", 255);
        sHigh = get_parameter("s_high");
        declare_parameter<int>("v_high", 255);
        vHigh = get_parameter("v_high");
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<GridFuelDetectorNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}