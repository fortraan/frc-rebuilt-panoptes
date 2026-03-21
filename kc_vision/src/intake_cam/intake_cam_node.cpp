#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <NvInfer.h>

#include "OakPipeline.h"

namespace {
    // hardcoded for testing
    constexpr auto ENGINE_FILE_NAME = "bnb-yolov26n-fp16+fp32.engine";
    constexpr auto WINDOW_NAME = "Raw Detections";
}

class NvRosLogger : public nvinfer1::ILogger {
    rclcpp::Logger rosLogger;
public:
    explicit NvRosLogger(const rclcpp::Logger& logger) : rosLogger(logger) { }
    void log(const Severity severity, const nvinfer1::AsciiChar* msg) noexcept override {
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
            case Severity::kERROR:
                RCLCPP_ERROR(rosLogger, "%s", msg);
                break;
            case Severity::kWARNING:
                RCLCPP_WARN(rosLogger, "%s", msg);
                break;
            case Severity::kINFO:
                RCLCPP_INFO(rosLogger, "%s", msg);
                break;
            case Severity::kVERBOSE:
                RCLCPP_DEBUG(rosLogger, "%s", msg);
                break;
        }
    }
};

class IntakeCam : public rclcpp::Node {
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPublisher;

    NvRosLogger nvLogger;
    std::unique_ptr<OakPipeline> oakPipeline;

    rclcpp::Time daiTimestampToRosTime(const std::chrono::time_point<std::chrono::steady_clock> daiTimestamp) {
        // current time in ros minus timestamp age. DAI uses steady_clock for timestamping.
        return get_clock()->now() - (std::chrono::steady_clock::now() - daiTimestamp);
    }

    void onDetectionsReceived(const std::shared_ptr<dai::SpatialImgDetections>& detections) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received %lu detections", detections->detections.size());
    }

    void onTracksReceived(const std::shared_ptr<dai::Tracklets>& tracks) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received %lu tracks", tracks->tracklets.size());
        if (tracks->tracklets.empty()) return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "intake_camera";
        marker.header.stamp = daiTimestampToRosTime(tracks->getTimestamp());
        marker.ns = "intake_camera";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        constexpr double FUEL_DIAMETER = 0.015; // m
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 1.8;
        marker.color.r = 252.0 / 255.0;
        marker.color.g = 198.0 / 255.0;
        marker.color.b =   3.0 / 255.0;
        marker.color.a =   1;
        marker.frame_locked = true;
        using namespace std::chrono_literals;
        marker.lifetime = rclcpp::Duration(100ms);
        marker.points.reserve(tracks->tracklets.size());
        for (const auto& track : tracks->tracklets) {
            if (track.status == dai::Tracklet::TrackingStatus::LOST ||
                track.status == dai::Tracklet::TrackingStatus::REMOVED) {
                continue;
            }
            geometry_msgs::msg::Point point;
            point.x = track.spatialCoordinates.x / 1000.0;
            point.y = track.spatialCoordinates.y / 1000.0;
            point.z = track.spatialCoordinates.z / 1000.0;
            marker.points.push_back(point);
        }
        markerPublisher->publish(marker);
    }

public:
    IntakeCam() : Node("intake_camera"), nvLogger(get_logger().get_child("TensorRT")) {
        RCLCPP_INFO(get_logger(), "Initializing...");

        markerPublisher = create_publisher<visualization_msgs::msg::Marker>(
            "intake_camera_markers", rclcpp::SensorDataQoS()
        );

        oakPipeline = OakPipeline::create(
            std::filesystem::path(ament_index_cpp::get_package_share_directory("kc_vision"))
            / "resources" / ENGINE_FILE_NAME,
            nvLogger
        );
        oakPipeline->addDetectionCallback([this](const auto& detections) {
            onDetectionsReceived(detections);
        });
        oakPipeline->addTrackingCallback([this](const auto& tracks) {
            onTracksReceived(tracks);
        });

        oakPipeline->start();

        RCLCPP_INFO(get_logger(), "Initialized successfully");
    }

    ~IntakeCam() noexcept override {
        oakPipeline->stop();
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    try {
        const auto node = std::make_shared<IntakeCam>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::runtime_error& e) {
        std::cerr << "Error! " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}