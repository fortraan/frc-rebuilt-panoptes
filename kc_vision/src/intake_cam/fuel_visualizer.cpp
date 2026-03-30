#include <chrono>
#include <memory>
#include <ranges>

#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {
    constexpr auto MARKER_NAMESPACE = "intake_camera";
    constexpr auto CLASS_ID = "fuel";
    using namespace std::chrono_literals;
    const rclcpp::Duration MARKER_LIFETIME(100ms);
}

class FuelVisualizer : public rclcpp::Node {
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPublisher;
    std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>> detectionsSubscription;

    void onDetectionsReceived(const std::shared_ptr<const vision_msgs::msg::Detection2DArray>& detections) const {
        visualization_msgs::msg::Marker marker;
        marker.header = detections->header;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.frame_locked = false;
        marker.ns = MARKER_NAMESPACE;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.pose.orientation.w = 1;
        marker.lifetime = MARKER_LIFETIME;
        marker.points.reserve(detections->detections.size());
        for (const auto& detection : detections->detections) {
            const auto resultIter = std::ranges::find_if(detection.results, [](const auto result) {
                return result.hypothesis.class_id == CLASS_ID;
            });
            if (resultIter == detection.results.end()) continue;
            marker.points.emplace_back(resultIter->pose.pose.position);
        }

        markerPublisher->publish(marker);
    }

public:
    FuelVisualizer() : Node("fuel_visualizer") {
        markerPublisher = create_publisher<visualization_msgs::msg::Marker>(
            "/markers", rclcpp::QoS(10)
        );
        detectionsSubscription = create_subscription<vision_msgs::msg::Detection2DArray>(
            "detections_localized", rclcpp::QoS(2),
            [this](std::shared_ptr<const vision_msgs::msg::Detection2DArray> msg) {
                onDetectionsReceived(msg);
            }
        );
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<FuelVisualizer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}