#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>

#include "utilities.h"

class DetectionLocationCalculator : public rclcpp::Node {
    std::optional<image_transport::ImageTransport> imageTransport;
    std::optional<image_transport::TransportHints> transportHints;
    std::optional<image_transport::Subscriber> depthSubscriber;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
    std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>> detectionsSubscription;
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> detectionsPublisher;

    std::shared_ptr<const sensor_msgs::msg::CameraInfo> cameraInfo;
    std::shared_ptr<const vision_msgs::msg::Detection2DArray> detections;

    geometry_msgs::msg::Point cameraToWorld(const double xImg, const double yImg, const double depth) const {
        KC_DEBUG_ASSERT_ROS(get_logger(), cameraInfo != nullptr, "cameraInfo is null!");
        KC_DEBUG_ASSERT_ROS(get_logger(), depth >= 0, "depth is negative!?");
        geometry_msgs::msg::Point ret;
        const auto fx = cameraInfo->k[0]; // x focal length
        const auto fy = cameraInfo->k[3 + 1]; // y focal length
        // these equations are derived from the pinhole model of a camera
        ret.x = xImg / (fx / depth);
        ret.y = yImg / (fy / depth);
        ret.z = depth;
        return ret;
    }

    void onDepthImageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
        if (cameraInfo == nullptr) {
            RCLCPP_ERROR(get_logger(), "camera_info hasn't been received yet. Can't process detections!");
            return;
        }

        const auto depth = cv_bridge::toCvShare(msg);

        const auto now = get_clock()->now();
        const auto depthAge = now - msg->header.stamp;
        const auto detectionsAge = now - detections->header.stamp;

        using namespace std::chrono;
        using namespace std::chrono_literals;
        constexpr auto DESYNC_WARN_THRESHOLD = 75ms;
        constexpr auto DESYNC_REJECT_THRESHOLD = 200ms;
        const auto desync = (depthAge - detectionsAge).to_chrono<milliseconds>();
        if (desync > DESYNC_REJECT_THRESHOLD) {
            RCLCPP_ERROR(
                get_logger(), "Depth and detections are too desynced (%ld ms) and will be rejected!",
                desync.count()
            );
            return;
        }
        if (desync > DESYNC_WARN_THRESHOLD) {
            RCLCPP_WARN(get_logger(), "Depth and detections are desynced by %ld ms!", desync.count());
        }

        auto localizedDetections = *detections;
        for (auto& detection : localizedDetections.detections) {
            geometry_msgs::msg::PoseWithCovariance pose;
            const auto [x, y] = detection.bbox.center.position;
            cv::Rect roi {
                cv::Point2d { x, y },
                cv::Size2d {
                    detection.bbox.size_x,
                    detection.bbox.size_y
                }
            };
            const auto averageDepth = cv::mean(depth->image(roi))[0];
            pose.pose.position = cameraToWorld(x - 0.5 * msg->width, y - 0.5 * msg->height, averageDepth);
            // todo covariance

            for (auto& result : detection.results) {
                result.pose = pose;
            }
        }
        detectionsPublisher->publish(localizedDetections);
    }

public:
    DetectionLocationCalculator() : Node("detection_location_calculator") {
        // according to the image_transport example, TransportHints doesn't set this up by itself
        declare_parameter<std::string>("image_transport", "raw");

        cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            "rgb/camera_info", rclcpp::ServicesQoS(),
            [this](std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg) {
                cameraInfo = std::move(msg);
            }
        );
        detectionsSubscription = create_subscription<vision_msgs::msg::Detection2DArray>(
            "detections", rclcpp::QoS(2),
            [this](std::shared_ptr<const vision_msgs::msg::Detection2DArray> msg) {
                detections = std::move(msg);
            }
        );
    }

    void init() {
        // image_transport needs a shared pointer to the node, which can't be done inside the constructor due to how
        // shared_from_this works.
        imageTransport.emplace(shared_from_this());
        transportHints.emplace(this);
        depthSubscriber.emplace(imageTransport->subscribe(
            "stereo/image", 3,
            &DetectionLocationCalculator::onDepthImageReceived, this,
            &*transportHints
        ));
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<DetectionLocationCalculator>();
    node->init();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}