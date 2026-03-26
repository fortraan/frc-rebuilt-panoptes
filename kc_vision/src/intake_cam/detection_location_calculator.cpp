#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.hpp>

class DetectionLocationCalculator : public rclcpp::Node {
    std::optional<image_transport::ImageTransport> imageTransport;
    std::optional<image_transport::TransportHints> transportHints;
    std::optional<image_transport::Subscriber> depthSubscriber;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
    std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>> detectionsSubscription;

    void onDepthImageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {

    }
public:
    DetectionLocationCalculator() : Node("detection_location_calculator") {
        // according to the image_transport example, TransportHints doesn't set this up by itself
        declare_parameter<std::string>("image_transport", "raw");
    }

    void init() {
        // image_transport needs a shared pointer to the node, which can't be done inside the constructor due to how
        // shared_from_this works.
        imageTransport.emplace(shared_from_this());
        transportHints.emplace(this);
        depthSubscriber.emplace(imageTransport->subscribe(
            "detections", 3,
            &DetectionLocationCalculator::onDepthImageReceived, this,
            &*transportHints
        ));
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<DetectionLocationCalculator>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}