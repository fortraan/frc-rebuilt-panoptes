#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <fmt/format.h>

#include <NvInfer.h>
#include <opencv4/opencv2/core/matx.hpp>

class InferenceNode : public rclcpp::Node {
    std::optional<image_transport::ImageTransport> imageTransport;
    std::optional<image_transport::TransportHints> transportHints;
    std::optional<image_transport::Subscriber> imageSubscriber;

    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> detectionPublisher;

    void onImageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {

    }

public:
    InferenceNode() : Node("inference") {
        declare_parameter<std::string>("image_transport", "raw");
        const std::filesystem::path enginePath(
            declare_parameter<std::string>("engine_file_path", "")
        );
        if (!std::filesystem::exists(enginePath)) {
            const std::string msg = fmt::format("Engine file \"{}\" does not exist!", enginePath.c_str());
            RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        }

        detectionPublisher = create_publisher<vision_msgs::msg::Detection2DArray>(
            "detections", rclcpp::QoS(2)
        );
    }

    void init() {
        imageTransport.emplace(shared_from_this());
        transportHints.emplace(this);
        imageSubscriber.emplace(imageTransport->subscribe(
            "rgb/image_rect", 2,
            &InferenceNode::onImageReceived, this,
            &*transportHints
        ));
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<InferenceNode>();
    node->init();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}