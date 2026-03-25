#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/SystemLogger.hpp>

class OakDriver : public rclcpp::Node {
    std::string frameId;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rawColorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr colorInfoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPublisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher;

    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::Camera> centerCamera;
    std::shared_ptr<dai::node::Camera> leftCamera;
    std::shared_ptr<dai::node::Camera> rightCamera;
    std::shared_ptr<dai::node::StereoDepth> depth;
    std::shared_ptr<dai::node::SystemLogger> systemLogger;

    std::shared_ptr<dai::MessageQueue> rawCenterCameraOutputQueue;
    std::shared_ptr<dai::MessageQueue> centerCameraOutputQueue;
    std::shared_ptr<dai::MessageQueue> depthOutputQueue;
    std::shared_ptr<dai::MessageQueue> systemLoggerOutputQueue;

    rclcpp::Time daiTimeToRosTime(const std::chrono::steady_clock::time_point& time) {
        // DAI timestamps are relative to std::chrono::steady_clock
        const auto age = std::chrono::steady_clock::now() - time;
        return get_clock()->now() - age;
    }

    void publishRawCenter(const std::shared_ptr<dai::ADatatype>& daiMsg) {
        if (daiMsg->getDatatype() != dai::DatatypeEnum::ImgFrame) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Received wrong datatype from raw camera output queue!");
            return;
        }
        const auto frame = std::static_pointer_cast<dai::ImgFrame>(daiMsg);
        auto rosMsg = rawColorPublisher->borrow_loaned_message();
        if (!rosMsg.is_valid()) {
            RCLCPP_ERROR(get_logger(), "Failed to borrow raw color message from middleware!");
            return;
        }
        auto& imageMsg = rosMsg.get();
        imageMsg.header.frame_id = frameId;
        imageMsg.header.stamp = daiTimeToRosTime(frame->getTimestamp());
        imageMsg.encoding = sensor_msgs::image_encodings::BGR8;
        imageMsg.is_bigendian = false;
        imageMsg.height = frame->getHeight(); // todo check if this is overall or per-plane
        imageMsg.width = frame->getWidth();
        imageMsg.step = frame->getStride();
        imageMsg.data.clear();
        imageMsg.data.reserve(frame->getData().size());
        std::copy_n(frame->getData().begin(), frame->getData().size(), imageMsg.data.begin());

        rawColorPublisher->publish(std::move(rosMsg));
    }

public:
    OakDriver() : Node("oak_driver") {
        rawColorPublisher = create_publisher<sensor_msgs::msg::Image>("color_raw", rclcpp::SensorDataQoS());
        colorPublisher = create_publisher<sensor_msgs::msg::Image>("color", rclcpp::SensorDataQoS());
        depthPublisher = create_publisher<sensor_msgs::msg::Image>("depth", rclcpp::SensorDataQoS());
        colorInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("color_info", rclcpp::ServicesQoS());
        depthInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("depth_info", rclcpp::ServicesQoS());
        diagnosticsPublisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::ServicesQoS());

        frameId = declare_parameter<std::string>("frame_id", "");
        const auto width = declare_parameter<int>("width", 512);
        const auto height = declare_parameter<int>("height", 512);
        if (width <= 0) {
            constexpr auto msg = "Width must be greater than zero!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }
        if (height <= 0) {
            constexpr auto msg = "Height must be greater than zero!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }
        const std::pair<uint32_t, uint32_t> size { width, height };

        centerCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        depth = pipeline.create<dai::node::StereoDepth>();
        systemLogger = pipeline.create<dai::node::SystemLogger>();

        const auto rawCenterOutput = centerCamera->requestOutput(
            { 1920, 1080 }, dai::ImgFrame::Type::BGR888i
        );
        const auto centerOutput = centerCamera->requestOutput(
            size, dai::ImgFrame::Type::RGB888p, dai::ImgResizeMode::CROP, std::nullopt, true
        );
        const auto leftOutput = leftCamera->requestOutput(size);
        const auto rightOutput = rightCamera->requestOutput(size);

        centerOutput->link(depth->inputAlignTo);
        leftOutput->link(depth->left);
        rightOutput->link(depth->right);

        systemLogger->setRate(1);

        rawCenterCameraOutputQueue = rawCenterOutput->createOutputQueue();
        centerCameraOutputQueue = centerOutput->createOutputQueue();
        depthOutputQueue = depth->depth.createOutputQueue();
        systemLoggerOutputQueue = systemLogger->out.createOutputQueue();

        rawCenterCameraOutputQueue->addCallback([this](std::shared_ptr<dai::ADatatype> msg) {
            publishRawCenter(msg);
        });
    }
};