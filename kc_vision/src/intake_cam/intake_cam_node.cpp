#include <filesystem>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <depthai/depthai.hpp>

#include "InferenceEngine.h"
#include "utilities.h"

namespace {
    // hardcoded for testing
    constexpr auto ENGINE_FILE_PATH = "/home/kernelchaos/Code/FrcScorekeeper/learn/train5/weights/bnb-yolov26n-fp16+fp32.engine";
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

    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::Camera> centerCam;
    std::shared_ptr<dai::node::Camera> leftCam;
    std::shared_ptr<dai::node::Camera> rightCam;
    std::shared_ptr<InferenceEngine> inferenceEngine;
    std::shared_ptr<dai::node::Display> display;
    // std::shared_ptr<dai::node::StereoDepth> stereo;
    // std::shared_ptr<dai::node::SpatialDetectionNetwork> detector;
    // std::shared_ptr<dai::node::ObjectTracker> objectTracker;

    std::shared_ptr<dai::MessageQueue> rawDetections;

    std::shared_ptr<rclcpp::TimerBase> performanceLoggingTimer;

    void onRawDetectionReceived(const std::shared_ptr<dai::ADatatype>& msg) {
        KC_DEBUG_ASSERT_ROS(
            get_logger(), msg->getDatatype() == dai::DatatypeEnum::ImgDetections, "Got wrong raw detection datatype!"
        );
        const auto detections = static_cast<const dai::ImgDetections&>(*msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Detections: %lu", detections.detections.size());
    }

    void logPerformance() const {
        const InferenceEngine::PerformanceMetrics performanceMetrics = inferenceEngine->performanceMetrics;
        RCLCPP_INFO(
            get_logger(),
            "Frame age: %ld us, pre-processing time: %ld us, inference time: %ld us, post-processing time: %ld us",
            performanceMetrics.frameAge.count(), performanceMetrics.preprocessingTime.count(),
            performanceMetrics.inferenceTime.count(), performanceMetrics.postprocessingTime.count()
        );
    }

public:
    IntakeCam() : Node("intake_camera"), nvLogger(get_logger().get_child("TensorRT")) {
        markerPublisher = create_publisher<visualization_msgs::msg::Marker>(
            "fuel_marker", rclcpp::SensorDataQoS()
        );

        // camera inputs
        centerCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        leftCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        rightCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        inferenceEngine = pipeline.create<InferenceEngine>(ENGINE_FILE_PATH, nvLogger);

        const auto centerCamStream = centerCam->requestOutput(
            { 640, 640 }, { }, dai::ImgResizeMode::STRETCH
        );
        centerCamStream->link(inferenceEngine->in);

        //display = pipeline.create<dai::node::Display>();
        //inferenceEngine->passthrough.link(display->input);

        // const auto leftCamOutput = leftCam->requestOutput({ 640, 400 });
        // const auto rightCamOutput = rightCam->requestOutput({ 640, 400 });
        //
        // stereo = pipeline.create<dai::node::StereoDepth>();
        // leftCamOutput->link(stereo->left);
        // rightCamOutput->link(stereo->right);
        //
        // const dai::NNArchive archive(
        //     std::filesystem::path(ament_index_cpp::get_package_share_directory("kc_vision"))
        //     / "resources" / "bnb-yolov9t-150.rvc2.tar.xz"
        // );
        // detector = pipeline.create<dai::node::SpatialDetectionNetwork>()->build(centerCam, stereo, archive);
        // detector->setConfidenceThreshold(0.6f);
        // detector->input.setBlocking(false);
        // detector->setBoundingBoxScaleFactor(0.5f);
        // detector->setDepthLowerThreshold(100);
        // detector->setDepthUpperThreshold(5000);
        //
        // objectTracker = pipeline.create<dai::node::ObjectTracker>();
        // objectTracker->setDetectionLabelsToTrack({ 1 }); // only track robots
        // objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
        // objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

        rawDetections = inferenceEngine->out.createOutputQueue();
        rawDetections->addCallback([this](const std::shared_ptr<dai::ADatatype>& msg) {
            onRawDetectionReceived(msg);
        });

        pipeline.start();

        using namespace std::chrono_literals;
        performanceLoggingTimer = create_wall_timer(10s, [this] {
            logPerformance();
        });
    }

    ~IntakeCam() noexcept override {
        pipeline.stop();
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    try {
        const auto node = std::make_shared<IntakeCam>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error! " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}