#include <filesystem>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>

//#include <depthai/depthai.hpp>

#include "InferenceEngine.h"

class IntakeCam : public rclcpp::Node {
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPublisher;

    std::unique_ptr<InferenceEngine> inferenceEngine;

    // dai::Pipeline pipeline;
    // std::shared_ptr<dai::node::Camera> centerCam;
    // std::shared_ptr<dai::node::Camera> leftCam;
    // std::shared_ptr<dai::node::Camera> rightCam;
    // std::shared_ptr<dai::node::StereoDepth> stereo;
    // std::shared_ptr<dai::node::SpatialDetectionNetwork> detector;
    // std::shared_ptr<dai::node::ObjectTracker> objectTracker;

public:
    IntakeCam() : Node("intake_camera") {
        markerPublisher = create_publisher<visualization_msgs::msg::Marker>(
            "fuel_marker", rclcpp::SensorDataQoS()
        );

        inferenceEngine = std::make_unique<InferenceEngine>(
            *this, "/home/kernelchaos/Code/FrcScorekeeper/learn/train5/weights/bnb-yolov26n-fp16+fp32.engine"
        );

        // camera inputs
        // centerCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        // leftCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        // rightCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        //
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