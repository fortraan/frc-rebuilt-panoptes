#include <filesystem>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <depthai/depthai.hpp>

class IntakeCam : public rclcpp::Node {
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> markerPublisher;

    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::Camera> centerCam;
    std::shared_ptr<dai::node::Camera> leftCam;
    std::shared_ptr<dai::node::Camera> rightCam;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<dai::node::SpatialDetectionNetwork> detector;
    std::shared_ptr<dai::node::ObjectTracker> objectTracker;

public:
    IntakeCam() : Node("intake_camera") {
        // camera inputs
        centerCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        leftCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        rightCam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        stereo = pipeline.create<dai::node::StereoDepth>();

        const dai::NNArchive archive(
            std::filesystem::path(ament_index_cpp::get_package_share_directory("kc_vision"))
            / "resources" / "bnb-yolov9t-150.rvc2.tar.xz"
        );
        detector = pipeline.create<dai::node::SpatialDetectionNetwork>()->build(centerCam, stereo, archive);

        objectTracker = pipeline.create<dai::node::ObjectTracker>();
        objectTracker->setDetectionLabelsToTrack({ 1 }); // only track robots
        objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
        objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);
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