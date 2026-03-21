#include "OakPipeline.h"

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/SpatialLocationCalculator.hpp>
#include <depthai/pipeline/node/ObjectTracker.hpp>

#include "Displays.h"
#include "InferenceEngine.h"

namespace {
    constexpr std::pair<uint32_t, uint32_t> IMAGE_SIZE { 640, 640 };
}

struct OakPipelineImpl : OakPipeline {
    dai::Pipeline pipeline;

    std::shared_ptr<dai::node::Camera> centerCamera;
    std::shared_ptr<dai::node::Camera> leftCamera;
    std::shared_ptr<dai::node::Camera> rightCamera;
    std::shared_ptr<dai::node::StereoDepth> depth;
    std::shared_ptr<dai::node::SpatialLocationCalculator> locationCalculator;
    std::shared_ptr<dai::node::ObjectTracker> objectTracker;

    std::shared_ptr<Displays> displays;

    std::shared_ptr<InferenceEngine> inferenceEngine;

    std::shared_ptr<dai::MessageQueue> detectionsQueue;
    std::shared_ptr<dai::MessageQueue> trackingQueue;

    OakPipelineImpl(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                    const std::shared_ptr<rclcpp::Clock>& clock);
    ~OakPipelineImpl() noexcept override;
    void start() override;
    void stop() override;
    void addDetectionCallback(DetectionCallback callback) override;
    void addTrackingCallback(TrackingCallback callback) override;
};

OakPipelineImpl::OakPipelineImpl(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                                 const std::shared_ptr<rclcpp::Clock>& clock) {
    centerCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    depth = pipeline.create<dai::node::StereoDepth>();
    locationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();
    objectTracker = pipeline.create<dai::node::ObjectTracker>();
    displays = pipeline.create<Displays>(depth->initialConfig->getMaxDisparity());

    inferenceEngine = pipeline.create<InferenceEngine>(enginePath, logger, clock);
    inferenceEngine->build(centerCamera);

    const auto leftStream = leftCamera->requestOutput(IMAGE_SIZE);
    const auto rightStream = rightCamera->requestOutput(IMAGE_SIZE);

    leftStream->link(depth->left);
    rightStream->link(depth->right);

    depth->depth.link(displays->inDepth);
    depth->depth.link(locationCalculator->inputDepth);

    inferenceEngine->passthrough.link(objectTracker->inputTrackerFrame);
    inferenceEngine->passthrough.link(objectTracker->inputDetectionFrame);
    inferenceEngine->passthrough.link(displays->inColor);
    inferenceEngine->passthrough.link(depth->inputAlignTo);
    inferenceEngine->out.link(locationCalculator->inputDetections);

    locationCalculator->outputDetections.link(objectTracker->inputDetections);
    locationCalculator->outputDetections.link(displays->inDetections);

    // detectionNetwork->setConfidenceThreshold(0.6f);
    // detectionNetwork->input.setBlocking(false);
    // detectionNetwork->setBoundingBoxScaleFactor(0.5f);
    // detectionNetwork->setDepthLowerThreshold(100);
    // detectionNetwork->setDepthUpperThreshold(5000);

    objectTracker->setDetectionLabelsToTrack({0});  // track only person
    objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    detectionsQueue = locationCalculator->out.createOutputQueue();
    trackingQueue = objectTracker->out.createOutputQueue();
}

OakPipelineImpl::~OakPipelineImpl() noexcept { }

void OakPipelineImpl::start() {
    pipeline.start();
}

void OakPipelineImpl::stop() {
    pipeline.stop();
}

void OakPipelineImpl::addDetectionCallback(DetectionCallback callback) {
    detectionsQueue->addCallback([callback](const std::shared_ptr<dai::ADatatype>& msg) {
        if (msg->getDatatype() == dai::DatatypeEnum::SpatialImgDetections) {
            callback(std::static_pointer_cast<dai::SpatialImgDetections>(msg));
        }
    });
}

void OakPipelineImpl::addTrackingCallback(TrackingCallback callback) {
    trackingQueue->addCallback([callback](const std::shared_ptr<dai::ADatatype>& msg) {
        if (msg->getDatatype() == dai::DatatypeEnum::Tracklets) {
            callback(std::static_pointer_cast<dai::Tracklets>(msg));
        }
    });
}

std::unique_ptr<OakPipeline> OakPipeline::create(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                                                 const std::shared_ptr<rclcpp::Clock>& clock) {
    return std::make_unique<OakPipelineImpl>(enginePath, logger, clock);
}
