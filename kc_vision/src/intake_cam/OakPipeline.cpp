#include "OakPipeline.h"

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/SpatialDetectionNetwork.hpp>
//#include <depthai/pipeline/node/SpatialLocationCalculator.hpp>
#include <depthai/pipeline/node/ObjectTracker.hpp>
#include <depthai/pipeline/node/host/Display.hpp>

#include "InferenceEngine.h"

namespace {
    constexpr std::pair<uint32_t, uint32_t> IMAGE_SIZE { 640, 640 };
    constexpr auto WINDOW_NAME = "Intake Camera";
}

struct OakPipelineImpl : OakPipeline {
    dai::Pipeline pipeline;

    std::shared_ptr<dai::node::Camera> centerCamera;
    std::shared_ptr<dai::node::Camera> leftCamera;
    std::shared_ptr<dai::node::Camera> rightCamera;
    std::shared_ptr<dai::node::StereoDepth> depth;
    std::shared_ptr<dai::node::SpatialDetectionNetwork> detectionNetwork;
    //std::shared_ptr<dai::node::SpatialLocationCalculator> locationCalculator;
    std::shared_ptr<dai::node::ObjectTracker> objectTracker;

    std::shared_ptr<dai::node::Display> display;

    //std::shared_ptr<InferenceEngine> inferenceEngine;

    std::shared_ptr<dai::MessageQueue> detectionsQueue;
    std::shared_ptr<dai::MessageQueue> trackingQueue;

    explicit OakPipelineImpl(const std::filesystem::path& enginePath, nvinfer1::ILogger& logger);
    ~OakPipelineImpl() noexcept override;
    void start() override;
    void stop() override;
    void addDetectionCallback(DetectionCallback callback) override;
    void addTrackingCallback(TrackingCallback callback) override;
};

OakPipelineImpl::OakPipelineImpl(const std::filesystem::path& enginePath, nvinfer1::ILogger& logger) {
    centerCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    depth = pipeline.create<dai::node::StereoDepth>();
    const dai::NNModelDescription model { "yolov6-nano" };
    detectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>()->build(centerCamera, depth, model);
    //locationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();
    objectTracker = pipeline.create<dai::node::ObjectTracker>();
    display = pipeline.create<dai::node::Display>(WINDOW_NAME);

    //inferenceEngine = pipeline.create<InferenceEngine>(enginePath, logger);

    //const auto centerStream = centerCamera->requestOutput(IMAGE_SIZE);
    const auto leftStream = leftCamera->requestOutput(IMAGE_SIZE);
    const auto rightStream = rightCamera->requestOutput(IMAGE_SIZE);

    //centerStream->link(inferenceEngine->in);

    leftStream->link(depth->left);
    rightStream->link(depth->right);
    //inferenceEngine->passthrough.link(depth->inputAlignTo);

    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);

    objectTracker->passthroughTrackerFrame.link(display->input);

    detectionNetwork->setConfidenceThreshold(0.6f);
    detectionNetwork->input.setBlocking(false);
    detectionNetwork->setBoundingBoxScaleFactor(0.5f);
    detectionNetwork->setDepthLowerThreshold(100);
    detectionNetwork->setDepthUpperThreshold(5000);

    objectTracker->setDetectionLabelsToTrack({0});  // track only person
    objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    detectionsQueue = detectionNetwork->spatialLocationCalculator->outputDetections.createOutputQueue();
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
        callback(std::static_pointer_cast<dai::SpatialImgDetections>(msg));
    });
}

void OakPipelineImpl::addTrackingCallback(TrackingCallback callback) {
    trackingQueue->addCallback([callback](const std::shared_ptr<dai::ADatatype>& msg) {
        callback(std::static_pointer_cast<dai::Tracklets>(msg));
    });
}

std::unique_ptr<OakPipeline> OakPipeline::create(const std::filesystem::path& enginePath, nvinfer1::ILogger &logger) {
    return std::make_unique<OakPipelineImpl>(enginePath, logger);
}
