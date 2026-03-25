#ifndef KC_VISION_INFERENCE_ENGINE_H
#define KC_VISION_INFERENCE_ENGINE_H

#include <chrono>
#include <filesystem>
#include <memory>
#include <atomic>

#include <NvInfer.h>
#include <thrust/device_ptr.h>

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/node/Camera.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

class NvRosLogger : public nvinfer1::ILogger {
    rclcpp::Logger rosLogger;
public:
    explicit NvRosLogger(const rclcpp::Logger& logger) : rosLogger(logger) { }
    void log(Severity severity, const nvinfer1::AsciiChar* msg) noexcept override;
};

class InferenceEngine : public dai::node::CustomThreadedNode<InferenceEngine> {
    rclcpp::Logger logger;
    std::shared_ptr<rclcpp::Clock> clock;

    NvRosLogger nvLogger;
    std::unique_ptr<nvinfer1::IRuntime> nvRuntime;
    std::unique_ptr<nvinfer1::ICudaEngine> nvEngine;
    std::unique_ptr<nvinfer1::IExecutionContext> nvContext;

    nvinfer1::Dims64 inputShape;
    size_t inputSize;
    thrust::device_ptr<float> inputTensor;
    thrust::device_ptr<float> outputTensor;

    cudaStream_t cudaStream;

    Input in { *this, { .blocking = false } };
    Output* cameraOutput;

public:
    struct PerformanceMetrics {
        std::chrono::microseconds frameAge;
        std::chrono::microseconds preprocessingTime;
        std::chrono::microseconds inferenceTime;
        std::chrono::microseconds postprocessingTime;
    };

    // Struct containing information on performance of the inference pipeline. Wrapped in std::atomic for thread-safety,
    // as the pipeline logic runs on a background thread.
    std::atomic<PerformanceMetrics> performanceMetrics;

    Output out { *this, { } };
    Output passthrough { *this, { } };

    std::atomic<float> confidenceThreshold = 0.5;

    InferenceEngine(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                    const std::shared_ptr<rclcpp::Clock>& clock);
    ~InferenceEngine() noexcept override;

    void build(const std::shared_ptr<dai::node::Camera>& camera);

    void run() override;
};

#endif //KC_VISION_INFERENCE_ENGINE_H