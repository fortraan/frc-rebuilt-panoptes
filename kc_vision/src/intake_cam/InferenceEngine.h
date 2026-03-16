#ifndef KC_VISION_INFERENCE_ENGINE_H
#define KC_VISION_INFERENCE_ENGINE_H

#include <chrono>
#include <filesystem>
#include <memory>
#include <atomic>

#include <NvInfer.h>
#include <thrust/device_ptr.h>

#include <depthai/pipeline/ThreadedHostNode.hpp>

class InferenceEngine : public dai::node::CustomThreadedNode<InferenceEngine> {
    std::unique_ptr<nvinfer1::IRuntime> nvRuntime;
    std::unique_ptr<nvinfer1::ICudaEngine> nvEngine;
    std::unique_ptr<nvinfer1::IExecutionContext> nvContext;

    thrust::device_ptr<float> inputTensor;
    thrust::device_ptr<float> outputTensor;

    cudaStream_t cudaStream;

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

    Input in;
    Output out;
    Output passthrough;

    std::atomic<float> confidenceThreshold = 0.5;

    InferenceEngine(const std::filesystem::path& enginePath, nvinfer1::ILogger& logger);
    ~InferenceEngine() noexcept override;

    void run() override;
};

#endif //KC_VISION_INFERENCE_ENGINE_H