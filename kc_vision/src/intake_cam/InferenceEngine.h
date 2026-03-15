#ifndef KC_VISION_INFERENCE_ENGINE_H
#define KC_VISION_INFERENCE_ENGINE_H

#include <filesystem>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <NvInfer.h>

#include <depthai/pipeline/node/host/HostNode.hpp>

class NvRosLogger : public nvinfer1::ILogger {
    rclcpp::Logger rosLogger;
public:
    explicit NvRosLogger(const rclcpp::Logger& logger);
    void log(Severity severity, const nvinfer1::AsciiChar* msg) noexcept override;
};

class InferenceEngine : public dai::node::CustomNode<InferenceEngine> {
    rclcpp::Logger inferenceLogger;
    NvRosLogger nvLogger;

    std::unique_ptr<nvinfer1::IRuntime> nvRuntime;
    std::unique_ptr<nvinfer1::ICudaEngine> nvEngine;
    std::unique_ptr<nvinfer1::IExecutionContext> nvContext;

    void* inputCudaMemory;
    void* outputCudaMemory;

public:
    InferenceEngine(const rclcpp::Node& node, const std::filesystem::path& enginePath);

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override;
};


#endif //KC_VISION_INFERENCE_ENGINE_H