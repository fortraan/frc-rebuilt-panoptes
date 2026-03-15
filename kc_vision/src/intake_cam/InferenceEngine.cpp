#include "InferenceEngine.h"

#include <stdexcept>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#include <fstream>

#include <fmt/format.h>

#include "utilities.h"

namespace {
    constexpr auto INPUT_TENSOR_NAME = "images";
    const nvinfer1::Dims4 INPUT_SHAPE { 1, 3, 640, 640 };
    constexpr auto INPUT_TYPE = nvinfer1::DataType::kFLOAT;

    constexpr auto OUTPUT_TENSOR_NAME = "output0";
    const nvinfer1::Dims3 OUTPUT_SHAPE { 1, 300, 6 };
    constexpr auto OUTPUT_TYPE = nvinfer1::DataType::kFLOAT;
}

NvRosLogger::NvRosLogger(const rclcpp::Logger& logger) : rosLogger(logger) { }

void NvRosLogger::log(const Severity severity, const nvinfer1::AsciiChar* msg) noexcept {
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

InferenceEngine::InferenceEngine(const rclcpp::Node& node, const std::filesystem::path& enginePath) :
    inferenceLogger(node.get_logger().get_child("inference")),
    nvLogger(node.get_logger().get_child("TensorRT"))
{
    nvRuntime = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(nvLogger));

    if (!std::filesystem::exists(enginePath)) {
        throw std::runtime_error(fmt::format("Engine file {} does not exist!", enginePath.string()));
    }
    // read the file into a vector
    const std::size_t serializedEngineSize = std::filesystem::file_size(enginePath);
    std::vector<uint8_t> serializedEngine(serializedEngineSize);
    std::ifstream engineStream(enginePath, std::ios::in | std::ios::binary);
    engineStream.read(reinterpret_cast<char*>(serializedEngine.data()), serializedEngineSize);

    nvEngine = std::unique_ptr<nvinfer1::ICudaEngine>(
        nvRuntime->deserializeCudaEngine(serializedEngine.data(), serializedEngineSize)
    );

    nvContext = std::unique_ptr<nvinfer1::IExecutionContext>(nvEngine->createExecutionContext());

    const nvinfer1::Dims64 inputShape = nvEngine->getTensorShape(INPUT_TENSOR_NAME);
    const nvinfer1::DataType inputType = nvEngine->getTensorDataType(INPUT_TENSOR_NAME);

    const nvinfer1::Dims64 outputShape = nvEngine->getTensorShape(OUTPUT_TENSOR_NAME);
    const nvinfer1::DataType outputType = nvEngine->getTensorDataType(OUTPUT_TENSOR_NAME);

    
}

std::shared_ptr<dai::Buffer> InferenceEngine::processGroup(std::shared_ptr<dai::MessageGroup> in) {
    return { };
}
