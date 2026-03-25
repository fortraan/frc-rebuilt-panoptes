#include "InferenceEngine.h"

#include <chrono>
#include <fstream>
#include <span>
#include <stdexcept>
#include <vector>

#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/ImgDetections.hpp>

#include <opencv2/core.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "utilities.h"

namespace {
    constexpr int MAX_NUM_DETECTIONS = 300;
    constexpr int INPUT_IMAGE_WIDTH = 512;
    constexpr int INPUT_IMAGE_HEIGHT = 512;

    constexpr auto INPUT_TENSOR_NAME = "images";
    constexpr nvinfer1::Dims64 INPUT_SHAPE { 4, { 1, 3, INPUT_IMAGE_HEIGHT, INPUT_IMAGE_WIDTH } };
    constexpr auto INPUT_TYPE = nvinfer1::DataType::kFLOAT;

    constexpr auto OUTPUT_TENSOR_NAME = "output0";
    constexpr nvinfer1::Dims64 OUTPUT_SHAPE { 3, { 1, MAX_NUM_DETECTIONS, 6 } };
    constexpr auto OUTPUT_TYPE = nvinfer1::DataType::kFLOAT;

    constexpr std::array LABELS { "fuel", "robot" };

    bool operator==(const nvinfer1::Dims64& a, const nvinfer1::Dims64& b) {
        if (a.nbDims != b.nbDims) return false;
        for (int32_t i = 0; i < a.nbDims; i++) {
            if (a.d[i] != b.d[i]) return false;
        }
        return true;
    }

    std::size_t tensorSize(const nvinfer1::DataType type, const nvinfer1::Dims64& dims) {
        // some data types are 4 bits, so the type size (in bytes) is multiplied by 2 and stored here.
        std::size_t typeSizeX2 = 0;
        switch (type) {
            case nvinfer1::DataType::kINT64:
                typeSizeX2 = 8 * 2;
                break;
            case nvinfer1::DataType::kFLOAT:
            case nvinfer1::DataType::kINT32:
                typeSizeX2 = 4 * 2;
                break;
            case nvinfer1::DataType::kHALF:
            case nvinfer1::DataType::kBF16:
                typeSizeX2 = 2 * 2;
                break;
            case nvinfer1::DataType::kFP8:
            // case nvinfer1::DataType::kE8M0:
            case nvinfer1::DataType::kUINT8:
            case nvinfer1::DataType::kINT8:
            case nvinfer1::DataType::kBOOL:
                typeSizeX2 = 1 * 2;
                break;
            // case nvinfer1::DataType::kFP4:
            case nvinfer1::DataType::kINT4:
                typeSizeX2 = 1;
                break;
        }

        std::size_t ret = typeSizeX2;
        for (int32_t i = 0; i < dims.nbDims; i++) {
            ret *= dims.d[i];
        }
        // since we multiplied by typeSize * 2, divide by 2
        ret /= 2;

        return ret;
    }

    constexpr const char* cudaDataTypeToStr(const nvinfer1::DataType type) {
        switch (type) {
            case nvinfer1::DataType::kFLOAT:
                return "FLOAT";
            case nvinfer1::DataType::kHALF:
                return "HALF";
            case nvinfer1::DataType::kINT8:
                return "INT8";
            case nvinfer1::DataType::kINT32:
                return "INT32";
            case nvinfer1::DataType::kBOOL:
                return "BOOL";
            case nvinfer1::DataType::kUINT8:
                return "UINT8";
            case nvinfer1::DataType::kFP8:
                return "FP8";
            case nvinfer1::DataType::kBF16:
                return "BF16";
            case nvinfer1::DataType::kINT64:
                return "INT64";
            case nvinfer1::DataType::kINT4:
                return "INT4";
            // case nvinfer1::DataType::kFP4:
            //     return "FP4";
            // case nvinfer1::DataType::kE8M0:
            //     return "E8M0";
            default:
                return "Unknown";
        }
    }
}

void NvRosLogger::log(const Severity severity, const nvinfer1::AsciiChar *msg) noexcept {
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

InferenceEngine::InferenceEngine(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                                 const std::shared_ptr<rclcpp::Clock>& clock) :
    logger(logger), clock(clock), nvLogger(logger.get_child("TensorRT")), cudaStream()
{
    setAlias("InferenceEngine");
    in.setName("inferenceInput");
    out.setName("inferenceOutput");
    passthrough.setName("inferencePassthrough");

    in.setBlocking(false);
    in.setMaxSize(2);

    nvRuntime = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(nvLogger));

    if (!std::filesystem::exists(enginePath)) {
        throw std::runtime_error(fmt::format("Engine file {} does not exist!", enginePath.string()));
    }
    // read the file into a vector
    const std::size_t serializedEngineSize = std::filesystem::file_size(enginePath);
    std::vector<uint8_t> serializedEngine(serializedEngineSize);
    std::ifstream engineStream(enginePath, std::ios::in | std::ios::binary);
    KC_DEBUG_ASSERT_ROS(
        logger,
        // max is at least 4GB, so this will probably never happen. however, it's the policy of this project that
        // assumptions are explicitly stated using assertions.
        serializedEngineSize < std::numeric_limits<std::streamsize>::max(),
        "Serialized engine is larger than maximum value of std::streamsize and will be truncated!"
    );
    engineStream.read(
        reinterpret_cast<char*>(serializedEngine.data()),
        static_cast<std::streamsize>(serializedEngineSize)
    );

    nvEngine = std::unique_ptr<nvinfer1::ICudaEngine>(
        nvRuntime->deserializeCudaEngine(serializedEngine.data(), serializedEngineSize)
    );
    nvContext = std::unique_ptr<nvinfer1::IExecutionContext>(nvEngine->createExecutionContext());

    inputShape = nvEngine->getTensorShape(INPUT_TENSOR_NAME);
    const nvinfer1::DataType inputType = nvEngine->getTensorDataType(INPUT_TENSOR_NAME);
    inputSize = tensorSize(inputType, inputShape);

    const nvinfer1::Dims64 outputShape = nvEngine->getTensorShape(OUTPUT_TENSOR_NAME);
    const nvinfer1::DataType outputType = nvEngine->getTensorDataType(OUTPUT_TENSOR_NAME);
    const std::size_t outputBufferSize = tensorSize(outputType, outputShape);

    RCLCPP_INFO(logger, "%s", fmt::format(
        "Input tensor \"{}\" is a {}:[{}] {}. Output tensor \"{}\" is a {}:[{}] {}",
        INPUT_TENSOR_NAME, inputShape.nbDims, fmt::join(inputShape.d, ", "), cudaDataTypeToStr(inputType),
        OUTPUT_TENSOR_NAME, outputShape.nbDims, fmt::join(outputShape.d, ", "), cudaDataTypeToStr(outputType)
    ).c_str());

    KC_DEBUG_ASSERT_ROS(logger, inputShape == INPUT_SHAPE, "Incorrect input shape!");
    KC_DEBUG_ASSERT_ROS(logger, inputType == INPUT_TYPE, "Incorrect input type!");
    KC_DEBUG_ASSERT_ROS(logger, outputShape == OUTPUT_SHAPE, "Incorrect output shape!");
    KC_DEBUG_ASSERT_ROS(logger, outputType == OUTPUT_TYPE, "Incorrect output type!");

    cudaError_t cudaResult;
    void* cudaPtr = nullptr;

    // the Jetson has unified memory, so we can use cudaMallocManaged to share memory between the CPU and GPU.
    // this provides a significant performance improvement. memory allocated by cudaMallocManaged is freed the
    // same way as with cudaMalloc, so it can be given to a thrust::device_ptr for automatic lifetime management.
    cudaResult = cudaMallocManaged(&cudaPtr, inputSize);
    if (cudaResult != cudaSuccess) {
        constexpr auto msg = "Failed to allocate GPU buffer for input tensor!";
        RCLCPP_FATAL(logger, msg);
        throw std::runtime_error(msg);
    }
    inputTensor = thrust::device_ptr<float>(static_cast<float*>(cudaPtr));

    cudaResult = cudaMallocManaged(&cudaPtr, outputBufferSize);
    if (cudaResult != cudaSuccess) {
        constexpr auto msg = "Failed to allocate GPU buffer for output tensor!";
        RCLCPP_FATAL(logger, msg);
        throw std::runtime_error(msg);
    }
    outputTensor = thrust::device_ptr<float>(static_cast<float*>(cudaPtr));

    nvContext->setTensorAddress(INPUT_TENSOR_NAME, inputTensor.get());
    nvContext->setTensorAddress(OUTPUT_TENSOR_NAME, outputTensor.get());

    // todo should this be non-blocking? (cudaStreamNonBlocking)
    cudaResult = cudaStreamCreate(&cudaStream);
    if (cudaResult != cudaSuccess) {
        constexpr auto msg = "Failed to create CUDA stream!";
        RCLCPP_FATAL(logger, msg);
        throw std::runtime_error(msg);
    }
}

InferenceEngine::~InferenceEngine() noexcept {
    cudaStreamDestroy(cudaStream);
}

void InferenceEngine::build(const std::shared_ptr<dai::node::Camera> &camera) {
    cameraOutput = camera->requestOutput(
        { inputShape.d[3], inputShape.d[2] }, dai::ImgFrame::Type::RGB888p,
        dai::ImgResizeMode::CROP, std::nullopt, true
    );
    if (cameraOutput == nullptr) {
        constexpr auto msg = "Could not create camera output!";
        RCLCPP_FATAL(logger, msg);
        throw std::runtime_error(msg);
    }
    in.setBlocking(false);
    cameraOutput->link(in);
}

void InferenceEngine::run() {
    cv::Mat input(inputShape.d[1] * inputShape.d[2], inputShape.d[3], CV_32FC1, inputTensor.get());

    while (mainLoop()) {
        std::shared_ptr<dai::ImgFrame> frame;

        {
            auto blockEvent = inputBlockEvent();
            frame = in.get<dai::ImgFrame>();
        }

        if (frame == nullptr) {
            //KC_DEBUG_ASSERT_ROS(logger, in.isConnected(), "Input is not connected!");
            RCLCPP_WARN_THROTTLE(logger, *clock, 500, "Failed to get frame from input!");
            continue;
        }

        const auto frameAge = std::chrono::steady_clock::now() - frame->getTimestamp();

        using Clock = std::chrono::high_resolution_clock;
        const auto start = Clock::now();

        const auto cvFrame = frame->getFrame();
        KC_DEBUG_ASSERT_ROS(logger, cvFrame.cols == inputShape.d[3], "Input width is incorrect!");
        KC_DEBUG_ASSERT_ROS(logger, cvFrame.rows == inputShape.d[1] * inputShape.d[2], "Input height is incorrect!");
        frame->getFrame().convertTo(input, CV_32FC1, 1.0 / 255.0);

        const auto preprocessingEnd = Clock::now();

        // run inference
        nvContext->enqueueV3(cudaStream);
        // wait for inference to complete. this must be done before reading from outputTensor
        cudaStreamSynchronize(cudaStream);

        const auto inferenceEnd = Clock::now();

        struct __attribute__((__packed__)) Detection {
            // top left corner
            float x0;
            float y0;
            // top right corner
            float x1;
            float y1;
            float confidence;
            float cls;
        };
        // map a struct to the output tensor
        const auto detectionPtr = reinterpret_cast<const Detection*>(outputTensor.get());

        const auto detectionsMsg = std::make_shared<dai::ImgDetections>();
        KC_DEBUG_ASSERT_ROS(logger, detectionsMsg->detections.empty(), "Detections isn't empty!");
        for (int i = 0; i < MAX_NUM_DETECTIONS; i++) {
            const auto& [x0, y0, x1, y1, confidence, cls] = detectionPtr[i];

            if (x0 == x1 || y0 == y1) continue;
            if (confidence < confidenceThreshold) continue;

            const auto labelIndex = static_cast<uint32_t>(std::round(cls));
            std::string labelName;
            if (labelIndex < LABELS.size()) {
                labelName = LABELS[labelIndex];
            }
            dai::ImgDetection detection = {
                dai::RotatedRect {
                    {
                        dai::Point2f { x0, y0, false },
                        dai::Point2f { x1, y1, false }
                    },
                    0
                },
                labelName, confidence, labelIndex
            };
            KC_DEBUG_ASSERT_ROS(logger, detection.getWidth() != 0, "Detection width is 0!");
            KC_DEBUG_ASSERT_ROS(logger, detection.getHeight() != 0, "Detection height is 0!");
            detectionsMsg->detections.emplace_back(std::move(detection));
        }
        detectionsMsg->transformation = frame->transformation;

        const auto postprocessingEnd = Clock::now();

        RCLCPP_DEBUG_THROTTLE(logger, *clock, 2000, "Detections: %lu", detectionsMsg->detections.size());

        out.send(detectionsMsg);

        // performanceMetrics is assigned to as an atomic operation. PerformanceMetrics is trivially copyable, so
        // assigning the entire struct is no different from assigning to the variables within, performance-wise.
        using namespace std::chrono;
        PerformanceMetrics metrics {
            // casting to microseconds loses some precision (default is nanoseconds), which requires an explicit
            // cast. if we kept it in nanoseconds, it would implicitly convert
            .frameAge = duration_cast<microseconds>(frameAge),
            .preprocessingTime = duration_cast<microseconds>(preprocessingEnd - start),
            .inferenceTime = duration_cast<microseconds>(inferenceEnd - preprocessingEnd),
            .postprocessingTime = duration_cast<microseconds>(postprocessingEnd - inferenceEnd)
        };
        performanceMetrics = metrics;

        RCLCPP_INFO(
            logger, "Detected: %lu Inference Performance: Pre-processing: %ld Processing: %ld Post-processing: %ld",
            detectionsMsg->detections.size(), metrics.preprocessingTime.count(), metrics.inferenceTime.count(),
            metrics.postprocessingTime.count()
        );

        passthrough.send(frame);
    }
}
