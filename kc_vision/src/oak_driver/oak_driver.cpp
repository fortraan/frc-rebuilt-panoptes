#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/SystemInformation.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/SystemLogger.hpp>

namespace {
    std::optional<std::string> daiDistortionModelToRosDistortionModel(const dai::CameraModel model) {
        switch (model) {
            case dai::CameraModel::Perspective:
                return std::nullopt;
            case dai::CameraModel::Fisheye:
                return sensor_msgs::distortion_models::EQUIDISTANT;
            case dai::CameraModel::Equirectangular:
                break;
            case dai::CameraModel::RadialDivision:
                break;
        }
        return std::nullopt;
    }

    std::optional<std::string> daiTypeToRosEncoding(const dai::ImgFrame::Type type) {
        switch (type) {
            case dai::ImgFrame::Type::YUV422i:
                // todo this encoding is deprecated
                return sensor_msgs::image_encodings::YUV422;
            case dai::ImgFrame::Type::YUV444p:
                return std::nullopt;
            case dai::ImgFrame::Type::YUV420p:
                return std::nullopt;
            case dai::ImgFrame::Type::YUV422p:
                return std::nullopt;
            case dai::ImgFrame::Type::YUV400p:
                return std::nullopt;
            case dai::ImgFrame::Type::RGBA8888:
                return sensor_msgs::image_encodings::RGBA8;
            case dai::ImgFrame::Type::RGB161616:
                return sensor_msgs::image_encodings::RGB16;
            case dai::ImgFrame::Type::RGB888p:
                return sensor_msgs::image_encodings::TYPE_8UC1;
            case dai::ImgFrame::Type::BGR888p:
                return sensor_msgs::image_encodings::TYPE_8UC1;
            case dai::ImgFrame::Type::RGB888i:
                return sensor_msgs::image_encodings::RGB8;
            case dai::ImgFrame::Type::BGR888i:
                return sensor_msgs::image_encodings::BGR8;
            case dai::ImgFrame::Type::LUT2:
                return std::nullopt;
            case dai::ImgFrame::Type::LUT4:
                return std::nullopt;
            case dai::ImgFrame::Type::LUT16:
                return std::nullopt;
            case dai::ImgFrame::Type::RAW16:
                return sensor_msgs::image_encodings::TYPE_16UC1;
            case dai::ImgFrame::Type::RAW14:
                return std::nullopt;
            case dai::ImgFrame::Type::RAW12:
                return std::nullopt;
            case dai::ImgFrame::Type::RAW10:
                return std::nullopt;
            case dai::ImgFrame::Type::RAW8:
                return sensor_msgs::image_encodings::TYPE_8UC1;
            case dai::ImgFrame::Type::PACK10:
                return std::nullopt;
            case dai::ImgFrame::Type::PACK12:
                return std::nullopt;
            case dai::ImgFrame::Type::YUV444i:
                return std::nullopt;
            case dai::ImgFrame::Type::NV12:
                return sensor_msgs::image_encodings::NV12;
            case dai::ImgFrame::Type::NV21:
                return sensor_msgs::image_encodings::NV21;
            case dai::ImgFrame::Type::BITSTREAM:
                return std::nullopt;
            case dai::ImgFrame::Type::HDR:
                return std::nullopt;
            case dai::ImgFrame::Type::RGBF16F16F16p:
                return std::nullopt;
            case dai::ImgFrame::Type::BGRF16F16F16p:
                return std::nullopt;
            case dai::ImgFrame::Type::RGBF16F16F16i:
                return std::nullopt;
            case dai::ImgFrame::Type::BGRF16F16F16i:
                return std::nullopt;
            case dai::ImgFrame::Type::GRAY8:
                return sensor_msgs::image_encodings::MONO8;
            case dai::ImgFrame::Type::GRAYF16:
                return sensor_msgs::image_encodings::MONO16;
            case dai::ImgFrame::Type::RAW32:
                return std::nullopt;
            case dai::ImgFrame::Type::NONE:
                return std::nullopt;
        }
        return std::nullopt;
    }
}

class OakDriver : public rclcpp::Node {
    std::string frameId;
    std::string diagnosticsPrefix;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr colorInfoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr planarColorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr planarInfoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPublisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher;

    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::Camera> centerCamera;
    std::shared_ptr<dai::node::Camera> leftCamera;
    std::shared_ptr<dai::node::Camera> rightCamera;
    std::shared_ptr<dai::node::StereoDepth> depth;
    std::shared_ptr<dai::node::SystemLogger> systemLogger;

    std::shared_ptr<dai::MessageQueue> centerCameraOutputQueue;
    std::shared_ptr<dai::MessageQueue> planarCenterCameraOutputQueue;
    std::shared_ptr<dai::MessageQueue> depthOutputQueue;
    std::shared_ptr<dai::MessageQueue> systemLoggerOutputQueue;

    rclcpp::Time daiTimeToRosTime(const std::chrono::steady_clock::time_point& time) {
        // DAI timestamps are relative to std::chrono::steady_clock
        const auto age = std::chrono::steady_clock::now() - time;
        return get_clock()->now() - age;
    }

    bool daiFrameToRosImage(const std::shared_ptr<dai::ImgFrame>& daiFrame, sensor_msgs::msg::Image& rosImage) {
        auto encoding = daiTypeToRosEncoding(daiFrame->getType());
        if (!encoding) return false;

        rosImage.header.stamp = daiTimeToRosTime(daiFrame->getTimestamp());
        rosImage.header.frame_id = frameId;
        rosImage.height = daiFrame->getHeight();
        rosImage.width = daiFrame->getWidth();
        rosImage.encoding = std::move(*encoding);
        rosImage.is_bigendian = false;
        rosImage.step = daiFrame->getStride();
        const auto numBytes = daiFrame->getData().size();
        rosImage.data.clear();
        rosImage.data.resize(numBytes);
        std::copy_n(daiFrame->getData().begin(), numBytes, rosImage.data.begin());

        return true;
    }

    void publishImage(const char* const name, const std::shared_ptr<dai::ADatatype>& daiMsg,
                      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher) {
        if (daiMsg->getDatatype() != dai::DatatypeEnum::ImgFrame) {
            RCLCPP_ERROR(get_logger(), "%s: Received wrong datatype from output queue!", name);
            return;
        }
        const auto frame = std::static_pointer_cast<dai::ImgFrame>(daiMsg);
        auto rosMsg = publisher->borrow_loaned_message();
        if (!rosMsg.is_valid()) {
            RCLCPP_ERROR(get_logger(), "%s: Failed to borrow message from middleware!", name);
            return;
        }
        if (!daiFrameToRosImage(frame, rosMsg.get())) {
            RCLCPP_ERROR(get_logger(), "%s: Failed to convert DAI frame to ROS Image message!", name);
            return;
        }
        publisher->publish(std::move(rosMsg));
    }

    void publishCameraInfo(const char* const name, const std::shared_ptr<dai::ADatatype>& daiMsg,
                           const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& publisher) {
        if (daiMsg->getDatatype() != dai::DatatypeEnum::ImgFrame) {
            RCLCPP_ERROR(get_logger(), "%s: Received wrong datatype from output queue!", name);
            return;
        }
        const auto frame = std::static_pointer_cast<dai::ImgFrame>(daiMsg);
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = daiTimeToRosTime(frame->getTimestamp());
        info.header.frame_id = frameId;
        info.height = frame->getPlaneHeight();
        info.width = frame->getWidth();
        const auto& transformation = frame->transformation;
        info.distortion_model = transformation.getDistortionModel();
    }

    void publishDiagnostics(const std::shared_ptr<dai::ADatatype>& daiMsg) {
        if (daiMsg->getDatatype() != dai::DatatypeEnum::SystemInformation) {
            RCLCPP_ERROR(get_logger(), "Received wrong datatype from SystemLogger!");
            return;
        }
        const auto systemInfo = std::static_pointer_cast<dai::SystemInformation>(daiMsg);

        using namespace diagnostic_msgs::msg;

        DiagnosticArray diagnostics;
        diagnostics.header.stamp = daiTimeToRosTime(systemInfo->getTimestamp());
        const std::string hardwareId = pipeline.getDefaultDevice()->getDeviceId();

        {
            constexpr auto TEMP_WARN_THRESHOLD = 70;
            const auto [css, mss, upa, dss, average] = systemInfo->chipTemperature;
            diagnostic_updater::DiagnosticStatusWrapper temperatureStatus;
            temperatureStatus.name = diagnosticsPrefix + "Temperature";
            temperatureStatus.message = "Temperature of the camera's subsystems";
            temperatureStatus.level =
                css < TEMP_WARN_THRESHOLD && dss < TEMP_WARN_THRESHOLD && mss < TEMP_WARN_THRESHOLD && upa < TEMP_WARN_THRESHOLD
                ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
            temperatureStatus.hardware_id = hardwareId;
            temperatureStatus.addf("Average", "%.2f C", average);
            temperatureStatus.addf("CPU", "%.2f C", css);
            temperatureStatus.addf("DRAM", "%.2f C", dss);
            temperatureStatus.addf("Media", "%.2f C", mss);
            temperatureStatus.addf("SHAVE", "%.2f C", upa);
            diagnostics.status.emplace_back(std::move(temperatureStatus));
        }

        const auto addMemoryUsageStatus = [&](const std::string& name, const dai::MemoryInfo& info){
            constexpr auto MEM_USAGE_WARN_THRESHOLD = 0.8;
            diagnostic_updater::DiagnosticStatusWrapper status;
            status.name = diagnosticsPrefix + name + " Memory Usage";
            status.message = "Memory usage of the " + name + " subsystem";
            status.level = info.used < MEM_USAGE_WARN_THRESHOLD * info.total
                ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
            status.hardware_id = hardwareId;
            status.add("Used", info.used);
            status.add("Remaining", info.remaining);
            status.add("Total", info.total);
            status.addf(
                "Utilization", "%.1f%%",
                100 * static_cast<double>(info.used) / static_cast<double>(info.total)
            );
            diagnostics.status.emplace_back(std::move(status));
        };
        addMemoryUsageStatus("DDR", systemInfo->ddrMemoryUsage);
        addMemoryUsageStatus("CMX", systemInfo->cmxMemoryUsage);
        addMemoryUsageStatus("Leon CSS", systemInfo->leonCssMemoryUsage);
        addMemoryUsageStatus("Leon MSS", systemInfo->leonMssMemoryUsage);

        {
            constexpr auto CPU_USAGE_WARN_THRESHOLD = 0.8;
            const auto css = systemInfo->leonCssCpuUsage.average;
            const auto mss = systemInfo->leonMssCpuUsage.average;
            diagnostic_updater::DiagnosticStatusWrapper cpuUsageStatus;
            cpuUsageStatus.name = diagnosticsPrefix + "CPU Usage";
            cpuUsageStatus.message = "CPU Usage of the camera's subsystems";
            cpuUsageStatus.level = css + mss < CPU_USAGE_WARN_THRESHOLD
                ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
            cpuUsageStatus.hardware_id = hardwareId;
            cpuUsageStatus.addf("Leon CSS", "%.1f%%", 100.0f * css);
            cpuUsageStatus.addf("Leon MSS", "%.1f%%", 100.0f * mss);
            cpuUsageStatus.addf("Total", "%.1f%%", 100.0f * (css + mss));
            diagnostics.status.emplace_back(std::move(cpuUsageStatus));
        }

        diagnosticsPublisher->publish(std::move(diagnostics));
    }

public:
    OakDriver() : Node("oak_driver") {
        colorPublisher = create_publisher<sensor_msgs::msg::Image>(
            "color/image_color", rclcpp::SensorDataQoS()
        );
        colorInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>(
            "color/camera_info", rclcpp::ServicesQoS()
        );
        planarColorPublisher = create_publisher<sensor_msgs::msg::Image>(
            "planar/image_rect", rclcpp::SensorDataQoS()
        );
        planarInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>(
            "planar/camera_info", rclcpp::ServicesQoS()
        );
        depthPublisher = create_publisher<sensor_msgs::msg::Image>(
            "depth/image_rect", rclcpp::SensorDataQoS()
        );
        depthInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>(
            "depth/camera_info", rclcpp::ServicesQoS()
        );
        diagnosticsPublisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::ServicesQoS()
        );

        frameId = declare_parameter<std::string>("frame_id", "");
        diagnosticsPrefix = declare_parameter<std::string>("diagnostics_prefix", "");
        const auto width = declare_parameter<int>("width", 512);
        const auto height = declare_parameter<int>("height", 512);
        if (width <= 0) {
            constexpr auto msg = "Width must be greater than zero!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }
        if (height <= 0) {
            constexpr auto msg = "Height must be greater than zero!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }
        const std::pair<uint32_t, uint32_t> size { width, height };

        centerCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        depth = pipeline.create<dai::node::StereoDepth>();
        systemLogger = pipeline.create<dai::node::SystemLogger>();

        const auto centerOutput = centerCamera->requestOutput(
            { 1920, 1080 }, dai::ImgFrame::Type::BGR888i
        );
        const auto planarCenterOutput = centerCamera->requestOutput(
            size, dai::ImgFrame::Type::RGB888p, dai::ImgResizeMode::CROP, std::nullopt, true
        );
        const auto leftOutput = leftCamera->requestOutput(size);
        const auto rightOutput = rightCamera->requestOutput(size);

        planarCenterOutput->link(depth->inputAlignTo);
        leftOutput->link(depth->left);
        rightOutput->link(depth->right);

        systemLogger->setRate(0.5);

        centerCameraOutputQueue = centerOutput->createOutputQueue();
        planarCenterCameraOutputQueue = planarCenterOutput->createOutputQueue();
        depthOutputQueue = depth->depth.createOutputQueue();
        systemLoggerOutputQueue = systemLogger->out.createOutputQueue();

        centerCameraOutputQueue->addCallback([this](std::shared_ptr<dai::ADatatype> msg) {
            constexpr auto name = "center";
            publishImage(name, msg, colorPublisher);
            static bool hasPublishedInfo = false;
            if (!hasPublishedInfo) {
                hasPublishedInfo = true;
                publishCameraInfo(name, msg, colorInfoPublisher);
            }
        });
        planarCenterCameraOutputQueue->addCallback([this](std::shared_ptr<dai::ADatatype> msg) {
            constexpr auto name = "planar";
            publishImage(name, msg, planarColorPublisher);
            static bool hasPublishedInfo = false;
            if (!hasPublishedInfo) {
                hasPublishedInfo = true;
                publishCameraInfo(name, msg, planarInfoPublisher);
            }
        });
        depthOutputQueue->addCallback([this](std::shared_ptr<dai::ADatatype> msg) {
            constexpr auto name = "depth";
            publishImage(name, msg, depthPublisher);
            static bool hasPublishedInfo = false;
            if (!hasPublishedInfo) {
                hasPublishedInfo = true;
                publishCameraInfo(name, msg, depthInfoPublisher);
            }
        });
        systemLoggerOutputQueue->addCallback([this](std::shared_ptr<dai::ADatatype> msg) {
            publishDiagnostics(msg);
        });
    }
};

