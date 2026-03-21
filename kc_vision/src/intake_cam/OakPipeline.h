#ifndef KC_VISION_OAK_PIPELINE_H
#define KC_VISION_OAK_PIPELINE_H

#include <filesystem>
#include <functional>
#include <memory>

#include <depthai/pipeline/datatype/SpatialImgDetections.hpp>
#include <depthai/pipeline/datatype/Tracklets.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

struct OakPipeline {
    using DetectionCallback = std::function<void(const std::shared_ptr<dai::SpatialImgDetections>&)>;
    using TrackingCallback = std::function<void(const std::shared_ptr<dai::Tracklets>&)>;

    static std::unique_ptr<OakPipeline> create(const std::filesystem::path& enginePath, rclcpp::Logger logger,
                                               const std::shared_ptr<rclcpp::Clock>& clock);
    virtual ~OakPipeline() noexcept = default;

    virtual void start() = 0;
    virtual void stop() = 0;

    // the provided callback will be called when a new set of detections is available
    virtual void addDetectionCallback(DetectionCallback callback) = 0;
    virtual void addTrackingCallback(TrackingCallback callback) = 0;
};


#endif //KC_VISION_OAK_PIPELINE_H