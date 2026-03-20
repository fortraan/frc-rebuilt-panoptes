#include "DetectionsDisplay.h"

#include <depthai/pipeline/datatype/SpatialImgDetections.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fmt/format.h>

DetectionsDisplay::DetectionsDisplay(const std::string& windowName) : windowName(windowName),
    inDetections(*this, {
        .name = "detections",
        .group = DEFAULT_GROUP,
        .blocking = DEFAULT_BLOCKING,
        .queueSize = DEFAULT_QUEUE_SIZE,
        .types = { { { dai::DatatypeEnum::SpatialImgDetections, true } } },
        .waitForMessage = true
    }),
    inImage(*this, {
        .name = "image",
        .group = DEFAULT_GROUP,
        .blocking = DEFAULT_BLOCKING,
        .queueSize = DEFAULT_QUEUE_SIZE,
        .types = { { { dai::DatatypeEnum::ImgFrame, true } } },
        .waitForMessage = true
    })
{
    cv::namedWindow(windowName);
}

DetectionsDisplay::~DetectionsDisplay() noexcept {
    cv::destroyWindow(windowName);
}

void DetectionsDisplay::run() {
    static const cv::Scalar ANNOTATION_COLOR { 0xff, 0x00, 0x00 };
    cv::Mat frame;
    while (mainLoop()) {
        std::shared_ptr<dai::SpatialImgDetections> detections;
        {
            const auto blockEvent = inputBlockEvent();
            detections = inDetections.get<dai::SpatialImgDetections>();
            inImage.get<dai::ImgFrame>()->getFrame().copyTo(frame);
        }

        for (const auto& detection : detections->detections) {
            const auto [corner, size] = detection
                .getBoundingBox()
                .denormalize(frame.cols, frame.rows)
                .getOuterXYWH();
            cv::rectangle(frame, cv::Rect2f {
                corner.x, corner.y, size.width, size.height
            }, ANNOTATION_COLOR);
            const auto [x, y, z] = detection.spatialCoordinates;
            const auto label = fmt::format(
                "{}: {} {}, {}, {}", detection.labelName, detection.confidence, x, y, z
            );
            cv::putText(
                frame, label, cv::Point2f { corner.x, corner.y },
                cv::FONT_HERSHEY_SIMPLEX, 8, ANNOTATION_COLOR
            );
        }

        cv::imshow(windowName, frame);
    }
}