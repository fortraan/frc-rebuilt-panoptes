#include "Displays.h"

#include <iostream>

#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/SpatialImgDetections.hpp>

#include <fmt/format.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "utilities.h"

namespace {
    const std::string COLOR_WINDOW_NAME = "Color";
    const std::string DEPTH_WINDOW_NAME = "Depth";
    const std::string DETECTIONS_WINDOW_NAME = "Detections";

    void bgrpToBgri(const std::shared_ptr<dai::ImgFrame>& imgFrame, cv::Mat& out) {
        KC_DEBUG_ASSERT(imgFrame->getType() == dai::ImgFrame::Type::BGR888p, "Wrong ImgFrame type!");
        const cv::Size size(static_cast<int>(imgFrame->getWidth()), static_cast<int>(imgFrame->getHeight()));
        uint8_t* imgFrameData = imgFrame->getData().data();
        const cv::Mat blue(size, CV_8UC1, imgFrameData);
        const cv::Mat green(size, CV_8UC1, imgFrameData + size.area());
        const cv::Mat red(size, CV_8UC1, imgFrameData + 2 * size.area());
        const std::array channels { blue, green, red };
        cv::merge(channels, out);
    }
}

Displays::Displays(const double maxDisparity) : maxDisparity(maxDisparity) { }

void Displays::run() {
    static const cv::Scalar ANNOTATION_COLOR { 0xff, 0x00, 0x00 };
    cv::Mat color, depth, invalidDepthMask, colorizedDepth;
    while (mainLoop()) {
        std::shared_ptr<dai::SpatialImgDetections> detections;
        {
            // todo this is *really* prone to locking up the entire system for some reason
            const auto blockEvent = inputBlockEvent();
            bgrpToBgri(inColor.get<dai::ImgFrame>(), color);
            inDepth.get<dai::ImgFrame>()->getFrame().convertTo(depth, CV_8UC1, 255 / maxDisparity);
            detections = inDetections.get<dai::SpatialImgDetections>();
        }

        cv::imshow(COLOR_WINDOW_NAME, color);
        cv::waitKey(1);

        cv::compare(depth, 0, invalidDepthMask, cv::CMP_EQ);
        depth = 255 - depth;
        depth.setTo(0, invalidDepthMask);
        cv::applyColorMap(depth, colorizedDepth, cv::ColormapTypes::COLORMAP_DEEPGREEN);
        cv::imshow(DEPTH_WINDOW_NAME, colorizedDepth);
        cv::waitKey(1);

        for (const auto& detection : detections->detections) {
            const auto [corner, size] = detection
                .getBoundingBox()
                .denormalize(color.cols, color.rows)
                .getOuterXYWH();
            cv::rectangle(color, cv::Rect2f {
                corner.x, corner.y, size.width, size.height
            }, ANNOTATION_COLOR);
            const auto [x, y, z] = detection.spatialCoordinates;
            const auto label = fmt::format(
                "{}: {:.2} {:4}, {:4}, {:4}", detection.labelName, detection.confidence,
                static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)
            );
            cv::putText(
                color, label, cv::Point2f { corner.x, corner.y },
                cv::FONT_HERSHEY_SIMPLEX, 0.75, ANNOTATION_COLOR
            );
        }
        cv::imshow(DETECTIONS_WINDOW_NAME, color);
        cv::waitKey(1);
    }
}
