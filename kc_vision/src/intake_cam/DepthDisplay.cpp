#include "DepthDisplay.h"

#include <depthai/pipeline/datatype/ImgFrame.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "utilities.h"

DepthDisplay::DepthDisplay(const double maxDisparity, const std::string& windowName) : maxDisparity(maxDisparity),
    windowName(windowName),
    in(*this, {
        .name = "in",
        .group = DEFAULT_GROUP,
        .blocking = DEFAULT_BLOCKING,
        .queueSize = DEFAULT_QUEUE_SIZE,
        .types = { { { dai::DatatypeEnum::ImgFrame, true } } },
        .waitForMessage = true
    })
{
    cv::namedWindow(windowName);
}

DepthDisplay::~DepthDisplay() noexcept {
    cv::destroyWindow(windowName);
}

void DepthDisplay::run() {
    const auto queue = in.createInputQueue();

    cv::Mat depthFrame, colorFrame;
    while (mainLoop()) {
        {
            const auto blockEvent = inputBlockEvent();
            const auto daiFrame = in.get<dai::ImgFrame>();
            daiFrame->getFrame().convertTo(depthFrame, CV_8UC1, 255 / maxDisparity);
        }

        cv::applyColorMap(depthFrame, colorFrame, cv::ColormapTypes::COLORMAP_INFERNO);
        cv::imshow(windowName, colorFrame);
    }
}