#include "DepthDisplay.h"

#include <opencv2/highgui.hpp>

#include "utilities.h"

namespace {
    const std::string INPUT_NAME = "in";
}

DepthDisplay::DepthDisplay(const double maxDisparity, std::string name) :
    maxDisparity(maxDisparity), windowName(std::move(name))
{
    sendProcessingToPipeline(true);
}

void DepthDisplay::build(Output* output) {
    output->link(inputs[INPUT_NAME]);
}

void DepthDisplay::onStart() {
    cv::namedWindow(windowName);
}

void DepthDisplay::onStop() {
    cv::destroyWindow(windowName);
}

std::shared_ptr<dai::Buffer> DepthDisplay::processGroup(std::shared_ptr<dai::MessageGroup> in) {
    if (in == nullptr) return nullptr;

    const auto frame = in->get<dai::ImgFrame>(INPUT_NAME);
    if (frame == nullptr) return nullptr;
    KC_DEBUG_ASSERT(frame->getDatatype() == dai::DatatypeEnum::ImgFrame, "Input is not an ImgFrame! UB will occur!");


}

