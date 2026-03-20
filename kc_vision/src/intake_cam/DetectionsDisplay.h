#ifndef KC_VISION_DETECTIONS_DISPLAY_H
#define KC_VISION_DETECTIONS_DISPLAY_H

#include <string>

#include <depthai/pipeline/ThreadedHostNode.hpp>

class DetectionsDisplay : public dai::node::CustomThreadedNode<DetectionsDisplay> {
    std::string windowName;

public:
    Input inDetections;
    Input inImage;

    explicit DetectionsDisplay(const std::string& windowName = "Detections");
    ~DetectionsDisplay() noexcept override;

    void run() override;
};
#endif //KC_VISION_DETECTIONS_DISPLAY_H