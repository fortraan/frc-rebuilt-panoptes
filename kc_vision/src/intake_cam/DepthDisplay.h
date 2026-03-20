#ifndef KC_VISION_DEPTH_DISPLAY_H
#define KC_VISION_DEPTH_DISPLAY_H

#include <string>

#include <depthai/pipeline/ThreadedHostNode.hpp>

class DepthDisplay : public dai::node::CustomThreadedNode<DepthDisplay> {
    double maxDisparity;
    std::string windowName;

public:
    Input in;

    explicit DepthDisplay(double maxDisparity, const std::string& windowName = "Depth");
    ~DepthDisplay() noexcept override;

    void run() override;
};

#endif //KC_VISION_DEPTH_DISPLAY_H