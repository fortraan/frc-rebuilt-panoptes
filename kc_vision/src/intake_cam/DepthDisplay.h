#ifndef KC_VISION_DEPTH_DISPLAY_H
#define KC_VISION_DEPTH_DISPLAY_H

#include <memory>
#include <string>
#include <utility>

#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/host/HostNode.hpp>

#include <opencv2/core/mat.hpp>

class DepthDisplay : public dai::node::CustomNode<DepthDisplay> {
    double maxDisparity;
    std::string windowName;
    cv::Mat buffer;

public:
    explicit DepthDisplay(double maxDisparity, std::string name = "Depth");
    ~DepthDisplay() noexcept override = default;

    void build(Output* output);

    void onStart() override;
    void onStop() override;

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override;
};
#endif //KC_VISION_DEPTH_DISPLAY_H