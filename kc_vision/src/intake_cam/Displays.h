#ifndef KC_VISION_DISPLAYS_H
#define KC_VISION_DISPLAYS_H

#include <depthai/pipeline/ThreadedHostNode.hpp>

class Displays : public dai::node::CustomThreadedNode<Displays> {
    double maxDisparity;

public:
    Input inColor { *this, { .blocking = false } };
    Input inDepth { *this, { .blocking = false } };
    Input inDetections { *this, { .blocking = false } };

    Displays(double maxDisparity);
    ~Displays() noexcept override = default;

    void run() override;
};

#endif //KC_VISION_DISPLAYS_H