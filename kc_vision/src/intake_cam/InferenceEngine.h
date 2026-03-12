#ifndef KC_VISION_INFERENCE_ENGINE_H
#define KC_VISION_INFERENCE_ENGINE_H

#include <depthai/pipeline/node/host/HostNode.hpp>

class InferenceEngine : public dai::node::CustomNode<InferenceEngine> {
public:
    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override;

    InferenceEngine();
};


#endif //KC_VISION_INFERENCE_ENGINE_H