#ifndef KC_VISION_OAK_PIPELINE_H
#define KC_VISION_OAK_PIPELINE_H

#include <depthai/depthai.hpp>

#include "InferenceEngine.h"

class OakPipeline {
    dai::Pipeline pipeline;

public:
    OakPipeline();
};


#endif //KC_VISION_OAK_PIPELINE_H