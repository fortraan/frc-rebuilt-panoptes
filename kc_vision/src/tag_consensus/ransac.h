#ifndef KC_VISION_RANSAC_H
#define KC_VISION_RANSAC_H

#include <vector>

struct Pose {
    double x, y, heading;
};

struct NormalDistribution {
    double mean, standardDeviation;
};

struct Model {
    NormalDistribution x, y, heading;
};

struct RansacConfig {
    int iterations = 10;
    int sampleSize = -1;
    double inlierDeviationThreshold = 1;
};

Model ransac(const std::vector<Pose>& poses, const RansacConfig& config = { });

#endif //KC_VISION_RANSAC_H