#ifndef KC_VISION_APECS_H
#define KC_VISION_APECS_H

#include <optional>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>

struct Observation {
    // each observation must have at least one estimate associated with it. an observation may
    // optionally have a second estimate. this secondary estimate may be used in situations where
    // one of the estimates cannot be rejected as an obvious outlier. if 2 estimates are provided,
    // then the primary estimate shall be the estimate with lower reprojection error.
    geometry_msgs::msg::TransformStamped primaryEstimate;
    std::optional<geometry_msgs::msg::TransformStamped> secondaryEstimate;
};

struct Model {
    geometry_msgs::msg::TransformStamped mean; // todo change this to Eigen types
    Eigen::Vector<double, 6> variance;
};

std::optional<Model> apecs(const std::vector<Observation>& observations);

#endif //KC_VISION_APECS_H