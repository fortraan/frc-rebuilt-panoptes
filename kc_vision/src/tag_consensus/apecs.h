#ifndef KC_VISION_APECS_H
#define KC_VISION_APECS_H

#include <optional>
#include <vector>

#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Dense>

struct Observation {
    // each observation must have at least one estimate associated with it. an observation may
    // optionally have a second estimate. this secondary estimate may be used in situations where
    // one of the estimates cannot be rejected as an obvious outlier. if 2 estimates are provided,
    // then the primary estimate shall be the estimate with lower reprojection error.
    rclcpp::Time time;
    geometry_msgs::msg::Pose primaryEstimate;
    std::optional<geometry_msgs::msg::Pose> secondaryEstimate;

    // explicitly defined constructors because clang is throwing a fit over list init
    Observation() = default;
    Observation(const rclcpp::Time& time, const geometry_msgs::msg::Pose& primaryEstimate,
                const std::optional<geometry_msgs::msg::Pose>& secondaryEstimate);
};

struct Model {
    rclcpp::Time time;
    geometry_msgs::msg::Pose mean;
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance;
};

std::optional<Model> apecs(const std::vector<Observation>& observations, const rclcpp::Logger& logger);

#endif //KC_VISION_APECS_H