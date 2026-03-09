#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <list>
#include <random>
#include <ranges>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <Eigen/Dense>

#include "ransac.h"

using namespace std::chrono_literals;

using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace {
    Pose transformToPose(const TransformStamped& transform) {
        return {
            .x = transform.transform.translation.x,
            .y = transform.transform.translation.y,
            // todo swing twist decomposition
            .heading = 0
        };
    }
}

class TagConsensus : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    std::vector<std::string> estimateFrameIds;
    rclcpp::Duration maxEstimateAge;

    std::shared_ptr<rclcpp::TimerBase> timer;

    void update() {
        const rclcpp::Time now = get_clock()->now();

        std::list<TransformStamped> estimates;

        // get all estimates
        std::ranges::transform(
            estimateFrameIds, std::back_inserter(estimates),
            [&](const std::string& frameId) {
                // look up the newest transform from "field" to the frame in question
                return buffer.lookupTransform(
                    frameId, "field", tf2::TimePointZero, 5ms
                );
            }
        );

        // reject obvious outliers and estimates that are too old
        std::erase_if(estimates, [&](const TransformStamped& transform) {
            if (now - transform.header.stamp > maxEstimateAge) {
                // this estimate is too old. reject it.
                return false;
            }

            if (transform.transform.translation.z > 1) {
                // above 1 meter. unless the robot suddenly learned how to hover,
                // this transform is probably invalid.
                return false;
            }

            // todo other checks

            return true;
        });

        // project transforms into 2d poses
        std::vector<Pose> poses;
        poses.reserve(estimates.size());
        std::ranges::transform(estimates, std::back_inserter(poses), transformToPose);

        const auto [x, y, heading] = ransac(poses);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = "field";
        // todo parameterize
        transform.child_frame_id = "vision_pose_estimate";
        transform.transform.translation.x = x.mean;
        transform.transform.translation.y = y.mean;
        transform.transform.translation.z = 0;
        transform.transform.rotation.w = std::cos(heading.mean / 2);
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = std::sin(heading.mean / 2);

        broadcaster.sendTransform(transform);
    }

public:
    TagConsensus() : Node("tag_consensus"), buffer(get_clock()), listener(buffer, this),
        broadcaster(this), maxEstimateAge(0s)
    {
        // todo change to prefix system
        estimateFrameIds = declare_parameter<std::vector<std::string>>("estimate_frame_ids", { });
        if (estimateFrameIds.empty()) {
            constexpr auto msg = "estimate_frame_ids is empty. There needs to be at least 1 ID "
                                 "for there to be any kind of consensus!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }
        maxEstimateAge = std::chrono::milliseconds(declare_parameter<int64_t>(
            "max_estimate_age_ms", 50
        ));

        timer = create_wall_timer(20ms, [this] { update(); });
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    try {
        const auto node = std::make_shared<TagConsensus>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error! " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}