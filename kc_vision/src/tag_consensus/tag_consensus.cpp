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

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "ransac.h"
#include "utilities.h"

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

    std::vector<std::string> posePrefixes;
    rclcpp::Duration maxEstimateAge;
    std::string consensusFrameId;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> posePublisher;

    std::shared_ptr<rclcpp::TimerBase> timer;

    void update() {
        // todo average pose times
        const rclcpp::Time now = get_clock()->now();

        std::list<TransformStamped> estimates;

        // get all estimates
        for (const auto& frameId : buffer.getAllFrameNames()) {
            // filter to only frame IDs under our prefixes
            if (std::ranges::any_of(posePrefixes, [&frameId](const std::string& prefix) {
                return frameId.starts_with(prefix);
            })) {
                try {
                    // look up the current transform from "field" to the frame in question
                    estimates.emplace_back(buffer.lookupTransform(
                        "field", frameId, tf2::TimePointZero, 5ms
                    ));
                } catch (const tf2::TransformException& exception) {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 200,
                        "Unable to lookup transform from field to %s: %s",
                        frameId.c_str(), exception.what()
                    );
                }
            }
        }

        const int originalNumEstimates = static_cast<int>(estimates.size());
        int rejectedForAge = 0, rejectedForObviousOutlier = 0;
        // reject obvious outliers and estimates that are too old
        estimates.remove_if([&](const TransformStamped& transform) {
            const rclcpp::Duration estimateAge = now - transform.header.stamp;
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Estimate age: %ld ms Threshold: %ld ms",
                estimateAge.to_chrono<std::chrono::milliseconds>().count(),
                maxEstimateAge.to_chrono<std::chrono::milliseconds>().count()
            );
            if (estimateAge > maxEstimateAge) {
                // this estimate is too old. reject it.
                rejectedForAge++;
                return true;
            }

            if (transform.transform.translation.z > 1) {
                // above 1 meter. unless the robot suddenly learned how to hover,
                // this transform is probably invalid.
                rejectedForObviousOutlier++;
                return true;
            }

            // todo other checks

            return false;
        });

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Out of %d poses, rejected %d old poses and %d obvious outliers, leaving %d poses for RANSAC",
            originalNumEstimates, rejectedForAge, rejectedForObviousOutlier,
            originalNumEstimates - rejectedForAge - rejectedForObviousOutlier
        );

        if (estimates.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No poses available for consensus!");
            return;
        }

        // project transforms into 2d poses
        std::vector<Pose> poses;
        poses.reserve(estimates.size());
        std::ranges::transform(estimates, std::back_inserter(poses), transformToPose);

        const auto [x, y, heading] = ransac(poses);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = "field";
        transform.child_frame_id = consensusFrameId;
        transform.transform.translation.x = x.mean;
        transform.transform.translation.y = y.mean;
        transform.transform.translation.z = 0;
        // todo swing twist decomp
        transform.transform.rotation.w = std::cos(heading.mean / 2);
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = std::sin(heading.mean / 2);

        broadcaster.sendTransform(transform);

        geometry_msgs::msg::PoseWithCovarianceStamped poseWithCovariance;
        poseWithCovariance.header.stamp = now;
        poseWithCovariance.header.frame_id = "field";
        poseWithCovariance.pose.pose.position.x = x.mean;
        poseWithCovariance.pose.pose.position.y = y.mean;
        poseWithCovariance.pose.pose.position.z = 0;
        poseWithCovariance.pose.pose.orientation = transform.transform.rotation;
        poseWithCovariance.pose.covariance[0 * 6 + 0] = std::pow(x.standardDeviation, 2); // x variance
        poseWithCovariance.pose.covariance[1 * 6 + 1] = std::pow(y.standardDeviation, 2); // y variance
        poseWithCovariance.pose.covariance[5 * 6 + 5] = std::pow(heading.standardDeviation, 2); // heading variance

        posePublisher->publish(poseWithCovariance);
    }

public:
    TagConsensus() : Node("tag_consensus"), buffer(get_clock(), 10s), listener(buffer, this),
        broadcaster(this), maxEstimateAge(0s)
    {
        posePrefixes = declare_parameter("pose_prefixes", std::vector<std::string>());
        if (posePrefixes.empty()) {
            constexpr auto msg = "pose_prefixes is empty. There needs to be at least 1 prefix"
                                 "for there to be any kind of consensus!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }

        maxEstimateAge = std::chrono::milliseconds(declare_parameter<int64_t>(
            "max_estimate_age_ms", 60
        ));

        const auto updateInterval = std::chrono::milliseconds(declare_parameter<int64_t>(
            "update_interval_ms", 20
        ));

        consensusFrameId = declare_parameter<std::string>("consensus_frame_id", "vision_pose_estimate");
        if (consensusFrameId.empty()) {
            constexpr auto msg = "consensus_frame_id cannot be empty!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }

        posePublisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            consensusFrameId, rclcpp::SensorDataQoS()
        );

        timer = create_wall_timer(updateInterval, [this] { update(); });

        RCLCPP_INFO(get_logger(), "Tag consensus initialized.");
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