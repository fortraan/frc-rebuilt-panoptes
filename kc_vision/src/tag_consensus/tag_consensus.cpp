#include <chrono>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <random>
#include <ranges>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "apecs.h"
#include "utilities.h"

using namespace std::chrono_literals;

using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace {
    const std::string FIXED_FRAME = "field";
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

    std::unordered_map<std::string, Observation> observations;

    bool shouldReject(const geometry_msgs::msg::TransformStamped& transform, const rclcpp::Time& now) const {
        if (now - transform.header.stamp > maxEstimateAge) return true;
        if (transform.transform.translation.z > 1) return true;
        return false;
    }

    void update() {
        const auto now = get_clock()->now();
        observations.clear();
        for (const auto& prefix : posePrefixes) {
            const auto primaryId = prefix + "/primary";
            const auto secondaryId = prefix + "/secondary";
            if (buffer.canTransform(FIXED_FRAME, primaryId, tf2::TimePointZero) &&
                buffer.canTransform(FIXED_FRAME, secondaryId, tf2::TimePointZero)) {
                const auto primaryTransform = buffer.lookupTransform(
                    FIXED_FRAME, primaryId, tf2::TimePointZero
                );
                const auto secondaryTransform = buffer.lookupTransform(
                    FIXED_FRAME, secondaryId, tf2::TimePointZero
                );
                const auto primaryRejected = shouldReject(primaryTransform, now);
                const auto secondaryRejected = shouldReject(secondaryTransform, now);
                if (primaryRejected && secondaryRejected) {
                    // both transforms rejected. don't do anything with this observation.
                    continue;
                }
                if (primaryRejected && !secondaryRejected) {
                    // use only the secondary transform. since the primary transform was rejected,
                    // the secondary transform is promoted to primary.
                    observations.emplace(prefix, Observation {
                        secondaryTransform, std::nullopt
                    });
                }
                if (!primaryRejected && secondaryRejected) {
                    // use only the primary transform
                    observations.emplace(prefix, Observation {
                        primaryTransform, std::nullopt
                    });
                }
                if (!primaryRejected && !secondaryRejected) {
                    // use both
                    observations.emplace(prefix, Observation {
                        primaryTransform, secondaryTransform
                    });
                }
            }
        }

        // todo change this to avoid copying
        std::vector<Observation> observationVector;
        observationVector.reserve(observations.size());
        std::ranges::transform(observations, std::back_inserter(observationVector), [](const auto& pair) {
            return pair.second;
        });
        auto consensus = apecs(observationVector);

        if (consensus) {
            consensus->mean.child_frame_id = consensusFrameId;
            broadcaster.sendTransform(consensus->mean);
            // todo send variance
        }
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