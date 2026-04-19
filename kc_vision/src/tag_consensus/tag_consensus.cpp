#include <chrono>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <kc_vision_msgs/msg/observation.hpp>

#include "apecs.h"
#include "utilities.h"

using namespace std::chrono_literals;

namespace {
    const std::string FIXED_FRAME = "field";
}

class TagConsensus : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    std::unordered_map<std::string, std::shared_ptr<kc_vision_msgs::msg::Observation>> observations;

    rclcpp::Duration maxEstimateAge;
    double maxReprojectionError;
    double maxRange;
    std::string consensusFrameId;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> posePublisher;
    std::shared_ptr<rclcpp::Subscription<kc_vision_msgs::msg::Observation>> observationSubscriber;
    std::shared_ptr<rclcpp::TimerBase> timer;

    void onObservationReceived(std::unique_ptr<kc_vision_msgs::msg::Observation> observation) {
        observations[observation->id] = std::move(observation);
    }

    bool shouldReject(const geometry_msgs::msg::Pose& pose, double reprojectionError, double range) {
        if (pose.position.z > 1) return true;
        if (pose.position.z < -0.4) return true;
        if (reprojectionError >= maxReprojectionError) return true;
        if (range >= maxRange) return true;
        return false;
    }

    void update() {
        const auto now = get_clock()->now();

        std::vector<Observation> apecsObservations;

        RCLCPP_DEBUG(get_logger(), "Performing APECS with %lu observations", observations.size());

        int rejectedForAge = 0;
        auto iter = observations.begin();
        const auto end = observations.end();
        while  (iter != end) {
            const auto observation = iter->second;
            if (now - observation->header.stamp > maxEstimateAge) {
                // this observation is too old and should be discarded.
                rejectedForAge++;
                iter = observations.erase(iter);
                continue;
            } else {
                iter++;
            }

            // copy into stamped poses because tf2 *INSISTS* on having stamps
            geometry_msgs::msg::PoseStamped primaryTagRelative, secondaryTagRelative;
            primaryTagRelative.header = observation->header;
            primaryTagRelative.pose = observation->primary;
            secondaryTagRelative.header = observation->header;
            secondaryTagRelative.pose = observation->secondary;

            geometry_msgs::msg::PoseStamped primary, secondary;
            buffer.transform(
                primaryTagRelative, primary, FIXED_FRAME,
                tf2::TimePointZero, observation->header.frame_id
            );
            buffer.transform(
                secondaryTagRelative, secondary, FIXED_FRAME,
                tf2::TimePointZero, observation->header.frame_id
            );

            const auto primaryRejected = shouldReject(primary.pose, observation->primary_reprojection_error, observation->primary_range);
            const auto secondaryRejected = shouldReject(secondary.pose, observation->secondary_reprojection_error, observation->secondary_range);

            if (primaryRejected && secondaryRejected) {
                // both transforms rejected. don't do anything with this observation.
                continue;
            }
            if (primaryRejected && !secondaryRejected) {
                // use only the secondary transform. since the primary transform was rejected,
                // the secondary transform is promoted to primary.
                apecsObservations.emplace_back(
                    observation->header.stamp, secondary.pose, std::nullopt
                );
            }
            if (!primaryRejected && secondaryRejected) {
                // use only the primary transform
                apecsObservations.emplace_back(
                    observation->header.stamp, primary.pose, std::nullopt
                );
            }
            if (!primaryRejected && !secondaryRejected) {
                // use both
                apecsObservations.emplace_back(
                    observation->header.stamp, primary.pose, secondary.pose
                );
            }
        }

        RCLCPP_DEBUG(get_logger(), "Rejected %d old observations. %lu left", rejectedForAge, apecsObservations.size());

        const auto consensus = apecs(apecsObservations);
        if (consensus) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = consensus->time;
            transform.header.frame_id = FIXED_FRAME;
            transform.child_frame_id = consensusFrameId;
            transform.transform.translation.x = consensus->mean.position.x;
            transform.transform.translation.y = consensus->mean.position.y;
            transform.transform.translation.z = consensus->mean.position.z;
            transform.transform.rotation = consensus->mean.orientation;
            broadcaster.sendTransform(transform);

            geometry_msgs::msg::PoseWithCovarianceStamped poseWithCovariance;
            poseWithCovariance.header = transform.header;
            poseWithCovariance.pose.pose = consensus->mean;
            std::ranges::copy_n(consensus->covariance.data(), 36, poseWithCovariance.pose.covariance.begin());
            posePublisher->publish(poseWithCovariance);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300, "APECS failed to reach consensus!");
        }
    }

public:
    TagConsensus() : Node("tag_consensus"), buffer(get_clock(), 10s), listener(buffer, this),
        broadcaster(this), maxEstimateAge(0s)
    {
        maxEstimateAge = std::chrono::milliseconds(declare_parameter<int64_t>(
            "max_estimate_age_ms", 60
        ));
        maxReprojectionError = declare_parameter<double>("max_reprojection_error", 10);
        maxRange = declare_parameter<double>("max_range", 4);

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

        observationSubscriber = create_subscription<kc_vision_msgs::msg::Observation>(
            "/observations", rclcpp::SensorDataQoS(),
            [this](std::unique_ptr<kc_vision_msgs::msg::Observation> observation) {
                onObservationReceived(std::move(observation));
            }
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