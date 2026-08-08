#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <kc_vision_msgs/msg/clumps.hpp>
#include <kc_vision_msgs/msg/clump.hpp>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rectangle2d.h>
#include <wpiutil/wpi/struct/Struct.h>

#include "nt_structs.h"

namespace {
    using namespace units::literals;
    constexpr frc::Pose3d DEFAULT_POSE(
        frc::Translation3d(8.259_m, 4.0215_m, 0_m),
        frc::Rotation3d()
    );
}


class RosNtBridge : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    nt::NetworkTableInstance ntInstance;

    nt::StructTopic<frc::Pose3d> visionPoseTopic;
    nt::StructTopic<frc::Pose3d> fusedPoseTopic;
    nt::StructArrayTopic<FuelClump> fuelClumpTopic;
    nt::DoubleTopic timeTopic;

    nt::StructPublisher<frc::Pose3d> visionPosePublisher;
    nt::StructSubscriber<frc::Pose3d> fusedPoseSubscriber;
    nt::StructArrayPublisher<FuelClump> fuelClumpPublisher;
    nt::DoublePublisher timePublisher;



    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> poseSubscription;
    std::shared_ptr<rclcpp::Subscription<kc_vision_msgs::msg::Clumps>> clumpsSubscription;
    std::shared_ptr<rclcpp::TimerBase> updateTimer;

    rclcpp::Time rioToRosTime(int64_t rioTimeMicros) {
        // todo
        return rclcpp::Time();
    }

    int64_t rosToRioTime(const rclcpp::Time& rosTime) {
        // todo
        return 0;
    }

    void onTimeSync(const nt::Event& event) {
        // note: this is called on a background thread!
        if (event.Is(nt::EventFlags::kTimeSync)) {
            rclcpp::Time rosSyncTime = get_clock()->now();
            int64_t rioSyncTime = event.GetTimeSyncEventData()->serverTimeOffset;
            // todo
        }
    }

    void receiveTransform() {
        const auto pose = fusedPoseSubscriber.Get();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "field";
        transform.header.stamp = get_clock()->now();
        transform.child_frame_id = "robot";
        transform.transform.translation.x = pose.X().value();
        transform.transform.translation.y = pose.Y().value();
        transform.transform.translation.z = pose.Z().value();
        const auto quat = pose.Rotation().GetQuaternion();
        transform.transform.rotation.w = quat.W();
        transform.transform.rotation.x = quat.X();
        transform.transform.rotation.y = quat.Y();
        transform.transform.rotation.z = quat.Z();

        broadcaster.sendTransform(transform);
    }

    void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
        //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "Pose Received");

        const frc::Pose3d pose3d(
            frc::Translation3d(
                units::meter_t(pose.pose.pose.position.x),
                units::meter_t(pose.pose.pose.position.y),
                units::meter_t(pose.pose.pose.position.z)
            ), frc::Rotation3d(frc::Quaternion(
                pose.pose.pose.orientation.w,
                pose.pose.pose.orientation.x,
                pose.pose.pose.orientation.y,
                pose.pose.pose.orientation.z
            ))
        );

        visionPosePublisher.Set(pose3d);
    }

    void onClumpsReceived(const kc_vision_msgs::msg::Clumps& msg) {
        const auto numClumps = msg.clumps.size();

        std::vector<FuelClump> clumpsVec{};
        clumpsVec.reserve(numClumps);

        for (int i = 0; i < numClumps; i++) {
            const auto clump = msg.clumps.at(i);
            frc::Translation2d centroid = frc::Translation2d(
                units::meter_t{clump.centroid.x},
                units::meter_t{clump.centroid.y}
            );

            frc::Translation2d boundsTopLeftCorner = frc::Translation2d(
                units::meter_t{clump.bounding_box.x},
                units::meter_t{clump.bounding_box.y}
            );
            frc::Translation2d boundsBottomRightCorner = boundsTopLeftCorner + frc::Translation2d(
                units::meter_t{clump.bounding_box.width},
                units::meter_t{clump.bounding_box.height}
            );

            clumpsVec.emplace_back(FuelClump {
                centroid,
                frc::Rectangle2d(
                    boundsTopLeftCorner,
                    boundsBottomRightCorner
                ),
                clump.area,
                clump.label
            });
        }
        fuelClumpPublisher.Set(clumpsVec);
    }

    void publishTime() {
        timePublisher.Set(get_clock()->now().seconds());
    }

public:
    RosNtBridge() : Node("ros_nt_bridge"), buffer(get_clock()), listener(buffer, this),
        broadcaster(this)
    {
        ntInstance = nt::NetworkTableInstance::Create();

        // todo parameterize these strings
        const auto table = ntInstance.GetTable("vision");
        visionPoseTopic = table->GetStructTopic<frc::Pose3d>("vision_pose_estimate");
        fusedPoseTopic = table->GetStructTopic<frc::Pose3d>("fused_pose_estimate");
        fuelClumpTopic = table->GetStructArrayTopic<FuelClump>("fuel_clumps");
        timeTopic = table->GetDoubleTopic("time");

        visionPosePublisher = visionPoseTopic.Publish();
        fusedPoseSubscriber = fusedPoseTopic.Subscribe(DEFAULT_POSE);
        fuelClumpPublisher = fuelClumpTopic.Publish();

        ntInstance.StartClient4("orin");
        ntInstance.SetServerTeam(6419);


        // ntInstance.AddTimeSyncListener(true, [this](const nt::Event& event) {
        //     onTimeSync(event);
        // });

        poseSubscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/visual_pose_estimate", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
                onPoseReceived(pose);
            }
        );

        clumpsSubscription = create_subscription<kc_vision_msgs::msg::Clumps>(
            "/intake_camera/clumps", rclcpp::SensorDataQoS(),
            [this](const kc_vision_msgs::msg::Clumps& msg) {
                onClumpsReceived(msg);
            }
        );

        using namespace std::chrono_literals;
        updateTimer = create_wall_timer(10ms, [this] {
            publishTime();
            receiveTransform();
        });
    }

    ~RosNtBridge() override {
        nt::NetworkTableInstance::Destroy(ntInstance);
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<RosNtBridge>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}