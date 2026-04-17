#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>

#include <frc/geometry/Pose3d.h>

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
    nt::DoubleTopic timeTopic;

    nt::StructPublisher<frc::Pose3d> visionPosePublisher;
    nt::StructSubscriber<frc::Pose3d> fusedPoseSubscriber;
    nt::DoublePublisher timePublisher;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> poseSubscription;
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
        timeTopic = table->GetDoubleTopic("time");

        visionPosePublisher = visionPoseTopic.Publish();
        fusedPoseSubscriber = fusedPoseTopic.Subscribe(DEFAULT_POSE);

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