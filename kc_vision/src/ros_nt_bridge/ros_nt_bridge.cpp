#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>

class RosNtBridge : public rclcpp::Node {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    nt::DoubleArrayTopic visionPoseTopic;
    nt::DoubleArrayTopic fusedPoseTopic;

    nt::DoubleArrayPublisher visionPosePublisher;
    nt::DoubleArraySubscriber fusedPoseSubscriber;

    std::shared_ptr<rclcpp::TimerBase> tformTimer;

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
        const nt::TimestampedDoubleArray msg = fusedPoseSubscriber.GetAtomic();

        if (msg.value.size() != 7) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Recieved ill-formed fused pose message from NetworkTables!"
            );
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "field";
        transform.header.stamp = rioToRosTime(msg.serverTime);
        transform.child_frame_id = "fused_pose_estimate";
        transform.transform.translation.x = msg.value[0];
        transform.transform.translation.y = msg.value[1];
        transform.transform.translation.z = msg.value[2];
        transform.transform.rotation.w = msg.value[3];
        transform.transform.rotation.x = msg.value[4];
        transform.transform.rotation.y = msg.value[5];
        transform.transform.rotation.z = msg.value[6];

        broadcaster.sendTransform(transform);
    }

    void publishTransform() {
        const geometry_msgs::msg::TransformStamped visionPoseEstimate = buffer.lookupTransform(
            "field", "vision_pose_estimate", tf2::TimePointZero
        );

        std::array msg {
            visionPoseEstimate.transform.translation.x,
            visionPoseEstimate.transform.translation.y,
            visionPoseEstimate.transform.translation.z,
            visionPoseEstimate.transform.rotation.w,
            visionPoseEstimate.transform.rotation.x,
            visionPoseEstimate.transform.rotation.y,
            visionPoseEstimate.transform.rotation.z
        };

        visionPosePublisher.Set(msg, rosToRioTime(visionPoseEstimate.header.stamp));
    }

    void bridgeTransform() {
        receiveTransform();
        publishTransform();
    }

public:
    RosNtBridge() : Node("ros_nt_bridge"), buffer(get_clock()), listener(buffer, this),
        broadcaster(this)
    {
        const auto ntInstance = nt::NetworkTableInstance::GetDefault();
        // todo parameterize these strings
        visionPoseTopic = ntInstance.GetDoubleArrayTopic("/vision/vision_pose_estimate");
        fusedPoseTopic = ntInstance.GetDoubleArrayTopic("/vision/fused_pose_estimate");

        visionPosePublisher = visionPoseTopic.Publish();
        fusedPoseSubscriber = fusedPoseTopic.Subscribe({ });

        ntInstance.AddTimeSyncListener(true, [this](const nt::Event& event) {
            onTimeSync(event);
        });

        using namespace std::chrono_literals;
        tformTimer = create_wall_timer(10ms, [this] { bridgeTransform(); });
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<RosNtBridge>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}