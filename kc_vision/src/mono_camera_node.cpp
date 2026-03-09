#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include "common/utilities.h"

// list of todos:
// undistortion before apriltag detection
// maybe change everything to float instead of double

namespace {
    constexpr auto DEFAULT_CAMERA_PATH = "/dev/video0";

    constexpr size_t CAMERA_MATRIX_SIZE = 9;
    constexpr std::array<size_t, 6> DISTORTION_COEFFICIENTS_ALLOWED_SIZES { 0, 4, 5, 8, 12, 14 };

    // todo units
    constexpr double TAG_SIZE = 6.5;

    const std::vector TAG_3D_POINTS {
        cv::Point3d { -TAG_SIZE / 2,  TAG_SIZE / 2, 0 },
        cv::Point3d {  TAG_SIZE / 2,  TAG_SIZE / 2, 0 },
        cv::Point3d {  TAG_SIZE / 2, -TAG_SIZE / 2, 0 },
        cv::Point3d { -TAG_SIZE / 2, -TAG_SIZE / 2, 0}
    };

    cv::Quatd rodriguesToQuaternion(const cv::Matx31d& vector) {
        const double theta = cv::norm(vector);
        return {
            std::cos(theta / 2),
            vector(0) * std::sin(theta / 2) / theta,
            vector(1) * std::sin(theta / 2) / theta,
            vector(2) * std::sin(theta / 2) / theta
        };
    }
}

class MonoCamera : public rclcpp::Node {
    std::string cameraFrameId;

    tf2_ros::TransformBroadcaster transformBroadcaster;

    cv::VideoCapture videoCapture;

    cv::Matx33d cameraMatrix;
    std::vector<double> distortionCoefficients;

    apriltag_detector_t* tagDetector;
    apriltag_family_t* tagFamily;

    std::atomic_bool workerThreadShouldExit;
    std::thread workerThread;

    void processDetection(const apriltag_detection_t* detection, const rclcpp::Time& frameAcquisitionTime) {
        cv::Matx<double, 4, 2> imgPoints;
        // detections returned by apriltag are in the following order:
        // bottom left, bottom right, top right, top left
        // OpenCV expects the corners in the following order:
        // top left, top right, bottom right, bottom left
        const auto assignFromApriltag = [&imgPoints, &detection](const int cvIdx, const int aprIndex) {
            imgPoints(cvIdx, 0) = detection->p[aprIndex][0];
            imgPoints(cvIdx, 1) = detection->p[aprIndex][1];
        };
        assignFromApriltag(0, 3);
        assignFromApriltag(1, 2);
        assignFromApriltag(2, 1);
        assignFromApriltag(3, 0);

        // compute possible poses
        std::vector<cv::Matx31d> translations, rotations;
        const int numPoses = cv::solvePnPGeneric(
            TAG_3D_POINTS, imgPoints, cameraMatrix, distortionCoefficients,
            rotations, translations, false, cv::SOLVEPNP_IPPE_SQUARE
        );

        KC_DEBUG_ASSERT_ROS(get_logger(), translations.size() != numPoses, "Incorrect number of translations!");
        KC_DEBUG_ASSERT_ROS(get_logger(), rotations.size() != numPoses, "Incorrect number of rotations!");

        for (int i = 0; i < numPoses; i++) {
            // publish transform
            geometry_msgs::msg::TransformStamped transform;

            // static frame of the tag on the field
            transform.header.frame_id = fmt::format("tag_{}", detection->id);
            // dynamic frame of the estimate of the camera's pose
            transform.child_frame_id = fmt::format("{}_estimate{}", cameraFrameId, i);
            // tf2 will automatically handle interpolating between times when calculating transforms, so the timestamp
            // of the transform needs to reflect when the measurement was taken. for is, that time is when we got the
            // frame from the camera.
            transform.header.stamp = frameAcquisitionTime;

            // the transform provided by solvePnP is from the camera to the tag. we need from the tag to the camera,
            // so we need to do a bit of math to invert the transformation. for the translation vector, that's as
            // simple as just negating it. for rotation, we take the conjugate of the quaternion to invert the rotation.
            transform.transform.translation.x = -translations[i](0);
            transform.transform.translation.y = -translations[i](1);
            transform.transform.translation.z = -translations[i](2);

            // quaternion conjugate: negate imaginary components
            // q = xi + yj + zk + w
            // q' = w - xi - yj - zk
            cv::Quatd quat = rodriguesToQuaternion(rotations[i]).conjugate();
            transform.transform.rotation.w = quat.w;
            transform.transform.rotation.x = quat.x;
            transform.transform.rotation.y = quat.y;
            transform.transform.rotation.z = quat.z;

            // in: transform from camera to tag
            // fixed frame: camera
            // target frame: robot frame
            // out: transform from robot to tag
            // needed: tag to robot
            // tag is parent in tree to produce robot pose estimate from tag
            //buffer.transform()

            // send it!
            transformBroadcaster.sendTransform(transform);
        }
    }

    void run() {
        cv::Mat frame, gray;

        auto lastFrameTime = get_clock()->now();
        while (!workerThreadShouldExit) {
            if (!videoCapture.read(frame)) continue;
            const rclcpp::Time thisFrameTime = get_clock()->now();

            const rclcpp::Time start = get_clock()->now();

            // convert to grayscale
            // todo it might be possible to have VideoCapture convert directly to this during decoding
            // look into cv::CAP_PROP_FORMAT
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            image_u8_t aprilImg { gray.cols, gray.rows, gray.cols, gray.data };
            const zarray_t* const detections = apriltag_detector_detect(tagDetector, &aprilImg);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t* detection = nullptr;
                zarray_get(detections, i, &detection);

                processDetection(detection, thisFrameTime);
            }

            const rclcpp::Time end = get_clock()->now();

            const auto latency = (end - start).to_chrono<std::chrono::milliseconds>();
            const double framerate = 1e6 / static_cast<double>(
                (thisFrameTime - lastFrameTime).to_chrono<std::chrono::microseconds>().count()
            );
            lastFrameTime = thisFrameTime;

            RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000,
                "Pipeline latency: %ld ms @ %f FPS", latency.count(), framerate
            );
        }
    }

public:
    MonoCamera() : Node("mono_camera"), transformBroadcaster(this), workerThreadShouldExit(false) {
        // declare and retrieve parameters
        cameraFrameId = declare_parameter<std::string>("camera_frame_id", "");
        const auto capturePath = declare_parameter<std::string>(
            "camera_path", DEFAULT_CAMERA_PATH
        );
        const auto cameraMatrixVec = declare_parameter<std::vector<double>>(
            "camera_matrix", { }
        );
        distortionCoefficients = declare_parameter<std::vector<double>>(
            "distortion_coefficients", { }
        );

        // validate camera frame ID
        if (cameraFrameId.empty()) {
            constexpr auto msg = "camera_frame_id cannot be blank!";
            RCLCPP_FATAL(get_logger(), msg);
            throw std::runtime_error(msg);
        }

        // convert camera matrix list to cv::Matx
        if (cameraMatrixVec.size() != CAMERA_MATRIX_SIZE) {
            const std::string msg = fmt::format(
            "camera_matrix has the wrong number of elements! Expected {}, got {}",
                CAMERA_MATRIX_SIZE, cameraMatrixVec.size()
            );
            RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        }
        std::memcpy(cameraMatrix.val, cameraMatrixVec.data(), 9);

        // validate number of distortion coefficients
        const bool distCoefNumValid = std::ranges::find(
            DISTORTION_COEFFICIENTS_ALLOWED_SIZES, distortionCoefficients.size()
        ) != DISTORTION_COEFFICIENTS_ALLOWED_SIZES.end();
        if (!distCoefNumValid) {
            const std::string msg = fmt::format(
                "distortion_coefficients has an invalid number of elements! Valid numbers are {}, but got {}!",
                fmt::join(DISTORTION_COEFFICIENTS_ALLOWED_SIZES, ", "),
                distortionCoefficients.size()
            );
            RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        }

        RCLCPP_DEBUG(get_logger(), "Opening video capture on %s", capturePath.c_str());
        videoCapture.open(capturePath);

        if (!videoCapture.isOpened()) {
            constexpr auto errorMsg = "Failed to open video capture!";
            RCLCPP_FATAL(get_logger(), errorMsg);
            throw std::runtime_error(errorMsg);
        }

        RCLCPP_DEBUG(get_logger(), "Opened video capture in %dx%d @ %.1f fps",
            static_cast<int>(videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT)),
            videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FPS)
        );

        tagDetector = apriltag_detector_create();
        tagFamily = tag36h11_create();
        apriltag_detector_add_family(tagDetector, tagFamily);

        // start the worker thread
        workerThread = std::thread(&MonoCamera::run, this);
    }

    ~MonoCamera() noexcept override {
        workerThreadShouldExit = true;
        workerThread.join();

        apriltag_detector_remove_family(tagDetector, tagFamily);
        tag36h11_destroy(tagFamily);
        apriltag_detector_destroy(tagDetector);
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto args = removeRosArgs(argc, argv);

    try {
        const auto node = std::make_shared<MonoCamera>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) {
        std::cerr << "Failed to initialize node! Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}