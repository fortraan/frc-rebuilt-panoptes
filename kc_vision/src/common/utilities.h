#ifndef KC_VISION_UTILITIES_H
#define KC_VISION_UTILITIES_H

#include <string>
#include <vector>

std::vector<std::string> removeRosArgs(int argc, const char* const argv[]);

#ifndef NDEBUG
#include <iostream>
#define KC_DEBUG_ASSERT(condition, msg) do { \
    if (!(condition)) { \
        std::cout << "Assertion failed! " __FILE__ ":" << __LINE__ << ": " << msg << std::endl; \
    } \
} while (false)
#define KC_DEBUG_ASSERT_ROS(logger, condition, msg) do { \
    if (!(condition)) { \
        RCLCPP_WARN_STREAM(logger, "Assertion failed! " __FILE__ ":" << __LINE__ << ": " << msg); \
    } \
} while (false)
#else
#define KC_DEBUG_ASSERT(condition, msg)
#define KC_DEBUG_ASSERT_ROS(logger, condition, msg)
#endif

#endif //KC_VISION_UTILITIES_H