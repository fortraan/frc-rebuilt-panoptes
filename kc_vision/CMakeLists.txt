cmake_minimum_required(VERSION 3.10)
project(kc_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(wpilib REQUIRED)
find_package(depthai REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(common STATIC src/common/utilities.cpp)
target_include_directories(common PUBLIC src/common)

add_executable(solve_pnp src/solve_pnp/solve_pnp.cpp)
target_include_directories(solve_pnp PRIVATE
        src/solve_pnp
        ${sensor_msgs_INCLUDE_DIRS}
        ${apriltag_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(solve_pnp PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        tf2_geometry_msgs::tf2_geometry_msgs
        fmt::fmt
        ${sensor_msgs_LIBRARIES}
        ${apriltag_msgs_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_executable(tag_consensus src/tag_consensus/tag_consensus.cpp src/tag_consensus/ransac.cpp)
target_include_directories(tag_consensus PRIVATE src/tag_consensus)
target_link_libraries(tag_consensus PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
)

add_executable(ros_nt_bridge src/ros_nt_bridge/ros_nt_bridge.cpp)
target_include_directories(ros_nt_bridge PRIVATE src/ros_nt_bridge)
target_link_libraries(ros_nt_bridge PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        ntcore
        wpimath
)

add_executable(intake_cam src/intake_cam/intake_cam_node.cpp)
target_include_directories(intake_cam PRIVATE src/intake_cam)
target_link_libraries(intake_cam PRIVATE
        common
        rclcpp::rclcpp
        depthai::core
)

install(TARGETS solve_pnp tag_consensus ros_nt_bridge
        DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS src/field_urdf_generator/apriltag_urdf_generator.py
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch urdf resources
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
