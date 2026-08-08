cmake_minimum_required(VERSION 3.19)
project(kc_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
# silence warnings from wpilib using an ancient version of CMake
set(CMAKE_WARN_DEPRECATED OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(kc_vision_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(wpilib REQUIRED)
find_package(Eigen3 REQUIRED)

# the following packages are platform-specific. The Jetson has these set up already,
# but if you're building locally you'll need an Nvidia GPU, the CUDA toolkit, and
# TensorRT installed.
#find_package(CUDAToolkit REQUIRED)
#find_package(Thrust REQUIRED CONFIG)
#thrust_create_target(libraries::Thrust)
#
#find_library(NVINFER NAMES nvinfer REQUIRED)
#find_library(NVINFERPLUGIN NAMES nvinfer_plugin REQUIRED)
#
#add_library(TensorRT INTERFACE)
#target_link_libraries(TensorRT INTERFACE
#        CUDA::cudart
#        ${NVINFER}
#        ${NVINFERPLUGIN}
#)
#target_include_directories(TensorRT INTERFACE
#
#)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

add_library(common STATIC src/common/utilities.cpp)
target_include_directories(common PUBLIC src/common)

add_executable(solve_pnp src/solve_pnp/solve_pnp.cpp)
target_include_directories(solve_pnp PRIVATE
        src/solve_pnp
        ${sensor_msgs_INCLUDE_DIRS}
        ${isaac_ros_apriltag_interfaces_INCLUDE_DIRS}
        ${kc_vision_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(solve_pnp PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        tf2_geometry_msgs::tf2_geometry_msgs
        fmt::fmt
        ${sensor_msgs_LIBRARIES}
        ${isaac_ros_apriltag_interfaces_LIBRARIES}
        ${kc_vision_msgs_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_executable(tag_consensus
        src/tag_consensus/tag_consensus.cpp
        src/tag_consensus/apecs.cpp
)
target_include_directories(tag_consensus PRIVATE
        src/tag_consensus
        ${kc_vision_msgs_INCLUDE_DIRS}
        ${diagnostic_msgs_INCLUDE_DIRS}
)
target_link_libraries(tag_consensus PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        Eigen3::Eigen
        tf2_geometry_msgs::tf2_geometry_msgs
        fmt::fmt
        ${kc_vision_msgs_LIBRARIES}
        ${diagnostic_msgs_LIBRARIES}
)


add_executable(ros_nt_bridge
        src/ros_nt_bridge/ros_nt_bridge.cpp
        src/ros_nt_bridge/nt_structs.cpp
)
target_include_directories(ros_nt_bridge PRIVATE
        src/ros_nt_bridge
        ${kc_vision_msgs_INCLUDE_DIRS}
)
target_link_libraries(ros_nt_bridge PRIVATE
        common
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        ntcore
        wpimath
        wpiutil
        ${kc_vision_msgs_LIBRARIES}
)

#add_executable(detection_location_calculator src/intake_cam/detection_location_calculator.cpp)
#target_include_directories(detection_location_calculator PRIVATE
#        src/intake_cam
#        ${sensor_msgs_INCLUDE_DIRS}
#        ${vision_msgs_INCLUDE_DIRS}
#)
#target_link_libraries(detection_location_calculator PRIVATE
#        common
#        rclcpp::rclcpp
#        opencv_core
#        image_transport::image_transport
#        cv_bridge::cv_bridge
#        ${sensor_msgs_LIBRARIES}
#        ${vision_msgs_LIBRARIES}
#)
#
#add_executable(fuel_visualizer src/intake_cam/fuel_visualizer.cpp)
#target_include_directories(fuel_visualizer PRIVATE
#        src/intake_cam
#        ${vision_msgs_INCLUDE_DIRS}
#        ${visualization_msgs_INCLUDE_DIRS}
#)
#target_link_libraries(fuel_visualizer PRIVATE
#        common
#        rclcpp::rclcpp
#        ${vision_msgs_LIBRARIES}
#        ${visualization_msgs_LIBRARIES}
#)

#add_executable(inference_node src/intake_cam/inference_node.cpp)
#target_include_directories(inference_node PRIVATE
#        src/intake_cam
#        ${sensor_msgs_INCLUDE_DIRS}
#        ${vision_msgs_INCLUDE_DIRS}
#)
#target_link_libraries(inference_node PRIVATE
#        common
#        rclcpp::rclcpp
#        image_transport::image_transport
#        CUDA::cudart
#        TensorRT
#        fmt::fmt
#        ${sensor_msgs_LIBRARIES}
#        ${vision_msgs_LIBRARIES}
#)

add_executable(grid_fuel_detector
        src/grid_fuel_detector/grid_fuel_detector_node.cpp
        src/grid_fuel_detector/GridFuelDetector.cpp
)
target_include_directories(grid_fuel_detector PRIVATE
        src/grid_fuel_detector
        ${OpenCV_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${kc_vision_msgs_INCLUDE_DIRS}
        ${diagnostic_msgs_INCLUDE_DIRS}
)
target_link_libraries(grid_fuel_detector PRIVATE
        common
        rclcpp::rclcpp
        image_transport::image_transport
        tf2_ros::tf2_ros
        tf2_eigen::tf2_eigen
        cv_bridge::cv_bridge
        Eigen3::Eigen
        ${OpenCV_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${kc_vision_msgs_LIBRARIES}
        ${diagnostic_msgs_LIBRARIES}
)

add_executable(grid_fuel_visualizer src/grid_fuel_detector/grid_fuel_visualizer.cpp)
target_include_directories(grid_fuel_visualizer PRIVATE
        ${nav_msgs_INCLUDE_DIRS}
        ${kc_vision_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(grid_fuel_visualizer PRIVATE
        common
        rclcpp::rclcpp
        ${nav_msgs_LIBRARIES}
        ${kc_vision_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
)

install(TARGETS
        solve_pnp tag_consensus ros_nt_bridge grid_fuel_detector grid_fuel_visualizer
        DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS src/field_urdf_generator/apriltag_urdf_generator.py
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch urdf resources
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
