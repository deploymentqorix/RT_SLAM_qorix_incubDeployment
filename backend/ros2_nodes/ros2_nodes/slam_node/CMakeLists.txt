cmake_minimum_required(VERSION 3.8)
project(slam_node)

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Define the C++ executable
add_executable(
    slam_main
    src/slam_logic.cpp
)

# Link ROS 2 dependencies using the ROS 2 function
ament_target_dependencies(
    slam_main
    rclcpp
    geometry_msgs
    sensor_msgs
)

# Link the non-ROS Eigen library using the standard CMake function
target_link_libraries(slam_main Eigen3::Eigen)

# Install the executable
install(
    TARGETS slam_main
    DESTINATION lib/${PROJECT_NAME}
)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ... (keep add_executable)

# Link all dependencies
ament_target_dependencies(
    slam_main
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs      # <-- Add this
    tf2_ros       # <-- Add this
)
target_link_libraries(slam_main Eigen3::Eigen)

# ... (keep install and ament_package)

ament_package()