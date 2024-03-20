cmake_minimum_required(VERSION 3.5)

# Find packages
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Create the Gazebo plugin
add_library(joy_control SHARED joy_control.cc)
target_link_libraries(joy_control ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${std_msgs_LIBRARIES})

# Install the plugin
install(TARGETS joy_control DESTINATION ${GAZEBO_PLUGIN_PATH})

