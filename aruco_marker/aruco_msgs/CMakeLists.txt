cmake_minimum_required(VERSION 3.5)
project(aruco_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Add this line to find std_msgs

# Declare the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoPose.msg"
  DEPENDENCIES std_msgs
)

# Install the message files
install(DIRECTORY
  msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
