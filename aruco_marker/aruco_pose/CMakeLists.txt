cmake_minimum_required(VERSION 3.8)
project(aruco_pose)

set(CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(aruco_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)

include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Set source files
set(aruco_pose_src
    src/aruco_pose.cpp
)

# Create executable
add_executable(aruco_pose_node ${aruco_pose_src})

# Link libraries
link_directories(${GST_LIBRARY_DIRS})
target_link_libraries(aruco_pose_node ${GST_LIBRARIES} ${OpenCV_LIBS})


ament_target_dependencies(aruco_pose_node
    rclcpp
    tf2 
    tf2_ros
    geometry_msgs
)


# Set source files
set(usb_cam_src
    src/usb_cam.cpp
)

# Create executable
add_executable(usb_cam_node ${usb_cam_src})

# Link libraries
target_link_libraries(usb_cam_node ${GST_LIBRARIES} ${OpenCV_LIBS})


ament_target_dependencies(usb_cam_node
    rclcpp
    sensor_msgs
    image_transport
    cv_bridge
    OpenCV
)


# Set source files
set(aruco_detector_src
    src/aruco_detector.cpp
)

# Create executable
add_executable(aruco_detector_node ${aruco_detector_src})

# Link libraries
target_link_libraries(aruco_detector_node ${GST_LIBRARIES} ${OpenCV_LIBS})


ament_target_dependencies(aruco_detector_node
    rclcpp
    sensor_msgs
    image_transport
    cv_bridge
    OpenCV
    tf2 
    tf2_ros
    geometry_msgs
)


# Set source files
set(aruco_detector_mul_src
    src/aruco_detector_mult.cpp
)

# Create executable
add_executable(aruco_detector_mul_node ${aruco_detector_mul_src})

# Link libraries
target_link_libraries(aruco_detector_mul_node ${GST_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


ament_target_dependencies(aruco_detector_mul_node
    rclcpp
    sensor_msgs
    image_transport
    cv_bridge
    OpenCV
    tf2 
    tf2_ros
    geometry_msgs
    aruco_msgs
)


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
    aruco_pose_node
    DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    usb_cam_node
    DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    aruco_detector_node
    DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    aruco_detector_mul_node
    DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
