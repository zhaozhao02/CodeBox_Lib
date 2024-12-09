cmake_minimum_required(VERSION 3.8)
project(aruco_tracking)

set (CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera REQUIRED)  # 查找服务端包

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

link_directories(${OpenCV_LIBRARY_DIRS})

set(ruco_tarcking_src
    src/tracking_node.cpp
   )
add_executable(aruco_tarcking_node ${ruco_tarcking_src})
target_link_libraries(aruco_tarcking_node
    ${OpenCV_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )


ament_target_dependencies(aruco_tarcking_node
    rclcpp
    tf2 
    tf2_ros
    nav_msgs
    geometry_msgs
    mavros_msgs
    aruco_msgs
    Eigen3
    std_srvs
    camera
  )

# target_compile_options(aruco_pose_node
#     PRIVATE -O3 -std=c++11
#     )


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

install(TARGETS
aruco_tarcking_node

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
