cmake_minimum_required(VERSION 3.8)
project(camera)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenCV 和 cv_bridge 路径设置（可以根据需要调整）
set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake)
set(cv_bridge_DIR /opt/ros/humble/share/cv_bridge/cmake)
set(camera_DIR /home/plkdz/mythings/camera_test/camera_test/src)

# 自动生成 compile_commands.json（可选）
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 禁用 Wayland 支持，启用 X11 和 GTK 支持（可以根据需要调整）
set(WITH_WAYLAND OFF CACHE BOOL "Disable Wayland support")
set(WITH_X11 ON CACHE BOOL "Enable X11 support")
set(WITH_GTK ON CACHE BOOL "Enable GTK support")

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/String.msg"    # 你的自定义消息
  "srv/SaveImage.srv" # 你的自定义服务
)

# 添加可执行文件
add_executable(usb_cam_forward src/usb_cam.cpp)

# 链接依赖项
ament_target_dependencies(usb_cam_forward
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  std_srvs
)

# 为生成的接口添加依赖
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(usb_cam_forward ${cpp_typesupport_target})

# 安装可执行文件
install(TARGETS usb_cam_forward
  DESTINATION lib/${PROJECT_NAME})

# 启用测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下行跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
