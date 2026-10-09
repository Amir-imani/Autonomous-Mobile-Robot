cmake_minimum_required(VERSION 3.8)
project(pi_camera_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(pi_camera_publisher_node src/pi_camera_publisher_node.cpp)
target_compile_features(pi_camera_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(pi_camera_publisher_node
  camera_info_manager
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(pi_camera_publisher_node PkgConfig::GSTREAMER)

install(TARGETS pi_camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
