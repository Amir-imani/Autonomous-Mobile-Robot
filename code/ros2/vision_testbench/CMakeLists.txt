cmake_minimum_required(VERSION 3.8)
project(vision_testbench)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(mock_camera_publisher_node src/mock_camera_publisher_node.cpp)
target_compile_features(mock_camera_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(mock_camera_publisher_node
  camera_info_manager
  rclcpp
  sensor_msgs
)
target_link_libraries(mock_camera_publisher_node PkgConfig::GSTREAMER Threads::Threads)

install(TARGETS mock_camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
