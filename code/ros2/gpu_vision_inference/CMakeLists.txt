cmake_minimum_required(VERSION 3.8)
project(gpu_vision_inference)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(distributed_vision_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

find_path(NVJPEG_INCLUDE_DIR nvjpeg.h HINTS ENV CUDA_PATH /usr/local/cuda /opt/cuda PATH_SUFFIXES include)
find_library(NVJPEG_LIBRARY nvjpeg HINTS ENV CUDA_PATH /usr/local/cuda /opt/cuda PATH_SUFFIXES lib64 lib/x64 lib)
find_library(CUDART_LIBRARY cudart HINTS ENV CUDA_PATH /usr/local/cuda /opt/cuda PATH_SUFFIXES lib64 lib/x64 lib)

add_executable(gpu_vision_inference_node
  src/academic_telemetry.cpp
  src/gpu_vision_inference_node.cpp
)
target_compile_features(gpu_vision_inference_node PUBLIC cxx_std_17)
ament_target_dependencies(gpu_vision_inference_node diagnostic_msgs distributed_vision_msgs rclcpp rmw sensor_msgs)
target_include_directories(gpu_vision_inference_node PRIVATE include)
target_include_directories(gpu_vision_inference_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gpu_vision_inference_node ${OpenCV_LIBS} Threads::Threads)
if(NVJPEG_INCLUDE_DIR AND NVJPEG_LIBRARY AND CUDART_LIBRARY)
  target_include_directories(gpu_vision_inference_node SYSTEM PRIVATE ${NVJPEG_INCLUDE_DIR})
  target_link_libraries(gpu_vision_inference_node ${NVJPEG_LIBRARY} ${CUDART_LIBRARY})
  target_compile_definitions(gpu_vision_inference_node PRIVATE GPU_VISION_HAS_NVJPEG=1)
  message(STATUS "gpu_vision_inference: nvJPEG decoding enabled")
else()
  message(WARNING "gpu_vision_inference: nvJPEG not found; only CPU JPEG decoding is available")
endif()

install(TARGETS gpu_vision_inference_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
