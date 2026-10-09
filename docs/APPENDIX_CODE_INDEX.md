# Appendix A — Source Code Index

This document is the repository-side index for the source-code appendix of the thesis.

## A.1 Low-level firmware

| File | Role |
|---|---|
| [main.cpp](../code/firmware/pico/src/main.cpp) | Main RP2040 control loop, scheduling, command parsing, safety logic, sensor acquisition, motor control, and telemetry |
| [Config.h](../code/firmware/pico/include/Config.h) | Hardware pins and timing/safety constants |
| [Robot_State.h](../code/firmware/pico/include/Robot_State.h) | Fixed binary telemetry payload and diagnostic flags |
| [Serial_Bridge.cpp](../code/firmware/pico/src/Serial_Bridge.cpp) | Pico-to-Pi binary framing and CRC-16/IBM |
| [Motor_BTS7960.cpp](../code/firmware/pico/src/Motor_BTS7960.cpp) | Motor-driver abstraction |
| [Encoder_AS5600.cpp](../code/firmware/pico/src/Encoder_AS5600.cpp) | Magnetic encoder acquisition, unwrap, RPM, and accumulated motion |
| [IMU_9DOF.cpp](../code/firmware/pico/src/IMU_9DOF.cpp) | IMU/magnetometer acquisition |
| [Filters.cpp](../code/firmware/pico/src/Filters.cpp) | RPM low-pass filter and complementary attitude filter |
| [Laser_Scanner.cpp](../code/firmware/pico/src/Laser_Scanner.cpp) | Single-point laser range sensor parsing |
| [Servo_Manager.cpp](../code/firmware/pico/src/Servo_Manager.cpp) | Bounded servo-angle motion |
| [Power_Monitor.cpp](../code/firmware/pico/src/Power_Monitor.cpp) | INA226/INA219 monitoring |
| [Neuro_PID.cpp](../code/firmware/pico/src/Neuro_PID.cpp) | Embedded 3–5–3 neural gain inference |
| [advanced_neuro_pid.py](../code/firmware/pico/training/advanced_neuro_pid.py) | Final offline Neuro-PID training workflow |

## A.2 ROS 2 core

| File | Role |
|---|---|
| [serial_bridge_node.py](../code/ros2/pico_serial_bridge/pico_serial_bridge/serial_bridge_node.py) | Bidirectional Pico ↔ Pi bridge and ROS topic fan-out |
| [differential_odometry_node.py](../code/ros2/pico_serial_bridge/pico_serial_bridge/differential_odometry_node.py) | Differential-drive odometry and TF generation |
| [lidar_scan_node.py](../code/ros2/pico_serial_bridge/pico_serial_bridge/lidar_scan_node.py) | Servo-LiDAR point accumulation into LaserScan |
| [robot_core.launch.py](../code/ros2/pico_serial_bridge/launch/robot_core.launch.py) | Core robot launch composition |
| [amr.urdf.xacro](../code/ros2/amr_description/urdf/amr.urdf.xacro) | Top-level robot model |
| [physical_parameters.yaml](../code/ros2/amr_description/config/physical_parameters.yaml) | Measured geometry and joint limits |

## A.3 Distributed vision

| File | Role |
|---|---|
| [pi_camera_publisher_node.cpp](../code/ros2/pi_camera_publisher/src/pi_camera_publisher_node.cpp) | Camera acquisition, compression, timestamp handling, and publication |
| [pi_camera.yaml](../code/ros2/pi_camera_publisher/config/pi_camera.yaml) | Camera configuration |
| [gpu_vision_inference_node.cpp](../code/ros2/gpu_vision_inference/src/gpu_vision_inference_node.cpp) | Decode, preprocessing, OpenCV-DNN inference path, NMS, freshness handling, and outputs |
| [academic_telemetry.cpp](../code/ros2/gpu_vision_inference/src/academic_telemetry.cpp) | Per-frame telemetry and distribution statistics |
| [gpu_inference.yaml](../code/ros2/gpu_vision_inference/config/gpu_inference.yaml) | Inference/runtime parameters |
| [Detection2D.msg](../code/ros2/distributed_vision_msgs/msg/Detection2D.msg) | Single detection message |
| [Detection2DArray.msg](../code/ros2/distributed_vision_msgs/msg/Detection2DArray.msg) | Detection-array message with timing/backend fields |
| [mock_camera_publisher_node.cpp](../code/ros2/vision_testbench/src/mock_camera_publisher_node.cpp) | Hardware-independent camera test source |

## A.4 Experimental evaluation

The complete reproducibility tooling is under [code/experiments](../code/experiments/). The main orchestration file is [run_all.py](../code/experiments/run_all.py). Supporting tools cover static auditing, core logic checks, host Neuro-PID timing, offload emulation, resource sampling, CPU stress, ROS 2 latency probing, statistical analysis, plots, and final-report generation.

## Versioning

For the submitted thesis, links should be changed from `main` to the final immutable thesis tag (planned: `thesis-v1.0`).
