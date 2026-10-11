# Appendix A — Compact Source Index

This short index supplements the thesis appendix. For descriptions of **all** major subsystems, the ROS 2 data flow, configuration, build/run requirements and experimental evidence, see the [full software guide](SOFTWARE_GUIDE.md). The original source is organized under [`code/`](../code/).

## A.1 RP2040 firmware and Neuro-PID

| File | Function |
|---|---|
| [`main.cpp`](../code/firmware/pico/src/main.cpp) | Scheduling, two-wheel control, safeguards and telemetry |
| [`Config.h`](../code/firmware/pico/include/Config.h) | Hardware/timing configuration |
| [`Robot_State.h`](../code/firmware/pico/include/Robot_State.h) | Binary telemetry layout |
| [`Neuro_PID.cpp`](../code/firmware/pico/src/Neuro_PID.cpp) | 3–5–3 network gain inference |
| [`advanced_neuro_pid.py`](../code/firmware/pico/training/advanced_neuro_pid.py) | Offline training |
| [`Motor_BTS7960.cpp`](../code/firmware/pico/src/Motor_BTS7960.cpp) | Motor drive |
| [`Encoder_AS5600.cpp`](../code/firmware/pico/src/Encoder_AS5600.cpp) | Wheel angle and RPM |
| [`IMU_9DOF.cpp`](../code/firmware/pico/src/IMU_9DOF.cpp) | Inertial sensing |
| [`Filters.cpp`](../code/firmware/pico/src/Filters.cpp) | Motion signal filtering |
| [`Laser_Scanner.cpp`](../code/firmware/pico/src/Laser_Scanner.cpp) | Range-sensor input |
| [`Servo_Manager.cpp`](../code/firmware/pico/src/Servo_Manager.cpp) | Servo control |
| [`Power_Monitor.cpp`](../code/firmware/pico/src/Power_Monitor.cpp) | Electrical monitoring |
| [`Serial_Bridge.cpp`](../code/firmware/pico/src/Serial_Bridge.cpp) | Binary framing and CRC |

## A.2 ROS 2 core

| File | Function |
|---|---|
| [`serial_bridge_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/serial_bridge_node.py) | Commands and telemetry |
| [`differential_odometry_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/differential_odometry_node.py) | Wheel odometry / TF |
| [`lidar_scan_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/lidar_scan_node.py) | LaserScan reconstruction |
| [`robot_core.launch.py`](../code/ros2/pico_serial_bridge/launch/robot_core.launch.py) | Core nodes launch |
| [`amr.urdf.xacro`](../code/ros2/amr_description/urdf/amr.urdf.xacro) | Robot model |
| [`physical_parameters.yaml`](../code/ros2/amr_description/config/physical_parameters.yaml) | Geometry parameters |

## A.3 Distributed vision

| File | Function |
|---|---|
| [`pi_camera_publisher_node.cpp`](../code/ros2/pi_camera_publisher/src/pi_camera_publisher_node.cpp) | Camera JPEG publisher |
| [`gpu_vision_inference_node.cpp`](../code/ros2/gpu_vision_inference/src/gpu_vision_inference_node.cpp) | Inference and freshness controls |
| [`academic_telemetry.cpp`](../code/ros2/gpu_vision_inference/src/academic_telemetry.cpp) | Latency and counters |
| [`Detection2D.msg`](../code/ros2/distributed_vision_msgs/msg/Detection2D.msg) | Object detection message |
| [`Detection2DArray.msg`](../code/ros2/distributed_vision_msgs/msg/Detection2DArray.msg) | Detection array |
| [`mock_camera_publisher_node.cpp`](../code/ros2/vision_testbench/src/mock_camera_publisher_node.cpp) | Synthetic camera source |

## A.4 Experiments

The main entry point is [`run_all.py`](../code/experiments/run_all.py), with analysis in [`analyze_results.py`](../code/experiments/analyze_results.py). Source-level checks, host Neuro-PID benchmarks, offload emulation, stress tests and ROS 2 probes are documented in the [experiment methodology](../code/experiments/README.md).

## Citation policy

The thesis should link to a **specific commit SHA** so the source snapshot does not move. No Git tag should be cited unless verified. The repository is private and requires authorized access.
