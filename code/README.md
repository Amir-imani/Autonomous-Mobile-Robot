# Source Code

This directory contains the software developed for the autonomous mobile robot thesis project.

## Structure

- `firmware/pico/` — RP2040 / Raspberry Pi Pico low-level firmware.
- `firmware/pico/training/` — offline Neuro-PID training utilities.
- `ros2/pico_serial_bridge/` — binary serial bridge, odometry, and LiDAR scan generation.
- `ros2/amr_description/` — Xacro/URDF, physical parameters, launch, and RViz configuration.
- `ros2/distributed_vision_msgs/` — custom object-detection message definitions.
- `ros2/pi_camera_publisher/` — CSI/GStreamer camera acquisition and ROS 2 publication.
- `ros2/gpu_vision_inference/` — distributed vision inference and academic telemetry.
- `ros2/vision_testbench/` — hardware-independent camera benchmark source.
- `experiments/` — audit, direct checks, emulation, stress tests, analysis, and reporting.

## Thesis-relevant entry points

| Component | Main file |
|---|---|
| RP2040 firmware | [main.cpp](firmware/pico/src/main.cpp) |
| Neuro-PID runtime | [Neuro_PID.cpp](firmware/pico/src/Neuro_PID.cpp) |
| Neuro-PID training | [advanced_neuro_pid.py](firmware/pico/training/advanced_neuro_pid.py) |
| Pico ↔ Pi serial bridge | [serial_bridge_node.py](ros2/pico_serial_bridge/pico_serial_bridge/serial_bridge_node.py) |
| Differential odometry | [differential_odometry_node.py](ros2/pico_serial_bridge/pico_serial_bridge/differential_odometry_node.py) |
| Servo-LiDAR aggregation | [lidar_scan_node.py](ros2/pico_serial_bridge/pico_serial_bridge/lidar_scan_node.py) |
| Robot description | [amr.urdf.xacro](ros2/amr_description/urdf/amr.urdf.xacro) |
| Camera publisher | [pi_camera_publisher_node.cpp](ros2/pi_camera_publisher/src/pi_camera_publisher_node.cpp) |
| GPU inference | [gpu_vision_inference_node.cpp](ros2/gpu_vision_inference/src/gpu_vision_inference_node.cpp) |
| Vision telemetry | [academic_telemetry.cpp](ros2/gpu_vision_inference/src/academic_telemetry.cpp) |
| Mock camera benchmark | [mock_camera_publisher_node.cpp](ros2/vision_testbench/src/mock_camera_publisher_node.cpp) |
| Experiment campaign | [run_all.py](experiments/run_all.py) |

## Reproducibility note

The benchmark suite deliberately distinguishes direct source execution, host/synthetic emulation, analytical estimates, and tests that require physical hardware. See `experiments/README.md` and `experiments/ARCHITECTURE_MAP.md`.

The repository will receive a fixed thesis tag before submission. Final thesis hyperlinks should use that tag rather than `main`.
