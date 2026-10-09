# Source Code

This directory contains the software developed for the autonomous mobile robot thesis project.

## Structure

- `firmware/pico/` — RP2040 / Raspberry Pi Pico low-level control firmware.
- `ros2/pico_serial_bridge/` — binary serial bridge between Raspberry Pi 5 and RP2040.
- `ros2/pi_camera_publisher/` — camera acquisition and ROS 2 image publication.
- `ros2/gpu_vision_inference/` — remote GPU vision inference node and telemetry.
- `ros2/vision_testbench/` — mock camera and academic benchmark support.
- `firmware/pico/training/` — offline Neuro-PID training utilities.

Additional ROS 2 packages from the thesis workspace (robot description, LiDAR aggregation, odometry, custom messages, launch/configuration files) are being added from the final project snapshot.

## Thesis-relevant entry points

| Component | Main file |
|---|---|
| RP2040 firmware | `firmware/pico/src/main.cpp` |
| Neuro-PID runtime | `firmware/pico/src/Neuro_PID.cpp` |
| Neuro-PID training | `firmware/pico/training/advanced_neuro_pid.py` |
| Pico ↔ Pi serial bridge | `ros2/pico_serial_bridge/pico_serial_bridge/serial_bridge_node.py` |
| Camera publisher | `ros2/pi_camera_publisher/src/pi_camera_publisher_node.cpp` |
| GPU inference | `ros2/gpu_vision_inference/src/gpu_vision_inference_node.cpp` |
| Vision telemetry | `ros2/gpu_vision_inference/src/academic_telemetry.cpp` |
| Mock camera benchmark | `ros2/vision_testbench/src/mock_camera_publisher_node.cpp` |

## Reproducibility note

The repository will receive a thesis snapshot tag before submission. Thesis hyperlinks should target that immutable tag rather than the moving `main` branch.
