# Source Code

This directory holds the research software. Read the [complete source and architecture guide](../docs/SOFTWARE_GUIDE.md) for detailed file descriptions, system data flow, dependencies, run guidance and limitations; refer to the [short appendix index](../docs/APPENDIX_CODE_INDEX.md) for thesis citations.

## Subsystems

- [`firmware/pico/`](firmware/pico/) — RP2040 control and safety loop, motor/encoder/IMU/LiDAR/servo/power drivers, protocol, embedded Neuro-PID, offline training scripts.
- [`ros2/pico_serial_bridge/`](ros2/pico_serial_bridge/) — Pico serial bridge, differential odometry, LiDAR aggregation and core launch.
- [`ros2/amr_description/`](ros2/amr_description/) — modular Xacro model, physical parameters, TF/RViz support.
- [`ros2/distributed_vision_msgs/`](ros2/distributed_vision_msgs/) — custom detection messages.
- [`ros2/pi_camera_publisher/`](ros2/pi_camera_publisher/) — Raspberry Pi camera and compressed JPEG publication.
- [`ros2/gpu_vision_inference/`](ros2/gpu_vision_inference/) — OpenCV DNN detection with optional GPU backends, latest-frame policy and telemetry.
- [`ros2/vision_testbench/`](ros2/vision_testbench/) — hardware-independent video/test-pattern publisher.
- [`experiments/`](experiments/) — software checks, host microbenchmarks, simulation, resource stress and reporting.

## Reproducibility

The [experiment README](experiments/README.md) distinguishes direct source execution from synthetic/emulated tests and experiments requiring actual devices, network or model dependencies. Do not interpret host emulation as physical GPU or navigation measurements. Use the immutable commit SHA referenced in the thesis for reproducibility.
