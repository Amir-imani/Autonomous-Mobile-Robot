# Software architecture and source guide

This guide accompanies Appendix A of the undergraduate thesis. Source code is under [`code/`](../code/); schematics/CAD are separate under `hardware/` and `mechanical/`. It describes the code actually present in this repository, not a completed autonomous-navigation or field-tested GPU deployment.

## System architecture and data flow

```text
 ROS 2 /cmd_vel (Twist)
          |
 Raspberry Pi 5: pico_serial_bridge
          |  binary command: AA BB + 16-byte payload + CRC16
          v
 RP2040/Pico: main.cpp -> motor drivers / Neuro-PID / AS5600
          |  76-byte RobotState telemetry + CRC16
          v
 Raspberry Pi 5: /pico/robot_state
          +--> differential_odometry_node --> /odom, odom->base TF
          +--> lidar_scan_node          --> /scan
          +--> /joint_states, diagnostics and power topics

 Raspberry Pi camera / mock test source
          |  compressed JPEG; sensor-data QoS, depth 1
          v
 /camera/image_raw/compressed --> external inference node
                                --> detections / annotated image / diagnostics
```

The control and safety loop stays on RP2040; remote vision augments perception and is **not** a prerequisite for local motor safety. ROS 2 package names and topic-specific behavior are documented in the corresponding source and configuration files.

## 1. Firmware on RP2040

Base directory: [`code/firmware/pico/`](../code/firmware/pico/).

| Path relative to firmware/pico | Purpose |
|---|---|
| [`src/main.cpp`](../code/firmware/pico/src/main.cpp) | Periodic work scheduling, command validation, dual-wheel control, watchdog, safe state and telemetry |
| [`include/Config.h`](../code/firmware/pico/include/Config.h) | Pin assignments, safety/timing constants and hardware configuration |
| [`include/Robot_State.h`](../code/firmware/pico/include/Robot_State.h) | Packed telemetry structure and diagnostics flags |
| [`include/ROS2_Structs.h`](../code/firmware/pico/include/ROS2_Structs.h) | Shared firmware-side structures for communication |
| [`src/Serial_Bridge.cpp`](../code/firmware/pico/src/Serial_Bridge.cpp) | Binary telemetry framing and CRC16/IBM |
| [`src/Motor_BTS7960.cpp`](../code/firmware/pico/src/Motor_BTS7960.cpp) | Motor-driver interface |
| [`src/Encoder_AS5600.cpp`](../code/firmware/pico/src/Encoder_AS5600.cpp) | Angle wrapping, wheel RPM and accumulated turns |
| [`src/IMU_9DOF.cpp`](../code/firmware/pico/src/IMU_9DOF.cpp) | IMU and magnetometer readings |
| [`src/Filters.cpp`](../code/firmware/pico/src/Filters.cpp) | RPM low-pass and complementary orientation filters |
| [`src/Laser_Scanner.cpp`](../code/firmware/pico/src/Laser_Scanner.cpp) | Serial distance-sensor parsing |
| [`src/Servo_Manager.cpp`](../code/firmware/pico/src/Servo_Manager.cpp) | Sweeping servo motion and angle bounds |
| [`src/Power_Monitor.cpp`](../code/firmware/pico/src/Power_Monitor.cpp) | Power-monitor readings |
| [`src/Neuro_PID.cpp`](../code/firmware/pico/src/Neuro_PID.cpp) | Embedded 3-input/5-hidden/3-output network for PID gains |
| [`training/advanced_neuro_pid.py`](../code/firmware/pico/training/advanced_neuro_pid.py) | Offline Neuro-PID training path |
| [`training/train_neuro_pid.py`](../code/firmware/pico/training/train_neuro_pid.py) | Additional training script |
| [`platformio.ini`](../code/firmware/pico/platformio.ini) | Build environment |

Matching declarations for firmware drivers are in [`include/`](../code/firmware/pico/include/). The neural network is trained off-device; the RP2040 executes a C++ forward pass, not PyTorch training. The command payload is 16 bytes and telemetry payload 76 bytes, both little-endian with the same CRC polynomial/initial value on Pico and Pi. Consult the source for the precise packet layout, safety conditions and calibrated parameters.

## 2. ROS 2 on Raspberry Pi 5

All ROS packages are in [`code/ros2/`](../code/ros2/).

**`pico_serial_bridge` (Python):**
- [`serial_bridge_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/serial_bridge_node.py): interprets `/cmd_vel`, converts it into left/right wheel targets and transports binary commands and telemetry.
- [`differential_odometry_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/differential_odometry_node.py): wheel odometry and odometry TF.
- [`lidar_scan_node.py`](../code/ros2/pico_serial_bridge/pico_serial_bridge/lidar_scan_node.py): accumulates servo-angle/distance samples as LaserScan.
- [`robot_core.launch.py`](../code/ros2/pico_serial_bridge/launch/robot_core.launch.py): composes the core robot nodes, including robot-state publication.
- [`package.xml`](../code/ros2/pico_serial_bridge/package.xml) and [`setup.py`](../code/ros2/pico_serial_bridge/setup.py): ROS package/build metadata.

**`amr_description` (Xacro/URDF):**
- [`amr.urdf.xacro`](../code/ros2/amr_description/urdf/amr.urdf.xacro) includes modular chassis, drivetrain, casters and sensor descriptions from the same directory.
- [`physical_parameters.yaml`](../code/ros2/amr_description/config/physical_parameters.yaml) holds robot dimensions and joint limits.
- [`display.launch.py`](../code/ros2/amr_description/launch/display.launch.py) and [`amr_description.rviz`](../code/ros2/amr_description/rviz/amr_description.rviz) support RViz inspection.

Odometry/TF/LaserScan are navigation **infrastructure**. This repository does not contain production Nav2 planners, SLAM Toolbox source, a map/ground-truth dataset or a complete obstacle-avoidance controller. Initial SLAM integration mentioned in the thesis must not be conflated with a completed evaluated navigation stack.

## 3. Camera and distributed vision

- [`pi_camera_publisher_node.cpp`](../code/ros2/pi_camera_publisher/src/pi_camera_publisher_node.cpp): Raspberry Pi camera capture, GStreamer/JPEG publication, timestamps and camera information.
- [`pi_camera.yaml`](../code/ros2/pi_camera_publisher/config/pi_camera.yaml), [`pi_camera.launch.py`](../code/ros2/pi_camera_publisher/launch/pi_camera.launch.py): camera parameters and launch.
- [`gpu_vision_inference_node.cpp`](../code/ros2/gpu_vision_inference/src/gpu_vision_inference_node.cpp): compressed-image receipt, latest-frame replacement, decoding, OpenCV DNN object detection, freshness constraints and structured outputs; optional CUDA/FP16/nvJPEG paths depend on installed build/hardware.
- [`academic_telemetry.cpp`](../code/ros2/gpu_vision_inference/src/academic_telemetry.cpp) and [`academic_telemetry.hpp`](../code/ros2/gpu_vision_inference/include/gpu_vision_inference/academic_telemetry.hpp): frame timing/counts and distribution statistics.
- [`gpu_inference.yaml`](../code/ros2/gpu_vision_inference/config/gpu_inference.yaml), [`gpu_inference.launch.py`](../code/ros2/gpu_vision_inference/launch/gpu_inference.launch.py): runtime settings and launch.
- [`Detection2D.msg`](../code/ros2/distributed_vision_msgs/msg/Detection2D.msg) and [`Detection2DArray.msg`](../code/ros2/distributed_vision_msgs/msg/Detection2DArray.msg): custom detection and timing/backend message types.
- [`mock_camera_publisher_node.cpp`](../code/ros2/vision_testbench/src/mock_camera_publisher_node.cpp): hardware-independent image publisher; test configurations and launches are alongside it.

The source expresses a GPU-oriented processing design; source availability alone is **not evidence** of measured CUDA/FP16 acceleration. Check the active backend, a compatible ONNX model and actual hardware before reporting GPU-specific results. The repository does not bundle a trained ONNX model.

## 4. Reproducible software experiments

Base directory: [`code/experiments/`](../code/experiments/). Consult its [detailed README](../code/experiments/README.md) before interpreting results.

| File | Function |
|---|---|
| [`run_all.py`](../code/experiments/run_all.py) | Launches the experimental campaign |
| [`codebase_audit.py`](../code/experiments/codebase_audit.py) | Codebase inventory and static checks |
| [`control_safety_checks.py`](../code/experiments/control_safety_checks.py) | Source-derived safety checks |
| [`core_logic_benchmark.py`](../code/experiments/core_logic_benchmark.py) | Direct functional/microbenchmark tests |
| [`host_neuro_benchmark.py`](../code/experiments/host_neuro_benchmark.py) | Host execution of production Neuro-PID C++ |
| [`offload_emulation.py`](../code/experiments/offload_emulation.py) | Application-layer processing/offload emulation |
| [`stress_benchmark.py`](../code/experiments/stress_benchmark.py) | Host CPU stress/scheduling |
| [`ros2_latency_probe.py`](../code/experiments/ros2_latency_probe.py) | Probe for ROS 2 deployments |
| [`network_emulation.sh`](../code/experiments/network_emulation.sh) | Linux tc/netem tooling for controlled network tests |
| [`resource_probe.py`](../code/experiments/resource_probe.py) | CPU/RAM resource sampling |
| [`analyze_results.py`](../code/experiments/analyze_results.py) | Experiment statistics |
| [`generate_plots.py`](../code/experiments/generate_plots.py) | Plot generation |
| [`generate_final_report.py`](../code/experiments/generate_final_report.py) | Report synthesis |

`run_all.ps1` and `run_all.sh` are host-specific runners; `firmware_host/` provides the test harness for embedded code on a desktop.

**Evidence distinction:** direct source tests, synthetic/host emulations, analytical estimates and unperformed hardware-dependent measurements are separate categories. Do not label loopback TCP as DDS latency, synthetic computation as YOLO inference, or modeled odometry as measured localization accuracy. Any actual ROS 2 / physical-camera / GPU evaluation needs a compatible ROS 2 environment, runtime dependencies and (where appropriate) physical devices.

## How to navigate and reproduce

1. Read [`code/README.md`](../code/README.md) for package entry points and [`experiments/README.md`](../code/experiments/README.md) for test prerequisites.
2. Build RP2040 firmware using [`platformio.ini`](../code/firmware/pico/platformio.ini) in an environment with supported board/toolchain and connected sensors.
3. Build ROS packages with a compatible ROS 2 development environment; dependency and launch declarations reside in each package's `package.xml`, `CMakeLists.txt`, `setup.py` and `launch/` files.
4. Set camera, serial port, robot geometry and inference parameters for the actual platform; never assume a demonstration configuration is a measured physical setup.
5. Run applicable host experiments using `python run_all.py` from `code/experiments/`; inspect produced manifests and evidence classes. ROS 2 and Linux network experiments require their own prerequisites.

## Thesis links and versioning

The thesis should point to a **fixed commit SHA** or a release tag at submission, not moving branch `main`. GitHub file links then use `https://github.com/Amir-imani/Autonomous-Mobile-Robot/blob/<commit-sha>/<path>`. This repository is private at the time this guide was prepared: reviewers need repository access to open hyperlinks or QR codes.
