# Executable architecture map

This map reflects repository code as inspected on 2026-09-05. It is not an
aspirational system diagram.

## Deployment and nodes

| Target | Process/node | Inputs | Outputs | Scheduling/threads |
|---|---|---|---|---|
| RP2040/Pico | firmware `main.cpp` core 0 | binary commands, encoders, IMU, GY-56 LiDAR, power monitors | 76-byte `RobotState`, motors, servos, USB diagnostics | IMU 100 Hz; motor 50 Hz; LiDAR 50 Hz; telemetry 50 Hz; diagnostic 10 Hz; 250 ms hardware watchdog |
| RP2040/Pico | firmware core 1 | normalized error/derivative/integral snapshot | 3-5-3 Neuro-PID gains | continuous loop with mutex exchange |
| Pi 5 | `serial_bridge_node` | `/cmd_vel`, serial telemetry | serial commands plus 12 ROS topics | serial worker thread; 50 Hz command heartbeat; 2 ms ROS queue-drain timer |
| Pi 5 | `differential_odometry_node` | `/pico/robot_state` | `/odom`; `odom -> base_footprint -> base_link` TF | subscription callback, receipt-time integration |
| Pi 5 | `lidar_scan_node` | `/pico/robot_state` | `/scan` | callback accumulator plus timeout timer |
| Pi 5 | `robot_state_publisher` | Xacro, `/joint_states` | robot TF tree | ROS package default behavior |
| Pi 5 | `pi_camera_publisher` | CSI/libcamera GStreamer; `/joint_states` | compressed image, CameraInfo, optional tilt TF | dedicated capture thread; tilt watchdog timer |
| Laptop GPU | `gpu_vision_inference` | compressed image | detections, annotated JPEG, diagnostics, CSV/JSON | subscriber overwrites a single pending frame; worker thread; async telemetry logger |
| Laptop test | `mock_camera_publisher` | MP4, webcam, or GStreamer test pattern | same image and CameraInfo topics as Pi camera | dedicated timed capture thread |

No services or actions are implemented. No Nav2, SLAM, planner, costmap, EKF
package, dynamics simulator, or bag replay is wired into a launch file.

## Topic contract

| Topic | Type | Writer -> reader | QoS in code |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | external navigation/operator -> serial bridge | reliable, volatile, keep-last 10 (integer shorthand) |
| `/pico/robot_state` | `std_msgs/Float32MultiArray` | serial bridge -> odometry, LiDAR | writer reliable K10; odometry reader reliable K10; LiDAR reader reliable K100 |
| `/pico/timestamp_ms` | `std_msgs/UInt32` | serial bridge -> diagnostics/consumers | reliable, volatile, K10 |
| `/pico/system_status_code` | `std_msgs/UInt8` | serial bridge -> diagnostics/consumers | reliable, volatile, K10 |
| `/pico/system_power` | `std_msgs/Float32MultiArray` `[V,A]` | serial bridge -> consumers | reliable, volatile, K10 |
| `/pico/motors_power` | `std_msgs/Float32MultiArray` `[V,A]` | serial bridge -> consumers | reliable, volatile, K10 |
| `/joint_states` | `sensor_msgs/JointState` | serial bridge -> state/camera publishers | writer reliable K10; camera reader SensorData K5 |
| `/diagnostics/encoder_left` | `std_msgs/Int32` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/diagnostics/encoder_right` | `std_msgs/Int32` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/diagnostics/system_flags` | `std_msgs/UInt32` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/diagnostics/safe_state_active` | `std_msgs/Bool` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/diagnostics/command_fresh` | `std_msgs/Bool` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/diagnostics/i2c_fault_latched` | `std_msgs/Bool` | serial bridge -> Foxglove/consumer | reliable, volatile, K10 |
| `/odom` | `nav_msgs/Odometry` | odometry -> future localization/navigation | reliable, volatile, K10 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR aggregator -> future SLAM/navigation | SensorDataQoS (best-effort, volatile) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Pi/mock camera -> GPU inference/Foxglove | best-effort, volatile, K1, 150 ms lifespan on writers; SensorData K1 reader |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Pi/mock camera -> consumers | reliable, transient-local, K1 |
| `/vision/gpu/detections` | `distributed_vision_msgs/Detection2DArray` | inference -> consumers | best-effort, volatile, K1, 250 ms lifespan |
| `/vision/gpu/image_annotated/compressed` | `sensor_msgs/CompressedImage` | inference -> Foxglove/operator | best-effort, volatile, K1, 250 ms lifespan |
| `/diagnostics/vision_gpu` | `diagnostic_msgs/DiagnosticArray` | inference -> monitoring | reliable, volatile, K1 |

`Detection2DArray` copies the source header and adds image dimensions, source
frame age, inference time, backend name and pixel-coordinate boxes. ROS 2
`CompressedImage` has no sequence field; the inference logger therefore uses a
subscriber-local counter and estimates overwrite/drop behavior rather than
proving network packet loss.

## Wire protocol

Both directions use `AA BB | payload | CRC-low CRC-high`, little-endian,
CRC-16/IBM polynomial `0xA001`, initial `0xFFFF`.

- Pi -> Pico: 16-byte `RobotCommand`; 20-byte frame.
- Pico -> Pi: 76-byte `RobotState`; 80-byte frame.
- Both nominally 50 Hz on 921600 baud.
- Protocol-only line load: `(20 + 80) * 50 * 10 = 50,000 bit/s`, or 5.425%.

The Pico production safe state is required by an I2C fault, encoder staleness
over 100 ms, command staleness over 100 ms, or a cleared drive-enable flag.
However, the Pi bridge holds and refreshes the most recent upstream `/cmd_vel`
for 2.0 s. This makes the upstream timeout, not the Pico UART watchdog, the
effective bound after a navigation publisher silently stops.

## Frames and dynamic extrinsics

```text
odom
└── base_footprint            (odometry node)
    └── base_link             (odometry roll/pitch transform)
        ├── wheel/caster/deck frames
        ├── imu_link
        ├── laser_mount_link
        │   ├── laser_link    (canonical scan frame)
        │   └── laser_servo_link -> laser_sensor_link
        └── camera_tilt_mount_link
            └── camera_link -> camera_optical_frame
```

The serial bridge currently publishes a parameter fallback for
`camera_tilt_joint`; the firmware `RobotState` has no camera tilt feedback. The
camera publisher subscribes to that joint and can broadcast the dynamic transform
directly, but direct TF is off by default to avoid two authorities. Dynamic camera
extrinsics therefore remain an interface placeholder until real tilt feedback is
added to the wire protocol or another authoritative joint-state source.

## Critical parameter groups

- Serial/control: ports `/dev/ttyAMA0` and `/dev/serial0`, 921600 baud, command
  50 Hz, upstream timeout 2.0 s, wheel radius 0.03335 m, base 0.28 m, maximum
  100 RPM.
- LiDAR: 0–180°, forward 90°, 1° requested bins, 0.02–4.0 m, reversal 1°,
  launch timeout 0.25 s. Launch scale is 0.01 despite millimetre firmware field.
- Live/mock camera: 1280×720 at 30 FPS, JPEG quality 85, maximum 4 MiB encoded
  frame, 150 ms lifespan; calibration optional by default.
- Inference: 640×640 input, confidence 0.35, NMS 0.45, FP16 requested, GPU
  required and CPU fallback disabled by default, 2 warmups, 100 detections, stale
  drop at 500 ms, warning at 150 ms, academic queue 8192.

Exact declarations remain authoritative in the source and YAML; this map avoids
copying every validation-only parameter.

