# Hardware-free experimental campaign

This directory is isolated instrumentation for the distributed AMR codebase. It
does not change robot runtime behavior. Raw output is immutable per run and every
result carries an evidence class.

## Actual architecture found

The repository contains six ROS 2 packages, not a complete navigation stack:

1. `pico_serial_bridge` (Python): bidirectional 921600-baud Pico framing, `/cmd_vel`
   conversion, telemetry fan-out, joint state, wheel odometry, and servo-LiDAR
   aggregation.
2. `amr_description` (CMake): Xacro TF/geometry, RViz display launch; no dynamics
   simulator.
3. `distributed_vision_msgs` (CMake): `Detection2D` and `Detection2DArray` messages.
4. `pi_camera_publisher` (C++): libcamera/GStreamer JPEG authority, CameraInfo,
   tilt feedback, optional dynamic TF.
5. `gpu_vision_inference` (C++): OpenCV DNN YOLO, CUDA/FP16 preference, optional
   nvJPEG, latest-frame worker, detections/annotated JPEG, diagnostics and academic
   telemetry.
6. `vision_testbench` (C++): video/webcam/test-pattern GStreamer publisher that
   follows the production compressed-image topic and QoS contract.

The core launch starts `robot_state_publisher`, serial bridge, differential
odometry, and LiDAR scan translation. No Nav2, SLAM Toolbox, Cartographer, Gazebo,
Webots, rosbag, ONNX model, video, or labeled dataset is present. Services and
actions are absent.

### Principal data paths

```text
/cmd_vel (Twist, reliable K10)
  -> serial_bridge_node -> AA BB + RobotCommand + CRC -> Pico @ 50 Hz

Pico RobotState + CRC @ 50 Hz
  -> serial_bridge_node
     -> /pico/robot_state (Float32MultiArray, reliable K10)
        -> differential_odometry_node -> /odom + odom/base TF
        -> lidar_scan_node -> /scan (SensorDataQoS)
     -> /joint_states + typed diagnostic/power scalar-array topics

Pi/mock GStreamer JPEG
  -> /camera/image_raw/compressed (best-effort, volatile, K1, lifespan 150 ms)
  -> gpu_vision_inference (SensorDataQoS K1, latest-frame overwrite)
     -> /vision/gpu/detections (best-effort, volatile, K1)
     -> /vision/gpu/image_annotated/compressed (same QoS)
     -> /diagnostics/vision_gpu (reliable, volatile, K1)
```

CameraInfo is reliable/transient-local/keep-last-1. The camera node normally lets
`robot_state_publisher` own the camera tilt joint TF; direct dynamic broadcasting
is an opt-in placeholder and warns about duplicate TF authority.

## Evidence classes and test plan

| Class | Meaning | Executed by `run_all.py` |
|---|---|---|
| A | Direct execution of repository software without physical I/O | CRC/framing/parser, RobotState validation, production odometry math, LiDAR aggregation, optimized production Neuro_PID C++ host build, Python syntax audit |
| B | Synthetic input, model, replay, or emulation | Source-derived command/safety checks; ideal differential-drive command-to-pose trace; local vs loopback-process compute; application-layer delay/jitter/loss/bandwidth; concurrent CPU stress |
| C | Partially testable or analytical | Serial capacity arithmetic; ROS 2 probe and Linux netem scripts prepared but not run on this Windows host; vision build blocked by ROS/OpenCV/GStreamer/model |
| D | Requires physical hardware/ground truth | Motor tracking, encoder/IMU accuracy, real stop distance, sensor calibration, Pi resource/thermal data, CSI/ISP performance, Wi-Fi/Ethernet DDS behavior, VO/SLAM accuracy |

### Metrics realistically available now

- A: pass/fail robustness, operation time/throughput on the development CPU,
  process CPU and RSS, deterministic odometry and LiDAR state behavior.
- B: end-to-end surrogate latency, processing time, process split CPU/RSS,
  throughput, seeded drops, latency variation, impairment sensitivity, and a
  50 Hz scheduling-lateness test under 0/1/4/16 CPU workers.
- C: theoretical serial/image payload bandwidth. ROS topic frequency, DDS loss,
  callback/executor latency and real vision telemetry become executable on Ubuntu
  ROS 2 Humble with the required packages/model.
- D: navigation-loop, planner, costmap and SLAM timings cannot be tested because
  those components do not exist in this codebase. Accuracy requires ground truth.

Jitter in the host emulation is defined as the mean absolute difference between
consecutive end-to-end latencies, `mean(|L[n]-L[n-1]|)`. Latency standard deviation
is reported separately. The future ROS probe also logs inter-arrival intervals.

## Run the current campaign

Windows PowerShell:

```powershell
Set-Location 'D:\Code Prj\PI5 & ROS2\experiments'
.\run_all.ps1
```

Linux:

```bash
cd '/path/to/PI5 & ROS2/experiments'
./run_all.sh
```

Defaults use three trials, 60 frames per impairment condition, a 150 KiB
synthetic payload, and fixed random seeds. `run_manifest.json` records exact argv,
environment, duration, return code, stdout and stderr for every step. `LATEST.txt`
points to the newest run; prior runs are never deleted or overwritten.

## ROS 2/DDS measurement on Ubuntu Humble

Build the workspace and run publisher/subscriber on the same synchronized host
first. For two hosts, measure and report residual PTP/NTP clock error because a
header timestamp and receive time otherwise do not define one-way latency.

```bash
python3 ros2_latency_probe.py --role both --rate 30 --payload-bytes 153600 \
  --duration 60 --output results/ros2_baseline.csv
```

The probe uses best-effort, volatile, keep-last-1 to match the camera path. The
synthetic sequence is encoded in `header.frame_id` solely for test instrumentation.
This is intentionally not claimed as a production image message extension.

## Linux network impairment

Apply only to an explicit test interface. Never use the management interface over
which you may lose access. Record the baseline qdisc and always clear after a run.

```bash
./network_emulation.sh show --interface veth-vision
./network_emulation.sh apply --interface veth-vision --delay-ms 25 --jitter-ms 10 --loss-percent 1 --rate-mbit 20
./network_emulation.sh clear --interface veth-vision
```

Use `tc/netem` runs in separate network namespaces/hosts for defensible network
evidence. The built-in Windows experiment is application-layer emulation only.

## Real vision benchmark (deferred prerequisites)

The existing `vision_testbench/launch/academic_benchmark.launch.py` is the correct
pipeline. It requires Ubuntu ROS 2 Humble, GStreamer plugins, OpenCV built with
CUDA (and optionally nvJPEG), a compatible YOLOv8 ONNX file and a declared video
or test pattern. Its CSV/JSON telemetry already records frame size/age, decode,
inference, total latency, inter-arrival jitter, overwrite/drop estimates and FPS.
Do not call those metrics GPU measurements unless diagnostics say the CUDA backend
and desired decoder were active.

## Interpretation rules

- Laptop microbenchmarks characterize software cost on this host only.
- The SHA-256 workload is never described as AI inference.
- Loopback TCP is never described as DDS or a physical network.
- Analytical payload bandwidth excludes middleware and link overhead.
- Simulated no-slip pose is never localization or tracking accuracy.
- A failed test remains in raw CSV and makes the campaign return nonzero.
