# Autonomous Mobile Robot

Source repository for the undergraduate thesis project:

**Design and Construction of a Mobile Robot with Intelligent Path Planning in Dynamic Environments**

The project implements a differential-drive mobile robot with a layered architecture:

- **Low-level control:** RP2040 / Raspberry Pi Pico
- **On-board computing:** Raspberry Pi 5 + ROS 2
- **Remote compute:** GPU-oriented distributed vision processing
- **Navigation infrastructure:** odometry, LiDAR scan aggregation, TF/URDF, and preliminary SLAM integration
- **Experimental evaluation:** communication, control, offloading, timing, and stress-test tooling

## Repository layout

```text
code/
  firmware/pico/  RP2040 firmware and Neuro-PID training
  ros2/           ROS 2 packages, robot description, camera and vision nodes
  experiments/    Reproducible hardware-free benchmark suite

hardware/         Schematics, PCB sources, BOM, and fabrication files
mechanical/       CAD, STL, and mechanical drawings
results/          Curated thesis evaluation outputs
docs/             Thesis-facing documentation and appendix indexes
```

## Source-code status

The thesis source-code snapshot is now organized under `code/`, including:

- Pico low-level firmware and hardware drivers
- Neuro-PID runtime and offline training scripts
- Pico ↔ Raspberry Pi 5 binary serial bridge
- Differential odometry and servo-LiDAR scan generation
- Xacro/URDF robot description and RViz configuration
- Custom distributed-vision messages
- Raspberry Pi camera publication node
- Remote GPU inference node and academic telemetry
- Vision testbench
- Hardware-free audit, benchmark, offload, stress, and analysis tools

See [docs/APPENDIX_CODE_INDEX.md](docs/APPENDIX_CODE_INDEX.md) for the thesis-facing source index.

## Important scope note

The repository snapshot does not claim a complete Nav2/SLAM deployment or measured GPU inference performance. Hardware-dependent and preliminary subsystems are documented according to the evidence available in the thesis.

## Thesis snapshot

A fixed release/tag will be created at final thesis submission so thesis hyperlinks can target an immutable revision rather than the moving `main` branch.
