# Autonomous Mobile Robot

Source repository for the undergraduate thesis project:

**Design and Construction of a Mobile Robot with Intelligent Path Planning in Dynamic Environments**

The project implements a differential-drive mobile robot with a layered architecture:

- **Low-level control:** RP2040 / Raspberry Pi Pico
- **On-board computing:** Raspberry Pi 5 + ROS 2
- **Remote compute:** GPU-accelerated vision and distributed processing
- **Navigation stack infrastructure:** odometry, LiDAR scan aggregation, TF/URDF, and preliminary SLAM integration
- **Experimental evaluation:** communication, control, offloading, timing, and stress-test tooling

## Repository layout

```text
code/
  firmware/       RP2040/Pico firmware and Neuro-PID training
  ros2/           ROS 2 packages and robot description
  vision/         Camera, GPU inference, and custom vision messages
  experiments/    Reproducible evaluation and benchmark scripts

hardware/         Schematics, PCB sources, BOM, and fabrication files
mechanical/       CAD, STL, and mechanical drawings
results/          Curated thesis evaluation outputs
docs/             Thesis-facing documentation and figures
```

> The repository is being assembled from the final thesis implementation. Hardware, mechanical, and curated result artifacts will be added in subsequent commits.

## Thesis snapshot

This repository will be tagged at the final thesis submission state so links in the thesis remain reproducible even if development continues afterward.
