# Autonomous Mobile Robot

Thesis project: **Design and Construction of a Mobile Robot with Intelligent Path Planning in Dynamic Environments**.

This repository contains the code, electronic design sources, and mechanical models for a differential-drive mobile robot. The software separates time-sensitive low-level control (RP2040), on-board ROS 2 integration (Raspberry Pi 5), and remote vision inference (external compute).

## Start here

- **[Software architecture and complete source guide](docs/SOFTWARE_GUIDE.md)** — subsystem flow, source-file descriptions, dependencies, how to navigate/run the code, and evidence limitations.
- **[Appendix A source index](docs/APPENDIX_CODE_INDEX.md)** — compact software source links for the thesis.
- **[Appendix B electrical design index](docs/APPENDIX_HARDWARE_INDEX.md)** — five circuit schematics, KiCad projects, PCB layouts and available fabrication outputs.
- **[Code overview](code/README.md)** — package entry points.
- **[Experimental methodology](code/experiments/README.md)** — reproducible source checks, host emulations and hardware prerequisites.

## Repository layout

```text
code/
  firmware/pico/       RP2040 firmware, drivers, Neuro-PID training
  ros2/                Six ROS 2 packages: bridge, model, camera, GPU inference, messages, testbench
  experiments/         Source checks, host benchmarks, controlled emulation and analysis
hardware/              KiCad schematics, PCBs and manufacturing outputs
mechanical/            CAD and exported mechanical drawings
results/               Thesis results directory
docs/                  Appendix index and architecture guide
```

## Implementation scope and evidence

The repository contains closed-loop wheel-control firmware, sensor interfaces, a Pico ↔ Pi binary bridge, wheel odometry, LiDAR sample aggregation, robot description, compressed-image publication, a GPU-capable inference node, and experiment tooling. It does **not** bundle a complete evaluated Nav2/SLAM deployment, a trained ONNX model, or verified physical GPU performance results. Host emulations and software tests must be reported separately from physical measurements.

## Stable thesis citations

For final thesis citations, prefer a full commit-SHA URL (a fixed snapshot) instead of a moving `main` link. A release tag is optional and should only be cited after it actually exists. The repository is private; reviewers need authorized GitHub access.
