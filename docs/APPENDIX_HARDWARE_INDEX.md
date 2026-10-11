# Appendix B — Electrical schematics and PCB design files

This repository index accompanies the Persian thesis Appendix B. The editable KiCad projects are under [`hardware/`](../hardware/), grouped into five independently documented circuit boards. Each board has a schematic (`.kicad_sch`), PCB layout (`.kicad_pcb`) and KiCad project settings (`.kicad_pro`). The `.kicad_sch` files describe connections, `.kicad_pcb` contains board layout/routing, and `.kicad_pro` stores project-level settings; none is interchangeable with fabrication outputs.

| Subsystem | Schematic | PCB layout | Project | Additional evidence |
|---|---|---|---|---|
| Main power input and distribution | [Input Power_robot.kicad_sch](../hardware/main_power_distribution/source/Input%20Power_robot.kicad_sch) | [PCB](../hardware/main_power_distribution/source/Input%20Power_robot.kicad_pcb) | [Project](../hardware/main_power_distribution/source/Input%20Power_robot.kicad_pro) | [Board notes](../hardware/main_power_distribution/README.md) |
| Motor power distribution | [Motor_P_robot.kicad_sch](../hardware/motor_power_distribution/source/Motor_P_robot.kicad_sch) | [PCB](../hardware/motor_power_distribution/source/Motor_P_robot.kicad_pcb) | [Project](../hardware/motor_power_distribution/source/Motor_P_robot.kicad_pro) | [Board notes](../hardware/motor_power_distribution/README.md) |
| RP2040 / Pico carrier board | [RP_Pico_robot.kicad_sch](../hardware/pico_carrier_board/source/RP_Pico_robot.kicad_sch) | [PCB](../hardware/pico_carrier_board/source/RP_Pico_robot.kicad_pcb) | [Project](../hardware/pico_carrier_board/source/RP_Pico_robot.kicad_pro) | [Gerbers/drills](../hardware/pico_carrier_board/fabrication/), [design images](../hardware/pico_carrier_board/images/), [notes](../hardware/pico_carrier_board/README.md) |
| Raspberry Pi 5 overvoltage protection | [RP_robot.kicad_sch](../hardware/raspberry_pi5_overvoltage_protection/source/RP_robot.kicad_sch) | [PCB](../hardware/raspberry_pi5_overvoltage_protection/source/RP_robot.kicad_pcb) | [Project](../hardware/raspberry_pi5_overvoltage_protection/source/RP_robot.kicad_pro) | [Gerbers/drills](../hardware/raspberry_pi5_overvoltage_protection/fabrication/), [images](../hardware/raspberry_pi5_overvoltage_protection/images/), [notes](../hardware/raspberry_pi5_overvoltage_protection/README.md) |
| STM8 fan and temperature controller | [Fan_robot_V3.kicad_sch](../hardware/stm8_fan_controller/source/Fan_robot_V3.kicad_sch) | [PCB](../hardware/stm8_fan_controller/source/Fan_robot_V3.kicad_pcb) | [Project](../hardware/stm8_fan_controller/source/Fan_robot_V3.kicad_pro) | [PDF schematic](../hardware/stm8_fan_controller/schematic/Fan_robot_V3.pdf), [3D render](../hardware/stm8_fan_controller/images/pcb_3d_top.jpg), [notes](../hardware/stm8_fan_controller/README.md) |

## Board purposes and boundaries

1. **Main power distribution:** input power routing, fuse protection, transient suppression and connections for current/voltage measurement via an INA-series monitoring module. Refer to schematic for exact values; mounted module identification is not firmly documented.
2. **Motor power distribution:** fused motor supply routing and monitoring-module connections; distinct from the BTS7960 motor driver/controller.
3. **Pico carrier:** breaks out Raspberry Pi Pico power and GPIO lines to system connectors. Full two-layer fabrication output and board-view images are included.
4. **Pi 5 protection:** schematic records fuse, TVS and crowbar-style protection elements intended for the Pi supply rail. The nominal Zener marking is **not** a verified board-level trip voltage; actual transient/fuse clearing requires physical testing.
5. **STM8 thermal board:** STM8S003F3P temperature inputs, PWM fan outputs, indicators and interfaces. Designed, fabricated and installed but **not activated or experimentally validated** in this thesis. No STM8 firmware is in this repository.

## Evidence and manufacture status

- **Complete KiCad project triplets:** present for all five boards.
- **Gerber and PTH/NPTH drill outputs:** present only for Pico carrier and Pi 5 protection board.
- **Standalone schematic PDF:** present for STM8 board only.
- **Structured BOMs and assembly drawings:** not present as complete released packages for these five circuits.
- **Validation:** source and fabrication artifacts do not establish that ERC/DRC passed or electrical/safety performance was measured. Recheck connector pinouts, component footprints, board revisions and protection performance before manufacturing or deployment.

## How to inspect

Open each `.kicad_pro` in a compatible KiCad version to navigate its schematic and PCB. Inspect the board-specific README for available exported images and fabrication files. When citing in a thesis, use the **fixed commit SHA** corresponding to the submitted source snapshot, not a moving branch ref. Because this repository is private, reviewers need authorized access.
