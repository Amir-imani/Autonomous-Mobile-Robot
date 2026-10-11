# Electrical design sources

Five custom electrical boards for the autonomous mobile robot are stored here. For a compact table of **every board's schematic, PCB, project file, and available manufacturing outputs**, see [Appendix B — Electrical design index](../docs/APPENDIX_HARDWARE_INDEX.md).

## Boards and documentation

| Board | Purpose | Detailed notes |
|---|---|---|
| [Main power distribution](main_power_distribution/) | Input routing, fuse and transient protection, INA monitor interfaces | [README](main_power_distribution/README.md) |
| [Motor power distribution](motor_power_distribution/) | Fused motor power routing and measurement interfaces | [README](motor_power_distribution/README.md) |
| [Raspberry Pi Pico carrier](pico_carrier_board/) | RP2040 I/O/power breakout to robot subsystems | [README](pico_carrier_board/README.md) |
| [Raspberry Pi 5 overvoltage protection](raspberry_pi5_overvoltage_protection/) | Fuse, TVS and crowbar-style protection design | [README](raspberry_pi5_overvoltage_protection/README.md) |
| [STM8 fan/temperature controller](stm8_fan_controller/) | NTC input and dual fan-PWM electronics | [README](stm8_fan_controller/README.md) |

Each directory's `source/` has the editable KiCad `.kicad_pro`, `.kicad_sch` and `.kicad_pcb` trio.

## Available outputs and limitations

- Pico carrier and Pi 5 protection directories include Gerber/drill exports and images.
- STM8 directory includes a single-page schematic PDF and a 3D image, but no STM8 firmware in this repository. This board was fabricated and mounted, **not run as a validated active subsystem in the thesis experiments**.
- Main/motor distribution boards have the editable KiCad triplets, but no packaged Gerber/drill exports, board photographs or structured BOMs.
- A KiCad schematic or Gerber package alone does not demonstrate ERC/DRC clearance or the tested electrical behavior. Confirm the as-built revision and rerun checks before manufacture.
