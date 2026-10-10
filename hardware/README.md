# Hardware

Hardware design files developed for the autonomous mobile robot are organized by custom PCB or electrical subsystem.

## Boards currently available

### Raspberry Pi Pico Carrier / Header Board

[Open the Pico carrier board directory](pico_carrier_board/)

Included:

- KiCad project, schematic, and PCB layout
- Complete two-layer Gerber set
- PTH and NPTH drill files
- Top and bottom 3D renders
- PCB Editor layout image

### Raspberry Pi 5 Overvoltage Protection Board

[Open the Raspberry Pi 5 protection board directory](raspberry_pi5_overvoltage_protection/)

Included:

- KiCad project, schematic, and PCB layout
- Complete two-layer Gerber set
- PTH and NPTH drill files
- 3D PCB render
- Schematic view
- README describing the protection role and thesis-use notes

Additional robot PCBs, BOMs, fabrication notes, and assembly documentation will be added here as their final design snapshots are prepared.

### STM8 Dual-Channel Temperature and Fan Controller

[Open the STM8 fan controller directory](stm8_fan_controller/)

Included:
- Editable KiCad project, schematic and PCB design
- One-page PDF schematic
- Top-side 3D board render

**Thesis status:** Board fabricated and mounted on the robot, but not used for active control or experimental validation. Gerber, drill, BOM and firmware were not supplied with this package.

### Main Power Distribution Board

[Open main power board](main_power_distribution/) — power input, fuse protection, distribution and INA-module measurement mounting. Original KiCad project, schematic and PCB source.

### Motor Power Distribution Board

[Open motor power board](motor_power_distribution/) — fused motor power routing and INA-module measurement mounting. Original KiCad project, schematic and PCB source.

For these two boards, photographs, Gerber/drill sets and BOMs were not included in the supplied ZIPs. They can be added separately after final review.
