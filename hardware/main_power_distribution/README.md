# Main Power Distribution Board

The main power PCB provides input routing, fuse protection, transient suppression, filtering/bulk capacitors and connection points for an INA-series voltage/current monitoring module. Electrical values and pin assignments are defined in the editable schematic.

## Design files

- [KiCad schematic](source/Input%20Power_robot.kicad_sch)
- [KiCad PCB](source/Input%20Power_robot.kicad_pcb)
- [KiCad project](source/Input%20Power_robot.kicad_pro)

## Manufacturing

The editable KiCad source is the design reference. To manufacture a board, inspect footprints and connector assignments, run ERC/DRC, and generate Gerber/drill outputs from the approved board revision. The mounted INA-series measurement module should be identified from the physical assembly when compiling a final BOM.
