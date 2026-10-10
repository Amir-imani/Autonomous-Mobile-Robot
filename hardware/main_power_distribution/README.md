# Main Power Distribution Board

Custom robot power input and distribution PCB designed in KiCad.

The board includes input power routing, fuse protection, transient suppression, bulk/filter capacitors and connections for current/voltage monitoring through an INA monitoring module. See the editable schematic for actual component values and wiring.

## Files
- `source/Input Power_robot.kicad_pro`
- `source/Input Power_robot.kicad_sch`
- `source/Input Power_robot.kicad_pcb`

## Documentation status

Only original KiCad design sources were supplied. **No Gerber, drill, BOM, assembly drawing or board photographs were provided**; none have been invented. The populated INA module is a distinct mounted assembly; verify its exact part before specifying the final BOM.

For manufacturing, run ERC/DRC and generate Gerber/drill from the reviewed source revision.
