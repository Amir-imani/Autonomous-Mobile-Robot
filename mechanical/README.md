# Mechanical Design — SolidWorks and Fabrication Files

CAD sources, STL meshes and PDF drawings for the custom robot chassis and mounting components are organized here. **[Appendix C mechanical index](../docs/APPENDIX_MECHANICAL_INDEX.md)** records each source filename, its part role, supported manufacturing quantity and material, and outstanding identification gaps.

## Summary for one robot

| Component | Qty | Material |
|---|---:|---|
| Upper and lower chassis sheets | 2 (one each) | 3 mm laser-cut sheet/board; exact board grade not recorded |
| Motor brackets (`4.1`, also `motor.pdf`) | 4 (two per motor) | ABS |
| Magnet/shaft adaptors (`mag`) | 2 | ABS-like resin |
| AS5600 sensor brackets | 2 | PLA+ |
| Camera holder | 1 | PLA+ |
| LiDAR holder | 1 | PLA+ |
| Raspberry Pi 5 underside fan/airflow assembly | 1 multi-part unit | Material/subpart count not recorded |

The file `Part21.SLDPRT` and the exact upper/lower naming of the chassis files remain unresolved. Their presence is documented without guessing a quantity or material.

## Source formats

- `.SLDPRT` — editable SolidWorks parts.
- `.STL` — mesh exports for additive manufacturing.
- `.pdf` — exported mechanical drawings and references.

A native `.SLDASM` assembly and consolidated fastener/material BOM are not included in this snapshot. The directory contains CAD/design artifacts and must not be interpreted as a complete manufacturing verification record.
