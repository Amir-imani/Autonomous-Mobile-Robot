# Appendix C — SolidWorks and mechanical fabrication files

This index documents the robot's mechanical CAD deliverables for the thesis. Files are preserved using their original filenames. Counts/materials below combine the mechanical-source inventory with the project's confirmed fabrication notes; a CAD file by itself does **not** establish print material or required build quantity.

## General construction

The robot uses two main chassis plates with a two-level frame, laser-cut from **3 mm sheet material** (described in the Persian thesis as *تخته*, board/sheet; the exact grade/type of wood or composite is not established in the repository). There is **one of each chassis plate**. Custom mounting parts are primarily printed; the exceptions specified below are the ABS motor brackets and ABS-like resin magnet couplers.

## Manufacturing overview

| Mechanical part / assembly | Quantity per robot | Material | Relevant repository files | Notes |
|---|---:|---|---|---|
| Main chassis plates, upper/lower | 2 total (1 each) | Laser-cut 3 mm board/sheet, exact substrate unspecified | [1.pdf](1.pdf), [2.pdf](2.pdf), [2.SLDPRT](2.SLDPRT) | Naming-to-upper/lower mapping needs drawing review; these are the chassis drawing/source candidates, not necessarily a one-file-per-plate pairing |
| Motor mounting bracket | 4 | ABS, 3D printed | [4.1.SLDPRT](4.1.SLDPRT), [4.1.STL](4.1.STL), [motor.pdf](motor.pdf) | Two brackets per motor: upper and lower; motor.pdf refers to the same bracket design |
| Magnet/shaft coupling mount | 2 | ABS-like 3D-printing resin | [mag.SLDPRT](mag.SLDPRT), [mag.STL](mag.STL), [magnet.pdf](magnet.pdf) | Holds/rotates magnets opposite the AS5600 magnetic encoders; two magnets are used |
| AS5600 sensor mounting bracket | 2 | PLA+, 3D printed | [AS5600.SLDPRT](AS5600.SLDPRT), [AS5600.STL](AS5600.STL), [AS5600.pdf](AS5600.pdf) | One per magnetic encoder |
| Camera mounting bracket | 1 | PLA+, 3D printed | [camera.SLDPRT](camera.SLDPRT), [camera.STL](camera.STL), [camera holder.pdf](camera%20holder.pdf) | One camera holder |
| LiDAR mounting bracket | 1 | PLA+, 3D printed | [LiDAR.SLDPRT](LiDAR.SLDPRT), [LiDAR.STL](LiDAR.STL), [LiDaR.pdf](LiDaR.pdf) | One servo-scanned range-sensor mount |
| Fan/airflow guide under Raspberry Pi 5 | 1 multi-part assembly | Material not confirmed | [Side.SLDPRT](Side.SLDPRT), [Fan1.pdf](Fan1.pdf) | Snap-fit multipart air duct/guide; one complete assembly, number of its subparts not yet established |
| Additional/unmapped CAD model | Not established | Not established | [Part21.SLDPRT](Part21.SLDPRT) | Kept with its original name; do not assign a manufacturing quantity or claim an identity until inspected |

## Actual files

```text
mechanical/
├── 1.pdf
├── 2.SLDPRT
├── 2.pdf
├── 4.1.SLDPRT
├── 4.1.STL
├── motor.pdf
├── AS5600.SLDPRT
├── AS5600.STL
├── AS5600.pdf
├── LiDAR.SLDPRT
├── LiDAR.STL
├── LiDaR.pdf
├── camera.SLDPRT
├── camera.STL
├── camera holder.pdf
├── mag.SLDPRT
├── mag.STL
├── magnet.pdf
├── Side.SLDPRT
├── Fan1.pdf
└── Part21.SLDPRT
```

## File formats and assembly guidance

- **`.SLDPRT`**: editable SolidWorks part sources (the provided snapshot does not include a complete native `.SLDASM` assembly).
- **`.STL`**: mesh export for 3D-print preparation; its presence does not establish slicer settings, infill, layer height, tolerances or filament brand.
- **`.pdf`**: exported design/manufacturing reference drawings; consult the actual drawing to assign geometric dimensions or precise chassis-layer names.
- Four motor brackets correspond to **two per motor** (top/bottom) for a two-wheel drive.
- The two `mag` couplers should not be confused with the **two physical encoder magnets**.
- The fan/airflow subsystem is **one assembled unit**, although it has multiple interlocking parts; do not write that its entire assembly is one printed piece.
- Unless explicitly confirmed by physical inventory, do not infer manufacture count/material from `Part21.SLDPRT`, `Side.SLDPRT`, or file count alone.

## Reproduction notes

The SolidWorks models are original editable design records, STL files are exports, and PDFs are reference drawings. Check dimensions, print orientation, mounting clearances and as-built changes before reproducing mechanical hardware. The repository does not substantiate a complete BOM with fasteners, supports, joints or individually verified fan subpart counts.

For the submitted thesis, cite a **specific commit SHA** or a checked release tag for fixed-source references. This repository is private, so examiners require explicit access.
