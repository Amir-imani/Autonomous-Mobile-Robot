# Appendix C — Mechanical design and fabrication index

This index identifies the CAD parts and drawing exports in the reorganized thesis mechanical source tree. The 21 original binary files were **moved and renamed without editing their blob contents**. See the project history for the legacy filenames.

## Structured files and assembly quantities

| Subsystem | File basename and directory | Quantity per robot | Material / manufacturing | Original file(s) |
|---|---|---:|---|---|
| Chassis, lower (first) deck | [`chassis/lower_deck/lower_chassis_plate`](../mechanical/chassis/lower_deck/) | 1 | 3 mm MDF, laser cut | `1.pdf` and `2.SLDPRT` |
| Chassis, upper (second) deck | [`chassis/upper_deck/upper_chassis_plate`](../mechanical/chassis/upper_deck/) | 1 | 3 mm MDF, laser cut | `2.pdf` and `Part21.SLDPRT` |
| Motor mounting bracket | [`motor_mounts/motor_mount_bracket`](../mechanical/motor_mounts/) | 4 | ABS, 3D printed | `4.1.SLDPRT`, `4.1.STL`, `motor.pdf` |
| Motor-shaft-to-magnet coupler | [`encoder_couplers/magnet_shaft_coupler`](../mechanical/encoder_couplers/) | 2 | ABS-like resin, 3D printed | `mag.SLDPRT`, `mag.STL`, `magnet.pdf` |
| AS5600 encoder bracket | [`sensor_mounts/as5600_encoder_mount`](../mechanical/sensor_mounts/) | 2 | PLA+, 3D printed | `AS5600.SLDPRT`, `AS5600.STL`, `AS5600.pdf` |
| Camera bracket | [`sensor_mounts/camera_mount`](../mechanical/sensor_mounts/) | 1 | PLA+, 3D printed | `camera.SLDPRT`, `camera.STL`, `camera holder.pdf` |
| LiDAR bracket | [`sensor_mounts/lidar_mount`](../mechanical/sensor_mounts/) | 1 | PLA+, 3D printed | `LiDAR.SLDPRT`, `LiDAR.STL`, `LiDaR.pdf` |
| Raspberry Pi 5 fan duct and airflow guide | [`thermal_airflow/raspberry_pi5_fan_duct`](../mechanical/thermal_airflow/) | 1 assembled set (multiple interlocking pieces) | 3 mm MDF, laser cut | `Side.SLDPRT`, `Fan1.pdf` |

**User-confirmed identification:** `1.pdf` and `2.SLDPRT` describe the **first/lower** chassis deck, while `2.pdf` and `Part21.SLDPRT` describe the **second/upper** deck. `Side.SLDPRT` and `Fan1.pdf` are two representations of one multipart duct design, and `4.1` is the four-copy motor bracket.

### File structure

```text
mechanical/
├── chassis/
│   ├── lower_deck/
│   │   ├── lower_chassis_plate.SLDPRT
│   │   └── lower_chassis_plate.pdf
│   └── upper_deck/
│       ├── upper_chassis_plate.SLDPRT
│       └── upper_chassis_plate.pdf
├── motor_mounts/
│   ├── motor_mount_bracket.SLDPRT
│   ├── motor_mount_bracket.STL
│   └── motor_mount_bracket.pdf
├── encoder_couplers/
│   ├── magnet_shaft_coupler.SLDPRT
│   ├── magnet_shaft_coupler.STL
│   └── magnet_shaft_coupler.pdf
├── sensor_mounts/
│   ├── as5600_encoder_mount.{SLDPRT,STL,pdf}
│   ├── camera_mount.{SLDPRT,STL,pdf}
│   └── lidar_mount.{SLDPRT,STL,pdf}
└── thermal_airflow/
    ├── raspberry_pi5_fan_duct.SLDPRT
    └── raspberry_pi5_fan_duct.pdf
```

## Mechanical design notes

- Two distinct, laser-cut 3 mm MDF deck plates, **one each**. `Part21.SLDPRT` is the upper chassis source, not an unknown extra part.
- Motor bracket `motor_mount_bracket`: print **four identical units in ABS**, two for each motor (top/bottom).
- Magnet shaft coupler: make **two in ABS-like resin**, one per drive wheel for correctly aligning the rotating magnet to its AS5600 encoder. The coupler is **not itself the magnet**.
- AS5600 bracket: **two** PLA+ printed mounts.
- Camera mount and LiDAR mount: **one of each** in PLA+.
- Fan guide: **one complete interlocking MDF assembly** routing airflow from a fan beneath Raspberry Pi 5. The provided information does not specify the count of individual MDF pieces, fan's exact model/size or the fan's electrical specifications; these are **not inferred** from the number of CAD/PDF files.
- Filament colors, slicing parameters, fasteners, tolerances, explicit assembly drawings and board/cutting layouts are not independently established in the supplied files.
- Counts are physical quantities to fabricate for one robot, **not** numbers of distinct digital files.

## Formats and reproduction

- `.SLDPRT`: editable SolidWorks part source.
- `.STL`: triangulated model for preparing 3D-printed components. These do not replace editable CAD.
- `.pdf`: design drawing or cut reference, not necessarily ready-to-machine CNC/G-code.
- The supplied repository does not contain a native SolidWorks `.SLDASM` assembly. The fan assembly is described through the provided model/drawing pair.

Before producing hardware, check dimensions, mating fit, part orientation and source-to-drawing consistency in SolidWorks; file renaming does not constitute geometric validation.

For thesis links, use the fixed commit of the selected branch. GitHub repository privacy may require reviewer invitations.
