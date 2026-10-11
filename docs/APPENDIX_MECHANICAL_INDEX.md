# Appendix C — Mechanical design and fabrication index

The robot's mechanical sources are arranged by subsystem. The quantities refer to one complete robot; all material specifications below correspond to the documented build.

## Parts, quantities and materials

| Part / subsystem | Source folder | Quantity | Material and process |
|---|---|---:|---|
| First (lower) chassis deck | [`chassis/lower_deck/`](../mechanical/chassis/lower_deck/) | 1 | 3 mm MDF, laser cut |
| Second (upper) chassis deck | [`chassis/upper_deck/`](../mechanical/chassis/upper_deck/) | 1 | 3 mm MDF, laser cut |
| Symmetrical motor mount bracket | [`motor_mounts/`](../mechanical/motor_mounts/) | 4 | ABS, 3D printed |
| Rotating magnet-to-motor-shaft coupler | [`encoder_couplers/`](../mechanical/encoder_couplers/) | 2 | ABS-like resin, 3D printed |
| AS5600 encoder mount | [`sensor_mounts/`](../mechanical/sensor_mounts/) | 2 | PLA+, 3D printed |
| Camera mount | [`sensor_mounts/`](../mechanical/sensor_mounts/) | 1 | PLA+, 3D printed |
| LiDAR mount | [`sensor_mounts/`](../mechanical/sensor_mounts/) | 1 | PLA+, 3D printed |
| Raspberry Pi 5 fan airflow guide | [`thermal_airflow/`](../mechanical/thermal_airflow/) | 1 complete multipart assembly | 3 mm MDF, laser cut |

## File structure

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

## Fabrication and assembly notes

- **Chassis:** two separate laser-cut 3 mm MDF plates (one lower and one upper).
- **Motor mounts:** four identical ABS brackets, two per motor, installed in a mirrored/symmetrical arrangement.
- **Encoder couplers:** two ABS-like resin interfaces between rotating motor shafts and encoder magnets; the couplers are distinct from the magnets themselves.
- **AS5600 mounts:** two PLA+ printed sensor brackets.
- **Camera and LiDAR mounts:** one PLA+ printed mount for each sensor.
- **Thermal airflow guide:** one multipart, interlocking 3 mm MDF housing guiding fan airflow under Raspberry Pi 5. The number of individual MDF pieces and the fan model are not established in the repository.

## Source formats and usage

- **`.SLDPRT`**: editable SolidWorks component design.
- **`.STL`**: triangular mesh export suitable for preparing 3D prints.
- **`.pdf`**: drawing or cutting reference.
- This source snapshot does not include a native `.SLDASM` assembly, complete fastener inventory or validated printing profiles. Verify source-to-drawing agreement, dimensions, external CAD references and fits before reproducing hardware.

For stable thesis citations, use the final selected revision's commit SHA. Repository access remains subject to its private visibility.
