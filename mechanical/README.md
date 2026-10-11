# Mechanical design — SolidWorks and manufacturing references

Custom mechanical parts of the differential-drive robot are organized by assembly, with consistent basenames for CAD sources, STL exports and PDF drawings. See [Appendix C index](../docs/APPENDIX_MECHANICAL_INDEX.md) for **quantity, material, purpose and original filenames**.

| Directory | Part group | Make per robot | Material |
|---|---|---:|---|
| [`chassis/lower_deck/`](chassis/lower_deck/) | First chassis deck | 1 | 3 mm MDF, laser cut |
| [`chassis/upper_deck/`](chassis/upper_deck/) | Second chassis deck | 1 | 3 mm MDF, laser cut |
| [`motor_mounts/`](motor_mounts/) | Four identical motor brackets | 4 | ABS, 3D printed |
| [`encoder_couplers/`](encoder_couplers/) | Magnet-to-shaft interfaces | 2 | ABS-like printed resin |
| [`sensor_mounts/`](sensor_mounts/) | AS5600 mounts (2), camera mount (1), LiDAR mount (1) | 4 | PLA+, 3D printed |
| [`thermal_airflow/`](thermal_airflow/) | Interlocking Raspberry Pi 5 airflow guide for fan | 1 assembly (multipart) | 3 mm MDF, laser cut |

The fan/airflow guide has an **unknown count of individual panels** in its interlocking assembly; do not treat one complete assembly as one laser-cut panel.

## Original-to-new filename correspondence

- `1.pdf` + `2.SLDPRT` → `chassis/lower_deck/lower_chassis_plate.*`
- `2.pdf` + `Part21.SLDPRT` → `chassis/upper_deck/upper_chassis_plate.*`
- `4.1.SLDPRT`, `4.1.STL`, `motor.pdf` → `motor_mounts/motor_mount_bracket.*`
- `mag.SLDPRT`, `mag.STL`, `magnet.pdf` → `encoder_couplers/magnet_shaft_coupler.*`
- `AS5600.*` → `sensor_mounts/as5600_encoder_mount.*`
- `camera.*`, `camera holder.pdf` → `sensor_mounts/camera_mount.*`
- `LiDAR.*`, `LiDaR.pdf` → `sensor_mounts/lidar_mount.*`
- `Side.SLDPRT` + `Fan1.pdf` → `thermal_airflow/raspberry_pi5_fan_duct.*`

All relocated CAD and drawing files retain their original binary Git blobs. No source geometry, dimensions, print profiles or PDF pages were changed during renaming. SolidWorks external references, if any, should be rechecked when opening renamed models.
