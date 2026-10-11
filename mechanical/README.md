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

## Design records and use

Each part group contains its available editable SolidWorks sources (`.SLDPRT`), print mesh exports (`.STL`), and/or reference drawings (`.pdf`). The two 3 mm MDF chassis decks are separate pieces (one each). The fan duct is one interlocking MDF assembly whose number of individual panels is not specified. Confirm fits and SolidWorks references before fabrication.
