# Thesis documentation and appendices

The following indexes provide compact appendix citations, while full engineering files remain under the linked source directories.

| Thesis appendix | Document | Source root |
|---|---|---|
| A — Software and experiments | [Source code index](APPENDIX_CODE_INDEX.md) | [code](../code/) |
| B — Schematics and PCBs | [Electrical design index](APPENDIX_HARDWARE_INDEX.md) | [hardware](../hardware/) |
| C — SolidWorks and fabrication | [Mechanical design index](APPENDIX_MECHANICAL_INDEX.md) | [mechanical](../mechanical/) |

For implementation architecture, interfaces and code entry points see [the software guide](SOFTWARE_GUIDE.md). For each board or mechanical assembly consult the corresponding README under `hardware/` or `mechanical/`.

## Thesis evidence and revision policy

The software tests include source-executed checks and emulated workloads, which must not be presented as physical robot/GPU measurements. The hardware and mechanical file sets are engineering deliverables; manufacturing drawings do not by themselves certify testing or tolerances.

For final submission, link to the exact Git commit used for this thesis snapshot. The repository is private, so external reviewers need collaborator access or an intentionally published copy.
