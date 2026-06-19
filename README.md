# BUPT 2026 EDC SmartCar

Hardware, firmware notes, and competition preparation materials for the BUPT 2026 Electronic Design Contest smart car project.

## Current Scope

The first milestone is a reliable baseline car:

- Line following is handled by MSPM0G3507 and the 8-channel grayscale sensor.
- CanMV-K230 is used only for visual correction of the pan-tilt target module.
- The chassis uses four MG310 7.4V Hall encoder geared motors.
- Two TB6612FNG modules drive the four motors.
- PCA9685 drives SG90 180-degree servos for the pan-tilt module.

## Repository Layout

```text
.
├─ docs/
│  └─ 北邮2026电赛小车_V1电路设计与采购清单.md
├─ references/
│  └─ vendor/
│     ├─ README.md
│     └─ 车车/                  # local vendor package, ignored by Git
└─ README.md
```

## Hardware Direction

The project keeps the control responsibilities separated:

- MSPM0G3507 is the main decision controller.
- Grayscale sensor, IMU, and motor encoders provide navigation feedback.
- K230 reports target offset through UART after the car reaches the operation area.
- MSPM0G3507 outputs motor PWM/direction, PCA9685 servo commands, and laser switch control.

## Notes

The vendor material folder is large, about 3.2 GB, and contains installers, disk images, videos, and PDFs. It is kept locally under `references/vendor/车车/` but ignored by Git so the GitHub repository remains lightweight.

See the electrical draft in `docs/` before wiring or powering any module.
