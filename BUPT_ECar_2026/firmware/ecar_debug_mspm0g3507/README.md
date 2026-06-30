# BUPT E-Car 2026 MSPM0 Firmware

Target board: `TI LP-MSPM0G3507`.

This firmware now contains the current task modes for line following,
sound/light keypoint signaling, K230 UART parsing, laser control, PCA9685
stepper gimbal control, route feed-forward, and fixed-point target shooting.

For the detailed handoff and current mode logic, read:

```text
../../docs/mode_handoff_2026-06-30.md
```

## Build And Flash

From PowerShell:

```powershell
cd E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507
.\flash_connected.cmd
```

`flash_connected.cmd` invokes `flash_connected.ps1`, builds the `ticlang`
project with TI Arm Clang, then flashes the generated
`ticlang/ecar_debug_mspm0g3507.out` through XDS110.

Useful options:

```powershell
.\flash_connected.ps1 -Clean
.\flash_connected.ps1 -NoFlash
.\flash_connected.ps1 -NoBuild
.\flash_connected.ps1 -SkipProbeCheck
.\flash_connected.ps1 -CcsRoot <path> -SdkRoot <path>
```

If flashing fails at probe detection, reconnect the MSPM0/XDS110 debug probe
and rerun the script. Seeing only the K230 COM port in Windows is not enough
for MSPM0 flashing.

## Button Menu

The external button is `START_KEY/PB2`, wired to GND with internal pull-up.

In idle:

```text
Short press: start current mode
Long press : switch to next mode
```

While running:

```text
Short press or long press: stop and return to idle
```

The source enum starts at 0, but the car-side/user-facing mode number is
normally discussed as enum + 1.

## Current Modes

```text
Mode 1: one-lap A->A line run with B/C/D/A sound-light prompts, auto stop
Mode 2: fixed-point target shooting, scan + aim, repeat 3 cycles
Mode 3: line following + gimbal route feed-forward + K230 trim
Mode 4: plain line following only
Mode 5: continuous keypoint line run with sound-light prompts, no auto stop
Mode 6: K230/gimbal/laser online test
Mode 7: gimbal sweep hardware test
```

Main tunables are in:

```text
src/board_config.h
```
