# BUPT E-Car 2026 Chassis Debug Firmware

Target board: `TI LP-MSPM0G3507`.

This firmware is for chassis bring-up only. It does not initialize or drive PCA9685, K230, or laser hardware.

## Directory

```text
ecar_debug_mspm0g3507/
  ecar_debug_mspm0g3507.syscfg
  src/
  ticlang/
```

## Build

An ASCII SDK junction is created at:

```text
D:\BUPT_ECar_2026\mspm0-sdk
```

It points to the local SDK under the original desktop workspace. The Makefile uses the junction to avoid Chinese path issues.

Build from PowerShell:

```powershell
cd D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\ticlang
& D:\Dev\ECar2026\Tools\ti\ccs-21.0.0\ccs\utils\bin\gmake.exe -f makefile
```

Output:

```text
D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\ticlang\ecar_debug_mspm0g3507.out
```

## CCS Import

Use CCS project-spec import:

```text
D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\ticlang\ecar_debug_mspm0g3507_LP_MSPM0G3507_nortos_ticlang.projectspec
```

If the normal "Import projects" dialog shows no discovered projects, use:

```text
File -> Import -> Code Composer Studio -> CCS Projects from ProjectSpec
```

or choose the `ticlang` folder when the importer asks for a search directory.

If CCS reports `Product MSPM0-SDK v0.0 is not currently installed`, open:

```text
Window -> Preferences -> Code Composer Studio -> Products
```

Add or rediscover:

```text
D:\BUPT_ECar_2026\mspm0-sdk
```

Then restart CCS. The bundled projectspec also uses absolute ASCII SDK paths, so it can import on machines where the SDK product registry is not set yet.

Important settings if you create the project manually:

```text
Device: MSPM0G3507
Toolchain: TI Arm Clang
SysConfig file: ecar_debug_mspm0g3507.syscfg
Linker command: ticlang/device_linker.cmd
Include paths: src, project root, SDK source, SDK CMSIS
```

The command-line Makefile is already verified and is the preferred first build path.

## Button Menu

The only external button is `START_KEY/PB2`, wired to GND and using the internal pull-up.

In idle:

```text
Short press: start current mode
Long press: switch to next mode
```

While running:

```text
Short press or long press: stop and return to idle
```

Mode number is confirmed by green LED and buzzer pulses after switching. Mode 0 beeps once, mode 1 twice, and so on.

## Modes

```text
0 Motor step
1 Encoder check
2 Grayscale monitor
3 Low-speed line follow
4 One-lap keypoint run
5 B-stop linkage placeholder
```

Tune the first-run values in:

```text
src/board_config.h
```

Start with `BOARD_MOTOR_TEST_PERCENT` and `BOARD_MOTOR_START_PERCENT` low until motor direction and gray polarity are confirmed.
