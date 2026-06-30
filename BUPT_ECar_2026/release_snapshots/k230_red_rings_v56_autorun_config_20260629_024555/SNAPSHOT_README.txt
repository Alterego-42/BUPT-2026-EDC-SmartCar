K230 red-rings aiming snapshot: v56_autorun_config
Created: 20260629_024555

Purpose:
- K230 can boot from /sdcard/main.py and run target tracking continuously without a PC.
- MSPM0 keeps deciding by mode whether to consume K230 UART packets.
- Tunable K230 parameters live in k230/k230_target_config.py.
- MSPM0 gimbal parameters live in firmware/.../src/board_config.h.

Deploy:
cd E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\k230相关\target_tracking_delivery_20260627
.\k230\k230_connected.ps1 -Port COM7 -Upload

Files:
- k230/main.py -> /sdcard/main.py
- k230/k230_target_uart.py -> /sdcard/k230_target_uart.py
- k230/k230_target_config.py -> /sdcard/k230_target_config.py
