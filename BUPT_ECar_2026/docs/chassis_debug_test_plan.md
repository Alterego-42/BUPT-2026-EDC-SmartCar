# 北邮 2026 电赛小车底盘调试测试文档

适用程序：

```text
D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507
```

本目录下还建立了 `D:\BUPT_ECar_2026\mspm0-sdk`，它是指向原本地 SDK 的 junction，用来绕开中文路径导致的 SysConfig/Makefile 问题。

本轮只调底盘循迹相关硬件：电机、TB6612、后轮编码器、八路灰度、IMU、按键、LED、蜂鸣器。PCA9685、K230、激光不接入程序，也不参与测试。

## 1. 上电前检查

断电测量：

```text
3V3-GND 不短路
V5L-GND 不短路
VBAT-GND 不短路
V5L 与 V5S 不导通
V5L 与 3V3 不直接导通
```

接线确认：

```text
PWM_L PA12 -> 两块 TB6612 PWMA
PWM_R PA13 -> 两块 TB6612 PWMB
L_IN1 PA8, L_IN2 PA27 -> 两块 TB6612 AIN1/AIN2
R_IN1 PB0, R_IN2 PB6 -> 两块 TB6612 BIN1/BIN2
STBY PB18 -> 两块 TB6612 STBY
START_KEY PB2 -> 按键 -> GND
```

驱动板可以按实际安装叫“前板/后板”或“左板/右板”，但程序只认左右轮组：

```text
左轮组控制网络 = PA12/PWM_L + PA8/L_IN1 + PA27/L_IN2 -> 两块 TB6612 的 A 通道
右轮组控制网络 = PA13/PWM_R + PB0/R_IN1 + PB6/R_IN2 -> 两块 TB6612 的 B 通道
```

如果你的“左驱动板”实际接两个后轮，“右驱动板”实际接两个前轮，也没问题，但必须满足：

```text
后轮板 AOUT -> LR 左后，BOUT -> RR 右后
前轮板 AOUT -> LF 左前，BOUT -> RF 右前
两块板 A 通道都接左轮组控制网络
两块板 B 通道都接右轮组控制网络
```

模式 0 里“左侧阶段”应该转 `LF + LR`，不是转某一整块驱动板；“右侧阶段”应该转 `RF + RR`。如果左侧阶段转了两个后轮，说明控制线按板子接错了，需要把 A/B 通道重新分到左右轮组。

首次调试建议四轮悬空，急停可随手触达。

## 2. 烧录与启动现象

编译输出：

```text
D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\ticlang\ecar_debug_mspm0g3507.out
```

上电后默认进入模式 0，蜂鸣器响 1 次。空闲时红/绿灯会慢闪。

按键规则：

```text
空闲短按：运行当前模式
空闲长按：切换下一个模式
运行短按：立即停止
运行长按：立即停止
```

## 3. 模式 0：电机点动

目的：先确认 TB6612、PWM、方向脚、左右分组。

步骤：

```text
1. 确保四轮悬空。
2. 上电后短按启动模式 0。
3. 程序循环执行：左侧低速转、停、右侧低速转、停、左右一起转。
```

合格现象：

```text
左侧阶段：LF/LR 同向转，RF/RR 不转
右侧阶段：RF/RR 同向转，LF/LR 不转
左右一起阶段：四轮都朝前进方向转
```

异常处理：

```text
左侧阶段只有前轮转：后轮板 A 通道没有接到左轮组网络，重点查后轮板 PWMA/AIN1/AIN2/STBY/VM/GND
右侧阶段只有前轮转：后轮板 B 通道没有接到右轮组网络，重点查后轮板 PWMB/BIN1/BIN2/STBY/VM/GND
第一阶段转两个前轮或两个后轮：A/B 控制线按“板子”接错了，必须改为两块板 A=左轮、两块板 B=右轮
同侧前后方向相反：优先交换该电机输出线
整侧方向反：改 src/board_config.h 的 BOARD_MOTOR_LEFT_INVERT 或 BOARD_MOTOR_RIGHT_INVERT
某侧不转：查 PWM、IN1/IN2、STBY、TB6612 VM/VCC/GND
```

## 4. 模式 1：编码器检查

目的：确认 LR/RR 后轮编码器 A/B 进 M0。

步骤：

```text
1. 长按切到模式 1，蜂鸣 2 次。
2. 短按启动。
3. 程序先转左侧，再转右侧。
```

合格现象：

```text
左侧点动结束后绿灯亮并短鸣
右侧点动结束后绿灯亮并稍长短鸣
```

异常处理：

```text
红灯或长鸣：对应侧编码器计数不足，检查 A/B/GND/电源
里程方向后续反向：改 BOARD_ENCODER_LEFT_INVERT 或 BOARD_ENCODER_RIGHT_INVERT
编码器高电平若为 5V：先加电平转换再接 M0
```

## 5. 模式 2：灰度监看

目的：确认 AD0/AD1/AD2 选通和 OUT 黑白极性。

步骤：

```text
1. 长按切到模式 2，蜂鸣 3 次。
2. 短按启动。
3. 手拿灰度模块在白底和黑线之间移动。
```

合格现象：

```text
看到黑线：绿灯亮
偏离中心较大：红灯也亮
完全看不到黑线：红灯亮、绿灯灭
```

异常处理：

```text
白底也一直显示看到线：翻转 BOARD_GRAY_BLACK_IS_HIGH
线在左边但车后续往反方向修：翻转 BOARD_GRAY_REVERSE_ORDER 或 BOARD_STEERING_INVERT
完全无变化：检查灰度 5V/GND/AD0/AD1/AD2/OUT
```

## 6. 模式 3：低速循迹

目的：完成无关键点的一段低速闭环。

步骤：

```text
1. 把车放在 A 点或直线黑线上，车头正对前进方向。
2. 长按切到模式 3，蜂鸣 4 次。
3. 短按启动，程序先等待 1 秒再前进。
4. 手扶急停，先跑 30-80 cm。
```

调参顺序：

```text
1. BOARD_GRAY_BLACK_IS_HIGH
2. BOARD_MOTOR_START_PERCENT
3. BOARD_STEERING_INVERT
4. BOARD_LINE_KP_PER_MILLE
5. BOARD_LINE_KD_PER_MILLE
```

现象判断：

```text
车向远离黑线方向修：改 BOARD_STEERING_INVERT
直线一直左偏一点：增大 BOARD_STRAIGHT_TRIM_PERCENT
直线一直右偏一点：减小 BOARD_STRAIGHT_TRIM_PERCENT，必要时改成负数
直线左右摆动大：降低 KP 或提高 KD
弯道冲出去：提高 KP 或降低速度
无黑线太久红灯报警停车：检查灰度高度、阈值、电源
```

模式 3 用严格丢线保护，适合调灰度和低速闭环。模式 4/5 面向正式赛道，允许通过无黑线空白段：

```text
BOARD_LINE_BRIDGE_MAX_MS = 6000U
BOARD_LINE_BRIDGE_MAX_MM = 1400
BOARD_LINE_BRIDGE_KEEP_LAST_MS = 350U
BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT = 18
BOARD_HEADING_KP_PER_DEG = 4
BOARD_BRIDGE_HEADING_OFFSET_X10 = +222
```

在空白段内，程序会用 IMU 航向保持或直行桥接；超过上述时间/距离仍找不到黑线才报警停车。
刚从黑弯出到白段时，程序会先保留上一帧转向最多 350 ms，同时叠加 IMU 航向修正，避免出弯瞬间变成纯直行。
`BOARD_BRIDGE_HEADING_OFFSET_X10` 用于补偿弧线出线后的白区直线方向，单位是 0.1 度；`+222` 表示 +22.2 度。本版按“出弯偏外 22.17 度，需要往里调”先取正号；如果实车更偏外或反向偏内，就把符号反过来。

## 7. 模式 4：一圈关键点提示

目的：对应题目“自动巡迹一圈并回到 A 点停车，有声光提示”。

关键点判定：

```text
当前关键点由稳定黑白切换触发：
B: 白 -> 黑
C: 黑 -> 白
D: 白 -> 黑
A: 黑 -> 白

编码器里程只作为防抖门限，防止灰度抖动太早误判。

当前按赛道尺寸先给了门限初值：
直道 1000 mm，半圆弯半径 400 mm，半圆弧长约 1257 mm。

BOARD_KEYPOINT_B_MM = 1000
BOARD_KEYPOINT_C_MM = 2257
BOARD_KEYPOINT_D_MM = 3257
BOARD_KEYPOINT_A_MM = 4513
BOARD_KEYPOINT_GATE_BEFORE_MM = 350
BOARD_KEYPOINT_TRANSITION_STABLE_MS = 60U

实车标定时：
1. 看 B/C/D/A 是否在真正黑白边界处提示。
2. 如果提示太早或误触发，减小 BOARD_KEYPOINT_GATE_BEFORE_MM 或提高稳定时间。
3. 如果提示太晚，增大 BOARD_KEYPOINT_GATE_BEFORE_MM，或减小对应 BOARD_KEYPOINT_*_MM。
4. 微调 src/board_config.h:
   BOARD_KEYPOINT_B_MM
   BOARD_KEYPOINT_C_MM
   BOARD_KEYPOINT_D_MM
   BOARD_KEYPOINT_A_MM
```

运行：

```text
1. 长按切到模式 4，蜂鸣 5 次。
2. 短按启动。
3. 运行中绿灯常亮，不跟随循迹矫正闪烁。
4. 到 B/C/D/A 时分别声光提示，到 A 后停车长鸣。
```

注意：关键点本体是黑白切换，编码器只用来做“还没到这个区域就不准触发”的门限。赛道、电池、轮胎、速度变化后，如果边界提示明显提前/滞后，需要重新微调门限。

## 8. 模式 5：B 点停车联动占位

目的：先完成题目“巡迹到 B 点停车，停车状态完成作业，随后继续到 C/D/A”的底盘部分。

当前程序行为：

```text
1. 从 A 点循迹。
2. 检测到 B 点白->黑切换后停车 BOARD_KEYPOINT_B_STOP_MS。
3. 停车期间声光提示，作为后续云台/机械臂作业占位。
4. 自动继续循迹到 C/D/A，经过每个关键点提示。
5. 回 A 停车。
```

后续接入云台时，把 B 点停车期间替换为 PCA9685/机械臂动作即可；本程序故意不驱动 PCA、K230、激光。

## 9. 测试记录表

| 日期 | 电池电压 | 模式 | 现象 | 参数改动 | 结论 |
| --- | --- | --- | --- | --- | --- |
| | | 0 电机 | | | |
| | | 1 编码器 | | | |
| | | 2 灰度 | | | |
| | | 3 循迹 | | | |
| | | 4 一圈 | | | |
| | | 5 B 停 | | | |

## 10. 常用参数位置

```text
D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\src\board_config.h
```

最常改：

```text
BOARD_MOTOR_LEFT_INVERT
BOARD_MOTOR_RIGHT_INVERT
BOARD_GRAY_BLACK_IS_HIGH
BOARD_GRAY_REVERSE_ORDER
BOARD_STEERING_INVERT
BOARD_MOTOR_START_PERCENT
BOARD_MOTOR_MAX_PERCENT
BOARD_STRAIGHT_TRIM_PERCENT
BOARD_LINE_KP_PER_MILLE
BOARD_LINE_KD_PER_MILLE
BOARD_LINE_BRIDGE_MAX_MM
BOARD_LINE_BRIDGE_KEEP_LAST_MS
BOARD_HEADING_KP_PER_DEG
BOARD_BRIDGE_HEADING_OFFSET_X10
BOARD_KEYPOINT_*_MM
```
