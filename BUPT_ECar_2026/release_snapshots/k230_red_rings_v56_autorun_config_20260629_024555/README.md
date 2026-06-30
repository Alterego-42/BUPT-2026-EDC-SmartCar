# K230 + STM32 激光动态瞄准交付包

打包日期：2026-06-27

本包用于小车运动场景下的视觉闭环激光瞄准：K230 识别白纸内的黑色中心靶点和红色激光点，计算两者偏差角度，通过 UART 输出给 STM32；STM32 驱动 PCA9685 + 双轴步进云台，使红点快速追向中心靶点。

## 当前比赛版备注（2026-06-29）

当前 BUPT_ECar_2026 已切到 MSPM0G3507 主控，K230 可脱离电脑持续运行：

```text
/sdcard/main.py                  K230 上电自启动入口
/sdcard/k230_target_uart.py      红色同心圆靶面识别和 UART 输出主程序
/sdcard/k230_target_config.py    现场可调参数“头文件”
```

K230 上电后会一直输出 `$K230,valid,yaw,pitch` 到 MSPM0。MSP0 侧只在对应模式启动云台闭环时响应 K230；其他模式下可以继续接收但不驱动云台。

当前封存版本：

```text
SCRIPT_VERSION = red_rings_v56_autorun_config
CONTROL_SEND_HZ = 45
FIXED_LASER_CX = 195.0
FIXED_LASER_CY = 93.0
```

调参优先改：

```text
k230/k230_target_config.py
```

然后把 `main.py`、`k230_target_uart.py`、`k230_target_config.py` 三件套复制到 CanMV 设备的 `sdcard` 根目录。Windows 资源管理器中 CanMV 通常显示为“便携式设备”，不是普通盘符；复制后可断开电脑，只保留 K230 独立供电。

如果需要脚本上传/调试：

```powershell
cd E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\k230相关\target_tracking_delivery_20260627
.\k230\k230_connected.ps1 -Port COM7 -Upload
.\k230\k230_connected.ps1 -Port COM7 -Run -Debug -Seconds 2
```

当前可回滚快照：

```text
E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\release_snapshots\k230_red_rings_v56_autorun_config_20260629_024555
```

## 目录结构

```text
target_tracking_delivery_20260627/
  README.md
  k230/
    k230_target_uart.py              # K230 主程序，当前 v12 快速版
    tools/
      k230_capture_frame.py          # K230 截图工具
      k230_probe_dark_blobs.py       # 黑色目标/黑点调试工具
  stm32/
    project/
      stm32f103c8_pca9685_k230_laser_test/
        Inc/
        Src/
        Drivers/
        MDK-ARM/test_in_keil.uvprojx # Keil 工程
    prebuilt/
      test_in_keil_v12_fast.hex      # 已编译固件
    logs/
      keil_build_fast_20260627.log
      flash_v12_fast.log
```

## 硬件连接

K230 UART2 到 STM32：

```text
K230 IO5 / UART2_TXD -> STM32 USART1_RX
K230 IO6 / UART2_RXD -> STM32 USART1_TX
GND                  -> GND
Baud                 -> 115200
```

STM32 侧：

```text
USART1      接收 K230 数据
PCA9685     I2C 地址 0x40
CH0~CH3     yaw / 水平方向电机
CH4~CH7     pitch / 垂直方向电机
PB0         激光开关，默认常亮
```

## 目标物要求

当前视觉策略面向以下靶面：

```text
白纸区域
粗黑色方框
黑色中心点
红色激光点
```

K230 会优先用白纸作为 ROI，目标点和红点都限制在白纸范围内。红点可以超出黑框，只要还在白纸里就继续追踪。

## K230 程序

主程序：

```text
k230/k230_target_uart.py
```

关键参数：

```text
SCRIPT_VERSION        = center_dot_laser_v12_fast_paper_roi
CONTROL_SEND_HZ       = 50
CONTROL_MAX_DEG       = 3.00
CONTROL_GAIN          = 0.75
ERROR_DEADBAND_PX     = 2.0
USB_DEBUG_ENABLED     = False
STM32_RX_DEBUG        = False
```

UART 输出格式固定为：

```text
$K230,valid,yaw,pitch
```

示例：

```text
$K230,1,2.35,-0.80
$K230,0,0.00,0.00
```

`valid=1` 表示检测到目标点和红点；`valid=0` 表示当前丢失或数据不稳定。`yaw/pitch` 单位为度，已经在 K230 侧完成当前云台轴映射：

```text
motor_yaw   = -image_pitch
motor_pitch = -image_yaw
```

## K230 运行方式

临时运行：

```powershell
python -m mpremote connect COM7 run .\k230\k230_target_uart.py
```

推荐写入 K230 SD 卡后运行：

```powershell
python -m mpremote connect COM7 fs cp .\k230\k230_target_uart.py :/sdcard/k230_target_uart.py
python -m mpremote connect COM7 exec "exec(open('/sdcard/k230_target_uart.py').read())"
```

断开电脑前建议确认：

```text
USB_DEBUG_ENABLED = False
```

这样 USB 不会持续打印调试信息，只保留 UART2 给 STM32 输出控制包。断开电脑后，K230 必须保持独立供电；如果拔线导致 K230 掉电，程序会停止，需要另做上电自启。

## STM32 程序

Keil 工程：

```text
stm32/project/stm32f103c8_pca9685_k230_laser_test/MDK-ARM/test_in_keil.uvprojx
```

主逻辑：

```text
stm32/project/stm32f103c8_pca9685_k230_laser_test/Src/main.c
```

已编译固件：

```text
stm32/prebuilt/test_in_keil_v12_fast.hex
```

当前 STM32 关键参数：

```text
UART_BAUDRATE          = 115200
VISION_TIMEOUT_MS      = 120
YAW_DEADBAND_CDEG      = 6
PITCH_DEADBAND_CDEG    = 6
H_STEP_MIN_MS          = 1
V_STEP_MIN_MS          = 1
H_STEPS_PER_FRAME      = 24
V_STEPS_PER_FRAME      = 24
AXIS_AUTO_LEARN_ENABLED= 0
UART_DEBUG_ENABLED     = 0
LASER_DEFAULT_ON       = 1
```

这版已经关闭学习逻辑和 STM32 UART 调试回传，STM32 不会主动向 K230 串口刷调试信息。

## STM32 编译和下载

用 Keil 打开：

```text
stm32/project/stm32f103c8_pca9685_k230_laser_test/MDK-ARM/test_in_keil.uvprojx
```

然后执行 Build 和 Download。

也可以使用包内 hex：

```text
stm32/prebuilt/test_in_keil_v12_fast.hex
```

最后一次下载日志：

```text
Erase Done. Programming Done. Verify OK.
```

## 速度设计依据

实际路径为两个半径 40cm 的半圆加中间 100cm 直线，约 16 秒一圈。

```text
周长 = 2 * 100 + 2 * pi * 40 = 451.3 cm
车速 = 451.3 / 16 = 28.2 cm/s
直道最近点视线角速度约 32.3 deg/s
半圆车身转动角速度约 40.4 deg/s
叠加后云台相对车身最坏约 56.6 deg/s
```

因此当前版本按 60 deg/s 以上的响应余量调参：K230 50Hz 输出，STM32 每个视觉帧最多 24 步，K230 单包控制限幅 3 度。

## 常见问题

### K230 报 snapshot chn(2) failed(3)

这是 K230 相机通道反复启动后偶发卡住。通常复位 K230 即可：

```powershell
python -m mpremote connect COM7 exec "import machine; machine.reset()"
```

复位后重新运行主程序。

### 红点在白纸内误跳

检查 K230 参数：

```text
LASER_HINT_MAX_DIST_PX      = 70
LASER_REACQUIRE_MAX_DIST_PX = 180
MAX_LASER_JUMP_PX          = 80
```

正常跟踪时红点只允许在上一帧附近跟踪；完全丢失后才在更大范围重捕获。

### 运动方向反了

优先检查 K230 的轴映射：

```python
motor_yaw = -image_pitch
motor_pitch = -image_yaw
```

如果只某一个电机方向反，检查 STM32：

```c
#define YAW_MOTOR_DIR   1
#define PITCH_MOTOR_DIR 1
```

### 调试时需要看完整日志

把 K230 里：

```python
USB_DEBUG_ENABLED = True
```

然后用 mpremote 运行即可看到 `$DBG`、`$TX` 等调试行。正式运动时建议保持 `False`。

## 当前状态备注

本包保存的是当前可用快速版。它已经在实际靶距下验证过能够检测：

```text
paper=1
target=dot
laser=red
valid=1
```

动态效果还需要按现场速度、光照、靶面尺寸继续微调 `CONTROL_GAIN`、`CONTROL_MAX_DEG`、红点阈值和 STM32 步进参数。
