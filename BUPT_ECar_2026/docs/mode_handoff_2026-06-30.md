# BUPT_ECar_2026 模式与调参交接文档

更新时间：2026-06-30  
适用目录：`BUPT_ECar_2026`  
主控工程：`firmware/ecar_debug_mspm0g3507`  
K230 工程：`k230相关/target_tracking_delivery_20260627`

这份文档以当前源码为准，重点给后续接手同学说明模式 1-6 的运行逻辑、状态机、关键参数和现场调试入口。仓库里早期的 `board_config_parameter_guide.md`、固件旧 README 曾对应底盘调试早期版本，模式编号和部分参数已经过期；后续请优先按本文和当前源码确认。

## 1. 当前硬件与连线

### 1.1 主控与外设

- 主控：TI MSPM0G3507，工程位于 `firmware/ecar_debug_mspm0g3507`。
- K230：运行 CanMV/MicroPython 脚本，负责视觉识别靶面并通过 UART 输出云台修正量。
- 云台：两个步进电机，经 PCA9685 + ULN 驱动。
- 激光：接 MSPM0 的 PB15，`BOARD_LASER_ACTIVE_HIGH = 1`，高电平打开。
- 灰度：八路数字灰度，当前按“黑色为高电平”解释。
- 按键：`START_KEY/PB2`，上拉输入，按下接地。

### 1.2 K230 与 MSPM0 串口

MSP0 侧代码使用 `UART1`：

- PA17 TX -> K230 RX
- PA18 RX -> K230 TX
- 波特率：115200

K230 侧脚本使用 UART2：

- K230 物理 pin 17 = IO5 = UART2_TXD
- K230 物理 pin 20 = IO6 = UART2_RXD
- 发送格式：`$K230,valid,yaw,pitch,target_ok\n`

其中：

- `valid`：本帧是否允许 MSPM0 云台执行 yaw/pitch 修正。
- `yaw`、`pitch`：单位是度，MSP0 解析成 centi-degree，即 0.01 度。
- `target_ok`：模式 2 搜索靶面专用确认位。旧的三字段包也能兼容，MSP0 会把 `target_ok` 默认成 `valid`。

## 2. 按键与总运行框架

### 2.1 模式切换

源码中的枚举从 0 开始，但车上/口头沟通通常按 1 开始数：

| 车上模式 | 源码枚举 | 入口函数 | 主要用途 |
| --- | --- | --- | --- |
| 模式 1 | `MODE_ONE_LAP_RUN` | `task_one_lap_run()` | A 点出发跑一圈，B/C/D/A 声光提示，回 A 自动停车 |
| 模式 2 | `MODE_TARGET_POINT_RUN` | `task_target_point_run()` | 定点打靶：扫靶、记录发现姿态、精确对靶，重复 3 次 |
| 模式 3 | `MODE_LINE_GIMBAL_RUN` | `task_line_gimbal_run()` | 循迹 + 云台路线前馈 + K230 视觉微调 |
| 模式 4 | `MODE_LINE_FOLLOW` | `task_line_follow_only()` | 纯循迹，无关键点声光，无白段桥接 |
| 模式 5 | `MODE_KEYPOINT_RUN` | `handle_keypoints(false, true)` | 持续循迹，B/C/D/A 声光提示，跑完一圈不停车 |
| 模式 6 | `MODE_GIMBAL_TEST` | `task_gimbal_test()` + `gimbal_task()` | K230/云台/激光在线测试 |

代码里还有模式 7：`MODE_GIMBAL_SWEEP_TEST`，用于两轴云台扫动硬件测试，本文不按比赛任务展开。

按键逻辑：

- 空闲时短按：启动当前模式。
- 空闲时长按：切换到下一个模式。
- 运行中短按或长按：停止并回到空闲。

### 2.2 `main.c` 的主循环

初始化顺序：

1. `SYSCFG_DL_init()`
2. `timebase_init()`
3. `ui_init()`
4. `sound_light_init()`
5. `motor_init()`
6. `grayscale_init()`
7. `encoder_init()`
8. `imu_init()`
9. `k230_uart_init()`
10. `gimbal_init()`

循环中每轮都会运行：

- `ui_task()`
- `sound_light_task()`
- `k230_uart_task()`
- 模式 6 运行中额外先调用一次 `gimbal_task()`
- 读取按键事件
- 运行当前模式的 `active_task()`
- 空闲时闪灯显示当前模式

`start_mode()` 会做这些事情：

- 编码器清零。
- 关键点、循迹、云台路线状态清零。
- 停电机。
- 如果当前模式需要云台，则 `gimbal_start()`，打开激光并初始化 PCA9685。
- 如果是模式 2，则 `target_mode_start()`。
- 进入 `RUN_ACTIVE`。

需要云台/激光的模式：

- 模式 2
- 模式 3
- 模式 6
- 模式 7

模式 1、4、5 不启动云台和激光。

## 3. 声光提示模块

文件：

- `src/sound_light.c`
- `src/sound_light.h`

声光模块是非阻塞的，靠主循环里的 `sound_light_task()` 更新，不会因为蜂鸣/亮灯停住循迹控制。

当前行为：

- `sound_light_keypoint(point_index)`：关键点提示，持续 `BOARD_KEYPOINT_BEEP_MS`，当前 180 ms。
- `sound_light_stop()`：停止提示，常亮/响 1000 ms。
- `sound_light_complete()`：完成提示，总时长 3000 ms，250 ms 亮、250 ms 灭。
- `sound_light_error()`：错误提示，总时长 600 ms，100 ms 闪烁。
- `sound_light_aiming_ok()`：模式 2 找到靶、进入对靶时短促提示。

注意：`point_index` 当前没有区分不同音型，B/C/D/A 都是同样的短提示。如果后续需要区分四个点，可在 `sound_light_keypoint()` 里按 `point_index` 改不同节奏。

## 4. 模式 1：A 到 A 一圈，声光提示，自动停车

源码：

- 枚举：`MODE_ONE_LAP_RUN`
- 入口：`task_one_lap_run()`
- 实际调用：`handle_keypoints(false, false)`

### 4.1 运行流程

1. 按键启动后等待 `BOARD_LINE_START_DELAY_MS`，当前 1000 ms。
2. 进入带白段桥接的循迹：`line_follow_update_ex(..., true)`。
3. 灰度状态发生稳定黑/白切换时，按顺序识别 B/C/D/A。
4. 每识别一个点，调用 `sound_light_keypoint(point_index)`。
5. 第 4 个点，也就是回到 A 后，调用 `finish_run()`：
   - 停车。
   - 关闭云台路线、云台、循迹输出。
   - 调用 `sound_light_stop()`。
   - 状态变为 `RUN_FINISHED`。

### 4.2 关键点判定

当前关键点完全按灰度“无黑线/有黑线”的稳定切换判断：

- `black_count == 0`：认为当前在白色区域。
- `black_count > 0`：认为当前在黑色区域。

期望序列：

- B：白 -> 黑
- C：黑 -> 白
- D：白 -> 黑
- A：黑 -> 白

稳定条件：

- 新状态持续 `BOARD_KEYPOINT_TRANSITION_STABLE_MS`，当前 60 ms。
- 两次有效提示间隔至少 `BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS`，当前 500 ms。

重要注意：`board_config.h` 中保留了 `BOARD_KEYPOINT_B_MM/C_MM/D_MM/A_MM` 和 gate 参数，但当前 `keypoint_transition_detected()` 里 `distance_mm` 被 `(void)distance_mm;` 忽略，也就是里程门控暂未启用。现在的稳定效果来自黑白切换本身和稳定时间/冷却时间。

### 4.3 可调参数

主要在 `src/board_config.h`：

- `BOARD_LINE_START_DELAY_MS`：启动延时。
- `BOARD_MOTOR_START_PERCENT`：基础速度。
- `BOARD_MOTOR_MAX_PERCENT`：电机输出限幅。
- `BOARD_STRAIGHT_TRIM_PERCENT`：直行左右偏修正。
- `BOARD_LINE_KP_PER_MILLE`、`BOARD_LINE_KD_PER_MILLE`：循迹 PD 参数。
- `BOARD_LINE_BRIDGE_MAX_MS`、`BOARD_LINE_BRIDGE_MAX_MM`：白段桥接最长时间/距离。
- `BOARD_LINE_BRIDGE_KEEP_LAST_MS`、`BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT`：刚出弯进入白段时保留上一帧转向。
- `BOARD_IMU_ENABLE_HEADING_HOLD`、`BOARD_HEADING_KP_PER_DEG`、`BOARD_BRIDGE_HEADING_OFFSET_X10`：白段 IMU 航向保持。
- `BOARD_KEYPOINT_TRANSITION_STABLE_MS`：黑白切换稳定时间。
- `BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS`：关键点提示冷却。
- `BOARD_KEYPOINT_BEEP_MS`：关键点声光时长。

## 5. 模式 2：定点打靶，扫靶 + 精确对靶，重复 3 次

源码：

- 枚举：`MODE_TARGET_POINT_RUN`
- 入口：`task_target_point_run()`
- 状态枚举：`TARGET_PHASE_*`

### 5.1 任务目标

设计目标是：

1. 初始让云台水平轴处于机械基准 0 度。
2. 小车被放在随机点位后，按键启动。
3. 云台先逆时针扫 180 度，再顺时针扫 360 度。
4. 发现靶面后，记录当前相对初始基准的云台 yaw/pitch。
5. 进入精确对靶，步幅较小。
6. 一次对靶完成后，等待丢靶，也就是人把车拿起来。
7. 丢靶后等待约 7 秒，让人把车放到下一个点位。
8. 云台回到 0 度，再重复搜索。
9. 共完成 3 次后结束。

模式 2 中底盘电机始终 `motor_stop()`，只动云台。

### 5.2 状态机

`target_mode_start()`：

- `gimbal_zero_pose()`：把当前云台位置当作 0。
- `g_target_cycle = 0`。
- 进入 `TARGET_PHASE_SCAN_CCW`。

`TARGET_PHASE_SCAN_CCW`：

- 逆时针搜索。
- 目标角由 `BOARD_TARGET_MODE_CCW_SIGN * BOARD_TARGET_MODE_SEARCH_SOFT_CDEG` 给出。
- 当前 `BOARD_TARGET_MODE_CCW_SIGN = -1`。
- 软目标是 `BOARD_GIMBAL_POSITION_LIMIT_CDEG`，当前 20000，即故意给一个足够远的角度。
- 真正停止半圈搜索靠硬步数限制：`BOARD_TARGET_MODE_SEARCH_HALF_STEPS = 2048L`。
- 实测约 `1024 updates = 90 度`，所以 2048 对应约 180 度。

`TARGET_PHASE_SCAN_CW`：

- 顺时针搜索。
- 硬步数限制：`BOARD_TARGET_MODE_SEARCH_FULL_STEPS = 4096L`，对应约 360 度。
- 如果仍未找到，先回零，再进入 `TARGET_PHASE_SEARCH_RETRY`，短暂等待后重新扫。

搜索阶段发现靶面必须满足：

- K230 帧是 fresh，未超过 `BOARD_K230_VISION_TIMEOUT_MS`，当前 300 ms。
- `frame.valid == true`。
- `frame.target_ok == true`。
- 至少 `BOARD_TARGET_MODE_VALID_MIN_FRAMES` 个新帧，当前 4 帧。
- 持续至少 `BOARD_TARGET_MODE_VALID_STABLE_MS`，当前 160 ms。
- 候选短时丢失会保留 `BOARD_TARGET_MODE_CANDIDATE_HOLD_MS`，当前 260 ms。

确认找到后：

- `target_mode_store_found_pose()` 保存当前云台姿态到 `g_target_found_yaw_cdeg[]`、`g_target_found_pitch_cdeg[]`。
- `sound_light_aiming_ok()` 给短提示。
- 进入 `TARGET_PHASE_AIM`。

`TARGET_PHASE_AIM`：

- 使用 K230 的 yaw/pitch 误差继续精确对靶。
- 单次命令限幅：`BOARD_TARGET_MODE_AIM_MAX_CDEG = 8`，即 0.08 度。
- 死区：`BOARD_TARGET_MODE_AIM_DEADBAND_CDEG = 1`，即 0.01 度。
- 固定步进间隔：`BOARD_TARGET_MODE_AIM_STEP_MS = 12`。
- 至少对靶 `BOARD_TARGET_MODE_AIM_MIN_MS = 900 ms` 后，才允许判完成。
- 误差进入 `BOARD_TARGET_MODE_AIM_STABLE_CDEG = 2`，并保持 `BOARD_TARGET_MODE_AIM_STABLE_MS = 700 ms`，认为本次完成。

完成一次后：

- `g_target_cycle++`。
- 如果已完成 `BOARD_TARGET_MODE_CYCLES = 3` 次，`complete_run()`。
- 否则提示一次，进入 `TARGET_PHASE_WAIT_LOST`。

`TARGET_PHASE_WAIT_LOST`：

- 如果 K230 仍有效，继续小步对靶。
- 如果丢失超过 `BOARD_TARGET_MODE_LOST_MS = 500 ms`，认为人已拿起/移开，进入 `TARGET_PHASE_WAIT_REPLACE`。

`TARGET_PHASE_WAIT_REPLACE`：

- 等待 `BOARD_TARGET_MODE_REPLACE_WAIT_MS = 7000 ms`。
- 然后进入 `TARGET_PHASE_RETURN_ZERO`。

`TARGET_PHASE_RETURN_ZERO`：

- 云台回到 0,0。
- 回零步进间隔 `BOARD_TARGET_MODE_RETURN_STEP_MS = 6 ms`。
- 回零容差 `BOARD_TARGET_MODE_ZERO_TOL_CDEG = 20`。
- 回零完成后再次 `gimbal_zero_pose()`，再进入下一轮搜索。

### 5.3 K230 如何告诉 MSPM0“找到靶”

K230 当前版本：`panel_center_v72_mode2_search_gate`

函数：`target_ok_for_mode2(target, valid)`

当前 `target_ok = 1` 的情况：

- `target[0] == "red_panel_center"`：识别到官方红色同心圆靶面，并能确定黑框/面板中心。
- 或 `target[0] == "red_panel_frame_center"` 且完整黑框质量足够：
  - `RED_PANEL_MODE2_FRAME_FALLBACK_ENABLED = True`
  - 黑框完整
  - `w >= RED_PANEL_MODE2_FRAME_MIN_W`，当前 42
  - `h >= RED_PANEL_MODE2_FRAME_MIN_H`，当前 52
  - `score >= RED_PANEL_MODE2_FRAME_MIN_SCORE`，当前 2400

这意味着模式 2 的“发现靶面”比模式 6 的普通跟踪更严格，目的是减少搜索时被其他黑色物体误触发停下。

### 5.4 当前现象与排查重点

如果“找到靶后看起来没对靶，只是停住”，先按以下顺序判断：

1. 看 K230 是否继续发送 `$K230,1,...,target_ok`。
2. 如果 `valid=1` 但云台不动，可能是误差已经落入 `BOARD_TARGET_MODE_AIM_DEADBAND_CDEG`，或步幅太小肉眼不明显。
3. 如果搜索刚找到就不再动，检查是否进入了 `TARGET_PHASE_AIM`，源码里确认找到后确实会进入 AIM。
4. 如果很容易误识别停下，收紧 K230 的 `target_ok_for_mode2()` 条件，不要直接放宽 MSPM0 的稳定时间。
5. 如果经过靶面但识别不到，先用 K230 `-Diagnose` 看当前 target 源是不是 `red_panel_center` 或 `red_panel_frame_center`。

### 5.5 可调参数

MSP0：`src/board_config.h`

- `BOARD_TARGET_MODE_CYCLES`：重复次数，当前 3。
- `BOARD_TARGET_MODE_CCW_SIGN`：逆时针方向符号。如果物理方向反了，改这个。
- `BOARD_TARGET_MODE_SEARCH_HALF_STEPS`：第一段 180 度搜索步数，当前 2048。
- `BOARD_TARGET_MODE_SEARCH_FULL_STEPS`：第二段 360 度搜索步数，当前 4096。
- `BOARD_TARGET_MODE_SCAN_STEP_MS`：搜索扫动速度，当前 18 ms。
- `BOARD_TARGET_MODE_RETURN_STEP_MS`：回零速度，当前 6 ms。
- `BOARD_TARGET_MODE_AIM_STEP_MS`：精对靶速度，当前 12 ms。
- `BOARD_TARGET_MODE_AIM_MAX_CDEG`：单帧精对靶命令限幅，当前 8。
- `BOARD_TARGET_MODE_AIM_DEADBAND_CDEG`：精对靶死区，当前 1。
- `BOARD_TARGET_MODE_AIM_STABLE_CDEG`：完成判定误差阈值，当前 2。
- `BOARD_TARGET_MODE_AIM_MIN_MS`：至少对靶时间，当前 900 ms。
- `BOARD_TARGET_MODE_AIM_STABLE_MS`：稳定完成时间，当前 700 ms。
- `BOARD_TARGET_MODE_VALID_STABLE_MS`：发现靶稳定时间，当前 160 ms。
- `BOARD_TARGET_MODE_VALID_MIN_FRAMES`：发现靶最小帧数，当前 4。
- `BOARD_TARGET_MODE_CANDIDATE_HOLD_MS`：短时丢帧候选保持，当前 260 ms。
- `BOARD_TARGET_MODE_LOST_MS`：丢靶判定时间，当前 500 ms。
- `BOARD_TARGET_MODE_REPLACE_WAIT_MS`：换点等待时间，当前 7000 ms。

K230：`k230/k230_target_config.py`

- `RED_PANEL_MODE2_FRAME_FALLBACK_ENABLED`
- `RED_PANEL_MODE2_FRAME_MIN_W`
- `RED_PANEL_MODE2_FRAME_MIN_H`
- `RED_PANEL_MODE2_FRAME_MIN_SCORE`
- 红环/黑框相关阈值见第 10 节。

## 6. 模式 3：循迹 + 云台路线前馈 + K230 视觉微调

源码：

- 枚举：`MODE_LINE_GIMBAL_RUN`
- 入口：`task_line_gimbal_run()`

### 6.1 运行流程

每轮：

1. 读取编码器距离：`encoder_get()`。
2. 把距离传给云台路线：`gimbal_route_set_distance(enc.distance_mm)`。
3. 执行持续关键点循迹：`handle_keypoints(false, true)`。
4. 如果仍处于 `RUN_ACTIVE`，调用 `gimbal_task()`。

这个模式会启动：

- 底盘循迹。
- 关键点声光提示。
- 激光。
- 云台。
- K230 视觉修正。
- 云台路线前馈窗口。

### 6.2 路线前馈逻辑

文件：

- `src/gimbal_route.c`
- `src/gimbal_route.h`

赛道抽象为：

- A-B：直道
- B-C：半圆弧
- C-D：直道
- D-A：半圆弧

路线节点：

- `BOARD_GIMBAL_ROUTE_A_MM = 0`
- `BOARD_GIMBAL_ROUTE_B_MM = BOARD_KEYPOINT_B_MM`
- `BOARD_GIMBAL_ROUTE_C_MM = BOARD_KEYPOINT_C_MM`
- `BOARD_GIMBAL_ROUTE_D_MM = BOARD_KEYPOINT_D_MM`
- `BOARD_GIMBAL_ROUTE_A2_MM = BOARD_KEYPOINT_A_MM`

每个节点有一个 yaw/pitch 中心角：

- `BOARD_GIMBAL_ROUTE_YAW_A_CDEG`
- `BOARD_GIMBAL_ROUTE_YAW_B_CDEG`
- `BOARD_GIMBAL_ROUTE_YAW_C_CDEG`
- `BOARD_GIMBAL_ROUTE_YAW_D_CDEG`
- `BOARD_GIMBAL_ROUTE_YAW_A2_CDEG`
- pitch 同理

当前这些中心角全部是 0，说明路线前馈还没有做实车标定。现在模式 3 可以运行，但主要还靠 K230 视觉；要真正适应“车一边循迹一边动态对靶”，需要补齐这些路线中心角。

### 6.3 前馈窗口与视觉微调

`gimbal_route_get_sample()` 会计算当前距离下的目标中心角。如果当前云台姿态在窗口内，不强制动；如果超出窗口，则给出回窗口的误差。

窗口参数：

- `BOARD_GIMBAL_ROUTE_YAW_WINDOW_CDEG = 700`，即约 ±7 度。
- `BOARD_GIMBAL_ROUTE_PITCH_WINDOW_CDEG = 450`，即约 ±4.5 度。

如果 K230 同时有有效帧：

- 视觉误差会以 `BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG = 18` 限幅叠加到路线误差上。
- 也就是说路线前馈负责让靶保持在视野里，K230 只做小范围微调。

### 6.4 可调参数

优先调：

- `BOARD_GIMBAL_ROUTE_YAW_A/B/C/D/A2_CDEG`
- `BOARD_GIMBAL_ROUTE_PITCH_A/B/C/D/A2_CDEG`
- `BOARD_GIMBAL_ROUTE_YAW_WINDOW_CDEG`
- `BOARD_GIMBAL_ROUTE_PITCH_WINDOW_CDEG`
- `BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG`

调参建议：

1. 先固定小车在 A/B/C/D/A2 对应位置，手动让靶面处于 K230 视野中心附近。
2. 记录此时云台相对模式启动 0 点的 yaw/pitch。
3. 填入各路线节点中心角。
4. 先把窗口设大，确认全程不丢靶。
5. 再逐步收窄窗口，让 K230 视觉只做小幅微调。

## 7. 模式 4：纯循迹

源码：

- 枚举：`MODE_LINE_FOLLOW`
- 入口：`task_line_follow_only()`

### 7.1 运行流程

1. 启动后等待 `BOARD_LINE_START_DELAY_MS`，当前 1000 ms。
2. 调用 `line_follow_update(millis(), enc.distance_mm)`。
3. `line_follow_update()` 内部等价于 `line_follow_update_ex(..., false)`，即不启用白段桥接。
4. 如果丢线超过 `BOARD_LINE_LOST_STOP_MS`，当前 1200 ms，则 `error_stop()`。

模式 4 不做：

- 关键点识别。
- 声光关键点提示。
- 白段桥接。
- 云台/激光/K230。

它适合单独验证灰度方向、PD 循迹和底盘基础稳定性。

### 7.2 可调参数

与模式 1/5 共用基础循迹参数：

- `BOARD_MOTOR_START_PERCENT`
- `BOARD_MOTOR_MAX_PERCENT`
- `BOARD_STRAIGHT_TRIM_PERCENT`
- `BOARD_LINE_KP_PER_MILLE`
- `BOARD_LINE_KD_PER_MILLE`
- `BOARD_LINE_CONTROL_PERIOD_MS`
- `BOARD_LINE_LOST_HOLD_MS`
- `BOARD_LINE_LOST_STOP_MS`

注意：如果要跑有大段白色间隙的正式赛道，不应使用模式 4，应使用模式 1/5 或未来完善后的模式 3。

## 8. 模式 5：持续循迹 + B/C/D/A 声光提示，不停车

源码：

- 枚举：`MODE_KEYPOINT_RUN`
- 入口：`handle_keypoints(false, true)`

### 8.1 运行流程

模式 5 和模式 1 的核心逻辑相同，区别是：

- `loop_forever = true`
- 识别到 A 后不停车，而是 `g_next_keypoint = 0`，继续下一圈。

也就是说模式 5 会持续：

1. 带白段桥接循迹。
2. 按 B/C/D/A 顺序识别黑白切换。
3. 每个点声光提示一次。
4. 到 A 后重置关键点序号，继续下一圈。

### 8.2 当前状态

这套循迹和关键点声光已经经过大量实车调参，用户确认过：

- 四点能响。
- 无明显噪声误触发。
- 模式 5 持续运行不停。

除非有明确需求，后续不要随意改动模式 5 的循迹相关参数，尤其是速度、PD、白段桥接和 IMU 出弯参数。

### 8.3 可调参数

同模式 1：

- 基础速度：`BOARD_MOTOR_START_PERCENT`
- 最大输出：`BOARD_MOTOR_MAX_PERCENT`
- 直行偏置：`BOARD_STRAIGHT_TRIM_PERCENT`
- 循迹增益：`BOARD_LINE_KP_PER_MILLE`、`BOARD_LINE_KD_PER_MILLE`
- 白段桥接：`BOARD_LINE_BRIDGE_*`
- IMU 出弯：`BOARD_IMU_ENABLE_HEADING_HOLD`、`BOARD_HEADING_KP_PER_DEG`、`BOARD_BRIDGE_HEADING_OFFSET_X10`
- 关键点稳定：`BOARD_KEYPOINT_TRANSITION_STABLE_MS`
- 关键点冷却：`BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS`
- 声光时长：`BOARD_KEYPOINT_BEEP_MS`

## 9. 模式 6：K230/云台/激光在线测试

源码：

- 枚举：`MODE_GIMBAL_TEST`
- 入口：`task_gimbal_test()`
- 实际云台控制：主循环里模式 6 运行时额外调用 `gimbal_task()`

### 9.1 运行流程

模式 6 启动时：

- `gimbal_start()`：
  - 打开激光。
  - 初始化 PCA9685。
  - 云台 yaw/pitch 位置计数归零。
- 主循环持续接收 K230 UART。
- 主循环持续调用 `gimbal_task()`。
- `task_gimbal_test()` 只负责显示 K230 通信状态，不负责产生云台动作。

### 9.2 LED 状态

`task_gimbal_test()` 中：

- 有 fresh 且 valid 的 K230 帧：绿灯常亮，红灯灭。
- 有帧但无效/过期：红灯闪。
- 有串口字节但未解析出帧：绿灯闪、红灯亮。
- 完全无数据：红灯常亮。

### 9.3 云台控制逻辑

文件：`src/gimbal.c`

`gimbal_task()`：

1. 检查云台是否 active。
2. 检查 PCA9685 是否 ready，不 ready 会按 `BOARD_GIMBAL_PCA_RETRY_MS` 重试。
3. 从 K230 读取 fresh frame。
4. 如果路线前馈 active，则先使用路线窗口误差，再叠加 K230 小幅视觉 trim。
5. 如果路线前馈不 active，则直接使用 K230 的 yaw/pitch。
6. 调用 `axis_update()` 驱动 yaw/pitch 步进。

步进参数：

- `BOARD_GIMBAL_YAW_DEADBAND_CDEG = 2`
- `BOARD_GIMBAL_PITCH_DEADBAND_CDEG = 2`
- `BOARD_GIMBAL_YAW_MAX_ERROR_CDEG = 12`
- `BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG = 12`
- `BOARD_GIMBAL_YAW_STEP_CDEG = 9`
- `BOARD_GIMBAL_PITCH_STEP_CDEG = 9`
- `BOARD_GIMBAL_YAW_STEP_MIN_MS = 3`
- `BOARD_GIMBAL_YAW_STEP_MAX_MS = 14`
- `BOARD_GIMBAL_PITCH_STEP_MIN_MS = 3`
- `BOARD_GIMBAL_PITCH_STEP_MAX_MS = 14`
- `BOARD_GIMBAL_I2C_WRITES_PER_TASK = 2`

`axis_compute_interval()` 会根据误差大小在 min/max step interval 之间插值：

- 误差越大，步进越快。
- 误差进入 deadband 后停止。

### 9.4 可调参数

MSP0：

- `BOARD_K230_VISION_TIMEOUT_MS`
- `BOARD_GIMBAL_YAW_DEADBAND_CDEG`
- `BOARD_GIMBAL_PITCH_DEADBAND_CDEG`
- `BOARD_GIMBAL_YAW_MAX_ERROR_CDEG`
- `BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG`
- `BOARD_GIMBAL_YAW_DIR`
- `BOARD_GIMBAL_PITCH_DIR`
- `BOARD_GIMBAL_YAW_STEP_CDEG`
- `BOARD_GIMBAL_PITCH_STEP_CDEG`
- `BOARD_GIMBAL_YAW_STEP_MIN_MS`
- `BOARD_GIMBAL_YAW_STEP_MAX_MS`
- `BOARD_GIMBAL_PITCH_STEP_MIN_MS`
- `BOARD_GIMBAL_PITCH_STEP_MAX_MS`
- `BOARD_GIMBAL_I2C_WRITES_PER_TASK`

K230：

- `CONTROL_SEND_HZ`
- `CONTROL_GAIN`
- `CONTROL_MAX_DEG`
- `FIXED_LASER_CONTROL_MAX_DEG`
- `FIXED_LASER_DEADBAND_PX`
- `RED_PANEL_CENTER_CONTROL_MAX_DEG`
- `RED_PANEL_CENTER_FINE_CONTROL_MAX_DEG`
- `PITCH_GUARD_*`

## 10. K230 当前视觉策略

主要文件：

- `k230/k230_target_uart.py`
- `k230/k230_target_config.py`
- `k230/main.py`
- `k230/k230_connected.ps1`

当前版本：

- `SCRIPT_VERSION = "panel_center_v72_mode2_search_gate"`
- `/sdcard/k230_target_uart_v72.py`
- `/sdcard/k230_target_config_v72.py`
- `/sdcard/main.py`

`main.py` 会在 K230 上电后自动 import `k230_target_uart_v72` 并持续运行。比赛时可以让 K230 全程独立运行，MSP0 只在对应模式下决定是否消费 UART 指令。

### 10.1 总体目标选择

当前靶面是学校要求的红色同心圆面板。K230 策略是：

1. 优先识别官方红环/黑框面板。
2. 对靶中心主要使用黑框/面板中心。
3. 红环更多作为“这是正确靶面”的确认/锚点。
4. 不再依赖真实激光红点识别作为主输入。
5. 激光位置按相机画面中的固定参考点估计。

原因：

- 远距离下真实激光红点太小，不稳定。
- 摄像头与激光相对安装固定，可以用近似固定参考点消除一部分误差。
- 但相机平面与靶面角度变化时，固定点仍会有偏差，所以保留了远近插值函数。

### 10.2 摄像头旋转

摄像头物理安装相对正面顺时针旋转 90 度：

- `CAMERA_ROTATION_CW_DEG = 90`

这会影响“截图方向”和“实物方向”的对应关系。当前代码已经考虑了旋转。现场讨论时特别注意：

- 截图中的水平宽度，对应现实靶面高度方向的投影。
- 当前固定激光远近参考选择 `FIXED_LASER_RANGE_BY_FRAME_IMAGE_W = True`，就是因为旋转后截图横向尺寸更接近现实靶高的距离指标。

### 10.3 固定激光参考点

当前配置：

- `FIXED_LASER_ENABLED = True`
- `REAL_LASER_PREFERRED = False`
- `FIXED_LASER_CLOSED_LOOP_ENABLED = True`
- `FIXED_LASER_CX = 207.0`
- `FIXED_LASER_CY = 82.0`
- `FIXED_LASER_RANGE_REF_ENABLED = True`
- `FIXED_LASER_RANGE_BY_FRAME_IMAGE_W = True`
- `FIXED_LASER_FAR_TARGET_W = 70.0`
- `FIXED_LASER_NEAR_TARGET_W = 125.0`
- `FIXED_LASER_FAR_CX = 207.0`
- `FIXED_LASER_FAR_CY = 82.0`
- `FIXED_LASER_NEAR_CX = 207.0`
- `FIXED_LASER_NEAR_CY = 82.0`

虽然远近插值已经启用，但当前远/近点数值一样，等价于固定参考点。后续如果重新标定不同距离下的偏差，可以只改 far/near 的 cx/cy，不需要改主算法。

### 10.4 K230 控制输出

当前平滑/输出参数：

- `CONTROL_SEND_HZ = 45`
- `LOST_SEND_HZ = 15`
- `TARGET_FILTER_WINDOW = 2`
- `TARGET_FILTER_ALPHA = 0.98`
- `LASER_FILTER_WINDOW = 2`
- `LASER_FILTER_ALPHA = 0.85`
- `CONTROL_GAIN = 0.45`
- `CONTROL_MAX_DEG = 0.16`
- `FIXED_LASER_CONTROL_MAX_DEG = 0.04`
- `FIXED_LASER_DEADBAND_PX = 5.0`
- `ERROR_DEADBAND_PX = 2.0`

经验：

- 想让反应更快，优先提高 `CONTROL_SEND_HZ`，不要一上来增大每帧角度。
- 远距离抖动/过冲，优先降低 `*_CONTROL_MAX_DEG` 或增大 deadband。
- 如果起步跟不上，检查 K230 帧率、MSP0 步进间隔和 I2C 写次数三者是否匹配。

### 10.5 黑框/红环关键阈值

黑框/面板：

- `SQUARE_THRESHOLD = (0, 20, -128, 127, -128, 127)`
- `RED_PANEL_FRAME_ENABLED = True`
- `RED_PANEL_FRAME_MIN_W = 34`
- `RED_PANEL_FRAME_MIN_H = 44`
- `RED_PANEL_FRAME_MIN_DENSITY = 1.5`
- `RED_PANEL_FRAME_MAX_DENSITY = 88.0`
- `RED_PANEL_RECT_CENTER_PRIORITY_ENABLED = True`
- `RED_PANEL_CENTER_FALLBACK_ENABLED = True`

红环：

- `RED_RINGS_TARGET_ENABLED = True`
- `RED_RINGS_THRESHOLDS = ((0, 100, 8, 127, -80, 45), (0, 100, 18, 127, -80, 60))`
- `RED_RINGS_COMPLETE_AIM_ENABLED = True`
- `RED_RINGS_DIRECT_AIM_FALLBACK_ENABLED = True`

模式 2 搜索确认：

- `RED_PANEL_MODE2_FRAME_FALLBACK_ENABLED = True`
- `RED_PANEL_MODE2_FRAME_MIN_W = 42`
- `RED_PANEL_MODE2_FRAME_MIN_H = 52`
- `RED_PANEL_MODE2_FRAME_MIN_SCORE = 2400`

## 11. `board_config.h` 当前参数快照

文件：`firmware/ecar_debug_mspm0g3507/src/board_config.h`

### 11.1 硬件极性

```c
BOARD_MOTOR_LEFT_INVERT             0
BOARD_MOTOR_RIGHT_INVERT            0
BOARD_ENCODER_LEFT_INVERT           0
BOARD_ENCODER_RIGHT_INVERT          0
BOARD_STEERING_INVERT               0
BOARD_BEEP_ACTIVE_HIGH              0
BOARD_GRAY_BLACK_IS_HIGH            1
BOARD_GRAY_REVERSE_ORDER            0
```

### 11.2 电机与循迹

```c
BOARD_PWM_PERIOD_COUNTS             1600U
BOARD_MOTOR_TEST_PERCENT            12
BOARD_MOTOR_START_PERCENT           30
BOARD_MOTOR_MAX_PERCENT             85
BOARD_MOTOR_SEARCH_PERCENT          11
BOARD_MOTOR_BRAKE_ON_STOP           0
BOARD_STRAIGHT_TRIM_PERCENT         10
BOARD_LINE_KP_PER_MILLE             9
BOARD_LINE_KD_PER_MILLE             22
BOARD_LINE_CONTROL_PERIOD_MS        10U
BOARD_LINE_START_DELAY_MS           1000U
BOARD_LINE_LOST_HOLD_MS             250U
BOARD_LINE_LOST_STOP_MS             1200U
BOARD_LINE_BRIDGE_MAX_MS            6000U
BOARD_LINE_BRIDGE_MAX_MM            1400
BOARD_LINE_BRIDGE_KEEP_LAST_MS      600U
BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT 22
```

### 11.3 编码器与 IMU

```c
BOARD_WHEEL_CIRCUMFERENCE_MM        210
BOARD_ENCODER_TICKS_PER_REV         1170
BOARD_ENCODER_SELFTEST_MIN_TICKS    4
BOARD_IMU_ENABLE_HEADING_HOLD       1
BOARD_IMU_VALID_MS                  300U
BOARD_HEADING_KP_PER_DEG            6
BOARD_HEADING_INVERT                0
BOARD_BRIDGE_HEADING_OFFSET_X10     222
```

### 11.4 关键点

```c
BOARD_KEYPOINT_B_MM                 1000
BOARD_KEYPOINT_C_MM                 2257
BOARD_KEYPOINT_D_MM                 3257
BOARD_KEYPOINT_A_MM                 4513
BOARD_KEYPOINT_GATE_BEFORE_MM       300
BOARD_KEYPOINT_GATE_AFTER_MM        900
BOARD_KEYPOINT_TRANSITION_STABLE_MS 60U
BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS   500U
BOARD_KEYPOINT_B_STOP_MS            2500U
BOARD_KEYPOINT_BEEP_MS              180U
```

注意：当前距离 gate 参数保留但未在关键点判定中使用。

### 11.5 云台/K230

```c
BOARD_LASER_ACTIVE_HIGH             1
BOARD_K230_VISION_TIMEOUT_MS        300U
BOARD_GIMBAL_YAW_DEADBAND_CDEG      2
BOARD_GIMBAL_PITCH_DEADBAND_CDEG    2
BOARD_GIMBAL_YAW_MAX_ERROR_CDEG     12
BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG   12
BOARD_GIMBAL_YAW_DIR                1
BOARD_GIMBAL_PITCH_DIR              1
BOARD_GIMBAL_YAW_STEP_CDEG          9
BOARD_GIMBAL_PITCH_STEP_CDEG        9
BOARD_GIMBAL_POSITION_LIMIT_CDEG    20000
BOARD_GIMBAL_YAW_STEP_MIN_MS        3U
BOARD_GIMBAL_YAW_STEP_MAX_MS        14U
BOARD_GIMBAL_PITCH_STEP_MIN_MS      3U
BOARD_GIMBAL_PITCH_STEP_MAX_MS      14U
BOARD_GIMBAL_YAW_STEPS_PER_FRAME    4U
BOARD_GIMBAL_PITCH_STEPS_PER_FRAME  4U
BOARD_GIMBAL_I2C_WRITES_PER_TASK    2U
BOARD_GIMBAL_LOCAL_FORCE_TEST       0
BOARD_GIMBAL_RELEASE_COILS_IDLE     0
BOARD_GIMBAL_PCA_RETRY_MS           500U
BOARD_PCA9685_I2C_TIMEOUT_MS        3U
```

### 11.6 模式 3 路线前馈

```c
BOARD_GIMBAL_ROUTE_ENABLE            1
BOARD_GIMBAL_ROUTE_LAP_MM            BOARD_KEYPOINT_A_MM
BOARD_GIMBAL_ROUTE_YAW_A_CDEG        0
BOARD_GIMBAL_ROUTE_YAW_B_CDEG        0
BOARD_GIMBAL_ROUTE_YAW_C_CDEG        0
BOARD_GIMBAL_ROUTE_YAW_D_CDEG        0
BOARD_GIMBAL_ROUTE_YAW_A2_CDEG       0
BOARD_GIMBAL_ROUTE_PITCH_A_CDEG      0
BOARD_GIMBAL_ROUTE_PITCH_B_CDEG      0
BOARD_GIMBAL_ROUTE_PITCH_C_CDEG      0
BOARD_GIMBAL_ROUTE_PITCH_D_CDEG      0
BOARD_GIMBAL_ROUTE_PITCH_A2_CDEG     0
BOARD_GIMBAL_ROUTE_YAW_WINDOW_CDEG   700
BOARD_GIMBAL_ROUTE_PITCH_WINDOW_CDEG 450
BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG  18
BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG 6000
BOARD_GIMBAL_ROUTE_POSITION_LIMIT_CDEG 8000
```

### 11.7 模式 2 定点打靶

```c
BOARD_TARGET_MODE_CYCLES             3U
BOARD_TARGET_MODE_HALF_TURN_TEST     0
BOARD_TARGET_MODE_CCW_SIGN           -1
BOARD_TARGET_MODE_SEARCH_HALF_CDEG   18432
BOARD_TARGET_MODE_SEARCH_SOFT_CDEG   BOARD_GIMBAL_POSITION_LIMIT_CDEG
BOARD_TARGET_MODE_SEARCH_HALF_STEPS  2048L
BOARD_TARGET_MODE_SEARCH_FULL_STEPS  4096L
BOARD_TARGET_MODE_SEARCH_TOL_CDEG    18
BOARD_TARGET_MODE_ZERO_TOL_CDEG      20
BOARD_TARGET_MODE_SCAN_STEP_MS       18U
BOARD_TARGET_MODE_RETURN_STEP_MS     6U
BOARD_TARGET_MODE_AIM_STEP_MS        12U
BOARD_TARGET_MODE_AIM_MAX_CDEG       8
BOARD_TARGET_MODE_AIM_DEADBAND_CDEG  1
BOARD_TARGET_MODE_AIM_STABLE_CDEG    2
BOARD_TARGET_MODE_AIM_MIN_MS         900U
BOARD_TARGET_MODE_AIM_STABLE_MS      700U
BOARD_TARGET_MODE_VALID_STABLE_MS    160U
BOARD_TARGET_MODE_VALID_MIN_FRAMES   4U
BOARD_TARGET_MODE_CANDIDATE_HOLD_MS  260U
BOARD_TARGET_MODE_LOST_MS            500U
BOARD_TARGET_MODE_REPLACE_WAIT_MS    7000U
BOARD_TARGET_MODE_SEARCH_RETRY_MS    600U
```

## 12. 编译、烧录与 K230 部署

### 12.1 MSPM0 一键编译烧录

目录：

```powershell
cd E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507
```

一键编译并烧录：

```powershell
.\flash_connected.cmd
```

常用选项在 `flash_connected.ps1`：

- `-Clean`：先清理再编译。
- `-NoBuild`：不编译，只烧录已有 `.out`。
- `-NoFlash`：只编译不烧录。
- `-SkipProbeCheck`：跳过 XDS110 探针检查。
- `-CcsRoot <path>`：指定 CCS 路径。
- `-SdkRoot <path>`：指定 MSPM0 SDK 路径。

如果烧录阶段提示找不到 XDS110，而 Windows 只看到 K230 COM 口，说明 MSPM0 调试器没接好或没被识别。先重新插 MSPM0/XDS110，再运行 `flash_connected.cmd`。

### 12.2 K230 上传并设为上电自启动

目录：

```powershell
cd E:\clawSpace\BUPT-2026-EDC-SmartCar\BUPT_ECar_2026\k230相关\target_tracking_delivery_20260627
```

上传 v72 脚本、配置和 boot 入口：

```powershell
.\k230\k230_connected.ps1 -Upload
```

脚本会先强制 K230 进入 REPL，再上传：

- `k230_target_uart.py` -> `/sdcard/k230_target_uart_v72.py`
- `k230_target_config.py` -> `/sdcard/k230_target_config_v72.py`
- `main.py` -> `/sdcard/main.py`

之后 K230 上电会自动运行 tracker。MSP0 不需要控制 K230 启停，只在模式 2/3/6 里使用串口数据。

### 12.3 K230 常用调试命令

重置到 REPL：

```powershell
.\k230\k230_connected.ps1 -Reset
```

抓一张图：

```powershell
.\k230\k230_connected.ps1 -Capture
```

输出路径：

```text
k230_capture_latest.jpg
```

单帧诊断，不控制云台：

```powershell
.\k230\k230_connected.ps1 -Diagnose
```

输出路径：

```text
k230_diag_latest.jpg
```

强制发送一次云台命令，用来排查 MSP0/云台是否响应：

```powershell
.\k230\k230_connected.ps1 -Force -Seconds 1 -ForceYaw 0.0 -ForcePitch 1.0
```

持续运行当前脚本：

```powershell
.\k230\k230_connected.ps1 -Run
```

如果 `/sdcard/*.py` 被运行中的脚本锁住，优先强制 `-Reset`。如果仍然有版本锁/旧脚本残留，现场最稳的办法是递增版本号，例如 v72 -> v73，同时改 `k230_connected.ps1` 的远端文件名和 `main.py` import。

## 13. 现场调试建议

### 13.1 不要轻易动模式 5 循迹参数

当前模式 5 已经实测：

- 循迹效果好。
- B/C/D/A 声光提示稳定。
- 无明显噪声误触发。
- 能持续跑圈。

如果只是调模式 2/6/K230，不要改这些：

- `BOARD_MOTOR_START_PERCENT`
- `BOARD_MOTOR_MAX_PERCENT`
- `BOARD_STRAIGHT_TRIM_PERCENT`
- `BOARD_LINE_KP_PER_MILLE`
- `BOARD_LINE_KD_PER_MILLE`
- `BOARD_LINE_BRIDGE_*`
- `BOARD_HEADING_*`
- `BOARD_KEYPOINT_*`

### 13.2 模式 2 搜索误识别

优先看 K230 的 target 源，不要盲目放宽 MSP0 稳定参数。

推荐流程：

1. K230 `-Diagnose`。
2. 看输出中的 target 源是否为 `red_panel_center` 或 `red_panel_frame_center`。
3. 如果是其他黑色区域，收紧 `target_ok_for_mode2()` 或 `RED_PANEL_MODE2_FRAME_*`。
4. 如果靶面在正前方却识别不到，先放宽红环/黑框检测阈值，再看 `mode2_target_ok`。

### 13.3 模式 6 对靶偏差

如果近距离准、远距离偏：

- 调 `FIXED_LASER_FAR_CX/FAR_CY` 和 `FIXED_LASER_NEAR_CX/NEAR_CY`。
- 当前远近值相同，意味着还没有真正建立远近补偿曲线。
- 因为摄像头旋转 90 度，远近估计使用目标在截图中的宽度 `t_w`。

如果响应慢：

- K230 先看 `CONTROL_SEND_HZ`。
- MSP0 再看 `BOARD_GIMBAL_*_STEP_MIN_MS/MAX_MS` 和 `BOARD_GIMBAL_I2C_WRITES_PER_TASK`。
- 不建议简单把每帧角度增大太多，否则远距离会更容易过冲。

### 13.4 模式 3 后续要做的工作

模式 3 的框架已经搭好，但路线中心角还全是 0。要完成“一边循迹一边动态对靶”，重点不是再猛调 K230，而是补上路线前馈：

1. 在 A/B/C/D/A2 记录理想 yaw/pitch。
2. 填入 `BOARD_GIMBAL_ROUTE_YAW_*` 和 `BOARD_GIMBAL_ROUTE_PITCH_*`。
3. 用较大的 `BOARD_GIMBAL_ROUTE_*_WINDOW_CDEG` 确保不丢靶。
4. 最后让 K230 只做小范围修正。

## 14. 代码入口速查

MSP0：

- `src/main.c`：模式状态机、按键、任务入口。
- `src/board_config.h`：所有主要现场参数。
- `src/line_follow.c`：循迹、白段桥接、IMU 航向保持。
- `src/sound_light.c`：声光提示。
- `src/gimbal.c`：云台步进控制、K230 误差执行。
- `src/gimbal_route.c`：模式 3 路线前馈窗口。
- `src/k230_uart.c`：K230 串口解析。
- `flash_connected.cmd` / `flash_connected.ps1`：一键编译烧录。

K230：

- `k230/k230_target_uart.py`：主视觉算法和串口输出。
- `k230/k230_target_config.py`：现场可调参数。
- `k230/main.py`：K230 上电自启动入口。
- `k230/k230_connected.ps1`：上传、运行、诊断、抓图、强制命令工具。

## 15. 当前已知风险

- 模式 2 搜索有概率误识别或漏识别，虽然已经加了 `target_ok` 搜索门控，但仍建议继续用实际靶面测试。
- 模式 2 找到靶后源码会进入精确对靶；如果现场看起来静止，可能是 K230 输出无效、误差落入死区、步幅太小或 PCA/I2C 未响应。
- 模式 3 路线前馈未完成标定，中心角全部为 0。
- K230 固定激光参考点是近似模型，不同距离/角度下仍可能有偏差。
- 关键点距离 gate 参数当前未生效，模式 1/5 依赖稳定黑白切换本身。
- 固件目录下旧 README 和早期参数文档可能保留历史信息，后续以本文和当前源码为准。
