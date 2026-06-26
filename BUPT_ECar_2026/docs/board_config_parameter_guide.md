# board_config.h 参数说明与调参指南

本文档对应当前 CCS 实际编译文件：

```text
C:\Users\xwmxc\workspace_ccstheia\ecar_debug_mspm0g3507_LP_MSPM0G3507_nortos_ticlang\board_config.h
```

注意：CCS 导入后，真正烧录用的是上面这个工作区文件。`D:\BUPT_ECar_2026\firmware\ecar_debug_mspm0g3507\src\board_config.h` 是源码副本，只有同步后才会影响 CCS 工作区。

## 调参总原则

```text
1. 每次只改 1 个参数，烧录后只观察 1 个主要现象。
2. 先调硬件极性，再调灰度方向，再调循迹，再调出弯桥接，最后调关键点。
3. 模式 2 看灰度，模式 3 调低速循迹，模式 4/5 跑正式白-黑-白-黑赛道。
4. 调完参数必须重新 Build/烧录；只保存文件不会进板子。
```

## 当前关键参数快照

```c
BOARD_MOTOR_START_PERCENT           24
BOARD_MOTOR_MAX_PERCENT             55
BOARD_STRAIGHT_TRIM_PERCENT         3
BOARD_LINE_KP_PER_MILLE             7
BOARD_LINE_KD_PER_MILLE             18
BOARD_LINE_BRIDGE_MAX_MM            1400
BOARD_LINE_BRIDGE_KEEP_LAST_MS      500U
BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT 18
BOARD_HEADING_KP_PER_DEG            5
BOARD_BRIDGE_HEADING_OFFSET_X10     222
BOARD_KEYPOINT_GATE_BEFORE_MM       350
BOARD_KEYPOINT_TRANSITION_STABLE_MS 60U
```

## 1. 硬件方向与极性

### BOARD_MOTOR_LEFT_INVERT / BOARD_MOTOR_RIGHT_INVERT

当前：

```c
#define BOARD_MOTOR_LEFT_INVERT  (0)
#define BOARD_MOTOR_RIGHT_INVERT (0)
```

含义：左右轮组电机方向反相开关。

怎么调：

```text
模式 0 四轮悬空测试。
左侧阶段 LF/LR 应同向前进，右侧阶段 RF/RR 应同向前进。
如果某一整侧都反着转，把对应参数改 1。
如果同侧前后轮方向相反，优先换该电机输出线，不要靠这个宏解决。
```

### BOARD_ENCODER_LEFT_INVERT / BOARD_ENCODER_RIGHT_INVERT

当前：

```c
#define BOARD_ENCODER_LEFT_INVERT  (0)
#define BOARD_ENCODER_RIGHT_INVERT (0)
```

含义：左右后轮编码器计数方向反相。

怎么调：

```text
模式 1 编码器检查通过后先不动。
如果后续关键点门限明显越来越离谱，或者一侧编码器方向反，改对应参数。
```

### BOARD_STEERING_INVERT

当前：

```c
#define BOARD_STEERING_INVERT (0)
```

含义：循迹修正方向反相。

怎么调：

```text
模式 3 低速放在黑线旁边测试。
如果车看到黑线后往远离黑线方向修，改成 1。
如果只是直线轻微偏左/偏右，不改这个，改 BOARD_STRAIGHT_TRIM_PERCENT。
```

### BOARD_BEEP_ACTIVE_HIGH

当前：

```c
#define BOARD_BEEP_ACTIVE_HIGH (0)
```

含义：蜂鸣器触发电平。你的蜂鸣器是低电平触发，所以保持 0。

怎么调：

```text
上电后蜂鸣器一直响，且确认不是程序故障时，优先检查这个。
低电平触发 = 0
高电平触发 = 1
```

## 2. 灰度传感器

### BOARD_GRAY_BLACK_IS_HIGH

当前：

```c
#define BOARD_GRAY_BLACK_IS_HIGH (1)
```

含义：灰度 OUT 读到高电平时是否表示黑线。

怎么调：

```text
模式 2 灰度监看。
黑线下绿灯应亮，白底下红灯亮/绿灯灭。
如果白底也一直认为看到线，或黑白完全反了，把 1 改 0。
```

### BOARD_GRAY_REVERSE_ORDER

当前：

```c
#define BOARD_GRAY_REVERSE_ORDER (0)
```

含义：八路灰度从左到右的顺序是否反了。

怎么调：

```text
如果黑线在车左侧，但程序表现像黑线在右侧，改成 1。
如果只是不跟线，先确认 BOARD_GRAY_BLACK_IS_HIGH 和 BOARD_STEERING_INVERT。
```

## 3. 电机速度与停车方式

### BOARD_PWM_PERIOD_COUNTS

当前：

```c
#define BOARD_PWM_PERIOD_COUNTS (1600U)
```

含义：PWM 周期计数，必须和 SysConfig 里的 PWM 周期匹配。

怎么调：

```text
通常不要改。
如果改 SysConfig 定时器周期，才同步改这里。
```

### BOARD_MOTOR_TEST_PERCENT

当前：

```c
#define BOARD_MOTOR_TEST_PERCENT (12)
```

含义：模式 0/1 电机点动测试占空比。

怎么调：

```text
悬空测试转不起来：加到 14-18。
测试太猛：降到 8-10。
这个参数不影响模式 3/4/5 正式循迹速度。
```

### BOARD_MOTOR_START_PERCENT

当前：

```c
#define BOARD_MOTOR_START_PERCENT (24)
```

含义：循迹基础速度。模式 3/4/5 正常前进主要看它。

怎么调：

```text
车太慢：每次 +2，例如 24 -> 26。
弯道冲出去、出弯漂得厉害：每次 -2。
常用范围：18-32。
```

### BOARD_MOTOR_MAX_PERCENT

当前：

```c
#define BOARD_MOTOR_MAX_PERCENT (55)
```

含义：电机最大输出限幅，也限制循迹修正时左右轮最大差速。

怎么调：

```text
弯道修不回来：可以 +5。
车突然猛摆或轮子打滑：可以 -5。
常用范围：40-70。
```

### BOARD_MOTOR_SEARCH_PERCENT

当前：

```c
#define BOARD_MOTOR_SEARCH_PERCENT (11)
```

含义：模式 3 严格循迹丢线时的找线转向强度。

怎么调：

```text
模式 4/5 走白段主要靠 bridge 和 IMU，不靠这个。
模式 3 丢线找不回来：略加。
模式 3 丢线后转得太猛：略减。
```

### BOARD_MOTOR_BRAKE_ON_STOP

当前：

```c
#define BOARD_MOTOR_BRAKE_ON_STOP (0)
```

含义：停车时是否使用 TB6612 刹车模式。

怎么调：

```text
保持 0。
之前出现过停止时异常高速转，和刹车/驱动板状态有关，所以正式调试先用滑行停。
```

### BOARD_STRAIGHT_TRIM_PERCENT

当前：

```c
#define BOARD_STRAIGHT_TRIM_PERCENT (3)
```

含义：直线静态微调。正数让左侧更快、右侧更慢，车会更偏右；负数让车更偏左。

怎么调：

```text
直线一直右偏：把数值减小，例如 3 -> 1 -> 0 -> -1。
直线一直左偏：把数值增大，例如 0 -> 1 -> 2。
每次只改 1-2，不要大跳。
常用范围：-5 到 +5。
```

注意：如果车在黑线上左右摆，不是这个参数的问题，优先调 KP/KD。

## 4. 编码器几何参数

### BOARD_WHEEL_CIRCUMFERENCE_MM

当前：

```c
#define BOARD_WHEEL_CIRCUMFERENCE_MM (210)
```

含义：轮子一圈对应的地面距离，单位 mm。

怎么调：

```text
如果编码器里程整体偏小，增大这个值。
如果编码器里程整体偏大，减小这个值。
```

### BOARD_ENCODER_TICKS_PER_REV

当前：

```c
#define BOARD_ENCODER_TICKS_PER_REV (1170)
```

含义：轮子转一圈对应的编码器计数。

怎么调：

```text
一般不动，除非确认编码器型号/减速比和当前值不一致。
如果只是关键点提前/滞后，优先调 BOARD_KEYPOINT_*_MM 和 BOARD_KEYPOINT_GATE_BEFORE_MM。
```

### BOARD_ENCODER_SELFTEST_MIN_TICKS

当前：

```c
#define BOARD_ENCODER_SELFTEST_MIN_TICKS (4)
```

含义：模式 1 点动时，判断编码器是否有计数的最低阈值。

怎么调：

```text
一般不动。
模式 1 明明有计数但偶发红灯，可以略降。
```

## 5. 基础循迹控制

### BOARD_LINE_KP_PER_MILLE

当前：

```c
#define BOARD_LINE_KP_PER_MILLE (7)
```

含义：循迹比例增益。黑线偏离中心越多，修正越强。

怎么调：

```text
弯道跟不住、车头往外跑：+1 或 +2。
直线左右蛇形摆动：-1 或 -2。
速度提高后通常需要略提高 KP 或降低速度。
```

### BOARD_LINE_KD_PER_MILLE

当前：

```c
#define BOARD_LINE_KD_PER_MILLE (18)
```

含义：循迹微分增益，用来抑制左右摆动。

怎么调：

```text
直线左右摆动：+2 到 +5。
车反应很抖、灰度噪声一来就抽动：-2 到 -5。
KD 太高会让车对灰度噪声敏感。
```

### BOARD_LINE_CONTROL_PERIOD_MS

当前：

```c
#define BOARD_LINE_CONTROL_PERIOD_MS (10U)
```

含义：循迹控制周期，10 ms 一次。

怎么调：

```text
一般不要改。
改小会更频繁但更吃噪声，改大会反应慢。
```

### BOARD_LINE_START_DELAY_MS

当前：

```c
#define BOARD_LINE_START_DELAY_MS (1000U)
```

含义：启动后等待时间。按键启动后先等 1 秒再走。

怎么调：

```text
想按下马上走：减小。
需要扶正车头/放手时间：增大。
```

### BOARD_LINE_LOST_HOLD_MS / BOARD_LINE_LOST_STOP_MS

当前：

```c
#define BOARD_LINE_LOST_HOLD_MS (250U)
#define BOARD_LINE_LOST_STOP_MS (1200U)
```

含义：模式 3 严格循迹丢线保护。

怎么调：

```text
模式 3 短暂丢线就停：适当增大 LOST_STOP。
模式 3 丢线后盲跑太久：减小 LOST_STOP。
模式 4/5 的白段桥接主要看下一组 bridge 参数。
```

## 6. 白段桥接与出弯修正

你的赛道是白 -> 黑 -> 白 -> 黑。模式 4/5 允许在白段无黑线时继续走，并用上一帧转向和 IMU 保方向。

### BOARD_LINE_BRIDGE_MAX_MS / BOARD_LINE_BRIDGE_MAX_MM

当前：

```c
#define BOARD_LINE_BRIDGE_MAX_MS (6000U)
#define BOARD_LINE_BRIDGE_MAX_MM (1400)
```

含义：模式 4/5 在白段最多允许无黑线行驶多久/多远。

怎么调：

```text
白色直道中途报警停车：增大 BRIDGE_MAX_MM，例如 1400 -> 1600。
车丢线后盲跑太久：减小 BRIDGE_MAX_MM。
你的白直道 1000 mm，当前 1400 mm 是留了 400 mm 余量。
```

### BOARD_LINE_BRIDGE_KEEP_LAST_MS

当前：

```c
#define BOARD_LINE_BRIDGE_KEEP_LAST_MS (500U)
```

含义：刚从黑弯出到白段时，继续保持上一帧弯道转向的时间。

怎么调：

```text
出弯瞬间像没有矫正、直接往外飘：+100，例如 500 -> 600。
出弯后内切太多、拐过头：-100。
常用范围：250-700。
```

### BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT

当前：

```c
#define BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT (18)
```

含义：出弯保持上一帧转向时，允许保留的最大修正强度。

怎么调：

```text
出弯内收不够：+2 到 +4。
出弯拐太狠：-2 到 -4。
常用范围：10-25。
```

## 7. IMU 航向保持

### BOARD_IMU_ENABLE_HEADING_HOLD

当前：

```c
#define BOARD_IMU_ENABLE_HEADING_HOLD (1)
```

含义：白段/短暂丢线时是否使用 IMU 航向保持。

怎么调：

```text
IMU 接线和数据正常时保持 1。
如果 IMU 数据明显乱跳，临时改 0，用纯桥接验证底盘。
```

### BOARD_IMU_VALID_MS

当前：

```c
#define BOARD_IMU_VALID_MS (300U)
```

含义：超过多久没收到 IMU 新数据，就认为航向无效。

怎么调：

```text
一般不动。
如果 IMU 帧率低但稳定，可以略增大。
```

### BOARD_HEADING_KP_PER_DEG

当前：

```c
#define BOARD_HEADING_KP_PER_DEG (5)
```

含义：IMU 航向误差修正强度。数值越大，白段方向修得越明显。

怎么调：

```text
白段出弯后仍然不往目标方向修：+1。
白段左右摆或修过头：-1。
常用范围：2-7。
```

### BOARD_HEADING_INVERT

当前：

```c
#define BOARD_HEADING_INVERT (0)
```

含义：IMU yaw 方向反相。

怎么调：

```text
如果白段航向修正方向完全反了，先尝试改这个。
如果只是出弯目标角度不对，优先改 BOARD_BRIDGE_HEADING_OFFSET_X10。
```

### BOARD_BRIDGE_HEADING_OFFSET_X10

当前：

```c
#define BOARD_BRIDGE_HEADING_OFFSET_X10 (222)
```

含义：进入白段时给 IMU 目标航向加一个偏置，单位是 0.1 度。222 表示 22.2 度。

怎么调：

```text
出弯后仍偏外：继续按同方向加大，例如 222 -> 260。
出弯后反而内切太多：减小，例如 222 -> 150。
如果改大后偏外更严重，说明符号反了，改成负数，例如 -222。
```

这个参数强烈依赖 IMU 安装方向和赛道方向，实车试出来的符号优先。

## 8. 关键点检测

模式 4/5 当前关键点不是纯编码器里程，而是稳定黑白切换：

```text
B: 白 -> 黑
C: 黑 -> 白
D: 白 -> 黑
A: 黑 -> 白
```

编码器距离只作为门限，防止还没到关键点区域时灰度抖动误判。

### BOARD_KEYPOINT_B_MM / C_MM / D_MM / A_MM

当前：

```c
#define BOARD_KEYPOINT_B_MM (1000)
#define BOARD_KEYPOINT_C_MM (2257)
#define BOARD_KEYPOINT_D_MM (3257)
#define BOARD_KEYPOINT_A_MM (4513)
```

含义：各关键点预计里程，用来打开黑白切换检测窗口。

怎么调：

```text
某个点总是太晚才提示：减小对应 BOARD_KEYPOINT_*_MM。
某个点还没到就可能提示：增大对应 BOARD_KEYPOINT_*_MM，或减小 GATE_BEFORE。
```

### BOARD_KEYPOINT_GATE_BEFORE_MM

当前：

```c
#define BOARD_KEYPOINT_GATE_BEFORE_MM (350)
```

含义：在预计关键点前多少 mm 开始允许检测黑白切换。

例子：

```text
B_MM = 1000, GATE_BEFORE = 350
那么从 650 mm 开始，白->黑 才可能被判为 B。
650 mm 前发生的灰度变化不算 B。
```

怎么调：

```text
提示太早或容易误触发：减小 GATE_BEFORE，例如 350 -> 250。
提示太晚，边界已经过了才允许检测：增大 GATE_BEFORE，例如 350 -> 450。
```

### BOARD_KEYPOINT_TRANSITION_STABLE_MS

当前：

```c
#define BOARD_KEYPOINT_TRANSITION_STABLE_MS (60U)
```

含义：黑白状态切换后，需要稳定多久才算关键点。

怎么调：

```text
灰度抖动导致误触发：增大，例如 60 -> 80。
高速下关键点容易漏检：减小，例如 60 -> 40。
常用范围：30-100。
```

### BOARD_KEYPOINT_B_STOP_MS

当前：

```c
#define BOARD_KEYPOINT_B_STOP_MS (2500U)
```

含义：模式 5 检测到 B 点后停车等待多久。

怎么调：

```text
后续接机械臂/云台时按动作耗时调整。
只测底盘时可以减小，比如 1000U。
```

### BOARD_KEYPOINT_BEEP_MS

当前：

```c
#define BOARD_KEYPOINT_BEEP_MS (180U)
```

含义：关键点提示蜂鸣/灯光持续时间。

怎么调：

```text
提示太短看不清：加到 250。
提示影响运行节奏：降到 80-120。
当前关键点提示是非阻塞的，不会停住循迹控制。
```

## 9. 按键与模式

### BOARD_BUTTON_DEBOUNCE_MS

当前：

```c
#define BOARD_BUTTON_DEBOUNCE_MS (35U)
```

含义：按键消抖时间。

怎么调：

```text
按一次触发多次：增大到 50。
按键反应太慢：减小到 20-30。
```

### BOARD_BUTTON_LONG_MS

当前：

```c
#define BOARD_BUTTON_LONG_MS (800U)
```

含义：长按判定时间。

怎么调：

```text
长按太难触发：减小到 600。
短按容易误判成长按：增大到 1000。
```

### BOARD_MODE_COUNT

当前：

```c
#define BOARD_MODE_COUNT (6U)
```

含义：模式总数。当前 0-5 共 6 个模式。

怎么调：

```text
不要单独改。
只有 main.c 增删模式时才同步改。
```

## 10. 常见现象速查

### 直线右偏

```text
优先改 BOARD_STRAIGHT_TRIM_PERCENT 往小调：
3 -> 1 -> 0 -> -1
```

### 直线左偏

```text
优先改 BOARD_STRAIGHT_TRIM_PERCENT 往大调：
0 -> 1 -> 2 -> 3
```

### 直线左右摆

```text
先降 KP：BOARD_LINE_KP_PER_MILLE 7 -> 6
或加 KD：BOARD_LINE_KD_PER_MILLE 18 -> 22
速度太快时先降 BOARD_MOTOR_START_PERCENT。
```

### 弯道冲出去

```text
先降速度：BOARD_MOTOR_START_PERCENT 24 -> 22
再加 KP：BOARD_LINE_KP_PER_MILLE 7 -> 8
必要时提高 BOARD_MOTOR_MAX_PERCENT。
```

### 出弯瞬间无矫正

```text
先加 BOARD_LINE_BRIDGE_KEEP_LAST_MS：500 -> 600
再加 BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT：18 -> 22
仍不够再加 BOARD_HEADING_KP_PER_DEG：5 -> 6
```

### 出弯内切太多

```text
先减 BOARD_LINE_BRIDGE_KEEP_LAST_MS：500 -> 400
再减 BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT：18 -> 14
或者减 BOARD_BRIDGE_HEADING_OFFSET_X10：222 -> 150
```

### 白段中途停车

```text
增大 BOARD_LINE_BRIDGE_MAX_MM：1400 -> 1600
确认模式是 4/5，不是模式 3。
模式 3 是严格丢线保护，会停车。
```

### 关键点提前误触发

```text
减小 BOARD_KEYPOINT_GATE_BEFORE_MM：350 -> 250
或增大 BOARD_KEYPOINT_TRANSITION_STABLE_MS：60 -> 80
```

### 关键点漏检或太晚

```text
增大 BOARD_KEYPOINT_GATE_BEFORE_MM：350 -> 450
或减小 BOARD_KEYPOINT_TRANSITION_STABLE_MS：60 -> 40
或减小对应 BOARD_KEYPOINT_*_MM。
```

## 11. 推荐调参顺序

```text
1. 模式 0：确认左右轮组和方向。
2. 模式 1：确认后轮编码器有计数。
3. 模式 2：确认黑白极性和灰度顺序。
4. 模式 3：低速黑线循迹，调 STEERING_INVERT、KP、KD、速度。
5. 模式 4：跑白-黑-白-黑，调白段桥接和出弯 IMU。
6. 模式 4/5：最后调关键点门限和 B 点停车。
```
