#Grayscale Sensor -> Four-wheel Motor Driver Board   Four-wheel Motor Driver Board -> PICO2
#       5V                    5V                                5V                     VBUS
#       GND                   GND                               GND                    GND
#Grayscale Sensor -> PICO2                                      SCL                    GP7   
#       AD0          GP2                                        SDA                    GP6
#       AD1          GP3
#       AD2          GP4
#       OUT          GP5
from machine import Pin,I2C
import time
import struct
########################################################
# 重要配置参数 Important Configuration Parameters
########################################################
# 根据自己使用的电机类型，选择对应的电机参数配置
# Select the corresponding motor parameter configuration according to the type of motor you use
MOTOR_TYPE = 5  #1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                #1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor
########################################################
# 将灰度巡线模块放置于赛道，观察正对着线的灯，如果灯亮，则数值为1，LINE_RAW_VALUE=1；如果灯灭，则数值为0，LINE_RAW_VALUE=0
# Place the grayscale line-following module on the track and observe the light facing the line. If the light is on, the value is 1, LINE_RAW_VALUE=1; if the light is off, the value is 0, LINE_RAW_VALUE=0
LINE_RAW_VALUE = 1
#########################################################



# 全局变量  Global variables
encoder_offset = [0] * 4
encoder_now = [0] * 4

# 电机驱动板类 Motor Driver Board Class
class MotorDriver:
    def __init__(self):
        """
        初始化电机驱动板
        Initialize the motor driver board
        """
        # 创建I2C通信对象   Create I2C communication object
        self.i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # 使用I2C1，SCL=GP7，SDA=GP6    
                                                                # Use I2C1, SCL=GP7, SDA=GP6
        # I2C地址   I2C Address
        self.MOTOR_MODEL_ADDR = 0x26

        # I2C寄存器定义 I2C Register Definition
        self.MOTOR_TYPE_REG = 0x01
        self.MOTOR_DEADZONE_REG = 0x02
        self.MOTOR_PLUSELINE_REG = 0x03
        self.MOTOR_PLUSEPHASE_REG = 0x04
        self.WHEEL_DIA_REG = 0x05
        self.SPEED_CONTROL_REG = 0x06
        self.PWM_CONTROL_REG = 0x07
        self.READ_TEN_M1_ENCODER_REG = 0x10
        self.READ_TEN_M2_ENCODER_REG = 0x11
        self.READ_TEN_M3_ENCODER_REG = 0x12
        self.READ_TEN_M4_ENCODER_REG = 0x13
        self.READ_ALLHIGH_M1_REG = 0x20
        self.READ_ALLLOW_M1_REG = 0x21
        self.READ_ALLHIGH_M2_REG = 0x22
        self.READ_ALLLOW_M2_REG = 0x23
        self.READ_ALLHIGH_M3_REG = 0x24
        self.READ_ALLLOW_M3_REG = 0x25
        self.READ_ALLHIGH_M4_REG = 0x26
        self.READ_ALLLOW_M4_REG = 0x27

    # 写数据到I2C寄存器 Write data to I2C register
    def i2c_write(self,addr, reg, data):
        self.i2c.writeto_mem(addr, reg, bytearray(data))

    # 从I2C寄存器读取数据   Reading data from I2C registers
    def i2c_read(self,addr, reg, length):
        return self.i2c.readfrom_mem(addr, reg, length)

    # 浮点数转字节  Floating point number to byte
    def float_to_bytes(self, f):
        return struct.pack('<f', f) # 小端字节序 Little-endian

    # 配置电机类型  Configure motor type
    def set_motor_type(self,data):
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.MOTOR_TYPE_REG, [data])

    # 配置死区  Configuring Dead Zone
    def set_motor_deadzone(self,data):
        buf = [(data >> 8) & 0xFF, data & 0xFF]
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.MOTOR_DEADZONE_REG, buf)

    # 配置磁环线    Configuring magnetic loop
    def set_pluse_line(self,data):
        buf = [(data >> 8) & 0xFF, data & 0xFF]
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.MOTOR_PLUSELINE_REG, buf)

    # 配置减速比    Configure the reduction ratio
    def set_pluse_phase(self,data):
        buf = [(data >> 8) & 0xFF, data & 0xFF]
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.MOTOR_PLUSEPHASE_REG, buf)

    # 配置轮子直径  Configuration Diameter
    def set_wheel_dis(self,data):
        bytes_data = self.float_to_bytes(data)
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.WHEEL_DIA_REG, list(bytes_data))

    # 控制速度  Controlling Speed
    def control_speed(self, m1, m2, m3, m4):
        speeds = [
            (m1 >> 8) & 0xFF, m1 & 0xFF,
            (m2 >> 8) & 0xFF, m2 & 0xFF,
            (m3 >> 8) & 0xFF, m3 & 0xFF,
            (m4 >> 8) & 0xFF, m4 & 0xFF
        ]
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.SPEED_CONTROL_REG, speeds)

    # 控制PWM(适用于无编码器的电机) Control PWM (for motors without encoder)
    def control_pwm(self, m1, m2, m3, m4):
        pwms = [
            (m1 >> 8) & 0xFF, m1 & 0xFF,
            (m2 >> 8) & 0xFF, m2 & 0xFF,
            (m3 >> 8) & 0xFF, m3 & 0xFF,
            (m4 >> 8) & 0xFF, m4 & 0xFF
        ]
        self.i2c_write(self.MOTOR_MODEL_ADDR, self.PWM_CONTROL_REG, pwms)

    # 读取编码器数据    Read encoder data
    def read_10_encoder(self):
        global encoder_offset
        formatted_values = []
        for i in range(4):
            reg = self.READ_TEN_M1_ENCODER_REG + i
            buf = self.i2c_read(self.MOTOR_MODEL_ADDR, reg, 2)
            encoder_offset[i] = (buf[0] << 8) | buf[1]
            if encoder_offset[i] & 0x8000:  # 检查最高位（符号位）是否为 1  Check if the highest bit (sign bit) is 1
                encoder_offset[i] -= 0x10000 # 将其转为负数 Turn it into a negative number
            formatted_values.append("M{}:{}".format(i + 1, encoder_offset[i]))
        return ", ".join(formatted_values)

    def read_all_encoder(self):
        global encoder_now
        formatted_values = []
        for i in range(4):
            high_reg = self.READ_ALLHIGH_M1_REG + (i * 2)
            low_reg = self.READ_ALLLOW_M1_REG + (i * 2)
            high_buf = self.i2c_read(self.MOTOR_MODEL_ADDR, high_reg, 2)
            low_buf = self.i2c_read(self.MOTOR_MODEL_ADDR, low_reg, 2)
            
            high_val = high_buf[0] <<8 | high_buf[1]
            low_val = low_buf[0] <<8 | low_buf[1]
            
            encoder_val = (high_val << 16) | low_val
            
            # 处理符号扩展，假设 32 位有符号整数    Handles sign extension, assuming 32-bit signed integers
            if encoder_val >= 0x80000000:  # 如果大于 2^31，说明应该是负数  If it is greater than 2^31, it should be a negative number
                encoder_val -= 0x100000000  # 将其转为负数  Turn it into a negative number
            encoder_now[i] = encoder_val
            formatted_values.append("M{}:{}".format(i + 1, encoder_now[i]))
        return ", ".join(formatted_values) 

    ##以下的参数根据自己的实际使用电机配置即可，只要配置一次即可，电机驱动板有断电保存功能
    ##The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
    def set_motor_parameter(self):
        if MOTOR_TYPE == 1:
            self.set_motor_type(1)  # 配置电机类型 / Configure motor type
            time.sleep(0.1)
            self.set_pluse_phase(30)  # 配置减速比，查电机手册得出 / Configure reduction ratio, refer to motor manual
            time.sleep(0.1)
            self.set_pluse_line(11)  # 配置磁环线，查电机手册得出 / Configure magnetic loop, refer to motor manual
            time.sleep(0.1)
            self.set_wheel_dis(67.00)  # 配置轮子直径，测量得出 / Configure wheel diameter, measured
            time.sleep(0.1)
            self.set_motor_deadzone(1600)  # 配置电机死区，实验得出 / Configure motor dead zone, experimentally obtained
            time.sleep(0.1)

        elif MOTOR_TYPE == 2:
            self.set_motor_type(2)
            time.sleep(0.1)
            self.set_pluse_phase(20)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(48.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1200)
            time.sleep(0.1)

        elif MOTOR_TYPE == 3:
            self.set_motor_type(3)
            time.sleep(0.1)
            self.set_pluse_phase(45)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(68.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1250)
            time.sleep(0.1)

        elif MOTOR_TYPE == 4:
            self.set_motor_type(4)
            time.sleep(0.1)
            self.set_pluse_phase(48)
            time.sleep(0.1)
            self.set_motor_deadzone(1000)
            time.sleep(0.1)

        elif MOTOR_TYPE == 5:
            self.set_motor_type(1)
            time.sleep(0.1)
            self.set_pluse_phase(40)
            time.sleep(0.1)
            self.set_pluse_line(11)
            time.sleep(0.1)
            self.set_wheel_dis(67.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)


# 灰度传感器类 Grayscale Sensor Class
class GrayscaleSensor:
    def __init__(self, ad0_pin=2, ad1_pin=3, ad2_pin=4, out_pin=5):
        """
        初始化灰度传感器
        Initialize the grayscale sensor
        """
        self.ad0 = Pin(ad0_pin, Pin.OUT)
        self.ad1 = Pin(ad1_pin, Pin.OUT)
        self.ad2 = Pin(ad2_pin, Pin.OUT)
        self.sensor_out = Pin(out_pin, Pin.IN)
        self.channels = 8
    
    def _select_channel(self, channel):
        """
        选择传感器通道
        Select the sensor channel
        """
        self.ad0.value((channel >> 0) & 0x01)
        self.ad1.value((channel >> 1) & 0x01)
        self.ad2.value((channel >> 2) & 0x01)
    
    def _read_out_value(self):
        """
        读取OUT引脚的值
        Read the value from the OUT pin
        """
        return self.sensor_out.value()
    
    def read_all(self):
        """
        读取所有通道
        Read all channels
        """
        values = []
        for i in range(self.channels):
            self._select_channel(i)
            time.sleep_us(50)
            values.append(self._read_out_value())
        return values
    
    def read_single(self, channel):
        """
        读取单个通道
        Read a single channel
        """
        if channel >= self.channels:
            return 0
        self._select_channel(channel)
        time.sleep_us(50)
        return self._read_out_value()
    
    def print_values(self):
        """
        打印所有传感器值
        Print all sensor values
        """
        values = self.read_all()
        for i, value in enumerate(values):
            print(f"[S{i+1}:{value}]", end=" ")
        print()
    
# 巡线类 Line Following Class
class LineFollowing:
    def __init__(self, motor, sensor):
        """
        初始化巡线控制器
        Initialize the line following controller
        """
        self.motor = motor        # 电机控制对象 Motor control object
        self.sensor = sensor      # 灰度传感器对象 Grayscale sensor object

        # PID参数 PID Parameters
        self.kp = 160.0    # 比例系数 Proportional coefficient - 提高响应速度
        self.ki = 0.5     # 积分系数 Integral coefficient - 增强稳定性
        self.kd = 20.0    # 微分系数 Derivative coefficient - 提高预判能力

        self.last_error = 0      # 上次偏差 Last error
        self.integral = 0        # 积分累加 Integral accumulation

        # 控制参数 Control Parameters
        self.base_speed = 330    # 基础速度 Base speed
        self.max_speed = 400     # 最大速度 Max speed

        # 传感器权重 Sensor weights (8个传感器的位置权重)
        self.sensor_weights = [-5.0, -4.0, -2.0, -1.0, 1.0, 2.0, 4.0, 5.0]

        # 安全锁标志位 Safety lock flag
        self.motor_locked = True    # 初始锁定电机，防止乱跑 Initially lock motors to prevent random movement

    def check_sensors_safe(self, sensor_values):
        """
        上电安全锁检查，当传感器全亮或全灭时不启动小车，防止乱跑
        Power-on safety lock check, do not start the car when the sensors are all on or all off to prevent random movement
        """
        first_value = sensor_values[0]
        for value in sensor_values:
            if value != first_value:
                return True
        return False

    def calculate_error(self, sensor_values):
        """
        计算偏差值 Calculate the deviation value
        """
        weighted_sum = 0
        active_sensors = 0

        for i in range(8):
            if sensor_values[i] == LINE_RAW_VALUE:
                weighted_sum += self.sensor_weights[i]
                active_sensors += 1

        # 如果没有检测到线，返回上次偏差（丢线处理）
        # If no line is detected, return the last deviation (line loss handling)
        if active_sensors == 0:
            return self.last_error

        # 计算加权平均偏差
        # Calculate weighted average deviation
        error = weighted_sum / active_sensors
        return error

    def pid_control(self, error):
        """
        PID控制计算 PID control calculation
        """
        # 添加死区，忽略微小偏差 / Add dead zone to ignore small deviations
        if abs(error) < 0.6:
            error = 0
        # 检测过零点，清零积分 / Detect zero crossing, clear integral
        if (self.last_error > 0 and error < 0) or (self.last_error < 0 and error > 0):
            self.integral = 0  # 偏差过零，清空积分/ Clear integral when deviation crosses zero

        # 动态调整积分限幅 / Dynamically adjust integral limit
        if abs(error) > 3.0:  # 大偏差时 / Large deviation
            integral_limit = 80
        elif abs(error) > 1.5:  # 中等偏差时 / Medium deviation
            integral_limit = 50
        else:  # 小偏差时 / Small deviation
            integral_limit = 20

        # 积分项，使用动态限幅 / Integral term with dynamic limit
        self.integral += error
        self.integral = max(-integral_limit, min(integral_limit, self.integral))

        # 微分项 / Derivative term
        derivative = error - self.last_error

        # PID计算 / PID calculation
        output = (self.kp * error +
                 self.ki * self.integral +
                 self.kd * derivative)

        # 更新上次偏差 / Update last error
        self.last_error = error

        return output

    def differential_speed_control(self, pid_output):
        """
        差速控制 Differential speed control
        """
        left_speed = self.base_speed + pid_output
        right_speed = self.base_speed - pid_output

        # 速度限制 / Speed limit
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        return left_speed, right_speed

    def follow_line(self):
        """
        巡线主函数 Main line following function
        """
        # 读取传感器值 / Read sensor values
        sensor_values = self.sensor.read_all()

        # 检查安全锁 / Check safety lock
        if self.motor_locked:
            if self.check_sensors_safe(sensor_values):
                self.motor_locked = False  # 解锁 / Unlock
            else:
                self.motor.control_pwm(0, 0, 0, 0)  # 确保电机停止 / Ensure motors are stopped
                return

        # 计算偏差 / Calculate deviation
        error = self.calculate_error(sensor_values)

        # PID控制计算 / PID control calculation
        pid_output = self.pid_control(error)

        # 差速控制 / Differential speed control
        left_speed, right_speed = self.differential_speed_control(pid_output)

        # 控制电机 (M1,M2为左轮，M3,M4为右轮)
        # Control motors (M1,M2 are left wheels, M3,M4 are right wheels)
        if(MOTOR_TYPE in [1, 2, 3,  5]):
            self.motor.control_speed(int(left_speed), int(left_speed),
                                    int(right_speed), int(right_speed))
        elif(MOTOR_TYPE == 4):
            self.motor.control_pwm(int(left_speed), int(right_speed),
                                    int(left_speed), int(right_speed))


def main() -> None:
    try:
        # 启动板载LED / Turn on onboard LED
        led = Pin(25, Pin.OUT)

        # 启动指示灯闪烁3次，确认主板上电并运行了程序
        # Blink the startup indicator light 3 times to confirm that the motherboard is powered on and running the program
        for i in range(3):
            led.on()
            time.sleep(0.2)
            led.off()
            time.sleep(0.2)

        # 初始化硬件 Initialize hardware
        sensor = GrayscaleSensor()
        motor = MotorDriver()

        # 配置电机参数 Configure motor parameters
        motor.set_motor_parameter()
        time.sleep(1)  # 等待电机初始化完成 Wait for motor initialization

        # 创建巡线控制器 Create line following controller
        line_follower = LineFollowing(motor, sensor)

        # 硬件初始化完成，LED常亮 / Hardware initialization is complete, LED is always on
        led.on()

        while True:
            # 执行巡线 Execute line following
            line_follower.follow_line()

            # 控制循环频率 Control loop frequency
            time.sleep_ms(50)  # 20Hz control frequency

    except KeyboardInterrupt:
        motor.control_pwm(0, 0, 0, 0)
        if 'led' in locals():
            led.off()
    except Exception as e:
        motor.control_pwm(0, 0, 0, 0)
        if 'led' in locals():
            led.off()

if __name__ == "__main__":
    main()
