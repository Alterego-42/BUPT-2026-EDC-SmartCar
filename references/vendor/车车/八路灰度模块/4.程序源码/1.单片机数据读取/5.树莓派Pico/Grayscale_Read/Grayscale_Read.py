#Grayscale Sensor / PICO2
#       5V          VBUS
#       GND         GND
#       AD0         GP2
#       AD1         GP3
#       AD2         GP4
#       OUT         GP5
from machine import Pin
import time
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
    
def main() -> None:
    try:
        sensor = GrayscaleSensor()
        while True:
            sensor.print_values()
            time.sleep_ms(60)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
