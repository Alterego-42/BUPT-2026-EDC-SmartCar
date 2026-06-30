import os
import time

from machine import FPIOA, UART


UART_ID = UART.UART2
UART_TX_PIN = 5
UART_RX_PIN = 6
UART_BAUD = 115200

SEND_HZ = 15
YAW = 0.0
PITCH = 0.0
SECONDS = 0.5
USB_PRINT_ENABLED = False


def setup_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    return UART(
        UART_ID,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def send_packet(uart, valid, yaw, pitch):
    line = "$K230,%d,%.2f,%.2f\n" % (valid, yaw, pitch)
    uart.write(line)
    return line.strip()


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    uart = setup_uart()
    start = time.ticks_ms()
    last = 0
    duration_ms = int(SECONDS * 1000)
    try:
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            os.exitpoint()
            now = time.ticks_ms()
            if time.ticks_diff(now, last) >= int(1000 / SEND_HZ):
                line = send_packet(uart, 1, YAW, PITCH)
                if USB_PRINT_ENABLED:
                    print("$TX,%s" % line)
                last = now
            time.sleep_ms(1)
    finally:
        for _ in range(6):
            line = send_packet(uart, 0, 0.0, 0.0)
            if USB_PRINT_ENABLED:
                print("$TX,%s" % line)
            time.sleep_ms(20)


main()
