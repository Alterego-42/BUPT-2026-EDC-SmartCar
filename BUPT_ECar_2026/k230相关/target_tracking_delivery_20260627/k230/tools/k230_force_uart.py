import os
import time

from machine import FPIOA, UART


UART_ID = UART.UART2
UART_TX_PIN = 5
UART_RX_PIN = 6
UART_BAUD = 115200

SEND_HZ = 50
PHASE_MS = 1800
FORCE_YAW = 1.50
FORCE_PITCH = 1.50


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


def run_phase(uart, name, yaw, pitch, duration_ms):
    print("$FORCE_PHASE,%s,yaw=%.2f,pitch=%.2f" % (name, yaw, pitch))
    start = time.ticks_ms()
    last = 0
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        os.exitpoint()
        now = time.ticks_ms()
        if time.ticks_diff(now, last) >= int(1000 / SEND_HZ):
            line = send_packet(uart, 1, yaw, pitch)
            print("$TX,%s" % line)
            last = now
        time.sleep_ms(1)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    uart = setup_uart()
    try:
        run_phase(uart, "yaw_pos", FORCE_YAW, 0.0, PHASE_MS)
        run_phase(uart, "yaw_neg", -FORCE_YAW, 0.0, PHASE_MS)
        run_phase(uart, "pitch_pos", 0.0, FORCE_PITCH, PHASE_MS)
        run_phase(uart, "pitch_neg", 0.0, -FORCE_PITCH, PHASE_MS)
    finally:
        for _ in range(8):
            print("$TX,%s" % send_packet(uart, 0, 0.0, 0.0))
            time.sleep_ms(20)
        print("$FORCE_DONE")


main()
