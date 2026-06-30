"""K230 standalone boot entry.

Copy this file to /sdcard/main.py together with k230_target_uart_v72.py and
k230_target_config_v72.py.  On power-up the K230 runs the tracker continuously,
while MSPM0 decides by mode whether to consume the UART packets.
"""

import gc
import time
import machine


RESTART_DELAY_MS = 1000


def run_tracker_once():
    try:
        import k230_target_uart_v72 as tracker
        tracker.DIAGNOSE_ONCE = False
        tracker.DEFAULT_RUN_SECONDS = None
        tracker.FORCE_COMMAND_ENABLED = False
        tracker.main(None)
    finally:
        gc.collect()


while True:
    try:
        run_tracker_once()
        break
    except KeyboardInterrupt:
        break
    except Exception as exc:
        try:
            print("$BOOT,reset_after_error=%s" % exc)
        except Exception:
            pass
        gc.collect()
        time.sleep_ms(RESTART_DELAY_MS)
        machine.reset()
