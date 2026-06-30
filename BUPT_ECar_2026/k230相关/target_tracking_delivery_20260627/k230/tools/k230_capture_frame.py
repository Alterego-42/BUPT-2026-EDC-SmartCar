import gc
import os
import time

from media.sensor import *
from media.media import *


FRAME_W = ALIGN_UP(320, 16)
FRAME_H = 240
CAPTURE_CHN = CAM_CHN_ID_2
CAMERA_START_RETRIES = 3
CAMERA_WARMUP_FRAMES = 15
CAMERA_WARMUP_MAX_TRIES = 80
CAMERA_RETRY_DELAY_MS = 120


def setup_camera():
    last_error = None
    for attempt in range(CAMERA_START_RETRIES):
        sensor = None
        try:
            sensor = Sensor()
            sensor.reset()
            sensor.set_framesize(width=800, height=480)
            sensor.set_pixformat(Sensor.YUV420SP)
            sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAPTURE_CHN)
            sensor.set_pixformat(Sensor.RGB565, chn=CAPTURE_CHN)
            MediaManager.init()
            sensor.run()
            time.sleep_ms(CAMERA_RETRY_DELAY_MS)

            good_frames = 0
            tries = 0
            while good_frames < CAMERA_WARMUP_FRAMES and tries < CAMERA_WARMUP_MAX_TRIES:
                tries += 1
                try:
                    sensor.snapshot(chn=CAPTURE_CHN)
                    good_frames += 1
                    time.sleep_ms(10)
                except Exception as exc:
                    last_error = exc
                    good_frames = 0
                    time.sleep_ms(CAMERA_RETRY_DELAY_MS)

            if good_frames >= CAMERA_WARMUP_FRAMES:
                return sensor

            last_error = "warmup timeout"
        except Exception as exc:
            last_error = exc

        try:
            if sensor:
                sensor.stop()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        time.sleep_ms(CAMERA_RETRY_DELAY_MS * (attempt + 1))

    raise RuntimeError("camera setup failed: %s" % last_error)


def snapshot_retry(sensor, tries):
    last_error = None
    for _ in range(tries):
        try:
            return sensor.snapshot(chn=CAPTURE_CHN)
        except Exception as exc:
            last_error = exc
            time.sleep_ms(CAMERA_RETRY_DELAY_MS)
    raise RuntimeError("capture snapshot failed: %s" % last_error)


def main():
    sensor = None
    saved = False
    try:
        sensor = setup_camera()
        img = snapshot_retry(sensor, 10)
        print("$CAPTURE,size=%dx%d" % (FRAME_W, FRAME_H))
        for path in (
            "/sdcard/k230_capture.jpg",
            "/data/k230_capture.jpg",
            "/flash/k230_capture.jpg",
            "k230_capture.jpg",
        ):
            try:
                img.save(path)
                print("$CAPTURE_SAVED,%s" % path)
                saved = True
                break
            except Exception as exc:
                print("$CAPTURE_SAVE_FAIL,%s,%s" % (path, exc))

        if not saved:
            print("$CAPTURE_IMG_DIR,%s" % ",".join(dir(img)))

        img = None
        gc.collect()
    finally:
        try:
            if sensor:
                sensor.stop()
        except Exception as exc:
            print("$CAPTURE_STOP_FAIL,%s" % exc)
        try:
            MediaManager.deinit()
        except Exception as exc:
            print("$CAPTURE_MEDIA_DEINIT_FAIL,%s" % exc)


main()
