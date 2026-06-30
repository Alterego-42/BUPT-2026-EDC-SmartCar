import gc
import time

from media.sensor import *
from media.media import *


FRAME_W = ALIGN_UP(320, 16)
FRAME_H = 240
CAPTURE_CHN = CAM_CHN_ID_2


def value(obj, index, fallback=0):
    try:
        return obj[index]
    except Exception:
        return fallback


def center(blob):
    try:
        return float(blob.cx()), float(blob.cy())
    except Exception:
        return float(value(blob, 0) + value(blob, 2) / 2.0), float(value(blob, 1) + value(blob, 3) / 2.0)


def main():
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
        for _ in range(15):
            sensor.snapshot(chn=CAPTURE_CHN)
            time.sleep_ms(10)

        img = sensor.snapshot(chn=CAPTURE_CHN)
        print("$PROBE,size=%dx%d" % (FRAME_W, FRAME_H))
        for lmax in (20, 30, 40, 50, 60, 70, 85):
            try:
                blobs = img.find_blobs(
                    [(0, lmax, -128, 127, -128, 127)],
                    pixels_threshold=1,
                    area_threshold=1,
                    merge=False,
                    margin=0,
                )
            except Exception as exc:
                print("$PROBE_FAIL,lmax=%d,%s" % (lmax, exc))
                continue

            small = []
            for b in blobs:
                x = int(value(b, 0))
                y = int(value(b, 1))
                w = int(value(b, 2))
                h = int(value(b, 3))
                pixels = int(value(b, 4, w * h))
                area = w * h
                if 1 <= pixels <= 500 and 1 <= area <= 600 and w <= 35 and h <= 35:
                    cx, cy = center(b)
                    small.append((pixels, area, x, y, w, h, cx, cy))

            small = sorted(small, key=lambda item: (-item[0], item[2], item[3]))[:12]
            print("$PROBE,lmax=%d,count=%d,small=%d" % (lmax, len(blobs), len(small)))
            for item in small:
                pixels, area, x, y, w, h, cx, cy = item
                print("$B,lmax=%d,pix=%d,area=%d,x=%d,y=%d,w=%d,h=%d,cx=%.1f,cy=%.1f" % (
                    lmax,
                    pixels,
                    area,
                    x,
                    y,
                    w,
                    h,
                    cx,
                    cy,
                ))

        img = None
        gc.collect()
    finally:
        try:
            if sensor:
                sensor.stop()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


main()
