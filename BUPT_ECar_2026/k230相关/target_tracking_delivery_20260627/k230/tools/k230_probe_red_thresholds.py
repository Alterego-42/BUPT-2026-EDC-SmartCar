import gc
import time

from media.sensor import *
from media.media import *


FRAME_W = ALIGN_UP(320, 16)
FRAME_H = 240
CAPTURE_CHN = CAM_CHN_ID_2

THRESHOLDS = (
    ("broad_a8", (0, 100, 8, 127, -80, 127)),
    ("a18_ball", (0, 100, 18, 127, -80, 127)),
    ("a18_b60", (0, 100, 18, 127, -80, 60)),
    ("a18_b40", (0, 100, 18, 127, -80, 40)),
    ("a18_b20", (0, 100, 18, 127, -80, 20)),
    ("a25_b60", (0, 100, 25, 127, -80, 60)),
    ("a25_b40", (0, 100, 25, 127, -80, 40)),
    ("a25_b20", (0, 100, 25, 127, -80, 20)),
    ("a35_b60", (0, 100, 35, 127, -80, 60)),
    ("a35_b40", (0, 100, 35, 127, -80, 40)),
)


def value(obj, index, fallback=0):
    try:
        return obj[index]
    except Exception:
        return fallback


def center(blob):
    try:
        return float(blob.cx()), float(blob.cy())
    except Exception:
        return (
            float(value(blob, 0) + value(blob, 2) / 2.0),
            float(value(blob, 1) + value(blob, 3) / 2.0),
        )


def print_blobs(img, label, threshold):
    try:
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=2,
            area_threshold=2,
            merge=False,
            margin=0,
        )
    except Exception as exc:
        print("$REDPROBE_FAIL,%s,%s" % (label, exc))
        return

    items = []
    total_pixels = 0
    for b in blobs:
        x = int(value(b, 0))
        y = int(value(b, 1))
        w = int(value(b, 2))
        h = int(value(b, 3))
        pix = int(value(b, 4, w * h))
        area = w * h
        cx, cy = center(b)
        density = (pix * 100.0 / area) if area else 0.0
        total_pixels += pix
        items.append((pix, area, x, y, w, h, cx, cy, density))

    items = sorted(items, key=lambda item: (-item[0], -item[1], item[2], item[3]))
    if items:
        x0 = min([item[2] for item in items])
        y0 = min([item[3] for item in items])
        x1 = max([item[2] + item[4] for item in items])
        y1 = max([item[3] + item[5] for item in items])
        print(
            "$REDPROBE,%s,count=%d,total_pix=%d,union=x%d,y%d,w%d,h%d,cx%.1f,cy%.1f"
            % (
                label,
                len(items),
                total_pixels,
                x0,
                y0,
                x1 - x0,
                y1 - y0,
                x0 + (x1 - x0) / 2.0,
                y0 + (y1 - y0) / 2.0,
            )
        )
    else:
        print("$REDPROBE,%s,count=0,total_pix=0" % label)

    for item in items[:8]:
        pix, area, x, y, w, h, cx, cy, density = item
        print(
            "$RB,%s,pix=%d,area=%d,x=%d,y=%d,w=%d,h=%d,cx=%.1f,cy=%.1f,dens=%.1f"
            % (label, pix, area, x, y, w, h, cx, cy, density)
        )


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
        for _ in range(25):
            sensor.snapshot(chn=CAPTURE_CHN)
            time.sleep_ms(10)

        img = sensor.snapshot(chn=CAPTURE_CHN)
        print("$REDPROBE,size=%dx%d" % (FRAME_W, FRAME_H))
        for label, threshold in THRESHOLDS:
            print_blobs(img, label, threshold)

        img = None
        gc.collect()
    finally:
        try:
            if sensor:
                sensor.stop()
        except Exception as exc:
            print("$REDPROBE_STOP_FAIL,%s" % exc)
        try:
            MediaManager.deinit()
        except Exception as exc:
            print("$REDPROBE_MEDIA_DEINIT_FAIL,%s" % exc)


main()
