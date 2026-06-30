import gc
import time

from media.sensor import *
from media.media import *


FRAME_W = ALIGN_UP(320, 16)
FRAME_H = 240
CAPTURE_CHN = CAM_CHN_ID_2

PAPER_THRESHOLDS = (
    ("paper_40", (40, 100, -50, 50, -65, 65)),
    ("paper_30", (30, 100, -60, 60, -75, 75)),
    ("paper_20", (20, 100, -70, 70, -85, 85)),
)
SQUARE_THRESHOLDS = (
    ("black_20", (0, 20, -128, 127, -128, 127)),
    ("black_28", (0, 28, -128, 127, -128, 127)),
    ("black_35", (0, 35, -128, 127, -128, 127)),
    ("black_50", (0, 50, -128, 127, -128, 127)),
    ("black_65", (0, 65, -128, 127, -128, 127)),
    ("black_85", (0, 85, -128, 127, -128, 127)),
)
LASER_THRESHOLD = ("laser", (12, 100, 30, 127, -20, 127))


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


def print_blobs(img, label, threshold, pixels_threshold, area_threshold, merge, margin, limit=8):
    try:
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=pixels_threshold,
            area_threshold=area_threshold,
            merge=merge,
            margin=margin,
        )
    except Exception as exc:
        print("$SCENE_FAIL,%s,%s" % (label, exc))
        return

    items = []
    for b in blobs:
        x = int(value(b, 0))
        y = int(value(b, 1))
        w = int(value(b, 2))
        h = int(value(b, 3))
        pix = int(value(b, 4, w * h))
        area = w * h
        cx, cy = center(b)
        density = (pix * 100.0 / area) if area else 0.0
        items.append((pix, area, x, y, w, h, cx, cy, density))

    items = sorted(items, key=lambda item: (-item[0], -item[1], item[2], item[3]))
    print("$SCENE,%s,count=%d" % (label, len(items)))
    for item in items[:limit]:
        pix, area, x, y, w, h, cx, cy, density = item
        print(
            "$B,%s,pix=%d,area=%d,x=%d,y=%d,w=%d,h=%d,cx=%.1f,cy=%.1f,dens=%.1f"
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
        print("$SCENE,size=%dx%d" % (FRAME_W, FRAME_H))

        for label, threshold in PAPER_THRESHOLDS:
            print_blobs(img, label, threshold, 300, 300, True, 8, 6)

        for label, threshold in SQUARE_THRESHOLDS:
            print_blobs(img, label + "_m8", threshold, 10, 10, True, 8, 10)
            print_blobs(img, label + "_m45", threshold, 10, 10, True, 45, 6)

        label, threshold = LASER_THRESHOLD
        print_blobs(img, label, threshold, 1, 1, True, 2, 8)

        img = None
        gc.collect()
    finally:
        try:
            if sensor:
                sensor.stop()
        except Exception as exc:
            print("$SCENE_STOP_FAIL,%s" % exc)
        try:
            MediaManager.deinit()
        except Exception as exc:
            print("$SCENE_MEDIA_DEINIT_FAIL,%s" % exc)


main()
