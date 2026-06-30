import gc
import math
import os
import time

from machine import FPIOA, UART
from media.sensor import *
from media.media import *


# CanMV-K230 V3.0 40-pin header:
# physical pin 17 = UART2_TXD(IO5), physical pin 20 = UART2_RXD(IO6).
UART_ID = UART.UART2
UART_TX_PIN = 5
UART_RX_PIN = 6
UART_BAUD = 115200

FRAME_W = ALIGN_UP(320, 16)
FRAME_H = 240
CAPTURE_CHN = CAM_CHN_ID_2

HFOV_DEG = 62.0
VFOV_DEG = 48.0

# Positive error means the laser dot is right/below the square center.
ERROR_YAW_SIGN = 1.0
ERROR_PITCH_SIGN = 1.0

CONTROL_MODE = "center_dot_laser_closed_loop"
SCRIPT_VERSION = "center_dot_laser_v22_dot_first"
LOST_SEND_HZ = 12
CONTROL_SEND_HZ = 50
DEBUG_PRINT_HZ = 8
TARGET_CANDIDATE_DEBUG = False
TARGET_CANDIDATE_DEBUG_HZ = 2
USB_DEBUG_ENABLED = False
STM32_RX_DEBUG = False
STM32_RX_MAX_BUFFER = 256
CONTROL_OUTPUT_ENABLED = True
DEFAULT_RUN_SECONDS = None
FORCE_COMMAND_ENABLED = False
FORCE_YAW = 0.0
FORCE_PITCH = 1.00

CAMERA_START_RETRIES = 3
CAMERA_WARMUP_FRAMES = 15
CAMERA_WARMUP_MAX_TRIES = 80
CAMERA_RETRY_DELAY_MS = 120

# White paper gate. When enabled, target and laser candidates must be inside
# the detected white paper bounding box.
PAPER_ROI_ENABLED = True
PAPER_THRESHOLD = (40, 100, -50, 50, -65, 65)
PAPER_MIN_PIXELS = 1200
PAPER_MIN_AREA = 1800
PAPER_MIN_W = 70
PAPER_MIN_H = 70
PAPER_MAX_AREA = FRAME_W * FRAME_H * 1.01
PAPER_MIN_DENSITY = 8
PAPER_FULLSCREEN_MIN_DENSITY = 70
PAPER_MERGE_MARGIN = 6
PAPER_INSET_PX = 2

# Dark, thick square frame. The coarse finder uses a lower L bound and a small
# merge margin to keep separate frame segments from swallowing the scene.
SQUARE_THRESHOLD = (0, 28, -128, 127, -128, 127)
SQUARE_RECT_THRESHOLD = 8000
SQUARE_EDGE_MARGIN = 0
SQUARE_MIN_W = 55
SQUARE_MIN_H = 55
SQUARE_MAX_W = FRAME_W - 6
SQUARE_MAX_H = FRAME_H - 6
SQUARE_MIN_AREA = 2500
SQUARE_MAX_AREA = FRAME_W * FRAME_H * 0.85
SQUARE_MIN_PIXELS = 80
SQUARE_MERGE_MARGIN = 45
SQUARE_MIN_ASPECT = 0.25
SQUARE_MAX_ASPECT = 4.00
SQUARE_MIN_DENSITY = 1
SQUARE_MAX_DENSITY = 85
SQUARE_UNION_MIN_BLOBS = 2
SQUARE_UNION_MIN_PIXELS = 180
SQUARE_PART_MIN_PIXELS = 18
SQUARE_PART_MIN_AREA = 25
SQUARE_PART_MAX_AREA = FRAME_W * FRAME_H * 0.25

# If the target has a black center mark, use it as the true aim point.
CENTER_DOT_THRESHOLDS = (
    (0, 24, -35, 25, -25, 12),
    (0, 32, -35, 25, -25, 12),
    (0, 40, -35, 25, -25, 12),
    (0, 48, -35, 25, -25, 12),
)
CENTER_DOT_THRESHOLD = CENTER_DOT_THRESHOLDS[-1]
CENTER_DOT_MIN_PIXELS = 2
CENTER_DOT_MIN_AREA = 2
CENTER_DOT_MIN_W = 2
CENTER_DOT_MIN_H = 2
CENTER_DOT_MAX_W = 25
CENTER_DOT_MAX_H = 25
CENTER_DOT_MAX_AREA = 500
CENTER_DOT_INNER_RATIO = 0.08
CENTER_DOT_MIN_DENSITY = 3
CENTER_DOT_MAX_ASPECT = 6.00
CENTER_DOT_MIN_SCORE = 2500
CENTER_DOT_MAX_LASER_DIST_PX = 180

# Dot-first acquisition. This path is deliberately stricter than the framed
# dot search so it can run even when the black frame is not detected.
CENTER_DOT_GLOBAL_ENABLED = True
CENTER_DOT_GLOBAL_MIN_PIXELS = 6
CENTER_DOT_GLOBAL_MIN_AREA = 4
CENTER_DOT_GLOBAL_MIN_W = 3
CENTER_DOT_GLOBAL_MIN_H = 3
CENTER_DOT_GLOBAL_MAX_W = 32
CENTER_DOT_GLOBAL_MAX_H = 32
CENTER_DOT_GLOBAL_MAX_AREA = 800
CENTER_DOT_GLOBAL_MIN_DENSITY = 5
CENTER_DOT_GLOBAL_MAX_ASPECT = 1.55
CENTER_DOT_GLOBAL_EDGE_MARGIN_PX = 10
CENTER_DOT_GLOBAL_PAPER_MARGIN_RATIO = 0.03
CENTER_DOT_GLOBAL_MIN_SCORE = 1350
CENTER_DOT_GLOBAL_FRAMELESS_MIN_SCORE = 3000
CENTER_DOT_GLOBAL_FRAMELESS_MAX_DIST_PX = 100
CENTER_DOT_GLOBAL_ISOLATION_MARGIN = 8
CENTER_DOT_GLOBAL_MERGED_MAX_W = 44
CENTER_DOT_GLOBAL_MERGED_MAX_H = 44
CENTER_DOT_GLOBAL_MERGED_MAX_AREA = 1500

CENTER_DOT_FALLBACK_ENABLED = False
CENTER_DOT_FALLBACK_MIN_PIXELS = 6
CENTER_DOT_FALLBACK_MIN_AREA = 8
CENTER_DOT_FALLBACK_MIN_W = 3
CENTER_DOT_FALLBACK_MIN_H = 3
CENTER_DOT_FALLBACK_MAX_W = 42
CENTER_DOT_FALLBACK_MAX_H = 42
CENTER_DOT_FALLBACK_MAX_AREA = 1500
CENTER_DOT_FALLBACK_MIN_DENSITY = 3
CENTER_DOT_FALLBACK_MAX_ASPECT = 6.00
CENTER_DOT_FALLBACK_MERGE_MARGIN = 8
CENTER_DOT_FALLBACK_MARGIN_RATIO = 0.08
CENTER_DOT_FALLBACK_PAPER_RELAX_PX = 70
CENTER_DOT_FALLBACK_MAX_CENTER_DIST_PX = 150
CENTER_DOT_FALLBACK_MIN_SCORE = 500

# Red laser dot.
LASER_THRESHOLD = (12, 100, 30, 127, -20, 127)
LASER_MIN_PIXELS = 2
LASER_MIN_AREA = 2
LASER_MIN_W = 1
LASER_MIN_H = 1
LASER_MAX_AREA = 220
LASER_MAX_W = 16
LASER_MAX_H = 16
LASER_HINT_MAX_DIST_PX = 70
LASER_REACQUIRE_MAX_DIST_PX = 180
LASER_GLOBAL_FALLBACK_ENABLED = True
LASER_GLOBAL_REACQUIRE_MAX_DIST_PX = 260

# Filtering: keep target center stable, keep laser responsive.
TARGET_FILTER_WINDOW = 3
TARGET_FILTER_ALPHA = 0.60
TARGET_FILTER_DEADBAND_PX = 0.8
TARGET_WARMUP = 3

LASER_FILTER_WINDOW = 3
LASER_FILTER_ALPHA = 0.85
LASER_FILTER_DEADBAND_PX = 0.4
LASER_WARMUP = 3

ERROR_DEADBAND_PX = 2.0
CONTROL_GAIN = 0.75
CONTROL_MAX_DEG = 3.00
COARSE_CONTROL_GAIN = 0.45
COARSE_CONTROL_MAX_DEG = 1.50
MAX_TARGET_JUMP_PX = 65
MAX_LASER_JUMP_PX = 80
LASER_HOLD_MISSES = 8
TARGET_STABLE_SPREAD_PX = 30
LASER_STABLE_SPREAD_PX = 45
TARGET_JUMP_RELOCK_COUNT = 4

# Target acquisition is dot-first. The black frame is only an auxiliary ROI or
# a coarse fallback when the center dot is not reliable.
FRAME_COARSE_ENABLED = True
FRAME_COARSE_THRESHOLD = (0, 28, -128, 127, -128, 127)
FRAME_COARSE_MERGE_MARGIN = 8
FRAME_COARSE_MIN_PIXELS = 45
FRAME_COARSE_MIN_AREA = 80
FRAME_COARSE_MIN_W = 10
FRAME_COARSE_MIN_H = 6
FRAME_COARSE_MAX_AREA = FRAME_W * FRAME_H * 0.35
FRAME_COARSE_MIN_DENSITY = 4
FRAME_COARSE_MAX_DENSITY = 98
FRAME_COARSE_MIN_ASPECT = 0.18
FRAME_COARSE_MAX_ASPECT = 8.0

fallback_debug_last_ms = 0


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


def value(obj, index, fallback=0):
    try:
        return obj[index]
    except Exception:
        return fallback


def blob_center(blob):
    try:
        return float(blob.cx()), float(blob.cy())
    except Exception:
        x = value(blob, 0)
        y = value(blob, 1)
        w = value(blob, 2)
        h = value(blob, 3)
        return float(x + w / 2.0), float(y + h / 2.0)


def median(values):
    values = sorted(values)
    count = len(values)
    if count == 0:
        return 0.0
    mid = count // 2
    if count & 1:
        return float(values[mid])
    return float(values[mid - 1] + values[mid]) / 2.0


def append_sample(samples, x, y, max_count):
    samples.append((float(x), float(y)))
    if len(samples) > max_count:
        samples.pop(0)


def filtered_point(samples, x, y, prev_x, prev_y, max_count, alpha, deadband):
    append_sample(samples, x, y, max_count)
    med_x = median([s[0] for s in samples])
    med_y = median([s[1] for s in samples])

    if prev_x is None:
        return med_x, med_y, med_x, med_y

    filt_x = prev_x
    filt_y = prev_y
    if abs(med_x - prev_x) > deadband:
        filt_x = prev_x + (med_x - prev_x) * alpha
    if abs(med_y - prev_y) > deadband:
        filt_y = prev_y + (med_y - prev_y) * alpha
    return med_x, med_y, filt_x, filt_y


def samples_spread(samples):
    if not samples:
        return 9999.0
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    return max(max(xs) - min(xs), max(ys) - min(ys))


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def rate_due(now_ms, last_ms, hz):
    if hz <= 0:
        return True
    return time.ticks_diff(now_ms, last_ms) >= int(1000 / hz)


def paper_rect(paper):
    if not paper:
        return None
    _src, x, y, w, h, _pixels, _score = paper
    return float(x), float(y), float(w), float(h)


def paper_bounds(paper):
    rect = paper_rect(paper)
    if not rect:
        return 0.0, 0.0, float(FRAME_W), float(FRAME_H)
    x, y, w, h = rect
    return x, y, x + w, y + h


def point_in_paper(cx, cy, paper, margin=0.0):
    if not PAPER_ROI_ENABLED or not paper:
        return True
    x0, y0, x1, y1 = paper_bounds(paper)
    return cx >= x0 + margin and cx <= x1 - margin and cy >= y0 + margin and cy <= y1 - margin


def box_in_paper(x, y, w, h, paper, margin=0.0):
    if not PAPER_ROI_ENABLED or not paper:
        return True
    x0, y0, x1, y1 = paper_bounds(paper)
    return (
        x >= x0 + margin
        and y >= y0 + margin
        and x + w <= x1 - margin
        and y + h <= y1 - margin
    )


def clamp_bounds_to_paper(x0, y0, x1, y1, paper):
    if PAPER_ROI_ENABLED and paper:
        px0, py0, px1, py1 = paper_bounds(paper)
        x0 = max(x0, px0)
        y0 = max(y0, py0)
        x1 = min(x1, px1)
        y1 = min(y1, py1)
    else:
        x0 = max(0.0, x0)
        y0 = max(0.0, y0)
        x1 = min(float(FRAME_W), x1)
        y1 = min(float(FRAME_H), y1)
    return x0, y0, x1, y1


def rect_bounds_from_corners(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs)
    y0 = min(ys)
    x1 = max(xs)
    y1 = max(ys)
    return x0, y0, x1 - x0, y1 - y0


def polygon_area(corners):
    area = 0.0
    count = len(corners)
    for i in range(count):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % count]
        area += x1 * y2 - x2 * y1
    if area < 0:
        area = -area
    return area / 2.0


def rect_corners(rect):
    try:
        corners = rect.corners()
        if corners and len(corners) >= 4:
            return [(float(p[0]), float(p[1])) for p in corners[:4]]
    except Exception:
        pass

    try:
        x = float(rect.x())
        y = float(rect.y())
        w = float(rect.w())
        h = float(rect.h())
    except Exception:
        try:
            r = rect.rect()
            x = float(r[0])
            y = float(r[1])
            w = float(r[2])
            h = float(r[3])
        except Exception:
            x = float(value(rect, 0))
            y = float(value(rect, 1))
            w = float(value(rect, 2))
            h = float(value(rect, 3))

    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]


def valid_square_box(x, y, w, h, area):
    if w < SQUARE_MIN_W or h < SQUARE_MIN_H:
        return False
    if w > SQUARE_MAX_W or h > SQUARE_MAX_H:
        return False
    if area < SQUARE_MIN_AREA or area > SQUARE_MAX_AREA:
        return False
    aspect = w / h if h else 0.0
    if aspect < SQUARE_MIN_ASPECT or aspect > SQUARE_MAX_ASPECT:
        return False
    if x < SQUARE_EDGE_MARGIN or y < SQUARE_EDGE_MARGIN:
        return False
    if x + w > FRAME_W - SQUARE_EDGE_MARGIN:
        return False
    if y + h > FRAME_H - SQUARE_EDGE_MARGIN:
        return False
    return True


def find_paper(img, laser_hint=None):
    if not PAPER_ROI_ENABLED:
        return ("paper", 0.0, 0.0, float(FRAME_W), float(FRAME_H), FRAME_W * FRAME_H, 1)

    try:
        blobs = img.find_blobs(
            [PAPER_THRESHOLD],
            pixels_threshold=PAPER_MIN_PIXELS,
            area_threshold=PAPER_MIN_AREA,
            merge=True,
            margin=PAPER_MERGE_MARGIN,
        )
    except Exception:
        return None

    hint_x = None
    hint_y = None
    if laser_hint:
        _src, hint_x, hint_y, _w, _h, _pixels, _score = laser_hint

    best = None
    best_score = -1
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if w < PAPER_MIN_W or h < PAPER_MIN_H:
            continue
        if area < PAPER_MIN_AREA or area > PAPER_MAX_AREA:
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < PAPER_MIN_DENSITY:
            continue
        if area > FRAME_W * FRAME_H * 0.92 and density < PAPER_FULLSCREEN_MIN_DENSITY:
            continue

        contains_hint = False
        if hint_x is not None and hint_y is not None:
            contains_hint = hint_x >= x and hint_x <= x + w and hint_y >= y and hint_y <= y + h

        center_bonus = 160.0 - abs((x + w / 2.0) - FRAME_W / 2.0) * 0.25
        center_bonus -= abs((y + h / 2.0) - FRAME_H / 2.0) * 0.25
        hint_bonus = 120000 if contains_hint else 0
        score = int(area * 1.4 + pixels * 0.25 + density * 15 + center_bonus + hint_bonus)
        if score > best_score:
            inset = PAPER_INSET_PX
            rx0 = clamp(x + inset, 0.0, float(FRAME_W - 1))
            ry0 = clamp(y + inset, 0.0, float(FRAME_H - 1))
            rx1 = clamp(x + w - inset, rx0 + 1.0, float(FRAME_W))
            ry1 = clamp(y + h - inset, ry0 + 1.0, float(FRAME_H))
            best = ("paper", rx0, ry0, rx1 - rx0, ry1 - ry0, pixels, score)
            best_score = score

    return best


def find_square_by_rects(img, paper):
    try:
        rects = img.find_rects(threshold=SQUARE_RECT_THRESHOLD)
    except Exception:
        return None

    best = None
    best_score = -1
    for r in rects:
        corners = rect_corners(r)
        x, y, w, h = rect_bounds_from_corners(corners)
        area = polygon_area(corners)
        if not valid_square_box(x, y, w, h, area):
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue
        cx = sum([p[0] for p in corners]) / 4.0
        cy = sum([p[1] for p in corners]) / 4.0
        try:
            mag = int(r.magnitude())
        except Exception:
            mag = int(area)
        edge_margin = min(cx, cy, FRAME_W - cx, FRAME_H - cy)
        score = int(area + mag * 3 + edge_margin)
        if score > best_score:
            best = ("rect", cx, cy, w, h, corners, score)
            best_score = score
    if best_score < CENTER_DOT_MIN_SCORE:
        return None
    return best


def find_square_by_blobs(img, paper):
    try:
        blobs = img.find_blobs(
            [SQUARE_THRESHOLD],
            pixels_threshold=SQUARE_MIN_PIXELS,
            area_threshold=SQUARE_MIN_AREA,
            merge=True,
            margin=SQUARE_MERGE_MARGIN,
        )
    except Exception:
        return None

    best = None
    best_score = -1
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if not valid_square_box(x, y, w, h, area):
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue
        if pixels < SQUARE_MIN_PIXELS:
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < SQUARE_MIN_DENSITY or density > SQUARE_MAX_DENSITY:
            continue
        cx, cy = blob_center(b)
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        squareness = min(w, h) / max(w, h) if max(w, h) else 0.0
        score = int(pixels * 6 + area * 0.35 + squareness * 1200)
        if score > best_score:
            best = ("blob", cx, cy, w, h, corners, score)
            best_score = score
    return best


def find_square_by_union(img, paper):
    try:
        blobs = img.find_blobs(
            [SQUARE_THRESHOLD],
            pixels_threshold=SQUARE_PART_MIN_PIXELS,
            area_threshold=SQUARE_PART_MIN_AREA,
            merge=False,
            margin=0,
        )
    except Exception:
        return None

    parts = []
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if pixels < SQUARE_PART_MIN_PIXELS:
            continue
        if area < SQUARE_PART_MIN_AREA or area > SQUARE_PART_MAX_AREA:
            continue
        if x < 0 or y < 0 or x + w > FRAME_W or y + h > FRAME_H:
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < SQUARE_MIN_DENSITY or density > 98.0:
            continue
        parts.append((x, y, w, h, pixels, area))

    if len(parts) < SQUARE_UNION_MIN_BLOBS:
        return None

    x0 = min([p[0] for p in parts])
    y0 = min([p[1] for p in parts])
    x1 = max([p[0] + p[2] for p in parts])
    y1 = max([p[1] + p[3] for p in parts])
    w = x1 - x0
    h = y1 - y0
    area = w * h
    pixels = sum([p[4] for p in parts])

    if pixels < SQUARE_UNION_MIN_PIXELS:
        return None
    if not valid_square_box(x0, y0, w, h, area):
        return None
    if not box_in_paper(x0, y0, w, h, paper):
        return None

    density = pixels * 100.0 / area if area else 0.0
    if density < SQUARE_MIN_DENSITY or density > SQUARE_MAX_DENSITY:
        return None

    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    squareness = min(w, h) / max(w, h) if max(w, h) else 0.0
    score = int(pixels * 5 + area * 0.20 + squareness * 1200 + len(parts) * 80)
    return ("union", cx, cy, w, h, corners, score)


def find_square_frame(img, paper):
    target = find_square_by_union(img, paper)
    if not target:
        target = find_square_by_blobs(img, paper)
    if not target:
        target = find_square_by_rects(img, paper)
    return target


def find_frame_coarse(img, paper):
    if not FRAME_COARSE_ENABLED:
        return None

    square = find_square_frame(img, paper)
    if square:
        return square

    try:
        blobs = img.find_blobs(
            [FRAME_COARSE_THRESHOLD],
            pixels_threshold=FRAME_COARSE_MIN_PIXELS,
            area_threshold=FRAME_COARSE_MIN_AREA,
            merge=True,
            margin=FRAME_COARSE_MERGE_MARGIN,
        )
    except Exception:
        return None

    best = None
    best_score = -999999
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h

        if pixels < FRAME_COARSE_MIN_PIXELS:
            continue
        if area < FRAME_COARSE_MIN_AREA or area > FRAME_COARSE_MAX_AREA:
            continue
        if w < FRAME_COARSE_MIN_W or h < FRAME_COARSE_MIN_H:
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue

        aspect = w / h if h else 0.0
        if aspect <= 0.0 or aspect > FRAME_COARSE_MAX_ASPECT or aspect < FRAME_COARSE_MIN_ASPECT:
            continue

        density = pixels * 100.0 / area if area else 0.0
        if density < FRAME_COARSE_MIN_DENSITY or density > FRAME_COARSE_MAX_DENSITY:
            continue

        cx, cy = blob_center(b)
        edge_penalty = 0.0
        if cx < 6.0 or cx > FRAME_W - 6.0 or cy < 6.0 or cy > FRAME_H - 6.0:
            edge_penalty = 250.0
        score = int(pixels * 8.0 + area * 0.35 + min(w, h) * 10.0 - edge_penalty)
        if score > best_score:
            corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
            best = ("frame_coarse", cx, cy, w, h, corners, score)
            best_score = score

    return best


def find_center_dot_in_bounds(blobs, center_x, center_y, x0, y0, x1, y1, paper):
    best = None
    best_score = -999999
    x0, y0, x1, y1 = clamp_bounds_to_paper(x0, y0, x1, y1, paper)
    if x1 <= x0 or y1 <= y0:
        return None
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if area < CENTER_DOT_MIN_AREA or area > CENTER_DOT_MAX_AREA:
            continue
        if w < CENTER_DOT_MIN_W or h < CENTER_DOT_MIN_H:
            continue
        if w > CENTER_DOT_MAX_W or h > CENTER_DOT_MAX_H:
            continue
        aspect = w / h if h else 0.0
        if aspect <= 0.0 or aspect > CENTER_DOT_MAX_ASPECT or aspect < (1.0 / CENTER_DOT_MAX_ASPECT):
            continue
        cx, cy = blob_center(b)
        if cx < x0 or cx > x1 or cy < y0 or cy > y1:
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < CENTER_DOT_MIN_DENSITY:
            continue
        dx = cx - center_x
        dy = cy - center_y
        center_dist2 = dx * dx + dy * dy
        score = int(pixels * 14 + density * 5 - area * 0.25 - center_dist2 * 0.04)
        if score > best_score:
            corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
            best = ("dot", cx, cy, w, h, corners, score)
            best_score = score
    return best


def find_center_dot(img, square, laser, paper):
    if not square:
        return None

    for threshold in CENTER_DOT_THRESHOLDS:
        try:
            blobs = img.find_blobs(
                [threshold],
                pixels_threshold=CENTER_DOT_MIN_PIXELS,
                area_threshold=CENTER_DOT_MIN_AREA,
                merge=False,
                margin=0,
            )
        except Exception:
            continue

        _src, sq_cx, sq_cy, _sq_w, _sq_h, sq_corners, _sq_score = square
        sx, sy, sw, sh = rect_bounds_from_corners(sq_corners)
        margin_x = max(4.0, sw * CENTER_DOT_INNER_RATIO)
        margin_y = max(4.0, sh * CENTER_DOT_INNER_RATIO)
        dot = find_center_dot_in_bounds(
            blobs,
            sq_cx,
            sq_cy,
            sx + margin_x,
            sy + margin_y,
            sx + sw - margin_x,
            sy + sh - margin_y,
            paper,
        )
        if dot:
            return dot

    return None


def dot_has_isolated_component(merged_blobs, cx, cy):
    for b in merged_blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        if cx < x or cx > x + w or cy < y or cy > y + h:
            continue

        area = w * h
        if w > CENTER_DOT_GLOBAL_MERGED_MAX_W:
            return False
        if h > CENTER_DOT_GLOBAL_MERGED_MAX_H:
            return False
        if area > CENTER_DOT_GLOBAL_MERGED_MAX_AREA:
            return False
        aspect = w / h if h else 0.0
        if aspect <= 0.0 or aspect > 2.30 or aspect < (1.0 / 2.30):
            return False
        return True
    return False


def find_center_dot_global(img, paper, square=None, laser=None):
    if not CENTER_DOT_GLOBAL_ENABLED or not paper:
        return None

    px0, py0, px1, py1 = paper_bounds(paper)
    pw = px1 - px0
    ph = py1 - py0
    if pw <= 0.0 or ph <= 0.0:
        return None

    margin = max(
        float(CENTER_DOT_GLOBAL_EDGE_MARGIN_PX),
        min(pw, ph) * CENTER_DOT_GLOBAL_PAPER_MARGIN_RATIO,
    )
    rx0 = clamp(px0 + margin, 0.0, float(FRAME_W - 1))
    ry0 = clamp(py0 + margin, 0.0, float(FRAME_H - 1))
    rx1 = clamp(px1 - margin, rx0 + 1.0, float(FRAME_W))
    ry1 = clamp(py1 - margin, ry0 + 1.0, float(FRAME_H))
    if rx1 <= rx0 or ry1 <= ry0:
        return None

    ref_x = px0 + pw / 2.0
    ref_y = py0 + ph / 2.0
    frame_inner = None
    has_frame_ref = square is not None
    if square:
        _src, sq_cx, sq_cy, _sq_w, _sq_h, sq_corners, _sq_score = square
        sx, sy, sw, sh = rect_bounds_from_corners(sq_corners)
        frame_margin_x = max(4.0, sw * CENTER_DOT_INNER_RATIO)
        frame_margin_y = max(4.0, sh * CENTER_DOT_INNER_RATIO)
        frame_inner = (
            sx + frame_margin_x,
            sy + frame_margin_y,
            sx + sw - frame_margin_x,
            sy + sh - frame_margin_y,
        )
        ref_x = sq_cx
        ref_y = sq_cy

    best = None
    best_score = -999999
    for threshold in CENTER_DOT_THRESHOLDS:
        try:
            blobs = img.find_blobs(
                [threshold],
                pixels_threshold=CENTER_DOT_GLOBAL_MIN_PIXELS,
                area_threshold=CENTER_DOT_GLOBAL_MIN_AREA,
                merge=False,
                margin=0,
            )
        except Exception:
            continue

        try:
            merged_blobs = img.find_blobs(
                [threshold],
                pixels_threshold=CENTER_DOT_GLOBAL_MIN_PIXELS,
                area_threshold=CENTER_DOT_GLOBAL_MIN_AREA,
                merge=True,
                margin=CENTER_DOT_GLOBAL_ISOLATION_MARGIN,
            )
        except Exception:
            merged_blobs = []

        for b in blobs:
            x = float(value(b, 0))
            y = float(value(b, 1))
            w = float(value(b, 2))
            h = float(value(b, 3))
            pixels = float(value(b, 4, w * h))
            area = w * h

            if pixels < CENTER_DOT_GLOBAL_MIN_PIXELS:
                continue
            if area < CENTER_DOT_GLOBAL_MIN_AREA or area > CENTER_DOT_GLOBAL_MAX_AREA:
                continue
            if w < CENTER_DOT_GLOBAL_MIN_W or h < CENTER_DOT_GLOBAL_MIN_H:
                continue
            if w > CENTER_DOT_GLOBAL_MAX_W or h > CENTER_DOT_GLOBAL_MAX_H:
                continue

            aspect = w / h if h else 0.0
            if aspect <= 0.0 or aspect > CENTER_DOT_GLOBAL_MAX_ASPECT:
                continue
            if aspect < (1.0 / CENTER_DOT_GLOBAL_MAX_ASPECT):
                continue

            cx, cy = blob_center(b)
            if cx < rx0 or cx > rx1 or cy < ry0 or cy > ry1:
                continue
            if not box_in_paper(x, y, w, h, paper):
                continue

            density = pixels * 100.0 / area if area else 0.0
            if density < CENTER_DOT_GLOBAL_MIN_DENSITY:
                continue

            if merged_blobs and not dot_has_isolated_component(merged_blobs, cx, cy):
                continue

            frame_bonus = 0.0
            if frame_inner:
                fx0, fy0, fx1, fy1 = frame_inner
                if cx >= fx0 and cx <= fx1 and cy >= fy0 and cy <= fy1:
                    frame_bonus = 1800.0
                else:
                    frame_bonus = -700.0

            size_score = min(pixels, 260.0) * 13.0
            density_score = density * 5.0
            roundness_score = min(aspect, 1.0 / aspect) * 900.0
            dx = cx - ref_x
            dy = cy - ref_y
            center_dist2 = dx * dx + dy * dy
            if not has_frame_ref and center_dist2 > (
                CENTER_DOT_GLOBAL_FRAMELESS_MAX_DIST_PX * CENTER_DOT_GLOBAL_FRAMELESS_MAX_DIST_PX
            ):
                continue
            center_penalty = center_dist2 * 0.030
            area_penalty = max(0.0, area - 420.0) * 0.9
            score = int(size_score + density_score + roundness_score + frame_bonus - center_penalty - area_penalty)
            min_score = CENTER_DOT_GLOBAL_MIN_SCORE
            if not has_frame_ref:
                min_score = CENTER_DOT_GLOBAL_FRAMELESS_MIN_SCORE

            if score >= min_score and score > best_score:
                corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                best = ("dot", cx, cy, w, h, corners, score)
                best_score = score

    if best:
        return best
    return None


def paper_as_target(paper):
    if not paper:
        return None
    _src, x, y, w, h, _pixels, score = paper
    cx = x + w / 2.0
    cy = y + h / 2.0
    corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    return ("paper", cx, cy, w, h, corners, score)


def find_center_dot_fallback(img, paper):
    global fallback_debug_last_ms

    if not CENTER_DOT_FALLBACK_ENABLED or not paper:
        return None

    blobs = []
    for threshold in CENTER_DOT_THRESHOLDS:
        try:
            found = img.find_blobs(
                [threshold],
                pixels_threshold=CENTER_DOT_FALLBACK_MIN_PIXELS,
                area_threshold=CENTER_DOT_FALLBACK_MIN_AREA,
                merge=False,
                margin=0,
            )
        except Exception:
            continue
        for b in found:
            blobs.append(b)

    if not blobs:
        return None

    px0, py0, px1, py1 = paper_bounds(paper)
    pw = px1 - px0
    ph = py1 - py0
    if pw <= 0.0 or ph <= 0.0:
        return None

    relax = CENTER_DOT_FALLBACK_PAPER_RELAX_PX
    margin_x = max(4.0, pw * CENTER_DOT_FALLBACK_MARGIN_RATIO)
    margin_y = max(4.0, ph * CENTER_DOT_FALLBACK_MARGIN_RATIO)
    rx0 = clamp(px0 - relax + margin_x, 0.0, float(FRAME_W - 1))
    ry0 = clamp(py0 - relax + margin_y, 0.0, float(FRAME_H - 1))
    rx1 = clamp(px1 + relax - margin_x, rx0 + 1.0, float(FRAME_W))
    ry1 = clamp(py1 + relax - margin_y, ry0 + 1.0, float(FRAME_H))
    if rx1 <= rx0 or ry1 <= ry0:
        return None

    paper_cx = FRAME_W / 2.0
    paper_cy = FRAME_H / 2.0
    max_center_dist = CENTER_DOT_FALLBACK_MAX_CENTER_DIST_PX
    max_center_dist2 = max_center_dist * max_center_dist

    best = None
    best_score = -999999
    total = 0
    reject_pixels = 0
    reject_area = 0
    reject_size = 0
    reject_pos = 0
    reject_aspect = 0
    reject_density = 0
    reject_center = 0
    dbg_x = 0.0
    dbg_y = 0.0
    dbg_w = 0.0
    dbg_h = 0.0
    dbg_pixels = 0.0
    dbg_area = 0.0
    for b in blobs:
        total += 1
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if area > dbg_area:
            dbg_x = x
            dbg_y = y
            dbg_w = w
            dbg_h = h
            dbg_pixels = pixels
            dbg_area = area
        if pixels < CENTER_DOT_FALLBACK_MIN_PIXELS:
            reject_pixels += 1
            continue
        if area < CENTER_DOT_FALLBACK_MIN_AREA or area > CENTER_DOT_FALLBACK_MAX_AREA:
            reject_area += 1
            continue
        if w < CENTER_DOT_FALLBACK_MIN_W or h < CENTER_DOT_FALLBACK_MIN_H:
            reject_size += 1
            continue
        if w > CENTER_DOT_FALLBACK_MAX_W or h > CENTER_DOT_FALLBACK_MAX_H:
            reject_size += 1
            continue
        cx, cy = blob_center(b)
        if cx < rx0 or cx > rx1 or cy < ry0 or cy > ry1:
            reject_pos += 1
            continue

        aspect = w / h if h else 0.0
        if aspect <= 0.0 or aspect > CENTER_DOT_FALLBACK_MAX_ASPECT or aspect < (1.0 / CENTER_DOT_FALLBACK_MAX_ASPECT):
            reject_aspect += 1
            continue

        density = pixels * 100.0 / area if area else 0.0
        if density < CENTER_DOT_FALLBACK_MIN_DENSITY:
            reject_density += 1
            continue

        dx = cx - paper_cx
        dy = cy - paper_cy
        center_dist2 = dx * dx + dy * dy
        if center_dist2 > max_center_dist2:
            reject_center += 1
            continue

        size_bonus = min(pixels, 350.0) * 16.0
        density_bonus = density * 4.0
        center_penalty = center_dist2 * 0.025
        area_penalty = max(0.0, area - 650.0) * 1.5
        score = int(size_bonus + density_bonus - center_penalty - area_penalty)
        if score > best_score:
            corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
            best = ("dot_fallback", cx, cy, w, h, corners, score)
            best_score = score

    if TARGET_CANDIDATE_DEBUG:
        now = time.ticks_ms()
        if rate_due(now, fallback_debug_last_ms, TARGET_CANDIDATE_DEBUG_HZ):
            if best:
                _src, b_cx, b_cy, b_w, b_h, _corners, _score = best
            else:
                b_cx = 0.0
                b_cy = 0.0
                b_w = 0.0
                b_h = 0.0
            print(
                "$DOTDBG,total=%d,best_score=%d,bcx=%.1f,bcy=%.1f,bw=%d,bh=%d,"
                "max_x=%d,max_y=%d,max_w=%d,max_h=%d,max_pix=%d,max_area=%d,"
                "rej_pix=%d,rej_area=%d,rej_size=%d,rej_pos=%d,rej_aspect=%d,rej_density=%d,rej_center=%d,"
                "rx0=%d,ry0=%d,rx1=%d,ry1=%d"
                % (
                    total,
                    int(best_score),
                    b_cx,
                    b_cy,
                    int(b_w),
                    int(b_h),
                    int(dbg_x),
                    int(dbg_y),
                    int(dbg_w),
                    int(dbg_h),
                    int(dbg_pixels),
                    int(dbg_area),
                    reject_pixels,
                    reject_area,
                    reject_size,
                    reject_pos,
                    reject_aspect,
                    reject_density,
                    reject_center,
                    int(rx0),
                    int(ry0),
                    int(rx1),
                    int(ry1),
                )
            )
            fallback_debug_last_ms = now

    if best and best_score >= CENTER_DOT_FALLBACK_MIN_SCORE:
        return best
    return None


def find_aim_target(img, laser, paper):
    dot = find_center_dot_global(img, paper, None, laser)
    if dot:
        return dot

    frame = find_square_frame(img, paper)
    dot = find_center_dot(img, frame, laser, paper)
    if dot:
        return dot

    dot = find_center_dot_global(img, paper, frame, laser)
    if dot:
        return dot

    if paper:
        dot = find_center_dot_fallback(img, paper)
        if dot:
            return dot

    if frame:
        return frame

    frame = find_frame_coarse(img, paper)
    if frame:
        return frame

    if paper:
        return paper_as_target(paper)
    return None



def find_laser(img, paper=None, hint_x=None, hint_y=None, hint_max_dist=None):
    try:
        blobs = img.find_blobs(
            [LASER_THRESHOLD],
            pixels_threshold=LASER_MIN_PIXELS,
            area_threshold=LASER_MIN_AREA,
            merge=True,
            margin=2,
        )
    except Exception:
        return None

    best = None
    best_score = -1
    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if area < LASER_MIN_AREA or area > LASER_MAX_AREA:
            continue
        if w < LASER_MIN_W or h < LASER_MIN_H:
            continue
        if w > LASER_MAX_W or h > LASER_MAX_H:
            continue
        if not box_in_paper(x, y, w, h, paper):
            continue
        aspect = w / h if h else 0.0
        if aspect < 0.25 or aspect > 3.50:
            continue
        density = pixels * 100.0 / area if area else 0.0
        cx, cy = blob_center(b)
        hint_bonus = 0.0
        if hint_x is not None and hint_y is not None:
            dx_hint = cx - hint_x
            dy_hint = cy - hint_y
            hint_dist2 = dx_hint * dx_hint + dy_hint * dy_hint
            max_dist = hint_max_dist if hint_max_dist is not None else LASER_HINT_MAX_DIST_PX
            if hint_dist2 > max_dist * max_dist:
                continue
            hint_bonus = max(0.0, max_dist - math.sqrt(hint_dist2)) * 12.0
        center_bonus = 100.0 - abs(cx - FRAME_W / 2.0) * 0.05 - abs(cy - FRAME_H / 2.0) * 0.05
        score = int(pixels * 8 + density * 4 - area + center_bonus + hint_bonus)
        if score > best_score:
            best = ("red", cx, cy, w, h, pixels, score)
            best_score = score
    return best


def pixel_delta_to_angle(dx, dy):
    fx = FRAME_W / (2.0 * math.tan(math.radians(HFOV_DEG) / 2.0))
    fy = FRAME_H / (2.0 * math.tan(math.radians(VFOV_DEG) / 2.0))
    yaw = ERROR_YAW_SIGN * math.degrees(math.atan(dx / fx))
    pitch = ERROR_PITCH_SIGN * math.degrees(math.atan(dy / fy))
    return yaw, pitch


def shape_command_angle(yaw, pitch, err_x, err_y, gain=CONTROL_GAIN, max_deg=CONTROL_MAX_DEG):
    if abs(err_x) <= ERROR_DEADBAND_PX:
        yaw = 0.0
    else:
        yaw = clamp(yaw * gain, -max_deg, max_deg)

    if abs(err_y) <= ERROR_DEADBAND_PX:
        pitch = 0.0
    else:
        pitch = clamp(pitch * gain, -max_deg, max_deg)

    return yaw, pitch


def map_to_motor_commands(image_yaw, image_pitch):
    # Observed wiring: STM32 yaw channel moves vertical error, STM32 pitch
    # channel moves horizontal error; both need inverted command signs.
    motor_yaw = -image_pitch
    motor_pitch = -image_yaw
    return motor_yaw, motor_pitch


def send_packet(uart, valid, yaw, pitch):
    # Keep the control link minimal and parseable by STM32.
    line = "$K230,%d,%.2f,%.2f\n" % (valid, yaw, pitch)
    uart.write(line)
    if USB_DEBUG_ENABLED:
        print("$TX,%s" % line.strip())
    return line


def poll_stm32_debug(uart, rx_buffer):
    if not STM32_RX_DEBUG:
        return rx_buffer

    try:
        n = uart.any()
    except Exception:
        return rx_buffer
    if not n:
        return rx_buffer

    try:
        data = uart.read(n)
    except TypeError:
        data = uart.read()
    if not data:
        return rx_buffer

    try:
        text = data.decode("utf-8")
    except Exception:
        text = repr(data)

    rx_buffer += text
    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        line = line.strip()
        if line:
            print("$STM32_RX,%s" % line)

    if len(rx_buffer) > STM32_RX_MAX_BUFFER:
        print("$STM32_RX_PART,%s" % rx_buffer[-STM32_RX_MAX_BUFFER:])
        rx_buffer = ""

    return rx_buffer


def print_debug(
    valid,
    paper,
    target,
    laser,
    med_tx,
    med_ty,
    filt_tx,
    filt_ty,
    med_lx,
    med_ly,
    filt_lx,
    filt_ly,
    err_x,
    err_y,
    raw_yaw,
    raw_pitch,
    yaw,
    pitch,
    cmd_yaw,
    cmd_pitch,
    target_spread,
    laser_spread,
    fps,
):
    t_src, t_cx, t_cy, t_w, t_h, _corners, t_score = target
    l_src, l_cx, l_cy, l_w, l_h, l_pixels, l_score = laser
    if paper:
        _p_src, p_x, p_y, p_w, p_h, _p_pixels, p_score = paper
        paper_ok = 1
    else:
        p_x = 0.0
        p_y = 0.0
        p_w = 0.0
        p_h = 0.0
        p_score = 0
        paper_ok = 0
    print(
        "$DBG,valid=%d,mode=%s,paper=%d,px=%d,py=%d,pw=%d,ph=%d,p_score=%d,target=%s,laser=%s,"
        "tcx=%.1f,tcy=%.1f,med_tcx=%.1f,med_tcy=%.1f,filt_tcx=%.1f,filt_tcy=%.1f,"
        "lcx=%.1f,lcy=%.1f,med_lcx=%.1f,med_lcy=%.1f,filt_lcx=%.1f,filt_lcy=%.1f,"
        "err_x=%.1f,err_y=%.1f,t_spread=%.1f,l_spread=%.1f,tw=%d,th=%d,lw=%d,lh=%d,t_score=%d,l_score=%d,"
        "fps=%.1f,raw_yaw=%.2f,raw_pitch=%.2f,img_yaw=%.2f,img_pitch=%.2f,cmd_yaw=%.2f,cmd_pitch=%.2f"
        % (
            valid,
            CONTROL_MODE,
            paper_ok,
            int(p_x),
            int(p_y),
            int(p_w),
            int(p_h),
            int(p_score),
            t_src,
            l_src,
            t_cx,
            t_cy,
            med_tx,
            med_ty,
            filt_tx,
            filt_ty,
            l_cx,
            l_cy,
            med_lx,
            med_ly,
            filt_lx,
            filt_ly,
            err_x,
            err_y,
            target_spread,
            laser_spread,
            int(t_w),
            int(t_h),
            int(l_w),
            int(l_h),
            int(t_score),
            int(l_score),
            fps,
            raw_yaw,
            raw_pitch,
            yaw,
            pitch,
            cmd_yaw,
            cmd_pitch,
        )
    )


def print_lost_debug(paper, has_target, has_laser, fps):
    if paper:
        _p_src, p_x, p_y, p_w, p_h, _p_pixels, p_score = paper
        paper_ok = 1
    else:
        p_x = 0.0
        p_y = 0.0
        p_w = 0.0
        p_h = 0.0
        p_score = 0
        paper_ok = 0
    print(
        "$DBG,valid=0,mode=%s,paper=%d,px=%d,py=%d,pw=%d,ph=%d,p_score=%d,target=%d,laser=%d,fps=%.1f,yaw=0.00,pitch=0.00"
        % (
            CONTROL_MODE,
            paper_ok,
            int(p_x),
            int(p_y),
            int(p_w),
            int(p_h),
            int(p_score),
            1 if has_target else 0,
            1 if has_laser else 0,
            fps,
        )
    )


def main(run_seconds=None):
    os.exitpoint(os.EXITPOINT_ENABLE)
    uart = setup_uart()
    sensor = None
    clock = time.clock()

    target_samples = []
    laser_samples = []
    prev_tx = None
    prev_ty = None
    prev_lx = None
    prev_ly = None
    last_lost_ms = 0
    last_control_ms = 0
    last_debug_ms = 0
    last_sent_valid = 0
    laser_missing_count = 0
    prev_target_src = None
    target_jump_reject_count = 0
    stm32_rx_buffer = ""

    try:
        sensor = setup_camera()
        if USB_DEBUG_ENABLED:
            print("square laser tracker started %s" % SCRIPT_VERSION)
        started_ms = time.ticks_ms()

        while True:
            os.exitpoint()
            if run_seconds is not None:
                elapsed_ms = time.ticks_diff(time.ticks_ms(), started_ms)
                if elapsed_ms >= int(run_seconds * 1000):
                    if USB_DEBUG_ENABLED:
                        print("square laser tracker finished")
                    break

            clock.tick()
            img = sensor.snapshot(chn=CAPTURE_CHN)
            fps = clock.fps()
            stm32_rx_buffer = poll_stm32_debug(uart, stm32_rx_buffer)

            if FORCE_COMMAND_ENABLED:
                now = time.ticks_ms()
                if rate_due(now, last_control_ms, CONTROL_SEND_HZ):
                    send_packet(uart, 1, FORCE_YAW, FORCE_PITCH)
                    last_control_ms = now
                if USB_DEBUG_ENABLED and rate_due(now, last_debug_ms, DEBUG_PRINT_HZ):
                    print(
                        "$DBG_FORCE,yaw=%.2f,pitch=%.2f,fps=%.1f"
                        % (FORCE_YAW, FORCE_PITCH, fps)
                    )
                    last_debug_ms = now
                continue

            laser_hint = None
            if prev_lx is not None and prev_ly is not None:
                laser_hint = find_laser(img, None, prev_lx, prev_ly, LASER_HINT_MAX_DIST_PX)
                if not laser_hint and LASER_GLOBAL_FALLBACK_ENABLED:
                    laser_hint = find_laser(img, None, prev_lx, prev_ly, LASER_GLOBAL_REACQUIRE_MAX_DIST_PX)
            if not laser_hint and LASER_GLOBAL_FALLBACK_ENABLED:
                laser_hint = find_laser(img, None, None, None, None)
            paper = find_paper(img, laser_hint)
            if paper:
                laser = laser_hint
                hint_x = prev_lx
                hint_y = prev_ly
                hint_max_dist = LASER_HINT_MAX_DIST_PX
                if not laser and hint_x is not None and hint_y is not None:
                    laser = find_laser(img, paper, hint_x, hint_y, hint_max_dist)
                    if not laser and LASER_GLOBAL_FALLBACK_ENABLED:
                        laser = find_laser(
                            img,
                            None,
                            hint_x,
                            hint_y,
                            LASER_GLOBAL_REACQUIRE_MAX_DIST_PX,
                        )
                else:
                    if not laser and LASER_GLOBAL_FALLBACK_ENABLED:
                        laser = find_laser(img, None, None, None, None)

                target = find_aim_target(img, laser, paper)
                if not target:
                    target = find_frame_coarse(img, None)
            else:
                laser = laser_hint
                if LASER_GLOBAL_FALLBACK_ENABLED:
                    if not laser and prev_lx is not None and prev_ly is not None:
                        laser = find_laser(img, None, prev_lx, prev_ly, LASER_GLOBAL_REACQUIRE_MAX_DIST_PX)
                    if not laser:
                        laser = find_laser(img, None, None, None, None)
                target = find_frame_coarse(img, None)
            target_rejected = False
            laser_rejected = False

            if target and prev_tx is not None:
                _src, t_cx, t_cy, _w, _h, _corners, _score = target
                if prev_target_src is not None and _src != prev_target_src:
                    target_samples = []
                    prev_tx = None
                    prev_ty = None
                else:
                    jump_x = t_cx - prev_tx
                    jump_y = t_cy - prev_ty
                    if (jump_x * jump_x + jump_y * jump_y) > (MAX_TARGET_JUMP_PX * MAX_TARGET_JUMP_PX):
                        target_jump_reject_count += 1
                        if target_jump_reject_count >= TARGET_JUMP_RELOCK_COUNT:
                            if USB_DEBUG_ENABLED:
                                print(
                                    "$DBG_RELOCK,kind=target_jump,cx=%.1f,cy=%.1f,prev_cx=%.1f,prev_cy=%.1f"
                                    % (t_cx, t_cy, prev_tx, prev_ty)
                                )
                            target_samples = []
                            prev_tx = None
                            prev_ty = None
                            prev_target_src = None
                            target_jump_reject_count = 0
                        else:
                            if USB_DEBUG_ENABLED:
                                print(
                                    "$DBG_REJECT,kind=target_jump,count=%d,cx=%.1f,cy=%.1f,prev_cx=%.1f,prev_cy=%.1f"
                                    % (target_jump_reject_count, t_cx, t_cy, prev_tx, prev_ty)
                                )
                            target = None
                            target_rejected = True
                if target and target_rejected:
                    target_rejected = False
            if target:
                target_jump_reject_count = 0
                prev_target_src = target[0]
            elif not target_rejected:
                target_jump_reject_count = 0
                prev_target_src = None

            if laser and prev_lx is not None:
                _src, l_cx, l_cy, _w, _h, _pixels, _score = laser
                jump_x = l_cx - prev_lx
                jump_y = l_cy - prev_ly
                if (jump_x * jump_x + jump_y * jump_y) > (MAX_LASER_JUMP_PX * MAX_LASER_JUMP_PX):
                    if USB_DEBUG_ENABLED:
                        print(
                            "$DBG_REJECT,kind=laser_jump,cx=%.1f,cy=%.1f,prev_cx=%.1f,prev_cy=%.1f"
                            % (l_cx, l_cy, prev_lx, prev_ly)
                        )
                    laser = None
                    laser_rejected = True

            if target and laser:
                laser_missing_count = 0
                _t_src, t_cx, t_cy, _t_w, _t_h, _corners, _t_score = target
                _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser

                med_tx, med_ty, filt_tx, filt_ty = filtered_point(
                    target_samples,
                    t_cx,
                    t_cy,
                    prev_tx,
                    prev_ty,
                    TARGET_FILTER_WINDOW,
                    TARGET_FILTER_ALPHA,
                    TARGET_FILTER_DEADBAND_PX,
                )
                med_lx, med_ly, filt_lx, filt_ly = filtered_point(
                    laser_samples,
                    l_cx,
                    l_cy,
                    prev_lx,
                    prev_ly,
                    LASER_FILTER_WINDOW,
                    LASER_FILTER_ALPHA,
                    LASER_FILTER_DEADBAND_PX,
                )
                prev_tx = filt_tx
                prev_ty = filt_ty
                prev_lx = filt_lx
                prev_ly = filt_ly

                err_x = filt_lx - filt_tx
                err_y = filt_ly - filt_ty
                raw_yaw, raw_pitch = pixel_delta_to_angle(err_x, err_y)
                target_is_fine = _t_src[:3] == "dot"
                if target_is_fine:
                    yaw, pitch = shape_command_angle(raw_yaw, raw_pitch, err_x, err_y)
                else:
                    yaw, pitch = shape_command_angle(
                        raw_yaw,
                        raw_pitch,
                        err_x,
                        err_y,
                        COARSE_CONTROL_GAIN,
                        COARSE_CONTROL_MAX_DEG,
                    )
                motor_yaw, motor_pitch = map_to_motor_commands(yaw, pitch)

                target_spread = samples_spread(target_samples)
                laser_spread = samples_spread(laser_samples)
                valid = 1
                if len(target_samples) < TARGET_WARMUP or len(laser_samples) < LASER_WARMUP:
                    valid = 0
                if target_spread > TARGET_STABLE_SPREAD_PX or laser_spread > LASER_STABLE_SPREAD_PX:
                    valid = 0

                out_yaw = motor_yaw
                out_pitch = motor_pitch
                if FORCE_COMMAND_ENABLED:
                    out_yaw = FORCE_YAW
                    out_pitch = FORCE_PITCH

                now = time.ticks_ms()
                send_valid = valid and CONTROL_OUTPUT_ENABLED
                if rate_due(now, last_control_ms, CONTROL_SEND_HZ) or (
                    last_sent_valid and not send_valid
                ):
                    if send_valid:
                        send_packet(uart, 1, out_yaw, out_pitch)
                        last_sent_valid = 1
                    else:
                        send_packet(uart, 0, 0.0, 0.0)
                        last_sent_valid = 0
                    last_control_ms = now

                if USB_DEBUG_ENABLED and rate_due(now, last_debug_ms, DEBUG_PRINT_HZ):
                    print_debug(
                        valid,
                        paper,
                        target,
                        laser,
                        med_tx,
                        med_ty,
                        filt_tx,
                        filt_ty,
                        med_lx,
                        med_ly,
                        filt_lx,
                        filt_ly,
                        err_x,
                        err_y,
                        raw_yaw,
                        raw_pitch,
                        yaw,
                        pitch,
                        out_yaw,
                        out_pitch,
                        target_spread,
                        laser_spread,
                        fps,
                    )
                    last_debug_ms = now
            else:
                if not target and not target_rejected:
                    target_samples = []
                    prev_tx = None
                    prev_ty = None
                if not laser and not laser_rejected:
                    laser_missing_count += 1
                    if laser_missing_count >= LASER_HOLD_MISSES:
                        laser_samples = []
                        prev_lx = None
                        prev_ly = None
                        laser_missing_count = 0

                now = time.ticks_ms()
                if time.ticks_diff(now, last_lost_ms) >= int(1000 / LOST_SEND_HZ):
                    send_packet(uart, 0, 0.0, 0.0)
                    last_sent_valid = 0
                    if USB_DEBUG_ENABLED:
                        print_lost_debug(paper, target is not None, laser is not None, fps)
                    last_lost_ms = now

            img = None
            gc.collect()

    except KeyboardInterrupt:
        if USB_DEBUG_ENABLED:
            print("square laser tracker stopped")
    finally:
        try:
            send_packet(uart, 0, 0.0, 0.0)
        except Exception:
            pass
        try:
            if sensor:
                sensor.stop()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


main(DEFAULT_RUN_SECONDS)
