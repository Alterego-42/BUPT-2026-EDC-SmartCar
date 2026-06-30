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
CAMERA_ROTATION_CW_DEG = 90

CONTROL_MODE = "red_rings_laser_closed_loop"
SCRIPT_VERSION = "red_rings_v56_autorun_config"
LOST_SEND_HZ = 15
CONTROL_SEND_HZ = 45
DEBUG_PRINT_HZ = 8
TARGET_CANDIDATE_DEBUG = False
TARGET_CANDIDATE_DEBUG_HZ = 2
USB_DEBUG_ENABLED = False
STM32_RX_DEBUG = False
STM32_RX_MAX_BUFFER = 256
CONTROL_OUTPUT_ENABLED = True
DEFAULT_RUN_SECONDS = None
DIAGNOSE_ONCE = False
DIAG_CAPTURE_PATH = "/sdcard/k230_diag.jpg"
FORCE_COMMAND_ENABLED = False
FORCE_YAW = 0.0
FORCE_PITCH = 1.00
CAMERA_START_RETRIES = 4
CAMERA_WARMUP_FRAMES = 8
CAMERA_WARMUP_MAX_TRIES = 60
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
PAPER_MERGE_MARGIN = 6
PAPER_INSET_PX = 2

# Dark, thick square frame. Keep this strict; broad thresholds merge the
# desk/shadow with the frame and make the tracker fall back to paper_center.
SQUARE_THRESHOLD = (0, 20, -128, 127, -128, 127)
SQUARE_RECT_THRESHOLD = 8000
SQUARE_EDGE_MARGIN = 0
SQUARE_MIN_W = 55
SQUARE_MIN_H = 55
SQUARE_MAX_W = FRAME_W - 6
SQUARE_MAX_H = FRAME_H - 6
SQUARE_MIN_AREA = 2500
SQUARE_MAX_AREA = FRAME_W * FRAME_H * 0.85
SQUARE_MIN_PIXELS = 80
SQUARE_MERGE_MARGIN = 8
SQUARE_MIN_ASPECT = 0.25
SQUARE_MAX_ASPECT = 4.00
SQUARE_MIN_DENSITY = 1
SQUARE_MAX_DENSITY = 85
SQUARE_UNION_MIN_BLOBS = 2
SQUARE_UNION_MIN_PIXELS = 180
SQUARE_PART_MIN_PIXELS = 18
SQUARE_PART_MIN_AREA = 25
SQUARE_PART_MAX_AREA = FRAME_W * FRAME_H * 0.25
SQUARE_RECT_MIN_SCORE = 2500
SQUARE_PAPER_RELAX_PX = 18
SQUARE_CLIPPED_FALLBACK_ENABLED = True
SQUARE_CLIPPED_MIN_PARTS = 1
SQUARE_CLIPPED_MIN_PIXELS = 140
SQUARE_CLIPPED_MIN_W = 45
SQUARE_CLIPPED_MIN_H = 45
SQUARE_CLIPPED_PAPER_RELAX_PX = 28
FRAME_CENTER_MIN_W = 70
FRAME_CENTER_MIN_H = 80
FRAME_CENTER_MIN_SCORE = 20000
FRAME_CENTER_MIN_ASPECT = 0.45
FRAME_CENTER_MAX_ASPECT = 1.80
FRAME_CENTER_MIN_EDGE_PX = 45
FRAME_PART_MAX_ASPECT = 2.20
FRAME_PART_MIN_EDGE_PX = 8

# If the target has a black center mark, use it as the true aim point.
CENTER_DOT_THRESHOLDS = (
    (0, 35, -128, 127, -128, 127),
    (0, 45, -128, 127, -128, 127),
    (0, 55, -128, 127, -128, 127),
    (0, 65, -128, 127, -128, 127),
)
CENTER_DOT_THRESHOLD = CENTER_DOT_THRESHOLDS[-1]
CENTER_DOT_MIN_PIXELS = 2
CENTER_DOT_MIN_AREA = 2
CENTER_DOT_MIN_W = 2
CENTER_DOT_MIN_H = 2
CENTER_DOT_MAX_W = 42
CENTER_DOT_MAX_H = 42
CENTER_DOT_MAX_AREA = 1000
CENTER_DOT_INNER_RATIO = 0.16
CENTER_DOT_MIN_DENSITY = 3
CENTER_DOT_MAX_ASPECT = 2.20
CENTER_DOT_ACCEPT_MIN_SCORE = 300
CENTER_DOT_MAX_CENTER_RATIO = 0.36
CENTER_DOT_MIN_CENTER_DIST_PX = 16
CENTER_DOT_MAX_CENTER_DIST_PX = 52
CENTER_DOT_PAPER_CENTER_RATIO = 0.24
CENTER_DOT_PAPER_MIN_CENTER_DIST_PX = 16
CENTER_DOT_PAPER_MAX_CENTER_DIST_PX = 42
CENTER_DOT_LASER_FALLBACK_ENABLED = False
CENTER_DOT_MAX_LASER_DIST_PX = 120
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
FRAME_CENTER_FALLBACK_ENABLED = True
PAPER_CENTER_FALLBACK_ENABLED = True
RED_RINGS_PAPER_CENTER_FALLBACK_ENABLED = False
RECT_TARGET_ENABLED = False
BLACK_FRAME_FALLBACK_ENABLED = False
FRAME_PART_RECENTER_ENABLED = True
FRAME_PART_RECENTER_GAIN = 0.55
FRAME_PART_RECENTER_DEADBAND_PX = 18.0
FRAME_PART_RECENTER_MAJOR_AXIS_RATIO = 1.35

# Required competition target: red concentric rings on a white panel.
# Use the red ring geometry as the true target when it is sufficiently visible.
# If only a partial ring set is visible, use it only as a recovery cue.
RED_RINGS_TARGET_ENABLED = True
RED_RINGS_THRESHOLDS = (
    (0, 100, 8, 127, -80, 45),
    (0, 100, 18, 127, -80, 60),
)
RED_RINGS_MIN_PART_PIXELS = 5
RED_RINGS_MIN_PART_AREA = 5
RED_RINGS_MAX_PART_AREA = FRAME_W * FRAME_H * 0.40
RED_RINGS_MIN_PART_DENSITY = 1.0
RED_RINGS_MAX_PART_DENSITY = 100.0
RED_RINGS_SOLID_PART_MIN_AREA = 320
RED_RINGS_SOLID_PART_MAX_DENSITY = 72.0
RED_RINGS_MIN_TOTAL_PIXELS = 10
RED_RINGS_MIN_PARTS = 1
RED_RINGS_MIN_W = 8
RED_RINGS_MIN_H = 1
RED_RINGS_MAX_W = FRAME_W - 2
RED_RINGS_MAX_H = FRAME_H - 2
RED_RINGS_MIN_ASPECT = 0.12
RED_RINGS_MAX_ASPECT = 8.00
RED_RINGS_UNION_MIN_DENSITY = 2.0
RED_RINGS_UNION_MAX_DENSITY = 100.0
RED_RINGS_COMPLETE_MIN_W = 85
RED_RINGS_COMPLETE_MIN_H = 85
RED_RINGS_COMPLETE_MIN_ASPECT = 0.55
RED_RINGS_COMPLETE_MAX_ASPECT = 1.85
RED_RINGS_COMPLETE_EDGE_MARGIN_PX = 1
RED_RINGS_PART_RECENTER_ENABLED = False
RED_RINGS_PART_RECENTER_GAIN = 0.50
RED_RINGS_PART_EDGE_GAIN = 0.75
RED_RINGS_PART_CONTROL_MAX_DEG = 0.08
RED_RINGS_PART_DEADBAND_PX = 12.0
RED_RINGS_PART_FINE_RADIUS_PX = 35.0
RED_RINGS_PART_FINE_CONTROL_MAX_DEG = 0.06
RED_RINGS_PAPER_RELAX_PX = -10
RED_RINGS_MAX_CENTER_JUMP_PX = 26
RED_RINGS_REJECT_HOLD_MISSES = 45
RED_RINGS_PART_EDGE_COMMAND_ENABLED = True
RED_RINGS_PART_EDGE_COMMAND_MARGIN_PX = 20
RED_RINGS_DEBUG = False

# Far target guard: at long range the red rings may collapse into a thin red
# fragment. Use the black panel only as a search gate / center fallback, never
# as a corner-tracking target.
RED_PANEL_FRAME_ENABLED = True
RED_PANEL_FRAME_MIN_PIXELS = 55
RED_PANEL_FRAME_MIN_AREA = 1000
RED_PANEL_FRAME_MAX_AREA = FRAME_W * FRAME_H * 0.70
RED_PANEL_FRAME_MIN_W = 34
RED_PANEL_FRAME_MIN_H = 44
RED_PANEL_FRAME_MAX_W = FRAME_W - 8
RED_PANEL_FRAME_MAX_H = FRAME_H - 8
RED_PANEL_FRAME_MIN_ASPECT = 0.22
RED_PANEL_FRAME_MAX_ASPECT = 1.20
RED_PANEL_FRAME_MIN_DENSITY = 1.5
RED_PANEL_FRAME_MAX_DENSITY = 88.0
RED_PANEL_FRAME_MERGE_MARGIN = 4
RED_PANEL_FRAME_PAPER_RELAX_PX = 26
RED_PANEL_FRAME_ROI_INSET_PX = 2
RED_PANEL_RECT_CENTER_PRIORITY_ENABLED = True
RED_PANEL_CENTER_FALLBACK_ENABLED = True
RED_PANEL_CENTER_MIN_RED_PIXELS = 20
RED_PANEL_CENTER_MIN_RED_PARTS = 3
RED_PANEL_CENTER_DEADBAND_PX = 8.0
RED_PANEL_CENTER_X_DEADBAND_PX = 14.0
RED_PANEL_CENTER_FINE_RADIUS_PX = 52.0
RED_PANEL_CENTER_CONTROL_MAX_DEG = 0.08
RED_PANEL_CENTER_TARGET_WARMUP = 1
RED_PANEL_CENTER_STABLE_SPREAD_PX = 10.0
RED_PANEL_CENTER_MAX_JUMP_PX = 24
RED_RINGS_PART_CONTROL_ENABLED = True
LASER_LOST_RECOVERY_ENABLED = False
LASER_LOST_RECOVERY_DEG = 0.08

# Soft pitch guard. The target board has nearly fixed height in the task, so
# if the target center climbs too high in the image, further upward commands
# are more likely to be a runaway than a useful correction.
PITCH_GUARD_ENABLED = True
PITCH_GUARD_MIN_TARGET_CY = 62.0
PITCH_GUARD_SOFT_TARGET_CY = 85.0
PITCH_GUARD_UP_CMD_SIGN = -1.0
PITCH_GUARD_MAX_UP_CMD = 0.04

# Red laser dot.
FIXED_LASER_ENABLED = True
FIXED_LASER_CX = 195.0
FIXED_LASER_CY = 93.0
LASER_THRESHOLD = (45, 100, 22, 127, -30, 127)
LASER_MIN_PIXELS = 8
LASER_MIN_AREA = 4
LASER_MIN_W = 2
LASER_MIN_H = 2
LASER_MAX_AREA = 90
LASER_MAX_W = 10
LASER_MAX_H = 10
LASER_MIN_DENSITY = 45.0
LASER_MIN_ROUNDNESS = 0.45
LASER_MIN_BRIGHT_PIXELS = 5
LASER_BRIGHT_THRESHOLD = (58, 100, 18, 127, -10, 127)
LASER_HINT_MAX_DIST_PX = 70
LASER_REACQUIRE_MAX_DIST_PX = 220
LASER_PAPER_RELAX_PX = 80

# Filtering: keep target center stable, keep laser responsive.
TARGET_FILTER_WINDOW = 2
TARGET_FILTER_ALPHA = 0.98
TARGET_FILTER_DEADBAND_PX = 0.3
TARGET_WARMUP = 1

LASER_FILTER_WINDOW = 2
LASER_FILTER_ALPHA = 0.85
LASER_FILTER_DEADBAND_PX = 0.4
LASER_WARMUP = 2

ERROR_DEADBAND_PX = 2.0
CONTROL_GAIN = 0.35
CONTROL_MAX_DEG = 0.20
FRAME_CENTER_DEADBAND_PX = 9.0
FRAME_CENTER_FINE_RADIUS_PX = 38.0
FRAME_CENTER_FINE_CONTROL_MAX_DEG = 0.04
RED_RINGS_DEADBAND_PX = 8.0
RED_RINGS_FINE_RADIUS_PX = 36.0
RED_RINGS_FINE_CONTROL_MAX_DEG = 0.08
RED_RINGS_SAFE_CONTROL_MAX_DEG = 0.08
PAPER_CENTER_CONTROL_MAX_DEG = 0.08
FRAME_PART_CONTROL_MAX_DEG = 0.10
MAX_TARGET_JUMP_PX = 70
MAX_LASER_JUMP_PX = 45
TARGET_HOLD_MISSES = 5
LASER_HOLD_MISSES = 8
TARGET_STABLE_SPREAD_PX = 30
LASER_STABLE_SPREAD_PX = 45
TARGET_HOLD_MS = 220
FRAME_CENTER_LOCK_FRAMES = 8
FRAME_CENTER_LOCK_MARGIN_PX = 26

CONFIG_OVERRIDES_ENABLED = True
CONFIG_PATH = "/sdcard/k230_target_config.py"


def load_config_overrides():
    if not CONFIG_OVERRIDES_ENABLED:
        return

    namespace = dict(globals())
    try:
        exec(open(CONFIG_PATH).read(), namespace)
    except Exception as exc:
        try:
            import k230_target_config as config
            namespace = config.__dict__
        except Exception as exc2:
            print("$CFG,load_error=%s,%s" % (exc, exc2))
            return

    for key, val in namespace.items():
        if key.startswith("_"):
            continue
        if key.isupper():
            globals()[key] = val


load_config_overrides()

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


def target_bounds(target):
    if not target:
        return 0.0, 0.0, float(FRAME_W), float(FRAME_H)
    _src, cx, cy, w, h, corners, _score = target
    if corners:
        x0, y0, bw, bh = rect_bounds_from_corners(corners)
    else:
        x0 = cx - w / 2.0
        y0 = cy - h / 2.0
        bw = w
        bh = h
    return x0, y0, x0 + bw, y0 + bh


def box_in_target_bounds(x, y, w, h, target, margin=0.0):
    if not target:
        return True
    x0, y0, x1, y1 = target_bounds(target)
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

        contains_hint = False
        hint_dist = 9999.0
        if hint_x is not None and hint_y is not None:
            contains_hint = hint_x >= x and hint_x <= x + w and hint_y >= y and hint_y <= y + h
            if contains_hint:
                blob_cx, blob_cy = blob_center(b)
                dx_hint = blob_cx - hint_x
                dy_hint = blob_cy - hint_y
                hint_dist = math.sqrt(dx_hint * dx_hint + dy_hint * dy_hint)

        center_bonus = 160.0 - abs((x + w / 2.0) - FRAME_W / 2.0) * 0.25
        center_bonus -= abs((y + h / 2.0) - FRAME_H / 2.0) * 0.25
        hint_bonus = (250000 - hint_dist * 180.0) if contains_hint else 0
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
        if not box_in_paper(x, y, w, h, paper, -SQUARE_PAPER_RELAX_PX):
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
    if best_score < SQUARE_RECT_MIN_SCORE:
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
        if not box_in_paper(x, y, w, h, paper, -SQUARE_PAPER_RELAX_PX):
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
        if not box_in_paper(x, y, w, h, paper, -SQUARE_PAPER_RELAX_PX):
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
    if not box_in_paper(x0, y0, w, h, paper, -SQUARE_PAPER_RELAX_PX):
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


def find_square_by_clipped_parts(img, paper):
    if not SQUARE_CLIPPED_FALLBACK_ENABLED:
        return None

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
        if not box_in_paper(x, y, w, h, paper, -SQUARE_CLIPPED_PAPER_RELAX_PX):
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < SQUARE_MIN_DENSITY or density > 98.0:
            continue
        parts.append((x, y, w, h, pixels, area))

    if len(parts) < SQUARE_CLIPPED_MIN_PARTS:
        return None

    x0 = min([p[0] for p in parts])
    y0 = min([p[1] for p in parts])
    x1 = max([p[0] + p[2] for p in parts])
    y1 = max([p[1] + p[3] for p in parts])
    w = x1 - x0
    h = y1 - y0
    area = w * h
    pixels = sum([p[4] for p in parts])

    if pixels < SQUARE_CLIPPED_MIN_PIXELS:
        return None
    if w < SQUARE_CLIPPED_MIN_W or h < SQUARE_CLIPPED_MIN_H:
        return None
    if w > SQUARE_MAX_W or h > SQUARE_MAX_H:
        return None
    if area < SQUARE_MIN_AREA or area > SQUARE_MAX_AREA:
        return None
    aspect = w / h if h else 0.0
    if aspect < SQUARE_MIN_ASPECT or aspect > SQUARE_MAX_ASPECT:
        return None
    if not box_in_paper(x0, y0, w, h, paper, -SQUARE_CLIPPED_PAPER_RELAX_PX):
        return None

    density = pixels * 100.0 / area if area else 0.0
    if density < SQUARE_MIN_DENSITY or density > SQUARE_MAX_DENSITY:
        return None

    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    score = int(pixels * 5 + area * 0.15 + len(parts) * 90)
    return ("clipped", cx, cy, w, h, corners, score)


def valid_red_panel_frame_box(x, y, w, h, area):
    if w < RED_PANEL_FRAME_MIN_W or h < RED_PANEL_FRAME_MIN_H:
        return False
    if w > RED_PANEL_FRAME_MAX_W or h > RED_PANEL_FRAME_MAX_H:
        return False
    if area < RED_PANEL_FRAME_MIN_AREA or area > RED_PANEL_FRAME_MAX_AREA:
        return False
    aspect = w / h if h else 0.0
    if aspect < RED_PANEL_FRAME_MIN_ASPECT or aspect > RED_PANEL_FRAME_MAX_ASPECT:
        return False
    if x < 0 or y < 0 or x + w > FRAME_W or y + h > FRAME_H:
        return False
    return True


def find_red_panel_frame(img, paper):
    if not RED_PANEL_FRAME_ENABLED:
        return None

    best = None
    best_score = -1

    try:
        rects = img.find_rects(threshold=SQUARE_RECT_THRESHOLD)
    except Exception:
        rects = []

    for r in rects:
        corners = rect_corners(r)
        x, y, w, h = rect_bounds_from_corners(corners)
        area = polygon_area(corners)
        if not valid_red_panel_frame_box(x, y, w, h, area):
            continue
        if not box_in_paper(x, y, w, h, paper, -RED_PANEL_FRAME_PAPER_RELAX_PX):
            continue
        cx = sum([p[0] for p in corners]) / 4.0
        cy = sum([p[1] for p in corners]) / 4.0
        try:
            mag = int(r.magnitude())
        except Exception:
            mag = int(area)
        squareness = min(w, h) / max(w, h) if max(w, h) else 0.0
        edge_margin = min(cx, cy, FRAME_W - cx, FRAME_H - cy)
        score = int(area * 0.9 + mag * 3 + squareness * 1000 + edge_margin)
        if score > best_score:
            best = ("red_panel_rect", cx, cy, w, h, corners, score)
            best_score = score

    try:
        blobs = img.find_blobs(
            [SQUARE_THRESHOLD],
            pixels_threshold=RED_PANEL_FRAME_MIN_PIXELS,
            area_threshold=RED_PANEL_FRAME_MIN_AREA,
            merge=True,
            margin=RED_PANEL_FRAME_MERGE_MARGIN,
        )
    except Exception:
        blobs = []

    for b in blobs:
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        if not valid_red_panel_frame_box(x, y, w, h, area):
            continue
        if not box_in_paper(x, y, w, h, paper, -RED_PANEL_FRAME_PAPER_RELAX_PX):
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < RED_PANEL_FRAME_MIN_DENSITY or density > RED_PANEL_FRAME_MAX_DENSITY:
            continue
        cx, cy = blob_center(b)
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        squareness = min(w, h) / max(w, h) if max(w, h) else 0.0
        score = int(pixels * 6 + area * 0.30 + squareness * 1000)
        if score > best_score:
            best = ("red_panel_blob", cx, cy, w, h, corners, score)
            best_score = score

    return best


def find_center_dot_in_bounds(
    blobs,
    center_x,
    center_y,
    x0,
    y0,
    x1,
    y1,
    paper,
    max_center_dist=None,
):
    best = None
    best_score = -999999
    x0, y0, x1, y1 = clamp_bounds_to_paper(x0, y0, x1, y1, paper)
    if x1 <= x0 or y1 <= y0:
        return None
    search_w = x1 - x0
    search_h = y1 - y0
    if max_center_dist is None:
        max_center_dist = max(
            CENTER_DOT_MIN_CENTER_DIST_PX,
            min(CENTER_DOT_MAX_CENTER_DIST_PX, min(search_w, search_h) * CENTER_DOT_MAX_CENTER_RATIO),
        )
    max_center_dist2 = max_center_dist * max_center_dist
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
        if center_dist2 > max_center_dist2:
            continue
        edge_dist = min(cx - x0, cy - y0, x1 - cx, y1 - cy)
        if edge_dist < 1.0:
            continue
        roundness = min(w, h) / max(w, h) if max(w, h) else 0.0
        center_penalty = center_dist2 * 1.8
        edge_bonus = min(edge_dist, 24.0) * 12.0
        score = int(pixels * 7 + density * 4 + roundness * 450.0 + edge_bonus - area * 0.10 - center_penalty)
        if score > best_score:
            corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
            best = ("dot", cx, cy, w, h, corners, score)
            best_score = score
    if best_score < CENTER_DOT_ACCEPT_MIN_SCORE:
        return None
    return best


def center_dot_matches_paper_center(dot, paper):
    if not dot or not paper:
        return True
    _src, cx, cy, _w, _h, _corners, _score = dot
    px0, py0, px1, py1 = paper_bounds(paper)
    pw = px1 - px0
    ph = py1 - py0
    if pw <= 0.0 or ph <= 0.0:
        return True
    pcx = (px0 + px1) / 2.0
    pcy = (py0 + py1) / 2.0
    max_dist = max(
        CENTER_DOT_PAPER_MIN_CENTER_DIST_PX,
        min(CENTER_DOT_PAPER_MAX_CENTER_DIST_PX, min(pw, ph) * CENTER_DOT_PAPER_CENTER_RATIO),
    )
    dx = cx - pcx
    dy = cy - pcy
    return (dx * dx + dy * dy) <= (max_dist * max_dist)


def find_center_dot(img, square, laser, paper):
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

        if paper:
            px0, py0, px1, py1 = paper_bounds(paper)
            pcx = (px0 + px1) / 2.0
            pcy = (py0 + py1) / 2.0
            pw = px1 - px0
            ph = py1 - py0
            max_dist = max(
                CENTER_DOT_PAPER_MIN_CENTER_DIST_PX,
                min(CENTER_DOT_PAPER_MAX_CENTER_DIST_PX, min(pw, ph) * CENTER_DOT_PAPER_CENTER_RATIO),
            )
            dot = find_center_dot_in_bounds(
                blobs,
                pcx,
                pcy,
                pcx - max_dist,
                pcy - max_dist,
                pcx + max_dist,
                pcy + max_dist,
                paper,
                max_dist,
            )
            if dot:
                return dot

        if square:
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
            if dot and center_dot_matches_paper_center(dot, paper):
                return dot

        if CENTER_DOT_LASER_FALLBACK_ENABLED and laser:
            _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser
            max_dist = CENTER_DOT_MAX_LASER_DIST_PX
            dot = find_center_dot_in_bounds(
                blobs,
                l_cx,
                l_cy,
                max(SQUARE_EDGE_MARGIN, l_cx - max_dist),
                max(SQUARE_EDGE_MARGIN, l_cy - max_dist),
                min(FRAME_W - SQUARE_EDGE_MARGIN, l_cx + max_dist),
                min(FRAME_H - SQUARE_EDGE_MARGIN, l_cy + max_dist),
                paper,
                max_dist,
            )
            if dot and center_dot_matches_paper_center(dot, paper):
                return dot

    return None


def square_center_target(square):
    if not FRAME_CENTER_FALLBACK_ENABLED or not square:
        return None
    src, cx, cy, w, h, corners, score = square
    aspect = w / h if h else 0.0
    if (
        src == "clipped"
        or w < FRAME_CENTER_MIN_W
        or h < FRAME_CENTER_MIN_H
        or score < FRAME_CENTER_MIN_SCORE
        or aspect < FRAME_CENTER_MIN_ASPECT
        or aspect > FRAME_CENTER_MAX_ASPECT
        or cx < FRAME_CENTER_MIN_EDGE_PX
        or cx > FRAME_W - FRAME_CENTER_MIN_EDGE_PX
        or cy < FRAME_CENTER_MIN_EDGE_PX
        or cy > FRAME_H - FRAME_CENTER_MIN_EDGE_PX
    ):
        if aspect > FRAME_PART_MAX_ASPECT or aspect < (1.0 / FRAME_PART_MAX_ASPECT):
            return None
        if cx < FRAME_PART_MIN_EDGE_PX or cx > FRAME_W - FRAME_PART_MIN_EDGE_PX:
            return None
        if cy < FRAME_PART_MIN_EDGE_PX or cy > FRAME_H - FRAME_PART_MIN_EDGE_PX:
            return None
        return ("frame_part", cx, cy, w, h, corners, score)
    return ("frame_center", cx, cy, w, h, corners, score)


def frame_part_recenter_target(target, laser):
    if not FRAME_PART_RECENTER_ENABLED or not target or not laser:
        return target
    if target[0] != "frame_part":
        return target

    src, part_cx, part_cy, w, h, corners, score = target
    _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser
    dx = part_cx - FRAME_W / 2.0
    dy = part_cy - FRAME_H / 2.0
    if abs(dx) < FRAME_PART_RECENTER_DEADBAND_PX:
        dx = 0.0
    if abs(dy) < FRAME_PART_RECENTER_DEADBAND_PX:
        dy = 0.0
    if abs(dx) > abs(dy) * FRAME_PART_RECENTER_MAJOR_AXIS_RATIO:
        dy = 0.0
    elif abs(dy) > abs(dx) * FRAME_PART_RECENTER_MAJOR_AXIS_RATIO:
        dx = 0.0
    aim_cx = l_cx - dx * FRAME_PART_RECENTER_GAIN
    aim_cy = l_cy - dy * FRAME_PART_RECENTER_GAIN
    aim_cx = clamp(aim_cx, 0.0, FRAME_W - 1.0)
    aim_cy = clamp(aim_cy, 0.0, FRAME_H - 1.0)
    return (src, aim_cx, aim_cy, w, h, corners, score)


def paper_center_target(paper):
    if not PAPER_CENTER_FALLBACK_ENABLED or not paper:
        return None
    _src, x, y, w, h, _pixels, score = paper
    cx = x + w / 2.0
    cy = y + h / 2.0
    corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    return ("paper_center", cx, cy, w, h, corners, score)


def fixed_laser_ref():
    return ("fixed", FIXED_LASER_CX, FIXED_LASER_CY, 4, 4, 999, 999)


def target_stability_src(src):
    return src


def target_hold_allowed(target):
    if not target:
        return False
    return (
        target[0] == "red_panel_center"
        or target[0] == "red_rings"
        or target[0] == "red_rings_part"
        or target[0] == "frame_center"
    )


def find_red_rings_target(img, paper, laser=None, panel=None):
    if not RED_RINGS_TARGET_ENABLED:
        return None
    if not paper:
        return None

    debug = RED_RINGS_DEBUG or DIAGNOSE_ONCE or TARGET_CANDIDATE_DEBUG
    total = 0
    kept = 0
    reject_pixels = 0
    reject_area = 0
    reject_paper = 0
    reject_density = 0
    max_pixels = 0.0
    max_area = 0.0
    max_density = 0.0
    max_x = 0.0
    max_y = 0.0
    max_w = 0.0
    max_h = 0.0

    try:
        blobs = img.find_blobs(
            RED_RINGS_THRESHOLDS,
            pixels_threshold=RED_RINGS_MIN_PART_PIXELS,
            area_threshold=RED_RINGS_MIN_PART_AREA,
            merge=False,
            margin=0,
        )
    except Exception:
        return None

    parts = []
    gate_margin = RED_PANEL_FRAME_ROI_INSET_PX if panel else -RED_RINGS_PAPER_RELAX_PX
    for b in blobs:
        total += 1
        x = float(value(b, 0))
        y = float(value(b, 1))
        w = float(value(b, 2))
        h = float(value(b, 3))
        pixels = float(value(b, 4, w * h))
        area = w * h
        density = pixels * 100.0 / area if area else 0.0
        if pixels > max_pixels:
            max_pixels = pixels
            max_area = area
            max_density = density
            max_x = x
            max_y = y
            max_w = w
            max_h = h
        if pixels < RED_RINGS_MIN_PART_PIXELS:
            reject_pixels += 1
            continue
        if area < RED_RINGS_MIN_PART_AREA or area > RED_RINGS_MAX_PART_AREA:
            reject_area += 1
            continue
        if panel:
            inside_gate = box_in_target_bounds(x, y, w, h, panel, gate_margin)
        else:
            inside_gate = box_in_paper(x, y, w, h, paper, gate_margin)
        if not inside_gate:
            reject_paper += 1
            continue
        if density < RED_RINGS_MIN_PART_DENSITY or density > RED_RINGS_MAX_PART_DENSITY:
            reject_density += 1
            continue
        if area >= RED_RINGS_SOLID_PART_MIN_AREA and density > RED_RINGS_SOLID_PART_MAX_DENSITY:
            reject_density += 1
            continue
        kept += 1
        parts.append((x, y, w, h, pixels, area))

    if len(parts) < RED_RINGS_MIN_PARTS:
        if debug:
            print(
                "$RINGDBG,total=%d,kept=%d,reason=parts,max_x=%d,max_y=%d,max_w=%d,max_h=%d,"
                "max_pix=%d,max_area=%d,max_den=%.1f,rej_pix=%d,rej_area=%d,rej_paper=%d,rej_den=%d"
                % (
                    total,
                    kept,
                    int(max_x),
                    int(max_y),
                    int(max_w),
                    int(max_h),
                    int(max_pixels),
                    int(max_area),
                    max_density,
                    reject_pixels,
                    reject_area,
                    reject_paper,
                    reject_density,
                )
            )
        return None

    x0 = min([p[0] for p in parts])
    y0 = min([p[1] for p in parts])
    x1 = max([p[0] + p[2] for p in parts])
    y1 = max([p[1] + p[3] for p in parts])
    w = x1 - x0
    h = y1 - y0
    total_pixels = sum([p[4] for p in parts])
    area = w * h
    union_density = total_pixels * 100.0 / area if area else 0.0
    if w < RED_RINGS_MIN_W or h < RED_RINGS_MIN_H:
        if debug:
            print("$RINGDBG,total=%d,kept=%d,reason=size,x=%d,y=%d,w=%d,h=%d,pix=%d" %
                  (total, kept, int(x0), int(y0), int(w), int(h), int(total_pixels)))
        return None
    if w > RED_RINGS_MAX_W or h > RED_RINGS_MAX_H:
        if debug:
            print("$RINGDBG,total=%d,kept=%d,reason=max_size,x=%d,y=%d,w=%d,h=%d,pix=%d" %
                  (total, kept, int(x0), int(y0), int(w), int(h), int(total_pixels)))
        return None
    aspect = w / h if h else 0.0
    if aspect < RED_RINGS_MIN_ASPECT or aspect > RED_RINGS_MAX_ASPECT:
        if debug:
            print("$RINGDBG,total=%d,kept=%d,reason=aspect,x=%d,y=%d,w=%d,h=%d,aspect=%.2f,pix=%d" %
                  (total, kept, int(x0), int(y0), int(w), int(h), aspect, int(total_pixels)))
        return None

    if total_pixels < RED_RINGS_MIN_TOTAL_PIXELS:
        if debug:
            print("$RINGDBG,total=%d,kept=%d,reason=pixels,x=%d,y=%d,w=%d,h=%d,pix=%d" %
                  (total, kept, int(x0), int(y0), int(w), int(h), int(total_pixels)))
        return None
    if union_density < RED_RINGS_UNION_MIN_DENSITY or union_density > RED_RINGS_UNION_MAX_DENSITY:
        if debug:
            print("$RINGDBG,total=%d,kept=%d,reason=union_density,x=%d,y=%d,w=%d,h=%d,pix=%d,den=%.1f" %
                  (total, kept, int(x0), int(y0), int(w), int(h), int(total_pixels), union_density))
        return None

    if panel:
        _p_src, panel_cx, panel_cy, panel_w, panel_h, _panel_corners, _panel_score = panel
        panel_min_side = min(panel_w, panel_h)
        if panel_min_side > 0:
            max_offset = max(8.0, panel_min_side * 0.32)
            center_dx = (x0 + w / 2.0) - panel_cx
            center_dy = (y0 + h / 2.0) - panel_cy
            if (center_dx * center_dx + center_dy * center_dy) > (max_offset * max_offset):
                if debug:
                    print("$RINGDBG,total=%d,kept=%d,reason=panel_center,x=%d,y=%d,w=%d,h=%d,pcx=%.1f,pcy=%.1f" %
                          (total, kept, int(x0), int(y0), int(w), int(h), panel_cx, panel_cy))
                return None

    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    complete = True
    if w < RED_RINGS_COMPLETE_MIN_W or h < RED_RINGS_COMPLETE_MIN_H:
        complete = False
    if aspect < RED_RINGS_COMPLETE_MIN_ASPECT or aspect > RED_RINGS_COMPLETE_MAX_ASPECT:
        complete = False
    if x0 <= RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
        complete = False
    if y0 <= RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
        complete = False
    if x1 >= FRAME_W - RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
        complete = False
    if y1 >= FRAME_H - RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
        complete = False

    if panel and not complete and RED_PANEL_CENTER_FALLBACK_ENABLED:
        if total_pixels >= RED_PANEL_CENTER_MIN_RED_PIXELS and len(parts) >= RED_PANEL_CENTER_MIN_RED_PARTS:
            _p_src, p_cx, p_cy, p_w, p_h, p_corners, p_score = panel
            src = "red_panel_center"
            red_x0 = x0
            red_y0 = y0
            red_w = w
            red_h = h
            cx = p_cx
            cy = p_cy
            w = p_w
            h = p_h
            corners = p_corners
            score = int(p_score + total_pixels * 10 + len(parts) * 80)
            if debug:
                print("$RINGDBG,total=%d,kept=%d,src=%s,cx=%.1f,cy=%.1f,red_x=%d,red_y=%d,red_w=%d,red_h=%d,w=%d,h=%d,pix=%d,den=%.1f" %
                      (total, kept, src, cx, cy, int(red_x0), int(red_y0), int(red_w), int(red_h), int(w), int(h), int(total_pixels), union_density))
            return (src, cx, cy, w, h, corners, score)

    src = "red_rings" if complete else "red_rings_part"
    if src == "red_rings_part":
        edge_margin = RED_RINGS_PART_EDGE_COMMAND_MARGIN_PX
        near_edge = (
            x0 <= edge_margin
            or y0 <= edge_margin
            or x1 >= FRAME_W - edge_margin
            or y1 >= FRAME_H - edge_margin
        )
        if not near_edge and (h <= 3.0 or w <= 3.0 or len(parts) < 2):
            if debug:
                print("$RINGDBG,total=%d,kept=%d,reason=thin_not_edge,x=%d,y=%d,w=%d,h=%d,pix=%d" %
                      (total, kept, int(x0), int(y0), int(w), int(h), int(total_pixels)))
            return None
    if src == "red_rings_part" and laser and RED_RINGS_PART_RECENTER_ENABLED:
        _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser
        recover_x = 0.0
        recover_y = 0.0
        if x0 <= RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
            recover_x += FRAME_W * 0.28
        if x1 >= FRAME_W - RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
            recover_x -= FRAME_W * 0.28
        if y0 <= RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
            recover_y += FRAME_H * 0.28
        if y1 >= FRAME_H - RED_RINGS_COMPLETE_EDGE_MARGIN_PX:
            recover_y -= FRAME_H * 0.28
        if recover_x == 0.0 and recover_y == 0.0:
            recover_x = (FRAME_W / 2.0 - cx) * RED_RINGS_PART_RECENTER_GAIN
            recover_y = (FRAME_H / 2.0 - cy) * RED_RINGS_PART_RECENTER_GAIN
        cx = clamp(l_cx + recover_x * RED_RINGS_PART_EDGE_GAIN, 0.0, FRAME_W - 1.0)
        cy = clamp(l_cy + recover_y * RED_RINGS_PART_EDGE_GAIN, 0.0, FRAME_H - 1.0)

    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    score = int(total_pixels * 12 + area * 0.08 + len(parts) * 120 - abs(aspect - 1.0) * 400.0)
    if debug:
        print("$RINGDBG,total=%d,kept=%d,src=%s,cx=%.1f,cy=%.1f,x=%d,y=%d,w=%d,h=%d,pix=%d,den=%.1f" %
              (total, kept, src, cx, cy, int(x0), int(y0), int(w), int(h), int(total_pixels), union_density))
    return (src, cx, cy, w, h, corners, score)


def red_rings_part_recenter_target(target, laser):
    if (
        not RED_RINGS_PART_RECENTER_ENABLED
        or not target
        or not laser
        or target[0] != "red_rings_part"
    ):
        return target

    src, _cx, _cy, w, h, corners, score = target
    _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser
    x0, y0, bw, bh = rect_bounds_from_corners(corners)
    x1 = x0 + bw
    y1 = y0 + bh

    recover_x = 0.0
    recover_y = 0.0
    left_depth = max(0.0, RED_RINGS_COMPLETE_EDGE_MARGIN_PX - x0)
    right_depth = max(0.0, x1 - (FRAME_W - RED_RINGS_COMPLETE_EDGE_MARGIN_PX))
    top_depth = max(0.0, RED_RINGS_COMPLETE_EDGE_MARGIN_PX - y0)
    bottom_depth = max(0.0, y1 - (FRAME_H - RED_RINGS_COMPLETE_EDGE_MARGIN_PX))
    vertical_depth = max(top_depth, bottom_depth)
    horizontal_depth = max(left_depth, right_depth)
    if vertical_depth > 0.0 and vertical_depth >= horizontal_depth * 0.75:
        if top_depth >= bottom_depth:
            recover_y = FRAME_H * 0.28
        else:
            recover_y = -FRAME_H * 0.28
    elif left_depth >= right_depth and left_depth > 0.0:
        recover_x = FRAME_W * 0.28
    elif right_depth > 0.0:
        recover_x = -FRAME_W * 0.28
    elif top_depth > 0.0:
        recover_y = FRAME_H * 0.28
    elif bottom_depth > 0.0:
        recover_y = -FRAME_H * 0.28
    if recover_x == 0.0 and recover_y == 0.0:
        ring_cx = x0 + bw / 2.0
        ring_cy = y0 + bh / 2.0
        recover_x = (FRAME_W / 2.0 - ring_cx) * RED_RINGS_PART_RECENTER_GAIN
        recover_y = (FRAME_H / 2.0 - ring_cy) * RED_RINGS_PART_RECENTER_GAIN

    cx = clamp(l_cx + recover_x * RED_RINGS_PART_EDGE_GAIN, 0.0, FRAME_W - 1.0)
    cy = clamp(l_cy + recover_y * RED_RINGS_PART_EDGE_GAIN, 0.0, FRAME_H - 1.0)
    return (src, cx, cy, w, h, corners, score)


def red_panel_center_target(panel):
    if (
        not RED_PANEL_RECT_CENTER_PRIORITY_ENABLED
        or not panel
        or panel[0] != "red_panel_rect"
    ):
        return None
    _src, cx, cy, w, h, corners, score = panel
    return ("red_panel_center", cx, cy, w, h, corners, score)


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


def find_square_target(img, laser, paper):
    panel = find_red_panel_frame(img, paper)
    panel_target = red_panel_center_target(panel)
    if panel_target:
        return panel_target

    red_target = find_red_rings_target(img, paper, laser, panel)
    if not red_target and not panel:
        red_target = find_red_rings_target(img, paper, laser, None)
    if red_target:
        if red_target[0] == "red_rings_part" and not RED_RINGS_PART_CONTROL_ENABLED:
            return None
        return red_target

    if not BLACK_FRAME_FALLBACK_ENABLED:
        if RED_RINGS_TARGET_ENABLED and not RED_RINGS_PAPER_CENTER_FALLBACK_ENABLED:
            return None
        return paper_center_target(paper)

    target = None
    if RECT_TARGET_ENABLED and paper:
        target = find_square_by_rects(img, paper)
    if not target and paper:
        target = find_square_by_blobs(img, paper)
    if not target and paper:
        target = find_square_by_union(img, paper)
    if not target and paper:
        target = find_square_by_clipped_parts(img, paper)
    if RECT_TARGET_ENABLED and not target:
        target = find_square_by_rects(img, None)
    if not target:
        target = find_square_by_blobs(img, None)
    if not target:
        target = find_square_by_union(img, None)
    if target:
        return square_center_target(target)
    return paper_center_target(paper)



def find_laser(img, paper=None, hint_x=None, hint_y=None, hint_max_dist=None):
    try:
        blobs = img.find_blobs(
            [LASER_THRESHOLD, LASER_BRIGHT_THRESHOLD],
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
        if not box_in_paper(x, y, w, h, paper, -LASER_PAPER_RELAX_PX):
            continue
        aspect = w / h if h else 0.0
        if aspect < 0.25 or aspect > 3.50:
            continue
        roundness = min(w, h) / max(w, h) if max(w, h) else 0.0
        if roundness < LASER_MIN_ROUNDNESS:
            continue
        density = pixels * 100.0 / area if area else 0.0
        if density < LASER_MIN_DENSITY:
            continue
        cx, cy = blob_center(b)
        bright_pixels = 0
        bx0 = int(max(0, x))
        by0 = int(max(0, y))
        bx1 = int(min(FRAME_W, x + w))
        by1 = int(min(FRAME_H, y + h))
        try:
            bright_blobs = img.find_blobs(
                [LASER_BRIGHT_THRESHOLD],
                roi=(bx0, by0, max(1, bx1 - bx0), max(1, by1 - by0)),
                pixels_threshold=1,
                area_threshold=1,
                merge=True,
                margin=1,
            )
            for bb in bright_blobs:
                bright_pixels += int(value(bb, 4, 0))
        except Exception:
            bright_pixels = int(pixels)
        if bright_pixels < LASER_MIN_BRIGHT_PIXELS:
            continue
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
        size_score = min(area, 180.0) * 1.5
        score = int(bright_pixels * 40 + pixels * 4 + density * 2 + roundness * 500 + size_score + center_bonus + hint_bonus - area * 0.2)
        if score > best_score:
            best = ("red", cx, cy, w, h, pixels, score)
            best_score = score
    return best


def rotate_camera_error_to_body(dx, dy):
    if CAMERA_ROTATION_CW_DEG == 90:
        # The camera image is rotated clockwise on the gimbal: image up is
        # physical right. Convert image axes back to the gimbal/body axes.
        return -dy, dx
    if CAMERA_ROTATION_CW_DEG == 180:
        return -dx, -dy
    if CAMERA_ROTATION_CW_DEG == 270:
        return dy, -dx
    return dx, dy


def pixel_delta_to_angle(dx, dy):
    fx = FRAME_W / (2.0 * math.tan(math.radians(HFOV_DEG) / 2.0))
    fy = FRAME_H / (2.0 * math.tan(math.radians(VFOV_DEG) / 2.0))
    yaw = ERROR_YAW_SIGN * math.degrees(math.atan(dx / fx))
    pitch = ERROR_PITCH_SIGN * math.degrees(math.atan(dy / fy))
    return yaw, pitch


def shape_axis_command(angle, err, deadband, max_deg):
    if abs(err) <= deadband:
        return 0.0
    return clamp(angle * CONTROL_GAIN, -max_deg, max_deg)


def shape_command_angle(yaw, pitch, err_x, err_y, target=None):
    yaw_deadband = ERROR_DEADBAND_PX
    pitch_deadband = ERROR_DEADBAND_PX
    yaw_max = CONTROL_MAX_DEG
    pitch_max = CONTROL_MAX_DEG

    if target and target[0] == "frame_center":
        yaw_deadband = FRAME_CENTER_DEADBAND_PX
        pitch_deadband = FRAME_CENTER_DEADBAND_PX
        if abs(err_x) <= FRAME_CENTER_FINE_RADIUS_PX:
            yaw_max = FRAME_CENTER_FINE_CONTROL_MAX_DEG
        if abs(err_y) <= FRAME_CENTER_FINE_RADIUS_PX:
            pitch_max = FRAME_CENTER_FINE_CONTROL_MAX_DEG
    elif target and target[0] == "red_rings":
        yaw_deadband = RED_RINGS_DEADBAND_PX
        pitch_deadband = RED_RINGS_DEADBAND_PX
        yaw_max = RED_RINGS_SAFE_CONTROL_MAX_DEG
        pitch_max = RED_RINGS_SAFE_CONTROL_MAX_DEG
        if abs(err_x) <= RED_RINGS_FINE_RADIUS_PX:
            yaw_max = RED_RINGS_FINE_CONTROL_MAX_DEG
        if abs(err_y) <= RED_RINGS_FINE_RADIUS_PX:
            pitch_max = RED_RINGS_FINE_CONTROL_MAX_DEG
    elif target and target[0] == "red_panel_center":
        yaw_deadband = RED_PANEL_CENTER_X_DEADBAND_PX
        pitch_deadband = RED_PANEL_CENTER_DEADBAND_PX
        yaw_max = RED_PANEL_CENTER_CONTROL_MAX_DEG
        pitch_max = RED_PANEL_CENTER_CONTROL_MAX_DEG
        if abs(err_x) <= RED_PANEL_CENTER_FINE_RADIUS_PX:
            yaw_max = RED_PANEL_CENTER_CONTROL_MAX_DEG
        if abs(err_y) <= RED_PANEL_CENTER_FINE_RADIUS_PX:
            pitch_max = RED_PANEL_CENTER_CONTROL_MAX_DEG
    elif target and target[0] == "red_rings_part":
        yaw_deadband = RED_RINGS_PART_DEADBAND_PX
        pitch_deadband = RED_RINGS_PART_DEADBAND_PX
        if abs(err_x) <= RED_RINGS_PART_FINE_RADIUS_PX:
            yaw_max = RED_RINGS_PART_FINE_CONTROL_MAX_DEG
        if abs(err_y) <= RED_RINGS_PART_FINE_RADIUS_PX:
            pitch_max = RED_RINGS_PART_FINE_CONTROL_MAX_DEG

    yaw = shape_axis_command(yaw, err_x, yaw_deadband, yaw_max)
    pitch = shape_axis_command(pitch, err_y, pitch_deadband, pitch_max)

    return yaw, pitch


def map_to_motor_commands(image_yaw, image_pitch):
    # Observed wiring with the camera mounted 90 degrees clockwise:
    # MSPM0 yaw channel corrects image vertical error, MSPM0 pitch channel
    # corrects image horizontal error with inverted polarity.
    motor_yaw = image_pitch
    motor_pitch = -image_yaw
    return motor_yaw, motor_pitch


def limit_paper_center_command(target, yaw, pitch):
    if target and target[0] == "paper_center":
        yaw = clamp(yaw, -PAPER_CENTER_CONTROL_MAX_DEG, PAPER_CENTER_CONTROL_MAX_DEG)
        pitch = clamp(pitch, -PAPER_CENTER_CONTROL_MAX_DEG, PAPER_CENTER_CONTROL_MAX_DEG)
    elif target and target[0] == "frame_part":
        yaw = clamp(yaw, -FRAME_PART_CONTROL_MAX_DEG, FRAME_PART_CONTROL_MAX_DEG)
        pitch = clamp(pitch, -FRAME_PART_CONTROL_MAX_DEG, FRAME_PART_CONTROL_MAX_DEG)
        yaw = -yaw
        pitch = -pitch
    elif target and (target[0] == "red_rings" or target[0] == "red_panel_center"):
        yaw = -yaw
        pitch = -pitch
        if target[0] == "red_panel_center":
            yaw = clamp(yaw, -RED_PANEL_CENTER_CONTROL_MAX_DEG, RED_PANEL_CENTER_CONTROL_MAX_DEG)
            pitch = clamp(pitch, -RED_PANEL_CENTER_CONTROL_MAX_DEG, RED_PANEL_CENTER_CONTROL_MAX_DEG)
    elif target and target[0] == "red_rings_part":
        yaw = clamp(yaw, -RED_RINGS_PART_CONTROL_MAX_DEG, RED_RINGS_PART_CONTROL_MAX_DEG)
        pitch = clamp(pitch, -RED_RINGS_PART_CONTROL_MAX_DEG, RED_RINGS_PART_CONTROL_MAX_DEG)
        yaw = -yaw
        pitch = -pitch
    return yaw, pitch


def red_rings_part_edge_command(target):
    if (
        not RED_RINGS_PART_EDGE_COMMAND_ENABLED
        or not target
        or target[0] != "red_rings_part"
    ):
        return None

    _src, _cx, _cy, _w, _h, corners, _score = target
    x0, y0, bw, bh = rect_bounds_from_corners(corners)
    x1 = x0 + bw
    y1 = y0 + bh
    margin = RED_RINGS_PART_EDGE_COMMAND_MARGIN_PX
    left_clip = x0 <= margin
    right_clip = x1 >= FRAME_W - margin
    top_clip = y0 <= margin
    bottom_clip = y1 >= FRAME_H - margin

    yaw = 0.0
    pitch = 0.0
    # Directly recover from clipped image edges. These signs are calibrated
    # for the current 90-degree clockwise camera mount and MSPM0 gimbal wiring.
    if bottom_clip and not top_clip:
        yaw = RED_RINGS_PART_CONTROL_MAX_DEG
    elif top_clip and not bottom_clip:
        yaw = -RED_RINGS_PART_CONTROL_MAX_DEG

    if right_clip and not left_clip:
        pitch = RED_RINGS_PART_CONTROL_MAX_DEG
    elif left_clip and not right_clip:
        pitch = -RED_RINGS_PART_CONTROL_MAX_DEG

    if yaw == 0.0 and pitch == 0.0:
        return None

    return yaw, pitch


def compute_motor_command(target, laser, err_x, err_y):
    if (
        laser
        and laser[0] == "fixed"
        and target
        and (target[0] == "red_rings" or target[0] == "red_rings_part" or target[0] == "red_panel_center")
    ):
        raw_yaw, raw_pitch = pixel_delta_to_angle(err_x, err_y)
        img_yaw, img_pitch = shape_command_angle(raw_yaw, raw_pitch, err_x, err_y, target)
        motor_yaw = -img_pitch
        motor_pitch = -img_yaw
        edge_command = red_rings_part_edge_command(target)
        if edge_command:
            motor_yaw, motor_pitch = edge_command
        if target[0] == "red_rings_part":
            motor_yaw = clamp(motor_yaw, -RED_RINGS_PART_CONTROL_MAX_DEG, RED_RINGS_PART_CONTROL_MAX_DEG)
            motor_pitch = clamp(motor_pitch, -RED_RINGS_PART_CONTROL_MAX_DEG, RED_RINGS_PART_CONTROL_MAX_DEG)
        return err_x, err_y, raw_yaw, raw_pitch, img_yaw, img_pitch, motor_yaw, motor_pitch

    body_err_x, body_err_y = rotate_camera_error_to_body(err_x, err_y)
    raw_yaw, raw_pitch = pixel_delta_to_angle(body_err_x, body_err_y)
    yaw, pitch = shape_command_angle(raw_yaw, raw_pitch, body_err_x, body_err_y, target)
    motor_yaw, motor_pitch = map_to_motor_commands(yaw, pitch)
    motor_yaw, motor_pitch = limit_paper_center_command(target, motor_yaw, motor_pitch)
    return body_err_x, body_err_y, raw_yaw, raw_pitch, yaw, pitch, motor_yaw, motor_pitch


def apply_pitch_guard(target, motor_yaw, motor_pitch):
    if not PITCH_GUARD_ENABLED or not target:
        return motor_yaw, motor_pitch
    if target[0] == "red_rings_part":
        return motor_yaw, motor_pitch

    _src, _cx, cy, _w, _h, _corners, _score = target
    if cy <= PITCH_GUARD_MIN_TARGET_CY:
        if motor_yaw * PITCH_GUARD_UP_CMD_SIGN > 0.0:
            motor_yaw = 0.0
    elif cy <= PITCH_GUARD_SOFT_TARGET_CY:
        if motor_yaw * PITCH_GUARD_UP_CMD_SIGN > PITCH_GUARD_MAX_UP_CMD:
            motor_yaw = PITCH_GUARD_UP_CMD_SIGN * PITCH_GUARD_MAX_UP_CMD
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


def diagnose_once():
    sensor = None
    try:
        sensor = setup_camera()
        img = sensor.snapshot(chn=CAPTURE_CHN)
        try:
            img.save(DIAG_CAPTURE_PATH)
            print("$DIAG_CAPTURE,%s" % DIAG_CAPTURE_PATH)
        except Exception as exc:
            print("$DIAG_CAPTURE_FAIL,%s" % exc)

        paper = find_paper(img, None)
        if FIXED_LASER_ENABLED:
            laser = fixed_laser_ref()
        else:
            laser = find_laser(img, paper, None, None, None)
        target = None
        panel = None

        if paper:
            panel = find_red_panel_frame(img, paper)
            target = find_square_target(img, laser, paper)
            target = frame_part_recenter_target(target, laser)
            target = red_rings_part_recenter_target(target, laser)
        else:
            panel = find_red_panel_frame(img, None)
            target = find_square_target(img, laser, None)
            target = frame_part_recenter_target(target, laser)
            target = red_rings_part_recenter_target(target, laser)

        if paper:
            _p_src, p_x, p_y, p_w, p_h, _p_pixels, p_score = paper
            print(
                "$DIAG_PAPER,ok=1,x=%d,y=%d,w=%d,h=%d,score=%d"
                % (int(p_x), int(p_y), int(p_w), int(p_h), int(p_score))
            )
        else:
            print("$DIAG_PAPER,ok=0")

        if panel:
            _panel_src, panel_cx, panel_cy, panel_w, panel_h, _panel_corners, panel_score = panel
            print(
                "$DIAG_PANEL,ok=1,cx=%.1f,cy=%.1f,w=%d,h=%d,score=%d"
                % (panel_cx, panel_cy, int(panel_w), int(panel_h), int(panel_score))
            )
        else:
            print("$DIAG_PANEL,ok=0")

        if target:
            t_src, t_cx, t_cy, t_w, t_h, _corners, t_score = target
            print(
                "$DIAG_TARGET,ok=1,src=%s,cx=%.1f,cy=%.1f,w=%d,h=%d,score=%d"
                % (t_src, t_cx, t_cy, int(t_w), int(t_h), int(t_score))
            )
        else:
            print("$DIAG_TARGET,ok=0")

        if laser:
            l_src, l_cx, l_cy, l_w, l_h, l_pixels, l_score = laser
            print(
                "$DIAG_LASER,ok=1,src=%s,cx=%.1f,cy=%.1f,w=%d,h=%d,pixels=%d,score=%d"
                % (l_src, l_cx, l_cy, int(l_w), int(l_h), int(l_pixels), int(l_score))
            )
        else:
            print("$DIAG_LASER,ok=0")

        if target and laser:
            err_x = l_cx - t_cx
            err_y = l_cy - t_cy
            body_err_x, body_err_y, raw_yaw, raw_pitch, yaw, pitch, motor_yaw, motor_pitch = compute_motor_command(
                target,
                laser,
                err_x,
                err_y,
            )
            motor_yaw, motor_pitch = apply_pitch_guard(target, motor_yaw, motor_pitch)
            print(
                "$DIAG_CMD,valid=1,err_x=%.1f,err_y=%.1f,body_err_x=%.1f,body_err_y=%.1f,"
                "raw_yaw=%.2f,raw_pitch=%.2f,img_yaw=%.2f,img_pitch=%.2f,cmd_yaw=%.2f,cmd_pitch=%.2f,"
                "would_send=$K230,1,%.2f,%.2f"
                % (
                    err_x,
                    err_y,
                    body_err_x,
                    body_err_y,
                    raw_yaw,
                    raw_pitch,
                    yaw,
                    pitch,
                    motor_yaw,
                    motor_pitch,
                    motor_yaw,
                    motor_pitch,
                )
            )
        else:
            print("$DIAG_CMD,valid=0,would_send=$K230,0,0.00,0.00")

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
    prev_target_src = None
    last_hold_target = None
    last_hold_ms = 0
    locked_frame_target = None
    locked_frame_age = 0
    last_lost_ms = 0
    last_control_ms = 0
    last_debug_ms = 0
    last_sent_valid = 0
    target_missing_count = 0
    laser_missing_count = 0
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

            paper = find_paper(img, None)
            if FIXED_LASER_ENABLED:
                laser = fixed_laser_ref()
            else:
                laser = find_laser(
                    img,
                    paper,
                    prev_lx,
                    prev_ly,
                    LASER_HINT_MAX_DIST_PX if prev_lx is not None else LASER_REACQUIRE_MAX_DIST_PX,
                )
            if paper:
                target = find_square_target(img, laser, paper)
                target = frame_part_recenter_target(target, laser)
                target = red_rings_part_recenter_target(target, laser)
            else:
                target = find_square_target(img, laser, None)
            target_rejected = False
            laser_rejected = False
            using_held_target = False

            if target and laser:
                _src0, _tcx0, _tcy0, _tw0, _th0, _corners0, _score0 = target
                _lsrc0, _lcx0, _lcy0, _lw0, _lh0, _lpix0, _lscore0 = laser
                if (
                    _src0 != "frame_center"
                    and _src0 != "red_rings"
                    and _src0 != "red_rings_part"
                    and _src0 != "red_panel_center"
                    and BLACK_FRAME_FALLBACK_ENABLED
                    and locked_frame_target
                    and locked_frame_age > 0
                ):
                    _lock_src, lock_cx, lock_cy, lock_w, lock_h, _lock_corners, _lock_score = locked_frame_target
                    if (
                        _lcx0 >= lock_cx - lock_w / 2.0 - FRAME_CENTER_LOCK_MARGIN_PX
                        and _lcx0 <= lock_cx + lock_w / 2.0 + FRAME_CENTER_LOCK_MARGIN_PX
                        and _lcy0 >= lock_cy - lock_h / 2.0 - FRAME_CENTER_LOCK_MARGIN_PX
                        and _lcy0 <= lock_cy + lock_h / 2.0 + FRAME_CENTER_LOCK_MARGIN_PX
                    ):
                        target = locked_frame_target
                        locked_frame_age -= 1

            if target and prev_tx is not None:
                _src, t_cx, t_cy, _w, _h, _corners, _score = target
                stable_src = target_stability_src(_src)
                max_target_jump = MAX_TARGET_JUMP_PX
                if stable_src == "red_panel_center":
                    max_target_jump = RED_PANEL_CENTER_MAX_JUMP_PX
                elif stable_src == "red_rings" or stable_src == "red_rings_part":
                    max_target_jump = RED_RINGS_MAX_CENTER_JUMP_PX
                jump_x = t_cx - prev_tx
                jump_y = t_cy - prev_ty
                if stable_src != "red_rings_part" and stable_src == prev_target_src and (
                    jump_x * jump_x + jump_y * jump_y
                ) > (max_target_jump * max_target_jump):
                    if USB_DEBUG_ENABLED:
                        print(
                            "$DBG_REJECT,kind=target_jump,cx=%.1f,cy=%.1f,prev_cx=%.1f,prev_cy=%.1f"
                            % (t_cx, t_cy, prev_tx, prev_ty)
                        )
                    target = None
                    target_rejected = True

            if laser and prev_lx is not None and laser[0] != "fixed":
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

            if not target and laser and last_hold_target:
                now = time.ticks_ms()
                if time.ticks_diff(now, last_hold_ms) <= TARGET_HOLD_MS:
                    target = last_hold_target
                    using_held_target = True

            if target and laser:
                target_missing_count = 0
                laser_missing_count = 0
                _t_src, t_cx, t_cy, _t_w, _t_h, _corners, _t_score = target
                _l_src, l_cx, l_cy, _l_w, _l_h, _l_pixels, _l_score = laser
                stable_t_src = target_stability_src(_t_src)

                if stable_t_src != prev_target_src:
                    target_samples = []
                    prev_tx = None
                    prev_ty = None
                    prev_target_src = stable_t_src

                if BLACK_FRAME_FALLBACK_ENABLED and _t_src == "frame_center":
                    locked_frame_target = target
                    locked_frame_age = FRAME_CENTER_LOCK_FRAMES

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
                body_err_x, body_err_y, raw_yaw, raw_pitch, yaw, pitch, motor_yaw, motor_pitch = compute_motor_command(
                    target,
                    laser,
                    err_x,
                    err_y,
                )
                motor_yaw, motor_pitch = apply_pitch_guard(target, motor_yaw, motor_pitch)

                target_spread = samples_spread(target_samples)
                laser_spread = samples_spread(laser_samples)
                valid = 1
                if len(target_samples) < TARGET_WARMUP or len(laser_samples) < LASER_WARMUP:
                    valid = 0
                if _t_src == "red_rings_part":
                    valid = 1
                if target_spread > TARGET_STABLE_SPREAD_PX or laser_spread > LASER_STABLE_SPREAD_PX:
                    valid = 0
                if _t_src == "red_rings_part":
                    valid = 1
                if _t_src == "red_panel_center":
                    if len(target_samples) < RED_PANEL_CENTER_TARGET_WARMUP:
                        valid = 0
                    if target_spread > RED_PANEL_CENTER_STABLE_SPREAD_PX:
                        valid = 0

                if valid and not using_held_target and target_hold_allowed(target):
                    last_hold_target = (_t_src, filt_tx, filt_ty, _t_w, _t_h, _corners, _t_score)
                    last_hold_ms = time.ticks_ms()

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
                if not target:
                    target_missing_count += 1
                    hold_misses = TARGET_HOLD_MISSES
                    if target_rejected and (
                        prev_target_src == "red_rings"
                        or prev_target_src == "red_rings_part"
                        or prev_target_src == "red_panel_center"
                    ):
                        hold_misses = RED_RINGS_REJECT_HOLD_MISSES
                    if target_missing_count >= hold_misses:
                        target_samples = []
                        prev_tx = None
                        prev_ty = None
                        prev_target_src = None
                        last_hold_target = None
                        target_missing_count = 0
                else:
                    target_missing_count = 0
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


def run_entry():
    if DIAGNOSE_ONCE:
        diagnose_once()
    else:
        main(DEFAULT_RUN_SECONDS)


if __name__ == "__main__":
    run_entry()
