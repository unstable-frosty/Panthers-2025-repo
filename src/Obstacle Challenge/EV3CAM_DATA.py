from machine import UART, LED
import sensor
import time

# ================================
# CONFIGURATION
# ================================
LINE_OFFSET = 159   # distance from center in pixels
LEFT_LINE_COLOR  = (255, 0, 0)    # Red
RIGHT_LINE_COLOR = (0, 255, 0)    # Green
CENTER_COLOR     = (0, 0, 255)
BLUE_LINE_COLOR  = (0, 0, 255)

# Y-threshold: ignore all blobs ABOVE this line (for detection + drawing)
Y_IGNORE_THRESHOLD = 30

#ROI unused
BOX_WIDTH   = 50
BOX_HEIGHT  = 50
BOX_Y_SHIFT = 70
BOX_X_DEVIATION = BOX_WIDTH // 2

SIDE_BOX_WIDTH   = 110
SIDE_BOX_HEIGHT  = 60
SIDE_BOX_Y_SHIFT = 70
SIDE_BOX_X_DEVIATION = SIDE_BOX_WIDTH // 2
ROI_X_SHIFT = 120

# Low-pass filter settings
LPF_ALPHA = 0.8
LPF_RESET_LOOPS = 7

# LED setup
led = LED("LED_GREEN")
led.on()

# Color Tracking Thresholds
thresholds = [
    (3, 29, 7, 36, 4, 21),       # RED
    (20, 60, -28, -7, 4, 27),    # GREEN
    (55, 72, 26, 48, -19, -7),    # MAGENTA
]

# Bit codes
RED_CODE     = 1
GREEN_CODE   = 2
MAGENTA_CODE = 4

uart = UART(1, 19200, timeout_char=100)

CODE_TO_COLOR = {
    RED_CODE:     (255,   0,   0),
    GREEN_CODE:   (0,   255,   0),
    MAGENTA_CODE: (255, 105, 180),
}

# ================================
# CAMERA SETUP
# ================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(False, exposure_us=100000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
time.sleep_ms(500)

# ================================
# UTILS: BOUNDING BOX CENTER
# ================================
def bbox_center(b):
    # Center of the rectangle (x + w/2, y + h/2)
    return b.x() + b.w() // 2, b.y() + b.h() // 2

# ================================
# FIND OBSTACLES + MAGENTA
# ================================
def find_largest_obstacle_and_magenta(img):
    largest_obstacle = None
    largest_obstacle_sig = 6    # 6 = no obstacle
    largest_obstacle_area = -1

    best_magenta = None
    best_magenta_area = -1

    cx0 = img.width() // 2
    cy0 = img.height() // 2

    
    blobs = img.find_blobs(thresholds, pixels_threshold=100, area_threshold=100, merge=False)

    for b in blobs:
        code = b.code()
        area = b.pixels()
        w = b.w()
        h = b.h()

        # use bounding-box center instead of centroid
        bx, by = bbox_center(b)

        # IGNORE ANY BLOB ABOVE THE Y-THRESHOLD
        if by < Y_IGNORE_THRESHOLD:
            continue

        # OBSTACLE CANDIDATE: RED OR GREEN
        if code & (RED_CODE | GREEN_CODE):

            # Ignore red blobs that are wider than tall
            if (code & RED_CODE) and (w > h):
                continue

            sig = 1 if (code & RED_CODE) else 2

            if area > largest_obstacle_area:
                largest_obstacle_area = area
                largest_obstacle = b
                largest_obstacle_sig = sig

        # MAGENTA (does NOT affect sig)
        if code & MAGENTA_CODE:
            if area > best_magenta_area:
                best_magenta_area = area
                best_magenta = b

    # RETURN OBSTACLE
    if largest_obstacle is None:
        obstacle = (0, 0, 0, 0, 6)  # sig = 6
    else:
        bx, by = bbox_center(largest_obstacle)
        obstacle = (
            bx - cx0,
            by - cy0,
            largest_obstacle.w(),
            largest_obstacle.h(),
            largest_obstacle_sig
        )

    # RETURN MAGENTA
    if best_magenta is None:
        magenta = (0, 0, 0, 0)
    else:
        mx, my = bbox_center(best_magenta)
        magenta = (
            mx - cx0,
            my - cy0,
            best_magenta.w(),
            best_magenta.h()
        )

    return obstacle, magenta, blobs

# ================================
# UART SENDER (UNCHANGED FORMAT)
# ================================
def send_uart_error_magenta(uart, error, osig, magenta):
    mcx, mcy, mw, mh = magenta
    uart.write("D,{},{},{},{},{},{}\n".format(int(error), int(osig), mcx, mcy, mw, mh))

# ================================
# LPF STATE
# ================================
filtered_ocx_rel = 0.0
lpf_loop_count = 0
last_sig = 6

# ================================
# MAIN LOOP
# ================================
while True:
    clock.tick()
    img = sensor.snapshot()

    obstacle, magenta, blobs = find_largest_obstacle_and_magenta(img)

    cx = img.width() // 2
    cy = img.height() // 2
    img.draw_cross(cx, cy, color=CENTER_COLOR, size=10)

    x_left  = cx - LINE_OFFSET
    x_right = cx + LINE_OFFSET

    # Left and right lines only
    img.draw_line(x_left, 0, x_left, img.height()-1, color=LEFT_LINE_COLOR, thickness=2)
    img.draw_line(x_right, 0, x_right, img.height()-1, color=RIGHT_LINE_COLOR, thickness=2)

    # DRAW Y-IGNORE BLUE LINE (visual only)
    img.draw_line(0, Y_IGNORE_THRESHOLD, img.width(), Y_IGNORE_THRESHOLD,
                  color=BLUE_LINE_COLOR, thickness=2)

    # Extract obstacle data
    ocx_rel, ocy_rel, ow, oh, osig = obstacle  # osig = 1 red, 2 green, 6 none
    base_sig = osig  # keep the original color code for internal logic

    error = 0
    too_close = 0  # kept only for printing/debug; no safety logic now

    # ================================
    # OBSTACLE FOUND
    # ================================
    if base_sig == 1 or base_sig == 2:

        # Absolute obstacle center of the *largest* obstacle (bbox-based)
        obstacle_abs_x = ocx_rel + cx
        obstacle_abs_y = ocy_rel + cy

        # No safety ROI / too_close checks anymore

        # ----- LPF (uses base_sig 1/2 only, unchanged) -----
        if base_sig != last_sig:
            filtered_ocx_rel = float(ocx_rel)
            lpf_loop_count = 0
        else:
            filtered_ocx_rel = (LPF_ALPHA * ocx_rel) + ((1.0 - LPF_ALPHA) * filtered_ocx_rel)
            lpf_loop_count += 1
            if lpf_loop_count >= LPF_RESET_LOOPS:
                filtered_ocx_rel = float(ocx_rel)
                lpf_loop_count = 0

        obj_x = cx + filtered_ocx_rel
        line_x = x_left if base_sig == 1 else x_right
        error = obj_x - line_x

        # osig is just the base color (1 = red, 2 = green)
        osig = base_sig

    else:
        base_sig = 6
        osig = 6
        error = 0
        lpf_loop_count = 0
        too_close = 0

    # track last base color sig (1,2,6) for LPF behavior
    last_sig = base_sig

    print("error:", error, "sig:", osig,
          "too_close:", too_close, "mag:", magenta)

    # ================================
    # DRAW VALID BLOBS ONLY
    # ================================
    for blob in blobs:
        code = blob.code()

        # Skip red blobs wider than tall
        if (code & RED_CODE) and (blob.w() > blob.h()):
            continue

        # Skip blobs above Y-threshold (using bbox center)
        bx, by = bbox_center(blob)
        if by < Y_IGNORE_THRESHOLD:
            continue

        if   code & RED_CODE:       color = CODE_TO_COLOR[RED_CODE]
        elif code & GREEN_CODE:     color = CODE_TO_COLOR[GREEN_CODE]
        elif code & MAGENTA_CODE:   color = CODE_TO_COLOR[MAGENTA_CODE]
        else:                       color = (255, 255, 255)

        img.draw_rectangle(blob.rect(), color=color, thickness=2)
        img.draw_cross(bx, by, color=color, size=10)

    # UART SEND (osig is now only 1, 2, or 6)
    send_uart_error_magenta(uart, error, osig, magenta)
