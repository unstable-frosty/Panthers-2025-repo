from machine import UART, LED
import sensor
import time

# ================================
# CONFIGURATION
# ================================
LINE_OFFSET = 170   # distance from center in pixels
LEFT_LINE_COLOR  = (255, 0, 0)   # Red
RIGHT_LINE_COLOR = (0, 255, 0)   # Green
CENTER_COLOR = (0, 0, 255)


# Low-pass filter settings (very light filtering)
LPF_ALPHA = 0.8         # 0..1, closer to 1 = lighter filter (more raw)
LPF_RESET_LOOPS = 10    # reset the filter every N loops

# --- LED setup ---
led = LED("LED_GREEN")
led.on()

# Color Tracking Thresholds
thresholds = [
    (29, 61, 23, 53, 17, 66),   # RED
    (41, 74, -42, 37, 11, 33), # GREEN
    (55, 72, 26, 48, -19, -7),  # MAGENTA
]

# Bit codes
RED_CODE     = 1
GREEN_CODE   = 2
MAGENTA_CODE = 4

uart = UART(1, 19200, timeout_char=100)

CODE_TO_COLOR = {
    RED_CODE:     (255, 0, 0),
    GREEN_CODE:   (0, 255, 0),
    MAGENTA_CODE: (255, 105, 180),
}

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(False, exposure_us=50000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
time.sleep_ms(500)  # Let Arduino get ready

def find_largest_obstacle_and_magenta(img):
    largest_obstacle = None
    largest_obstacle_sig = 6   # 6 = no obstacle
    largest_obstacle_area = -1
    best_magenta = None
    best_magenta_area = -1

    cx0 = img.width() // 2
    cy0 = img.height() // 2

    blobs = img.find_blobs(thresholds, pixels_threshold=100, area_threshold=100, merge=False)
    for b in blobs:
        code = b.code()
        area = b.pixels()

        # RED or GREEN = obstacle
        if code & (RED_CODE | GREEN_CODE):
            sig = 1 if (code & RED_CODE) else 2
            if area > largest_obstacle_area:
                largest_obstacle_area = area
                largest_obstacle = b
                largest_obstacle_sig = sig

        # MAGENTA = special target/marker
        if code & MAGENTA_CODE:
            if area > best_magenta_area:
                best_magenta_area = area
                best_magenta = b

    # Obstacle tuple: (x_rel, y_rel, w, h, sig)
    if largest_obstacle is None:
        obstacle = (0, 0, 0, 0, 6)  # 6 = no obstacle
    else:
        obstacle = (
            largest_obstacle.cx() - cx0,  # x relative to center
            largest_obstacle.cy() - cy0,  # y relative to center
            largest_obstacle.w(),
            largest_obstacle.h(),
            largest_obstacle_sig
        )

    # Magenta tuple: (x_rel, y_rel, w, h)
    if best_magenta is None:
        magenta = (0, 0, 0, 0)
    else:
        magenta = (
            best_magenta.cx() - cx0,
            best_magenta.cy() - cy0,
            best_magenta.w(),
            best_magenta.h()
        )

    return obstacle, magenta, blobs

# ================================
# NEW UART SENDER
# ================================
def send_uart_error_magenta(uart, error, osig, magenta):
    """
    Sends:
    D,<error>,<sig>,<mcx_rel>,<mcy_rel>,<mw>,<mh>\n
    """
    mcx, mcy, mw, mh = magenta

    # Cast error to int to keep it compact (you can remove int() if you want float)
    uart.write("D,{},{},{},{},{},{}\n".format(int(error), int(osig), mcx, mcy, mw, mh))

# ================================
# LPF STATE
# ================================
filtered_ocx_rel = 0.0
lpf_loop_count = 0
last_sig = 6  # start with "no obstacle"

while True:
    clock.tick()
    img = sensor.snapshot()

    # 1) FIRST: detect blobs on a clean image
    obstacle, magenta, blobs = find_largest_obstacle_and_magenta(img)

    # 2) THEN: draw center and guide lines (purely visual)
    cx = img.width() // 2
    cy = img.height() // 2
    img.draw_cross(cx, cy, color=CENTER_COLOR, size=10)

    x_left  = cx - LINE_OFFSET   # red line x (image coords)
    x_right = cx + LINE_OFFSET   # green line x (image coords)

    img.draw_line(x_left, 0, x_left, img.height() - 1,
                  color=LEFT_LINE_COLOR, thickness=2)
    img.draw_line(x_right, 0, x_right, img.height() - 1,
                  color=RIGHT_LINE_COLOR, thickness=2)

    # ====== ERROR + LPF ON X COORDINATE ======
    ocx_rel, ocy_rel, ow, oh, osig = obstacle  # ocx_rel is relative to center

    # Default error when no obstacle
    error = 0

    if osig == 1 or osig == 2:
        # If sig changed (RED <-> GREEN or from no obstacle), reset filter
        if osig != last_sig:
            filtered_ocx_rel = float(ocx_rel)
            lpf_loop_count = 0
        else:
            # Apply low-pass filter (very light)
            filtered_ocx_rel = (LPF_ALPHA * ocx_rel) + ((1.0 - LPF_ALPHA) * filtered_ocx_rel)
            lpf_loop_count += 1

            # Reset filter every LPF_RESET_LOOPS
            if lpf_loop_count >= LPF_RESET_LOOPS:
                filtered_ocx_rel = float(ocx_rel)
                lpf_loop_count = 0

        # Use filtered x for error computation
        if osig == 1:  # RED obstacle -> compare to RED (left) line
            obj_x = cx + filtered_ocx_rel
            line_x = x_left
            error = obj_x - line_x
        elif osig == 2:  # GREEN obstacle -> compare to GREEN (right) line
            obj_x = cx + filtered_ocx_rel
            line_x = x_right
            error = obj_x - line_x
    else:
        # No obstacle: reset error
        error = 0
        lpf_loop_count = 0  # optional: clear count when no obstacle

    # Update last_sig for next loop
    last_sig = osig

    # Debug print on OpenMV serial / IDE
    print("error:", error, "sig:", osig, "mag:", magenta)

    # 3) Draw blobs AFTER detection (visual only)
    for blob in blobs:
        code = blob.code()
        if   code & RED_CODE:     draw_color = CODE_TO_COLOR[RED_CODE]
        elif code & GREEN_CODE:   draw_color = CODE_TO_COLOR[GREEN_CODE]
        elif code & MAGENTA_CODE: draw_color = CODE_TO_COLOR[MAGENTA_CODE]
        else:                     draw_color = (255, 255, 255)

        img.draw_rectangle(blob.rect(), color=draw_color, thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=draw_color, size=10)

    # 4) UART after everything
    send_uart_error_magenta(uart, error, osig, magenta)
