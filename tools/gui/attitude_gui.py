"""Attitude indicator GUI for the STM32 firmware.

Reads telemetry lines over a serial port and draws an artificial horizon
in real time. Without a board connected, the sliders drive the horizon
(simulation mode), which is handy to check the drawing code.

Protocol (one ASCII line per sample, sent at ~50 Hz by the firmware):

    ATT,<roll>,<pitch>\\n      angles in degrees, vehicle frame

Usage:
    pip install -r requirements.txt
    python attitude_gui.py
"""

import math
import queue
import threading
import time

import dearpygui.dearpygui as dpg
import serial
from serial.tools import list_ports


# ============================================================
# CONFIGURATION
# ============================================================

VIEWPORT_WIDTH = 1200
VIEWPORT_HEIGHT = 760
CONTROL_PANEL_WIDTH = 280

DEFAULT_BAUDRATE = "115200"

# Vertical field of view of the horizon: pixels per degree = height / FOV
PITCH_FOV_DEG = 60.0

HORIZON_TAG = "attitude_horizon"
HORIZON_CONTAINER_TAG = "horizon_container"

STATUS_TAG = "connection_status"
PORT_COMBO_TAG = "serial_port"
BAUDRATE_COMBO_TAG = "serial_baudrate"

ROLL_TEXT_TAG = "roll_text"
PITCH_TEXT_TAG = "pitch_text"
RATE_TEXT_TAG = "rate_text"

ROLL_SLIDER_TAG = "roll_slider"
PITCH_SLIDER_TAG = "pitch_slider"

SKY_COLOUR = (45, 140, 200, 255)
GROUND_COLOUR = (150, 95, 55, 255)
WHITE = (255, 255, 255, 255)
AIRCRAFT_COLOUR = (255, 220, 20, 255)


# ============================================================
# SERIAL STATE
# ============================================================

serial_port: serial.Serial | None = None
serial_running = False

# The UART thread produces samples, the Dear PyGui thread consumes them.
telemetry_queue: queue.Queue[tuple[float, float]] = queue.Queue()

# Received-sample counter for the rate indicator.
samples_received = 0
rate_window_start = time.monotonic()


# ============================================================
# HORIZON DRAWING
# ============================================================

def rotate_point(
    x: float,
    y: float,
    center_x: float,
    center_y: float,
    angle_rad: float,
) -> tuple[float, float]:
    """Rotate a point around a center."""

    relative_x = x - center_x
    relative_y = y - center_y

    rotated_x = relative_x * math.cos(angle_rad) - relative_y * math.sin(angle_rad)
    rotated_y = relative_x * math.sin(angle_rad) + relative_y * math.cos(angle_rad)

    return center_x + rotated_x, center_y + rotated_y


def transform_point(
    x: float,
    y: float,
    center_x: float,
    center_y: float,
    roll_deg: float,
    pitch_offset: float,
) -> tuple[float, float]:
    """Apply the pitch offset, then the roll rotation."""

    return rotate_point(
        x,
        y + pitch_offset,
        center_x,
        center_y,
        math.radians(-roll_deg),
    )


def draw_attitude_indicator(
    width: int,
    height: int,
    roll_deg: float,
    pitch_deg: float,
) -> None:
    """Redraw the whole artificial horizon."""

    if not dpg.does_item_exist(HORIZON_TAG) or width <= 0 or height <= 0:
        return

    dpg.delete_item(HORIZON_TAG, children_only=True)

    center_x = width / 2
    center_y = height / 2

    pixels_per_degree = height / PITCH_FOV_DEG
    pitch_offset = pitch_deg * pixels_per_degree

    # Oversized geometry so no empty corners appear when rotated.
    radius = math.hypot(width, height)

    left = center_x - radius
    right = center_x + radius
    top = center_y - radius
    bottom = center_y + radius

    def t(x: float, y: float) -> tuple[float, float]:
        return transform_point(x, y, center_x, center_y, roll_deg, pitch_offset)

    # Sky
    dpg.draw_polygon(
        [t(left, top), t(right, top), t(right, center_y), t(left, center_y)],
        fill=SKY_COLOUR,
        color=SKY_COLOUR,
        parent=HORIZON_TAG,
    )

    # Ground
    dpg.draw_polygon(
        [t(left, center_y), t(right, center_y), t(right, bottom), t(left, bottom)],
        fill=GROUND_COLOUR,
        color=GROUND_COLOUR,
        parent=HORIZON_TAG,
    )

    # Horizon line
    dpg.draw_line(
        t(left, center_y),
        t(right, center_y),
        color=WHITE,
        thickness=3,
        parent=HORIZON_TAG,
    )

    draw_pitch_ladder(width, height, center_x, center_y, roll_deg, pitch_offset)
    draw_aircraft_symbol(width, height)

    dpg.set_value(ROLL_TEXT_TAG, f"Roll:  {roll_deg:6.1f}°")
    dpg.set_value(PITCH_TEXT_TAG, f"Pitch: {pitch_deg:6.1f}°")


def draw_pitch_ladder(
    width: int,
    height: int,
    center_x: float,
    center_y: float,
    roll_deg: float,
    pitch_offset: float,
) -> None:
    """Draw the pitch marks every 5 degrees, labelled every 10."""

    pixels_per_degree = height / PITCH_FOV_DEG

    for pitch_mark in range(-30, 31, 5):
        if pitch_mark == 0:
            continue

        mark_y = center_y - pitch_mark * pixels_per_degree

        if pitch_mark % 10 == 0:
            half_width = width * 0.11
            thickness = 3
        else:
            half_width = width * 0.065
            thickness = 2

        start = transform_point(
            center_x - half_width, mark_y, center_x, center_y, roll_deg, pitch_offset
        )
        end = transform_point(
            center_x + half_width, mark_y, center_x, center_y, roll_deg, pitch_offset
        )

        dpg.draw_line(start, end, color=WHITE, thickness=thickness, parent=HORIZON_TAG)

        if pitch_mark % 10 == 0:
            label_position = transform_point(
                center_x + half_width + 12,
                mark_y - 8,
                center_x,
                center_y,
                roll_deg,
                pitch_offset,
            )

            dpg.draw_text(
                label_position,
                str(abs(pitch_mark)),
                color=WHITE,
                size=16,
                parent=HORIZON_TAG,
            )


def draw_aircraft_symbol(width: int, height: int) -> None:
    """Fixed aircraft reference symbol."""

    center_x = width / 2
    center_y = height / 2

    wing_length = width * 0.12
    center_gap = width * 0.025

    dpg.draw_line(
        (center_x - wing_length, center_y),
        (center_x - center_gap, center_y),
        color=AIRCRAFT_COLOUR,
        thickness=5,
        parent=HORIZON_TAG,
    )

    dpg.draw_line(
        (center_x + center_gap, center_y),
        (center_x + wing_length, center_y),
        color=AIRCRAFT_COLOUR,
        thickness=5,
        parent=HORIZON_TAG,
    )

    dpg.draw_line(
        (center_x, center_y - 10),
        (center_x, center_y + 18),
        color=AIRCRAFT_COLOUR,
        thickness=5,
        parent=HORIZON_TAG,
    )

    dpg.draw_circle(
        (center_x, center_y),
        radius=5,
        fill=AIRCRAFT_COLOUR,
        color=AIRCRAFT_COLOUR,
        parent=HORIZON_TAG,
    )


# ============================================================
# SERIAL
# ============================================================

def refresh_serial_ports() -> None:
    """Refresh the list of available serial ports."""

    ports = [port.device for port in list_ports.comports()]

    dpg.configure_item(PORT_COMBO_TAG, items=ports)

    current_port = dpg.get_value(PORT_COMBO_TAG)

    if ports and current_port not in ports:
        dpg.set_value(PORT_COMBO_TAG, ports[0])

    if not ports:
        dpg.set_value(PORT_COMBO_TAG, "")


def connect_serial() -> None:
    """Open the port and start the reader thread."""

    global serial_port
    global serial_running

    if serial_port is not None and serial_port.is_open:
        return

    port_name = dpg.get_value(PORT_COMBO_TAG)

    if not port_name:
        dpg.set_value(STATUS_TAG, "No port selected")
        return

    try:
        baudrate = int(dpg.get_value(BAUDRATE_COMBO_TAG))
    except (TypeError, ValueError):
        dpg.set_value(STATUS_TAG, "Invalid baud rate")
        return

    try:
        serial_port = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )

        serial_port.reset_input_buffer()
        serial_running = True

        threading.Thread(
            target=serial_reader_loop,
            name="STM32-UART-Reader",
            daemon=True,
        ).start()

        dpg.set_value(STATUS_TAG, f"Connected: {port_name}")

    except (serial.SerialException, OSError) as error:
        serial_port = None
        serial_running = False
        dpg.set_value(STATUS_TAG, f"Error: {error}")


def disconnect_serial() -> None:
    """Stop the reader thread and close the port."""

    global serial_port
    global serial_running

    serial_running = False

    port_to_close = serial_port
    serial_port = None

    if port_to_close is not None:
        try:
            if port_to_close.is_open:
                port_to_close.close()
        except serial.SerialException:
            pass

    if dpg.does_item_exist(STATUS_TAG):
        dpg.set_value(STATUS_TAG, "Disconnected (simulation mode)")


def serial_reader_loop() -> None:
    """Reader thread: one telemetry line per iteration."""

    global serial_running

    while serial_running:
        port = serial_port

        if port is None or not port.is_open:
            break

        try:
            raw_line = port.readline()

            if not raw_line:
                continue

            line = raw_line.decode("ascii", errors="replace").strip()

            if line:
                parse_telemetry_line(line)

        except (serial.SerialException, OSError) as error:
            print(f"UART read error: {error}")
            serial_running = False
            break


def parse_telemetry_line(line: str) -> None:
    """Parse one line. Supported: ATT,<roll>,<pitch>"""

    fields = line.split(",")
    message_type = fields[0].strip().upper()

    if message_type == "ATT":
        parse_attitude_message(fields, line)
        return

    print(f"Unknown UART message: {line!r}")


def parse_attitude_message(fields: list[str], original_line: str) -> None:
    global samples_received

    if len(fields) != 3:
        print(f"Malformed ATT message: {original_line!r}")
        return

    try:
        roll = float(fields[1])
        pitch = float(fields[2])
    except ValueError:
        print(f"Invalid roll or pitch: {original_line!r}")
        return

    # Basic protection against corrupted values.
    if not math.isfinite(roll) or not math.isfinite(pitch):
        return

    samples_received += 1
    telemetry_queue.put((roll, pitch))


def update_from_sensor_data() -> None:
    """
    Runs on the Dear PyGui thread every frame.

    Drains the queue and keeps only the newest sample, so the drawing never
    lags behind the board when the GUI renders slower than data arrives.
    """

    latest: tuple[float, float] | None = None

    while True:
        try:
            latest = telemetry_queue.get_nowait()
        except queue.Empty:
            break

    if latest is None:
        return

    roll, pitch = latest
    width, height = dpg.get_item_rect_size(HORIZON_TAG)

    draw_attitude_indicator(int(width), int(height), roll, pitch)


def update_rate_indicator() -> None:
    """Show the telemetry rate, refreshed once per second."""

    global samples_received
    global rate_window_start

    elapsed = time.monotonic() - rate_window_start

    if elapsed < 1.0:
        return

    rate_hz = samples_received / elapsed
    samples_received = 0
    rate_window_start = time.monotonic()

    dpg.set_value(RATE_TEXT_TAG, f"Rate:  {rate_hz:5.1f} Hz")


# ============================================================
# SIMULATION MODE
# ============================================================

def update_from_controls() -> None:
    """Drive the horizon from the sliders (no board needed)."""

    if serial_port is not None and serial_port.is_open:
        return

    width, height = dpg.get_item_rect_size(HORIZON_TAG)

    draw_attitude_indicator(
        int(width),
        int(height),
        float(dpg.get_value(ROLL_SLIDER_TAG)),
        float(dpg.get_value(PITCH_SLIDER_TAG)),
    )


# ============================================================
# GUI
# ============================================================

def create_control_panel() -> None:
    available_ports = [port.device for port in list_ports.comports()]

    dpg.add_text("CONNECTION")
    dpg.add_separator()

    dpg.add_combo(
        tag=PORT_COMBO_TAG,
        items=available_ports,
        default_value=available_ports[0] if available_ports else "",
        label="Port",
        width=120,
    )

    dpg.add_combo(
        tag=BAUDRATE_COMBO_TAG,
        items=["9600", "57600", "115200"],
        default_value=DEFAULT_BAUDRATE,
        label="Baud",
        width=120,
    )

    with dpg.group(horizontal=True):
        dpg.add_button(label="Refresh", width=110, callback=refresh_serial_ports)
        dpg.add_button(label="Connect", width=110, callback=connect_serial)

    dpg.add_button(label="Disconnect", width=226, callback=disconnect_serial)

    dpg.add_text("Disconnected (simulation mode)", tag=STATUS_TAG)

    dpg.add_spacer(height=10)
    dpg.add_text("ATTITUDE")
    dpg.add_separator()

    dpg.add_text("Roll:     0.0°", tag=ROLL_TEXT_TAG)
    dpg.add_text("Pitch:    0.0°", tag=PITCH_TEXT_TAG)
    dpg.add_text("Rate:     -- Hz", tag=RATE_TEXT_TAG)

    dpg.add_spacer(height=10)
    dpg.add_text("SIMULATION (no board)")
    dpg.add_separator()

    dpg.add_slider_float(
        tag=ROLL_SLIDER_TAG,
        label="Roll",
        min_value=-60,
        max_value=60,
        default_value=0,
        width=170,
        callback=update_from_controls,
    )

    dpg.add_slider_float(
        tag=PITCH_SLIDER_TAG,
        label="Pitch",
        min_value=-30,
        max_value=30,
        default_value=0,
        width=170,
        callback=update_from_controls,
    )


def resize_horizon() -> None:
    """Fit the drawlist to the available space."""

    if not dpg.does_item_exist(HORIZON_CONTAINER_TAG):
        return

    container_width, container_height = dpg.get_item_rect_size(HORIZON_CONTAINER_TAG)

    if container_width <= 0 or container_height <= 0:
        return

    horizon_width = max(1, int(container_width) - 8)
    horizon_height = max(1, int(container_height) - 8)

    current_width, current_height = dpg.get_item_rect_size(HORIZON_TAG)

    # Avoid redrawing every frame when the size has not changed.
    if int(current_width) == horizon_width and int(current_height) == horizon_height:
        return

    dpg.configure_item(HORIZON_TAG, width=horizon_width, height=horizon_height)

    draw_attitude_indicator(
        horizon_width,
        horizon_height,
        float(dpg.get_value(ROLL_SLIDER_TAG)),
        float(dpg.get_value(PITCH_SLIDER_TAG)),
    )


def main() -> None:
    dpg.create_context()

    with dpg.window(
        tag="main_window",
        no_title_bar=True,
        no_move=True,
        no_resize=True,
        no_collapse=True,
        no_scrollbar=True,
        no_scroll_with_mouse=True,
    ):
        with dpg.table(
            header_row=False,
            borders_innerV=True,
            policy=dpg.mvTable_SizingStretchProp,
        ):
            dpg.add_table_column(
                label="Attitude",
                width_stretch=True,
                init_width_or_weight=1.0,
            )

            dpg.add_table_column(
                label="Controls",
                width_fixed=True,
                init_width_or_weight=CONTROL_PANEL_WIDTH,
            )

            with dpg.table_row():
                with dpg.child_window(
                    tag=HORIZON_CONTAINER_TAG,
                    border=False,
                    no_scrollbar=True,
                    no_scroll_with_mouse=True,
                ):
                    dpg.add_drawlist(tag=HORIZON_TAG, width=850, height=700)

                with dpg.child_window(
                    width=CONTROL_PANEL_WIDTH,
                    border=False,
                    no_scrollbar=True,
                    no_scroll_with_mouse=True,
                ):
                    create_control_panel()

    dpg.create_viewport(
        title="STM32 Attitude Indicator",
        width=VIEWPORT_WIDTH,
        height=VIEWPORT_HEIGHT,
        resizable=True,
    )

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)

    # First frame: Dear PyGui still needs to compute item sizes.
    dpg.render_dearpygui_frame()

    resize_horizon()
    update_from_controls()

    # Manual render loop so telemetry is consumed every frame.
    while dpg.is_dearpygui_running():
        resize_horizon()
        update_from_sensor_data()
        update_rate_indicator()
        dpg.render_dearpygui_frame()

    disconnect_serial()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
