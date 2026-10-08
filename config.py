from enum import Enum
from typing import Tuple

class AppState(str, Enum):

    SET_ORIGIN = "SET_ORIGIN"
    SET_TARGET = "SET_TARGET"
    ARMED = "ARMED"
    MOVING = "MOVING"

AUTO_MOTION_TRIGGER_DEG = 1.8
MOTION_SETTLE_FRAMES = 10
MOTION_SETTLE_SPREAD_DEG = 0.8

DEFAULT_CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
FRAME_UPDATE_INTERVAL_MS = 25

DEFAULT_MARKER_MODE = "dark"

DEFAULT_COLOR_PRESET = "Dark"

DEFAULT_DARK_THRESHOLD = 85

MIN_CIRCULARITY = 0.30

EXPECTED_RADIUS_TOLERANCE = 35

HSV_PRESETS = {
    "Dark": {

        "lower1": (0, 0, 0),
        "upper1": (180, 255, 75),
        "lower2": None,
        "upper2": None,
    },
    "Red": {
        "lower1": (0, 100, 100),
        "upper1": (10, 255, 255),
        "lower2": (160, 100, 100),
        "upper2": (180, 255, 255),
    },
    "Green": {
        "lower1": (35, 80, 80),
        "upper1": (85, 255, 255),
        "lower2": None,
        "upper2": None,
    },
    "Blue": {
        "lower1": (100, 100, 80),
        "upper1": (140, 255, 255),
        "lower2": None,
        "upper2": None,
    },
    "Custom": {
        "lower1": (0, 100, 100),
        "upper1": (10, 255, 255),
        "lower2": (160, 100, 100),
        "upper2": (180, 255, 255),
    }
}

MIN_MARKER_AREA = 8
MAX_MARKER_AREA = 1200

MIN_POINTER_RADIUS = 20
MAX_POINTER_RADIUS = 750

STABILITY_FRAMES = 7
STABILITY_THRESHOLD_PIXELS = 2.0
STABILITY_TIMEOUT_SEC = 3.5

AUTO_MOTION_TRIGGER_DEG = 1.2
AUTO_DETECT_DEFAULT = True

DEFAULT_BAUDRATE = 115200
SUPPORTED_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]
SERIAL_TIMEOUT_SEC = 0.1
SERIAL_COMMAND_TIMEOUT_SEC = 25.0

DEFAULT_STEPS = 100
DEFAULT_DIRECTION = "CW"

COLOR_CROSSHAIR: Tuple[int, int, int] = (0, 255, 255)
COLOR_CROSSHAIR_ACTIVE: Tuple[int, int, int] = (0, 165, 255)
COLOR_ORIGIN: Tuple[int, int, int] = (255, 255, 0)
COLOR_PROTRACTOR_LINE: Tuple[int, int, int] = (30, 30, 235)
COLOR_PROTRACTOR_DOT: Tuple[int, int, int] = (30, 30, 235)
COLOR_PREVIEW_LINE: Tuple[int, int, int] = (0, 215, 255)
COLOR_TEXT_BG: Tuple[int, int, int] = (20, 20, 20)
COLOR_TEXT_FG: Tuple[int, int, int] = (255, 255, 255)
COLOR_HUD_BG: Tuple[int, int, int] = (15, 23, 42)

ROTATION_ARC_COLORS = [
    (30, 30, 235),
    (235, 120, 30),
    (30, 200, 70),
    (0, 180, 255),
    (200, 50, 200),
]

LINE_THICKNESS_RADIAL = 2
ARC_THICKNESS = 2
ARC_RADIUS_BASE = 80
ORIGIN_MARKER_RADIUS = 8
DOT_MARKER_RADIUS = 6

SIMULATION_MOTOR_SPEED_DPS = 180.0
SIM_STEPS_PER_REVOLUTION = 200
