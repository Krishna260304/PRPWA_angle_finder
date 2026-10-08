import math
import threading
import time
from typing import List, Tuple, Optional
import cv2
import numpy as np

import config

class SyntheticCamera:

    def __init__(self, width: int = config.CAMERA_WIDTH, height: int = config.CAMERA_HEIGHT):
        self.width = width
        self.height = height

        self.center_x = self.width // 2 - 10
        self.center_y = self.height // 2 + 15

        self.arm_length = 165
        self.arm_width = 12
        self.tip_marker_radius = 8

        self.current_angle = 35.0
        self.target_angle = 35.0
        self.is_moving = False
        self.rotation_speed_dps = config.SIMULATION_MOTOR_SPEED_DPS
        self.last_update_time = time.time()

        self.tip_color_bgr = (25, 25, 25)
        self.marker_mode = config.DEFAULT_MARKER_MODE

        self._bg = self._create_workbench_background()

    def set_resolution(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self.center_x = self.width // 2 - 10
        self.center_y = self.height // 2 + 15
        scale = height / 720.0
        self.arm_length = int(165 * scale)
        self.arm_width = max(8, int(12 * scale))
        self.tip_marker_radius = max(6, int(8 * scale))
        self._bg = self._create_workbench_background()

    def _create_workbench_background(self) -> np.ndarray:
        bg = np.full((self.height, self.width, 3), (225, 228, 232), dtype=np.uint8)

        grid_spacing = 40
        for x in range(0, self.width, grid_spacing):
            cv2.line(bg, (x, 0), (x, self.height), (210, 215, 220), 1)
        for y in range(0, self.height, grid_spacing):
            cv2.line(bg, (0, y), (self.width, y), (210, 215, 220), 1)

        motor_size = 130
        mx1 = self.center_x - motor_size // 2
        my1 = self.center_y - motor_size // 2
        mx2 = self.center_x + motor_size // 2
        my2 = self.center_y + motor_size // 2

        cv2.rectangle(bg, (mx1 + 4, my1 + 4), (mx2 + 4, my2 + 4), (170, 175, 180), -1)

        cv2.rectangle(bg, (mx1, my1), (mx2, my2), (48, 52, 58), -1)
        cv2.rectangle(bg, (mx1, my1), (mx2, my2), (80, 85, 95), 2)

        corner_offsets = [
            (-motor_size // 2 + 12, -motor_size // 2 + 12),
            (motor_size // 2 - 12, -motor_size // 2 + 12),
            (-motor_size // 2 + 12, motor_size // 2 - 12),
            (motor_size // 2 - 12, motor_size // 2 - 12),
        ]
        for ox, oy in corner_offsets:
            cv2.circle(bg, (self.center_x + ox, self.center_y + oy), 5, (160, 165, 175), -1)
            cv2.circle(bg, (self.center_x + ox, self.center_y + oy), 5, (30, 30, 30), 1)

        cv2.circle(bg, (self.center_x, self.center_y), 38, (35, 38, 42), -1)
        cv2.circle(bg, (self.center_x, self.center_y), 38, (120, 125, 135), 2)
        cv2.circle(bg, (self.center_x, self.center_y), 24, (70, 75, 82), -1)

        return bg

    def command_move(self, direction: str, steps: int, step_angle: float = 1.8) -> None:
        if steps in (512, 1024, 2048, 3072, 4096):
            angular_change = steps * (360.0 / 2048.0)
        else:
            angular_change = steps * step_angle

        if direction.upper() in ("CW", "CLOCKWISE"):
            self.target_angle = (self.current_angle + angular_change) % 360.0
            self._move_direction = 1
        else:
            self.target_angle = (self.current_angle - angular_change) % 360.0
            self._move_direction = -1

        self.is_moving = True
        self._angular_remaining = angular_change
        self.last_update_time = time.time()

    def stop(self) -> None:
        self.is_moving = False
        self._angular_remaining = 0.0

    def update_physics(self) -> None:
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        if not self.is_moving:
            return

        step_move = self.rotation_speed_dps * dt
        if step_move >= self._angular_remaining:
            self.current_angle = self.target_angle
            self.is_moving = False
            self._angular_remaining = 0.0
        else:
            self._angular_remaining -= step_move
            self.current_angle = (self.current_angle + self._move_direction * step_move) % 360.0

    def get_tip_position(self) -> Tuple[int, int]:
        rad = math.radians(self.current_angle)
        dx = self.arm_length * math.sin(rad)
        dy = -self.arm_length * math.cos(rad)
        tip_x = int(round(self.center_x + dx))
        tip_y = int(round(self.center_y + dy))
        return tip_x, tip_y

    def read_frame(self) -> np.ndarray:
        self.update_physics()
        frame = self._bg.copy()

        tip_x, tip_y = self.get_tip_position()

        rad = math.radians(self.current_angle)
        perp_rad = rad + math.pi / 2.0

        w_base = self.arm_width / 2.0
        w_tip = 3.0

        p1 = (int(self.center_x + w_base * math.cos(perp_rad)),
              int(self.center_y + w_base * math.sin(perp_rad)))
        p2 = (int(self.center_x - w_base * math.cos(perp_rad)),
              int(self.center_y - w_base * math.sin(perp_rad)))
        p3 = (int(tip_x - w_tip * math.cos(perp_rad)),
              int(tip_y - w_tip * math.sin(perp_rad)))
        p4 = (int(tip_x + w_tip * math.cos(perp_rad)),
              int(tip_y + w_tip * math.sin(perp_rad)))

        arm_pts = np.array([p1, p2, p3, p4], dtype=np.int32)

        shadow_pts = arm_pts + np.array([3, 4], dtype=np.int32)
        cv2.fillPoly(frame, [shadow_pts], (180, 185, 190), cv2.LINE_AA)

        cv2.fillPoly(frame, [arm_pts], (250, 252, 255), cv2.LINE_AA)
        cv2.polylines(frame, [arm_pts], isClosed=True, color=(160, 165, 175), thickness=1, lineType=cv2.LINE_AA)

        cv2.circle(frame, (self.center_x, self.center_y), 11, (180, 185, 190), -1, cv2.LINE_AA)
        cv2.circle(frame, (self.center_x, self.center_y), 6, (90, 95, 105), -1, cv2.LINE_AA)
        cv2.circle(frame, (self.center_x, self.center_y), 11, (120, 125, 135), 1, cv2.LINE_AA)

        tip_color = (25, 25, 25) if self.marker_mode == "dark" else self.tip_color_bgr
        cv2.circle(frame, (tip_x, tip_y), self.tip_marker_radius, tip_color, -1, cv2.LINE_AA)
        cv2.circle(frame, (tip_x, tip_y), self.tip_marker_radius, (255, 255, 255), 1, cv2.LINE_AA)

        noise = np.random.normal(0, 1.8, frame.shape).astype(np.int16)
        frame_noisy = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return frame_noisy

CANDIDATE_RESOLUTIONS = [
    (3840, 2160, "4K UHD (3840x2160)"),
    (2560, 1440, "2K QHD (2560x1440)"),
    (1920, 1080, "1080p FHD (1920x1080)"),
    (1280, 720,  "720p HD (1280x720)"),
    (1024, 768,  "XGA (1024x768)"),
    (800, 600,   "SVGA (800x600)"),
    (640, 480,   "480p SD (640x480)"),
]

class CameraManager:

    _cached_resolutions: dict = {}

    def __init__(self):
        self.cap: Optional[cv2.VideoCapture] = None
        self.synthetic_cam: Optional[SyntheticCamera] = None

        self.is_running: bool = False
        self.is_synthetic_mode: bool = False
        self.camera_index: int = config.DEFAULT_CAMERA_INDEX
        self.current_width: int = config.CAMERA_WIDTH
        self.current_height: int = config.CAMERA_HEIGHT
        self.supported_resolutions: List[Tuple[int, int, str]] = []

        self.latest_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None

        self.fps: float = 0.0
        self._frame_count: int = 0
        self._fps_start_time: float = time.time()

    @staticmethod
    def list_available_cameras(max_tested: int = 4) -> List[int]:
        available = []
        for idx in range(max_tested):
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if cv2.CAP_DSHOW else cv2.VideoCapture(idx)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        available.append(idx)
                cap.release()
            except Exception:
                pass
        return available

    @classmethod
    def probe_supported_resolutions(cls, camera_index: int,
                                   open_cap: Optional[cv2.VideoCapture] = None) -> List[Tuple[int, int, str]]:

        if camera_index in cls._cached_resolutions:
            return cls._cached_resolutions[camera_index]

        if camera_index == -1:
            sim_res = [
                (3840, 2160, "4K UHD (3840x2160)"),
                (1920, 1080, "1080p FHD (1920x1080)"),
                (1280, 720,  "720p HD (1280x720)"),
            ]
            cls._cached_resolutions[-1] = sim_res
            return sim_res

        cap = open_cap
        own_cap = False
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap = cv2.VideoCapture(camera_index)
            own_cap = True

        supported: List[Tuple[int, int, str]] = []
        if cap is not None and cap.isOpened():
            for w, h, label in CANDIDATE_RESOLUTIONS:
                try:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.shape[1] == w and frame.shape[0] == h:
                        supported.append((w, h, label))
                except Exception:
                    pass

            if own_cap:
                cap.release()

        if not supported:
            supported = [
                (1920, 1080, "1080p FHD (1920x1080)"),
                (1280, 720,  "720p HD (1280x720)"),
            ]

        cls._cached_resolutions[camera_index] = supported
        return supported

    def set_resolution(self, width: int, height: int) -> Tuple[int, int]:
        with self.frame_lock:
            if self.is_synthetic_mode and self.synthetic_cam is not None:
                self.synthetic_cam.set_resolution(width, height)
                self.current_width = width
                self.current_height = height
            elif self.cap is not None and self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                act_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                act_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.current_width = act_w if act_w > 0 else width
                self.current_height = act_h if act_h > 0 else height
            else:
                self.current_width = width
                self.current_height = height

        return self.current_width, self.current_height

    def start(self, camera_index: int = 0, width: Optional[int] = None,
              height: Optional[int] = None, force_synthetic: bool = False) -> bool:

        self.stop()
        self.camera_index = camera_index

        if force_synthetic or camera_index == -1:
            self.is_synthetic_mode = True
            init_w = width or 1920
            init_h = height or 1080
            self.synthetic_cam = SyntheticCamera(init_w, init_h)
            self.current_width = init_w
            self.current_height = init_h
            self.supported_resolutions = self.probe_supported_resolutions(-1)
        else:
            try:

                self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
                if not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(camera_index)

                if self.cap.isOpened():

                    self.supported_resolutions = self.probe_supported_resolutions(camera_index, self.cap)

                    if width and height:
                        target_w, target_h = width, height
                    elif any(r[0] == 1920 for r in self.supported_resolutions):
                        target_w, target_h = 1920, 1080
                    elif any(r[0] == 3840 for r in self.supported_resolutions):
                        target_w, target_h = 3840, 2160
                    elif self.supported_resolutions:
                        target_w, target_h = self.supported_resolutions[0][0], self.supported_resolutions[0][1]
                    else:
                        target_w, target_h = config.CAMERA_WIDTH, config.CAMERA_HEIGHT

                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_w)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
                    self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

                    act_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    act_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    self.current_width = act_w if act_w > 0 else target_w
                    self.current_height = act_h if act_h > 0 else target_h
                    self.is_synthetic_mode = False
                else:

                    self.is_synthetic_mode = True
                    self.synthetic_cam = SyntheticCamera()
                    self.supported_resolutions = self.probe_supported_resolutions(-1)
            except Exception:
                self.is_synthetic_mode = True
                self.synthetic_cam = SyntheticCamera()
                self.supported_resolutions = self.probe_supported_resolutions(-1)

        self.is_running = True
        self._fps_start_time = time.time()
        self._frame_count = 0

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return True

    def _capture_loop(self) -> None:
        target_period = 1.0 / config.CAMERA_FPS

        while self.is_running:
            loop_start = time.time()

            if self.is_synthetic_mode and self.synthetic_cam is not None:
                frame = self.synthetic_cam.read_frame()
                ret = True
            elif self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
            else:
                ret, frame = False, None

            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame

                self._frame_count += 1
                now = time.time()
                elapsed = now - self._fps_start_time
                if elapsed >= 1.0:
                    self.fps = self._frame_count / elapsed
                    self._frame_count = 0
                    self._fps_start_time = now
            else:
                time.sleep(0.01)

            elapsed_work = time.time() - loop_start
            sleep_time = target_period - elapsed_work
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self.frame_lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            return False, None

    def stop(self) -> None:
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.8)
        self.thread = None

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        with self.frame_lock:
            self.latest_frame = None

    def is_connected(self) -> bool:
        return self.is_running and (self.latest_frame is not None)
