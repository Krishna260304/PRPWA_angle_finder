import math
from collections import deque
from typing import List, Tuple, Optional, Dict, Any
import cv2
import numpy as np

import config
from angle_tracker import AngleTracker

class VisionTracker:

    def __init__(self) -> None:
        self.origin: Optional[Tuple[int, int]] = None
        self.radius: Optional[float] = None

        self.locked_positions: List[Dict[str, Any]] = []

        self.measurements: List[Dict[str, Any]] = []

        self.current_pt: Optional[Tuple[int, int]] = None
        self.current_angle: Optional[float] = None
        self.is_moving: bool = False
        self._motion_trigger_count: int = 0

        self.recent_angles: deque = deque(maxlen=config.MOTION_SETTLE_FRAMES)

        self._filtered_angle: Optional[float] = None

        self.last_frame_dot_angle: Optional[float] = None
        self.accumulated_rotation_deg: float = 0.0

        self.calibrated_arm_brightness: float = 200.0
        self.calibrated_dot_darkness: float = 40.0
        self.calibrated_contrast: float = 160.0

    def clear(self) -> None:
        self.origin = None
        self.radius = None
        self.locked_positions.clear()
        self.measurements.clear()
        self.current_pt = None
        self.current_angle = None
        self.is_moving = False
        self._motion_trigger_count = 0
        self.recent_angles.clear()
        self._filtered_angle = None
        self.last_frame_dot_angle = None
        self.accumulated_rotation_deg = 0.0
        self.calibrated_arm_brightness = 200.0
        self.calibrated_dot_darkness = 40.0
        self.calibrated_contrast = 160.0

    def reset_positions_keep_origin(self) -> None:
        self.radius = None
        self.locked_positions.clear()
        self.measurements.clear()
        self.current_pt = None
        self.current_angle = None
        self.is_moving = False
        self._motion_trigger_count = 0
        self.recent_angles.clear()
        self._filtered_angle = None
        self.last_frame_dot_angle = None
        self.accumulated_rotation_deg = 0.0
        self.calibrated_arm_brightness = 200.0
        self.calibrated_dot_darkness = 40.0
        self.calibrated_contrast = 160.0

    def rescale(self, old_w: int, old_h: int, new_w: int, new_h: int) -> None:
        if old_w <= 0 or old_h <= 0 or (old_w == new_w and old_h == new_h):
            return

        sx = new_w / float(old_w)
        sy = new_h / float(old_h)

        if self.origin is not None:
            self.origin = (int(round(self.origin[0] * sx)), int(round(self.origin[1] * sy)))

        if self.radius is not None:
            self.radius = self.radius * sx

        if self.current_pt is not None:
            self.current_pt = (int(round(self.current_pt[0] * sx)), int(round(self.current_pt[1] * sy)))

        for pos in self.locked_positions:
            px, py = pos["pt"]
            pos["pt"] = (int(round(px * sx)), int(round(py * sy)))

        for meas in self.measurements:
            fx, fy = meas["from_pt"]
            tx, ty = meas["to_pt"]
            meas["from_pt"] = (int(round(fx * sx)), int(round(fy * sy)))
            meas["to_pt"] = (int(round(tx * sx)), int(round(ty * sy)))

    def set_origin(self, pt: Tuple[int, int]) -> None:
        self.origin = pt

    def set_initial_target(self, frame: np.ndarray, click_pt: Tuple[int, int]) -> Tuple[int, int]:
        if self.origin is None:
            return click_pt

        ox, oy = self.origin
        cx, cy = click_pt
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        h, w = gray.shape[:2]
        scale = max(1.0, h / 720.0)

        win = max(12, int(round(16 * scale)))
        x1, y1 = max(0, cx - win), max(0, cy - win)
        x2, y2 = min(w, cx + win), min(h, cy + win)
        patch = gray[y1:y2, x1:x2]

        sub_cx = float(cx)
        sub_cy = float(cy)
        if patch.size > 0:
            inv = 255.0 - patch.astype(np.float32)
            thresh = np.percentile(inv, 60)
            inv[inv < thresh] = 0.0
            m = cv2.moments(inv)
            if m["m00"] > 0:
                sub_cx = float(x1 + m["m10"] / m["m00"])
                sub_cy = float(y1 + m["m01"] / m["m00"])

        self.radius = math.hypot(sub_cx - ox, sub_cy - oy)
        target_pt = (int(round(sub_cx)), int(round(sub_cy)))
        self.current_pt = target_pt
        p1_angle = AngleTracker.point_to_angle(self.origin, (sub_cx, sub_cy))
        self.current_angle = p1_angle
        self._filtered_angle = p1_angle
        self.last_frame_dot_angle = p1_angle
        self.accumulated_rotation_deg = 0.0
        self._motion_trigger_count = 0

        rad = math.radians(p1_angle)
        sin_a = math.sin(rad)
        cos_a = math.cos(rad)
        arm_px = []
        for f in (0.35, 0.55, 0.75):
            ax = int(round(ox + f * self.radius * sin_a))
            ay = int(round(oy - f * self.radius * cos_a))
            if 0 <= ax < w and 0 <= ay < h:
                arm_px.append(float(gray[ay, ax]))
        if arm_px:
            self.calibrated_arm_brightness = float(np.mean(arm_px))

        c_r = max(3, int(round(3 * scale)))
        c_patch = gray[max(0, target_pt[1] - c_r):target_pt[1] + c_r + 1,
                       max(0, target_pt[0] - c_r):target_pt[0] + c_r + 1]
        if c_patch.size > 0:
            self.calibrated_dot_darkness = float(np.mean(c_patch))
        self.calibrated_contrast = max(40.0, self.calibrated_arm_brightness - self.calibrated_dot_darkness)

        p1 = {
            "num": 1,
            "pt": target_pt,
            "angle": p1_angle
        }
        self.locked_positions = [p1]
        self.measurements.clear()
        self.recent_angles.clear()
        self.is_moving = False
        return target_pt

    def detect_black_dot(self, frame_bgr: np.ndarray,
                         frame_gray: Optional[np.ndarray] = None) -> Tuple[Optional[Tuple[int, int]], Optional[float]]:

        if self.origin is None or self.radius is None:
            return None, None

        ox, oy = self.origin
        R = self.radius
        gray = frame_gray if frame_gray is not None else (
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr
        )
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV) if frame_bgr.ndim == 3 else None
        h, w = gray.shape[:2]

        scale = max(1.0, h / 720.0)
        c_r = max(3, int(round(3 * scale)))
        s_r = max(10, int(round(12 * scale)))
        win_r = max(8, int(round(9 * scale)))
        ksize = 3 if scale < 1.5 else 5
        blurred = cv2.GaussianBlur(gray, (ksize, ksize), 0)

        angles_deg = np.arange(0, 360, 1.0)
        best_score = -999999.0
        best_angle_deg = None

        min_arm_gate = max(75.0, self.calibrated_arm_brightness * 0.45)

        for a_deg in angles_deg:
            a_rad = math.radians(a_deg)
            sin_a = math.sin(a_rad)
            cos_a = math.cos(a_rad)

            cx = int(round(ox + R * sin_a))
            cy = int(round(oy - R * cos_a))

            if not (s_r + 2 <= cx < w - s_r - 2 and s_r + 2 <= cy < h - s_r - 2):
                continue

            if hsv is not None:
                wire_color = False
                for f in (0.35, 0.55, 0.75):
                    ax = int(round(ox + f * R * sin_a))
                    ay = int(round(oy - f * R * cos_a))
                    if 0 <= ax < w and 0 <= ay < h:
                        if int(hsv[ay, ax, 1]) > 70:
                            wire_color = True
                            break
                if wire_color:
                    continue
                if int(hsv[cy, cx, 1]) > 70:
                    continue

            arm_pixels = []
            for f in (0.35, 0.55, 0.75):
                ax = int(round(ox + f * R * sin_a))
                ay = int(round(oy - f * R * cos_a))
                if 0 <= ax < w and 0 <= ay < h:
                    arm_pixels.append(float(blurred[ay, ax]))
            arm_val = float(np.mean(arm_pixels)) if arm_pixels else 0.0

            if arm_val < min_arm_gate:
                continue

            c_val = float(np.mean(blurred[cy - c_r:cy + c_r + 1, cx - c_r:cx + c_r + 1]))

            s_val = float(np.mean(blurred[cy - s_r:cy + s_r + 1, cx - s_r:cx + s_r + 1]))

            contrast = s_val - c_val

            if contrast < 18.0:
                continue

            score = (contrast * 2.0) + arm_val - c_val

            if score > best_score:
                best_score = score
                best_angle_deg = a_deg

        if best_angle_deg is None:
            return self.current_pt, self.current_angle

        best_rad = math.radians(best_angle_deg)
        approx_x = int(round(ox + R * math.sin(best_rad)))
        approx_y = int(round(oy - R * math.cos(best_rad)))

        x1 = max(0, approx_x - win_r)
        y1 = max(0, approx_y - win_r)
        x2 = min(w, approx_x + win_r + 1)
        y2 = min(h, approx_y + win_r + 1)
        patch = gray[y1:y2, x1:x2]

        sub_x = float(approx_x)
        sub_y = float(approx_y)
        if patch.size > 0:
            inv = 255.0 - patch.astype(np.float32)
            thresh = np.percentile(inv, 60)
            inv[inv < thresh] = 0.0
            m = cv2.moments(inv)
            if m["m00"] > 0:
                sub_x = float(x1 + m["m10"] / m["m00"])
                sub_y = float(y1 + m["m01"] / m["m00"])

        det_angle = AngleTracker.point_to_angle(self.origin, (sub_x, sub_y))
        refined_pt = (int(round(sub_x)), int(round(sub_y)))
        return refined_pt, det_angle

    def process_frame(self, frame_bgr: np.ndarray) -> Optional[Dict[str, Any]]:
        if self.origin is None or self.radius is None or len(self.locked_positions) == 0:
            return None

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        pt, angle = self.detect_black_dot(frame_bgr, gray)
        if pt is None or angle is None:
            return None

        if self._filtered_angle is None:
            self._filtered_angle = angle
        else:
            diff = (angle - self._filtered_angle + 180.0) % 360.0 - 180.0
            if abs(diff) < 0.8:
                self._filtered_angle = (self._filtered_angle + 0.15 * diff) % 360.0
            else:
                self._filtered_angle = angle

        self.current_pt = pt
        self.current_angle = self._filtered_angle

        if self.last_frame_dot_angle is None:
            self.last_frame_dot_angle = angle

        frame_delta = (angle - self.last_frame_dot_angle + 180.0) % 360.0 - 180.0
        self.last_frame_dot_angle = angle

        last_locked = self.locked_positions[-1]
        disp_from_locked = (angle - last_locked["angle"] + 180.0) % 360.0 - 180.0

        if not self.is_moving:
            if abs(disp_from_locked) >= config.AUTO_MOTION_TRIGGER_DEG and abs(frame_delta) < 30.0:
                self._motion_trigger_count += 1
                if self._motion_trigger_count >= 3:
                    self.is_moving = True
                    self._motion_trigger_count = 0
                    self.accumulated_rotation_deg = disp_from_locked
                    self.recent_angles.clear()
                    self.recent_angles.append(angle)
            else:
                self._motion_trigger_count = 0
        else:

            if abs(frame_delta) < 45.0:
                self.accumulated_rotation_deg += frame_delta
            self.recent_angles.append(angle)

            if len(self.recent_angles) == self.recent_angles.maxlen:
                base_ang = self.recent_angles[0]
                diffs = [((a - base_ang + 180.0) % 360.0 - 180.0) for a in self.recent_angles]
                spread = max(diffs) - min(diffs)
                if spread < config.MOTION_SETTLE_SPREAD_DEG and abs(self.accumulated_rotation_deg) >= config.AUTO_MOTION_TRIGGER_DEG:

                    return self.lock_position()

        return None

    def lock_position(self) -> Optional[Dict[str, Any]]:
        if not self.locked_positions or self.current_pt is None or self.current_angle is None:
            return None

        prev_pos = self.locked_positions[-1]
        new_num = prev_pos["num"] + 1

        if len(self.recent_angles) >= 3:
            base_a = self.recent_angles[0]
            diffs = [((a - base_a + 180.0) % 360.0 - 180.0) for a in self.recent_angles]
            settled_curr_angle = (base_a + (sum(diffs) / len(diffs))) % 360.0
        else:
            settled_curr_angle = self.current_angle

        prev_angle = prev_pos["angle"]
        in_flight_accum = self.accumulated_rotation_deg

        if abs(in_flight_accum) >= 1.0:
            is_cw = in_flight_accum > 0
            if is_cw:
                motion = "Clockwise"
                static_frac = (settled_curr_angle - prev_angle) % 360.0
                full_revs = max(0, int(round((in_flight_accum - static_frac) / 360.0)))
                exact_angle = full_revs * 360.0 + static_frac
                signed_delta = exact_angle
            else:
                motion = "Anti-Clockwise"
                static_frac = (prev_angle - settled_curr_angle) % 360.0
                full_revs = max(0, int(round((abs(in_flight_accum) - static_frac) / 360.0)))
                exact_angle = full_revs * 360.0 + static_frac
                signed_delta = -exact_angle
        else:

            diff = (settled_curr_angle - prev_angle + 180.0) % 360.0 - 180.0
            if diff > 0.05:
                motion = "Clockwise"
            elif diff < -0.05:
                motion = "Anti-Clockwise"
            else:
                motion = "None"
            exact_angle = abs(diff)
            signed_delta = diff

        signed_delta = round(signed_delta, 2)
        abs_delta = round(exact_angle, 2)

        if abs_delta < config.AUTO_MOTION_TRIGGER_DEG:
            self.is_moving = False
            self.recent_angles.clear()
            self.accumulated_rotation_deg = 0.0
            return None

        new_pos = {
            "num": new_num,
            "pt": self.current_pt,
            "angle": round(settled_curr_angle, 2)
        }
        self.locked_positions.append(new_pos)
        self.is_moving = False
        self.recent_angles.clear()
        self.accumulated_rotation_deg = 0.0
        self.last_frame_dot_angle = settled_curr_angle
        self.current_angle = round(settled_curr_angle, 2)
        self._filtered_angle = round(settled_curr_angle, 2)

        color_idx = (len(self.measurements)) % len(config.ROTATION_ARC_COLORS)
        arc_color = config.ROTATION_ARC_COLORS[color_idx]

        rec = {
            "rotation_num": len(self.measurements) + 1,
            "from_point_num": prev_pos["num"],
            "to_point_num": new_num,
            "from_pt": prev_pos["pt"],
            "to_pt": new_pos["pt"],
            "prev_angle": prev_pos["angle"],
            "curr_angle": new_pos["angle"],
            "signed_delta": signed_delta,
            "angle_deg": abs_delta,
            "motion": motion,
            "color": arc_color
        }
        self.measurements.append(rec)
        return rec

    def render_overlays(self, frame: np.ndarray, mouse_pos: Optional[Tuple[int, int]],
                        show_crosshair: bool) -> None:

        if self.origin is not None:
            self._draw_origin(frame, self.origin)

        num_locked = len(self.locked_positions)

        if num_locked == 1:

            p1 = self.locked_positions[0]
            self._draw_radial_line(frame, self.origin, p1["pt"])
            self._draw_position_dot(frame, p1["pt"], p1["num"])

            if self.is_moving and self.current_pt is not None and self.current_angle is not None:
                self._draw_live_line(frame, self.origin, self.current_pt)
                signed_delta = self.accumulated_rotation_deg
                abs_delta = abs(signed_delta)
                if abs_delta > 0.5:
                    self._draw_angle_arc(
                        frame, self.origin, p1["pt"], self.current_pt,
                        signed_delta, abs_delta, (0, 255, 255), None
                    )

        elif num_locked >= 2:

            prev_pos = self.locked_positions[-2]
            curr_pos = self.locked_positions[-1]
            last_meas = self.measurements[-1] if self.measurements else None

            if not self.is_moving:

                self._draw_radial_line(frame, self.origin, prev_pos["pt"])
                self._draw_position_dot(frame, prev_pos["pt"], prev_pos["num"])

                self._draw_radial_line(frame, self.origin, curr_pos["pt"])
                self._draw_position_dot(frame, curr_pos["pt"], curr_pos["num"])

                if last_meas:
                    self._draw_angle_arc(
                        frame, self.origin, prev_pos["pt"], curr_pos["pt"],
                        last_meas["signed_delta"], last_meas["angle_deg"],
                        last_meas["color"], last_meas["rotation_num"]
                    )
            else:

                self._draw_radial_line(frame, self.origin, curr_pos["pt"])
                self._draw_position_dot(frame, curr_pos["pt"], curr_pos["num"])

                if self.current_pt is not None and self.current_angle is not None:

                    self._draw_live_line(frame, self.origin, self.current_pt)

                    signed_delta = self.accumulated_rotation_deg
                    abs_delta = abs(signed_delta)
                    if abs_delta > 0.5:
                        self._draw_angle_arc(
                            frame, self.origin, curr_pos["pt"], self.current_pt,
                            signed_delta, abs_delta, (0, 255, 255), None
                        )

        if show_crosshair and mouse_pos is not None:
            self._draw_crosshair(frame, mouse_pos)

    @staticmethod
    def _draw_origin(frame: np.ndarray, origin: Tuple[int, int]) -> None:
        ox, oy = origin
        r = config.ORIGIN_MARKER_RADIUS

        cv2.circle(frame, (ox, oy), r + 4, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.circle(frame, (ox, oy), r + 4, config.COLOR_ORIGIN, 1, cv2.LINE_AA)
        cv2.circle(frame, (ox, oy), 3, (0, 0, 255), -1, cv2.LINE_AA)

        cv2.line(frame, (ox - r - 6, oy), (ox + r + 6, oy), config.COLOR_ORIGIN, 1, cv2.LINE_AA)
        cv2.line(frame, (ox, oy - r - 6), (ox, oy + r + 6), config.COLOR_ORIGIN, 1, cv2.LINE_AA)

        lbl = f"Origin O ({ox},{oy})"
        cv2.putText(frame, lbl, (ox + 10, oy + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, lbl, (ox + 10, oy + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, config.COLOR_ORIGIN, 1, cv2.LINE_AA)

    @staticmethod
    def _draw_radial_line(frame: np.ndarray, origin: Tuple[int, int],
                          pt: Tuple[int, int]) -> None:

        color = config.COLOR_PROTRACTOR_LINE
        cv2.line(frame, origin, pt, (0, 0, 0), config.LINE_THICKNESS_RADIAL + 2, cv2.LINE_AA)
        cv2.line(frame, origin, pt, color, config.LINE_THICKNESS_RADIAL, cv2.LINE_AA)

    @staticmethod
    def _draw_live_line(frame: np.ndarray, origin: Tuple[int, int],
                        pt: Tuple[int, int]) -> None:

        cv2.line(frame, origin, pt, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.line(frame, origin, pt, config.COLOR_PREVIEW_LINE, 2, cv2.LINE_AA)
        cv2.circle(frame, pt, 6, config.COLOR_PREVIEW_LINE, -1, cv2.LINE_AA)

    @staticmethod
    def _draw_position_dot(frame: np.ndarray, pt: Tuple[int, int], number: int) -> None:
        px, py = pt
        r = config.DOT_MARKER_RADIUS

        cv2.circle(frame, (px, py), r + 2, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.circle(frame, (px, py), r, config.COLOR_PROTRACTOR_DOT, -1, cv2.LINE_AA)
        cv2.circle(frame, (px, py), r, (255, 255, 255), 1, cv2.LINE_AA)

        num_str = str(number)
        (tw, th), _ = cv2.getTextSize(num_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.putText(frame, num_str, (px - tw // 2, py + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, num_str, (px - tw // 2, py + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

    @staticmethod
    def _draw_angle_arc(frame: np.ndarray, origin: Tuple[int, int],
                        p1: Tuple[int, int], p2: Tuple[int, int],
                        signed_delta: float, angle_deg: float,
                        color: Tuple[int, int, int], rot_num: Optional[int]) -> None:

        if angle_deg < 0.2:
            return

        start_angle, end_angle, mid_angle = AngleTracker.get_arc_parameters(
            origin, p1, p2, signed_delta
        )

        d1 = math.hypot(p1[0] - origin[0], p1[1] - origin[1])
        d2 = math.hypot(p2[0] - origin[0], p2[1] - origin[1])
        base_r = int(min(d1, d2) * 0.44)
        offset = 0 if rot_num is None else ((rot_num - 1) % 3) * 16
        arc_r = max(40, min(base_r + offset, 160))

        num_full_revs = int(angle_deg // 360.0)
        rem_deg = angle_deg % 360.0

        for rev_i in range(1, num_full_revs + 1):
            ring_r = max(24, arc_r - (rev_i * 12))
            cv2.circle(frame, origin, ring_r, (0, 0, 0), config.ARC_THICKNESS + 2, cv2.LINE_AA)
            cv2.circle(frame, origin, ring_r, color, config.ARC_THICKNESS, cv2.LINE_AA)

        if rem_deg >= 0.5 or num_full_revs == 0:
            cv2.ellipse(frame, origin, (arc_r, arc_r), 0,
                        start_angle, end_angle, (0, 0, 0), config.ARC_THICKNESS + 2, cv2.LINE_AA)
            cv2.ellipse(frame, origin, (arc_r, arc_r), 0,
                        start_angle, end_angle, color, config.ARC_THICKNESS, cv2.LINE_AA)

        if rem_deg < 5.0 and num_full_revs > 0:
            screen_ref = AngleTracker.point_to_screen_angle(origin, p1)
            mid_angle = screen_ref + (35.0 if signed_delta >= 0 else -35.0)

        text_str = f"{angle_deg:.2f}°"
        rad = math.radians(mid_angle)
        text_r = arc_r + 28
        tx = int(origin[0] + text_r * math.cos(rad))
        ty = int(origin[1] + text_r * math.sin(rad))

        (tw, th), _ = cv2.getTextSize(text_str, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        bx1 = tx - 4
        by1 = ty - th - 4
        bx2 = tx + tw + 4
        by2 = ty + 4

        h, w = frame.shape[:2]
        bx1 = max(4, min(w - tw - 12, bx1))
        by1 = max(4, min(h - th - 12, by1))
        bx2 = bx1 + tw + 8
        by2 = by1 + th + 8

        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (255, 255, 255), -1)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 2)
        cv2.putText(frame, text_str, (bx1 + 4, by2 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)

    @staticmethod
    def _draw_crosshair(frame: np.ndarray, mouse_pos: Tuple[int, int]) -> None:
        mx, my = mouse_pos
        h, w = frame.shape[:2]

        if not (0 <= mx < w and 0 <= my < h):
            return

        color = config.COLOR_CROSSHAIR_ACTIVE

        cv2.line(frame, (0, my), (w, my), (0, 0, 0), 2, cv2.LINE_AA)
        cv2.line(frame, (0, my), (w, my), color, 1, cv2.LINE_AA)
        cv2.line(frame, (mx, 0), (mx, h), (0, 0, 0), 2, cv2.LINE_AA)
        cv2.line(frame, (mx, 0), (mx, h), color, 1, cv2.LINE_AA)

        cv2.circle(frame, (mx, my), 15, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.circle(frame, (mx, my), 15, color, 1, cv2.LINE_AA)
        cv2.circle(frame, (mx, my), 2, color, -1, cv2.LINE_AA)

        coord_text = f"X:{mx} Y:{my}"
        cv2.putText(frame, coord_text, (mx + 8, my - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, coord_text, (mx + 8, my - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    @staticmethod
    def draw_hud(frame: np.ndarray, step_text: str, fps: float) -> None:
        h, w = frame.shape[:2]
        hud_h = 34

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, hud_h), config.COLOR_HUD_BG, -1)
        cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)

        cv2.putText(frame, step_text, (14, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)

        fps_text = f"FPS: {fps:.1f}"
        cv2.putText(frame, fps_text, (w - 100, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)