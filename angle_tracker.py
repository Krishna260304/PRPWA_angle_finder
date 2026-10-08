import math
from typing import Tuple

class AngleTracker:
    @staticmethod
    def point_to_angle(center: Tuple[int, int], point: Tuple[int, int]) -> float:
        cx, cy = center
        px, py = point

        dx = px - cx
        dy = cy - py

        angle = math.degrees(math.atan2(dx, dy)) % 360.0
        return round(angle, 2)

    @staticmethod
    def point_to_screen_angle(center: Tuple[int, int], point: Tuple[int, int]) -> float:
        cx, cy = center
        px, py = point

        dx = px - cx
        dy = py - cy

        angle = math.degrees(math.atan2(dy, dx)) % 360.0
        return angle

    @staticmethod
    def calculate_angular_difference(
        previous_angle: float, current_angle: float
    ) -> Tuple[float, float, str]:
        delta = (current_angle - previous_angle + 180.0) % 360.0 - 180.0
        signed_delta = round(delta, 2)
        abs_delta = round(abs(delta), 2)

        if signed_delta > 0.05:
            motion = "Clockwise"
        elif signed_delta < -0.05:
            motion = "Anti-Clockwise"
        else:
            motion = "None"

        return signed_delta, abs_delta, motion

    @staticmethod
    def get_arc_parameters(
        center: Tuple[int, int],
        ref_point: Tuple[int, int],
        current_point: Tuple[int, int],
        signed_delta: float,
    ) -> Tuple[float, float, float]:
        screen_ref = AngleTracker.point_to_screen_angle(center, ref_point)
        rem_delta = (
            signed_delta % 360.0
            if signed_delta >= 0
            else -((-signed_delta) % 360.0)
        )

        if signed_delta >= 0:
            start_angle = screen_ref
            end_angle = screen_ref + abs(rem_delta)
            mid_angle = screen_ref + (abs(rem_delta) / 2.0)
        else:
            start_angle = screen_ref - abs(rem_delta)
            end_angle = screen_ref
            mid_angle = screen_ref - (abs(rem_delta) / 2.0)

        return start_angle, end_angle, mid_angle
