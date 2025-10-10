#!/usr/bin/env python3
# lidar_system.py
# LiDAR high-level processing helpers.
# Author: Daniel Würmli

from ..low_level.lidar_driver import MS200Driver
import threading

def _angdiff(a, b):
    """Smallest angle difference in degrees (−180..+180, absolute value)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)

class LiDARSystem:
    """Either wraps an existing driver or instantiates one on demand.
    Angles (degrees):
      - 0   = right
      - 180 = left
      - 270 = front
    """
    def __init__(self, drv: MS200Driver = None, offset_mm: float = 42.0):
        # Use passed-in driver if provided, otherwise create a new one
        self._drv = drv if drv is not None else MS200Driver(offset_mm=offset_mm)
        self._lock = threading.Lock()

    # ---------- Raw access ----------
    def get_points(self):
        # List of (angle_deg, distance_mm, intensity)
        return self._drv.get_points()

    # ---------- Measurement helpers ----------
    def _distance_nearest(self, target_deg: float, window_deg: float = 1.0):
        """
        Select the measurement with the smallest angular difference near target_deg within ±window/2. Returns distance in mm or None.
        """
        pts = self.get_points()
        if not pts:
            return None
        half = window_deg * 0.5
        best = None
        best_dang = None
        for a, d, i in pts:
            dang = _angdiff(a, target_deg)
            if dang <= half:
                if best is None or dang < best_dang:
                    best = d
                    best_dang = dang
        return best

    def _distance_window(self, target_deg: float, span_deg: float = 10.0, mode: str = "median"):
        """
        Fenster-Auswertung um target_deg (± span/2). mode: 'median' oder 'min'
        """
        pts = self.get_points()
        if not pts:
            return None
        half = span_deg * 0.5
        vals = [d for (a, d, i) in pts if _angdiff(a, target_deg) <= half]
        if not vals:
            return None
        vals.sort()
        if mode == "min":
            return vals[0]
        n = len(vals)
        m = n // 2
        return vals[m] if (n % 2 == 1) else 0.5 * (vals[m - 1] + vals[m])

    # ---------- Exact single beams ----------
    def front_distance_exact(self):
        return self._distance_nearest(270.0, window_deg=1.0)

    def left_distance_exact(self):
        return self._distance_nearest(180.0, window_deg=1.0)

    def right_distance_exact(self):
        return self._distance_nearest(0.0, window_deg=1.0)

    # ---------- Windowed variants ----------
    def front_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(270.0, span_deg=span_deg, mode=mode)

    def left_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(180.0, span_deg=span_deg, mode=mode)

    def right_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(0.0, span_deg=span_deg, mode=mode)

    # ---------- Orientation helpers (wall alignment) ----------
    # Left side: 168° (rear), 192° (front) – small windows, using min for outlier robustness
    def left_back_mm(self, span_deg=6.0):
        return self._distance_window(168.0, span_deg=span_deg, mode="min")

    def left_front_mm(self, span_deg=6.0):
        return self._distance_window(192.0, span_deg=span_deg, mode="min")

    # Right side: 12° (rear), 348° (front)
    def right_back_mm(self, span_deg=6.0):
        return self._distance_window(12.0, span_deg=span_deg, mode="min")

    def right_front_mm(self, span_deg=6.0):
        return self._distance_window(348.0, span_deg=span_deg, mode="min")
