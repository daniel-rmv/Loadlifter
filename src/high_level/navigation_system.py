#!/usr/bin/env python3
# navigation_system.py
# High-level navigation controller.
# Author: Daniel Würmli

import time, json, os, math

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
RESET  = "\033[0m"

def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

class NavigationSystem:
    def __init__(self, motor_sys, lidar_sys, cfg: dict):
        """
        All values are sourced from cfg; missing required keys raise ValueError.
        """
        self.motors = motor_sys
        self.lidar  = lidar_sys
        self.cfg    = cfg or {}

        def need(key):
            if key not in self.cfg:
                raise ValueError(f"NavigationSystem cfg missing key: {key}")
            return self.cfg[key]

        # --- Required configuration values ---
        self.left_target = float(need("left_target_mm"))
        self.front_stop  = float(need("front_stop_mm"))
        self.forward     = float(need("forward_mm_s"))
        self.left_mode   = str(need("left_mode"))
        self.front_mode  = str(need("front_mode"))

        self.Kp_err     = float(need("Kp_err"))
        self.Kp_orient  = float(need("Kp_orient"))
        self.MAX_YAW    = int(need("MAX_YAW"))
        self.MIN_YAW    = int(need("MIN_YAW"))
        self.TOL_MM     = float(need("TOL_MM"))

        self.GUARD_EXTRA = float(need("GUARD_EXTRA"))
        self.MIN_FWD     = float(need("MIN_FWD"))
        self.MAX_SLOWERR = float(need("MAX_SLOWERR"))
        self.Kp_center   = float(need("Kp_center"))

        self.calib_file  = need("calib_file")
        self.yaw_fast    = float(need("yaw_fast"))
        self.yaw_slow    = float(need("yaw_slow"))
        self.ratio_fast  = float(need("ratio_fast"))
        self.brake_opp   = float(need("brake_opp"))
        self.brake_time  = float(need("brake_time"))
        self.rotation_scaling = float(self.cfg.get("rotation_scaling", 1.0))
        self.rotation_scaling_90 = float(self.cfg.get("rotation_scaling_90", self.rotation_scaling))
        self.rotation_scaling_180 = float(self.cfg.get("rotation_scaling_180", self.rotation_scaling))

        self.right_open_mm              = float(need("right_open_mm"))
        self.extra_forward_after_open_s = float(need("extra_forward_after_open_s"))
        self.sidekick_initial_forward_s = float(need("sidekick_initial_forward_s"))
        self.side_dead_end_mm           = float(need("side_dead_end_mm"))
        self.side_rejoin_front_mm       = float(need("side_rejoin_front_mm"))
        self.side_final_forward_s       = float(need("side_final_forward_s"))
        self.right_open_require_rearm   = bool(need("right_open_require_rearm"))
        # rearm_mm is optional; defaults to None
        self.right_open_rearm_mm        = self.cfg.get("right_open_rearm_mm", None)

        # Optional buzzer timings
        self.buzzer_start_s = float(self.cfg.get("buzzer_start_s", 3.0))
        self.buzzer_end_s   = float(self.cfg.get("buzzer_end_s", 3.0))

        # Channel alignment tuning (optional)
        self.channel_center_tol_mm        = float(self.cfg.get("channel_center_tol_mm", 25.0))
        self.channel_front_center_tol_mm  = float(self.cfg.get("channel_front_center_tol_mm", 25.0))
        self.channel_orientation_tol_deg  = float(self.cfg.get("channel_orientation_tol_deg", 2.5))
        self.channel_align_max_iter       = int(self.cfg.get("channel_align_max_iter", 8))
        self.channel_front_max_mm         = float(self.cfg.get("channel_front_max_mm", 3000.0))
        self.channel_front_min_points     = int(self.cfg.get("channel_front_min_points", 15))
        self.channel_front_band_mm        = float(self.cfg.get("channel_front_band_mm", 80.0))
        self.channel_strafe_kp            = float(self.cfg.get("channel_strafe_kp", 0.06))
        self.channel_strafe_min_pulse     = int(self.cfg.get("channel_strafe_min_pulse", 8))
        self.channel_strafe_max_pulse     = int(self.cfg.get("channel_strafe_max_pulse", 20))
        self.channel_strafe_min_time_s    = float(self.cfg.get("channel_strafe_min_time_s", 0.12))
        self.channel_strafe_max_time_s    = float(self.cfg.get("channel_strafe_max_time_s", 0.45))
        self.channel_strafe_time_k        = float(self.cfg.get("channel_strafe_time_k", 0.0012))
        self.channel_strafe_step_mm       = float(self.cfg.get("channel_strafe_step_mm", 50.0))
        self.channel_rotation_kp          = float(self.cfg.get("channel_rotation_kp", 0.3))
        self.channel_rotation_min_deg     = float(self.cfg.get("channel_rotation_min_deg", 0.5))
        self.channel_rotation_max_deg     = float(self.cfg.get("channel_rotation_max_deg", 3.0))
        self.channel_rotation_step_deg    = float(self.cfg.get("channel_rotation_step_deg", 1.0))
        self.channel_orientation_max_steps = int(self.cfg.get("channel_orientation_max_steps", 12))
        self.channel_orientation_stall_limit = int(self.cfg.get("channel_orientation_stall_limit", 4))
        self.channel_orientation_improve_tol_deg = float(self.cfg.get("channel_orientation_improve_tol_deg", 0.15))
        self.channel_strafe_max_steps     = int(self.cfg.get("channel_strafe_max_steps", 8))
        self.channel_strafe_stall_limit   = int(self.cfg.get("channel_strafe_stall_limit", 4))
        self.channel_strafe_improve_tol_mm = float(self.cfg.get("channel_strafe_improve_tol_mm", 3.0))

    # ---------- Utility ----------
    def hard_zero(self, repeats=5, sleep_s=0.05):
        for _ in range(int(repeats)):
            try:
                if hasattr(self.motors, "drive_full"):
                    self.motors.drive_full(0.0, 0, 0)
                else:
                    self.motors.drive(forward_mm_s=0.0, yaw_pulses=0)
            except Exception:
                pass
            time.sleep(sleep_s)
        try:
            self.motors.stop()
        except Exception:
            pass

    def shutdown(self):
        self.hard_zero()

    @staticmethod
    def _normalize_angle_deg(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def _within_sector(angle: float, start: float, end: float) -> bool:
        angle = angle % 360.0
        start = start % 360.0
        end = end % 360.0
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end

    @staticmethod
    def _polar_to_xy(angle_deg: float, distance_mm: float):
        rad = math.radians(angle_deg)
        x = distance_mm * math.cos(rad)
        y = distance_mm * math.sin(rad)
        return (x, y)

    @staticmethod
    def _estimate_line_angle(points):
        if not points or len(points) < 2:
            return None
        mx = sum(p[0] for p in points) / len(points)
        my = sum(p[1] for p in points) / len(points)
        sxx = syy = sxy = 0.0
        for x, y in points:
            dx = x - mx
            dy = y - my
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy
        if sxx + syy == 0.0:
            return None
        angle_rad = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        return math.degrees(angle_rad)

    # ---------- LiDAR simple reads ----------
    def read_left_mm(self):
        return (self.lidar.left_distance_exact() if self.left_mode=="single"
                else self.lidar.left_distance_mm(span_deg=10.0))
    def read_right_mm(self):
        return (self.lidar.right_distance_exact() if self.left_mode=="single"
                else self.lidar.right_distance_mm(span_deg=10.0))
    def read_front_mm(self):
        return (self.lidar.front_distance_exact() if self.front_mode=="single"
                else self.lidar.front_distance_mm(span_deg=10.0))

    # ---------- Orientation terms ----------
    def _yaw_from_orientation_left(self):
        db = self.lidar.left_back_mm(span_deg=6.0)   # 168 deg
        df = self.lidar.left_front_mm(span_deg=6.0)  # 192 deg
        if db is None or df is None: return 0
        diff = df - db
        yaw = int(diff * self.Kp_orient)
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    def _yaw_from_orientation_right(self):
        db = self.lidar.right_back_mm(span_deg=6.0)   # 12 deg
        df = self.lidar.right_front_mm(span_deg=6.0)  # 348 deg
        if db is None or df is None: return 0
        diff = df - db
        yaw = -int(diff * self.Kp_orient)
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    # ---------- Error to yaw ----------
    def _yaw_from_error_left(self, err_mm):
        if abs(err_mm) <= self.TOL_MM: return 0
        mag = int(abs(err_mm) * self.Kp_err)
        mag = _clamp(mag, self.MIN_YAW, self.MAX_YAW)
        return +mag if err_mm > 0 else -mag

    def _yaw_from_error_right(self, err_mm):
        if abs(err_mm) <= self.TOL_MM: return 0
        mag = int(abs(err_mm) * self.Kp_err)
        mag = _clamp(mag, self.MIN_YAW, self.MAX_YAW)
        return -mag if err_mm > 0 else +mag

    def _fwd_scale_from_error(self, err_mm):
        e = min(abs(err_mm), self.MAX_SLOWERR)
        drop = 0.7 * (e / self.MAX_SLOWERR)
        return max(self.MIN_FWD, 1.0 - drop)

    # ---------- FOLLOW LEFT until front stop ----------
    def follow_left_until_stop(self):
        print(YELLOW + "[MODE] follow LEFT wall then STOP at front" + RESET)
        while True:
            df = self.read_front_mm()
            dl = self.read_left_mm()
            if df is None or dl is None:
                print(RED + "[WARN] LiDAR returned no values" + RESET)
                time.sleep(0.05); continue

            if df <= self.front_stop:
                print(RED + f"[STOP] front {df:.0f}mm <= {self.front_stop:.0f}mm" + RESET)
                self.motors.stop(); self.hard_zero()
                return

            err = dl - self.left_target
            yaw = _clamp(self._yaw_from_error_left(err) + self._yaw_from_orientation_left(),
                         -self.MAX_YAW, self.MAX_YAW)

            if dl < (self.left_target - self.GUARD_EXTRA):
                force = max(self.MIN_YAW, int((self.left_target - dl) * 0.1))
                yaw = -abs(force)

            fwd = self.forward * self._fwd_scale_from_error(err)
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            else:
                self.motors.drive_full(forward_mm_s=fwd, strafe_pulses=0, yaw_pulses=yaw)
            time.sleep(0.05)

    # ---------- FOLLOW RIGHT until front stop ----------
    def follow_right_until_stop(self):
        print(YELLOW + "[MODE] follow RIGHT wall then STOP at front" + RESET)
        while True:
            df = self.read_front_mm()
            dr = self.read_right_mm()
            if df is None or dr is None:
                print(RED + "[WARN] LiDAR returned no values" + RESET)
                time.sleep(0.05); continue

            if df <= self.front_stop:
                print(RED + f"[STOP] front {df:.0f}mm <= {self.front_stop:.0f}mm" + RESET)
                self.motors.stop(); self.hard_zero()
                return

            err = dr - self.left_target
            yaw = _clamp(self._yaw_from_error_right(err) + self._yaw_from_orientation_right(),
                         -self.MAX_YAW, self.MAX_YAW)

            if dr < (self.left_target - self.GUARD_EXTRA):
                force = max(self.MIN_YAW, int((self.left_target - dr) * 0.1))
                yaw = +abs(force)

            fwd = self.forward * self._fwd_scale_from_error(err)
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            else:
                self.motors.drive_full(forward_mm_s=fwd, strafe_pulses=0, yaw_pulses=yaw)
            time.sleep(0.05)

    # ---------- LEFT until right-open OR front-stop (with optional re-arm) ----------
    def follow_left_until_right_open_or_front(self, right_open_mm=None,
                                              require_rearm=None, rearm_below_mm=None):
        right_open_mm = float(right_open_mm if right_open_mm is not None else self.right_open_mm)
        if require_rearm is None:
            require_rearm = bool(self.right_open_require_rearm)
        if rearm_below_mm is None:
            rearm_below_mm = self.right_open_rearm_mm if self.right_open_rearm_mm is not None else right_open_mm
        rearm_below_mm = float(rearm_below_mm)

        armed = not require_rearm
        print(YELLOW + f"[MODE] LEFT-follow RO={right_open_mm:.0f} FRONT={self.front_stop:.0f}  armed={armed}" + RESET)

        while True:
            df = self.read_front_mm()
            dl = self.read_left_mm()
            dr = self.read_right_mm()
            if df is None or dl is None or dr is None:
                print(RED + "[WARN] LiDAR returned no values" + RESET)
                time.sleep(0.05); continue

            if df <= self.front_stop:
                print(RED + f"[EVENT] FRONT STOP @ {df:.0f}mm" + RESET)
                self.motors.stop(); self.hard_zero()
                return "front_stop"

            if not armed and dr < rearm_below_mm:
                armed = True
                print(BLUE + f"[RE-ARM] right={dr:.0f}mm < {rearm_below_mm:.0f}mm → armed=True" + RESET)

            if armed and dr >= right_open_mm:
                print(GREEN + f"[EVENT] RIGHT OPEN @ {dr:.0f}mm ≥ {right_open_mm:.0f}mm" + RESET)
                return "right_open"

            err = dl - self.left_target
            yaw = _clamp(self._yaw_from_error_left(err) + self._yaw_from_orientation_left(),
                         -self.MAX_YAW, self.MAX_YAW)
            if dl < (self.left_target - self.GUARD_EXTRA):
                force = max(self.MIN_YAW, int((self.left_target - dl) * 0.1))
                yaw = -abs(force)
            fwd = self.forward * self._fwd_scale_from_error(err)
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            else:
                self.motors.drive_full(forward_mm_s=fwd, strafe_pulses=0, yaw_pulses=yaw)
            time.sleep(0.05)

    # ---------- CENTERED forward until front-threshold ----------
    def centered_forward_until_front(self, front_thresh_mm=540.0):
        print(YELLOW + f"[MODE] CENTERED driving (front stop @ {front_thresh_mm:.0f}mm)" + RESET)
        Kp_center = self.Kp_center
        while True:
            df = self.read_front_mm()
            dl = self.read_left_mm()
            dr = self.read_right_mm()
            if df is None or dl is None or dr is None:
                print(RED + "[WARN] LiDAR returned no values" + RESET)
                time.sleep(0.05); continue

            if df <= front_thresh_mm:
                print(RED + f"[STOP] front {df:.0f}mm <= {front_thresh_mm:.0f}mm" + RESET)
                self.motors.stop(); self.hard_zero()
                return

            diff = dl - dr
            yaw  = int(_clamp(diff * Kp_center, -self.MAX_YAW, self.MAX_YAW))
            if abs(diff) <= self.TOL_MM:
                yaw = 0

            fwd = self.forward
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            else:
                self.motors.drive_full(forward_mm_s=fwd, strafe_pulses=0, yaw_pulses=yaw)
            time.sleep(0.05)

    def _collect_channel_stats(self, expect_front_wall=True):
        pts = self.lidar.get_points()
        if not pts:
            return None
        front_pts = []
        left_pts = []
        right_pts = []
        front_left = []
        front_right = []
        for ang, dist, _ in pts:
            if dist is None or dist <= 0:
                continue
            if self.channel_front_max_mm and dist > self.channel_front_max_mm:
                continue
            ang = ang % 360.0
            if self._within_sector(ang, 220.0, 320.0):
                xy = self._polar_to_xy(ang, dist)
                front_pts.append(xy)
                if xy[0] <= 0.0:
                    front_left.append(dist)
                else:
                    front_right.append(dist)
            if self._within_sector(ang, 150.0, 210.0):
                left_pts.append(self._polar_to_xy(ang, dist))
            if self._within_sector(ang, 330.0, 30.0):
                right_pts.append(self._polar_to_xy(ang, dist))

        left_dist = self.read_left_mm()
        right_dist = self.read_right_mm()
        diff_lr = None
        if left_dist is not None and right_dist is not None:
            diff_lr = left_dist - right_dist

        front_center = None
        front_band_center = None
        if front_pts:
            front_center = sum(x for x, _ in front_pts) / len(front_pts)
            closest_y = min(y for _, y in front_pts)
            band = self.channel_front_band_mm
            band_pts = [x for x, y in front_pts if abs(y - closest_y) <= band]
            if band_pts:
                front_band_center = sum(band_pts) / len(band_pts)

        front_angle = self._estimate_line_angle(front_pts) if front_pts else None
        left_angle = self._estimate_line_angle(left_pts) if left_pts else None
        right_angle = self._estimate_line_angle(right_pts) if right_pts else None

        orientation_terms = []
        if front_angle is not None:
            orientation_terms.append(self._normalize_angle_deg(front_angle))
        if left_angle is not None:
            orientation_terms.append(self._normalize_angle_deg(left_angle - 90.0))
        if right_angle is not None:
            orientation_terms.append(self._normalize_angle_deg(right_angle + 90.0))

        orientation_error = None
        if orientation_terms:
            orientation_error = sum(orientation_terms) / len(orientation_terms)

        front_span_diff = None
        if front_left and front_right:
            front_span_diff = (sum(front_left) / len(front_left)) - (sum(front_right) / len(front_right))

        return {
            "left_dist": left_dist,
            "right_dist": right_dist,
            "diff_lr": diff_lr,
            "front_center": front_center,
            "front_band_center": front_band_center,
            "front_angle": front_angle,
            "left_angle": left_angle,
            "right_angle": right_angle,
            "orientation_error": orientation_error,
            "front_span_diff": front_span_diff,
            "front_points": len(front_pts),
            "left_points": len(left_pts),
            "right_points": len(right_pts),
        }

    def _strafe_adjust(self, error_mm):
        if error_mm is None:
            return False
        sign = 1 if error_mm > 0 else -1
        magnitude = abs(float(error_mm))
        if magnitude < self.channel_center_tol_mm * 0.25:
            return False
        step_mm = min(magnitude, self.channel_strafe_step_mm)
        pulse_raw = step_mm * self.channel_strafe_kp
        pulse = max(self.channel_strafe_min_pulse,
                    min(self.channel_strafe_max_pulse, int(round(pulse_raw))))
        duration = step_mm * self.channel_strafe_time_k
        duration = max(self.channel_strafe_min_time_s,
                       min(self.channel_strafe_max_time_s, duration))
        direction = "left" if sign > 0 else "right"
        print(BLUE + f"[STRAFE] {direction} pulse={pulse} dur={duration:.2f}s (err={error_mm:.0f}mm)" + RESET)
        if sign > 0:
            if hasattr(self.motors, "strafe_left"):
                self.motors.strafe_left(pulse)
            else:
                self.motors.drive_full(0.0, strafe_pulses=+pulse, yaw_pulses=0)
        else:
            if hasattr(self.motors, "strafe_right"):
                self.motors.strafe_right(pulse)
            else:
                self.motors.drive_full(0.0, strafe_pulses=-pulse, yaw_pulses=0)
        time.sleep(duration)
        self.hard_zero()
        return True

    def _rotate_adjust(self, error_deg):
        if error_deg is None:
            return False
        sign = 1 if error_deg > 0 else -1
        magnitude = abs(float(error_deg))
        if magnitude < self.channel_orientation_tol_deg * 0.5:
            return False
        step = min(self.channel_rotation_step_deg,
                   self.channel_rotation_max_deg,
                   magnitude)
        if step < self.channel_rotation_min_deg:
            step = magnitude
        if step < self.channel_rotation_min_deg:
            return False
        if sign > 0:
            self.rotate_left_deg(step)
        else:
            self.rotate_right_deg(step)
        time.sleep(0.1)
        return True

    def align_storage_channel(self, expect_front_wall=True):
        print(YELLOW + "[ALIGN] Storage channel centering" + RESET)
        success = False
        orientation_valid_streak = 0
        strafe_valid_streak = 0
        orientation_best = None
        orientation_stall = 0
        orientation_attempts = 0
        orientation_sign_last = None
        orientation_flip_count = 0
        strafe_best = None
        strafe_stall = 0
        strafe_attempts = 0
        strafe_last_sign = None
        strafe_flip_attempted = False
        last_stats = None

        for _ in range(max(1, self.channel_align_max_iter)):
            stats = self._collect_channel_stats(expect_front_wall=expect_front_wall)
            if not stats:
                print(RED + "[WARN] LiDAR returned no points for channel alignment" + RESET)
                break
            last_stats = stats
            acted = False
            orientation_valid = False
            strafe_valid = False

            orientation_error = stats.get("orientation_error")
            if orientation_error is not None:
                cur = abs(orientation_error)
                if cur > self.channel_orientation_tol_deg:
                    if orientation_best is None or cur < orientation_best - self.channel_orientation_improve_tol_deg:
                        orientation_best = cur
                        orientation_stall = 0
                    else:
                        orientation_stall += 1

                    rotate_sign = 1 if orientation_error > 0 else -1
                    if orientation_sign_last is not None and rotate_sign != orientation_sign_last:
                        orientation_flip_count += 1
                    else:
                        orientation_flip_count = 0

                    if orientation_flip_count >= 1:
                        orientation_attempts = self.channel_orientation_max_steps
                        orientation_stall = self.channel_orientation_stall_limit
                    elif (orientation_attempts < self.channel_orientation_max_steps and
                          orientation_stall < self.channel_orientation_stall_limit):
                        if self._rotate_adjust(orientation_error):
                            orientation_attempts += 1
                            orientation_sign_last = rotate_sign
                            acted = True
                    if acted:
                        orientation_valid = False
                        strafe_valid = False
                        continue
                else:
                    orientation_best = cur if orientation_best is None else min(orientation_best, cur)
                    orientation_stall = 0
                    orientation_sign_last = 1 if orientation_error > 0 else (-1 if orientation_error < 0 else orientation_sign_last)
                    orientation_valid = True

            diff_lr = stats.get("diff_lr")
            front_band_center = stats.get("front_band_center")
            strafe_candidates = []
            if diff_lr is not None:
                strafe_candidates.append(("lr", diff_lr, self.channel_center_tol_mm))
            if (expect_front_wall and
                    stats.get("front_points", 0) >= self.channel_front_min_points and
                    front_band_center is not None):
                strafe_candidates.append(("front", -front_band_center, self.channel_front_center_tol_mm))

            if strafe_candidates:
                candidate = max(strafe_candidates, key=lambda item: abs(item[1]))
                source, value, tolerance = candidate
                cur = abs(value)
                if cur > tolerance:
                    improved = (strafe_best is None or cur < strafe_best - self.channel_strafe_improve_tol_mm)
                    if improved:
                        strafe_best = cur
                        strafe_stall = 0
                        strafe_flip_attempted = False
                    else:
                        strafe_stall += 1
                    if (strafe_attempts < self.channel_strafe_max_steps and
                        strafe_stall < self.channel_strafe_stall_limit):
                        target_value = value
                        sign = 1 if value > 0 else -1
                        if (not improved and
                                strafe_last_sign is not None and
                                sign == strafe_last_sign and
                                not strafe_flip_attempted):
                            target_value = -value
                            sign = -sign
                            strafe_flip_attempted = True
                        else:
                            strafe_flip_attempted = False
                        if self._strafe_adjust(target_value):
                            strafe_attempts += 1
                            strafe_last_sign = sign
                            acted = True
                    if acted:
                        orientation_valid = False
                        strafe_valid = False
                        continue
                else:
                    strafe_best = cur if strafe_best is None else min(strafe_best, cur)
                    strafe_stall = 0
                    strafe_flip_attempted = False
                    strafe_valid = True

            if orientation_valid:
                orientation_valid_streak += 1
            else:
                orientation_valid_streak = 0
            if strafe_valid:
                strafe_valid_streak += 1
            else:
                strafe_valid_streak = 0

            if orientation_valid_streak >= 2 and strafe_valid_streak >= 2:
                success = True
                break

        if not success and last_stats:
            orientation_error = last_stats.get("orientation_error")
            diff_lr = last_stats.get("diff_lr")
            front_band_center = last_stats.get("front_band_center")
            orientation_ok = (orientation_error is None or abs(orientation_error) <= self.channel_orientation_tol_deg)
            sides_ok = (diff_lr is None or abs(diff_lr) <= self.channel_center_tol_mm)
            front_ok = (front_band_center is None or abs(front_band_center) <= self.channel_front_center_tol_mm)
            success = orientation_ok and sides_ok and front_ok

        if success:
            print(GREEN + "[ALIGN] Channel alignment complete" + RESET)
        else:
            print(RED + "[WARN] Channel alignment incomplete" + RESET)
        self.hard_zero()
        return success

    def straight_forward_until_front(self, front_thresh_mm=540.0):
        print(YELLOW + f"[MODE] STRAIGHT driving (front stop @ {front_thresh_mm:.0f}mm)" + RESET)
        while True:
            df = self.read_front_mm()
            if df is None:
                print(RED + "[WARN] LiDAR returned no front distance" + RESET)
                time.sleep(0.05)
                continue
            if df <= front_thresh_mm:
                print(RED + f"[STOP] front {df:.0f}mm <= {front_thresh_mm:.0f}mm" + RESET)
                self.motors.stop()
                self.hard_zero()
                return
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=self.forward, yaw_pulses=0)
            else:
                self.motors.drive_full(forward_mm_s=self.forward, strafe_pulses=0, yaw_pulses=0)
            time.sleep(0.05)

    def channel_align_and_forward(self, front_thresh_mm=540.0, expect_front_wall=True):
        self.align_storage_channel(expect_front_wall=expect_front_wall)
        self.straight_forward_until_front(front_thresh_mm=front_thresh_mm)

    def move_to_front_distance(self, target_mm, tolerance_mm=15.0, max_iters=80, speed_mm_s=None,
                               maintain_center=False, expect_front_wall=True):
        target_mm = float(target_mm)
        tolerance_mm = max(1.0, float(tolerance_mm))
        base_speed = abs(float(speed_mm_s)) if speed_mm_s is not None else abs(self.forward)
        print(YELLOW + f"[ALIGN] Adjusting front distance to {target_mm:.0f}±{tolerance_mm:.0f}mm" + RESET)
        if maintain_center:
            self.align_storage_channel(expect_front_wall=expect_front_wall)
        val = None
        error = None
        for _ in range(int(max_iters)):
            val = self.read_front_mm()
            if val is None:
                print(RED + "[WARN] LiDAR returned no front distance during adjustment" + RESET)
                time.sleep(0.1)
                continue
            error = val - target_mm
            if abs(error) <= tolerance_mm:
                break
            direction = 1 if error > 0 else -1
            move_speed = base_speed
            duration = max(0.02, min(0.45, abs(error) / max(120.0, move_speed)))
            if direction > 0:
                if hasattr(self.motors, "drive"):
                    self.motors.drive(forward_mm_s=move_speed, yaw_pulses=0)
                else:
                    self.motors.drive_full(forward_mm_s=move_speed, strafe_pulses=0, yaw_pulses=0)
            else:
                if hasattr(self.motors, "drive"):
                    self.motors.drive(forward_mm_s=-move_speed, yaw_pulses=0)
                else:
                    self.motors.drive_full(forward_mm_s=-move_speed, strafe_pulses=0, yaw_pulses=0)
            time.sleep(duration)
            self.hard_zero()
            if maintain_center:
                self.align_storage_channel(expect_front_wall=expect_front_wall)
        else:
            print(RED + "[WARN] Front distance coarse adjustment did not converge" + RESET)
            return False

        self.hard_zero()
        time.sleep(0.08)
        val = self.read_front_mm()
        if val is not None and abs(val - target_mm) <= tolerance_mm:
            print(GREEN + f"[ALIGN] Front distance reached ({val:.0f}mm)" + RESET)
            return True

        print(YELLOW + "[ALIGN] Fine front distance tuning" + RESET)
        for _ in range(16):
            val = self.read_front_mm()
            if val is None:
                time.sleep(0.1)
                continue
            error = val - target_mm
            if abs(error) <= tolerance_mm:
                self.hard_zero()
                print(GREEN + f"[ALIGN] Front distance reached ({val:.0f}mm)" + RESET)
                return True
            move_speed = base_speed
            duration = max(0.02, min(0.25, abs(error) / max(100.0, move_speed)))
            if error > 0:
                if hasattr(self.motors, "drive"):
                    self.motors.drive(forward_mm_s=move_speed, yaw_pulses=0)
                else:
                    self.motors.drive_full(forward_mm_s=move_speed, strafe_pulses=0, yaw_pulses=0)
            else:
                if hasattr(self.motors, "drive"):
                    self.motors.drive(forward_mm_s=-move_speed, yaw_pulses=0)
                else:
                    self.motors.drive_full(forward_mm_s=-move_speed, strafe_pulses=0, yaw_pulses=0)
            time.sleep(duration)
            self.hard_zero()
            time.sleep(0.05)
            if maintain_center:
                self.align_storage_channel(expect_front_wall=expect_front_wall)

        print(RED + f"[WARN] Front distance adjustment incomplete (last error={error:.0f}mm)" + RESET)
        return False

    def wait_for_valid_scan(self, axes=("front",), timeout_s=1.0) -> bool:
        """Block until requested LiDAR axes report values or the timeout elapses."""
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            ok = True
            for axis in axes:
                if axis == "front":
                    ok = ok and (self.read_front_mm() is not None)
                elif axis == "left":
                    ok = ok and (self.read_left_mm() is not None)
                elif axis == "right":
                    ok = ok and (self.read_right_mm() is not None)
                else:
                    raise ValueError(f"Unknown LiDAR axis '{axis}'")
                if not ok:
                    break
            if ok:
                return True
            time.sleep(0.05)
        return False

    # ---------- Timed forward ----------
    def timed_forward(self, seconds):
        print(BLUE + f"[FORWARD] timed {seconds:.2f}s" + RESET)
        t_end = time.time() + float(seconds)
        while time.time() < t_end:
            if hasattr(self.motors, "drive"):
                self.motors.drive(forward_mm_s=self.forward, yaw_pulses=0)
            else:
                self.motors.drive_full(forward_mm_s=self.forward, strafe_pulses=0, yaw_pulses=0)
            time.sleep(0.05)
        self.motors.stop(); self.hard_zero()

    # ---------- Turns ----------
    def _load_turn_json(self):
        if not self.calib_file or not os.path.exists(self.calib_file):
            return None
        try:
            with open(self.calib_file, "r") as f:
                return json.load(f)
        except Exception:
            return None

    def _rotate_signed(self, target_deg=90.0, left_positive=True):
        # Rotation speed may come from calibration JSON; otherwise fall back to pulse/time heuristics.
        data = self._load_turn_json()
        dps_fast = dps_slow = None
        brake_deg = 0.0

        if data and "deg_per_s" in data and "left" in data["deg_per_s"]:
            # Only read calibrated dps_* values when available; otherwise leave them as None
            try:
                dps_fast = float(data["deg_per_s"]["left"].get(str(abs(self.yaw_fast)), 0.0) or 0.0)
                dps_slow = float(data["deg_per_s"]["left"].get(str(abs(self.yaw_slow)), 0.0) or 0.0)
            except Exception:
                dps_fast = dps_slow = None
        if data and "brake" in data and "left" in data["brake"]:
            for k, val in data["brake"]["left"].items():
                try:
                    if f"opp{abs(self.brake_opp)}" in k and f"t{self.brake_time:.2f}" in k:
                        brake_deg = float(val); break
                except Exception:
                    pass

        # If no dps values are calibrated, distribute the ratio using relative timings (no absolute DPS).
        # No hard defaults for dps_*; use loops with short slices and total time via forward/ratio
        base_scale = self.rotation_scaling if self.rotation_scaling > 0 else 1.0
        abs_target = abs(float(target_deg))
        if abs_target >= 175.0:
            specific_scale = self.rotation_scaling_180 if self.rotation_scaling_180 > 0 else base_scale
        elif abs_target <= 95.0:
            specific_scale = self.rotation_scaling_90 if self.rotation_scaling_90 > 0 else base_scale
        else:
            specific_scale = base_scale
        target_eff = (float(target_deg) + float(brake_deg)) * specific_scale
        fast_ratio = float(self.ratio_fast)
        slow_ratio = 1.0 - fast_ratio

        # If no dps_* values are given we approximate via fixed step time windows
        # total duration proportional to target_eff (heuristic) so rotation actually occurs
        if not dps_fast or not dps_slow:
            total_s = 0.012 * target_eff  # 12ms per degree as a heuristic baseline (tune via config ratios)
            t_fast = total_s * fast_ratio
            t_slow = total_s * slow_ratio
        else:
            fast_deg = target_eff * fast_ratio
            slow_deg = target_eff - fast_deg
            t_fast = fast_deg / dps_fast
            t_slow = slow_deg / dps_slow

        def _pulse(value, fallback=1):
            val = abs(float(value))
            if val < 1.0:
                val *= 100.0
            val = int(round(val))
            return fallback if val == 0 else val

        yaw_fast = _pulse(self.yaw_fast)
        yaw_slow = _pulse(self.yaw_slow)
        brake_opp_val = _pulse(self.brake_opp, fallback=0)

        if not left_positive:
            yaw_fast = -yaw_fast
            yaw_slow = -yaw_slow
            brake_opp = abs(brake_opp_val)
        else:
            brake_opp = -abs(brake_opp_val)

        print(YELLOW + f"[TURN] {'LEFT' if left_positive else 'RIGHT'} {target_deg:.1f}° (eff={target_eff:.1f}°)" + RESET)

        # FAST
        t_end = time.time() + t_fast
        while time.time() < t_end:
            if hasattr(self.motors, "yaw_spin"):
                self.motors.yaw_spin(yaw_fast)
            else:
                self.motors.drive_full(0.0, 0, yaw_fast)
            time.sleep(0.01)

        # SLOW
        t_end = time.time() + t_slow
        while time.time() < t_end:
            if hasattr(self.motors, "yaw_spin"):
                self.motors.yaw_spin(yaw_slow)
            else:
                self.motors.drive_full(0.0, 0, yaw_slow)
            time.sleep(0.01)

        # Counter braking
        if self.brake_opp != 0 and self.brake_time > 0:
            if hasattr(self.motors, "brake_yaw"):
                self.motors.brake_yaw(opposite_pulses=brake_opp, duration=self.brake_time)
            else:
                self.motors.drive_full(0.0, 0, brake_opp)
                time.sleep(self.brake_time)

        self.hard_zero()

    def rotate_left_deg(self, deg):  self._rotate_signed(target_deg=deg, left_positive=True)
    def rotate_right_deg(self, deg): self._rotate_signed(target_deg=deg, left_positive=False)
    def rotate_left_180_calibrated(self, target_deg=180.0): self.rotate_left_deg(target_deg)
