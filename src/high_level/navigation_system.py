#!/usr/bin/env python3
# navigation_system.py
# High-level navigation controller.
# Author: Daniel Würmli

import time, json, os

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
        db = self.lidar.left_back_mm(span_deg=6.0)   # 168°
        df = self.lidar.left_front_mm(span_deg=6.0)  # 192°
        if db is None or df is None: return 0
        diff = df - db
        yaw = int(diff * self.Kp_orient)
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    def _yaw_from_orientation_right(self):
        db = self.lidar.right_back_mm(span_deg=6.0)   # 12°
        df = self.lidar.right_front_mm(span_deg=6.0)  # 348°
        if db is None or df is None: return 0
        diff = df - db
        yaw = -int(diff * self.Kp_orient)
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    # ---------- Error → Yaw ----------
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
