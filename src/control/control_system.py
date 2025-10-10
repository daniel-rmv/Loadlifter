#!/usr/bin/env python3
# control_system.py
# Main CLI orchestrator for robot control modes.
# Author: Daniel Würmli


import argparse, time, sys, json, os
from importlib import import_module

from ..high_level.motor_system import MotorSystem
from ..high_level.navigation_system import NavigationSystem
from ..low_level.lidar_driver import MS200Driver
from ..high_level.lidar_system import LiDARSystem
from ..high_level.buzzer_system import BuzzerSystem
from ..high_level.arm_system import ArmSystem
from ..high_level.camera_system import CameraSystem
from .modes.remote import run as run_remote_mode
from .modes.follow_wall import run as run_follow_wall
from .modes.follow_route import run as run_follow_route

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.normpath(os.path.join(BASE_DIR, "..", "utils", "robot_config.json"))

# ---------- Utils ----------
def die(msg: str, code: int = 1):
    print(f"[FATAL] {msg}")
    sys.exit(code)

def load_config(path: str) -> dict:
    if not path or not os.path.exists(path):
        die(f"Configuration file not found: {path}")
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
    except Exception as e:
        die(f"Configuration file could not be read: {e}")
    if not isinstance(cfg, dict):
        die("Configuration file does not contain a JSON object.")
    return cfg

def set_if_not_none(cfg: dict, key: str, value):
    if value is not None:
        cfg[key] = value

def parse_maybe_bool(val):
    if val is None:
        return None
    if isinstance(val, bool):
        return val
    s = str(val).strip().lower()
    if s in ("1","true","t","yes","y","on"):  return True
    if s in ("0","false","f","no","n","off"): return False
    raise ValueError(f"Invalid boolean value: {val!r}")


def _run_arm_mode(name: str, arm, **kwargs):
    try:
        module = import_module(f"{__package__}.modes.{name}")
    except ModuleNotFoundError as exc:
        die(f"Arm module '{name}' not found: {exc}")
    runner = getattr(module, "run", None)
    if runner is None:
        die(f"Arm module '{name}' does not define a run() function.")
    try:
        return runner(arm, **kwargs)
    except ValueError as exc:
        die(str(exc))


ARM_ACTIONS = [
    {"module": "list_poses", "check": lambda a: bool(a.arm_list_poses), "kwargs": lambda a: {}},
    {"module": "apply_posture", "check": lambda a: bool(a.arm_apply_posture), "kwargs": lambda a: {"pose": a.arm_apply_posture}},
    {"module": "apply_posture", "check": lambda a: bool(getattr(a, "arm_pose", None)), "kwargs": lambda a: {"pose": a.arm_pose}},
    {"module": "factory_home", "check": lambda a: bool(a.arm_factory_home), "kwargs": lambda a: {}},
    {"module": "home", "check": lambda a: bool(a.arm_home), "kwargs": lambda a: {}},
    {"module": "set_home", "check": lambda a: bool(a.arm_set_home), "kwargs": lambda a: {}},
    {"module": "clear_home", "check": lambda a: bool(a.arm_clear_home), "kwargs": lambda a: {}},
    {"module": "calibrate", "check": lambda a: bool(getattr(a, "arm_calibrate", False)), "kwargs": lambda a: {}},
    {"module": "pickup", "check": lambda a: bool(getattr(a, "arm_pickup", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_front_right", "check": lambda a: bool(getattr(a, "arm_getobject_floor_front_right", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_right_front", "check": lambda a: bool(getattr(a, "arm_getobject_floor_right_front", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_front_left", "check": lambda a: bool(getattr(a, "arm_getobject_floor_front_left", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_left_front", "check": lambda a: bool(getattr(a, "arm_getobject_floor_left_front", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_right_left", "check": lambda a: bool(getattr(a, "arm_getobject_floor_right_left", False)), "kwargs": lambda a: {}},
    {"module": "arm_getobject_floor_left_right", "check": lambda a: bool(getattr(a, "arm_getobject_floor_left_right", False)), "kwargs": lambda a: {}},
    {"module": "grip_object", "check": lambda a: bool(getattr(a, "arm_grip_object", False)), "kwargs": lambda a: {}},
    {"module": "gripper_open", "check": lambda a: bool(a.arm_gripper_open), "kwargs": lambda a: {}},
    {"module": "gripper_close", "check": lambda a: bool(a.arm_gripper_close), "kwargs": lambda a: {}},
    {"module": "gripper_object", "check": lambda a: bool(a.arm_gripper_object), "kwargs": lambda a: {}},
    {"module": "gripper_percent", "check": lambda a: a.arm_gripper_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_gripper_percent)}},
    {"module": "roll_zero", "check": lambda a: bool(a.arm_roll_zero), "kwargs": lambda a: {}},
    {"module": "roll_left_max", "check": lambda a: bool(a.arm_roll_left_max), "kwargs": lambda a: {}},
    {"module": "roll_right_max", "check": lambda a: bool(a.arm_roll_right_max), "kwargs": lambda a: {}},
    {"module": "roll_percent", "check": lambda a: a.arm_roll_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_roll_percent)}},
    {"module": "pitch_zero", "check": lambda a: bool(a.arm_pitch_zero), "kwargs": lambda a: {}},
    {"module": "pitch_percent", "check": lambda a: a.arm_pitch_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_pitch_percent)}},
    {"module": "elbow_zero", "check": lambda a: bool(a.arm_elbow_zero), "kwargs": lambda a: {}},
    {"module": "elbow_percent", "check": lambda a: a.arm_elbow_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_elbow_percent)}},
    {"module": "shoulder_zero", "check": lambda a: bool(a.arm_shoulder_zero), "kwargs": lambda a: {}},
    {"module": "shoulder_percent", "check": lambda a: a.arm_shoulder_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_shoulder_percent)}},
    {"module": "base_zero", "check": lambda a: bool(a.arm_base_zero), "kwargs": lambda a: {}},
    {"module": "base_left_45", "check": lambda a: bool(a.arm_base_left_45), "kwargs": lambda a: {}},
    {"module": "base_right_45", "check": lambda a: bool(a.arm_base_right_45), "kwargs": lambda a: {}},
    {"module": "base_left_90", "check": lambda a: bool(a.arm_base_left_90), "kwargs": lambda a: {}},
    {"module": "base_right_90", "check": lambda a: bool(a.arm_base_right_90), "kwargs": lambda a: {}},
    {"module": "base_left_max", "check": lambda a: bool(a.arm_base_left_max), "kwargs": lambda a: {}},
    {"module": "base_right_max", "check": lambda a: bool(a.arm_base_right_max), "kwargs": lambda a: {}},
    {"module": "base_percent", "check": lambda a: a.arm_base_percent is not None, "kwargs": lambda a: {"percent": float(a.arm_base_percent)}},
    {"module": "turn_left", "check": lambda a: a.arm_turn_left is not None, "kwargs": lambda a: {"degrees": float(a.arm_turn_left)}},
    {"module": "turn_right", "check": lambda a: a.arm_turn_right is not None, "kwargs": lambda a: {"degrees": float(a.arm_turn_right)}},
    {"module": "wrist_roll_left", "check": lambda a: a.arm_wrist_roll_left is not None, "kwargs": lambda a: {"degrees": float(a.arm_wrist_roll_left)}},
    {"module": "wrist_roll_right", "check": lambda a: a.arm_wrist_roll_right is not None, "kwargs": lambda a: {"degrees": float(a.arm_wrist_roll_right)}},
    {"module": "wrist_pitch_up", "check": lambda a: a.arm_wrist_pitch_up is not None, "kwargs": lambda a: {"degrees": float(a.arm_wrist_pitch_up)}},
    {"module": "wrist_pitch_down", "check": lambda a: a.arm_wrist_pitch_down is not None, "kwargs": lambda a: {"degrees": float(a.arm_wrist_pitch_down)}},
    {"module": "elbow_up", "check": lambda a: a.arm_elbow_up is not None, "kwargs": lambda a: {"degrees": float(a.arm_elbow_up)}},
    {"module": "elbow_down", "check": lambda a: a.arm_elbow_down is not None, "kwargs": lambda a: {"degrees": float(a.arm_elbow_down)}},
    {"module": "shoulder_up", "check": lambda a: a.arm_shoulder_up is not None, "kwargs": lambda a: {"degrees": float(a.arm_shoulder_up)}},
    {"module": "shoulder_down", "check": lambda a: a.arm_shoulder_down is not None, "kwargs": lambda a: {"degrees": float(a.arm_shoulder_down)}},
    {"module": "move_deg", "check": lambda a: a.arm_move_deg is not None, "kwargs": lambda a: {"joint": a.arm_move_deg[0], "degrees": float(a.arm_move_deg[1])}},
    {"module": "move_pulse", "check": lambda a: a.arm_move_pulse is not None, "kwargs": lambda a: {"joint": a.arm_move_pulse[0], "pulse": int(a.arm_move_pulse[1])}},
    {"module": "read_deg", "check": lambda a: bool(a.arm_read_deg), "kwargs": lambda a: {"joint": a.arm_read_deg}},
    {"module": "read_pulse", "check": lambda a: bool(a.arm_read_pulse), "kwargs": lambda a: {"joint": a.arm_read_pulse}},
    {"module": "unload_joint", "check": lambda a: bool(a.arm_unload_joint), "kwargs": lambda a: {"joint": a.arm_unload_joint}},
    {"module": "load_joint", "check": lambda a: bool(a.arm_load_joint), "kwargs": lambda a: {"joint": a.arm_load_joint}},
    {"module": "unload_all", "check": lambda a: bool(a.arm_unload), "kwargs": lambda a: {}},
    {"module": "load_all", "check": lambda a: bool(a.arm_load), "kwargs": lambda a: {}},
]

def main():
    ap = argparse.ArgumentParser(description="Robot Control System without calibration but with full arm flag support")

    # --- Robot-Modes ---
    ap.add_argument("--remote", action="store_true")
    ap.add_argument("--follow_wall", action="store_true")
    ap.add_argument("--follow_route", action="store_true")
    ap.add_argument("--camera_stream", action="store_true")
    ap.add_argument("--mode", choices=["remote","follow_wall","beep"], default=None)

    # --- ARM FLAGS (without calibration) ---
    ap.add_argument("--arm_port", default="/dev/serial0")
    ap.add_argument("--arm_baud", type=int, default=115200)
    ap.add_argument("--arm_no_gpio", action="store_true")

    # Home / Posen
    ap.add_argument("--arm_home", action="store_true")
    ap.add_argument("--arm_factory_home", action="store_true")
    ap.add_argument("--arm_set_home", action="store_true")
    ap.add_argument("--arm_clear_home", action="store_true")
    ap.add_argument("--arm_list_poses", action="store_true")
    ap.add_argument("--arm_apply_posture", type=str)
    ap.add_argument("--arm_pose", type=str)  # Alias (compatibility)
    ap.add_argument("--arm_calibrate", action="store_true")
    ap.add_argument("--arm_pickup", action="store_true")
    ap.add_argument("--arm_getobject_floor_front_right", action="store_true")
    ap.add_argument("--arm_getobject_floor_right_front", action="store_true")
    ap.add_argument("--arm_getobject_floor_front_left", action="store_true")
    ap.add_argument("--arm_getobject_floor_left_front", action="store_true")
    ap.add_argument("--arm_getobject_floor_right_left", action="store_true")
    ap.add_argument("--arm_getobject_floor_left_right", action="store_true")
    ap.add_argument("--arm_grip_object", action="store_true")

    # Gripper
    ap.add_argument("--arm_gripper_open", action="store_true")
    ap.add_argument("--arm_gripper_close", action="store_true")
    ap.add_argument("--arm_gripper_object", action="store_true")
    ap.add_argument("--arm_gripper_percent", type=float)

    # Roll
    ap.add_argument("--arm_roll_zero", action="store_true")
    ap.add_argument("--arm_roll_left_max", action="store_true")
    ap.add_argument("--arm_roll_right_max", action="store_true")
    ap.add_argument("--arm_roll_percent", type=float)

    # Pitch
    ap.add_argument("--arm_pitch_zero", action="store_true")
    ap.add_argument("--arm_pitch_percent", type=float)

    # Elbow
    ap.add_argument("--arm_elbow_zero", action="store_true")
    ap.add_argument("--arm_elbow_percent", type=float)

    # Shoulder
    ap.add_argument("--arm_shoulder_zero", action="store_true")
    ap.add_argument("--arm_shoulder_percent", type=float)

    # Base
    ap.add_argument("--arm_base_zero", action="store_true")
    ap.add_argument("--arm_base_left_45", action="store_true")
    ap.add_argument("--arm_base_right_45", action="store_true")
    ap.add_argument("--arm_base_left_90", action="store_true")
    ap.add_argument("--arm_base_right_90", action="store_true")
    ap.add_argument("--arm_base_left_max", action="store_true")
    ap.add_argument("--arm_base_right_max", action="store_true")
    ap.add_argument("--arm_base_percent", type=float)

    # Relative Moves
    ap.add_argument("--arm_turn_left", type=float)
    ap.add_argument("--arm_turn_right", type=float)
    ap.add_argument("--turn_degree", type=float)
    ap.add_argument("--arm_wrist_roll_left", type=float)
    ap.add_argument("--arm_wrist_roll_right", type=float)
    ap.add_argument("--arm_wrist_pitch_up", type=float)
    ap.add_argument("--arm_wrist_pitch_down", type=float)
    ap.add_argument("--arm_elbow_up", type=float)
    ap.add_argument("--arm_elbow_down", type=float)
    ap.add_argument("--arm_shoulder_up", type=float)
    ap.add_argument("--arm_shoulder_down", type=float)

    # Generische Moves & Readback
    ap.add_argument("--arm_move_deg", nargs=2, metavar=("JOINT","DEG"))
    ap.add_argument("--arm_move_pulse", nargs=2, metavar=("JOINT","PULSE"))
    ap.add_argument("--arm_read_deg", type=str, metavar="JOINT")
    ap.add_argument("--arm_read_pulse", type=str, metavar="JOINT")

    # Power
    ap.add_argument("--arm_unload", action="store_true")
    ap.add_argument("--arm_load", action="store_true")
    ap.add_argument("--arm_unload_joint", type=str)
    ap.add_argument("--arm_load_joint", type=str)

    # --- Config & Navigation Overrides ---
    ap.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--left_target", type=float, default=None)
    ap.add_argument("--front_stop",  type=float, default=None)

    args = ap.parse_args()

    # ---------- Turn-only helper ----------
    if args.turn_degree is not None:
        cfg = load_config(args.config)

        class _NullLidar:
            def left_distance_exact(self, *_, **__): return None
            def right_distance_exact(self, *_, **__): return None
            def front_distance_exact(self, *_, **__): return None
            def left_back_mm(self, *_, **__): return None
            def left_front_mm(self, *_, **__): return None
            def right_back_mm(self, *_, **__): return None
            def right_front_mm(self, *_, **__): return None

        motors = MotorSystem()
        nav = NavigationSystem(motors, _NullLidar(), cfg=cfg)
        try:
            deg = float(args.turn_degree)
            if deg >= 0:
                nav.rotate_right_deg(deg)
            else:
                nav.rotate_left_deg(-deg)
        finally:
            try:
                motors.stop()
            except Exception:
                pass
        return

    # ---------- ARM EARLY-EXIT ----------
    def _any_arm_flag_set(a):
        return any(action["check"](a) for action in ARM_ACTIONS)

    if _any_arm_flag_set(args):
        arm = ArmSystem(port=args.arm_port, baud=args.arm_baud, use_gpio=not args.arm_no_gpio)
        try:
            for action in ARM_ACTIONS:
                if not action["check"](args):
                    continue
                try:
                    kwargs = action["kwargs"](args)
                except ValueError as exc:
                    die(str(exc))
                except Exception as exc:
                    die(f"Arm-flag execution failed ({action['module']}): {exc}")
                _run_arm_mode(action["module"], arm, **kwargs)
        finally:
            try:
                arm.cleanup()
            except Exception:
                pass

        sys.exit(0)  # Arm commands finished

    # ---------- Robot-Modes ----------
    mode = None
    if args.remote: mode = "remote"
    elif args.follow_wall: mode = "follow_wall"
    elif args.follow_route: mode = "follow_route"
    elif args.camera_stream: mode = "camera_stream"
    else: mode = args.mode

    if mode is None:
        print("Usage: --remote | --follow_wall | --follow_route | --camera_stream")
        print("(Arm flags operate as early exits, e.g. --arm_home)")
        return

    cfg = load_config(args.config)

    # CLI to configuration overrides
    set_if_not_none(cfg, "left_target_mm", args.left_target)
    set_if_not_none(cfg, "front_stop_mm",  args.front_stop)
    camera_cfg = cfg.get("camera") if isinstance(cfg.get("camera"), dict) else None

    def _build_camera(default_enabled=True):
        if isinstance(camera_cfg, dict):
            return CameraSystem(
                source=camera_cfg.get("source", "0"),
                port=int(camera_cfg.get("port", 5000)),
                quality=int(camera_cfg.get("quality", 70)),
                outw=int(camera_cfg.get("outw", 960)),
                maxfps=int(camera_cfg.get("maxfps", 30)),
                host=camera_cfg.get("host", "0.0.0.0"),
            )
        return CameraSystem()

    def _bool_from(val, default=False):
        if val is None:
            return default
        try:
            parsed = parse_maybe_bool(val)
        except ValueError as exc:
            print(f"[WARN] Camera configuration boolean value invalid: {exc}")
            return default
        return default if parsed is None else bool(parsed)

    camera_sys = None
    if mode in ("remote", "follow_wall", "follow_route"):
        if camera_cfg and _bool_from(camera_cfg.get("enabled"), False):
            camera_sys = _build_camera()
            background = _bool_from(camera_cfg.get("background"), True)
            camera_sys.start(background=background)
    elif mode == "camera_stream":
        camera_sys = _build_camera()
        try:
            camera_sys.start(background=False)
        except KeyboardInterrupt:
            pass
        finally:
            camera_sys.stop()
        return

    # Remote / Route
    if mode == "remote":
        motors = MotorSystem()
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)
        try:
            run_remote_mode(motors, buzzer, cfg)
        except KeyboardInterrupt:
            pass
        finally:
            try: motors.stop()
            except Exception: pass
            try: buzzer.off()
            except Exception: pass
            buzzer.close()
        if camera_sys:
            camera_sys.stop()
        return

    elif mode in ("follow_wall", "follow_route"):
        motors = MotorSystem()
        drv   = MS200Driver()
        lidar = LiDARSystem(drv)
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)
        drv.start(); time.sleep(0.4)

        nav = NavigationSystem(motors, lidar, cfg=cfg)
        try:
            buzzer.on(); time.sleep(cfg.get("buzzer_start_s", 3.0)); buzzer.off()

            if mode == "follow_wall":
                run_follow_wall(nav, cfg, buzzer, drv)
            else:
                run_follow_route(nav, cfg, buzzer, drv)

        except KeyboardInterrupt:
            pass
        finally:
            try: nav.shutdown()
            except Exception: pass
            try: motors.stop()
            except Exception: pass
            try: drv.stop()
            except Exception: pass
            try: buzzer.off()
            except Exception: pass
            buzzer.close()
        if camera_sys:
            camera_sys.stop()
        return

    elif mode == "beep":
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)
        buzzer.on(); time.sleep(1.0); buzzer.off()
        buzzer.close()
        if camera_sys:
            camera_sys.stop()
        return

if __name__ == "__main__":
    main()
