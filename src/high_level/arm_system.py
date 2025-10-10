#!/usr/bin/env python3
# arm_system.py
# High-level ArmSystem that wraps the low-level ServoController
# Author: Daniel Würmli

"""
High-level ArmSystem built on top of low_level.servo_controller
- High-level entry points: open_gripper(), wave(), pickup(), home(), apply_posture(), ...
- No calibration logic. Provides load_all()/unload_all() as well as shoulder_up()/shoulder_down().
"""

from typing import Dict, Tuple, Union, List
import os, json, time
from ..low_level.servo_controller import ServoController, DEFAULT_BAUD

# ---------- Joint IDs and limits ----------
JOINTS: Dict[str, int] = {
    'gripper'    : 1,
    'wrist_roll' : 2,
    'wrist_pitch': 3,
    'elbow'      : 4,
    'shoulder'   : 5,
    'base'       : 6,
}

# Pulse constants
GRIPPER_OPEN_PULSE   = 32
GRIPPER_CLOSE_PULSE  = 607
GRIPPER_OBJECT_PULSE = 422

WRIST_ROLL_LEFT_MAX_PULSE  = 25
WRIST_ROLL_RIGHT_MAX_PULSE = 995
WRIST_ROLL_ZERO_PULSE      = 500

WRIST_PITCH_DOWN_MAX_PULSE = 55
WRIST_PITCH_UP_MAX_PULSE   = 970
WRIST_PITCH_ZERO_PULSE     = 500

ELBOW_UP_MAX_PULSE   = 0
ELBOW_DOWN_MAX_PULSE = 1008
ELBOW_ZERO_PULSE     = 500

SHOULDER_FRONT_DEG = 29.52   # measured forward position
SHOULDER_BACK_DEG  = 208.08  # measured backward limit
SHOULDER_DOWN_MAX_PULSE = 655
SHOULDER_UP_MAX_PULSE   = 877
SHOULDER_ZERO_PULSE     = 500

BASE_RIGHT_MAX_PULSE = 0
BASE_LEFT_MAX_PULSE  = 1000
BASE_RIGHT_45_PULSE  = 305
BASE_LEFT_45_PULSE   = 693
BASE_RIGHT_90_PULSE  = 122
BASE_LEFT_90_PULSE   = 868
BASE_ZERO_PULSE      = 500

# Lookup tables
PULSE_RANGE: Dict[str, Tuple[int, int]] = {j: (0, 1000) for j in JOINTS}
DEG_RANGE:   Dict[str, Tuple[float, float]] = {j: (0.0, 240.0) for j in JOINTS}
SOFT_LIMITS: Dict[str, Tuple[float, float]] = DEG_RANGE.copy()
SOFT_LIMITS['shoulder'] = (SHOULDER_FRONT_DEG, SHOULDER_BACK_DEG)

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def deg_to_pulse(joint: str, deg: float) -> int:
    p0, p1 = PULSE_RANGE[joint]; d0, d1 = DEG_RANGE[joint]
    deg = clamp(deg, d0, d1)
    return int(round((deg - d0) * (p1 - p0) / (d1 - d0) + p0))

def pulse_to_deg(joint: str, pulse: int) -> float:
    p0, p1 = PULSE_RANGE[joint]; d0, d1 = DEG_RANGE[joint]
    pulse = clamp(pulse, p0, p1)
    return (pulse - p0) * (d1 - d0) / (p1 - p0) + d0

# Pose definitions
POSTURE_FRONT_DOWN = {'wrist_pitch': 192, 'elbow': 707, 'shoulder': 268, 'wrist_roll': 500}
def _mk_pose(base_pulse, gripper_pulse): return {'base': base_pulse, **POSTURE_FRONT_DOWN, 'gripper': gripper_pulse}
POSTURES = {
    'pose_base_zero'      : _mk_pose(BASE_ZERO_PULSE,      GRIPPER_OPEN_PULSE),
    'pose_base_left_45'   : _mk_pose(BASE_LEFT_45_PULSE,   GRIPPER_OPEN_PULSE),
    'pose_base_left_90'   : _mk_pose(BASE_LEFT_90_PULSE,   GRIPPER_OPEN_PULSE),
    'pose_base_left_max'  : _mk_pose(BASE_LEFT_MAX_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_45'  : _mk_pose(BASE_RIGHT_45_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_90'  : _mk_pose(BASE_RIGHT_90_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_max' : _mk_pose(BASE_RIGHT_MAX_PULSE, GRIPPER_OPEN_PULSE),

    'pose_base_zero_closed'      : _mk_pose(BASE_ZERO_PULSE,      GRIPPER_CLOSE_PULSE),
    'pose_base_left_45_closed'   : _mk_pose(BASE_LEFT_45_PULSE,   GRIPPER_CLOSE_PULSE),
    'pose_base_left_90_closed'   : _mk_pose(BASE_LEFT_90_PULSE,   GRIPPER_CLOSE_PULSE),
    'pose_base_left_max_closed'  : _mk_pose(BASE_LEFT_MAX_PULSE,  GRIPPER_CLOSE_PULSE),
    'pose_base_right_45_closed'  : _mk_pose(BASE_RIGHT_45_PULSE,  GRIPGER_CLOSE_PULSE) if False else _mk_pose(BASE_RIGHT_45_PULSE, GRIPPER_CLOSE_PULSE),
    'pose_base_right_90_closed'  : _mk_pose(BASE_RIGHT_90_PULSE,  GRIPPER_CLOSE_PULSE),
    'pose_base_right_max_closed' : _mk_pose(BASE_RIGHT_MAX_PULSE, GRIPPER_CLOSE_PULSE),
}

CUSTOM_POSE_FILE = os.path.expanduser('~/.armpi_postures.json')
if os.path.exists(CUSTOM_POSE_FILE):
    try:
        with open(CUSTOM_POSE_FILE, 'r') as f: POSTURES.update(json.load(f))
    except Exception as e:
        print("[WARN] Failed to load custom poses:", e)

HOME_FILE = os.path.expanduser('~/.armpi_home.json')
DEFAULT_DURATION_MS = 1200
BASE_DURATION_MS = 1200
GRIP_DURATION_MS = 700

class ArmSystem:
    def __init__(self, port: str = '/dev/serial0', baud: int = DEFAULT_BAUD, ids: Dict[str,int]=None, use_gpio: bool=True):
        self.bus = ServoController(port, baud, use_gpio=use_gpio)
        self.joints = ids if ids else JOINTS.copy()
        self.home_pose = self._load_home_file()

    # --- Basic ---
    def _sid(self, joint: Union[str,int]) -> int:
        if isinstance(joint, int): return joint
        return self.joints[joint]

    def _name(self, joint: Union[str,int]) -> str:
        if isinstance(joint, str): return joint
        for k, v in self.joints.items():
            if v == joint: return k
        return str(joint)

    def move_joint_deg(self, joint: Union[str,int], angle_deg: float, duration_ms: int = DEFAULT_DURATION_MS):
        jname = self._name(joint)
        angle_deg = clamp(angle_deg, *SOFT_LIMITS[jname])
        pulse = deg_to_pulse(jname, angle_deg)
        self.move_joint_pulse(joint, pulse, duration_ms)

    def move_joint_pulse(self, joint: Union[str,int], pulse: int, duration_ms: int = DEFAULT_DURATION_MS):
        self.bus.move_time_write(self._sid(joint), int(pulse), int(duration_ms))

    def move_pose_deg(self, angles, duration_ms: int = DEFAULT_DURATION_MS):
        if isinstance(angles, dict):
            for j, a in angles.items():
                self.move_joint_deg(j, a, duration_ms)
        else:
            base, shoulder, elbow, wrist_pitch, wrist_roll, gripper = angles
            for j, a in zip(['base','shoulder','elbow','wrist_pitch','wrist_roll','gripper'],
                            [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]):
                self.move_joint_deg(j, a, duration_ms)

    # --- Gripper ---
    def set_gripper_percent(self, percent: float, duration_ms: int = GRIP_DURATION_MS):
        percent = clamp(percent, 0, 100)
        pmin, pmax = GRIPPER_OPEN_PULSE, GRIPPER_CLOSE_PULSE
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('gripper', pulse, duration_ms)

    def open_gripper(self, duration_ms: int = GRIP_DURATION_MS):  self.move_joint_pulse('gripper', GRIPPER_OPEN_PULSE, duration_ms)
    def close_gripper(self, duration_ms: int = GRIP_DURATION_MS): self.move_joint_pulse('gripper', GRIPPER_CLOSE_PULSE, duration_ms)
    def gripper_object(self, duration_ms: int = GRIP_DURATION_MS): self.move_joint_pulse('gripper', GRIPPER_OBJECT_PULSE, duration_ms)

    # --- Wrist Roll / Pitch ---
    def roll_zero(self, duration_ms=DEFAULT_DURATION_MS):       self.move_joint_pulse('wrist_roll',  WRIST_ROLL_ZERO_PULSE, duration_ms)
    def roll_left_max(self, duration_ms=DEFAULT_DURATION_MS):   self.move_joint_pulse('wrist_roll',  WRIST_ROLL_LEFT_MAX_PULSE, duration_ms)
    def roll_right_max(self, duration_ms=DEFAULT_DURATION_MS):  self.move_joint_pulse('wrist_roll',  WRIST_ROLL_RIGHT_MAX_PULSE, duration_ms)
    def roll_percent(self, percent: float, duration_ms=DEFAULT_DURATION_MS):
        percent = clamp(percent, 0, 100)
        pmin, pmax = WRIST_ROLL_RIGHT_MAX_PULSE, WRIST_ROLL_LEFT_MAX_PULSE
        self.move_joint_pulse('wrist_roll', int(pmin + (pmax-pmin)*(percent/100.0)), duration_ms)

    def pitch_zero(self, duration_ms=DEFAULT_DURATION_MS):      self.move_joint_pulse('wrist_pitch', WRIST_PITCH_ZERO_PULSE, duration_ms)
    def pitch_percent(self, percent: float, duration_ms=DEFAULT_DURATION_MS):
        percent = clamp(percent, 0, 100)
        pmin, pmax = WRIST_PITCH_DOWN_MAX_PULSE, WRIST_PITCH_UP_MAX_PULSE
        self.move_joint_pulse('wrist_pitch', int(pmin + (pmax-pmin)*(percent/100.0)), duration_ms)

    # --- Elbow / Shoulder relative helpers ---
    def read_joint_pulse(self, joint): return self.bus.read_pos(self._sid(joint))
    def read_joint_deg(self, joint):
        raw = self.read_joint_pulse(joint)
        return None if raw is None else pulse_to_deg(self._name(joint), raw)

    def _rel_move_joint_deg(self, joint: str, ddeg: float, duration_ms: int = DEFAULT_DURATION_MS):
        cur = self.read_joint_deg(joint)
        if cur is None:
            lo, hi = SOFT_LIMITS[joint]
            cur = (lo + hi) / 2.0
        self.move_joint_deg(joint, cur + ddeg, duration_ms)

    def elbow_zero(self, duration_ms=DEFAULT_DURATION_MS):      self.move_joint_pulse('elbow', ELBOW_ZERO_PULSE, duration_ms)
    def elbow_percent(self, percent: float, duration_ms=DEFAULT_DURATION_MS):
        percent = clamp(percent, 0, 100)
        pmin, pmax = ELBOW_UP_MAX_PULSE, ELBOW_DOWN_MAX_PULSE
        self.move_joint_pulse('elbow', int(pmin + (pmax-pmin)*(percent/100.0)), duration_ms)

    def shoulder_zero(self, duration_ms=DEFAULT_DURATION_MS):   self.move_joint_pulse('shoulder', SHOULDER_ZERO_PULSE, duration_ms)
    def shoulder_percent(self, percent: float, duration_ms=DEFAULT_DURATION_MS):
        percent = clamp(percent, 0, 100)
        deg = SHOULDER_FRONT_DEG + (SHOULDER_BACK_DEG - SHOULDER_FRONT_DEG) * (percent / 100.0)
        self.move_joint_deg('shoulder', deg, duration_ms)

    # --- Base absolute helpers ---
    def base_zero(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_ZERO_PULSE, duration_ms)

    def base_left_45(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_RIGHT_45_PULSE, duration_ms)

    def base_right_45(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_LEFT_45_PULSE, duration_ms)

    def base_left_90(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_RIGHT_90_PULSE, duration_ms)

    def base_right_90(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_LEFT_90_PULSE, duration_ms)

    def base_left_max(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_RIGHT_MAX_PULSE, duration_ms)

    def base_right_max(self, duration_ms=BASE_DURATION_MS):
        self.move_joint_pulse('base', BASE_LEFT_MAX_PULSE, duration_ms)

    def base_percent(self, percent: float, duration_ms=BASE_DURATION_MS):
        percent = clamp(percent, 0, 100)
        pmin, pmax = BASE_RIGHT_MAX_PULSE, BASE_LEFT_MAX_PULSE
        self.move_joint_pulse('base', int(pmin + (pmax - pmin) * (percent / 100.0)), duration_ms)

    # >>> fehlten bei dir: jetzt vorhanden
    def elbow_up(self, deg, duration_ms=DEFAULT_DURATION_MS):      self._rel_move_joint_deg('elbow',     +abs(deg), duration_ms)
    def elbow_down(self, deg, duration_ms=DEFAULT_DURATION_MS):    self._rel_move_joint_deg('elbow',     -abs(deg), duration_ms)
    def wrist_pitch_up(self, deg, duration_ms=DEFAULT_DURATION_MS):   self._rel_move_joint_deg('wrist_pitch', +abs(deg), duration_ms)
    def wrist_pitch_down(self, deg, duration_ms=DEFAULT_DURATION_MS): self._rel_move_joint_deg('wrist_pitch', -abs(deg), duration_ms)
    def wrist_roll_left(self, deg, duration_ms=DEFAULT_DURATION_MS):  self._rel_move_joint_deg('wrist_roll',  +abs(deg), duration_ms)
    def wrist_roll_right(self, deg, duration_ms=DEFAULT_DURATION_MS): self._rel_move_joint_deg('wrist_roll',  -abs(deg), duration_ms)
    def turn_left(self, deg, duration_ms=BASE_DURATION_MS):     self._rel_move_joint_deg('base',      +abs(deg), duration_ms)
    def turn_right(self, deg, duration_ms=BASE_DURATION_MS):    self._rel_move_joint_deg('base',      -abs(deg), duration_ms)
    def shoulder_up(self, deg, duration_ms=DEFAULT_DURATION_MS):   self._rel_move_joint_deg('shoulder',  +abs(deg), duration_ms)
    def shoulder_down(self, deg, duration_ms=DEFAULT_DURATION_MS): self._rel_move_joint_deg('shoulder',  -abs(deg), duration_ms)

    # --- Home ---
    def _load_home_file(self, path: str = HOME_FILE):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f: d = json.load(f)
                return {k: float(v) for k, v in d.items() if k in JOINTS}
            except Exception:
                return None
        return None

    def set_home_from_current(self, save: bool = True, path: str = HOME_FILE):
        pose = {}
        missing = []
        for j in self.joints:
            raw = self.read_joint_pulse(j)
            if raw is None:
                # try degree reading as fallback
                deg = self.read_joint_deg(j)
                if deg is not None:
                    pose[j] = float(deg)
                    continue
                missing.append(j)
                continue
            pose[j] = float(pulse_to_deg(j, raw))
        if missing:
            raise RuntimeError("Keine Positionsdaten fuer: " + ", ".join(missing) + ". Zuerst --arm_load.")
        self.home_pose = pose
        if save:
            with open(path, 'w') as f: json.dump(pose, f, indent=2)
        return pose

    def clear_home_file(self, path: str = HOME_FILE):
        if os.path.exists(path): os.remove(path)

    def home(self, duration_ms=1800):
        if self.home_pose is None: self.factory_home(duration_ms)
        else: self.move_pose_deg(self.home_pose, duration_ms)

    def factory_home(self, duration_ms=1800):
        mids = {}
        for j in self.joints:
            pulse = GRIPPER_OPEN_PULSE if j == 'gripper' else 500
            mids[j] = pulse_to_deg(j, pulse)
        self.move_pose_deg(mids, duration_ms)

    # --- Aktionen ---
    def wave(self, reps=5, amp=25, duration_ms=300, prep_shoulder=40, prep_elbow=60, settle_ms=600):
        self.shoulder_up(prep_shoulder, settle_ms)
        self.elbow_up(prep_elbow, settle_ms)
        time.sleep(settle_ms / 1000)
        for _ in range(reps):
            self.wrist_roll_left(amp, duration_ms); time.sleep(duration_ms / 1000)
            self.wrist_roll_right(amp, duration_ms); time.sleep(duration_ms / 1000)

    def grip_object(self, duration_ms=GRIP_DURATION_MS): self.move_joint_pulse('gripper', GRIPPER_OBJECT_PULSE, duration_ms)

    def pickup(self, sh_down=18.0, el_down=35.0, lift=35.0, grip_percent_open=10.0, grip_percent_pick=55.0, final_close_percent=85.0, dur_small=DEFAULT_DURATION_MS, grip_time=GRIP_DURATION_MS):
        self.apply_posture('pose_base_zero', duration_ms=dur_small); time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_open, grip_time); time.sleep(grip_time / 1000)
        self.shoulder_down(sh_down, dur_small); self.elbow_down(el_down, dur_small); time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_pick, grip_time); time.sleep(grip_time / 1000)
        self.elbow_up(lift, dur_small); self.shoulder_up(sh_down, dur_small); time.sleep(dur_small / 1000)
        self.set_gripper_percent(final_close_percent, grip_time); time.sleep(grip_time / 1000)

    def getobject(self, base_to_pick=0.0, sh_down=18.0, el_down=35.0, lift=35.0, base_to_place=90.0, place_sh_down=18.0, place_el_down=35.0, dur_travel=BASE_DURATION_MS, dur_small=DEFAULT_DURATION_MS, grip_percent_open=10.0, grip_time=GRIP_DURATION_MS, grip_percent_pick=60.0):
        self.apply_posture('pose_base_zero', duration_ms=dur_small); time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_open, grip_time); time.sleep(grip_time / 1000)
        if base_to_pick != 0:
            (self.turn_right if base_to_pick < 0 else self.turn_left)(abs(base_to_pick), dur_travel); time.sleep(dur_travel / 1000)
        self.shoulder_down(sh_down, dur_small); self.elbow_down(el_down, dur_small); time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_pick, grip_time); time.sleep(grip_time / 1000)
        self.elbow_up(lift, dur_small); self.shoulder_up(sh_down, dur_small); time.sleep(dur_small / 1000)
        if base_to_place != 0:
            (self.turn_right if base_to_place < 0 else self.turn_left)(abs(base_to_place), dur_travel); time.sleep(dur_travel / 1000)
        self.shoulder_down(place_sh_down, dur_small); self.elbow_down(place_el_down, dur_small); time.sleep(dur_small / 1000)
        self.open_gripper(grip_time); time.sleep(grip_time / 1000)
        self.elbow_up(place_el_down, dur_small); self.shoulder_up(place_sh_down, dur_small); time.sleep(dur_small / 1000)

    def list_poses(self) -> List[str]: return list(POSTURES.keys())

    def apply_posture(self, name: str, duration_ms=800):
        if name not in POSTURES: raise ValueError(f"Pose '{name}' not found.")
        pose = POSTURES[name]
        if name.endswith('_closed'): self.close_gripper(400); time.sleep(0.4)
        else:                        self.open_gripper(300);  time.sleep(0.3)
        for j, p in pose.items():
            self.move_joint_pulse(j, int(p), duration_ms)

    # --- Power ---
    def unload_joint(self, joint): self.bus.unload(self._sid(joint))
    def load_joint(self, joint):   self.bus.load(self._sid(joint))

    def unload_all(self):
        for name in self.joints:
            try: self.unload_joint(name)
            except Exception: pass

    def load_all(self):
        for name in self.joints:
            try: self.load_joint(name)
            except Exception: pass

    def cleanup(self): self.bus.cleanup()
