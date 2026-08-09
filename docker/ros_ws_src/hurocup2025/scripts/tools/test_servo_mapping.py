#!/usr/bin/env python3
# encoding: utf-8
"""
test_servo_mapping.py — Verify joint→servo ID mapping using walking_module.so

Uses WalkingModule to compute the robot's standing pose, then nudges each
joint so you can physically confirm the correct servo moves.

IMPORTANT: Stop bringup before running — uses serial port directly.

Usage (run inside ainex container):
  python3 test_servo_mapping.py                     # sweep all 14 walking joints
  python3 test_servo_mapping.py --joint r_hip_yaw   # test one joint by name
  python3 test_servo_mapping.py --servo 1            # test one joint by servo ID
  python3 test_servo_mapping.py --joint l_knee --nudge 30  # custom nudge degrees
"""

import sys
import math
import time
import argparse
import yaml

sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/src')
sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/src')
sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ros_robot_controller/src')
from ainex_kinematics.kinematics import LegIK
from ainex_kinematics.walking_module import WalkingModule
from ainex_sdk import Board

# ---------------------------------------------------------------------------
# Constants — from ainex_controller.py lines 24–25
# ---------------------------------------------------------------------------
ENCODER_TICKS_PER_RADIAN = 180 / math.pi / 240 * 1000   # ≈ 238.73

# ---------------------------------------------------------------------------
# Joint → servo ID mapping — from ainex_controller.py lines 50–65
# (14 walking joints only; arms/head not controlled by WalkingModule)
# ---------------------------------------------------------------------------
JOINT_ID = {
    'l_ank_roll':   1,   'r_ank_roll':   2,
    'l_ank_pitch':  3,   'r_ank_pitch':  4,
    'l_knee':       5,   'r_knee':       6,
    'l_hip_pitch':  7,   'r_hip_pitch':  8,
    'l_hip_roll':   9,   'r_hip_roll':  10,
    'l_hip_yaw':   11,   'r_hip_yaw':   12,
    'l_sho_pitch': 13,   'r_sho_pitch': 14,
}

# Reverse map: servo_id → joint_name
SERVO_JOINT = {v: k for k, v in JOINT_ID.items()}

# joint_index for WalkingModule (order matches WalkingModule internal arrays)
JOINT_INDEX = {
    'r_hip_yaw':   0,  'r_hip_roll':  1,  'r_hip_pitch': 2,
    'r_knee':      3,  'r_ank_pitch': 4,  'r_ank_roll':  5,
    'l_hip_yaw':   6,  'l_hip_roll':  7,  'l_hip_pitch': 8,
    'l_knee':      9,  'l_ank_pitch': 10, 'l_ank_roll':  11,
    'r_sho_pitch': 12, 'l_sho_pitch': 13,
}

# Sweep order: ankle → knee → hip (proximal to distal, easier to observe)
SWEEP_ORDER = [
    'l_ank_roll', 'r_ank_roll',
    'l_ank_pitch', 'r_ank_pitch',
    'l_knee', 'r_knee',
    'l_hip_pitch', 'r_hip_pitch',
    'l_hip_roll', 'r_hip_roll',
    'l_hip_yaw', 'r_hip_yaw',
    'l_sho_pitch', 'r_sho_pitch',
]

# ---------------------------------------------------------------------------
# Servo configuration — from servo_controller.yaml
# ---------------------------------------------------------------------------
def load_servo_cfg(yaml_path):
    """Return {servo_id: {'init': int, 'flipped': bool}} for all 24 servos."""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    cfg = {}
    for ctl_params in data['controllers'].values():
        if ctl_params.get('type') != 'JointPositionController':
            continue
        s = ctl_params['servo']
        cfg[s['id']] = {
            'init':    s['init'],
            'flipped': s['min'] > s['max'],
        }
    return cfg


def angle2pulse(servo_id, angle_rad, servo_cfg):
    """Convert joint angle (radians) to servo pulse (0–1000)."""
    cfg = servo_cfg[servo_id]
    scale = -ENCODER_TICKS_PER_RADIAN if cfg['flipped'] else ENCODER_TICKS_PER_RADIAN
    return max(0, min(1000, cfg['init'] + int(round(angle_rad * scale))))


# ---------------------------------------------------------------------------
# Standing pose via WalkingModule.init_pose_tra()
# ---------------------------------------------------------------------------
def compute_standing_pose(wm):
    """
    Replay the init-pose trajectory and return the final standing-pose angles
    as {joint_name: angle_radians} for all 14 walking joints.
    """
    # __init__ already called init_pose_tra(self.present_position) internally;
    # walking_state is already 'WalkingInitPose', so just run the trajectory.
    last_goals = None
    while not wm.init_pose_finish():
        result = wm.run({})
        last_goals = result[2]   # dict: {joint_name: JointPosition}
    if last_goals is None:
        return {}
    return {k: v.goal_position for k, v in last_goals.items()}


# ---------------------------------------------------------------------------
# Hardware helpers
# ---------------------------------------------------------------------------
def move_to_standing(board, standing_angles, servo_cfg, duration_ms=1000):
    """Send all 14 walking joints to their standing pose in one batch command."""
    cmds = []
    for jname, angle in standing_angles.items():
        sid = JOINT_ID[jname]
        pulse = angle2pulse(sid, angle, servo_cfg)
        cmds.append([sid, pulse])
        print(f"  {jname:<14} servo {sid:2d}  angle={math.degrees(angle):+6.1f}°  pulse={pulse}")
    board.bus_servo_set_position(duration_ms / 1000.0, cmds)


def nudge_joint(board, joint_name, servo_id, base_angle_rad, nudge_deg, servo_cfg):
    """Nudge a single joint by nudge_deg degrees, then return to base angle."""
    nudge_rad = math.radians(nudge_deg)
    base_pulse   = angle2pulse(servo_id, base_angle_rad, servo_cfg)
    nudged_pulse = angle2pulse(servo_id, base_angle_rad + nudge_rad, servo_cfg)

    print(f"  → moving to {base_angle_rad + nudge_rad:+.3f} rad  (pulse {nudged_pulse})")
    board.bus_servo_set_position(0.4, [[servo_id, nudged_pulse]])
    time.sleep(1.5)

    print(f"  → returning to {base_angle_rad:+.3f} rad  (pulse {base_pulse})")
    board.bus_servo_set_position(0.3, [[servo_id, base_pulse]])
    time.sleep(0.6)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Test servo mapping using walking_module.so')
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument('--joint',  help='Joint name to test (e.g. r_hip_yaw)')
    grp.add_argument('--servo',  type=int, help='Servo ID to test (1–14)')
    parser.add_argument('--nudge', type=float, default=20.0,
                        help='Nudge amount in degrees (default: 20)')
    args = parser.parse_args()

    # ---- paths ----
    walking_param_path = (
        '/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')
    servo_cfg_path = (
        '/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/servo_controller.yaml')

    # ---- build WalkingModule ----
    print("Loading walking_module.so + kinematics.so …")
    ik = LegIK()
    with open(walking_param_path) as f:
        walking_param = yaml.safe_load(f)
    init_position = [0.0] * 14
    wm = WalkingModule(ik, walking_param, JOINT_INDEX, init_position,
                       walking_param['trajectory_step_s'])
    period_s = walking_param['period_time'] / 1000.0
    print(f"  WalkingModule OK  (period={period_s:.3f}s, "
          f"trajectory_step={walking_param['trajectory_step_s']}s)")

    # ---- compute standing pose ----
    print("\nComputing standing pose …")
    servo_cfg = load_servo_cfg(servo_cfg_path)
    standing_angles = compute_standing_pose(wm)
    print(f"  Done — {len(standing_angles)} joints")

    # ---- init hardware ----
    print("\nConnecting to servos (ainex_sdk.Board) …")
    board = Board()
    board.enable_reception()
    time.sleep(0.3)

    # ---- move to standing pose ----
    print("\nMoving all joints to standing pose (1 s ramp):")
    move_to_standing(board, standing_angles, servo_cfg, duration_ms=1000)
    print("  (waiting 1.5 s …)")
    time.sleep(1.5)

    # ---- build target list ----
    if args.joint:
        if args.joint not in JOINT_ID:
            print(f"ERROR: unknown joint '{args.joint}'. Valid joints:")
            for j in SWEEP_ORDER:
                print(f"  {j}")
            sys.exit(1)
        targets = [args.joint]
    elif args.servo:
        if args.servo not in SERVO_JOINT:
            print(f"ERROR: servo ID {args.servo} not in walking joints (1–14).")
            sys.exit(1)
        targets = [SERVO_JOINT[args.servo]]
    else:
        targets = SWEEP_ORDER

    # ---- nudge loop ----
    print(f"\n{'='*60}")
    print(f"Testing {len(targets)} joint(s) — nudge = {args.nudge:+.1f}°")
    print(f"Watch which servo moves, then press Enter to continue (q = quit).")
    print(f"{'='*60}\n")

    for i, jname in enumerate(targets):
        sid   = JOINT_ID[jname]
        base  = standing_angles.get(jname, 0.0)
        index = JOINT_INDEX[jname]

        print(f"[{i+1}/{len(targets)}]  Joint: {jname:<14}  "
              f"servo ID: {sid:2d}  "
              f"array index: {index:2d}  "
              f"standing angle: {math.degrees(base):+.1f}°")
        nudge_joint(board, jname, sid, base, args.nudge, servo_cfg)

        if len(targets) > 1:
            try:
                resp = input("  Press Enter for next joint, 'q' to quit: ").strip().lower()
                if resp == 'q':
                    break
            except (EOFError, KeyboardInterrupt):
                break

    # ---- return to standing pose ----
    print("\nReturning all joints to standing pose …")
    cmds = [[JOINT_ID[jn], angle2pulse(JOINT_ID[jn], a, servo_cfg)]
            for jn, a in standing_angles.items()]
    board.bus_servo_set_position(0.5, cmds)
    time.sleep(1.0)
    print("Done.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted.')
