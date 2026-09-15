"""Lite3 UDP wire protocol: frame builder + telemetry parser.

Wire format (community-verified against DeepRoboticsLab/Lite3_MotionSDK and
EzioPeter/Lite3_UDP, and used in production by the Lite3 Pilot web console):

Frame = 12-byte little-endian header + optional payload:
    code  (I)  command/action id (e.g. 0x21010202 = stand)
    value (I)  second field: payload length (e.g. 8 for a double) or int value
    type  (I)  frame kind (0 = simple/action, 1 = typed payload frame)

Velocity frames carry an 8-byte little-endian double after the header and set
the value field to 8. Each axis is a separate frame: x=0x140 (forward/back),
y=0x145 (lateral strafe), z=0x141 (yaw/turn).

Telemetry: robot streams frames with code 0x0901 to UDP 43897. Header is 12 B
("<iii"), followed by a 200-byte payload in field order:
    "<ii"          basic_state, gait_state
    "3d" x6        rpy, rpy_vel, acc, pos_world, vel_world, vel_body
    "IB3xI"        touch_stair, charging, (3 pad), error_state
    "id"           motion_state, battery (double, fraction 0..1)
    "iBB2x2d"      task_state, need_move, zero_pos, (2 pad), ultrasound(2 x double)
(Identical to the layout consumed by the Lite3 Pilot console, which has been
validated against live robot telemetry.)
"""

import struct

# Ports
CMD_PORT = 43893    # robot listens here for command frames
STATE_PORT = 43897  # robot streams 0x0901 telemetry here (set in network.toml)
CAMERA_PORT = 43899 # robot's app-service port (camera / AI service commands)

# Camera / AI service control (AppSimpleCMD to CAMERA_PORT)
FRAME_CAMERA_ON = 0x21012109    # value 0x40 = start the robot's camera services
FRAME_CAMERA_OFF = 0x21012109   # value 0x00 = stop them
FRAME_CAMERA_QUERY = 0x2101210D # query state: replies 0x11 active / 0x10 inactive
CAMERA_VALUE_ON = 0x40
CAMERA_RTSP_PATH = "/test"      # RTSP stream name on port 8554
CAMERA_RTSP_PORT = 8554

# Simple/action frame codes (type 0, value 0 unless noted)
FRAME_AUTO_MODE = 0x21010C03   # navigation/auto mode (velocity-follow)
FRAME_MANUAL_MODE = 0x21010C02 # remote-control mode
# 0x21010202 is a STAND<->SIT TOGGLE (one command, not separate poses - the
# Lite3 motion host switches posture on each frame; community tables and live
# tests agree). Stand/sit callers must track the current posture.
FRAME_STAND_SIT = 0x21010202
FRAME_STAND = 0x21010202       # same toggle - see FRAME_STAND_SIT
FRAME_SIT = 0x21010202         # same toggle - see FRAME_STAND_SIT
FRAME_HELLO = 0x21010506       # official 'hello' pose action - plays from SITTING
FRAME_STOP = 0x21010C0B        # action stop - send with value 0 AND 1
FRAME_HEARTBEAT = 0x21040001   # keep-alive; motion host expects >= 2 Hz

# Velocity frame codes (type 1, payload = one double, value field = 8)
VEL_X = 0x140   # forward / back  (+ = forward)
VEL_Y = 0x145   # lateral strafe   (+ = left/right per robot convention)
VEL_Z = 0x141   # yaw / turn       (+ = one direction; flip sign if reversed)

# Telemetry
STATE_STRUCT = struct.Struct("<ii" + "3d" * 6 + "IB3xIid" + "iBB2x2d")  # 200 B
STATE_HEADER = 12


def build_frame(code, typ=0, value=0, payload=None):
    """Build a 12-byte header + optional payload frame (see module docstring).

    Pass `payload_double` handled by :func:`velocity_frame`; for a raw bytes
    payload set `payload` and `value` to its length.
    """
    head = struct.pack(
        "<III", int(code) & 0xFFFFFFFF, int(value) & 0xFFFFFFFF, int(typ) & 0xFFFFFFFF
    )
    return head + (payload or b"")


def velocity_frame(code, velocity):
    """Velocity frame: header (code, value=8, type=1) + one f64 little-endian."""
    return build_frame(code, typ=1, value=8, payload=struct.pack("<d", float(velocity)))


def parse_state(buf):
    """Decode one telemetry packet. Returns a dict or None.

    Accepts exactly the stock-firmware layout (12 B header + 200 B payload,
    total 212 B). Newer RL-firmware frames (216/220 B, extra robot_policy_state)
    are rejected here rather than misread - callers can detect that case by
    length and log a firmware-update hint."""
    if len(buf) != STATE_HEADER + STATE_STRUCT.size:
        return None
    code, _size, _cons = struct.unpack_from("<iii", buf, 0)
    if code != 0x0901:
        return None
    f = STATE_STRUCT.unpack_from(buf, STATE_HEADER)
    i = 0

    def take(n):
        nonlocal i
        out = f[i:i + n]
        i += n
        return out

    basic_state, gait_state = take(2)
    rpy = take(3)
    rpy_vel = take(3)
    acc = take(3)
    pos_world = take(3)
    vel_world = take(3)
    vel_body = take(3)
    (touch_stair,) = take(1)
    (charging,) = take(1)
    (error_state,) = take(1)
    (motion_state,) = take(1)
    (battery,) = take(1)
    (task_state,) = take(1)
    (need_move,) = take(1)
    (zero_pos,) = take(1)
    ultrasound = take(2)
    return {
        "code": "0x%04X" % code,
        "basic_state": basic_state,
        "gait_state": gait_state,
        "rpy": [round(x, 3) for x in rpy],           # roll/pitch/yaw rad
        "rpy_vel": [round(x, 3) for x in rpy_vel],
        "acc": [round(x, 3) for x in acc],
        "pos": [round(x, 2) for x in pos_world],      # world xyz; z = body height
        "vel_world": [round(x, 3) for x in vel_world],
        "vel_body": [round(x, 3) for x in vel_body],
        "touch_stair": touch_stair,
        "charging": bool(charging),
        "error_state": error_state,
        "motion_state": motion_state,
        "battery": round(battery, 4),                 # fraction 0..1
        "task_state": task_state,
        "need_move": bool(need_move),
        "zero_position": bool(zero_pos),
        "ultrasound": [round(x, 2) for x in ultrasound],
    }


def parse_joint_state(buf):
    """Decode a 0x0902 joint-state stream: 12 x f64 (LF RF LB RB x 3 joints).

    Returns the raw wire values; the official Lite3_ROS node negates all 12
    when publishing joint_states (Jetson2Motion.cpp), so flip signs here if
    conventional angles are ever needed for display."""
    if len(buf) < STATE_HEADER + 96:
        return None
    return [round(x, 3) for x in struct.unpack_from("<12d", buf, STATE_HEADER)]


def parse_handle_state(buf):
    """Decode a 0x0905 handle/gamepad stream: 6 x f64 =
    left_axis_forward, left_axis_side, right_axis_yaw,
    goal_vel_forward, goal_vel_side, goal_vel_yaw."""
    if len(buf) < STATE_HEADER + 48:
        return None
    v = struct.unpack_from("<6d", buf, STATE_HEADER)
    return {
        "left_axis_forward": round(v[0], 3),
        "left_axis_side": round(v[1], 3),
        "right_axis_yaw": round(v[2], 3),
        "goal_vel_forward": round(v[3], 3),
        "goal_vel_side": round(v[4], 3),
        "goal_vel_yaw": round(v[5], 3),
    }


class Frame:
    """One named protocol constant (convenience for CLI/raw use)."""

    def __init__(self, code, typ=0, value=0):
        self.code = int(code)
        self.typ = int(typ)
        self.value = int(value)

    def bytes(self):
        return build_frame(self.code, typ=self.typ, value=self.value)

    def __repr__(self):
        return "Frame(0x%08X type=%d value=%d)" % (self.code, self.typ, self.value)


# ---------------------------------------------------------------- actions
# Full action catalog (same set as the Lite3 Pilot web console's buttons).
#   posture: required start pose ('stand'/'sit'/None)
#   force:   apply the posture step even when the tracked state is unknown
#   once:    single-shot switch (modes/gaits) instead of a 3x 1 Hz replay
ACTIONS = {
    "stand_up":     {"code": FRAME_STAND_SIT, "posture": None,    "toggle": True,
                     "label": "Stand (posture toggle)"},
    "sit_down":     {"code": FRAME_STAND_SIT, "posture": None,    "toggle": True,
                     "label": "Sit (posture toggle)"},
    "hello":        {"code": FRAME_HELLO,     "posture": "sit",   "force": True,
                     "label": "Hello pose"},
    "dance":        {"code": 0x2101030C,      "posture": "stand",
                     "label": "Dance / moonwalk"},
    "twist":        {"code": 0x21010204,      "posture": "stand",
                     "label": "Twist"},
    "twist_jump":   {"code": 0x2101020D,      "posture": "stand",
                     "label": "Twist jump"},
    "turn_over":    {"code": 0x21010205,      "posture": "sit",   "force": True,
                     "label": "Turn over (rolls onto back)"},
    "backflip":     {"code": 0x21010502,      "posture": "sit",   "force": True,
                     "label": "Backflip"},
    "long_jump":    {"code": 0x2101050B,      "posture": "sit",   "force": True,
                     "label": "Long jump"},
    "recover_left":  {"code": 0x21010205,     "posture": None,
                      "label": "Recover from back (left roll)"},
    "recover_right": {"code": 0x21010205,     "posture": None,
                      "label": "Recover from back (right roll)"},
    "mode_manual":  {"code": 0x21010C02,      "once": True,
                     "label": "Mode: manual/remote"},
    "mode_move":    {"code": 0x21010D06,      "once": True,
                     "label": "Mode: move"},
    "gait_slow":    {"code": 0x21010300,      "once": True, "label": "Gait: slow"},
    "gait_medium":  {"code": 0x21010307,      "once": True, "label": "Gait: medium"},
    "gait_fast":    {"code": 0x21010303,      "once": True, "label": "Gait: fast"},
    "gait_crawl":   {"code": 0x21010406,      "once": True, "label": "Gait: crawl"},
}
