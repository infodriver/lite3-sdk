"""lite3sdk - zero-dependency Python control SDK for the DeepRobotics Jueying Lite3.

Talks to the robot directly over UDP (no ROS, no robot-side changes), using the
same verified wire protocol as the Lite3 Pilot web console:

  commands  -> robot UDP 43893   (auto-mode, velocity, motion/action frames)
  telemetry <- robot UDP 43897   (frame 0x0901: battery / IMU / gait / errors)

Typical use:

    import lite3sdk

    dog = lite3sdk.Lite3("192.168.2.1")   # robot motion host on its Wi-Fi
    with dog:
        dog.stand()
        dog.drive(vx=0.4, wz=0.2)          # forward + turn
        dog.wait(1.5)                      # keep sending for 1.5 s
        dog.stop()
        print(dog.battery_pct, dog.telemetry)
"""

from .protocol import (
    Frame,
    CMD_PORT,
    STATE_PORT,
    FRAME_AUTO_MODE,
    FRAME_STAND,
    FRAME_SIT,
    FRAME_STAND_SIT,
    FRAME_HELLO,
    FRAME_STOP,
    FRAME_HEARTBEAT,
    VEL_X,
    VEL_Y,
    VEL_Z,
    build_frame,
    parse_state,
    parse_joint_state,
    parse_handle_state,
    ACTIONS,
    ACTION_GROUPS,
    CAMERA_PORT,
)
from .robot import Lite3, MotionError

__version__ = "0.1.0"

__all__ = [
    "Lite3",
    "MotionError",
    "Frame",
    "CMD_PORT",
    "STATE_PORT",
    "FRAME_AUTO_MODE",
    "FRAME_STAND",
    "FRAME_SIT",
    "FRAME_STAND_SIT",
    "FRAME_HELLO",
    "FRAME_STOP",
    "FRAME_HEARTBEAT",
    "VEL_X",
    "VEL_Y",
    "VEL_Z",
    "build_frame",
    "parse_state",
    "parse_joint_state",
    "parse_handle_state",
    "ACTIONS",
    "ACTION_GROUPS",
    "CAMERA_PORT",
    "__version__",
]
