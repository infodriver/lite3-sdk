"""Command-line interface for the Lite3 SDK.

Run from the source tree (no install needed):

    python3 -m lite3sdk.cli status
    python3 -m lite3sdk.cli stand
    python3 -m lite3sdk.cli hello
    python3 -m lite3sdk.cli drive --vx 0.4 --wz 0.3 --seconds 2
    python3 -m lite3sdk.cli stop
    python3 -m lite3sdk.cli estop --yes
    python3 -m lite3sdk.cli reset-estop

The robot must be reachable (join its Wi-Fi / be on its LAN); the motion
host is 192.168.2.1 by default (--ip to override).
"""

import argparse
import json
import sys
import time

from . import __version__
from .robot import DEFAULT_IP, Lite3, MotionError

_EPILOG = "Safety: keep the dog leashed / on its stand and clear of people."


def _conn(args, listen=True):
    state_port = args.state_port if listen else 0
    dog = Lite3(robot_ip=args.ip, cmd_port=args.cmd_port, state_port=state_port,
                max_vx=args.max_vx, max_wz=args.max_wz,
                allow_motion_without_telemetry=not args.require_telemetry)
    dog.connect()
    return dog


def cmd_status(dog, args):
    if args.wait and dog.state_port:
        dog.wait_telemetry(timeout=args.wait)
    print(json.dumps(dog.state(), indent=2, default=str))


def cmd_stand(dog, args):
    dog.stand()
    print("stand: sent (burst %d ms)" % (args.ms or 600))


def cmd_sit(dog, args):
    dog.sit()
    print("sit: sent (burst %d ms)" % (args.ms or 600))


def cmd_hello(dog, args):
    print("hello: stand + wiggle… (Ctrl-C aborts; estop latches)")
    dog.hello()


def cmd_drive(dog, args):
    seconds = args.seconds if args.seconds is not None else 1.0
    print("drive: vx=%.2f vy=%.2f wz=%.2f for %.1f s (max vx %.1f, wz %.1f)"
          % (args.vx, args.vy, args.wz, seconds, args.max_vx, args.max_wz))
    dog.drive(vx=args.vx, vy=args.vy, wz=args.wz)
    dog.wait(seconds)
    dog.stop()
    print("drive: done (stop frames sent)")


def cmd_stop(dog, args):
    dog.stop()
    print("stop: sent")


def cmd_estop(dog, args):
    if not args.yes:
        print("E-stop halts motion until reset. Re-run with --yes to latch.")
        sys.exit(2)
    dog.estop()
    print("E-STOP LATCHED (software). Physical E-stop on the robot always wins.")


def cmd_reset_estop(dog, args):
    dog.reset_estop()
    print("E-stop reset - motion re-armed")


def cmd_raw(dog, args):
    frame = dog.send_raw(int(args.code, 0), typ=args.type, value=args.value,
                         repeats=args.repeats, interval=args.interval)
    print("raw frame -> %s (%d B)" % (frame.hex(), len(frame)))


def cmd_camera(dog, args):
    if args.action == "on":
        print(dog.camera_on())
        print("stream:", dog.camera_url)
    elif args.action == "off":
        print(dog.camera_off())
    else:
        print(dog.camera_state())
        print("stream:", dog.camera_url)


def cmd_action(dog, args):
    ok = dog.action(args.name)
    print("action '%s' -> %s" % (args.name, "done" if ok is not False else "cancelled"))


def cmd_actions(dog, args):
    for name, spec in sorted(dog.list_actions().items()):
        print("%-14s 0x%08X  %s" % (name, spec["code"], spec.get("label", "")))


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="lite3sdk", description="DeepRobotics Jueying Lite3 control SDK CLI",
        epilog=_EPILOG)
    ap.add_argument("--ip", default=DEFAULT_IP, help="robot motion host "
                    "(default %s)" % DEFAULT_IP)
    ap.add_argument("--cmd-port", type=int, default=43893)
    ap.add_argument("--state-port", type=int, default=43897)
    ap.add_argument("--max-vx", type=float, default=0.5, help="speed clamp m/s")
    ap.add_argument("--max-wz", type=float, default=0.8, help="yaw clamp rad/s")
    ap.add_argument("--require-telemetry", action="store_true",
                    help="refuse motion until 0x0901 telemetry is flowing")
    ap.add_argument("--version", action="version", version="lite3sdk " + __version__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("status", help="show telemetry + connection state")
    p.add_argument("--wait", type=float, default=3.0,
                   help="seconds to wait for telemetry before printing (0 = skip)")
    p.set_defaults(fn=cmd_status)

    p = sub.add_parser("stand", help="stand up")
    p.add_argument("--ms", type=int, default=None)
    p.set_defaults(fn=cmd_stand)

    p = sub.add_parser("sit", help="sit / lie down")
    p.add_argument("--ms", type=int, default=None)
    p.set_defaults(fn=cmd_sit)

    p = sub.add_parser("hello", help="official hello pose (plays from sitting)")
    p.set_defaults(fn=cmd_hello)

    p = sub.add_parser("drive", help="send a velocity for N seconds then stop")
    p.add_argument("--vx", type=float, default=0.0, help="forward m/s")
    p.add_argument("--vy", type=float, default=0.0, help="lateral m/s")
    p.add_argument("--wz", type=float, default=0.0, help="yaw rad/s")
    p.add_argument("--seconds", type=float, default=None,
                   help="duration (default 1.0 s)")
    p.set_defaults(fn=cmd_drive)

    p = sub.add_parser("stop", help="brake (official stop frames)")
    p.set_defaults(fn=cmd_stop)

    p = sub.add_parser("estop", help="latch software E-stop")
    p.add_argument("--yes", action="store_true", help="confirm")
    p.set_defaults(fn=cmd_estop)

    p = sub.add_parser("reset-estop", help="unlatch software E-stop")
    p.set_defaults(fn=cmd_reset_estop)

    p = sub.add_parser("raw", help="send one raw frame")
    p.add_argument("code", help="frame code, hex like 0x21010202 or decimal")
    p.add_argument("--type", type=int, default=0)
    p.add_argument("--value", type=int, default=0)
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--interval", type=float, default=0.0)
    p.set_defaults(fn=cmd_raw)

    p = sub.add_parser("camera", help="camera services: on | off | state")
    p.add_argument("action", choices=["on", "off", "state"])
    p.set_defaults(fn=cmd_camera)

    p = sub.add_parser("actions", help="list every catalog action (same as the app)")
    p.set_defaults(fn=cmd_actions)

    p = sub.add_parser("action", help="play any catalog action by name")
    p.add_argument("name")
    p.set_defaults(fn=cmd_action)

    # one shortcut subcommand per action (dance, backflip, ...)
    from .protocol import ACTIONS as _ACTIONS
    for _name, _spec in sorted(_ACTIONS.items()):
        if _name in ("stand_up", "sit_down", "hello"):
            continue                      # already have stand/sit/hello subcommands
        sp = sub.add_parser(_name, help="%s (0x%08X)" % (_spec.get("label", _name), _spec["code"]))
        sp.set_defaults(fn=cmd_action, name=_name)

    args = ap.parse_args(argv)
    listen = args.cmd == "status"  # only status needs the telemetry socket
    try:
        dog = _conn(args, listen=listen)
    except OSError as exc:
        print("connect failed: %s (is port %s free / robot on this LAN?)"
              % (exc, args.state_port), file=sys.stderr)
        sys.exit(1)
    try:
        args.fn(dog, args)
        # brief tail so the last burst actually leaves this host
        time.sleep(0.2)
    except MotionError as exc:
        print("refused: %s" % exc, file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        dog.close()


if __name__ == "__main__":
    main()
