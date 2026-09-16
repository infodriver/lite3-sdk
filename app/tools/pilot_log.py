#!/usr/bin/env python3
"""Pilot log collector for the DeepRobotics Jueying Lite3.

Records what is available over the high-level UDP interface (no robot-side
software needed to listen):

  * 0x0901 robot state  - rpy, rpy rates, acc, pos, body/world velocity,
                          battery, gait/motion/error flags, ultrasound
  * 0x0902 joint state  - 12 joint angles (LF/RF/LB/RB x3) if the firmware streams it
  * 0x0905 handle state - remote sticks + goal velocities if streamed
  * every command we send (timestamp + values), when --maneuver is used

Output: CSV (machine readable) + JSON (metadata + rows) + raw hex capture.

The suggested test protocol can be automated with --maneuver:
    hold neutral 5-10 s, one slow move for 2-3 s, hold 2 s, return, hold 5 s
Here that is a low-speed straight move (our channel is whole-body velocity, not
per-joint torque), so the log contains commanded vs measured whole-body motion.

NOTE: joint-level commanded/measured position+velocity, torque/current and the
controller state at 1 kHz require the manufacturer's low-level tooling
(Lite3_MotionSDK or the ROS2 bridge) running with robot-side config
(~/jy_exe/conf/network.toml pointing at this host) - see --print-checklist.

Usage:
  python3 tools/pilot_log.py --seconds 30 --label neutral_hold
  python3 tools/pilot_log.py --maneuver --label slow_move_1 --outdir ~/lite3/logs
"""
import argparse
import csv
import json
import os
import socket
import struct
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # allow importing lite3sdk from repo root
try:
    from lite3sdk import protocol as P   # noqa: E402
except Exception:                        # pragma: no cover
    P = None

META_DEFAULTS = {
    "robot_model": "DeepRobotics Jueying Lite3",
    "controller": "Lite3 motion host (high-level UDP interface)",
    "firmware": "<unknown - read Deeprcs version on the motion host / name.toml>",
    "sdk": "lite3sdk (high-level UDP) / Lite3_MotionSDK (low-level, not used here)",
    "logging_frequency_hz": "as streamed by robot (0x0901); commands logged per send",
    "units": "SI: m, m/s, rad, rad/s, deg for angles in exports; battery 0-1",
    "joint_limits": "<from manufacturer: low-level SDK / manual>",
    "speed_limits_cmd": "vx<=0.5 m/s, vy<=0.4 m/s, wz<=0.8 rad/s (console clamps)",
    "accel_limits": "<not exposed on the high-level interface>",
    "payload": "<operator to state>",
    "command_interface": "UDP 43893 (velocity frames 0x140/0x145/0x141 + actions)",
}

CHECKLIST = """
To capture the FULL dataset the partner asked for (per-joint commanded vs
measured pos/vel, torque/current, controller state, warnings at 1 kHz):

1. Get access to the robot's motion host (ssh ysc@192.168.2.1 - password needed).
2. Point telemetry at this computer: edit ~/jy_exe/conf/network.toml
       ip = '<this computer IP>'
   then restart the motion program: cd ~/jy_exe/scripts && sudo ./stop.sh && sudo ./restart.sh
3. Run the manufacturer's low-level tooling from this computer:
   - DeepRoboticsLab/Lite3_MotionSDK (C++): its demo already receives RobotData
     (joint pos/vel, torque/current, controller state); add a 1 kHz CSV writer,
   - or Lite3_ROS (ROS2 bridge): ros2 bag record /joint_states /leg_odom2 /imu/data /handle_state
4. Re-run this script simultaneously for the high-level command/state timeline.
"""


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + \
           ".%03d" % int((time.time() % 1) * 1000)


def post_json(path, body, base="http://localhost:8123"):
    try:
        req = urllib.request.Request(
            base + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read() or b"{}")
    except Exception as exc:
        return {"ok": False, "msg": str(exc)}


def post_cmd(body, api="http://localhost:8123/api/cmd"):
    return post_json("/api/cmd", body, base=api.rsplit("/api/cmd", 1)[0] or "http://localhost:8123")


def main():
    ap = argparse.ArgumentParser(description="Lite3 pilot log collector")
    ap.add_argument("--robot-ip", default="192.168.2.1")
    ap.add_argument("--state-port", type=int, default=43897)
    ap.add_argument("--outdir", default=os.path.expanduser("~/lite3/logs"))
    ap.add_argument("--label", default="pilot")
    ap.add_argument("--seconds", type=float, default=30.0, help="recording length")
    ap.add_argument("--maneuver", action="store_true",
                    help="run: hold 8s, slow forward 2.5s, hold 2s, return 2.5s, hold 5s")
    ap.add_argument("--speed", type=float, default=0.15, help="maneuver speed m/s (slow)")
    ap.add_argument("--print-checklist", action="store_true")
    ap.add_argument("--no-reconnect", action="store_true",
                    help="do not reconnect the console after capture")
    ap.add_argument("--meta", action="append", default=[],
                    help="extra metadata key=value (e.g. payload=2kg)")
    args = ap.parse_args()

    if args.print_checklist:
        print(CHECKLIST)
        return

    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(args.outdir, "%s-%s" % (args.label, stamp))

    meta = dict(META_DEFAULTS)
    meta.update({"label": args.label, "started": now_iso(),
                 "robot_ip": args.robot_ip, "state_port": args.state_port,
                 "maneuver": args.maneuver, "maneuver_speed_mps": args.speed})
    for kv in args.meta:
        if "=" in kv:
            k, v = kv.split("=", 1)
            meta[k.strip()] = v.strip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.state_port))
    except OSError as exc:
        if exc.errno == 48:      # address in use -> usually the console itself
            print("telemetry port busy - asking the console to disconnect...")
            print("   ", post_json("/api/disconnect", {}))
            time.sleep(1.5)
            sock.bind(("0.0.0.0", args.state_port))
            meta_extra = {"console_disconnected_for_capture": True}
        else:
            raise
    else:
        meta_extra = {}
    meta.update(meta_extra)
    sock.settimeout(0.5)

    rows, raw = [], []
    t0 = time.monotonic()
    next_cmd = None
    plan = None
    if args.maneuver:
        s = args.speed
        plan = [(0.0, 0.0, "hold neutral"), (8.0, s, "slow forward"),
                (10.5, 0.0, "hold"), (12.5, -s, "return (reverse)"),
                (15.0, 0.0, "hold"), (20.0, None, "stop")]
        print("maneuver plan:", plan)

    print("recording for %.0fs -> %s.{csv,json,hex}" % (args.seconds, base))
    while time.monotonic() - t0 < args.seconds:
        el = time.monotonic() - t0
        # maneuver timeline
        if plan is not None:
            while plan and el >= plan[0][0]:
                _t, v, what = plan.pop(0)
                if v is None:
                    res = post_cmd({"type": "stop"})
                else:
                    res = post_cmd({"type": "velocity", "vx": v, "vy": 0, "wz": 0})
                rows.append({"t": now_iso(), "elapsed_s": round(el, 3),
                             "kind": "command", "detail": what,
                             "cmd_vx": v if v is not None else 0, "cmd_vy": 0,
                             "cmd_wz": 0, "result": json.dumps(res)[:120]})
                print("  [%6.2fs] %-18s %s" % (el, what, res))
        # telemetry
        try:
            data, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        raw.append(data.hex())
        if len(data) < 12:
            continue
        code = struct.unpack_from("<i", data, 0)[0]
        entry = {"t": now_iso(), "elapsed_s": round(el, 3), "kind": "frame",
                 "code": "0x%04X" % code, "size": len(data)}
        if P is not None:
            st = P.parse_state(data) if code == 0x0901 else None
            if st:
                for k in ("basic_state", "gait_state", "motion_state", "error_state",
                          "battery", "charging", "rpy", "rpy_vel", "vel_body",
                          "pos", "ultrasound", "touch_stair", "task_state"):
                    entry[k] = st.get(k)
            j = P.parse_joint_state(data) if code == 0x0902 else None
            if j:
                entry["joints"] = j
            h = P.parse_handle_state(data) if code == 0x0905 else None
            if h:
                entry["handle"] = h
        rows.append(entry)

    # exports
    if rows:
        keys = sorted({k for r in rows for k in r})
        with open(base + ".csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: (json.dumps(v) if isinstance(v, (list, dict)) else v)
                            for k, v in r.items()})
    with open(base + ".json", "w") as fh:
        json.dump({"metadata": meta, "rows": rows}, fh, indent=2)
    with open(base + ".hex", "w") as fh:
        fh.write("\n".join(raw))
    print("done: %d rows, %d raw frames" % (len(rows), len(raw)))
    print("  %s.csv | %s.json | %s.hex" % (base, base, base))
    if not raw:
        print("NOTE: no telemetry received - the robot must stream to this computer")
        print("      (see --print-checklist)")
    if meta.get("console_disconnected_for_capture") and not args.no_reconnect:
        print("reconnecting console:", post_json("/api/connect", {"robot_ip": args.robot_ip}))


if __name__ == "__main__":
    main()
