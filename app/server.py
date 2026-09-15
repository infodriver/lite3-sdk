#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lite3 Web Pilot — web console for the DeepRobotics Jueying Lite3 robot dog.

Talks to the robot over UDP (motion host at 192.168.2.1, e.g.):
  * commands   -> robot UDP :43893   (auto-mode, velocity, motion frames)
  * telemetry  <- robot UDP :43897   (frame code 0x0901: battery/IMU/gait/...)

Camera: the Lite3 has no public RTSP/MJPEG endpoint, so the camera source is
pluggable (RTSP / MJPEG / HTTP-JPEG snapshot) and is proxied to the browser as
MJPEG via ffmpeg (or a plain HTTP poller). See README.md.

Protocol reference:
  * official SDK: DeepRoboticsLab/Lite3_MotionSDK  (motion host 192.168.2.1,
    UDP 43893/43897, robot streams to the IP set in ~/jy_exe/conf/network.toml)
  * community wire reimplementation: EzioPeter/Lite3_UDP (auto-mode 0x21010C03,
    velocity 0x141+double, rotate 0x21010135, state frame 0x0901 struct)

Python 3 stdlib only. Safety: never rely on this for collision avoidance;
keep the robot leashed/on a stand and use the physical E-stop when near it.
"""

import argparse
import datetime
import faulthandler
import json
import mimetypes
import os
import shutil
import socket
import struct
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(ROOT, "static")
CONFIG_PATH = os.path.join(ROOT, "config.json")

DEFAULTS = {
    "robot_ip": "192.168.2.1",
    "cmd_port": 43893,          # robot listens here for commands (UDP)
    "state_port": 43897,        # robot streams telemetry here (UDP)
    "state_bind": "0.0.0.0",
    "http_port": 8123,
    "limits": {
        "max_vx": 0.5,          # m/s  (Lite3 cruise speeds are modest)
        "max_vy": 0.4,          # m/s lateral (slide left/right)
        "max_wz": 0.8,          # rad/s (only used if turn frame configured)
        "rate_hz": 50,          # command frame rate while driving
        "tail_ms": 300,         # keep sending zero velocity this long after release
        "action_ms": 600,       # how long preset/raw action frames are repeated
    },
    # Frames understood by the Lite3 motion host (community-verified).
    # RETROID-style joystick frames (DeepRoboticsLab/gamepad): the robot accepts
    # the same packets the official app sends -> udp :12121, 42-byte frames,
    # crc16 = sum of the 32 payload bytes.
    "joystick": {
        "port": 12121,
        "id": 1,               # kRetroid
        "seq": 0,
        "yaw_sign": -1,        # right_axis_x is negated on the robot
    },
    # move the dog (community-verified 0x141 = forward/back + auto-mode).
    # velocity frames - OFFICIAL DeepRobotics transfer mapping (confirmed live:
    # 0x140 = forward, 0x145 = lateral, 0x141 = yaw/turn)
    "velocity": {"code": "0x140", "type": 1, "payload": "double", "sign": 1},
    "velocity_lateral": {"code": "0x145", "type": 1, "payload": "double", "sign": 1},
    "velocity_yaw": {"code": "0x141", "type": 1, "payload": "double", "sign": 1,
                      "note": "flip sign to 1/-1 if turning direction is reversed"},
    "velocity_codes": {"x": "0x140", "y": "0x145", "z": "0x141"},
    "velocity_type": 1,
    "yaw_negate": True,
    "auto_mode": {"code": "0x21010C03", "type": 0, "value": 0},
    # Discrete commands (MotionSimpleCMD 12-byte frames, community-verified
    # against the official Lite3 protocol; comzeon/robot-deeprobotics-lite3).
    "actions": {
        "stand_up": {
            "enabled": True,
            "note": "stand up (0x21010202)",
            "frame": {"code": "0x21010202", "type": 0, "value": 0},
        },
        "sit_down": {
            "enabled": True,
            "note": "sit / lie down (0x21010203)",
            "frame": {"code": "0x21010203", "type": 0, "value": 0},
        },
    },
    "heartbeat": {"code": "0x21040001", "type": 0, "value": 0,
                    "interval_ms": 300},
    "stop_action": {"code": "0x21010C0B", "type": 0},
    "allow_motion_without_telemetry": False,
    "wifi": {
        "enabled": True,
        "robot_ssid": "YSC-JYML-XXXXX",
        "robot_password": "REPLACE_ME",
        "robot_subnet": "192.168.2.",
        "home_ssid": "",
        "retries": 3,
    },
    # how the app switches the robot's camera driver on/off
    # (DeepRoboticsLab transfer: AppSimpleCMD 0x21012109 -> perception host udp 43899;
    #  0x40 = start camera/AI services, 0x00 = stop)
    "robot_camera": {
        "port": 43899,
        "on": {"code": "0x21012109", "type": 0, "value": 0x40},
        "off": {"code": "0x21012109", "type": 0, "value": 0x00},
    },
    "camera": {
        "enabled": False,
        "kind": "auto",          # auto | ffmpeg | http_jpeg
        "url": "",               # rtsp://... | http://.../stream.mjpg | http://.../snap.jpg
        "fps": 8,
        "scale": "640:-1",       # ffmpeg -vf scale - lower = smoother on weak links
        "quality": 7,            # ffmpeg -q:v for the mjpeg proxy (higher = smaller)
        "auto_probe": True,
    },
    # camera presets on the robot - {robot_ip} is replaced by the robot IP
    "cameras": [
        {"name": "Robot Cam (front)", "url": "rtsp://{robot_ip}:8554/test",
         "kind": "ffmpeg", "fps": 8, "scale": "640:-1"},
    ],
    "camera_active": 0,
}

STATE_STRUCT = struct.Struct("<ii" + "3d" * 6 + "IB3xIid" + "iBB2x2d")  # 200 B payload
STATE_TOTAL = 12 + STATE_STRUCT.size  # 12 B header + payload


def now_iso():
    return datetime.datetime.now().isoformat(timespec="milliseconds")


def to_u32(x):
    return struct.pack("<I", x & 0xFFFFFFFF)


WIFI_CFG = {"robot_ssid": "YSC-JYML-XXXXX",
            "robot_password": "REPLACE_ME",
            "robot_subnet": "192.168.2.",
            "home_ssid": "",
            "retries": 3}


def cfg_parse_int(v):
    if isinstance(v, str) and v.lower().startswith("0x"):
        return int(v, 16)
    return int(v)


def wifi_ctl(action):
    """action: 'join_robot' | 'join_home' | 'status'"""
    # imported lazily to keep module import side-effect free in tests
    import subprocess as sp

    def run(cmd):
        return sp.run(cmd, capture_output=True, text=True, timeout=30)

    def current_ip():
        r = run(["ipconfig", "getifaddr", "en0"])
        return (r.stdout or "").strip() or None

    def iface():
        # prefer en0 (Wi-Fi); fall back to first airport-capable device
        for dev in ("en0", "en1"):
            r = run(["networksetup", "-getairportnetwork", dev])
            if r.returncode == 0 and "not associated" not in r.stdout.lower():
                return dev
        return "en0"

    try:
        if action == "status":
            ip = current_ip()
            return {"ok": True, "ip": ip}
        dev = iface()
        if action == "join_robot":
            ssid = WIFI_CFG["robot_ssid"]
            pw = WIFI_CFG["robot_password"]
            for _ in range(int(WIFI_CFG.get("retries", 3))):
                run(["networksetup", "-setairportnetwork", dev, ssid, pw])
                time.sleep(4)
                ip = current_ip()
                if ip and ip.startswith(WIFI_CFG.get("robot_subnet", "192.168.2.")):
                    return {"ok": True, "ip": ip, "joined": ssid}
            return {"ok": False, "msg": f"could not join {ssid} (is the dog powered on?)",
                    "ip": current_ip()}
        if action == "join_home":
            ssid = WIFI_CFG.get("home_ssid")
            if ssid:
                # saved keychain password is used when none is supplied
                run(["networksetup", "-setairportnetwork", dev, ssid])
                time.sleep(6)
                ip = current_ip()
                if ip and not ip.startswith(WIFI_CFG.get("robot_subnet", "192.168.2.")):
                    return {"ok": True, "ip": ip, "joined": ssid}
            # fallback: cycle Wi-Fi so macOS auto-joins the preferred network
            run(["networksetup", "-setairportpower", dev, "off"])
            time.sleep(2)
            run(["networksetup", "-setairportpower", dev, "on"])
            time.sleep(10)
            return {"ok": True, "ip": current_ip(), "note": "auto-join"}
    except Exception as exc:
        return {"ok": False, "msg": str(exc)}
    return {"ok": False, "msg": "unknown action"}


# ---------------------------------------------------------------- drive


def ssh_robot(ip, command, timeout=12):
    """Best-effort SSH to the robot (ysc / default password) - silent.
    Returns (ok, output_tail). Used to start the robot's camera service."""
    import re as _re
    if not shutil.which("expect") or not _re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip or ""):
        return False, ""
    script = (
        '#!/usr/bin/expect -f\n'
        'set timeout %d\n'
        'spawn ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null '
        '-o ConnectTimeout=4 ysc@%s %s\n'
        'expect {\n'
        '  "*assword:*" { send "\'\r"; exp_continue }\n'
        '  "*yes/no*" { send "yes\r"; exp_continue }\n'
        '  eof\n'
        '}\n'
        'catch wait result\n'
        'exit [lindex $result 3]\n'
        % (timeout, ip, command)
    )
    import tempfile
    try:
        with tempfile.NamedTemporaryFile("w", suffix=".exp", delete=False) as fh:
            fh.write(script)
            path = fh.name
        r = subprocess.run(["expect", path], capture_output=True, text=True, timeout=timeout + 4)
        os.unlink(path)
        return r.returncode == 0, (r.stdout + r.stderr)[-300:]
    except Exception:
        return False, ""


CAMERA_DRIVERS = [
    "realsense_ros2.service",   # official Lite3 camera/vision service (Jetson2App)
    "voa_ros2.service",          # vision obstacle avoidance stack (Jetson2App)
    "realsense2_camera.service",
    "dr_camera.service",
    "rs_camera.service",
    "lite3_camera.service",
]


def robot_camera_ssh_on(ip):
    """Try to start the robot's camera drivers over SSH (systemctl) on every
    reachable robot host (motion host + perception hosts per Lite3 docs).
    Silent, best-effort."""
    import socket as _sk
    candidates = [ip]
    for extra in ("192.168.1.103", "192.168.1.120", "192.168.2.1"):
        if extra not in candidates:
            candidates.append(extra)
    cmd = "".join(f"systemctl start {u} 2>/dev/null || true; " for u in CAMERA_DRIVERS)
    cmd += "echo done"
    for host in candidates:
        try:
            s = _sk.create_connection((host, 22), timeout=0.5)
            s.close()
        except Exception:
            continue
        ok, _out = ssh_robot(host, "sh -c '" + cmd + "'")
        if ok:
            return True
    return False


def _probe_endpoint(url):
    try:
        ff = shutil.which("ffprobe") or "ffprobe"
        args = ([ff, "-v", "error", "-rtsp_transport", "tcp"]
                if url.startswith("rtsp") else [ff, "-v", "error"])
        args += ["-show_entries", "format=format_name", "-of", "csv=p=0",
                 "-timeout", "2500000", "-rw_timeout", "2500000", url]
        r = subprocess.run(args, capture_output=True, text=True, timeout=6)
        return r.returncode == 0 and r.stdout.strip()
    except Exception:
        return False


def discover_camera_endpoint(ip):
    """Fresh probe: first RTSP / HTTP-MJPEG endpoint on the robot that plays."""
    import socket as _sk
    urls = []
    for port in (8554, 554):
        for p in ("/test", "/live", "/ch0", "/ch1", "/stream", "/main",
                  "/h264", "/video", "/0", "/1", "/"):
            urls.append(f"rtsp://{ip}:{port}{p}")
    for port in (8080, 8090):
        for p in ("/?action=stream", "/stream/video.mjpeg", "/mjpeg",
                  "/?action=stream&type=mjpeg"):
            urls.append(f"http://{ip}:{port}{p}")
    for u in urls:
        try:
            from urllib.parse import urlparse
            pr = urlparse(u)
            s = _sk.create_connection((pr.hostname, pr.port or 80), timeout=0.35)
            s.close()
        except Exception:
            continue
        if _probe_endpoint(u):
            return u
    return None


CAM_BOOT_LOCK = threading.Lock()
CAM_BOOT_ACTIVE = False
CAM_BOOT_LAST = 0.0


def camera_boot(engine, camera):
    """From-scratch auto camera connection: robot reachable -> enable the robot
    camera, find its stream, open it. Quiet, retries briefly, stops cleanly."""
    global CAM_BOOT_ACTIVE, CAM_BOOT_LAST
    with CAM_BOOT_LOCK:
        if CAM_BOOT_ACTIVE:
            return                              # one discovery at a time
        if time.monotonic() - CAM_BOOT_LAST < 10:
            return                              # avoid duplicate boots
        CAM_BOOT_ACTIVE = True
    def work():
        global CAM_BOOT_ACTIVE, CAM_BOOT_LAST
        try:
            ip = str(engine.cfg.data.get("robot_ip", "192.168.2.1"))
            reachable = False
            for _ in range(6):
                try:
                    r = subprocess.run(["ping", "-c", "1", "-W", "1000", ip],
                                       capture_output=True, timeout=3)
                    if r.returncode == 0:
                        reachable = True
                        break
                except Exception:
                    pass
                time.sleep(1)
            if not reachable:
                return
            try:
                robot_camera_ssh_on(ip)   # start the robot's camera drivers (best effort)
            except Exception:
                pass
            rc = engine.cfg.data.get("robot_camera", {})
            idx = engine.cfg.data.get("camera_active", 0)
            cams = engine.cfg.data.get("cameras", [])
            cam = engine.cfg.data["camera"]
            if cams and 0 <= idx < len(cams):
                for k in ("url", "kind", "fps", "scale", "quality"):
                    if cams[idx].get(k) is not None:
                        cam[k] = cams[idx][k]
            cam["enabled"] = True
            engine.cfg.save()
            engine.log_msg("info", "robot connected - opening camera…")


            # act like the official controller (RETROID) so the robot enables
            # its video: stream neutral joystick frames while we wait
            presence_stop = threading.Event()

            def joystick_presence():
                while not presence_stop.is_set():
                    # pause the controller-impersonation while the user drives,
                    # so joystick movement and camera connect never fight
                    driving = False
                    with engine.lock:
                        d = engine.desired
                        tail = engine.cfg.data["limits"]["tail_ms"]
                        if (time.monotonic() - d["ts"]) * 1000.0 < tail:
                            if abs(d["vx"]) > 1e-3 or abs(d.get("vy", 0.0)) > 1e-3 or abs(d["wz"]) > 1e-3:
                                driving = True
                    if driving:
                        presence_stop.wait(0.15)
                        continue
                    try:
                        try:
                            frame12121 = engine._joy_frame(0.0, 0.0)
                        except Exception:
                            frame12121 = None
                        try:
                            import socket as _sk
                            for _port, _fr in ((12121, frame12121),):
                                if _fr:
                                    _s = _sk.socket(_sk.AF_INET, _sk.SOCK_DGRAM)
                                    _s.sendto(_fr, (ip, _port))
                                    _s.close()
                        except Exception:
                            pass
                        # low-rate presence on the motion-host command port too
                        try:
                            hb = engine.cfg.data.get("heartbeat", {})
                            engine._send_locked(engine.build_frame(hb))
                        except Exception:
                            pass
                    except Exception:
                        pass
                    presence_stop.wait(0.12)

            threading.Thread(target=joystick_presence, daemon=True).start()

            introspected = False

            def open_stream():
                nonlocal introspected
                # ask the robot to switch its camera driver on, every attempt
                send_app_cmd(ip, rc.get("port", 43899), rc.get("on", {}), repeats=5)
                send_app_cmd(ip, 43893, rc.get("on", {}), repeats=3)
                url0 = (cam.get("url") or "")
                endpoint = None
                # 1) previously working endpoint
                if url0 and (url0.startswith(ip) or "{robot_ip}" in url0):
                    cand = url0.replace("{robot_ip}", ip)
                    if _probe_endpoint(cand):
                        endpoint = cand
                # 2) ask the robot what it actually serves (SSH introspection)
                if not endpoint and not introspected:
                    introspected = True
                    try:
                        s_urls, s_paths = robot_camera_introspect(ip)
                        for u in s_urls:
                            if _probe_endpoint(u):
                                endpoint = u
                                break
                        if not endpoint:
                            for p in s_paths:
                                cand = f"rtsp://{ip}:8554/{p.lstrip('/')}"
                                if _probe_endpoint(cand):
                                    endpoint = cand
                                    break
                    except Exception:
                        pass
                # 3) generic auto-discover
                if not endpoint:
                    endpoint = discover_camera_endpoint(ip)
                if not endpoint:
                    # also probe the documented perception-host addresses
                    for alt in ("192.168.2.1", "192.168.1.103", "192.168.1.120"):
                        if alt == ip:
                            continue
                        endpoint = discover_camera_endpoint(alt)
                        if endpoint:
                            break
                if not endpoint:
                    return False
                cam["url"] = endpoint
                if endpoint.startswith("http"):
                    cam["kind"] = "ffmpeg"
                if cams:
                    cams[0]["url"] = endpoint
                engine.cfg.save()
                camera.stop()
                camera._forced_url = endpoint
                camera.start()
                return True
                cam["url"] = endpoint
                if endpoint.startswith("http"):
                    cam["kind"] = "ffmpeg"
                if cams:
                    cams[0]["url"] = endpoint
                engine.cfg.save()
                camera.stop()
                camera._forced_url = endpoint
                camera.start()
                return True

            for _ in range(10):
                if open_stream():
                    # confirm frames actually flow; re-discover once if not
                    for _ in range(3):
                        time.sleep(5)
                        if camera.frames and camera.frames > 0:
                            presence_stop.set()
                            return
                        err = camera.error or ""
                        if "404" in err or "exited" in err:
                            camera.stop()
                            camera._url = None
                            camera._forced_url = None
                            break
                    else:
                        continue
                    continue
                time.sleep(3)
            presence_stop.set()
            engine.log_msg("info", "robot camera not publishing yet - will open when it is")
        except Exception:
            pass  # silent - the camera is optional
        finally:
            CAM_BOOT_ACTIVE = False
            CAM_BOOT_LAST = time.monotonic()
    threading.Thread(target=work, daemon=True).start()


def robot_camera_introspect(ip):
    """Read-only SSH: find what camera/stream the robot actually runs.
    Returns (rtsp_urls, candidate_paths). Best-effort, silent."""
    import socket as _sk, re as _re
    try:
        s = _sk.create_connection((ip, 22), timeout=0.5)
        s.close()
    except Exception:
        return [], []
    cmd = (
        "ss -tlnp 2>/dev/null | grep -E '8554|554|8080|8090'; "
        "ps -eo args 2>/dev/null | grep -iE 'rtsp|mediamtx|gst-launch|realsense|ffmpeg|camera' | grep -v grep | head -30; "
        "cat /etc/mediamtx.yml /etc/rtsp-simple-server.yml /etc/mediamtx.yaml 2>/dev/null | grep -iE 'path|url' | head -40; "
        "systemctl list-units --type=service --state=running 2>/dev/null | grep -iE 'camera|realsense|voa|vision'; "
        "echo done"
    )
    ok, out = ssh_robot(ip, "sh -c '" + cmd + "'", timeout=10)
    urls = list(dict.fromkeys(_re.findall(r"rtsp://[^\s'\"&]+", out or "")))
    paths = []
    if not urls:
        for m in _re.finditer(r"(?i)(?:path|Path)\s*[:=]\s*([A-Za-z0-9_/.-]{1,40})", out or ""):
            p = m.group(1).strip().strip('"\'')
            if p and p not in paths:
                paths.append(p)
    return urls, paths


def send_app_cmd(ip, port, frame_cfg, value=None, repeats=6, interval=0.15):
    """Fire AppSimpleCMD frames at the robot (UDP, e.g. camera on/off).
    Best effort - UDP gives no reply. Returns number sent."""
    import socket as _sk
    sent = 0
    try:
        code = cfg_parse_int(frame_cfg.get("code", 0))
        typ = cfg_parse_int(frame_cfg.get("type", 0))
        if value is None:
            value = cfg_parse_int(frame_cfg.get("value", 0))
        frame = struct.pack("<III", code & 0xFFFFFFFF, value & 0xFFFFFFFF, typ & 0xFFFFFFFF)
        s = _sk.socket(_sk.AF_INET, _sk.SOCK_DGRAM)
        try:
            for _ in range(repeats):
                s.sendto(frame, (ip, port))
                sent += 1
                time.sleep(interval)
        finally:
            s.close()
    except Exception:
        pass
    return sent


def discover_robot():
    """Find the DeepRobotics robot on the current LAN.

    Signature: SSH (22) open on the same host as an RTSP port (554/8554).
    That combination is rare on a home LAN, so false positives are unlikely.
    Returns dict with found/ip/msg. Does not change config on its own.
    """
    import socket as sk
    import subprocess as sp
    try:
        r = sp.run(["ipconfig", "getifaddr", "en0"], capture_output=True, text=True, timeout=8)
        ip = (r.stdout or "").strip()
        if not ip or "." not in ip:
            return {"found": False, "msg": "no Wi-Fi IP"}
        prefix = ip.rsplit(".", 1)[0] + "."
        mylast = int(ip.rsplit(".", 1)[1])
    except Exception as exc:
        return {"found": False, "msg": str(exc)}

    def has_port(h, p, t=0.3):
        try:
            s = sk.create_connection((h, p), timeout=t)
            s.close()
            return True
        except Exception:
            return False

    rtsp_hosts = []
    lock = threading.Lock()

    def scan_one(i):
        h = prefix + str(i)
        if has_port(h, RTSP_PORTS[1]) or has_port(h, RTSP_PORTS[0]):
            with lock:
                rtsp_hosts.append(h)

    ts = [threading.Thread(target=scan_one, args=(i,)) for i in range(1, 255)
          if i != mylast]
    for t in ts:
        t.start()
    for t in ts:
        t.join(timeout=10)
    for h in sorted(rtsp_hosts):
        if has_port(h, 22, t=0.4):
            return {"found": True, "ip": h,
                    "msg": f"robot found at {h}"}
    # RTSP host without SSH on the same box: verify it really is the robot
    # by asking for its /test stream (home IP cameras don't have that path)
    for h in sorted(rtsp_hosts):
        for port in (RTSP_PORTS[1], RTSP_PORTS[0]):
            u = f"rtsp://{h}:{port}/test"
            try:
                ff = shutil.which("ffprobe") or "ffprobe"
                r = sp.run([ff, "-v", "error", "-rtsp_transport", "tcp",
                            "-show_entries", "format=format_name", "-of", "csv=p=0",
                            "-timeout", "2000000", "-rw_timeout", "2000000", u],
                           capture_output=True, text=True, timeout=6)
                if r.returncode == 0 and r.stdout.strip():
                    return {"found": True, "ip": h,
                            "msg": f"robot camera found at {h} (rtsp /test)"}
            except Exception:
                continue
    if rtsp_hosts:
        return {"found": False, "ip": None,
                "msg": "RTSP hosts without SSH seen: %s (not the robot?)" % ",".join(rtsp_hosts)}
    return {"found": False, "msg": "no robot found on this network"}


# ---------------------------------------------------------------- wifi scan
SWIFT_SCAN_SRC = r'''
import Foundation
import CoreWLAN
let iface = CWWiFiClient.shared().interface()
var out: [[String: Any]] = []
if let results = try? iface?.scanForNetworks(withSSID: nil, includeHidden: true) {
    for n in results {
        out.append(["ssid": n.ssid ?? "", "rssi": n.rssiValue, "bssid": n.bssid ?? ""])
    }
}
if let data = try? JSONSerialization.data(withJSONObject: out) {
    print(String(data: data, encoding: .utf8) ?? "[]")
}
'''

_SCAN_CACHE = {"at": 0.0, "list": []}
_SCAN_LOCK = threading.Lock()


def wifi_scan(force=False):
    """Scan nearby Wi-Fi networks (CoreWLAN via swift). Cached ~20s."""
    now = time.monotonic()
    if not force and now - _SCAN_CACHE["at"] < 20 and _SCAN_CACHE["list"]:
        return {"ok": True, "networks": _SCAN_CACHE["list"], "cached": True}
    with _SCAN_LOCK:
        now = time.monotonic()
        if not force and now - _SCAN_CACHE["at"] < 20 and _SCAN_CACHE["list"]:
            return {"ok": True, "networks": _SCAN_CACHE["list"], "cached": True}
        import subprocess as sp
        import tempfile
        script = os.path.join(tempfile.gettempdir(), "l3_wifi_scan.swift")
        try:
            with open(script, "w") as fh:
                fh.write(SWIFT_SCAN_SRC)
            r = sp.run(["swift", script], capture_output=True, text=True, timeout=45)
            nets = json.loads(r.stdout or "[]")
            nets = [n for n in nets if n.get("ssid")]
            nets.sort(key=lambda n: -(n.get("rssi") or -100))
            _SCAN_CACHE.update({"at": time.monotonic(), "list": nets})
            return {"ok": True, "networks": nets, "cached": False}
        except Exception as exc:
            return {"ok": False, "msg": f"scan failed: {exc}"}


def wifi_connect(ssid, password=None):
    """Join an arbitrary Wi-Fi network by SSID (password optional if saved)."""
    import subprocess as sp
    if not ssid:
        return {"ok": False, "msg": "no SSID given"}
    try:
        dev = "en0"
        r = sp.run(["networksetup", "-getairportnetwork", "en0"],
                   capture_output=True, text=True, timeout=10)
        if r.returncode != 0 or "not associated" in r.stdout.lower():
            for cand in ("en1", "en2"):
                r = sp.run(["networksetup", "-getairportnetwork", cand],
                           capture_output=True, text=True, timeout=10)
                if r.returncode == 0 and "not associated" not in r.stdout.lower():
                    dev = cand
                    break
        args = ["networksetup", "-setairportnetwork", dev, ssid]
        if password:
            args.append(password)
        for _ in range(2):
            sp.run(args, capture_output=True, text=True, timeout=30)
            time.sleep(5)
            ip = sp.run(["ipconfig", "getifaddr", dev], capture_output=True,
                        text=True, timeout=10).stdout.strip()
            if ip:
                return {"ok": True, "ip": ip, "joined": ssid}
        return {"ok": False, "msg": f"could not join '{ssid}' (wrong password or out of range?)"}
    except Exception as exc:
        return {"ok": False, "msg": str(exc)}


class Config:
    def __init__(self):
        self.data = json.loads(json.dumps(DEFAULTS))
        self.load()

    def load(self):
        try:
            with open(CONFIG_PATH) as fh:
                loaded = json.load(fh)
            self.data = deep_merge(json.loads(json.dumps(DEFAULTS)), loaded)
        except FileNotFoundError:
            self.save()
        except Exception as exc:
            print(f"[config] could not load config.json: {exc}")

    def save(self):
        try:
            with open(CONFIG_PATH, "w") as fh:
                json.dump(self.data, fh, indent=2)
        except Exception as exc:
            print(f"[config] could not save config.json: {exc}")


def deep_merge(base, extra):
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


class CommandEngine:
    """UDP command stream + telemetry receiver for the Lite3."""

    def __init__(self, config):
        self.cfg = config
        self.lock = threading.Lock()
        self.running = True

        self.cmd_sock = None
        self.state_sock = None
        self.state_thr = None

        self.estop = False
        self.posture = None  # tracked stand/sit ('None' = unknown until first toggle)
        self._gen = 0        # connect-generation: retires old telemetry/watch threads
        self.connected = False
        self.ping_ok = False
        self.ping_at = 0.0

        self.desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}  # ts = monotonic
        self.joy = {"x": 0.0, "y": 0.0, "ts": 0.0}
        self.burst = []               # list of bytes to repeat
        self.burst_until = 0.0

        self.telemetry = None          # latest decoded 0x0901 dict
        self.telemetry_at = 0.0
        self.frame_count = 0
        self.last_code = None

        self.log = []                  # ring buffer of events

    # ------------------------------------------------------------------ log
    def log_msg(self, level, text):
        with self.lock:
            self.log.append({"t": now_iso(), "lvl": level, "msg": text})
            del self.log[:-80]

    # ------------------------------------------------------------ lifecycle
    def connect(self):
        with self.lock:
            if self.connected and self.cmd_sock is not None:
                return self          # idempotent: repeat Connect presses are no-ops
            self._gen = getattr(self, "_gen", 0) + 1
            gen = self._gen
            self._open_sockets_locked()
            self.connected = True
            self.estop = False
            self._pinger = True
        self.log_msg("info", f"connected: sending cmds to {self.cfg.data['robot_ip']}:"
                             f"{self.cfg.data['cmd_port']}")
        threading.Thread(target=self.ping, daemon=True).start()
        threading.Thread(target=self._ping_loop, args=(gen,), daemon=True).start()
        threading.Thread(target=self._camera_watch, args=(gen,), daemon=True).start()

    def _camera_watch(self, gen=None):
        """Keep the robot camera feed open while connected: if the worker is
        down, re-send the robot-side camera-ON command and restart the feed.
        Paced so it never hammers the robot (>=8 s between attempts)."""
        while self.running and gen == getattr(self, "_gen", gen):
            with self.lock:
                if not self.connected:
                    return
                estop = self.estop
            cam = getattr(self, "_camera", None)
            if cam is not None and not estop:
                try:
                    st = cam.status()
                except Exception:
                    st = None
                running = bool(st and st.get("running"))
                if not running and self.cfg.data.get("camera", {}).get("enabled"):
                    now = time.monotonic()
                    if now - getattr(self, "_cam_watch_at", 0.0) >= 8.0:
                        self._cam_watch_at = now
                        try:
                            rc = self.cfg.data.get("robot_camera", {})
                            ip = self.cfg.data.get("robot_ip", "192.168.2.1")
                            send_app_cmd(ip, rc.get("port", 43899), rc.get("on", {}),
                                         repeats=3)
                            time.sleep(2.5)  # let the robot's camera service wake up
                            cam.start()
                            self.log_msg("info", "camera watchdog: feed restarted")
                        except Exception as exc:
                            self.log_msg("error", "camera watchdog: %s" % exc)
            time.sleep(2)

    def _ping_loop(self, gen=None):
        # keep the reachability flag fresh while connected
        while self.running and gen == getattr(self, "_gen", gen):
            with self.lock:
                if not self.connected or not getattr(self, "_pinger", False):
                    return
            time.sleep(5)
            try:
                self.ping()
            except Exception:
                pass
            cur = bool(self.ping_ok)
            prev = getattr(self, "_ping_ok_prev", None)
            self._ping_ok_prev = cur
            if cur and prev is False:
                cam = getattr(self, "_camera", None)
                if cam is not None and not cam.running:
                    try:
                        camera_boot(self, cam)
                    except Exception:
                        pass

    def _open_sockets_locked(self):
        c = self.cfg.data
        self.close_sockets_locked()
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.settimeout(1)
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind((c["state_bind"], c["state_port"]))
        self.state_sock.settimeout(0.5)
        gen = getattr(self, "_gen", 0)
        self.state_thr = threading.Thread(target=self._recv_loop, args=(gen,), daemon=True)
        self.state_thr.start()

    def close_sockets_locked(self):
        for s in (self.state_sock, self.cmd_sock):
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
        self.cmd_sock = None
        self.state_sock = None

    def disconnect(self):
        with self.lock:
            self._gen = getattr(self, "_gen", 0) + 1   # retire connect-generation threads
            self._pinger = False
            self.close_sockets_locked()
            self.connected = False
            self.desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}
            self.joy = {"x": 0.0, "y": 0.0, "ts": 0.0}
            self.burst = []
            self.telemetry = None
        self.log_msg("info", "disconnected")

    # ------------------------------------------------------------- telemetry
    def _recv_loop(self, gen=None):
        while self.running and gen == getattr(self, "_gen", gen):
            sock = self.state_sock
            if sock is None:
                time.sleep(0.2)
                continue
            try:
                data, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue
            self.frame_count += 1
            if len(data) >= 12:
                code, _size, _cons = struct.unpack_from("<iii", data, 0)
                self.last_code = code
            if len(data) >= STATE_TOTAL:
                parsed = self._parse_state(data[:STATE_TOTAL])
                if parsed is not None:
                    self.telemetry = parsed
                    self.telemetry_at = time.monotonic()

    def _parse_state(self, buf):
        code, _size, _cons = struct.unpack_from("<iii", buf, 0)
        if code != 0x0901:
            return None
        f = STATE_STRUCT.unpack_from(buf, 12)
        i = 0

        def take(n):
            nonlocal i
            out = f[i:i + n]
            i += n
            return out

        basic_state, gait_state = take(2)
        rpy = take(3)
        rpy_vel = take(3)
        xyz_acc = take(3)
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
            "rpy": [round(x, 3) for x in rpy],
            "rpy_vel": [round(x, 3) for x in rpy_vel],
            "acc": [round(x, 3) for x in xyz_acc],
            "pos": [round(x, 2) for x in pos_world],
            "vel_world": [round(x, 3) for x in vel_world],
            "vel_body": [round(x, 3) for x in vel_body],
            "touch_stair": touch_stair,
            "charging": bool(charging),
            "error_state": error_state,
            "motion_state": motion_state,
            "battery": round(battery, 4),
            "task_state": task_state,
            "need_move": bool(need_move),
            "zero_position": bool(zero_pos),
            "ultrasound": [round(x, 2) for x in ultrasound],
        }

    # -------------------------------------------------------------- commands
    def build_frame(self, frame_cfg, value=None, payload_bytes=None):
        """frame_cfg: {code, type, value?, payload?('double'|'int32'|'hex')}"""
        code = cfg_parse_int(frame_cfg.get("code", 0))
        typ = cfg_parse_int(frame_cfg.get("type", 0))
        second = cfg_parse_int(frame_cfg.get("value", 0))
        payload = payload_bytes
        pkind = frame_cfg.get("payload")
        if pkind == "double":
            payload = struct.pack("<d", float(value if value is not None else 0.0))
            second = 8
        elif pkind == "int32":
            payload = None
            second = cfg_parse_int(value if value is not None else frame_cfg.get("value", 0))
        elif pkind == "hex":
            payload = bytes.fromhex(frame_cfg.get("hex", ""))
            second = len(payload)
        head = struct.pack("<III", code & 0xFFFFFFFF, second & 0xFFFFFFFF, typ & 0xFFFFFFFF)
        return head + (payload or b"")

    def _send_locked(self, frame_bytes):
        if self.cmd_sock is not None:
            try:
                self.cmd_sock.sendto(frame_bytes,
                                     (self.cfg.data["robot_ip"], self.cfg.data["cmd_port"]))
            except OSError as exc:
                self.log_msg("error", f"send failed: {exc}")

    def ping(self):
        ip = self.cfg.data["robot_ip"]
        try:
            res = subprocess.run(["ping", "-c", "1", "-W", "1200", ip],
                                 capture_output=True, timeout=3)
            ok = res.returncode == 0
        except Exception:
            ok = False
        with self.lock:
            self.ping_ok = ok
            self.ping_at = time.monotonic()
        self.log_msg("info", f"ping {ip}: {'OK' if ok else 'no reply'}")

    def cmd_velocity(self, vx, vy=0.0, wz=0.0):
        with self.lock:
            if self.estop:
                return {"ok": False, "msg": "E-STOP latched - press Return first"}
            if not self.connected:
                return {"ok": False, "msg": "not connected"}
            vx = max(-self.cfg.data["limits"]["max_vx"],
                     min(self.cfg.data["limits"]["max_vx"], float(vx)))
            vy = max(-self.cfg.data["limits"].get("max_vy", 0.4),
                     min(self.cfg.data["limits"].get("max_vy", 0.4), float(vy)))
            wz = max(-self.cfg.data["limits"]["max_wz"],
                     min(self.cfg.data["limits"]["max_wz"], float(wz)))
            if not self._telemetry_recent() and not self.cfg.data["allow_motion_without_telemetry"]:
                return {"ok": False,
                        "msg": "no telemetry from robot - is 43897 open / network.toml set? "
                               "(motion blocked unless 'allow without telemetry' is on)"}
            self.desired = {"vx": vx, "vy": vy, "wz": wz, "ts": time.monotonic()}
            return {"ok": True}

    def cmd_stop(self):
        with self.lock:
            if not self.connected:
                return {"ok": False, "msg": "not connected"}
            self.desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": time.monotonic()}
            self.joy = {"x": 0.0, "y": 0.0, "ts": time.monotonic()}
            # official stop: send action-stop value 0 AND 1
            sc = self.cfg.data.get("stop_action", {})
            frames = [self.build_frame(sc, value=v) for v in (0, 1)]
            self._burst_frames_locked(frames, ms=400)
        return {"ok": True}

    def cmd_estop(self):
        with self.lock:
            self.estop = True
            self.posture = None  # e-stop can drop/sit the dog - state unknown now
            self.desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}
            self.joy = {"x": 0.0, "y": 0.0, "ts": 0.0}
            sc = self.cfg.data.get("stop_action", {})
            frames = [self.build_frame(sc, value=v) for v in (0, 1)]
            self.burst = frames
            self.burst_until = time.monotonic() + 1.5
        self.log_msg("warn", "E-STOP LATCHED (software). Use the physical E-stop too!")
        return {"ok": True}

    def cmd_reset_estop(self):
        with self.lock:
            self.estop = False
        self.log_msg("info", "E-stop reset")
        return {"ok": True}




    # 0x21010202 is a stand<->sit TOGGLE on the Lite3 (community tables + our
    # own test: re-sending it while standing made the dog sit). Stand/sit and
    # hello must therefore track the expected posture instead of blind-sending.
    POSTURE_TOGGLE = 0x21010202
    HELLO_POSE = 0x21010506  # official Lite3_LLM 'hello' action - plays from sitting

    def _send_posture(self, want):
        """Toggle to 'want' ('stand'|'sit') when the tracked posture differs.
        posture None (unknown) -> send and assume the requested state."""
        with self.lock:
            if self.estop:
                return {"ok": False, "msg": "E-STOP latched - press Return first"}
            if not self.connected:
                return {"ok": False, "msg": "not connected"}
            cur = self.posture
            if cur == want:
                return {"ok": True, "msg": "already " + want + " - nothing sent"}
            frame = self.build_frame({"code": "0x%X" % self.POSTURE_TOGGLE,
                                      "type": 0, "value": 0})
            self._burst_frames_locked([frame], ms=900)
            self.posture = want
            note = "" if cur is not None else " (state unknown - assuming " + want + " after toggle)"
        # log OUTSIDE the lock - log_msg acquires the same lock (non-reentrant)
        self.log_msg("info", "posture -> " + want + note)
        return {"ok": True, "msg": "posture -> " + want}

    def cmd_stand(self):
        return self._send_posture("stand")

    def cmd_sit(self):
        return self._send_posture("sit")

    def cmd_hello(self):
        with self.lock:
            if self.estop:
                return {"ok": False, "msg": "E-STOP latched - press Return first"}
            if not self.connected:
                return {"ok": False, "msg": "not connected"}
        threading.Thread(target=self._hello_thread, daemon=True).start()
        return {"ok": True}

    def _hello_thread(self):
        """Official 'hello' pose (0x21010506), which plays from the SITTING
        posture: sit first if needed, then play the pose. Cancellable via
        E-stop (the toggle dance is why the old stand+wiggle version made the
        dog sit instead of greeting)."""
        try:
            if self.posture != "sit":
                r = self._send_posture("sit")
                self.log_msg("info", "hello: sit first (%s)" % r["msg"])
                end = time.monotonic() + 2.2  # let the sit transition finish
                while time.monotonic() < end:
                    if self._estop_latched():
                        return
                    time.sleep(0.05)
            else:
                self.log_msg("info", "hello: already sitting")
            time.sleep(0.6)
            frame = self.build_frame({"code": "0x%X" % self.HELLO_POSE,
                                      "type": 0, "value": 0})
            # official Lite3_LLM impl re-sends the pose at ~1 Hz while engaged
            for i in range(3):
                with self.lock:
                    if not self.connected or self.estop:
                        return
                    self._send_locked(frame)
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    if self._estop_latched():
                        return
                    time.sleep(0.05)
            self.log_msg("info", "hello done")
        except Exception as exc:
            self.log_msg("error", f"hello failed: {exc}")

    def _estop_latched(self):
        with self.lock:
            return self.estop or not self.connected

    def cmd_action(self, name):
        # stand/sit share the 0x21010202 toggle - route through posture tracking
        if name in ("stand_up", "sit_down"):
            return self._send_posture("stand" if name == "stand_up" else "sit")
        acts = self.cfg.data["actions"]
        if name not in acts or not acts[name].get("enabled"):
            return {"ok": False, "msg": f"action '{name}' not enabled/configured"}
        fc = acts[name].get("frame", {})
        if not fc.get("code"):
            return {"ok": False,
                    "msg": f"'{name}' needs its real action code - capture it from the official "
                           f"app and set config.json actions.{name}.frame.code (hex). Nothing sent "
                           "for safety."}
        post = acts[name].get("posture")
        if post or acts[name].get("replay") == "slow":
            # threaded playback: optional posture step, then slow 1 Hz repeats
            # (rolls/poses need the official 1 Hz pattern, not a 50 Hz burst)
            threading.Thread(target=self._pose_thread,
                             args=(name, acts[name]), daemon=True).start()
            return {"ok": True,
                    "msg": f"'{name}' starting - pose: {post} first"}
        if acts[name].get("once"):
            # single-shot switch (modes/gaits): one frame, no burst - bursting a
            # toggle-type command could flip it right back
            with self.lock:
                if not self.connected or self.estop:
                    return {"ok": False, "msg": "not connected"}
                self._send_locked(self.build_frame(fc))
            self.log_msg("info", f"action '{name}' (once) -> sent")
            return {"ok": True}
        frame = self.build_frame(fc)
        self._burst_frames([frame])
        self.log_msg("info", f"action '{name}' -> {frame.hex()}")
        return {"ok": True}

    def _pose_thread(self, name, acfg):
        """Posture-aware pose playback. Only toggles posture when we KNOW it is
        wrong (e.g. tracked 'sit' but the action needs stand). If the tracked
        posture is unknown, send the action directly instead of blind-toggling -
        a wrong toggle (e.g. stand->sit) makes standing actions do nothing.
        Then re-send the pose at ~1 Hz (official Lite3_LLM pattern).
        Cancellable via E-stop."""
        try:
            post = acfg.get("posture")
            cur = self.posture
            if not post:
                self.log_msg("info", "%s: playing action (slow replay)" % name)
            elif cur is None and not acfg.get("force_posture"):
                self.log_msg("info", "%s: posture unknown - sending %s action "
                                     "directly (press STAND/SIT first if the dog is "
                                     "in the other pose)" % (name, post))
            elif cur != post:
                r = self._send_posture(post)
                if not r.get("ok"):
                    return
                self.log_msg("info", "%s: pose %s first (%s)" % (name, post, r["msg"]))
                end = time.monotonic() + 2.4   # let the posture transition finish
                while time.monotonic() < end:
                    if self._estop_latched():
                        return
                    time.sleep(0.05)
            else:
                self.log_msg("info", "%s: already %s" % (name, post))
            time.sleep(0.4)
            frame = self.build_frame(acfg["frame"])
            for _ in range(3):
                with self.lock:
                    if not self.connected or self.estop:
                        return
                    self._send_locked(frame)
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    if self._estop_latched():
                        return
                    time.sleep(0.05)
            self.log_msg("info", "action '%s' done" % name)
        except Exception as exc:
            self.log_msg("error", f"action '{name}' failed: {exc}")


    def cmd_raw(self, frame_cfg):
        try:
            frame = self.build_frame(frame_cfg)
        except Exception as exc:
            return {"ok": False, "msg": f"bad frame: {exc}"}
        self._burst_frames([frame])
        self.log_msg("info", f"raw frame -> {frame.hex()}")
        return {"ok": True, "hex": frame.hex()}

    def _burst_frames(self, frames):
        with self.lock:
            if not self.connected or self.estop:
                return False
            return self._burst_frames_locked(frames)

    def _burst_frames_locked(self, frames, ms=None):
        if not self.connected:
            return False
        self.burst = frames
        ms = ms if ms is not None else self.cfg.data["limits"]["action_ms"]
        self.burst_until = time.monotonic() + ms / 1000.0
        return True

    # ------------------------------------------------------------ main loop
    def tick_loop(self):
        interval = 1.0 / max(5, self.cfg.data["limits"]["rate_hz"])
        while self.running:
            t0 = time.monotonic()
            connected = estop = burst_send = False
            burst_frames = []
            with self.lock:
                connected = self.connected
                if connected:
                    estop = self.estop
                    if estop:
                        # keep pushing the stop frames while latched, then quiet
                        if self.burst and t0 < self.burst_until:
                            burst_frames = list(self.burst)
                            burst_send = True
                        else:
                            self.burst = []
                    else:
                        # heartbeat keep-alive (motion host expects >=2 Hz)
                        hb = self.cfg.data.get("heartbeat", {})
                        hb_ms = int(hb.get("interval_ms", 300))
                        if t0 - getattr(self, "_last_hb", 0.0) >= hb_ms / 1000.0:
                            self._last_hb = t0
                            self._send_locked(self.build_frame(hb))
                        # RETROID-style joystick frames take priority while held
                        j = getattr(self, "joy", None)
                        joy_active = bool(j is not None and
                                          (t0 - j["ts"]) * 1000.0 < self.cfg.data["limits"]["tail_ms"])
                        if joy_active:
                            self._send_joy_locked(j["x"], j["y"], j.get("vy", 0.0))
                        elif self.burst and t0 < self.burst_until:
                            burst_frames = list(self.burst)
                            burst_send = True
                        else:
                            self.burst = []
                            d = self.desired
                            age_ms = (t0 - d["ts"]) * 1000.0
                            if age_ms < self.cfg.data["limits"]["tail_ms"]:
                                vf = self.cfg.data.get("velocity",
                                                       {"code": "0x140", "type": 1,
                                                        "payload": "double", "sign": 1})
                                lf = self.cfg.data.get("velocity_lateral",
                                                       {"code": "0x145", "type": 1,
                                                        "payload": "double"})
                                yf = self.cfg.data.get("velocity_yaw",
                                                       {"code": "0x141", "type": 1,
                                                        "payload": "double"})
                                self._send_locked(self.build_frame(
                                    vf, value=d["vx"] * float(vf.get("sign", 1))))
                                self._send_locked(self.build_frame(
                                    lf, value=d.get("vy", 0.0) * float(lf.get("sign", 1))))
                                self._send_locked(self.build_frame(
                                    yf, value=d.get("wz", 0.0) * float(yf.get("sign", 1))))
                                self._send_locked(self.build_frame(
                                    self.cfg.data["auto_mode"]))
                            elif d["vx"] != 0.0 or d["wz"] != 0.0:
                                # driver stopped sending (stick centered/released):
                                # zero the request AND fire the official brake
                                # frames - never let the robot coast on stale cmd
                                self.desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": t0}
                                sc = self.cfg.data.get("stop_action", {})
                                try:
                                    self._burst = [self.build_frame(sc, value=v) for v in (0, 1)]
                                    self._burst_until = time.monotonic() + 0.35
                                except Exception:
                                    pass
                            # idle keep-alive: stay in auto mode so the first
                            # joystick input is always accepted instantly
                            if t0 - getattr(self, "_last_auto_idle", 0.0) >= 1.0:
                                self._last_auto_idle = t0
                                self._send_locked(self.build_frame(
                                    self.cfg.data["auto_mode"]))
            # ---- outside the lock: burst repeats + pacing (never sleep in lock) ----
            if burst_send:
                for fb in burst_frames:
                    self._send_locked(fb)
            dt = time.monotonic() - t0
            if not connected:
                time.sleep(0.1)
            elif estop:
                time.sleep(0.02)
            elif interval - dt > 0:
                time.sleep(interval - dt)

    # ---------------------------------------------------------- joystick
    def cmd_joystick(self, x, y, vy=0.0):
        """x: -1..1 (yaw/turn), y: -1..1 (forward/back), vy: -1..1 (side-walk).
        RETROID-style frame."""
        with self.lock:
            if self.estop:
                return {"ok": False, "msg": "E-STOP latched - press Return first"}
            if not self.connected:
                return {"ok": False, "msg": "not connected"}
            if not hasattr(self, "joy"):
                self.joy = {"x": 0.0, "y": 0.0, "vy": 0.0, "ts": 0.0}
            self.joy.update({"x": max(-1.0, min(1.0, float(x))),
                             "y": max(-1.0, min(1.0, float(y))),
                             "vy": max(-1.0, min(1.0, float(vy))),
                             "ts": time.monotonic()})
            return {"ok": True}

    def _joy_frame(self, x, y, vy=0.0):
        """42-byte RetroidGamepadData packet (DeepRoboticsLab/gamepad spec).
        x = yaw (right stick X), y = forward (left stick Y), vy = side-walk
        (left stick X)."""
        js = self.cfg.data.get("joystick", {})
        seq = int(js.get("seq", 0)) & 0xFFFF
        js["seq"] = seq + 1
        yaw_sign = -1 if js.get("yaw_sign", -1) else 1
        rng = int(js.get("range", 1000)) or 1000
        rx = int(round(yaw_sign * x * rng))
        ly = int(round(y * rng))
        lx = int(round(vy * rng))
        rx = max(-rng, min(rng, rx))
        ly = max(-rng, min(rng, ly))
        lx = max(-rng, min(rng, lx))
        payload = struct.pack("<10Hhhhh2H", *([0] * 10), lx, ly, rx, 0, 0, 0)
        crc = sum(payload) & 0xFFFF
        head = struct.pack("<2sBHHBH", b"\x55\x66", 0, 32, seq, 1, crc)
        return head + payload

    def _send_joy_locked(self, x, y, vy=0.0):
        if self.cmd_sock is not None:
            try:
                js = self.cfg.data.get("joystick", {})
                self.cmd_sock.sendto(self._joy_frame(x, y, vy),
                                     (self.cfg.data["robot_ip"], js.get("port", 12121)))
            except OSError:
                pass

    def _telemetry_recent(self, window=5.0):
        if self.telemetry is None:
            return False
        return (time.monotonic() - self.telemetry_at) < window

    # --------------------------------------------------------------- status
    def state(self):
        with self.lock:
            t = self.telemetry
            battery = None
            if t is not None:
                battery = t["battery"]
            return {
                "connected": bool(self.connected and (self.ping_ok
                                                      or self._telemetry_recent(8))),
                "robot_ip": self.cfg.data["robot_ip"],
                "cmd_port": self.cfg.data["cmd_port"],
                "state_port": self.cfg.data["state_port"],
                "ping_ok": self.ping_ok,
                "estop": self.estop,
                "telemetry_age_ms": int((time.monotonic() - self.telemetry_at) * 1000)
                if t is not None else None,
                "telemetry": t,
                "battery_pct": round(battery * 100.0, 1) if battery is not None else None,
                "charging": bool(t["charging"]) if t is not None else None,
                "height_m": round(t["pos"][2], 3) if t is not None else None,
                "frame_count": self.frame_count,
                "last_frame_code": self.last_code,
                "log": list(self.log),
                "engine": {
                    "allow_motion_without_telemetry": bool(
                        self.cfg.data["allow_motion_without_telemetry"]),
                    "max_vx": self.cfg.data["limits"]["max_vx"],
                    "max_vy": self.cfg.data["limits"].get("max_vy", 0.4),
                    "max_wz": self.cfg.data["limits"]["max_wz"],
                    "joystick": {k: self.cfg.data.get("joystick", {}).get(k)
                                 for k in ("port", "range", "mode", "curve", "scale")},
                    "signs": {k: int(self.cfg.data.get(k, {}).get("sign", 1))
                              for k in ("velocity", "velocity_lateral", "velocity_yaw")},
                    "wifi": {k: self.cfg.data.get("wifi", {}).get(k)
                             for k in ("robot_ssid", "home_ssid", "robot_subnet")},
                },
                "actions": [
                    {"name": n, "note": a.get("note", ""),
                     "icon": a.get("icon", ""), "label": a.get("label", ""),
                     "group": a.get("group", "action")}
                    for n, a in self.cfg.data["actions"].items()
                    if a.get("enabled")
                ],
            }


class CameraWorker:
    """Fetches robot/attached camera frames and exposes the latest JPEG."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.proc = None
        self.frame = None
        self.frame_at = 0.0
        self.frames = 0
        self.error = None
        self.last_error_at = 0.0

    def status(self):
        with self.lock:
            return {
                "running": self.running,
                "source": self.cfg.data["camera"].get("url", ""),
                "effective": getattr(self, "_url", None),
                "kind": self.cfg.data["camera"].get("kind", "auto"),
                "frames": self.frames,
                "age_ms": int((time.monotonic() - self.frame_at) * 1000) if self.frame else None,
                "error": self.error if (time.monotonic() - self.last_error_at) < 10 else None,
            }

    def start(self):
        # concurrent start() calls (Connect spam + camera watchdog) must not
        # spawn multiple ffmpeg pipelines / probe storms
        if not hasattr(self, "_start_lk"):
            self._start_lk = threading.Lock()
        if not self._start_lk.acquire(blocking=False):
            return
        try:
            self._start_impl()
        finally:
            self._start_lk.release()

    def _start_impl(self):
        cam = self.cfg.data["camera"]
        if not cam.get("url") or not cam.get("enabled"):
            self.error = "camera disabled or no URL set"
            self.last_error_at = time.monotonic()
            return
        # use the already-discovered endpoint when set, else auto-probe
        forced = getattr(self, "_forced_url", None)
        resolved = forced or self.resolve_url()
        self._forced_url = None
        if resolved is None:
            self.error = ("no robot camera found - is the robot on this network? "
                          "(checked rtsp :8554/:554 and mjpg :8080/:8090)")
            self.last_error_at = time.monotonic()
            return
        # wait out a previous decode loop that is still shutting down
        th = self.thread
        if th is not None and th.is_alive():
            th.join(timeout=2)
        if th is not None and th.is_alive():
            self.error = "previous camera stream is still shutting down"
            self.last_error_at = time.monotonic()
            return
        with self.lock:
            self.running = True
            self.frame = None
            self.error = None
            self._url = resolved
            if self.thread is None or not self.thread.is_alive():
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()

    def stop(self):
        with self.lock:
            self.running = False
            if self.proc is not None:
                try:
                    self.proc.terminate()
                except Exception:
                    pass
                self.proc = None

    def resolve_url(self):
        """Auto-detect the robot camera: try the known RTSP + MJPEG endpoints
        and use the first one that answers."""
        cam = self.cfg.data["camera"]
        ip = str(self.cfg.data.get("robot_ip", "192.168.2.1"))
        raw = cam.get("url") or ""
        substituted = raw.replace("{robot_ip}", ip)
        if not bool(cam.get("auto_probe", True)):
            return substituted or None
        candidates = [
            f"rtsp://{ip}:8554/test",
            f"rtsp://{ip}:8554/live",
            f"rtsp://{ip}:554/test",
            f"rtsp://{ip}:8554/ch0",
            f"http://{ip}:8080/?action=stream",
            f"http://{ip}:8090/?action=stream",
            f"http://{ip}:8080/stream/video.mjpeg",
        ]
        import socket as _sock
        open_tcp = []
        for u in candidates:
            try:
                from urllib.parse import urlparse
                pr = urlparse(u)
                s = _sock.create_connection((pr.hostname, pr.port or 80), timeout=0.4)
                s.close()
                open_tcp.append(u)
            except Exception:
                pass
        if not open_tcp:
            return None
        for u in open_tcp[:3]:
            try:
                ff = shutil.which("ffprobe") or "ffprobe"
                args = ([ff, "-v", "error", "-rtsp_transport", "tcp"]
                        if u.startswith("rtsp") else [ff, "-v", "error"])
                args += ["-show_entries", "format=format_name", "-of", "csv=p=0",
                         "-timeout", "3000000", "-rw_timeout", "3000000", u]
                r = subprocess.run(args, capture_output=True, text=True, timeout=6)
                if r.returncode == 0 and r.stdout.strip():
                    cams = self.cfg.data.get("cameras", [])
                    if cams:
                        cams[0]["url"] = u
                    cam["url"] = u
                    if u.startswith("http"):
                        cam["kind"] = "ffmpeg"
                    self.cfg.save()
                    return u
            except Exception:
                continue
        return open_tcp[0]

    def _set_err(self, msg):
        with self.lock:
            self.error = msg
            self.last_error_at = time.monotonic()

    def _emit(self, jpeg):
        """Store one decoded frame for /cam.mjpeg."""
        now = time.monotonic()
        with self.lock:
            self.frame = jpeg
            self.frame_at = now
            self.frames += 1

    def _run(self):
        """Decode loop: ffmpeg pulls the source (RTSP / MJPEG / file) and
        re-emits JPEG frames for /cam.mjpeg, photo and recording."""
        cam = self.cfg.data["camera"]
        url = getattr(self, "_url", None)
        if not url:
            self._set_err("no camera url to open")
            return
        ff = shutil.which("ffmpeg")
        if not ff:
            self._set_err("ffmpeg not found (brew install ffmpeg)")
            return
        if cam.get("kind") == "http_jpeg":
            self._run_http_jpeg(url)
            return
        import select as _sel
        args = [ff, "-hide_banner", "-loglevel", "error", "-an"]
        if url.lower().startswith("rtsp"):
            # NOTE: -stimeout / -rw_timeout were removed in ffmpeg 8 - passing
            # them makes the whole command fail instantly (silent camera death)
            args += ["-rtsp_transport", "tcp",
                     "-fflags", "nobuffer",
                     "-flags", "low_delay",
                     "-analyzeduration", "300000",
                     "-probesize", "300000"]
        args += ["-i", url]
        scale = str(cam.get("scale") or "").strip()
        if scale:
            args += ["-vf", "scale=" + scale]
        args += ["-q:v", str(int(cam.get("quality", 7))),
                 "-f", "mjpeg", "pipe:1"]
        try:
            proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL)
        except Exception as exc:
            self._set_err("ffmpeg failed to start: %s" % exc)
            return
        with self.lock:
            self.proc = proc
        fd = proc.stdout.fileno()
        tail = b""
        last_emit = time.monotonic()
        # The mjpeg demuxer has no timestamps, so an HTTP-MJPEG source would
        # otherwise be re-emitted as fast as it can be read; pace to the
        # configured fps (RTSP sources are already real-time paced).
        target_fps = max(1, int(cam.get("fps") or 12))
        per = 1.0 / target_fps
        prev = 0.0
        try:
            while self.running and proc.poll() is None:
                ready = _sel.select([fd], [], [], 1.0)[0]
                if not ready:
                    # source stalled (or the stream died silently)
                    if time.monotonic() - last_emit > 6.0:
                        break
                    continue
                chunk = os.read(fd, 65536)
                if not chunk:
                    break
                data = tail + chunk
                tail = b""
                pos = 0
                while True:
                    s = data.find(b"\xff\xd8", pos)
                    if s < 0:
                        break
                    e = data.find(b"\xff\xd9", s + 2)
                    if e < 0:
                        tail = data[s:]  # frame continues in the next chunk
                        break
                    now = time.monotonic()
                    if prev:
                        wait = per - (now - prev)
                        if wait > 0:
                            time.sleep(wait)
                    prev = time.monotonic()
                    self._emit(data[s:e + 2])
                    last_emit = time.monotonic()
                    pos = e + 2
        finally:
            try:
                proc.terminate()
            except Exception:
                pass
            try:
                proc.wait(timeout=3)
            except Exception:
                pass
            with self.lock:
                self.running = False
                if self.proc is proc:
                    self.proc = None

    def _run_http_jpeg(self, url):
        """Legacy plain-JPEG endpoint, polled at the configured rate."""
        import urllib.request as _ur
        cam = self.cfg.data["camera"]
        interval = 1.0 / max(1, int(cam.get("fps", 10)))
        while self.running:
            try:
                with _ur.urlopen(url, timeout=5) as r:
                    data = r.read()
                if data[:2] == b"\xff\xd8":
                    self._emit(data)
            except Exception:
                pass
            time.sleep(interval)
        with self.lock:
            self.running = False
            self.proc = None

    # ------------------------------------------------------------- capture
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    engine = None      # set by main
    camera = None

    # ------------------------------------------------------------- helpers
    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode() or "{}")
        except Exception:
            return {}

    def _log_req(self, *_):
        pass  # keep the console quiet

    def _safe_connect(self):
        time.sleep(1.0)  # let the link settle
        try:
            self.engine.connect()
            self._maybe_start_camera()
        except Exception as exc:
            self.engine.log_msg("error", f"auto-connect failed: {exc}")

    def _maybe_start_camera(self):
        """Auto camera connection (from scratch controller)."""
        camera_boot(self.engine, self.camera)

    def _camera_on_with_retry(self, reason="camera starting"):
        camera_boot(self.engine, self.camera)

    def _handle_wifi_action(self, action):
        """join_robot | join_home - shared by GET and POST (the rolled-back UI
        calls these as GET from autoLink/buttons; without these routes the
        console can never join the robot Wi-Fi)."""
        res = wifi_ctl(action)
        if res.get("ok"):
            if action == "join_robot":
                # re-point at the robot and connect automatically
                self.engine.cfg.data["robot_ip"] = WIFI_CFG["robot_subnet"] + "1"
                self.engine.cfg.save()
                threading.Thread(target=self._safe_connect, daemon=True).start()
            elif action == "join_home":
                self.engine.disconnect()
        self._json(res, 200 if res.get("ok") else 400)

    def _handle_discover(self):
        """Robot discovery - shared by GET and POST."""
        res = discover_robot()
        if res.get("found"):
            self.engine.cfg.data["robot_ip"] = res["ip"]
            self.engine.cfg.save()
            self.engine.log_msg("info", "robot discovered at %s - connecting" % res["ip"])
            threading.Thread(target=self._safe_connect, daemon=True).start()
        self._json(res)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/" or path == "/index.html":
            self._serve_file("index.html")
        elif path in ("/app.js", "/style.css"):
            self._serve_file(path.lstrip("/"))
        elif path == "/api/state":
            cam = self.camera.status()
            cams = self.engine.cfg.data.get("cameras", [])
            cam["cameras"] = [{"name": c.get("name", "Camera %d" % (i + 1)),
                                "has_url": bool(c.get("url"))}
                               for i, c in enumerate(cams)]
            cam["active_name"] = cams[self.engine.cfg.data.get("camera_active", 0)].get("name") if cams else None
            st = self.engine.state()
            st["camera"] = cam
            self._json(st)
        elif path == "/api/ping":
            threading.Thread(target=self.engine.ping, daemon=True).start()
            self._json({"ok": True, "msg": "ping started"})
        elif path == "/api/wifi/status":
            self._json(wifi_ctl("status"))
        elif path == "/api/wifi/scan":
            self._json(wifi_scan(force="force=1" in self.path))
        elif path == "/cam.mjpeg":
            self._stream_mjpeg()
        elif path == "/api/robot/discover":
            self._handle_discover()
        elif path == "/api/wifi/join_robot":
            self._handle_wifi_action("join_robot")
        elif path == "/api/wifi/home":
            self._handle_wifi_action("join_home")
        else:
            self._json({"ok": False, "msg": "not found"}, 404)

    def _serve_file(self, name):
        full = os.path.join(STATIC_DIR, name)
        if not os.path.isfile(full):
            self._json({"ok": False, "msg": "missing static file"}, 404)
            return
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        with open(full, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        deadline = time.monotonic() + 6
        last_sent_at = -1.0
        while True:
            cam = self.camera
            with cam.lock:
                frame = cam.frame
                frame_at = cam.frame_at
            now = time.monotonic()
            if frame is not None and frame_at != last_sent_at:
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(frame)).encode() +
                                     b"\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                last_sent_at = frame_at
                deadline = time.monotonic() + 6
            elif now > deadline:
                return
            else:
                time.sleep(0.02)

    # ---------------------------------------------------------------- POST
    def do_POST(self):
        path = self.path.split("?")[0]
        body = self._read_body()
        if path == "/api/connect":
            if "robot_ip" in body:
                self.engine.cfg.data["robot_ip"] = str(body["robot_ip"])
            if "cmd_port" in body:
                self.engine.cfg.data["cmd_port"] = int(body["cmd_port"])
            if "state_port" in body:
                self.engine.cfg.data["state_port"] = int(body["state_port"])
            self.engine.cfg.save()
            try:
                self.engine.connect()
                self._maybe_start_camera()
                self._json({"ok": True})
            except Exception as exc:
                self._json({"ok": False, "msg": f"connect failed: {exc}"}, 400)
        elif path == "/api/disconnect":
            self.engine.disconnect()
            self._json({"ok": True})
        elif path == "/api/wifi/join_robot":
            res = wifi_ctl("join_robot")
            if res.get("ok"):
                # re-point at the robot and connect automatically
                self.engine.cfg.data["robot_ip"] = WIFI_CFG["robot_subnet"] + "1"
                self.engine.cfg.save()
                threading.Thread(target=self._safe_connect, daemon=True).start()
            self._json(res, 200 if res.get("ok") else 400)
        elif path == "/api/wifi/home":
            res = wifi_ctl("join_home")
            self.engine.disconnect()
            self._json(res, 200 if res.get("ok") else 400)
        elif path == "/api/wifi/connect":
            ssid = str(body.get("ssid", "")).strip()
            pw = body.get("password")
            if pw in (None, ""):
                pw = None
            wf = self.engine.cfg.data.get("wifi", {})
            if pw is None and ssid == wf.get("robot_ssid"):
                pw = wf.get("robot_password")   # use the saved robot password
            res = wifi_connect(ssid, pw)
            code = 200 if res.get("ok") else 400
            # never echo a password back in logs/responses
            res.pop("password", None)
            if res.get("ok"):
                self.engine.cfg.save()
            self._json(res, code)
        elif path == "/api/camera/switch":
            idx = body.get("index")
            if idx is None:
                name = body.get("name")
                cams = self.engine.cfg.data.get("cameras", [])
                idx = next((i for i, c in enumerate(cams) if c.get("name") == name), None)
            try:
                idx = int(idx)
            except (TypeError, ValueError):
                self._json({"ok": False, "msg": "bad camera index"}, 400)
                return
            cams = self.engine.cfg.data.get("cameras", [])
            if not cams or not (0 <= idx < len(cams)):
                self._json({"ok": False, "msg": "bad camera index"}, 400)
                return
            preset = cams[idx]
            self.engine.cfg.data["camera_active"] = idx
            cam = self.engine.cfg.data["camera"]
            for k in ("url", "kind", "fps", "scale", "quality"):
                if k in preset:
                    cam[k] = preset[k]
            cam["enabled"] = bool(preset.get("url"))
            self.engine.cfg.save()
            self.camera.stop()
            if cam["enabled"]:
                self.camera.start()
            self._json({"ok": True, "name": preset.get("name"), "switched": True})
        elif path == "/api/camera/diagnose":
            rows = []
            ip = self.engine.cfg.data.get("robot_ip", "192.168.2.1")
            import socket as _sk, subprocess as _sp
            # 1) robot reachable?
            try:
                r = _sp.run(["ping", "-c", "1", "-W", "1000", ip], capture_output=True, timeout=3)
                rows.append(("robot reachable (ping)", "yes" if r.returncode == 0 else "no reply"))
            except Exception as exc:
                rows.append(("robot reachable (ping)", "error " + str(exc)))
            # 2) open ports
            open_p = []
            for p in (22, 554, 8554, 8080, 8090, 80, 43893, 43899, 12121):
                try:
                    s = _sk.create_connection((ip, p), timeout=0.4)
                    s.close()
                    open_p.append(str(p))
                except Exception:
                    pass
            rows.append(("open robot ports", ", ".join(open_p) if open_p else "none"))
            # 3) try RTSP paths - keep the real error for each
            if "8554" in open_p or "554" in open_p:
                tried = []
                for port in (8554, 554):
                    for p in ("/test", "/live", "/ch0", "/ch1", "/stream", "/main"):
                        url = f"rtsp://{ip}:{port}{p}"
                        try:
                            ff = shutil.which("ffprobe") or "ffprobe"
                            rr = _sp.run([ff, "-v", "error", "-rtsp_transport", "tcp",
                                          "-show_entries", "format=format_name", "-of", "csv=p=0",
                                          "-timeout", "1200000", "-rw_timeout", "1200000", url],
                                         capture_output=True, text=True, timeout=4)
                            if rr.returncode == 0 and rr.stdout.strip():
                                tried.append(f"{url} -> WORKS")
                            else:
                                tried.append(f"{url} -> {(rr.stderr or rr.stdout).strip()[:70] or 'no stream'}")
                        except Exception as exc:
                            tried.append(f"{url} -> timeout")
                for t in tried[:8]:
                    rows.append(("rtsp probe", t))
            else:
                rows.append(("rtsp probe", "RTSP server not listening (no 8554/554 open)"))
            # 4) camera driver cmd
            rows.append(("camera-ON cmd", "sent to udp 43899/43893 (SDK 0x21012109) - robot must publish /test after it"))
            rows.append(("camera drivers", ", ".join(CAMERA_DRIVERS)))
            rows.append(("driver start (ssh)", "attempted over ssh:22 when open, hosts 2.1/1.103/1.120"))
            rows.append(("streaming client active", "controller frames sent to :12121 while waiting"))
            eff = (self.camera.status().get("effective") or "")
            rows.append(("saved stream url", eff or str(self.engine.cfg.data.get("camera", {}).get("url", ""))))
            self._json({"ok": True, "rows": rows})
        elif path == "/api/robot/discover":
            res = discover_robot()
            if res.get("found"):
                self.engine.cfg.data["robot_ip"] = res["ip"]
                self.engine.cfg.save()
                self.engine.log_msg("info", f"robot discovered at {res['ip']} - connecting")
                threading.Thread(target=self._safe_connect, daemon=True).start()
            self._json(res)
        elif path == "/api/camera/on":
            rc = self.engine.cfg.data.get("robot_camera", {})
            ip = self.engine.cfg.data.get("robot_ip", "192.168.2.1")
            n = send_app_cmd(ip, rc.get("port", 43899), rc.get("on", {}))
            if n == 0:
                self._json({"ok": False, "msg": "could not reach the robot camera control"}, 200)
                return
            # load the front-cam preset into the active camera config
            idx = self.engine.cfg.data.get("camera_active", 0)
            cams = self.engine.cfg.data.get("cameras", [])
            cam = self.engine.cfg.data["camera"]
            if cams and 0 <= idx < len(cams):
                for k in ("url", "kind", "fps", "scale", "quality"):
                    if cams[idx].get(k) is not None:
                        cam[k] = cams[idx][k]
            cam["enabled"] = True
            self.engine.cfg.save()
            self.engine.log_msg("info", f"camera ON cmd sent to robot ({ip}:{rc.get('port')}) - starting feed")
            self.camera.stop()
            self.camera.start()
            time.sleep(4)
            st = self.camera.status()
            self._json({"ok": True,
                        "frames": st.get("frames"),
                        "running": st.get("running"),
                        "error": st.get("error"),
                        "msg": "camera live" if st.get("frames") else
                        "switch sent; waiting for stream (robot camera may take a few seconds)"})
        elif path == "/api/camera/off":
            rc = self.engine.cfg.data.get("robot_camera", {})
            ip = self.engine.cfg.data.get("robot_ip", "192.168.2.1")
            send_app_cmd(ip, rc.get("port", 43899), rc.get("off", {}))
            self.camera.stop()
            self.engine.log_msg("info", "camera OFF cmd sent to robot; feed stopped")
            self._json({"ok": True, "msg": "camera OFF"})
        elif path == "/api/camera/autoconfigure":
            # find the robot, probe its real camera stream, persist it, start it
            res = {"ok": False, "msg": ""}
            d = discover_robot()
            if d.get("found"):
                self.engine.cfg.data["robot_ip"] = d["ip"]
            ip = self.engine.cfg.data.get("robot_ip", "192.168.2.1")
            # ask the robot to switch its camera driver on (SDK AppSimpleCMD)
            rc = self.engine.cfg.data.get("robot_camera", {})
            send_app_cmd(ip, rc.get("port", 43899), rc.get("on", {}))
            time.sleep(2)
            url = self.camera.resolve_url()
            if not url:
                res["msg"] = (f"no camera stream found on {ip} (rtsp :8554/:554 /test, mjpg :8080/:8090). "
                               "Is the Mac on the robot's network and is the robot's camera switched on?")
                self._json(res, 200)
                return
            # persist the concrete working URL into the front-cam preset
            cams = self.engine.cfg.data.get("cameras", [])
            if cams:
                cams[0]["url"] = url
            cam = self.engine.cfg.data["camera"]
            cam["url"] = url
            cam["enabled"] = True
            self.engine.cfg.save()
            self.engine.log_msg("info", f"camera autoconfigured: {url}")
            self.camera.stop()
            self.camera.start()
            time.sleep(4)
            st = self.camera.status()
            res = {"ok": bool(st.get("frames")),
                   "robot_ip": ip,
                   "url": url,
                   "running": st.get("running"),
                   "frames": st.get("frames"),
                   "error": st.get("error"),
                   "msg": "camera live" if st.get("frames") else "stream started but no frames yet"}
            self._json(res)
        elif path == "/api/cmd":
            typ = body.get("type")
            eng = self.engine
            if typ == "velocity":
                res = eng.cmd_velocity(float(body.get("vx", 0)),
                                       float(body.get("vy", 0)),
                                       float(body.get("wz", 0)))
            elif typ == "joystick":
                res = eng.cmd_joystick(float(body.get("x", 0)), float(body.get("y", 0)),
                                       float(body.get("vy", 0)))
            elif typ == "stop":
                res = eng.cmd_stop()
            elif typ == "hello":
                res = eng.cmd_hello()
            elif typ == "estop":
                res = eng.cmd_estop()
            elif typ == "reset_estop":
                res = eng.cmd_reset_estop()
            elif typ == "action":
                res = eng.cmd_action(str(body.get("name", "")))
            elif typ == "raw":
                res = eng.cmd_raw(body.get("frame", {}))
            else:
                res = {"ok": False, "msg": f"unknown cmd type '{typ}'"}
            self._json(res, 200 if res.get("ok") else 400)
        elif path == "/api/config":
            if body.get("section") == "camera":
                cam = self.engine.cfg.data["camera"]
                for k in ("enabled", "url", "kind", "fps", "scale", "quality"):
                    if k in body:
                        cam[k] = body[k]
                # when starting without an explicit URL, load the active preset
                if body.get("start") and not body.get("url"):
                    idx = self.engine.cfg.data.get("camera_active", 0)
                    cams = self.engine.cfg.data.get("cameras", [])
                    if cams and 0 <= idx < len(cams):
                        pr = cams[idx]
                        for k in ("url", "kind", "fps", "scale", "quality"):
                            if pr.get(k) is not None:
                                cam[k] = pr[k]
                # keep the active preset in sync so edits stick to this camera
                idx = self.engine.cfg.data.get("camera_active", 0)
                cams = self.engine.cfg.data.get("cameras", [])
                if cams and 0 <= idx < len(cams):
                    for k in ("url", "kind", "fps", "scale", "quality"):
                        if k in body:
                            cams[idx][k] = cam[k]
                self.engine.cfg.save()
                if body.get("start"):
                    # send the SDK camera-ON cmd and retry until the stream publishes
                    self._camera_on_with_retry("camera ON requested")
                else:
                    self.camera.stop()
                self._json({"ok": True})
            elif body.get("section") == "engine":
                eng = self.engine.cfg.data
                if "allow_motion_without_telemetry" in body:
                    eng["allow_motion_without_telemetry"] = bool(
                        body["allow_motion_without_telemetry"])
                if "robot_ip" in body and body["robot_ip"]:
                    eng["robot_ip"] = str(body["robot_ip"]).strip()
                for k in ("velocity", "velocity_lateral", "velocity_yaw"):
                    sk = k + "_sign"
                    if sk in body and body[sk] in (1, -1, "1", "-1"):
                        eng.get(k, {})["sign"] = int(body[sk])
                for k in ("max_vx", "max_wz"):
                    if k in body and body[k] is not None:
                        eng["limits"][k] = float(body[k])
                if body.get("joystick_mode") in ("velocity", "retroid"):
                    eng.get("joystick", {})["mode"] = body["joystick_mode"]
                if body.get("joystick_yaw_sign") in (1, -1, "1", "-1"):
                    eng.get("joystick", {})["yaw_sign"] = int(body["joystick_yaw_sign"])
                for jk, conv in (("joystick_curve", float), ("joystick_scale", float)):
                    if jk in body and body[jk] is not None:
                        try:
                            eng.get("joystick", {})[jk.replace("joystick_", "")] = conv(body[jk])
                        except (TypeError, ValueError):
                            pass
                self.engine.cfg.save()
                self._json({"ok": True})
            else:
                self._json({"ok": False, "msg": "unknown config section"}, 400)
        else:
            self._json({"ok": False, "msg": "not found"}, 404)


def main():
    ap = argparse.ArgumentParser(description="Lite3 Web Pilot")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()

    cfg = Config()
    if args.port:
        cfg.data["http_port"] = args.port
    WIFI_CFG.update({k: v for k, v in cfg.data.get("wifi", {}).items()
                     if k in WIFI_CFG})

    engine = CommandEngine(cfg)
    camera = CameraWorker(cfg)
    engine._camera = camera
    Handler.engine = engine
    Handler.camera = camera

    threading.Thread(target=engine.tick_loop, daemon=True).start()

    httpd = ThreadingHTTPServer((args.host, cfg.data["http_port"]), Handler)
    host_ip = get_lan_ip()
    print("=" * 62)
    print("  Lite3 Web Pilot")
    print(f"  Robot        : {cfg.data['robot_ip']}:{cfg.data['cmd_port']} (udp)")
    print(f"  Telemetry    : udp :{cfg.data['state_port']}  (frame 0x0901)")
    print(f"  Web console  : http://localhost:{cfg.data['http_port']}")
    if host_ip:
        print(f"  On your LAN  : http://{host_ip}:{cfg.data['http_port']}")
    print("  Press Ctrl+C to quit.")
    print("=" * 62)
    try:
        faulthandler.enable()  # SIGABRT dumps every thread's python traceback
    except Exception:
        pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        engine.running = False
        camera.stop()
        engine.disconnect()
        httpd.server_close()


def get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


if __name__ == "__main__":
    main()
