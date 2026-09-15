# Lite3 Web Pilot 🐕

A local web console for the **DeepRobotics Jueying Lite3** robot dog:
live camera pane, joystick / keyboard driving, command buttons, telemetry
(battery, IMU, gait, errors) and a software E-stop.

Status readout covers everything the robot streams: roll/pitch/yaw, body
height, position, body/world velocities, acceleration, ultrasound, charging
and error/motion flags ("Live state" panel). Additional motion actions
configured in `config.json` render automatically as buttons next to
STAND/SIT.

It talks to the robot directly over **UDP** — no ROS, no robot-side code changes
needed for motion. Camera source is pluggable (see below).

---

## Quick start

1. **Put your computer on the robot's Wi-Fi.** The dog creates its own access
   point, e.g. `YSC-JYML-XXXXX` (the on-board computer user is `ysc`).
   Password: see the QR label on the robot.
   Once joined you should be on `192.168.2.x` — the robot (motion host) is at
   **`192.168.2.1`** (that's the gateway your phone showed).

2. **Run the server** (Python 3 stdlib only, no pip installs):
   ```bash
   ./run.sh
   # or: python3 server.py [--port 8123]
   ```
   It prints the URL — open **http://localhost:8123** on this computer, or
   `http://<this-computer-ip>:8123` from your phone/RETROID once they're on the
   same Wi-Fi.

3. Click **Connect**, watch the LED, then drive with the on-screen stick or
   **WASD / arrows**. **Space** or the red button = E-stop (hold it, then press
   "Reset E-stop" to re-arm).

> ⚠️ **Safety.** Keep the dog leashed / on its stand and ≥5 m clear while
> testing. The software E-stop only stops *this* tool sending frames — the
> **physical E-stop on the robot always wins**. The robot also returns control
> to its own damping controller if no UDP frames arrive for ~1 s.

---

## How it talks to the robot

| Channel | Direction | Port | Notes |
| --- | --- | --- | --- |
| Commands (UDP) | you → robot | `43893` | auto-mode `0x21010C03`, velocity `0x141` + m/s double (community-verified wire format) |
| Telemetry (UDP) | robot → you | `43897` | frame `0x0901`: battery, gait/motion state, rpy, velocities, charging, errors, ultrasound |

Reference: official `DeepRoboticsLab/Lite3_MotionSDK` (motion host = `192.168.2.1`
on network segment 2) and `EzioPeter/Lite3_UDP` (wire re-implementation).

### Telemetry needs one one-time robot-side setting
The motion host streams telemetry to the IP stored in its config (default
`192.168.1.102`). To receive battery/IMU here:

```bash
# from a computer on the robot Wi-Fi (IP 192.168.2.x):
ssh ysc@192.168.2.1        # password: '  (a single quote)
cd ~/jy_exe/conf
nano network.toml          # set ip = '<your computer IP on 192.168.2.x>'
# save, then restart the motion program:
cd ~/jy_exe/scripts
sudo ./stop.sh && sudo ./restart.sh
```

Motion commands work **without** this step. If you haven't done it, tick
**"allow motion w/o telemetry"** in the UI (or set
`allow_motion_without_telemetry: true` in `config.json`).

### Known robot quirk
Right after a fresh boot some Lite3 units ignore UDP until the official app
(or a ROS `/cmd_vel` publish on the perception host) has been used once — the
robot may need a quick "wake up" first. If nothing moves and the app works
fine, wake it with the app once, then come back to the web console.

---

## Camera

The Lite3 has **no public RTSP/MJPEG endpoint** — the app's video channel is
proprietary. So the console proxies any source you give it:

* ⚙ **Source** → paste a URL and pick kind:
  * `http_jpeg` — a plain `http://…/snapshot.jpg` endpoint (polled)
  * `ffmpeg` / `auto` — anything ffmpeg can open: `rtsp://…`, `http://…/stream.mjpg`, local file…
* The backend re-streams it to the browser as MJPEG at `/cam.mjpeg`.
* Options: `fps`, `scale` (e.g. `960:-1`), `quality`. Needs `ffmpeg` on the
  host only for non-JPEG sources.

Ideas for the Lite3 specifically:
* If your robot has the **Explorer** camera module, expose it on the perception
  host (ROS image topic → `ros2 run v4l2_camera`-style node or a small RTSP
  relay) and point the console at that.
* Or attach a USB/network camera and point the console at its stream.

---

## Preset commands & raw frames

The Lite3 app's full command table is not public and differs per firmware.
The console ships with **one verified datagram** (auto-mode + velocity driving).
Discrete poses (stand / sit / body up-down…) need **your** robot's codes:

1. Capture what the official app sends:
   ```bash
   # on a laptop on the robot Wi-Fi:
   sudo tcpdump -X -i en0 udp port 43893
   # or: tcpdump -x port 43893
   ```
   then trigger the pose in the app and read the frame hex: 12-byte header =
   `code` (4), `value/size` (4), `type` (4), optionally followed by a payload.
2. Add to `config.json` under `actions`, e.g.:
   ```json
   "stand_up": { "enabled": true, "note": "captured from app vX.Y",
     "frame": { "code": "0x21010102", "type": 0, "value": 0 } }
   ```
   (int-style command → use `value`; float payload → `"payload": "double"`;
   raw bytes → `"payload": "hex", "hex": "…"`.)
3. Restart the server — the button appears under **Motion**.

Or use **Advanced → raw frames** in the UI to type a frame and send it once
(12 repeats over ~0.6 s) without editing JSON.

---

## config.json reference

| Key | Default | Meaning |
| --- | --- | --- |
| `robot_ip` | `192.168.2.1` | motion host (ethernet setups use `192.168.1.120`) |
| `cmd_port` / `state_port` | `43893` / `43897` | UDP ports |
| `limits.max_vx` | `0.5` | max forward speed m/s |
| `limits.max_wz` | `0.8` | max yaw (unused until a turn frame is wired up) |
| `limits.tail_ms` | `300` | zero-frames sent after you release the stick |
| `velocity` / `auto_mode` | — | frames used while driving |
| `actions` | — | named preset frames (buttons in the UI) |
| `camera` | — | source URL/kind/fps/scale |
| `allow_motion_without_telemetry` | `false` | skip the "telemetry seen recently" gate |

## Files

```
server.py          backend: UDP driver, telemetry parser, camera proxy, HTTP
static/index.html  console UI
static/style.css
static/app.js
config.json        robot + camera + command presets
run.sh             launcher
```

---

## Verified protocol & app notes (Sep 2026)

Motion host: **192.168.2.1** on the robot's Wi-Fi (`YSC-JYML-…` hotspot),
**192.168.1.120** on ethernet. Commands -> UDP **43893**, telemetry <- UDP **43897**.

| Command | Code | Notes |
|---|---|---|
| Posture **STAND/SIT** | `0x21010202` | **one toggle** for both (stand <-> sit) - never blind-send twice |
| Auto/navigation mode | `0x21010C03` | streamed as keep-alive |
| Manual/remote mode | `0x21010C02` | experimental |
| Velocity x / y / z | `0x140` / `0x145` / `0x141` | type 1, f64 payload, value field = 8 |
| Stop / brake | `0x21010C0B` | send value `0` **and** `1` |
| Heartbeat | `0x21040001` | >= 2 Hz expected |
| Hello pose | `0x21010506` | plays **from sitting** (verified) |
| Dance / moonwalk | `0x2101030C` | verified |
| Twist / twist jump | `0x21010204` / `0x2101020D` | standing |
| Turn over / backflip / long jump | `0x21010205` / `0x21010502` / `0x2101050B` | sitting; **not yet verified on every firmware** - capture from the official remote if a code is ignored |
| Gaits slow/medium/fast | `0x21010300` / `0x21010307` / `0x21010303` | experimental |
| Camera driver ON/OFF | `0x21012109` value `0x40` / `0` -> UDP 43899 | starts the robot's camera services |
| Camera/AI service query | `0x2101210D` | replies `0x11` active / `0x10` inactive |

**Joystick** — two on-screen pads (left: forward/back + turn, right: slide sideways),
spring-centered (release = stop), speed curve + scale in Advanced, and an optional
**official Retroid mode** that sends the same 42-byte `0x55 0x66` frames the physical
remote uses (UDP -> robot **12121**; axes `left_x/left_y/right_x/right_y`, range ±1000).

**Camera gotcha (ffmpeg 8):** `-stimeout` / `-rw_timeout` were removed in ffmpeg 8 —
passing them makes ffmpeg exit instantly and the feed dies silently. The console now
omits them, plus runs a camera watchdog that re-sends the robot's camera-ON command
and reopens the stream whenever it drops.

**Telemetry** (battery/IMU) needs a one-time robot-side edit: set `ip = '<your Mac IP>'`
in `~/jy_exe/conf/network.toml` and restart the motion program.

**Companion SDK:** the Python control SDK lives at
https://github.com/infodriver/lite3-sdk (`lite3sdk` package + CLI).

**Diagnostics tools:** `tools/retroid_listen.py` decodes official remote frames
(point the RETROID controlapp at this Mac's IP:12121) - use it to capture the real
codes for any action this firmware ignores.
