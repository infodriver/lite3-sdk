"""Lite3 robot driver: UDP command pump + telemetry receiver.

Mirrors the verified behaviour of the Lite3 Pilot web console:

* while connected, a background pump sends the heartbeat frame every 300 ms
  (the motion host expects >= 2 Hz) and re-arms auto-mode once per second so
  the first velocity frame is always accepted;
* velocity requests are sent at up to 50 Hz and kept alive for ``tail_ms``
  after the last :meth:`Lite3.drive` call, then zeroed;
* actions (stand/sit/stop/hello/raw) are burst at the tick rate;
* E-stop latches: it stops everything and pushes the official stop frames
  (0x21010C0B with value 0 and 1) for 1.5 s; nothing moves again until
  :meth:`Lite3.reset_estop`.
"""

import socket
import struct
import threading
import time

from . import protocol as P

DEFAULT_IP = "192.168.2.1"  # robot motion host on the robot's own Wi-Fi


class MotionError(RuntimeError):
    """Raised when a motion call is refused (not connected / E-stop latched)."""


class Lite3:
    """Control a DeepRobotics Jueying Lite3 over UDP.

    Args:
        robot_ip: motion host (192.168.2.1 on the robot Wi-Fi, 192.168.1.120
            on ethernet setups).
        cmd_port / state_port: UDP ports (43893 / 43897 defaults).
        bind_state: local address to bind the telemetry socket on (default
            "0.0.0.0"). Pass ``state_port=0`` to disable telemetry listening.
        max_vx / max_vy / max_wz: speed clamps in m/s and rad/s.
        rate_hz: velocity frame rate while driving.
        tail_ms: keep sending the last velocity this long after drive() ends.
        heartbeat_ms: keep-alive interval (motion host expects >= 2 Hz).
        allow_motion_without_telemetry: if False, drive() is refused until a
            0x0901 telemetry packet has arrived recently (needs network.toml
            on the robot pointing at this computer - see README).
    """

    def __init__(self, robot_ip=DEFAULT_IP, cmd_port=P.CMD_PORT, state_port=P.STATE_PORT,
                 bind_state="0.0.0.0", max_vx=0.5, max_vy=0.4, max_wz=0.8,
                 rate_hz=50, tail_ms=300, heartbeat_ms=300,
                 allow_motion_without_telemetry=True, camera_port=P.CAMERA_PORT):
        self.robot_ip = robot_ip
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.bind_state = bind_state
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_wz = float(max_wz)
        self.rate_hz = max(5.0, float(rate_hz))
        self.tail_ms = int(tail_ms)
        self.heartbeat_ms = int(heartbeat_ms)
        self.allow_motion_without_telemetry = bool(allow_motion_without_telemetry)
        self.camera_port = int(camera_port)
        self.yaw_sign = -1.0  # matches console calibration; flip if reversed

        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._estop = False
        self.posture = None      # tracked stand/sit (None = unknown until first toggle)
        self._cmd_sock = None
        self._state_sock = None
        self._desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}
        self._burst = []
        self._burst_until = 0.0
        self._last_hb = 0.0
        self._last_auto = 0.0

        self.telemetry = None       # latest decoded 0x0901 dict (or None)
        self._telemetry_at = 0.0
        self.joint_state = None     # latest 0x0902 joint-state stream (12 f64)
        self.handle_state = None    # latest 0x0905 remote/handle stream
        self.frame_count = 0
        self.last_code = None
        self._hb_seq = 0

    # ------------------------------------------------------------ lifecycle
    @property
    def connected(self):
        return self._connected

    @property
    def estop(self):
        return self._estop

    def connect(self):
        """Open sockets, bind the telemetry receiver and start the pump."""
        with self._lock:
            if self._connected:
                return self
            self._running = True
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cmd_sock.settimeout(1)
            if self.state_port:
                self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._state_sock.bind((self.bind_state, self.state_port))
                self._state_sock.settimeout(0.5)
                threading.Thread(target=self._recv_loop, daemon=True).start()
            threading.Thread(target=self._pump_loop, daemon=True).start()
            self._connected = True
            self._estop = False
        return self

    def close(self):
        """Close sockets and stop background threads."""
        with self._lock:
            self._running = False
            self._connected = False
            for s in (self._state_sock, self._cmd_sock):
                if s is not None:
                    try:
                        s.close()
                    except OSError:
                        pass
            self._state_sock = None
            self._cmd_sock = None
            self._burst = []
            self._desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}
            self.telemetry = None
            self.joint_state = None
            self.handle_state = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------- telemetry
    @property
    def telemetry_age(self):
        """Seconds since the last telemetry packet, or None."""
        if self.telemetry is None:
            return None
        return time.monotonic() - self._telemetry_at

    @property
    def battery_pct(self):
        t = self.telemetry
        return round(t["battery"] * 100.0, 1) if t else None

    @property
    def charging(self):
        t = self.telemetry
        return bool(t["charging"]) if t else None

    @property
    def height_m(self):
        t = self.telemetry
        return round(t["pos"][2], 3) if t else None

    def wait_telemetry(self, timeout=10.0):
        """Block until a fresh telemetry packet arrives. Returns it or None."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            t = self.telemetry
            if t is not None:
                return t
            time.sleep(0.1)
        return None

    def _recv_loop(self):
        while self._running:
            sock = self._state_sock
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
                code = struct.unpack_from("<i", data, 0)[0]
                self.last_code = code
                if code == 0x0901:
                    parsed = P.parse_state(data)  # exact-size (212 B) guard inside
                    if parsed is not None:
                        self.telemetry = parsed
                        self._telemetry_at = time.monotonic()
                elif code == 0x0902:
                    self.joint_state = P.parse_joint_state(data)
                elif code == 0x0905:
                    self.handle_state = P.parse_handle_state(data)

    # -------------------------------------------------------------- commands
    def _require_ready(self):
        with self._lock:
            if not self._connected:
                raise MotionError("not connected - call connect() first")
            if self._estop:
                raise MotionError("E-STOP latched - call reset_estop() first")

    def _telemetry_recent(self, window=5.0):
        if self.telemetry is None:
            return False
        return (time.monotonic() - self._telemetry_at) < window

    def _send(self, frame):
        try:
            self._cmd_sock.sendto(frame, (self.robot_ip, self.cmd_port))
        except OSError:
            pass

    def _burst_frames(self, frames, ms=None):
        with self._lock:
            if not self._connected or self._estop:
                return False
            self._burst = frames
            self._burst_until = time.monotonic() + (ms or 600) / 1000.0
            return True

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        """Request body velocity: vx forward m/s, vy lateral m/s, wz yaw rad/s.

        Values are clamped to the configured maxima. The pump keeps sending
        them (at up to rate_hz) for ``tail_ms`` after the last call.
        """
        self._require_ready()
        if not self._telemetry_recent() and not self.allow_motion_without_telemetry:
            raise MotionError(
                "no telemetry from the robot - open UDP 43897 / set network.toml, "
                "or construct with allow_motion_without_telemetry=True")
        vx = max(-self.max_vx, min(self.max_vx, float(vx)))
        vy = max(-self.max_vy, min(self.max_vy, float(vy)))
        wz = max(-self.max_wz, min(self.max_wz, float(wz)))
        with self._lock:
            self._desired = {"vx": vx, "vy": vy, "wz": wz, "ts": time.monotonic()}

    def wait(self, seconds):
        """Keep the current velocity request alive for ``seconds`` (blocks).

        Note: the pump itself already tails the request for ``tail_ms``; use
        wait() when you want a timed move and the drive() call happens first.
        """
        time.sleep(max(0.0, seconds))

    def stop(self):
        """Brake: send the official action-stop frames (value 0 and 1)."""
        self._require_ready()
        frames = [P.build_frame(P.FRAME_STOP, value=v) for v in (0, 1)]
        self._burst_frames(frames, ms=400)
        with self._lock:
            self._desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}

    def estop(self):
        """Software E-stop: latch + push stop frames for 1.5 s. Physical
        E-stop on the robot always wins; reset with reset_estop()."""
        self._require_ready()
        with self._lock:
            self._estop = True
            self.posture = None  # e-stop can drop/sit the dog - state unknown now
            self._desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": 0.0}
            frames = [P.build_frame(P.FRAME_STOP, value=v) for v in (0, 1)]
            self._burst = frames
            self._burst_until = time.monotonic() + 1.5

    def reset_estop(self):
        with self._lock:
            self._estop = False
            self._burst = []

    def _send_posture(self, want, ms=900):
        """0x21010202 is a stand<->sit TOGGLE on the Lite3 - send it only when
        the tracked posture differs (None = unknown: send and assume)."""
        self._require_ready()
        with self._lock:
            if self._estop:
                raise MotionError("E-STOP latched - call reset_estop() first")
            if self.posture == want:
                return False
            self._burst = [P.build_frame(P.FRAME_STAND_SIT)]
            self._burst_until = time.monotonic() + ms / 1000.0
            self.posture = want
            return True

    def stand(self, ms=900):
        """Stand up (toggle-safe: no-op if already standing per tracked state)."""
        self._send_posture("stand", ms)

    def sit(self, ms=900):
        """Sit down (toggle-safe: no-op if already sitting per tracked state)."""
        self._send_posture("sit", ms)

    def hello(self):
        """Official 'hello' pose (0x21010506), which plays from the SITTING
        posture: sit first if needed, then re-send the pose at ~1 Hz. Blocking;
        aborts early if estop() is called."""
        return self.action("hello")

    # ------------------------------------------------------------- actions
    @staticmethod
    def list_actions():
        """All named actions with their codes/notes (same catalog as the app)."""
        return {k: dict(v) for k, v in P.ACTIONS.items()}

    def action(self, name):
        """Play any catalog action by name (see list_actions()).

        Posture-aware like the app: sit/stand first when needed (and force it
        for actions that require a specific pose), then re-send the action at
        ~1 Hz; mode/gait switches are single-shot. Blocking, E-stop aborts.
        """
        spec = P.ACTIONS.get(name)
        if spec is None:
            raise ValueError("unknown action '%s' (see list_actions())" % name)
        self._require_ready()
        if spec.get("toggle"):
            return self._send_posture("stand" if name == "stand_up" else "sit")
        post = spec.get("posture")
        if post:
            cur = self.posture
            if cur is None and spec.get("force"):
                self._send_posture(post)
                self._settle(2.4)
            elif cur is not None and cur != post:
                self._send_posture(post)
                self._settle(2.4)
            if self._estop:
                return False
            self._settle(0.4)
        frame = P.build_frame(spec["code"])
        if spec.get("once"):
            self._send(frame)
            return True
        for _ in range(3):
            if self._estop:
                return False
            self._send(frame)
            if not self._settle(1.0):
                return False
        return True

    def _settle(self, seconds):
        """Sleep while remaining cancellable by estop(); False if cancelled."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._estop:
                return False
            time.sleep(0.05)
        return True

    # --- convenience wrappers: dog.dance(), dog.backflip(), ... ---
    def dance(self):        return self.action("dance")
    def twist(self):        return self.action("twist")
    def twist_jump(self):   return self.action("twist_jump")
    def turn_over(self):    return self.action("turn_over")
    def backflip(self):     return self.action("backflip")
    def long_jump(self):    return self.action("long_jump")
    def recover_left(self):  return self.action("recover_left")
    def recover_right(self): return self.action("recover_right")
    def mode_manual(self):  return self.action("mode_manual")
    def mode_move(self):    return self.action("mode_move")
    def gait_slow(self):    return self.action("gait_slow")
    def gait_medium(self):  return self.action("gait_medium")
    def gait_fast(self):    return self.action("gait_fast")
    def gait_crawl(self):   return self.action("gait_crawl")

    def send_raw(self, code, typ=0, value=0, payload=None, repeats=1, interval=0.0):
        """Send one raw frame (see protocol.build_frame). Burst it with
        repeats/interval if wanted. Returns the frame bytes."""
        self._require_ready()
        frame = P.build_frame(code, typ=typ, value=value, payload=payload)
        if repeats > 1:
            self._burst_frames([frame], ms=max(50, int(repeats * interval * 1000)))
            for _ in range(repeats - 1):
                if self._estop:
                    break
                time.sleep(interval)
        else:
            self._send(frame)
        return frame

    # ---------------------------------------------------------------- pump
    def _pump_loop(self):
        interval = 1.0 / self.rate_hz
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                if not self._connected:
                    break
                if self._estop:
                    if self._burst and t0 < self._burst_until:
                        for fb in self._burst:
                            self._send(fb)
                    else:
                        self._burst = []
                    time.sleep(0.02)
                    continue
                # heartbeat keep-alive (motion host expects >= 2 Hz)
                if t0 - self._last_hb >= self.heartbeat_ms / 1000.0:
                    self._last_hb = t0
                    # official robot-side monitor wants a monotonic counter value
                    # (0 is skipped) - Lite3_VMC parse_cmd semantics
                    self._hb_seq = (self._hb_seq + 1) & 0xFFFFFFFF
                    self._send(P.build_frame(P.FRAME_HEARTBEAT, value=self._hb_seq))
                # action/stop burst takes priority
                if self._burst and t0 < self._burst_until:
                    for fb in self._burst:
                        self._send(fb)
                else:
                    self._burst = []
                    d = self._desired
                    age_ms = (t0 - d["ts"]) * 1000.0
                    if age_ms < self.tail_ms:
                        self._send(P.velocity_frame(P.VEL_X, d["vx"]))
                        self._send(P.velocity_frame(P.VEL_Y, d["vy"]))
                        self._send(P.velocity_frame(P.VEL_Z, d["wz"] * self.yaw_sign))
                        self._send(P.build_frame(P.FRAME_AUTO_MODE))
                    elif d["vx"] or d["vy"] or d["wz"]:
                        self._desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "ts": t0}
                # idle keep-alive: stay in auto mode so input is always accepted
                if t0 - self._last_auto >= 1.0:
                    self._last_auto = t0
                    self._send(P.build_frame(P.FRAME_AUTO_MODE))
            dt = time.monotonic() - t0
            if interval - dt > 0:
                time.sleep(interval - dt)

    # --------------------------------------------------------------- camera
    @property
    def camera_url(self):
        """RTSP URL of the robot camera (open with ffmpeg/VLC/OpenCV)."""
        return "rtsp://%s:%d%s" % (self.robot_ip, P.CAMERA_RTSP_PORT,
                                   P.CAMERA_RTSP_PATH)

    def _send_app(self, code, value, repeats=3, interval=0.15):
        """Send AppSimpleCMD frames to the robot's app-service port."""
        with self._lock:
            if not self._connected:
                raise MotionError("not connected - call connect() first")
        sent = 0
        for _ in range(max(1, repeats)):
            try:
                self._cmd_sock.sendto(
                    struct.pack("<III", code & 0xFFFFFFFF, value & 0xFFFFFFFF, 0),
                    (self.robot_ip, self.camera_port))
                sent += 1
            except OSError:
                pass
            time.sleep(interval)
        return sent

    def camera_on(self, repeats=3):
        """Start the robot's camera services (then stream appears on camera_url)."""
        n = self._send_app(P.FRAME_CAMERA_ON, P.CAMERA_VALUE_ON, repeats)
        return {"ok": n > 0, "sent": n, "url": self.camera_url}

    def camera_off(self, repeats=2):
        """Stop the robot's camera services."""
        n = self._send_app(P.FRAME_CAMERA_OFF, 0, repeats)
        return {"ok": n > 0, "sent": n}

    def camera_state(self, timeout=2.0):
        """Query the camera/AI service state (0x2101210D).
        Returns {"active": bool|None, "reply": hex|None}."""
        with self._lock:
            if not self._connected:
                raise MotionError("not connected - call connect() first")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((self.bind_state, 0))
            s.settimeout(timeout)
            s.sendto(struct.pack("<III", P.FRAME_CAMERA_QUERY, 0, 0),
                     (self.robot_ip, self.camera_port))
            data, _ = s.recvfrom(4096)
            code, value, _t = struct.unpack_from("<III", data, 0)
            return {"active": value == 0x11, "reply": "0x%08X value=0x%X" % (code, value)}
        except socket.timeout:
            return {"active": None, "reply": None}
        except OSError as exc:
            return {"active": None, "reply": "err: %s" % exc}
        finally:
            s.close()

    # --------------------------------------------------------------- status
    def state(self):
        """Snapshot dict: connectivity, e-stop, latest telemetry, counters."""
        with self._lock:
            return {
                "connected": self._connected,
                "robot_ip": self.robot_ip,
                "cmd_port": self.cmd_port,
                "state_port": self.state_port,
                "estop": self._estop,
                "telemetry_age_ms": int((time.monotonic() - self._telemetry_at) * 1000)
                if self.telemetry is not None else None,
                "telemetry": self.telemetry,
                "joints": self.joint_state,
                "remote": self.handle_state,
                "battery_pct": self.battery_pct,
                "charging": self.charging,
                "height_m": self.height_m,
                "frame_count": self.frame_count,
                "last_frame_code": ("0x%08X" % self.last_code)
                if self.last_code is not None else None,
            }
