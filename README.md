# lite3sdk · Lite3 Python Control SDK 🐕

Zero-dependency Python SDK to drive a **DeepRobotics Jueying Lite3** robot dog
straight over **UDP** — no ROS, no robot-side code, no pip installs. It uses
the same verified wire protocol as the **Lite3 Pilot** web console
(`lite3-web-pilot/` in this workspace).

| Channel | Direction | Port | Notes |
| --- | --- | --- | --- |
| Commands (UDP) | you → robot | `43893` | auto-mode `0x21010C03`, velocity `0x140`/`0x145`/`0x141` + f64, stand/sit/stop/heartbeat |
| Telemetry (UDP) | robot → you | `43897` | frame `0x0901`: battery, IMU rpy, velocities, charging, errors, ultrasound |

## Quick start

1. **Put your computer on the robot's Wi-Fi** (e.g. `YSC-JYML-XXXXX`,
   password from the robot's QR label). You land on `192.168.2.x`; the motion
   host is **`192.168.2.1`** (ethernet setups: `192.168.1.120`).

2. **Run from this folder** (stdlib only — `setup.sh` is optional):

   ```bash
   cd lite3_sdk
   python3 -m lite3sdk.cli status          # telemetry + battery after ~3 s
   python3 -m lite3sdk.cli stand
   python3 -m lite3sdk.cli hello           # stand + friendly wiggle
   python3 -m lite3sdk.cli drive --vx 0.4 --wz 0.3 --seconds 2
   python3 -m lite3sdk.cli stop
   python3 -m lite3sdk.cli estop --yes     # software E-stop latch
   python3 -m lite3sdk.cli reset-estop
   ```

3. **Or use it as a library:**

   ```python
   import lite3sdk

   dog = lite3sdk.Lite3("192.168.2.1")
   with dog:
       dog.stand()
       dog.drive(vx=0.4, wz=0.2)
       dog.wait(1.5)
       dog.stop()
       print(dog.state())
   ```

## Telemetry setup (one time, robot side)

Motion commands work without this. To *receive* telemetry, the robot's motion
host must stream to this computer's IP (default target is another address):

```bash
# from a computer on the robot Wi-Fi:
ssh ysc@192.168.2.1          # password: '  (a single quote)
cd ~/jy_exe/conf
nano network.toml            # set ip = '<your computer IP on 192.168.2.x>'
cd ~/jy_exe/scripts && sudo ./stop.sh && sudo ./restart.sh
```

Without it, construct with `allow_motion_without_telemetry=True` (the default)
or drive() is refused until a packet arrives.

## API at a glance

| Member | Meaning |
| --- | --- |
| `Lite3(ip, cmd_port, state_port, max_vx, max_wz, …)` | driver; `state_port=0` disables telemetry listening |
| `connect()` / `close()` / context manager | lifecycle (background pump + telemetry receiver start/stop) |
| `drive(vx, vy=0, wz=0)` | request velocity (clamped); kept alive `tail_ms`, then zeroed |
| `wait(seconds)` | hold the current request while blocking |
| `stop()` | official brake frames (value 0 + 1) |
| `estop()` / `reset_estop()` | software E-stop latch (1.5 s stop burst) |
| `stand()` / `sit(ms)` | pose actions |
| `hello()` | stand + yaw wiggle (blocking, E-stop aborts) |
| `send_raw(code, type, value, payload, repeats)` | arbitrary frame |
| `telemetry`, `state()`, `battery_pct`, `charging`, `height_m`, `wait_telemetry()` | telemetry access |
| `lite3sdk.protocol` | `build_frame`, `velocity_frame`, `parse_state`, frame constants |

## Wire protocol

Frame = 12-byte little-endian header `code / value / type` + optional payload.
Velocity frames: type 1, payload one f64, value field = 8. Axes: `0x140` x,
`0x145` y, `0x141` z (yaw). Actions: stand `0x21010202`, sit `0x21010203`,
stop `0x21010C0B` (values 0 and 1), heartbeat `0x21040001` (≥ 2 Hz), auto-mode
`0x21010C03`. Telemetry 0x0901 payload: see `lite3sdk/protocol.py`.

Reference: `DeepRoboticsLab/Lite3_MotionSDK`, `EzioPeter/Lite3_UDP`, and the
Lite3 Pilot console in this workspace.

## Files

```
lite3_sdk/
  lite3sdk/__init__.py   exports
  lite3sdk/protocol.py   frame builder + telemetry parser
  lite3sdk/robot.py      Lite3 driver (pump + receiver)
  lite3sdk/cli.py        python3 -m lite3sdk.cli
  examples/basic.py      scripted move
  examples/hello.py      minimal hello
  pyproject.toml         optional pip packaging (no deps)
  LICENSE                MIT
```

> ⚠️ **Safety.** Keep the dog leashed / on its stand and ≥ 5 m clear while
> testing. The software E-stop only stops *this* SDK sending frames — the
> **physical E-stop always wins**. The robot returns to its own damping
> controller ~1 s after the last UDP frame.
