# DeepRoboticsLab research for the Lite3 stack — 2026-09-06

Read-only survey of the DeepRoboticsLab GitHub org done 2026-09-06. Source of
truth: shallow clones in `/tmp/dr-research/<repo>` (all commits/paths/lines
below cite those clones; repos are `github.com/DeepRoboticsLab/<name>`).
Nothing in our stack was modified.

## 1) Summary verdict

**Our console+SDK wire model is confirmed correct by the official repos (ports
43893/43897/43899/12121, 12-byte `code|value|type` header, f64 velocity codes
0x140/0x145/0x141, 0x0901 state frame, 42-byte Retroid joystick frames,
`~/jy_exe/conf/network.toml` telemetry target) — no frame code we currently use
is contradicted, but the official sources add: 3 new pose/action codes
(dance/hello/talent from Lite3_LLM), an extra `robot_policy_state` field in the
official 0x0901 layout (RL-firmware variant — ours is 8 B shorter), a
monotonic-counter requirement for the 0x21040001 heartbeat value, two
additional streamed frames we ignore (0x0902 joint state, 0x0905 handle/goal
state), and a service-state query code 0x2101210D. The new "unified"
deep-robotics-sdk2 is DR02 (humanoid-ish) ROS 2 only — **not Lite3, nothing to
port**.**

## 2) Repo table

| Repo | Relevance | What we should take |
|---|---|---|
| Lite3_ROS | HIGH — official Jetson⇄robot UDP bridge (latest commit "适配RL版本") | velocity codes+sign conventions confirmed; 0x0902/0x0905 stream frames to add; official RobotState layout (w/ robot_policy_state) to diff vs our 200 B struct; 0x21012109 semantics + 0x2101210D query; joystick-frame spec proof |
| Lite3_LLM | HIGH — smallest official action-code table for Lite3 | 3 new single-value action codes: dance 0x2101030C, hello 0x21010506, talent 0x21010204 |
| Lite3_VMC | MED — research MPC controller, but bundles robot-side SDK receive code | heartbeat value must be monotonic counter (0 skipped; >250 ms gap = timeout); remote analog codes 0x21010130/131/135 (value = int16/32768) corroborate our "rotate 0x21010135" community note; JOINT_* low-level codes 0x0902/03/04 (note 0x0902 dual meaning) |
| Lite3_MotionSDK | MED — low-level 1 kHz joint PD SDK (not our protocol level) | authoritative network.toml layout + name.toml `standard_1/2` firmware knob + Deeprcs version gate; 12-byte header + type:8/count:24 field semantics; 1 s low-level command timeout → damping |
| gamepad | HIGH for joystick — official Retroid(Lite3)/Skydroid(X30) receiver + controlapp.apk | exact 42 B Retroid frame + ±1000 range + sum-checksum spec (we already match); dev-host listener pattern on :12121 worth porting so a real remote can drive us |
| robotserver_sdk | LOW — X30 Pro navigation host TCP/XML robotserver (192.168.1.106), not Lite3/UDP | error-code taxonomy as a documentation pattern only; nothing portable |
| deep-robotics-sdk2 | LOW for us — "new unified SDK" is DR02 Pro/Std ROS 2 + MuJoCo, pushed 2026-08-28; no Lite3, no UDP wire | conceptual: named action tables + "current action" query + explicit state machine (food for our config.json actions design, no codes to port) |
| deep-robotics-msg | LOW — ROS 2 msg/srv for the DR02 SDK (`drdds`), no Lite3 wire content | pattern: StdStatus = state+error_code, FaultEvent snapshot (mirrors our error_state int); nothing portable |
| deep-robotics-teleoperate | NONE — DR02 Pro humanoid PICO-XR teleop (VR), not Lite3 | nothing |

## 3) Detailed findings per repo

### 3.1 Lite3_ROS  (HEAD e4ab5dc, 2026-02-05, last commit msg: 适配RL版本 = "adapted for RL version")

ROS 2 package `transfer` bridging the perception host to the Lite3 motion host
and to the phone app. Path prefix below = `Lite3_ROS/src/transfer/`.

**Ports (launch/transfer_launch.py:10,19-20)** — matches our model exactly:
`jetson2app` local_port 43899; `jetson2motion` target_ip 192.168.1.120,
target_port 43893 (motion host), local_port 43897 (telemetry listen).

**Velocity commands (src/Jetson2Motion.cpp:356,371,383)** — decimal, then hex:
`cmd_code = 320` = **0x140** linear.x, `325` = **0x145** linear.y, `321` =
**0x141** angular.z, each `size = 8; type = 1; data = double`. This is exactly
our verified f64 velocity triplet. Sign conventions to note:
- yaw is negated before send: `-1 * msg->angular.z` (Jetson2Motion.cpp:383-385);
  ROS Twist positive angular.z = turn left, so the raw wire value for 0x141 is
  the right-hand/opposite sign (consistent with our `yaw_negate: True` /
  joystick `yaw_sign: -1` defaults).
- published /cmd_vel example rate is 10 Hz (README.md "ros2 topic pub -r 10").

**Telemetry frames decoded on :43897 (Jetson2Motion.cpp)** — dispatch is by
payload length (`ParseFrame` switch on `sizeof(...Received)`), codes checked
after cast:
- `dr->code == 2305` = **0x0901 robot state** (:131) → topics /leg_odom,
  /leg_odom2, /imu/data, ultrasound.
- `dr->code == 2306` = **0x0902 joint state** (:199) → /joint_states,
  struct = 12 doubles LF_Joint..RB_Joint_2, order LF RF LB RB (3 joints each).
  **All 12 values are negated** in the ROS mapping (:207-224: `position[i] =
  -joint_state->...`) — treat official "joint angles" as sign-flipped vs the
  raw wire value.
- `dr->code == 2309` = **0x0905 handle/gamepad state** (:240) → /handle_state
  Twist: linear.x = left_axis_forward, linear.y = left_axis_side,
  angular.z = **-**right_axis_yaw. Struct = 6 doubles: left_axis_forward,
  left_axis_side, right_axis_yaw, goal_vel_forward, goal_vel_side,
  goal_vel_yaw (include/protocol.hpp:60-78) — i.e. the robot re-streams the
  physical remote sticks AND the resulting goal velocities. **We currently
  drop both 0x0902 and 0x0905** (our receive loop only parses 0x0901).
- `dr->code == 0x010901` (:254) = low-level IMU frame (timestamp + 9 floats,
  angles in degrees) — used by the low-level flow; not needed for us.

**Official RobotState layout — diff vs our 0x0901 struct**
(include/protocol.hpp:29-48, struct RobotState):
```cpp
int   robot_basic_state;   // @0
int   robot_gait_state;    // @4
int   robot_policy_state;  // @8   <-- NOT in our parse
double rpy[3]; double rpy_vel[3]; double xyz_acc[3];
double pos_world[3]; double vel_world[3]; double vel_body[3];
unsigned touch_down_and_stair_trot; bool is_charging;
unsigned error_state; int robot_motion_state; double battery_level;
int task_state; bool is_robot_need_move; bool zero_position_flag;
double ultrasound[2];
```
Field order and names otherwise match our parse
(`STATE_STRUCT = "<ii" + "3d"*6 + "IB3xIid" + "iBB2x2d"` = 200 B payload +
12 B header, server.py:128). The difference is exactly the extra
`int robot_policy_state` (+4 B) after gait_state. Compiled sizes (macOS arm64,
checked with a standalone repro): default-aligned RobotState = **208 B** data
(220 B with header), `#pragma pack(4)` variant = 204 B data (216 B total);
our verified stock-firmware frame = **200 B payload / 212 B total**. The ROS
node only parses frames whose length equals `sizeof(RobotStateReceived)`, so it
expects the larger (RL-era) layout; our dog's firmware streams the 200 B
variant and our parse is correct for it. **Flag**: the two layouts are
firmware-version dependent; if we ever update the dog's motion firmware to the
RL variant ("适配RL版本"), the state frame grows and every trailing field
shifts. (Related firmware knob: `name.toml standard_1/standard_2`, §3.4.)

**App command port 43899 (src/Jetson2App.cpp)** — the phone app talks to the
perception host here with 12 B AppSimpleCMD (`cmd_code, cmd_value, type`):
- `case 0x21012109` (:141): `cmd_value == 0x40` starts the robot-side AI
  services (`systemctl start realsense_ros2.service` + `voa_ros2.service`,
  :152-155) **only if the IMU topic is alive** (:150 gating); `cmd_value ==
  0x00` stops both (:163-168). This is the frame we use for "camera driver
  power" — official semantics are broader (Realsense + obstacle-avoidance), so
  our 0x21012109=0x40 is also what makes RTSP /test appear.
- `case 0x2101210D` (:174-196): **query AI-service state**; the host replies to
  the requester with the same code, value **0x11 = active / 0x10 = inactive**.
  We don't use this yet — it is a cheaper camera/AI-ready check than polling
  RTSP.

**Joystick frame spec (include/protocol.hpp:103-149 + Jetson2App.cpp:208-250)** —
the app (Retroid handheld, `ControllerType::kRetroid = 1`, protocol.hpp:111)
streams 42-byte frames: header `stx {0x55,0x66}` + ctrl u8 + `data_len` u16
(=32) + seq u16 + id u8 + `checksum` u16; payload = 10×u16 buttons +
4×i16 axes (left_axis_x/y, right_axis_x/y) + 2×u16 axis buttons.
- Checksum is a **byte sum of the 32 payload bytes**, not CRC-16
  (Jetson2App.cpp:219-224; the `checksum` comment says "CRC-16" but the code
  sums bytes).
- Axes are normalized /1000 (kJoystickRange=1000); D-pad up/down/left/right is
  synthesized from the left stick at ±1000 extremes (:227-238).
- Button bit order (protocol.hpp:147+): R1,L1,start,select,R2,L2,A,B,X,Y,
  left,right,up,down,left_axis_button,right_axis_button (LSB-first u16).
- **Our console already emits this exact format** (server.py `_joy_frame`,
  `struct.pack("<10Hhhhh2H")` + `<2sBHHBH` header, sum checksum) and sends it
  to robot :12121. Verified match, no change needed.

### 3.2 Lite3_LLM  (HEAD 6af0356, 2025-03-10)

ROS 1 node + python demo that turns an LLM's answer into single-value motion
commands (12 B header, `paramters_size = 0, type = 0`, sent to 192.168.1.120:
43893):

| LLM task (llm_demo.py system prompt :174) | control# | code (llm_control.cpp:61,64,67) |
|---|---|---|
| 跳舞 dance | 6 | **0x2101030C** |
| 你好 hello | 7 | **0x21010506** |
| 表演才艺 show a talent | 8 | **0x21010204** |

All three are empty-payload type-0 frames, same shape as our stand/sit
(0x21010202/0x21010203); the node repeats the code at 1 Hz while engaged.
0x21010204 sits in the same 0x210102xx posture family as our verified
stand/sit. These are the only official Lite3 pose/action codes we found in any
repo — a natural extension for our config.json `actions` (safety: dance/hello
move the dog; test on a stand).

### 3.3 Lite3_VMC  (HEAD 724aa54, 2024-09-16)

MPC "virtual model control" research stack for Lite3; bundles the robot-side
low-level SDK (`src/quadruped/extern/deeprobotics_legged_sdk`).

**Heartbeat semantics (…/src/parse_cmd.cpp:116-144)** — robot-side receive of
**0x21040001** single-value commands on :43897:
```cpp
case 0x21040001:
    value = (uint32_t)nc.get_command_value();
    ...
    if(value != 0 && value != last_value + 1){ cout << "error  drop"; drop_count++; }
    ...
    if(value == 0){ break; }                 // value 0 is skipped, not counted
    ...
    if(dt_time > 250){ time_out_count++; }   // inter-heartbeat gap >250 ms = timeout
```
i.e. the official robot-side monitor expects the heartbeat **value to be a
monotonically increasing counter starting at 1** and flags gaps > 250 ms.
Our console sends a static `value: 0` every 300 ms (config.json heartbeat +
server.py:1102-1106). This is fine for the high-level app-mode watchdog we've
verified, but it diverges from official semantics — recommend sending an
incrementing counter (and, where a low-level/SDK watchdog is in play, staying
well under 250 ms).

**Remote analog command codes (src/quadruped/src/controllers/qr_desired_state_command.cpp:68-75,113-136)** —
a 12-byte 3×u32 LE (code/value/type) stream on :43892 that this controller
treats as the official remote's motion commands:
- **0x21010130** → vx, **0x21010131** → vy, **0x21010135** → yaw rate, each
  `value/32768.0 × MAX_*` (int16-scaled analog, :114/:123/:131; yaw negated).
- 0x21010201 = stop/MPC stance (B), 0x21010307 = exit/sit down (Y),
  0x21010402 / 0x21010406 = body up/down (shoulder keys), 0x21010300 = gait
  toggle.
This corroborates the "rotate 0x21010135" line in our server.py header comment
(which came from the EzioPeter/Lite3_UDP community protocol): 0x21010135 is the
official remote yaw channel, distinct from the SDK's 0x141+f64 yaw. We do not
need to add it — but it confirms both velocity families exist on the robot.

**Low-level joint command codes (…/include/parse_cmd.h:32-34)**:
`JOINT_POS_CMD 0x0902`, `JOINT_VEL_CMD 0x0903`, `JOINT_TOR_CMD 0x0904`
(received as kMessValues); parse_cmd.cpp:92 also handles `case 0x0906`
(RobotData update — the low-level telemetry code per Lite3_MotionSDK changelog)
and `case 600` (0x258) = vision "send_pose" payload with a stamp field.
**Quirk worth knowing**: 0x0902 has two meanings on the wire — a *streamed*
joint-state frame (Lite3_ROS §3.1, code 2306) and a *command* to write joint
positions (low-level SDK). Context (port + direction) disambiguates; verify
empirically before relying on either.

### 3.4 Lite3_MotionSDK  (HEAD b30a3ec, 2026-01-16)

The official C++ **low-level** SDK (precompiled `.so` + eigen). Not the
protocol level we speak (we are high-level: auto-mode/actions/velocity frames).
It sends per-joint `pos, vel, kp, kd, t_ff` RobotCmd at 1 kHz and receives
RobotData; control law `T = kp·(pos_goal−pos_real) + kd·(vel_goal−vel_real) +
t_ff` (README.md §2). Header format (include/common/command.h) = the same
12-byte envelope we pack: `code u32 | value/params_size u32 | type:8 +
count:24 u32` with `type 0 = single value, 1 = multiple values`.

The README is the best official ops documentation we found:
- **network.toml confirmed** (§5, README.md:122-130):
  ```toml
  ip = '192.168.1.102'  # Motion host will send data to this IP address
  target_port = 43897
  local_port = 43893
  ```
  Exactly our model: the motion host streams telemetry to `ip:43897` and
  listens for commands on `local_port 43893` (file lives at
  `~/jy_exe/conf/network.toml` on the robot). This is the setting our README
  already tells users to change.
- Motion-host addresses (§4): WiFi segment 1 → 192.168.1.120, segment 2 →
  192.168.2.1; ethernet → 192.168.1.120. Default ssh `ysc` / password `'`
  (single quote); fallbacks `user/123456`, `firefly/firefly`.
- **Firmware model knob** (§5, :136-160): after `cd ~/jy_exe/scripts; ./stop.sh;
  ./run.sh` check the "Deeprcs Version"; if it is older than 1.4.24(97) (user)
  or 1.4.21(94) (firefly), edit `~/jy_exe/conf/name.toml` from
  `name = standard_1` to `name = standard_2`, then restart with
  `sudo ./stop.sh; sudo ./restart.sh` under `~/jy_exe`. This is the
  firmware/model variant switch and is very likely the same axis as the
  RL-variant state frame in §3.1. Worth checking on our dog (see §4 rec #6).
- Low-level watchdog (§2, :55): "When the underlying controller has not
  received the commands from SDK for more than 1 second, it will retrieve
  control right, enter a damping protection mode…" — relevant only if we ever
  drive joints directly (lite3_sdk is high-level, so N/A today).
- Troubleshooting (§7.2): `sudo tcpdump -x port 43897 -i lo` (SDK on motion
  host) / `-i p2p0` (wifi dev host) / `-i eth1` (user) / `-i eth0` (firefly);
  restart motion program with `cd ~/jy_exe; sudo ./stop.sh; sudo ./restart.sh`;
  check process `jy_exe`.
- Demo caution: robot must be in its ready position before the zero-reset/stand
  demo runs (i.e. a posture/wake-up requirement exists for low-level flows).

### 3.5 gamepad  (HEAD 18c05f2, 2025-10-23)

Official UDP gamepad-event receiver used with the official handheld remotes
(Retroid for Lite3, Skydroid for X30), plus the phone app APK
(`controlapp.apk` in-repo). Default receive port **12121**
(example/example_retroid.cpp:93 `RetroidGamepad rc(12121)`; include/gamepad_keys.h:32
`kDefaultPort = 12121`). Constants: `kJoystickRange = 1000`
(gamepad_keys.h:29), `GamepadType kRetroid = 1 / kSkydroid = 2`
(gamepad_keys.h:42-43). Frame format identical to §3.1 (same 0x55 0x66
structure, 16 channels, 10 Retroid buttons). README.md:15-20 shows the remote
app is pointed at an IP+port, so a dev host (not just the robot) can receive
the physical remote's frames.
Relevance: byte-for-byte confirmation of the joystick frames our console
already synthesizes; and a ready-made pattern if we want to *listen* for a real
Retroid remote on :12121 and feed it into the console as an input source.

### 3.6 robotserver_sdk  (HEAD f70df5e, 2025-03-31) — one-liner

X30 Pro navigation-host SDK (TCP + XML "robotserver" on 192.168.1.106; nav
task codes 1002/1003/1004/1007) — nothing to do with Lite3's UDP motion
protocol; only a nicely enumerated failure taxonomy (types.h
ErrorStatus_Navigation) worth imitating when we document our own status codes.

### 3.7 deep-robotics-sdk2  (HEAD a465beb, 2026-08-28 — "new unified SDK")

README root table covers exactly two products: **DR02 Pro and DR02 Std**, as
ROS 2 packages (`dr02_pro`, `dr02_std`) exchanging `drdds` topics on AOS/NOS
hosts (10.21.33.103/.106) or MuJoCo sim. No Lite3, no raw UDP, no Python SDK.
It is the DR02-line successor to the old per-model SDKs, not a Lite3 protocol
change. Its docs do show a clean product-level pattern (named action tables
e.g. 0x3000 greeting … 0x300b clap, EXAMPLES_CN.md:82-93; "current action"
query /ACTION_INFO; state machine Idle→SuspendedStand→RLControl) that is
conceptually what our config.json `actions` + telemetry `motion_state` already
approximate — inspiration only, nothing to port.

### 3.8 deep-robotics-msg  (HEAD aa646c1, 2026-08-04) — one-liner

ROS 2 msg/srv package (`drdds`) backing the DR02 SDK (Joints/Gait/MotionInfo/
FaultEvent… plus StdStatus `int32 state, uint32 error_code`); no Lite3 wire
content, but the "state + error_code" StdStatus shape is a tidy model for our
future status payloads.

### 3.9 deep-robotics-teleoperate  (HEAD 12623bd, 2026-08-31) — one-liner

PICO-XR teleoperation for the DR02 Pro **humanoid** (waist/arm IK, data
collection) — VR/humanoid work, irrelevant to Lite3.

## 4) Recommended changes (ordered by effort/value) for OUR console + SDK

1. **Add the 3 official action codes to config.json actions** (tiny, high
   value, Lite3_LLM llm_control.cpp:61-67): `dance 0x2101030C`, `hello
   0x21010506`, `talent 0x21010204` — all type 0 / value 0 like stand/sit.
   Keep them in a clearly-labeled "extra poses" group and note they move the
   dog (test on the stand first).
2. **Send a monotonic counter as the 0x21040001 heartbeat value** (tiny):
   increment per send instead of static 0 (Lite3_VMC parse_cmd.cpp:116-128
   expects counter, skips value 0). Keep ~300 ms for high-level mode, but
   document that any low-level/SDK watchdog flags gaps > 250 ms (:140).
3. **Parse the extra streamed frames on :43897** (small-medium): add optional
   decoders for code 2306/**0x0902** (12×f64 joint angles, sign-flipped per
   Lite3_ROS Jetson2Motion.cpp:199-224) and code 2309/**0x0905** (6×f64
   handle/goal state, Jetson2Motion.cpp:240-250) — instant "physical remote
   connected + its goal velocities" and "per-joint pose" telemetry with no new
   robot config. Log-and-count any other codes/lengths we currently drop.
4. **Guard the 0x0901 parse against the RL-firmware layout** (small): keep our
   verified 200 B struct as default; if frame length is 216/220 B, log a hint
   that firmware now includes `robot_policy_state` (protocol.hpp:32) and fields
   shifted. One-size parse will silently misread battery/ultrasound on newer
   firmware.
5. **Camera-ready check via 0x2101210D instead of RTSP polling** (medium):
   send the 12 B query to :43899 and treat a reply value 0x11 as
   active/0x10 as inactive (Jetson2App.cpp:174-196); also adopt the
   "sensors-alive gating" pattern (Jetson2App.cpp:150 refuses to start services
   when IMU is dead) — don't enable camera/motion when telemetry is stale.
6. **Document firmware-variant gotchas in our README** (small): check the
   robot's Deeprcs version and `~/jy_exe/conf/name.toml` standard_1/2 state
   (Lite3_MotionSDK README.md:136-160), the 0x0902 dual meaning (stream vs
   low-level command), the wire sign conventions (0x141 yaw negated vs Twist,
   joint angles negated), and that network.toml is the confirmed telemetry
   target file.
7. **(Optional, later) Real-remote input**: receive genuine Retroid frames on
   :12121 (gamepad repo example_retroid.cpp pattern) and let the physical
   remote drive the console — we already have the exact frame decoder upstream;
   frames from the remote and our synthetic frames are interchangeable.

Not recommended: adopting anything from deep-robotics-sdk2/-msg/-teleoperate/
robotserver_sdk (different product lines); switching our velocity channels to
the remote analog codes 0x21010130/131/135 (VMC-only evidence, int16-scaled,
and our 0x140/0x145/0x141 f64 path is confirmed by Lite3_ROS).
