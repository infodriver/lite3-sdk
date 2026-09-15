#!/usr/bin/env python3
"""Listen for official RETROID/Skydroid gamepad frames (DeepRoboticsLab/gamepad
spec) and print what the remote sends - buttons + axes.

Use it to discover exactly which button/frame the official app uses for an
action (e.g. turn over), then wire that into the Lite3 Pilot console.

Run:  python3 tools/retroid_listen.py [port]
Then point the controlapp on the RETROID at THIS machine's IP + that port
(default 12121) and press buttons - each change is logged.
"""
import socket
import struct
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 12121

# RetroidKeys bit order (gamepad_keys.h): LSB first
BUTTONS = ["R1", "L1", "start", "select", "R2", "L2", "A", "B", "X", "Y",
           "left", "right", "up", "down", "left_axis_button", "right_axis_button"]

def decode(payload):
    buttons = struct.unpack_from("<10H", payload, 0)
    axes = struct.unpack_from("<4h", payload, 20)
    axis_buttons = struct.unpack_from("<2H", payload, 28)
    pressed = [BUTTONS[i] for i in range(16) if (buttons[0] >> i) & 1]
    return buttons, axes, axis_buttons, pressed

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", PORT))
    print(f"listening for gamepad frames on udp :{PORT} (Ctrl-C to stop)")
    last = None
    while True:
        data, addr = s.recvfrom(4096)
        if len(data) < 2 or data[:2] != b"\x55\x66":
            print(f"[{addr[0]}] non-retroid packet ({len(data)}B): {data[:24].hex()}")
            continue
        ctrl, dlen, seq, gid, crc = struct.unpack_from("<BHHBH", data, 2)
        payload = data[9:9 + dlen]
        if len(payload) < 32:
            continue
        buttons, axes, ab, pressed = decode(payload[:32])
        sig = (tuple(pressed), axes)
        if sig == last:
            continue               # only log changes
        last = sig
        t = time.strftime("%H:%M:%S")
        print(f"[{t}] {addr[0]} id={gid} seq={seq} "
              f"buttons={pressed or '-'} axes={axes} axis_buttons={ab}")

if __name__ == "__main__":
    main()
