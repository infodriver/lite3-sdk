#!/usr/bin/env python3
"""Scripted move: stand, drive a small square-ish path, stop, sit.

Usage: python3 examples/basic.py [robot_ip]
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import lite3sdk  # noqa: E402

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.1"
dog = lite3sdk.Lite3(ip, allow_motion_without_telemetry=True)
with dog:
    print("telemetry:", dog.wait_telemetry(timeout=5) is not None)
    dog.stand()
    dog.wait(1.0)
    dog.drive(vx=0.3)          # forward 0.3 m/s
    dog.wait(1.0)
    dog.drive(vx=0.0, wz=0.4)  # turn in place
    dog.wait(0.8)
    dog.drive(vx=-0.2)         # back up
    dog.wait(0.6)
    dog.stop()
    dog.sit()
    print("finished - battery now %s%%" % dog.battery_pct)
