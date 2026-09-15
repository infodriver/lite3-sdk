#!/usr/bin/env python3
"""Minimal hello: stand up and wiggle."""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import lite3sdk  # noqa: E402

dog = lite3sdk.Lite3(sys.argv[1] if len(sys.argv) > 1 else "192.168.2.1")
with dog:
    print("battery: %s%%" % dog.battery_pct)
    dog.hello()
    print("done - dog says hi 👋")
