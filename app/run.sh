#!/bin/sh
# Lite3 Web Pilot launcher
cd "$(dirname "$0")"
exec python3 server.py "$@"
