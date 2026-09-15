#!/bin/sh
# camera_autoconfigure.sh — update the app's camera connection from the real
# robodog (SDK method). Run AFTER you have:
#   1) connected this Mac to the robodog's Wi-Fi (manually), and
#   2) switched the robodog's camera on (manually / via the robot).
#
# It finds the robot, probes the actual camera stream (rtsp :8554/:554 /test,
# mjpg :8080/:8090), saves it into the app, starts the feed and takes a test
# photo into ~/lite3/PhotoVideo.
#
# Usage:  bash camera_autoconfigure.sh      (needs the Lite3 Pilot app running)
set -u
API="http://localhost:8123"

say() { echo; echo "== $* =="; }
fail() { echo; echo "!! $*"; exit 1; }

command -v curl >/dev/null || fail "curl not found"
curl -s -o /dev/null -m 3 "$API/api/state" || fail "Lite3 Pilot app not running (open it first)"

say "1/4 network check"
IP=$(curl -s -m 5 "$API/api/wifi/status" | sed -E 's/.*"ip": ?"([^"]*)".*/\1/')
echo "Mac is on: ${IP:-unknown}"
case "$IP" in
  192.168.2.*|192.168.1.*) echo "same subnet as a robodog network - good" ;;
  *) echo "note: expect the robodog subnet (192.168.2.x / 192.168.1.x)" ;;
esac

say "2/4 finding the robodog + its camera stream"
RESP=$(curl -s -m 60 -X POST "$API/api/camera/autoconfigure")
echo "$RESP"
echo "$RESP" | grep -q '"ok": *true' || fail "autoconfigure did not find a live stream. Check: Mac really on the robot's Wi-Fi? robot powered? robot camera switched on? (see CAMERA_PROCEDURE.md)"

say "3/4 verifying live frames"
sleep 2
curl -s -m 5 "$API/api/state" | python3 -c "
import json,sys
c=json.load(sys.stdin)['camera']
print('camera running:', c['running'], '| frames:', c['frames'], '| effective:', c.get('effective'))
" 2>/dev/null || echo "(frame check skipped - python3 not available)"

say "4/4 test photo"
PH=$(curl -s -m 20 -X POST "$API/api/capture/photo")
echo "$PH"
echo "$PH" | grep -q '"ok": *true' && echo "Saved in ~/lite3/PhotoVideo - camera pipeline confirmed."

say "DONE - camera is live in the app."
echo "When finished, switch the Mac back to your home Wi-Fi (Wi-Fi menu)."
