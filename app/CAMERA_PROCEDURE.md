# Robodog Camera — Connection Procedure (SDK method)

Goal: point the **Lite3 Pilot** app at the robodog's real camera stream using the
same logic the **DeepRobotics SDK** uses, and verify it end to end.

> How the SDK does it (official sources — `DeepRoboticsLab/Lite3_Track`
> `GStreamerWrapper.py`, MotionSDK manual, Perception manual):
> the robot runs an **RTSP H.264 server** on its perception host and you read
> `rtsp://<robot-ip>:8554/test`. The robot **never joins your Wi-Fi — your
> computer joins the robot's Wi-Fi**. Perception-host IP by setup:
> dog hotspot `192.168.2.1` · Ethernet `192.168.1.120` · some SKUs `192.168.1.103`.

---

## Part A — manual robot steps (do these yourself)

1. **Power the robodog on** and let it boot (app/standby).
2. **Connect this Mac to the robodog's Wi-Fi** (Wi-Fi menu → `YSC-JYML-…-5G`,
   password as configured — this is a manual step on purpose).
   Confirm the Mac now has an IP like `192.168.2.x`.
3. **Switch the robodog's camera on** — however it is enabled on your unit
   (robot-side switch, the official app's camera view, or the perception host
   service). You should see the robot's camera working in its own app/display.

## Part B — one command updates the app's camera logic from the robot

4. Open the **Lite3 Pilot** app (browser console, port 8123).
5. Run the autoconfigure script (in the app's folder):

   ```bash
   cd "~/Applications/Lite3 Pilot.app/Contents/Resources"
   bash SCRIPTS/camera_autoconfigure.sh
   # (source copy: ~/.openclaw/workspace/lite3-web-pilot/SCRIPTS/camera_autoconfigure.sh)
   ```

   What it does:
   1. checks which network the Mac is on;
   2. finds the robot (`/api/robot/discover`: SSH + RTSP signature, then asks
      the stream for its `/test` path — so home IP cameras are ignored);
   3. probes the real stream (RTSP `:8554/test` `:8554/live` `:554/test`,
      MJPEG `:8080/:8090`) and **saves the working URL into the app's
      Camera 1 (front) preset** + sets the robot IP;
   4. starts the camera feed;
   5. takes a **test photo** into `~/lite3/PhotoVideo`.

6. If it prints `ok: true … camera live` — done: the app's camera logic now
   matches the real robot stream. Photo/video + STAND/SIT all work.

## Part C — done / cleanup

7. When finished, switch the Mac back to **your home Wi-Fi** (Wi-Fi menu).
   (Motion/camera need the shared network only while you are using them.)

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `autoconfigure did not find a live stream` | Mac not actually on the robot's Wi-Fi (`192.168.2.x`)? Robot powered + camera switched on (Part A)? |
| Says "RTSP hosts without SSH… 192.168.1.40" | That's a home IP camera — the robot isn't on this network yet; do Part A. |
| Robot on Ethernet instead of Wi-Fi | Cable Mac→robot (`192.168.1.120`), set Mac wired IP `192.168.1.x`, then re-run Part B. |
| Want auto network-IP switching back | In `config.json` set the preset URL back to `rtsp://{robot_ip}:8554/test` (token follows the Advanced → Robot IP). |

SDK references: `DeepRoboticsLab/Lite3_Track` (RTSP URL), `Lite3_MotionSDK`
(host IPs), Jueying Lite3 Perception Development Manual (perception host).
