/* Lite3 Web Pilot — simplified console: Wi-Fi connect + robot commands */
"use strict";

const $ = (id) => document.getElementById(id);
const els = {
  led: $("connLed"), connText: $("connText"),
  connectBtn: $("connectBtn"),
  estop: $("estopBtn"), stop: $("stopBtn"), returnBtn: $("returnBtn"),
  stand: $("standBtn"), sit: $("sitBtn"), hello: $("helloBtn"), dance: $("danceBtn"),
  turnOver: $("turnOverBtn"), turnL: $("turnLBtn"), turnR: $("turnRBtn"),
  camState: $("camState"), camImg: $("camImg"), camPlaceholder: $("camPlaceholder"),
  actionBtns: $("actionBtns"), actionHint: $("actionHint"),
  robotIpAdv: $("robotIpAdv"),
  stick: $("stick"), knob: $("stickKnob"),
  stickR: $("stickR"), knobR: $("stickRKnob"),
  rawCode: $("rawCode"), rawValue: $("rawValue"), rawType: $("rawType"), rawHex: $("rawHex"),
  rawSendBtn: $("rawSendBtn"), clearLogBtn: $("clearLogBtn"), logBox: $("logBox"),
  signVx: $("signVx"), signYaw: $("signYaw"), signLat: $("signLat"),
  joyMode: $("joyMode"), joyCurve: $("joyCurve"), joyScale: $("joyScale"),
  camFps: $("camFps"), camScale: $("camScale"), camQuality: $("camQuality"),
  camApply: $("camApply"), camOff: $("camOff"),
};

/* config-driven action buttons (stand/sit are built-in; anything added to
   config.json actions renders here once — server refuses frames without a code) */
let actionsSig = "";
function renderActions(actions) {
  const sig = JSON.stringify((actions || []).map(a => a.name + a.note + (a.group || "")));
  if (sig === actionsSig) return;
  actionsSig = sig;
  els.actionBtns.innerHTML = "";
  const acts = (actions || []).filter(a =>
    !(a.name === "stand_up" || a.name === "sit_down" ||
      a.name === "dance" || a.name === "hello_pose" ||
      a.name === "turn_over")); // built-in buttons
  const groups = [
    { id: "recovery", title: "Recovery (from upside down)" },
    { id: "action", title: "Poses & actions" },
    { id: "mode", title: "Modes" },
    { id: "gait", title: "Gaits" },
  ];
  for (const g of groups) {
    const items = acts.filter(a => (a.group || "action") === g.id);
    if (!items.length) continue;
    const box = document.createElement("div");
    box.className = "cmd-group";
    const t = document.createElement("div");
    t.className = "group-title";
    t.textContent = g.title;
    const row = document.createElement("div");
    row.className = "group-btns";
    for (const a of items) {
      const b = document.createElement("button");
      b.className = "btn label-btn";
      b.disabled = !state.connected || state.estop;
      const icon = a.icon || "";
      const label = a.label || a.name.replace(/_/g, " ");
      const text = (icon ? icon + "&nbsp; " : "") + label;
      b.innerHTML = text;
      b.title = a.name + (a.note ? " - " + a.note : "");
      b.onclick = () => runAction(a.name, (icon ? icon + "…" : "…"), b, text);
      row.appendChild(b);
    }
    box.append(t, row);
    els.actionBtns.appendChild(box);
  }
  els.actionBtns.hidden = acts.length === 0;
  els.actionHint.hidden = acts.length === 0;
}

const ROBOT_IP = "192.168.2.1";
const state = {
  connected: false, estop: false, logSeen: 0,
  keys: new Set(), stickVec: { x: 0, y: 0 }, stickActive: false,
  joyMode: "velocity", joyCurve: 1.25, joyScale: 1.0,
  robotIp: "192.168.2.1", lastCamTry: 0, prevOk: false, camImgOk: false,
};

/* ---------------- api ---------------- */
async function api(path, body) {
  const opts = { method: body ? "POST" : "GET", headers: {} };
  if (body) { opts.headers["Content-Type"] = "application/json"; opts.body = JSON.stringify(body); }
  const r = await fetch(path, opts);
  let j = null;
  try { j = await r.json(); } catch (_) { j = {}; }
  if (!r.ok && !j.ok) throw new Error(j.msg || ("HTTP " + r.status));
  return j;
}

/* ---------------- poll / render ---------------- */
async function poll() {
  try { render(await api("/api/state")); }
  catch (e) { setLed(false, "console error"); }
}

function setLed(connected, text, cls) {
  els.led.className = "led " + (connected ? (cls || "on") : "off");
  els.connText.textContent = text || "";
}

function render(s) {
  const ok = s.connected;
  state.connected = ok;
  state.estop = !!s.estop;
  state.robotIp = s.robot_ip || ROBOT_IP;
  els.estop.disabled = !ok;
  els.connectBtn.disabled = ok;
  els.connectBtn.textContent = ok ? "✅ Connected" : "🔌 Connect";
  els.hello.disabled = !ok || state.estop;
  els.dance.disabled = !ok || state.estop;
  els.turnOver.disabled = !ok || state.estop;
  els.turnL.disabled = !ok || state.estop;
  els.turnR.disabled = !ok || state.estop;
  els.returnBtn.hidden = !state.estop;
  els.returnBtn.disabled = !state.estop;
  els.stand.disabled = !ok || state.estop;
  els.sit.disabled = !ok || state.estop;
  els.stop.disabled = !ok || state.estop;
  els.rawSendBtn.disabled = !ok;
  // keep config-driven action buttons in sync with connection state
  for (const b of els.actionBtns.querySelectorAll("button"))
    if (!b.classList.contains("sending")) b.disabled = !ok || state.estop;

  if (!ok) setLed(false, "not connected - robot not reachable (same network needed)");
  else if (state.estop) setLed(true, "E-STOP", "err");
  else setLed(true, s.robot_ip, "on");


  // log
  if (s.log && s.log.length !== state.logSeen) {
    for (const l of s.log.slice(state.logSeen)) log(l.lvl, l.msg);
    state.logSeen = s.log.length;
  }

  renderActions(s.actions);
  if (s.engine && s.engine.signs) {
    els.signVx.value = String(s.engine.signs.velocity || 1);
    els.signYaw.value = String(s.engine.signs.velocity_yaw || 1);
    els.signLat.value = String(s.engine.signs.velocity_lateral || 1);
  }

  // robot camera - opens automatically while the robot is connected
  const cam = s.camera;
  if (cam) {
    const camOn = !!cam.running;
    els.camState.textContent = camOn ? (cam.frames ? cam.frames + " frames" : "starting…") : "";
    if (cam.running && cam.frames > 0 && !state.camImgOk) {
      els.camImg.src = "/cam.mjpeg?ts=" + Date.now();
      els.camImg.hidden = false;
      els.camPlaceholder.hidden = true;
      state.camImgOk = true;
    }
    // keep the feed open: never rip the image out on a blip - the server
    // watchdog restarts the worker and the img reconnects itself on error
    if (state.camImgOk) els.camImg.hidden = false;
    els.camPlaceholder.hidden = state.camImgOk;
    // auto-open on connect
    const now = Date.now();
    if (ok && !state.prevOk) state.lastCamTry = 0;   // instant start on new connection
    state.prevOk = ok;
    if (ok && !cam.running && now - state.lastCamTry > 6000) {
      state.lastCamTry = now;
      api("/api/config", { section: "camera", enabled: true, start: true })
        .then(() => log("info", "robot connected - camera opening automatically…"))
        .catch(() => {});
    }
    if (!ok && cam.running) {
      api("/api/config", { section: "camera", enabled: false }).catch(() => {});
    }
  }
}

/* ---------------- log ---------------- */
function log(lvl, msg) {
  const line = document.createElement("div");
  line.className = "log-line " + (lvl === "error" ? "error" : lvl === "warn" ? "warn" : "info");
  const t = document.createElement("span"); t.className = "lt"; t.textContent = new Date().toLocaleTimeString();
  const m = document.createElement("span"); m.className = "lm"; m.textContent = msg;
  line.append(t, m);
  els.logBox.appendChild(line);
  while (els.logBox.childElementCount > 200) els.logBox.firstChild.remove();
  els.logBox.scrollTop = els.logBox.scrollHeight;
}

/* ---------------- robot link (auto) ---------------- */
async function tryConnect(ip) {
  if (!ip || state.connected) return state.connected;
  try {
    log("info", "connecting to robot at " + ip + "…");
    await api("/api/connect", { robot_ip: ip });
    await poll();
  } catch (e) { /* keep state.connected as truth */ }
  return state.connected;
}

/* First-time auto-connect, best-effort, no prompts:
   1) try the configured robot IP (already on the robot's Wi-Fi / LAN IP)
   2) discover the robot on the CURRENT network (no Wi-Fi switch, internet stays up)
   3) if allowJoin: join the robot's own Wi-Fi using the saved password (no prompt) */
async function autoLink(allowJoin) {
  if (state.connected) return true;
  if (await tryConnect(state.robotIp || ROBOT_IP)) return true;
  if (Date.now() - (state.lastDiscover || 0) > 15000) {
    state.lastDiscover = Date.now();
    try {
      const d = await api("/api/robot/discover");
      if (d && d.found && d.ip) { state.robotIp = d.ip; if (await tryConnect(d.ip)) return true; }
    } catch (_) {}
  }
  if (allowJoin) {
    try {
      log("info", "robot not on this network — joining its Wi-Fi (saved password)");
      const w = await api("/api/wifi/join_robot");
      if (w && w.ok) {
        await new Promise(r => setTimeout(r, 2500));
        state.robotIp = "192.168.2.1";
        if (await tryConnect("192.168.2.1")) return true;
      }
    } catch (_) {}
  }
  return state.connected;
}

// used by action buttons: quick direct attempt at the configured IP
async function ensureLink() {
  if (state.connected) return true;
  const s = await api("/api/state");
  state.connected = !!s.connected;
  if (state.connected) return true;
  return tryConnect(state.robotIp || ROBOT_IP);
}

async function cmd(body) {
  try { return await api("/api/cmd", body); }
  catch (e) { log("error", e.message); return { ok: false }; }
}

/* ---------------- stand up / sit down ---------------- */
async function runAction(name, label, btn, doneLabel) {
  if (!(await ensureLink())) return;
  btn.disabled = true;
  btn.classList.add("sending");
  btn.innerHTML = label;
  const r = await cmd({ type: "action", name: name });
  if (!r.ok) log("error", name + ": " + (r.msg || "failed"));
  else log("info", name.replace("_", " ") + " command sent");
  setTimeout(() => {
    btn.disabled = !state.connected || state.estop;
    btn.classList.remove("sending");
    btn.innerHTML = doneLabel;
  }, 2000);}

// wire the built-in STAND / SIT buttons (config actions cover the rest)
els.hello.onclick = async () => {
  els.hello.disabled = true;
  els.hello.classList.add("sending");
  els.hello.innerHTML = "👋&nbsp; SAYING HELLO…";
  const r = await cmd({ type: "hello" });
  if (!r.ok) log("error", "hello: " + (r.msg || "failed"));
  setTimeout(() => { els.hello.disabled = !state.connected || state.estop; els.hello.classList.remove("sending"); els.hello.innerHTML = "👋&nbsp; HELLO"; }, 3500);
};
els.stand.onclick = () => runAction("stand_up", "🐕&nbsp; STANDING…", els.stand, "🐕&nbsp; STAND");
els.sit.onclick = () => runAction("sit_down", "🪑&nbsp; SITTING…", els.sit, "🪑&nbsp; SIT");
els.dance.onclick = () => runAction("dance", "🕺&nbsp; DANCING…", els.dance, "🕺&nbsp; DANCE");
els.turnOver.onclick = () => runAction("turn_over", "🔄&nbsp; TURNING OVER…", els.turnOver, "🔄&nbsp; TURN OVER");

// hold-to-slide: LEFT / RIGHT buttons side-walk in place while held
let turnHold = null;
function turnStep(dir) {
  if (state.joyMode === "retroid") {
    cmd({ type: "joystick", x: 0, y: 0,
          vy: dir * (state.joyScale || 1) * (state.latSign || 1) }).catch(() => {});
  } else {
    const vy = (state.maxVy || 0.4) * (state.joyScale || 1) * dir;
    cmd({ type: "velocity", vx: 0, vy: vy, wz: 0 }).catch(() => {});
  }
}
function startTurn(dir, btn) {
  if (!state.connected || state.estop) return;
  stopTurn();
  btn.classList.add("sending");
  turnStep(dir);
  turnHold = { timer: setInterval(() => turnStep(dir), 80), btn: btn };
}
function stopTurn() {
  if (!turnHold) return;
  clearInterval(turnHold.timer);
  turnHold.btn.classList.remove("sending");
  turnHold = null;
  cmd({ type: "stop" }).catch(() => {});
}
els.turnL.addEventListener("pointerdown", (e) => { e.preventDefault(); startTurn(-1, els.turnL); });
els.turnR.addEventListener("pointerdown", (e) => { e.preventDefault(); startTurn(1, els.turnR); });
["pointerup", "pointercancel", "pointerleave"].forEach((ev) => {
  els.turnL.addEventListener(ev, stopTurn);
  els.turnR.addEventListener(ev, stopTurn);
});
window.addEventListener("pointerup", stopTurn);
window.addEventListener("blur", stopTurn);

/* ---------------- drive input layer (joystick + keyboard) ---------------- */
const KEYS = {
  KeyW: "fwd", ArrowUp: "fwd", KeyS: "back", ArrowDown: "back",
  KeyA: "left", ArrowLeft: "left", KeyD: "right", ArrowRight: "right",
};
const joy = { x: 0, y: 0, active: false, id: null, last: 0 };
// second (right-side) pad: left/right = SLIDE sideways, up/down = forward/back
const joyR = { x: 0, y: 0, active: false, id: null, last: 0 };
// latch: after the stick is released off-center, keep driving at that speed
// until the user stops it (STOP button / E-stop / key tap / center release)
const hold = { vx: 0, wz: 0, active: false };

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function setKnob() {
  const R = Math.max(6, (els.stick.clientWidth / 2) - 33); // keep knob inside circle (knob half = 33)
  els.knob.style.transform = "translate(calc(-50% + " + (joy.x * R).toFixed(1) +
    "px), calc(-50% + " + (joy.y * R).toFixed(1) + "px))";
}
function vecFromEvent(e) {
  const r = els.stick.getBoundingClientRect();
  const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  return {
    x: clamp((e.clientX - cx) / (r.width / 2), -1, 1),
    y: clamp((e.clientY - cy) / (r.height / 2), -1, 1),
  };
}
function setKnobR() {
  const R = Math.max(6, (els.stickR.clientWidth / 2) - 33);
  els.knobR.style.transform = "translate(calc(-50% + " + (joyR.x * R).toFixed(1) +
    "px), calc(-50% + " + (joyR.y * R).toFixed(1) + "px))";
}
function vecFromEventR(e) {
  const r = els.stickR.getBoundingClientRect();
  const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  return {
    x: clamp((e.clientX - cx) / (r.width / 2), -1, 1),
    y: clamp((e.clientY - cy) / (r.height / 2), -1, 1),
  };
}
function joyPos() {
  const L = joy.active ? { x: joy.x, y: joy.y } : null;
  const R = joyR.active ? { x: joyR.x, y: joyR.y } : null;
  if (L || R) {
    // left pad: up/down = forward/back, left/right = turn
    // right pad: left/right = SLIDE sideways, up/down = forward/back
    let fy = L ? L.y : 0;
    if (R && (!L || Math.abs(R.y) > Math.abs(L.y))) fy = R.y;
    return { x: L ? L.x : 0, y: fy, slide: R ? R.x : 0 };
  }
  let vx = 0, wz = 0;
  if (state.keys.has("KeyW") || state.keys.has("ArrowUp")) vx += 1;
  if (state.keys.has("KeyS") || state.keys.has("ArrowDown")) vx -= 1;
  if (state.keys.has("KeyD") || state.keys.has("ArrowRight")) wz += 1;   // turn right
  if (state.keys.has("KeyA") || state.keys.has("ArrowLeft")) wz -= 1;    // turn left
  const mag = Math.hypot(vx, wz);
  if (mag) return { x: wz / mag, y: vx / mag, keys: true };
  return { x: 0, y: 0 };
}

// map stick/keys to body velocities: up = forward (+vx), right = turn right (+wz)
function driveVec() {
  const p = joyPos();
  if (p.hold) return { vx: hold.vx, vy: hold.vy || 0, wz: hold.wz, hold: true };
  const dz = 0.08, m = Math.hypot(p.x, p.y, p.slide || 0);
  if (m < dz && !p.keys) return null;
  // normalize past the deadzone so small pushes still count
  let s = m < dz ? 0 : Math.min(1, (m - dz) / (1 - dz));
  // stick curve (expo): >1 gives finer control near center (RC-remote feel)
  if (state.joyCurve > 1) s = Math.pow(s, state.joyCurve);
  const sc = state.joyScale || 1;
  // left stick: up/down = forward/back, left/right = TURN
  // right stick: left/right = SIDE-WALK (slide)
  // When reversing, mirror the steer so "stick left" still turns the robot
  // left on screen (instead of the mirrored reverse-steering feel).
  let vx = -p.y * s * sc;
  let wz = p.x * s * sc;
  let vy = (p.slide || 0) * s * sc;
  if (vx < 0) wz = -wz;
  return { vx: vx, vy: vy, wz: wz };
}

let lastDrive = { vx: 0, wz: 0, t: 0 };
function driveTick() {
  const now = Date.now();
  const vec = driveVec();
  const changed = vec && lastDrive && (vec.vx !== lastDrive.vx || vec.vy !== (lastDrive.vy || 0) || vec.wz !== lastDrive.wz);
  const sendNow = vec && (now - lastDrive.t > 60 || changed);
  if (sendNow) {
    lastDrive = { vx: vec.vx, vy: vec.vy || 0, wz: vec.wz, t: now };
    if (state.joyMode === "retroid") {
      // official SDK path: RETROID joystick frames (0x55 0x66) -> robot :12121
      cmd({ type: "joystick", x: vec.wz, y: vec.vx, vy: vec.vy || 0 }).catch(() => {});
    } else {
      cmd({ type: "velocity", vx: vec.vx, vy: vec.vy || 0, wz: vec.wz }).catch(() => {});
    }
    els.joyInfo.textContent = "fwd " + vec.vx.toFixed(2) + " · side " + (vec.vy || 0).toFixed(2) + " · turn " + vec.wz.toFixed(2);
    if (vec.hold && !els.joyInfo.textContent.startsWith("HOLDING"))
      els.joyInfo.textContent = "HOLDING fwd " + vec.vx.toFixed(2) + " · side " + (vec.vy || 0).toFixed(2) + " · turn " + vec.wz.toFixed(2) + " (STOP to halt)";
  } else if (!vec && (lastDrive.vx !== 0 || lastDrive.wz !== 0 || (lastDrive.vy || 0) !== 0)) {
    lastDrive = { vx: 0, vy: 0, wz: 0, t: now };
    hold.active = false;
    // stick is centered / released: full stop - zero velocity PLUS the
    // official brake frames so the robot halts immediately (no coast)
    cmd({ type: "stop" }).catch(() => {});
    if (state.joyMode === "retroid") cmd({ type: "joystick", x: 0, y: 0 }).catch(() => {});
    els.joyInfo.textContent = "idle";
  }
}
setInterval(driveTick, 60);

// joystick pointer events
els.stick.addEventListener("pointerdown", e => {
  joy.active = true; joy.id = e.pointerId; joy.last = Date.now();
  els.knob.style.transition = "";  // follow the pointer instantly while held
  els.stick.classList.add("active");
  try { els.stick.setPointerCapture(e.pointerId); } catch (_) {}
  const v = vecFromEvent(e); joy.x = v.x; joy.y = v.y; setKnob();
  driveTick();   // drive instantly on press
});
// stick back at center while held -> stop signal already flows via vec null
els.stick.addEventListener("pointermove", e => {
  if (!joy.active || e.pointerId !== joy.id) return;
  joy.last = Date.now();
  const v = vecFromEvent(e); joy.x = v.x; joy.y = v.y; setKnob();
  driveTick();   // follow the pointer with zero latency
});
function releaseJoy() {
  // SPRING-CENTERED (physical-remote feel): releasing always returns the
  // stick to center and stops - no latch. Drive only happens while held.
  if (joy.active && joy.id != null) {
    try { els.stick.releasePointerCapture(joy.id); } catch (_) {}
  }
  joy.active = false; joy.id = null; joy.x = 0; joy.y = 0; joy.last = 0;
  els.knob.style.transition = "transform 160ms ease-out";  // snap back to center
  setKnob();
  els.stick.classList.remove("active");
  setTimeout(() => { if (!joy.active) els.knob.style.transition = ""; }, 220);
  // stop IMMEDIATELY at release - don't wait for the next 60ms drive tick
  if (lastDrive.vx !== 0 || lastDrive.wz !== 0) {
    lastDrive = { vx: 0, wz: 0, t: Date.now() };
    cmd({ type: "stop" }).catch(() => {});
    if (state.joyMode === "retroid") cmd({ type: "joystick", x: 0, y: 0 }).catch(() => {});
    els.joyInfo.textContent = "idle";
  }
}
function joyUp(e) {
  if (!joy.active || e.pointerId !== joy.id) return;
  releaseJoy();
}

/* ---- right pad (slide) ---- */
els.stickR.addEventListener("pointerdown", e => {
  joyR.active = true; joyR.id = e.pointerId; joyR.last = Date.now();
  els.knobR.style.transition = "";
  els.stickR.classList.add("active");
  try { els.stickR.setPointerCapture(e.pointerId); } catch (_) {}
  const v = vecFromEventR(e); joyR.x = v.x; joyR.y = v.y; setKnobR();
  driveTick();
});
els.stickR.addEventListener("pointermove", e => {
  if (!joyR.active || e.pointerId !== joyR.id) return;
  joyR.last = Date.now();
  const v = vecFromEventR(e); joyR.x = v.x; joyR.y = v.y; setKnobR();
  driveTick();
});
function releaseJoyR() {
  if (joyR.active && joyR.id != null) {
    try { els.stickR.releasePointerCapture(joyR.id); } catch (_) {}
  }
  joyR.active = false; joyR.id = null; joyR.x = 0; joyR.y = 0; joyR.last = 0;
  els.knobR.style.transition = "transform 160ms ease-out";
  setKnobR();
  els.stickR.classList.remove("active");
  setTimeout(() => { if (!joyR.active) els.knobR.style.transition = ""; }, 220);
  if (lastDrive.vx !== 0 || lastDrive.wz !== 0 || (lastDrive.vy || 0) !== 0) {
    lastDrive = { vx: 0, vy: 0, wz: 0, t: Date.now() };
    cmd({ type: "stop" }).catch(() => {});
    els.joyInfo.textContent = "idle";
  }
}
function joyUpR(e) {
  if (!joyR.active || e.pointerId !== joyR.id) return;
  releaseJoyR();
}
els.stickR.addEventListener("pointerup", joyUpR);
els.stickR.addEventListener("pointercancel", joyUpR);
els.stickR.addEventListener("lostpointercapture", joyUpR);
function releaseAll() { releaseJoy(); releaseJoyR(); }
// the missing release wiring: without these the stick stays "held" forever
// after a drag ends (especially release outside the pad or tab switch)
els.stick.addEventListener("pointerup", joyUp);
els.stick.addEventListener("pointercancel", joyUp);
els.stick.addEventListener("lostpointercapture", joyUp);
// release capture at the WINDOW level: catches pointerup/pointercancel even
// if it happens outside the pad. (No time-based watchdog here - holding the
// stick still at a position must keep driving indefinitely.)
window.addEventListener("pointerup", e => {
  if (joy.active && e.pointerId === joy.id) releaseJoy();
  if (joyR.active && e.pointerId === joyR.id) releaseJoyR();
});
window.addEventListener("pointercancel", e => {
  if (joy.active && e.pointerId === joy.id) releaseJoy();
  if (joyR.active && e.pointerId === joyR.id) releaseJoyR();
});
// page hidden (tab switch) while driving -> let go
document.addEventListener("visibilitychange", () => {
  if (document.hidden) releaseAll();
});
// mouse hover stop: with the stick latched (released off-center), simply
// bringing the MOUSE back over the pad center (no click needed) releases it
window.addEventListener("pointermove", e => {
  if (!hold.active || e.pointerType !== "mouse" || e.buttons) return;
  const r = els.stick.getBoundingClientRect();
  const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  const vx = (e.clientX - cx) / (r.width / 2), vy = (e.clientY - cy) / (r.height / 2);
  if (Math.hypot(vx, vy) < 0.12) {
    hold.active = false;
    log("info", "stick back at center - hold released");
  }
});
// tab/window loses focus while dragging -> let go too
window.addEventListener("blur", () => {
  releaseAll();
  if (hold.active) { hold.active = false; cmd({ type: "velocity", vx: 0, vy: 0, wz: 0 }); }
});

// E-STOP / stop / return
els.estop.onclick = () => {
  if (!state.connected) return;
  hold.active = false;
  stopTurn();
  log("warn", "E-STOP pressed (latched until Return)");
  cmd({ type: "estop" });
};
els.returnBtn.onclick = () => { cmd({ type: "reset_estop" }); log("info", "E-stop reset"); };
els.stop.onclick = () => { hold.active = false; stopTurn(); cmd({ type: "stop" }); log("info", "stop sent"); };

// keyboard: WASD / arrows drive, Space = hold-to-E-stop
let spaceHeld = false;
function typing(e) {
  const t = e.target;
  return t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA");
}
window.addEventListener("keydown", e => {
  if (typing(e)) return;
  if (e.code === "Space") {
    e.preventDefault();
    if (!e.repeat && !spaceHeld) { spaceHeld = true; if (!state.estop) cmd({ type: "estop" }); }
    return;
  }
  if (KEYS[e.code]) {
    e.preventDefault();
    if (hold.active) { hold.active = false; log("info", "key tap stops hold"); }
    state.keys.add(e.code);
  }
});
window.addEventListener("keyup", e => {
  if (typing(e)) return;
  if (e.code === "Space") { e.preventDefault(); if (spaceHeld) { spaceHeld = false; if (state.estop) cmd({ type: "reset_estop" }); } return; }
  if (KEYS[e.code]) state.keys.delete(e.code);
});

els.robotIpAdv.onchange = async () => {
  try {
    await api("/api/config", { section: "engine", robot_ip: els.robotIpAdv.value.trim() });
    log("info", "robot IP set to " + els.robotIpAdv.value.trim());
  } catch (e) { log("error", "robot IP: " + e.message); }
};

/* ---------------- capture: photo & video ---------------- */

function applySign(kind, val) {
  api("/api/config", { section: "engine", [kind + "_sign"]: parseInt(val, 10) })
    .then(() => log("info", kind + " sign → " + val))
    .catch(() => {});
}
els.signVx.onchange = (e) => applySign("velocity", e.target.value);
els.signYaw.onchange = (e) => applySign("velocity_yaw", e.target.value);
els.signLat.onchange = (e) => applySign("velocity_lateral", e.target.value);
els.joyMode.onchange = async (e) => {
  try {
    await api("/api/config", { section: "engine", joystick_mode: e.target.value });
    state.joyMode = e.target.value;
    hold.active = false;
    log("info", "joystick mode -> " + e.target.value);
  } catch (err) { log("error", "joystick mode: " + err.message); }
};
els.joyCurve.onchange = async (e) => {
  const v = parseFloat(e.target.value);
  try {
    await api("/api/config", { section: "engine", joystick_curve: v });
    state.joyCurve = v;
    hold.active = false;
    log("info", "stick curve -> " + v);
  } catch (err) { log("error", "stick curve: " + err.message); }
};
els.joyScale.onchange = async (e) => {
  const v = parseFloat(e.target.value);
  try {
    await api("/api/config", { section: "engine", joystick_scale: v });
    state.joyScale = v;
    hold.active = false;
    log("info", "speed scale -> " + v);
  } catch (err) { log("error", "speed scale: " + err.message); }
};

/* camera quality/size controls */
els.camApply.onclick = async () => {
  try {
    await api("/api/config", {
      section: "camera",
      fps: parseInt(els.camFps.value, 10) || 6,
      scale: els.camScale.value.trim() || "480:-1",
      quality: parseInt(els.camQuality.value, 10) || 11,
      enabled: true,
      start: true,
    });
    log("info", "camera profile applied (" + els.camFps.value + " fps, " +
        els.camScale.value + ", q" + els.camQuality.value + ")");
  } catch (err) { log("error", "camera: " + err.message); }
};
els.camOff.onclick = async () => {
  try {
    await api("/api/config", { section: "camera", enabled: false });
    log("info", "camera off");
  } catch (err) { log("error", "camera: " + err.message); }
};

/* ---------------- one-click connect ---------------- */
els.connectBtn.onclick = async () => {
  els.connectBtn.disabled = true;
  els.connectBtn.textContent = "Connecting…";
  try {
    const ip = state.robotIp || ROBOT_IP;
    log("info", "connecting to robot at " + ip + "…");
    await api("/api/connect", { robot_ip: ip });
    await poll();
    if (state.connected) {
      log("info", "robot connected - launching camera…");
      api("/api/config", { section: "camera", enabled: true, start: true })
        .then(() => log("info", "camera opening…"))
        .catch(() => {});
    } else {
      log("warn", "no reply from the robot - is it powered on and on this network?");
    }
  } catch (e) {
    log("error", "connect: " + e.message);
  }
  els.connectBtn.disabled = state.connected;
  els.connectBtn.textContent = state.connected ? "✅ Connected" : "🔌 Connect";
};

/* ---------------- misc ---------------- */
els.clearLogBtn.onclick = () => { els.logBox.innerHTML = ""; state.logSeen = 0; };
// persistent camera stream: if the mjpeg socket ever drops, reconnect
els.camImg.onerror = () => {
  if (!state.connected) return;
  clearTimeout(state.camRetry);
  state.camRetry = setTimeout(() => {
    if (state.connected) els.camImg.src = "/cam.mjpeg?ts=" + Date.now();
  }, 2500);
};
window.addEventListener("beforeunload", () => { try { navigator.sendBeacon("/api/cmd", JSON.stringify({ type: "stop" })); } catch (_) {} });

/* ---------------- boot ---------------- */
(async function boot() {
  log("info", "Lite3 Pilot ready.");
  setInterval(poll, 600);
  await poll();
  try {
    const s = await api("/api/state");
    if (s.robot_ip) els.robotIpAdv.value = s.robot_ip;
    if (s.engine && s.engine.joystick) {
      state.joyMode = s.engine.joystick.mode || "velocity";
      els.joyMode.value = state.joyMode;
      state.joyCurve = s.engine.joystick.curve || 1.25;
      els.joyCurve.value = String(state.joyCurve);
      state.joyScale = s.engine.joystick.scale || 1;
      els.joyScale.value = String(state.joyScale);
    }
    if (s.engine && s.engine.max_wz) state.maxWz = s.engine.max_wz;
    if (s.engine && s.engine.max_vy) state.maxVy = s.engine.max_vy;
    if (s.engine && s.engine.signs) state.latSign = s.engine.signs.velocity_lateral || 1;
  } catch (_) {}
  if (!state.connected) autoLink(false);                          // first open: connect/discover only - never auto-switch Wi-Fi (Kai: "dont switch wifi"); join is manual via 📶
  setInterval(() => { if (!state.connected) autoLink(false); }, 9000);  // keep trying quietly
})();
