#!/usr/bin/env python3
"""
fpv_web_server.py

Laptop-hosted browser UI for:
- Discovering robots by ROS topics (/ROBOT/cmd_vel and /ROBOT/image_raw)
- Showing all camera thumbnails (JPEG snapshots)
- Showing low-latency main FPV via WebRTC for the currently "active robot"
- Sending touch-friendly drive commands to the selected robot
- Enforcing "one human controls one robot at a time" via a per-robot lock

Video model:
- WebRTC ONLY for the active robot (main view)
- JPEG thumbnails for all robots (scalable)

Teleop integration:
- Your keyboard teleop publishes /active_robot when 'm' selects a robot. :contentReference[oaicite:4]{index=4}
- This server subscribes to /active_robot and auto-switches the main stream in all connected browsers.
"""

import asyncio
import json
import re
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

import aiohttp
from aiohttp import web

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image

# WebRTC
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack

# Frame handling
import numpy as np
from av import VideoFrame
from PIL import Image as PILImage
import io


CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")
IMG_RE = re.compile(r"^/([^/]+)/image_raw$")


def now_s() -> float:
    return time.time()


@dataclass
class ControlLock:
    controller_id: str
    last_heartbeat_s: float


class RosFpvHub(Node):
    """
    ROS side:
    - Track robots from topic discovery
    - Subscribe to /active_robot (from teleop) and rebroadcast to web clients
    - Subscribe to each robot's image topic on-demand and store latest frame
    - Publish cmd_vel to /<robot>/cmd_vel when web controls are used
    - Publish /active_robot when web UI switches robots (so teleop + web stay consistent)
    """
    def __init__(self):
        super().__init__("robot_legion_fpv_web")

        self.declare_parameter("jpeg_thumb_hz", 2.0)  # browser polls, but also useful for load expectations
        self.jpeg_thumb_hz = float(self.get_parameter("jpeg_thumb_hz").value)

        self.active_robot: Optional[str] = None
        self.active_robot_sub = self.create_subscription(String, "/active_robot", self._on_active_robot, 10)
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)

        # Latest frames by robot
        self._latest_img_msg: Dict[str, Image] = {}
        self._img_subs: Dict[str, any] = {}

        # cmd_vel publishers by robot (created lazily)
        self._cmd_pubs: Dict[str, any] = {}

        # Subscribers count by image topic for light bookkeeping
        self._known_robots_cache: Set[str] = set()

        # A simple timer to refresh discovery cache
        self.create_timer(0.5, self._refresh_discovery_cache)

        self.get_logger().info("FPV hub started (ROS side).")

    def _on_active_robot(self, msg: String):
        self.active_robot = msg.data.strip() or None

    def set_active_robot_from_web(self, robot: str):
        robot = (robot or "").strip()
        if not robot:
            return
        m = String()
        m.data = robot
        self.active_robot_pub.publish(m)
        self.active_robot = robot

    def _refresh_discovery_cache(self):
        robots = set()
        for (t, _types) in self.get_topic_names_and_types():
            m1 = CMD_VEL_RE.match(t)
            if m1:
                robots.add(m1.group(1))
                continue
            m2 = IMG_RE.match(t)
            if m2:
                robots.add(m2.group(1))
                continue
        self._known_robots_cache = robots

    def list_robots(self) -> Set[str]:
        return set(self._known_robots_cache)

    def ensure_image_subscription(self, robot: str):
        robot = robot.strip()
        if not robot:
            return
        if robot in self._img_subs:
            return
        topic = f"/{robot}/image_raw"

        def cb(msg: Image, r=robot):
            self._latest_img_msg[r] = msg

        self._img_subs[robot] = self.create_subscription(Image, topic, cb, 10)
        self.get_logger().info(f"Subscribed to {topic}")

    def get_latest_frame_rgb(self, robot: str) -> Optional[np.ndarray]:
        """
        Returns HxWx3 uint8 RGB frame if available.
        Supports: rgb8, bgr8, mono8.
        """
        msg = self._latest_img_msg.get(robot)
        if msg is None:
            return None

        enc = (msg.encoding or "").lower().strip()
        h = int(msg.height)
        w = int(msg.width)

        if h <= 0 or w <= 0:
            return None

        data = msg.data  # bytes-like

        if enc == "rgb8":
            arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
            return arr
        if enc == "bgr8":
            arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
            return arr[:, :, ::-1]  # BGR->RGB
        if enc == "mono8":
            g = np.frombuffer(data, dtype=np.uint8).reshape((h, w))
            return np.stack([g, g, g], axis=-1)

        # Fallback: try to interpret as RGB8-sized buffer
        expected = h * w * 3
        if len(data) >= expected:
            arr = np.frombuffer(data[:expected], dtype=np.uint8).reshape((h, w, 3))
            return arr
        return None

    def publish_cmd(self, robot: str, lin: float, ang: float):
        robot = robot.strip()
        if not robot:
            return
        topic = f"/{robot}/cmd_vel"
        pub = self._cmd_pubs.get(robot)
        if pub is None:
            pub = self.create_publisher(Twist, topic, 10)
            self._cmd_pubs[robot] = pub

        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        pub.publish(t)


class RosVideoTrack(VideoStreamTrack):
    """
    WebRTC video track that pulls the latest ROS frame for a single robot.
    """
    def __init__(self, hub: RosFpvHub, robot: str, fps: float = 20.0):
        super().__init__()
        self.hub = hub
        self.robot = robot
        self.period = 1.0 / max(1.0, float(fps))
        self._last_sent = 0.0

    async def recv(self) -> VideoFrame:
        # pacing
        now = time.time()
        dt = now - self._last_sent
        if dt < self.period:
            await asyncio.sleep(self.period - dt)
        self._last_sent = time.time()

        frame = self.hub.get_latest_frame_rgb(self.robot)
        if frame is None:
            # send a black frame if nothing yet
            frame = np.zeros((240, 320, 3), dtype=np.uint8)

        vf = VideoFrame.from_ndarray(frame, format="rgb24")
        vf.pts, vf.time_base = await self.next_timestamp()
        return vf


class FpvWebServer:
    def __init__(self, hub: RosFpvHub):
        self.hub = hub

        # Websocket clients: client_id -> websocket
        self.ws_clients: Dict[str, web.WebSocketResponse] = {}

        # Per-robot control locks
        self.locks: Dict[str, ControlLock] = {}
        self.lock_timeout_s = 5.0  # if no heartbeat

        # Active robot per client (what they are *viewing* / controlling)
        self.client_active_robot: Dict[str, str] = {}

        # WebRTC peer connections
        self.pcs: Set[RTCPeerConnection] = set()

        # housekeeping
        asyncio.create_task(self._lock_reaper())

    async def _lock_reaper(self):
        while True:
            await asyncio.sleep(1.0)
            t = now_s()
            dead = []
            for robot, lk in self.locks.items():
                if (t - lk.last_heartbeat_s) > self.lock_timeout_s:
                    dead.append(robot)
            for robot in dead:
                self.locks.pop(robot, None)
                await self._broadcast({"type": "lock_update", "locks": self._locks_public()})

    def _locks_public(self) -> Dict[str, str]:
        return {robot: lk.controller_id for robot, lk in self.locks.items()}

    async def _broadcast(self, msg: Dict):
        data = json.dumps(msg)
        for _cid, ws in list(self.ws_clients.items()):
            try:
                await ws.send_str(data)
            except Exception:
                pass

    def _robot_is_locked_by_other(self, robot: str, client_id: str) -> bool:
        lk = self.locks.get(robot)
        if lk is None:
            return False
        return lk.controller_id != client_id

    def _touch_lock(self, robot: str, client_id: str):
        self.locks[robot] = ControlLock(controller_id=client_id, last_heartbeat_s=now_s())

    # ---------------- HTTP handlers ----------------

    async def handle_index(self, request: web.Request):
        html = _INDEX_HTML
        return web.Response(text=html, content_type="text/html")

    async def handle_app_js(self, request: web.Request):
        return web.Response(text=_APP_JS, content_type="application/javascript")

    async def handle_style_css(self, request: web.Request):
        return web.Response(text=_STYLE_CSS, content_type="text/css")

    async def handle_api_state(self, request: web.Request):
        robots = sorted(self.hub.list_robots())
        return web.json_response({
            "robots": robots,
            "active_robot": self.hub.active_robot,
            "locks": self._locks_public(),
        })

    async def handle_api_jpeg(self, request: web.Request):
        robot = (request.query.get("robot") or "").strip()
        if not robot:
            return web.Response(status=400, text="robot query param required")

        self.hub.ensure_image_subscription(robot)
        frame = self.hub.get_latest_frame_rgb(robot)
        if frame is None:
            return web.Response(status=404, text="no frame yet")

        img = PILImage.fromarray(frame, mode="RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return web.Response(body=buf.getvalue(), content_type="image/jpeg")

    # ---------------- WebRTC offer/answer ----------------

    async def handle_webrtc_offer(self, request: web.Request):
        """
        POST JSON:
          { "robot": "robotX", "sdp": "...", "type": "offer" }
        returns:
          { "sdp": "...", "type": "answer" }
        """
        body = await request.json()
        robot = (body.get("robot") or "").strip()
        if not robot:
            return web.Response(status=400, text="robot required")

        self.hub.ensure_image_subscription(robot)

        offer = RTCSessionDescription(sdp=body["sdp"], type=body["type"])
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_state_change():
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self.pcs.discard(pc)

        # Add a single video track for the requested robot
        pc.addTrack(RosVideoTrack(self.hub, robot=robot, fps=20.0))

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        })

    # ---------------- WebSocket (control + events) ----------------

    async def handle_ws(self, request: web.Request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = request.query.get("client_id") or f"client_{int(time.time()*1000)}"
        self.ws_clients[client_id] = ws

        # initial push
        await ws.send_str(json.dumps({
            "type": "hello",
            "client_id": client_id,
            "robots": sorted(self.hub.list_robots()),
            "active_robot": self.hub.active_robot,
            "locks": self._locks_public(),
        }))

        try:
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except Exception:
                    continue

                mtype = data.get("type")

                if mtype == "heartbeat":
                    robot = (data.get("robot") or "").strip()
                    if robot:
                        # only refresh if they currently own it (or it is free)
                        if not self._robot_is_locked_by_other(robot, client_id):
                            self._touch_lock(robot, client_id)
                            await self._broadcast({"type": "lock_update", "locks": self._locks_public()})
                    continue

                if mtype == "set_active_robot":
                    robot = (data.get("robot") or "").strip()
                    if not robot:
                        continue
                    self.client_active_robot[client_id] = robot
                    # publish to ROS so teleop + all browsers stay consistent
                    self.hub.set_active_robot_from_web(robot)
                    await self._broadcast({"type": "active_robot", "robot": robot})
                    continue

                if mtype == "drive":
                    robot = (data.get("robot") or "").strip()
                    lin = float(data.get("lin", 0.0))
                    ang = float(data.get("ang", 0.0))

                    if not robot:
                        continue

                    # enforce lock
                    if self._robot_is_locked_by_other(robot, client_id):
                        await ws.send_str(json.dumps({
                            "type": "error",
                            "message": f"Robot '{robot}' is currently controlled by someone else."
                        }))
                        continue

                    self._touch_lock(robot, client_id)
                    self.hub.publish_cmd(robot, lin, ang)
                    continue

        finally:
            # Cleanup
            self.ws_clients.pop(client_id, None)
            self.client_active_robot.pop(client_id, None)

            # Release locks owned by this client
            to_release = [r for r, lk in self.locks.items() if lk.controller_id == client_id]
            for r in to_release:
                self.locks.pop(r, None)
            await self._broadcast({"type": "lock_update", "locks": self._locks_public()})

        return ws


# ---------------- Minimal embedded frontend ----------------

_STYLE_CSS = r"""
:root {
  --bg: #0b0f14;
  --panel: #121a22;
  --panel2: #0f1620;
  --text: #d9e2ef;
  --muted: #9db0c8;
  --accent: #4aa3ff;
  --danger: #ff4a4a;
  --ok: #39d98a;
  --border: rgba(255,255,255,0.10);
}

* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
  background: var(--bg);
  color: var(--text);
}

header {
  padding: 12px 16px;
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  gap: 12px;
}

header .title {
  font-weight: 700;
}

header .status {
  color: var(--muted);
  font-size: 13px;
}

main {
  display: grid;
  grid-template-columns: 1.4fr 0.6fr;
  gap: 12px;
  padding: 12px;
  height: calc(100vh - 56px);
}

@media (max-width: 980px) {
  main { grid-template-columns: 1fr; height: auto; }
}

.card {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 14px;
  overflow: hidden;
}

.card .card-hdr {
  padding: 10px 12px;
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.card .card-hdr .label { font-weight: 600; }
.card .card-hdr .small { color: var(--muted); font-size: 12px; }

.viewer {
  padding: 10px;
  display: grid;
  grid-template-rows: auto 1fr auto;
  gap: 10px;
}

.video-wrap {
  background: #000;
  border-radius: 12px;
  overflow: hidden;
  border: 1px solid var(--border);
  width: 100%;
  aspect-ratio: 16 / 9;
  display: grid;
  place-items: center;
}
video {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.controls {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 10px;
  padding: 8px 0;
}

.btn {
  border: 1px solid var(--border);
  background: var(--panel2);
  color: var(--text);
  padding: 12px 10px;
  border-radius: 12px;
  font-size: 14px;
  touch-action: manipulation;
  user-select: none;
}
.btn:active { transform: scale(0.99); }
.btn.primary { border-color: rgba(74,163,255,0.5); }
.btn.danger { border-color: rgba(255,74,74,0.5); }
.btn[disabled] {
  opacity: 0.5;
}

.grid {
  padding: 10px;
  display: grid;
  grid-template-rows: auto 1fr;
  gap: 10px;
}

.thumb-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 10px;
}

.thumb {
  background: #000;
  border: 1px solid var(--border);
  border-radius: 12px;
  overflow: hidden;
  position: relative;
  aspect-ratio: 16 / 10;
  cursor: pointer;
}

.thumb img {
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}

.badge {
  position: absolute;
  left: 8px;
  top: 8px;
  background: rgba(0,0,0,0.55);
  border: 1px solid rgba(255,255,255,0.15);
  padding: 4px 8px;
  border-radius: 999px;
  font-size: 12px;
}

.badge.locked { border-color: rgba(255,74,74,0.5); }
.badge.free { border-color: rgba(57,217,138,0.5); }
"""

_INDEX_HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Legion FPV</title>
  <link rel="stylesheet" href="/style.css" />
</head>
<body>
<header>
  <div class="title">Robot Legion FPV</div>
  <div class="status" id="status">Connecting…</div>
</header>

<main>
  <section class="card">
    <div class="card-hdr">
      <div>
        <div class="label">Active Robot</div>
        <div class="small" id="activeRobotLabel">(none)</div>
      </div>
      <button class="btn primary" id="switchBtn">Switch Robots</button>
    </div>

    <div class="viewer">
      <div class="video-wrap">
        <video id="mainVideo" autoplay playsinline muted></video>
      </div>

      <div class="controls">
        <button class="btn primary" data-cmd="fwd">Forward</button>
        <button class="btn danger" data-cmd="stop">Stop</button>
        <button class="btn primary" data-cmd="back">Backward</button>
        <button class="btn" data-cmd="left">Rotate Left</button>
        <button class="btn" data-cmd="zero">Hold</button>
        <button class="btn" data-cmd="right">Rotate Right</button>
      </div>

      <div class="small" style="color: var(--muted); font-size: 12px;">
        Tip: Use touch buttons on phones/tablets. The active robot is also synced with your keyboard teleop via <code>/active_robot</code>.
      </div>
    </div>
  </section>

  <aside class="card">
    <div class="card-hdr">
      <div>
        <div class="label">All Robots</div>
        <div class="small">Tap a tile to make it active (video switches automatically)</div>
      </div>
    </div>

    <div class="grid">
      <div class="thumb-grid" id="thumbGrid"></div>
    </div>
  </aside>
</main>

<script src="/app.js"></script>
</body>
</html>
"""

_APP_JS = r"""
let clientId = "c_" + Math.floor(Math.random() * 1e9);
let ws = null;

let robots = [];
let locks = {};
let activeRobot = null;

let pc = null;
let mainVideo = null;

function qs(id){ return document.getElementById(id); }

function setStatus(txt){ qs("status").textContent = txt; }
function setActiveLabel(txt){ qs("activeRobotLabel").textContent = txt || "(none)"; }

function isLockedByOther(robot){
  const owner = locks[robot];
  if (!owner) return false;
  return owner !== clientId;
}

async function fetchState(){
  const r = await fetch("/api/state");
  return await r.json();
}

function renderThumbs(){
  const grid = qs("thumbGrid");
  grid.innerHTML = "";

  for (const r of robots){
    const div = document.createElement("div");
    div.className = "thumb";

    const img = document.createElement("img");
    img.src = `/api/jpeg?robot=${encodeURIComponent(r)}&t=${Date.now()}`;
    img.alt = r;

    const badge = document.createElement("div");
    const locked = !!locks[r];
    badge.className = "badge " + (locked ? "locked" : "free");
    badge.textContent = locked ? `LOCKED` : `FREE`;

    const badge2 = document.createElement("div");
    badge2.className = "badge";
    badge2.style.top = "34px";
    badge2.textContent = r;

    if (r === activeRobot){
      div.style.outline = "2px solid rgba(74,163,255,0.65)";
      div.style.outlineOffset = "0px";
    }

    div.appendChild(img);
    div.appendChild(badge);
    div.appendChild(badge2);

    div.onclick = () => setActiveRobot(r);

    grid.appendChild(div);
  }
}

async function refreshThumbImages(){
  const imgs = document.querySelectorAll(".thumb img");
  for (const img of imgs){
    const u = new URL(img.src, window.location.origin);
    u.searchParams.set("t", Date.now().toString());
    img.src = u.toString();
  }
}

async function setupWebRTC(robot){
  if (!robot) return;

  // Close previous
  if (pc){
    try { pc.close(); } catch(e){}
    pc = null;
  }

  pc = new RTCPeerConnection();
  pc.ontrack = (evt) => {
    if (evt.track.kind === "video"){
      mainVideo.srcObject = evt.streams[0];
    }
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  const resp = await fetch("/webrtc/offer", {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify({ robot, sdp: pc.localDescription.sdp, type: pc.localDescription.type })
  });

  if (!resp.ok){
    setStatus("WebRTC offer failed: " + (await resp.text()));
    return;
  }

  const ans = await resp.json();
  await pc.setRemoteDescription(ans);
}

function wsSend(obj){
  if (ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify(obj));
  }
}

function setActiveRobot(robot){
  activeRobot = robot;
  setActiveLabel(robot);

  wsSend({ type: "set_active_robot", robot });

  // Main view uses WebRTC
  setupWebRTC(robot);

  renderThumbs();
}

function startHeartbeat(){
  setInterval(() => {
    if (!activeRobot) return;
    wsSend({ type: "heartbeat", robot: activeRobot });
  }, 1000);
}

function drive(lin, ang){
  if (!activeRobot) return;

  // If locked by someone else, UI will still show video but no driving
  if (isLockedByOther(activeRobot)){
    setStatus(`Robot '${activeRobot}' is locked by another user.`);
    return;
  }

  wsSend({ type: "drive", robot: activeRobot, lin, ang });
}

function bindControls(){
  const map = {
    fwd:  [0.35, 0.0],
    back: [-0.25, 0.0],
    left: [0.0,  1.2],
    right:[0.0, -1.2],
    stop: [0.0,  0.0],
    zero: [0.0,  0.0],
  };

  for (const btn of document.querySelectorAll("button[data-cmd]")){
    const cmd = btn.getAttribute("data-cmd");

    // Touch-friendly: press/hold to move, release to stop (except stop/zero)
    const onDown = (e) => {
      e.preventDefault();
      const v = map[cmd] || [0,0];
      drive(v[0], v[1]);
    };
    const onUp = (e) => {
      e.preventDefault();
      if (cmd !== "stop" && cmd !== "zero"){
        drive(0.0, 0.0);
      }
    };

    btn.addEventListener("pointerdown", onDown);
    btn.addEventListener("pointerup", onUp);
    btn.addEventListener("pointercancel", onUp);
    btn.addEventListener("pointerleave", onUp);
  }

  qs("switchBtn").onclick = () => {
    // simple prompt; scalable later (modal/search)
    const r = prompt("Enter robot name (matches /<robot>/cmd_vel):", activeRobot || "");
    if (r && r.trim()){
      setActiveRobot(r.trim());
    }
  };
}

async function main(){
  mainVideo = qs("mainVideo");

  setStatus("Loading state…");
  const s = await fetchState();
  robots = s.robots || [];
  locks = s.locks || {};
  activeRobot = s.active_robot || (robots[0] || null);

  setStatus("Connecting…");
  ws = new WebSocket(`ws://${window.location.host}/ws?client_id=${encodeURIComponent(clientId)}`);

  ws.onopen = () => {
    setStatus("Connected");
  };

  ws.onmessage = (evt) => {
    let data = null;
    try { data = JSON.parse(evt.data); } catch(e){ return; }

    if (data.type === "hello"){
      robots = data.robots || robots;
      locks = data.locks || locks;
      if (data.active_robot) activeRobot = data.active_robot;
      renderThumbs();
      if (activeRobot) setActiveRobot(activeRobot);
      return;
    }

    if (data.type === "lock_update"){
      locks = data.locks || {};
      renderThumbs();
      return;
    }

    if (data.type === "active_robot"){
      // teleop or other client changed it
      activeRobot = data.robot;
      setActiveLabel(activeRobot);
      setupWebRTC(activeRobot);
      renderThumbs();
      return;
    }

    if (data.type === "error"){
      setStatus(data.message || "Error");
      return;
    }
  };

  ws.onclose = () => setStatus("Disconnected");
  ws.onerror = () => setStatus("WebSocket error");

  bindControls();
  setActiveLabel(activeRobot);

  renderThumbs();
  if (activeRobot) await setupWebRTC(activeRobot);

  // refresh thumbs periodically
  setInterval(refreshThumbImages, 800);
  startHeartbeat();
}

main();
"""

# ---------------- Entrypoint ----------------

def _spin_ros_in_thread(node: Node):
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


async def main_async():
    rclpy.init(args=None)
    hub = RosFpvHub()

    # spin ROS in background thread
    t = threading.Thread(target=_spin_ros_in_thread, args=(hub,), daemon=True)
    t.start()

    server = FpvWebServer(hub)

    app = web.Application()
    app.add_routes([
        web.get("/", server.handle_index),
        web.get("/app.js", server.handle_app_js),
        web.get("/style.css", server.handle_style_css),

        web.get("/api/state", server.handle_api_state),
        web.get("/api/jpeg", server.handle_api_jpeg),

        web.post("/webrtc/offer", server.handle_webrtc_offer),
        web.get("/ws", server.handle_ws),
    ])

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host="0.0.0.0", port=8080)
    await site.start()

    hub.get_logger().info("FPV Web UI running on http://localhost:8080")

    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        # close webrtc pcs
        for pc in list(server.pcs):
            try:
                await pc.close()
            except Exception:
                pass

        hub.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()

