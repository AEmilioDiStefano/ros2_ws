// Robot Legion FPV UI (ROS-native)
// - Video: web_video_server on :8080
// - Control/status: rosbridge_server on :9090 (roslibjs)

let ros = null;

// Avoid leaking MJPEG connections/timers
let tileTimers = new Map();
let currentBigSrc = "";

let robots = [];
let activeRobot = null;      // big screen
let controlledRobot = null;  // robot we have claimed

// Stable per-tab client identity
const CLIENT_ID_KEY = "robot_legion_client_id";
let clientId = localStorage.getItem(CLIENT_ID_KEY);
if (!clientId) {
  clientId = (crypto.randomUUID ? crypto.randomUUID() : String(Math.random()).slice(2));
  localStorage.setItem(CLIENT_ID_KEY, clientId);
}

let displayName = localStorage.getItem("robot_legion_display_name") || "";

let statusSub = null;
let claimPub = null;
let releasePub = null;
let cmdPub = null;
let heartbeatPub = null;

function $(id){ return document.getElementById(id); }

function rosbridgeUrl(){
  // If you later host this remotely, you can change this to wss://yourdomain/ws etc.
  const host = location.hostname;
  return `ws://${host}:9090`;
}

function setConnStatus(text){
  const el = $("connStatus");
  if (el) el.textContent = text;
}

function setActiveRobotText(){
  const el = $("activeRobotText");
  if (!el) return;
  el.textContent = activeRobot ? activeRobot : "none";
}

function streamUrlForRobot(robot){
  // IMPORTANT: web_video_server expects literal slashes in the topic query param.
  // encodeURIComponent() breaks this by turning "/" into "%2F", which web_video_server rejects.
  const topic = `/${robot}/camera/image_raw`;
  const safeTopic = encodeURI(topic); // preserves "/"
  return `http://${location.hostname}:8080/stream?topic=${safeTopic}`;
}

function snapshotUrlForRobot(robot){
  // For thumbnails: use snapshot endpoint (cheap) and cache-bust so the browser refreshes.
  const topic = `/${robot}/camera/image_raw`;
  const safeTopic = encodeURI(topic);
  return `http://${location.hostname}:8080/snapshot?topic=${safeTopic}&_=${Date.now()}`;
}

function updateBigVideo(){
  const bigImg = $("bigImg");
  const bigLabel = $("bigLabel");
  if (!bigImg || !bigLabel) return;

  if (!activeRobot){
    // Close any existing MJPEG connection.
    if (bigImg.getAttribute("src")) bigImg.setAttribute("src", "");
    bigImg.removeAttribute("src");
    currentBigSrc = "";
    bigLabel.textContent = "No robot selected";
    return;
  }

  const you = (controlledRobot === activeRobot) ? " (YOU CONTROL)" : "";
  bigLabel.textContent = `Active video: ${activeRobot}${you}`;

  const desired = streamUrlForRobot(activeRobot);
  if (desired !== currentBigSrc){
    // Close the previous MJPEG connection before opening a new one.
    bigImg.setAttribute("src", "");
    bigImg.src = desired;
    currentBigSrc = desired;
  }
}

function buildTile(robotInfo){
  const tile = document.createElement("div");
  tile.className = "tile";
  tile.dataset.robot = robotInfo.robot;

  const img = document.createElement("img");
  // Use snapshots for tiles to avoid overloading web_video_server with multiple MJPEG streams.
  img.src = snapshotUrlForRobot(robotInfo.robot);
  img.loading = "lazy";

  const meta = document.createElement("div");
  meta.className = "meta";

  const nameRow = document.createElement("div");
  nameRow.className = "nameRow";

  const rn = document.createElement("div");
  rn.className = "robotName";
  rn.textContent = robotInfo.robot;

  const badge = document.createElement("div");
  const locked = !!robotInfo.claimed_by;
  badge.className = "badge " + (locked ? "locked" : "free");
  badge.textContent = locked ? "CONTROLLED" : "IDLE";

  nameRow.appendChild(rn);
  nameRow.appendChild(badge);

  const sub = document.createElement("div");
  sub.className = "sub";
  if (!robotInfo.has_video && !robotInfo.controllable) sub.textContent = "no video, no control";
  else if (!robotInfo.has_video) sub.textContent = "no video";
  else if (!robotInfo.controllable) sub.textContent = "video only";
  else if (locked) sub.textContent = `by ${robotInfo.claimed_by.display_name || "someone"}`;
  else sub.textContent = "tap/click to control";

  meta.appendChild(nameRow);
  meta.appendChild(sub);

  tile.appendChild(img);
  tile.appendChild(meta);

  // Refresh thumbnail snapshots at a low rate (keeps UI responsive and avoids MJPEG overload).
  // 1 Hz is a good starting point over Wi-Fi; increase to 2–3 Hz if you have headroom.
  const timer = setInterval(() => {
    // Only refresh if the tile is still attached (defensive)
    if (!tile.isConnected) return;
    img.src = snapshotUrlForRobot(robotInfo.robot);
  }, 1000);
  tileTimers.set(robotInfo.robot, timer);

  if (robotInfo.robot === activeRobot) tile.classList.add("active");
  if (locked && robotInfo.claimed_by.client_id !== clientId) tile.classList.add("locked");

  tile.addEventListener("click", () => {
    activeRobot = robotInfo.robot;
    setActiveRobotText();
    updateBigVideo();

    // Don't allow stealing control
    if (locked && robotInfo.claimed_by.client_id !== clientId) return;
    claimRobot(robotInfo.robot);
  });

  return tile;
}

function renderTiles(){
  const tiles = $("tiles");
  if (!tiles) return;

  // Clear any existing thumbnail refresh timers to avoid leaks.
  for (const [, timer] of tileTimers.entries()){
    clearInterval(timer);
  }
  tileTimers.clear();

  tiles.innerHTML = "";
  for (const r of robots){
    tiles.appendChild(buildTile(r));
  }
}

function applyStatus(payload){
  robots = payload.robots || [];

  // If we haven't picked a robot yet, prefer "mine" (if claimed), else first with video
  if (!activeRobot){
    const mine = robots.find(r => r.claimed_by && r.claimed_by.client_id === clientId);
    if (mine) activeRobot = mine.robot;
    if (!activeRobot){
      const firstVid = robots.find(r => r.has_video);
      activeRobot = firstVid ? firstVid.robot : null;
    }
  }

  // Keep controlledRobot aligned with reality
  const mineNow = robots.find(r => r.claimed_by && r.claimed_by.client_id === clientId);
  controlledRobot = mineNow ? mineNow.robot : null;

  // If activeRobot disappears, pick another
  if (activeRobot && !robots.find(r => r.robot === activeRobot)){
    const firstVid = robots.find(r => r.has_video);
    activeRobot = firstVid ? firstVid.robot : null;
  }

  setActiveRobotText();
  renderTiles();
  updateBigVideo();
}

function publish(topicObj, msg){
  try { topicObj.publish(new ROSLIB.Message(msg)); }
  catch(e){ console.warn("publish failed:", e); }
}

function claimRobot(robot){
  if (!claimPub) return;
  publish(claimPub, {
    client_id: clientId,
    display_name: displayName,
    robot: robot
  });
}

function releaseRobot(robot){
  if (!releasePub) return;
  publish(releasePub, {
    client_id: clientId,
    robot: robot
  });
}

function sendCmd(linear, angular){
  if (!cmdPub) return;
  if (!controlledRobot) return;

  publish(cmdPub, {
    client_id: clientId,
    robot: controlledRobot,
    linear: linear,
    angular: angular
  });
}

function startHeartbeatLoop(){
  // Keep-alive / presence for arbiter (helps it release control if browser disappears)
  setInterval(() => {
    if (!heartbeatPub) return;
    publish(heartbeatPub, {
      client_id: clientId,
      display_name: displayName
    });
  }, 1000);
}

function hookNumpad(){
  const bind = (id, lin, ang) => {
    const el = $(id);
    if (!el) return;
    const press = (e) => { e.preventDefault(); sendCmd(lin, ang); };
    el.addEventListener("mousedown", press);
    el.addEventListener("touchstart", press, { passive: false });
  };

  // Numpad-ish layout:
  // 7 8 9 -> fwd-left, fwd, fwd-right
  // 4 5 6 -> rot-left, stop, rot-right
  // 1 2 3 -> back-left, back, back-right
  bind("btn7",  0.4,  0.8);
  bind("btn8",  0.6,  0.0);
  bind("btn9",  0.4, -0.8);

  bind("btn4",  0.0,  1.0);
  bind("btn5",  0.0,  0.0);
  bind("btn6",  0.0, -1.0);

  bind("btn1", -0.4,  0.8);
  bind("btn2", -0.6,  0.0);
  bind("btn3", -0.4, -0.8);

  const switchBtn = $("btnSwitch");
  if (switchBtn){
    switchBtn.addEventListener("click", () => {
      // Only switch selection (claim happens on tile click)
      activeRobot = null;
      updateBigVideo();
      setActiveRobotText();
    });
  }
}

function hookNameSetter(){
  const input = $("displayName");
  const setBtn = $("setNameBtn");
  if (input) input.value = displayName;

  if (!setBtn || !input) return;

  setBtn.addEventListener("click", () => {
    const dn = input.value.trim();
    displayName = dn ? dn.slice(0, 40) : "";
    localStorage.setItem("robot_legion_display_name", displayName);
  });
}

function connectRosbridge(){
  setConnStatus("connecting...");
  ros = new ROSLIB.Ros({ url: rosbridgeUrl() });

  ros.on("connection", () => {
    setConnStatus("connected");

    claimPub = new ROSLIB.Topic({ ros, name: "/fpv/claim_req", messageType: "robot_legion_teleop_python/msg/FpvClaimReq" });
    releasePub = new ROSLIB.Topic({ ros, name: "/fpv/release_req", messageType: "robot_legion_teleop_python/msg/FpvReleaseReq" });
    cmdPub = new ROSLIB.Topic({ ros, name: "/fpv/cmd_req", messageType: "robot_legion_teleop_python/msg/FpvCmdReq" });
    heartbeatPub = new ROSLIB.Topic({ ros, name: "/fpv/heartbeat", messageType: "robot_legion_teleop_python/msg/FpvHeartbeat" });

    statusSub = new ROSLIB.Topic({ ros, name: "/fpv/status", messageType: "std_msgs/msg/String" });

    statusSub.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        if (payload && payload.type === "status") applyStatus(payload);
      } catch(e){
        console.warn("bad status payload:", e);
      }
    });
  });

  ros.on("error", (e) => {
    console.warn("rosbridge error:", e);
    setConnStatus("error");
  });

  ros.on("close", () => {
    setConnStatus("disconnected");
  });
}

function boot(){
  hookNumpad();
  hookNameSetter();
  connectRosbridge();
  startHeartbeatLoop();

  window.addEventListener("beforeunload", () => {
    if (controlledRobot) releaseRobot(controlledRobot);
  });
}

boot();
