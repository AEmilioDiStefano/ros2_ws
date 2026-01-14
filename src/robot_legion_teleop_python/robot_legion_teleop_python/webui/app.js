let ws = null;
let clientId = null;

let robots = [];
let activeRobot = null;      // big screen
let controlledRobot = null;  // robot we have claimed

function $(id){ return document.getElementById(id); }

function setConnStatus(text){
  $("connStatus").textContent = text;
}

function wsSend(obj){
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify(obj));
}

function buildTile(robotInfo){
  const tile = document.createElement("div");
  tile.className = "tile";
  tile.dataset.robot = robotInfo.robot;

  const img = document.createElement("img");
  img.src = `/stream/${robotInfo.robot}?t=${Date.now()}`; // cache-buster
  img.loading = "lazy";

  const meta = document.createElement("div");
  meta.className = "meta";

  const nameRow = document.createElement("div");
  nameRow.className = "nameRow";

  const rn = document.createElement("div");
  rn.className = "robotName";
  rn.textContent = robotInfo.robot;

  const badge = document.createElement("div");
  badge.className = "badge " + (robotInfo.locked ? "locked" : "free");
  badge.textContent = robotInfo.locked ? "CONTROLLED" : "IDLE";

  nameRow.appendChild(rn);
  nameRow.appendChild(badge);

  const sub = document.createElement("div");
  sub.className = "sub";

  if (!robotInfo.has_video && !robotInfo.has_control){
    sub.textContent = "no video, no control";
  } else if (!robotInfo.has_video){
    sub.textContent = "no video";
  } else if (!robotInfo.has_control){
    sub.textContent = "video only";
  } else {
    sub.textContent = robotInfo.locked ? `by ${robotInfo.controller}` : "tap/click to control";
  }

  meta.appendChild(nameRow);
  meta.appendChild(sub);

  tile.appendChild(img);
  tile.appendChild(meta);

  // Styling
  if (robotInfo.locked) tile.classList.add("locked");
  if (robotInfo.robot === activeRobot) tile.classList.add("active");

  // Touch/click to claim & focus
  tile.addEventListener("click", () => {
    activeRobot = robotInfo.robot;
    updateBigVideo();

    // Claim only if not locked by someone else
    if (robotInfo.locked && robotInfo.controller && controlledRobot !== robotInfo.robot){
      // Can't steal control
      return;
    }
    wsSend({type:"claim", robot: robotInfo.robot});
  });

  return tile;
}

function renderTiles(){
  const tiles = $("tiles");
  tiles.innerHTML = "";
  for (const r of robots){
    tiles.appendChild(buildTile(r));
  }
}

function updateBigVideo(){
  const bigImg = $("bigImg");
  const bigLabel = $("bigLabel");

  if (!activeRobot){
    bigImg.removeAttribute("src");
    bigLabel.textContent = "No robot selected";
    return;
  }
  bigLabel.textContent = `Active video: ${activeRobot}` + (controlledRobot === activeRobot ? " (YOU CONTROL)" : "");
  bigImg.src = `/stream/${activeRobot}?t=${Date.now()}`;
}

function applyStatus(payload){
  robots = payload.robots || [];
  if (!activeRobot){
    activeRobot = payload.suggested_active || null;
  }
  renderTiles();
  updateBigVideo();
}

function hookNumpad(){
  const pads = document.querySelectorAll(".pad");
  for (const b of pads){
    const code = b.dataset.cmd;
    const sendStop = () => sendCmd(0, 0);

    const down = (ev) => {
      ev.preventDefault();
      if (!controlledRobot) return;
      const [lin, ang] = keyToTwist(code);
      sendCmd(lin, ang);
    };
    const up = (ev) => {
      ev.preventDefault();
      if (!controlledRobot) return;
      sendStop();
    };

    // pointer events cover mouse + touch
    b.addEventListener("pointerdown", down);
    b.addEventListener("pointerup", up);
    b.addEventListener("pointercancel", up);
    b.addEventListener("pointerleave", up);
  }

  $("switchBtn").addEventListener("click", () => {
    // Just prompt user with a simple selection list
    const free = robots.filter(r => !r.locked || r.robot === controlledRobot).map(r => r.robot);
    const choice = window.prompt("Enter robot name to control:\n\n" + free.join("\n"));
    if (!choice) return;
    activeRobot = choice.trim();
    updateBigVideo();
    wsSend({type:"claim", robot: activeRobot});
  });
}

function keyToTwist(code){
  // Numpad mapping requested:
  // 7 circle left fwd, 8 fwd, 9 circle right fwd
  // 4 rotate left, 5 stop, 6 rotate right
  // 1 circle left back, 2 back, 3 circle right back
  //
  // We'll express as Twist (lin, ang) and let motor_driver_node do diff-drive.
  // Circle = reduced angular + linear mix.
  const L = 0.45; // lin magnitude (server clamps anyway)
  const A = 1.2;  // ang magnitude (server clamps anyway)

  switch(code){
    case "8": return [ +L, 0 ];
    case "2": return [ -L, 0 ];
    case "4": return [ 0, +A ];
    case "6": return [ 0, -A ];
    case "5": return [ 0, 0 ];

    case "7": return [ +L, +A*0.7 ];
    case "9": return [ +L, -A*0.7 ];
    case "1": return [ -L, +A*0.7 ];
    case "3": return [ -L, -A*0.7 ];
    default: return [0,0];
  }
}

function sendCmd(lin, ang){
  if (!controlledRobot) return;
  wsSend({type:"cmd", robot: controlledRobot, lin, ang});
}

function startHeartbeat(){
  setInterval(() => {
    if (!controlledRobot) return;
    wsSend({type:"heartbeat", robot: controlledRobot});
  }, 700);
}

function connectWS(){
  const proto = (location.protocol === "https:") ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    setConnStatus("connected");
  };

  ws.onclose = () => {
    setConnStatus("disconnected");
    clientId = null;
    controlledRobot = null;
    setTimeout(connectWS, 1000);
  };

  ws.onmessage = (evt) => {
    let msg = null;
    try { msg = JSON.parse(evt.data); } catch { return; }

    if (msg.type === "hello"){
      clientId = msg.client_id;
      return;
    }

    if (msg.type === "status"){
      applyStatus(msg.data || {});
      return;
    }

    if (msg.type === "claim_result"){
      if (msg.ok){
        controlledRobot = msg.robot;
        activeRobot = msg.robot;
        updateBigVideo();
      } else {
        // failed claim - keep UI but no control
      }
      return;
    }
  };
}

function boot(){
  hookNumpad();
  connectWS();
  startHeartbeat();

  $("setNameBtn").addEventListener("click", () => {
    const dn = $("nameInput").value.trim();
    if (dn) wsSend({type:"set_name", display_name: dn});
  });
}

boot();
