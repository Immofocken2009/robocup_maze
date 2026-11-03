"use strict";

const $ = (id) => document.getElementById(id);
const canvas = $("maze");
const ctx = canvas.getContext("2d");

let ws = null;
let world = null;     // das echte Labyrinth (kommt einmal pro geladenem Labyrinth)
let state = null;     // aktueller Stand der Simulation (kommt nach jeder Aktion)
let control = null;   // läuft / pausiert / fertig, Geschwindigkeit
let settingsLoaded = false;

let T = 40;           // Pixel pro Kachel
const PAD = 10;

const FLOOR_NAMES = { normal: "weiß", black: "schwarz", blue: "blau", silver: "silber", red: "rot" };
const RED_MODE_NAMES = { zuletzt: "zuletzt erkunden", meiden: "meiden", normal: "normal erkunden" };
const SPEED_MIN = 0.5, SPEED_MAX = 100;
const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

// --- Verbindung --------------------------------------------------------------

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 1000); };
  ws.onmessage = (ev) => handle(JSON.parse(ev.data));
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

function setConn(ok) {
  const el = $("conn");
  el.textContent = ok ? "Verbunden" : "Keine Verbindung zum Pi, versuche erneut";
  el.className = "conn " + (ok ? "ok" : "off");
}

function handle(msg) {
  if (msg.type === "settings") {
    applySettings(msg);
  } else if (msg.type === "world") {
    world = msg;
    state = null;
    anim.to = null;
    updateWorldInfo();
    resize();
  } else if (msg.type === "state") {
    moveRobot(msg);
    state = msg;
    updatePanels();
  } else if (msg.type === "control") {
    control = msg;
    updateControls();
  }
  requestDraw();
}

// --- Roboter-Animation ---------------------------------------------------------

const anim = { from: null, to: null, start: 0, dur: 0 };

function moveRobot(msg) {
  const r = msg.robot;
  if (!anim.to) {
    anim.to = { x: r.x, y: r.y, dir: r.dir, angle: r.dir };
    anim.from = anim.to;
    anim.dur = 0;
    return;
  }
  if (r.x === anim.to.x && r.y === anim.to.y && r.dir === anim.to.dir) return;
  let turn = ((r.dir - anim.to.dir) % 4 + 4) % 4;   // kürzeste Drehung
  if (turn > 2) turn -= 4;
  anim.from = robotPose(performance.now());
  anim.to = { x: r.x, y: r.y, dir: r.dir, angle: anim.to.angle + turn };
  anim.start = performance.now();
  anim.dur = reducedMotion.matches ? 0 : msg.anim_ms;
}

function animProgress(now) {
  return anim.dur > 0 ? Math.min(1, (now - anim.start) / anim.dur) : 1;
}

function robotPose(now) {
  const t = animProgress(now);
  const e = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
  const lerp = (a, b) => a + (b - a) * e;
  return {
    x: lerp(anim.from.x, anim.to.x),
    y: lerp(anim.from.y, anim.to.y),
    angle: lerp(anim.from.angle, anim.to.angle),
  };
}

// --- Zeichnen ------------------------------------------------------------------

let colors = null;

function readColors() {
  const css = getComputedStyle(document.documentElement);
  const v = (name) => css.getPropertyValue(name).trim();
  colors = {
    tile: v("--tile"), unknown: v("--unknown"), wall: v("--wall"), truth: v("--truth"),
    robot: v("--robot"), robotMark: v("--robot-mark"), path: v("--path"),
    frontier: v("--frontier"), muted: v("--muted"),
    floors: { normal: v("--tile"), black: v("--black"), blue: v("--blue"),
              silver: v("--silver"), red: v("--red") },
  };
}

const px = (x) => PAD + x * T;
const py = (y) => PAD + (world.height - 1 - y) * T;
const key = (x, y) => `${x},${y}`;

function resize() {
  if (!world) return;
  const availW = canvas.parentElement.clientWidth;
  const availH = Math.max(320, window.innerHeight - 220);
  T = Math.max(18, Math.floor(Math.min((availW - 2 * PAD) / world.width,
                                       (availH - 2 * PAD) / world.height, 84)));
  const w = world.width * T + 2 * PAD;
  const h = world.height * T + 2 * PAD;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  canvas.style.width = w + "px";
  canvas.style.height = h + "px";
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  requestDraw();
}

let drawQueued = false;
function requestDraw() {
  if (!drawQueued) {
    drawQueued = true;
    requestAnimationFrame(draw);
  }
}

function draw(now) {
  drawQueued = false;
  if (!world) return;
  if (!colors) readColors();
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const showTruth = $("show-truth").checked;
  const known = new Map((state ? state.known_tiles : []).map((t) => [key(t.x, t.y), t]));

  // Kacheln: bekannte in ihrer Farbe, unbekannte grau (echter Boden blass, wenn eingeblendet)
  for (const t of world.tiles) {
    const k = known.get(key(t.x, t.y));
    ctx.fillStyle = k ? colors.floors[k.floor] : colors.unknown;
    ctx.fillRect(px(t.x), py(t.y), T, T);
    if (!k && showTruth && t.floor !== "normal") {
      ctx.globalAlpha = 0.3;
      ctx.fillStyle = colors.floors[t.floor];
      ctx.fillRect(px(t.x), py(t.y), T, T);
      ctx.globalAlpha = 1;
    }
  }

  // Kachelfugen
  ctx.strokeStyle = colors.truth;
  ctx.globalAlpha = 0.3;
  ctx.lineWidth = 1;
  for (const t of world.tiles) ctx.strokeRect(px(t.x) + 0.5, py(t.y) + 0.5, T - 1, T - 1);
  ctx.globalAlpha = 1;

  // Start
  ctx.fillStyle = colors.muted;
  ctx.font = `600 ${Math.round(T * 0.3)}px system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("S", px(world.start[0]) + T / 2, py(world.start[1]) + T / 2);

  // Grenzkacheln
  if (state) {
    ctx.strokeStyle = colors.frontier;
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    for (const [x, y] of state.frontier) ctx.strokeRect(px(x) + 5, py(y) + 5, T - 10, T - 10);
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  // Wände: echte dünn, erkannte als Holzwand
  ctx.lineCap = "round";
  if (showTruth) {
    ctx.strokeStyle = colors.truth;
    ctx.lineWidth = 1.5;
    for (const t of world.tiles) for (const d of t.walls) drawWall(t.x, t.y, d);
  }
  if (state) {
    ctx.strokeStyle = colors.wall;
    ctx.lineWidth = Math.max(3, T / 9);
    for (const w of state.known_walls) if (w.wall) drawWall(w.x, w.y, w.dir);
  }

  const pose = anim.to ? robotPose(now) : null;

  // Geplanter Weg
  if (state && pose && state.planned.length > 1) {
    ctx.strokeStyle = colors.path;
    ctx.fillStyle = colors.path;
    ctx.lineWidth = Math.max(2, T / 16);
    ctx.lineJoin = "round";
    ctx.beginPath();
    ctx.moveTo(px(pose.x) + T / 2, py(pose.y) + T / 2);
    for (const [x, y] of state.planned.slice(1)) ctx.lineTo(px(x) + T / 2, py(y) + T / 2);
    ctx.stroke();
    const [ex, ey] = state.planned[state.planned.length - 1];
    ctx.beginPath();
    ctx.arc(px(ex) + T / 2, py(ey) + T / 2, T * 0.09, 0, Math.PI * 2);
    ctx.fill();
  }

  // Roboter
  if (pose) {
    const r = T * 0.3;
    ctx.save();
    ctx.translate(px(pose.x) + T / 2, py(pose.y) + T / 2);
    ctx.rotate(pose.angle * Math.PI / 2);
    ctx.fillStyle = colors.robot;
    ctx.beginPath();
    ctx.roundRect(-r, -r, 2 * r, 2 * r, r * 0.25);
    ctx.fill();
    ctx.fillStyle = colors.robotMark;
    ctx.beginPath();
    ctx.moveTo(0, -r * 0.7);
    ctx.lineTo(r * 0.5, r * 0.35);
    ctx.lineTo(-r * 0.5, r * 0.35);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
    if (animProgress(now) < 1) requestDraw();
  }
}

function drawWall(x, y, dir) {
  const X = px(x), Y = py(y);
  const seg = {
    N: [X, Y, X + T, Y],
    S: [X, Y + T, X + T, Y + T],
    W: [X, Y, X, Y + T],
    E: [X + T, Y, X + T, Y + T],
  }[dir];
  ctx.beginPath();
  ctx.moveTo(seg[0], seg[1]);
  ctx.lineTo(seg[2], seg[3]);
  ctx.stroke();
}

// --- Seitenspalte --------------------------------------------------------------

function fmtTime(s) {
  return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

function updatePanels() {
  const s = state;
  $("st-mode").textContent = s.mode.charAt(0).toUpperCase() + s.mode.slice(1);
  $("st-time").textContent = fmtTime(s.time);
  $("st-limit").textContent = fmtTime(s.time_limit);
  const ratio = Math.min(1, s.time / s.time_limit);
  $("timebar-fill").style.width = `${ratio * 100}%`;
  $("timebar-fill").classList.toggle("low", ratio > 0.85);
  $("st-visited").textContent = `${s.stats.visited} von ${s.stats.reachable} Kacheln`;
  $("st-actions").textContent = s.stats.actions;
  $("st-action").textContent = s.last_action;
  $("st-checkpoint").textContent = s.checkpoint ? `${s.checkpoint[0]}, ${s.checkpoint[1]}` : "–";

  for (const part of ["checkpoints", "blue", "reliability", "exit", "total"]) {
    $(`sc-${part}`).textContent = s.score[part];
  }
  $("sc-lop").textContent = s.score.lop ? `${s.score.lop}× Lack of Progress` : "";

  for (const side of ["front", "left", "right"]) {
    const v = s.sensors[side];
    $(`se-${side}`).textContent = v === null ? "außer Reichweite" : `${v.toFixed(1)} cm`;
    $(`sw-${side}`).classList.toggle("on", s.sensors.walls[side]);
  }
  if (!colors) readColors();
  $("se-swatch").style.background = colors.floors[s.sensors.color];
  $("se-color").textContent = FLOOR_NAMES[s.sensors.color];
  $("se-gyro").textContent = `${s.sensors.gyro}°`;

  $("events").replaceChildren(...s.events.slice().reverse().map((text) => {
    const li = document.createElement("li");
    li.textContent = text;
    return li;
  }));
}

function updateWorldInfo() {
  const parts = [world.name];
  if (world.seed !== null && world.seed !== undefined) parts.push(`Seed ${world.seed}`);
  parts.push(`${world.width} × ${world.height}`, `${world.reachable} erreichbare Kacheln`);
  if (world.floating) parts.push(`${world.floating} davon floating`);
  $("maze-info").textContent = parts.join(", ") + (world.saved ? "" : " (nicht gespeichert)");
  $("btn-save").disabled = world.saved;
  $("notes").replaceChildren(...world.notes.map((text) => {
    const li = document.createElement("li");
    li.textContent = text;
    return li;
  }));
}

function applySettings(s) {
  if (settingsLoaded) return;
  settingsLoaded = true;
  const form = $("gen-form");
  for (const [name, value] of Object.entries(s.defaults)) form.elements[name].value = value;
  form.elements.width.max = s.max_size;
  form.elements.height.max = s.max_size;
  $("sizes").replaceChildren(...s.field_sizes.map(([w, h]) => {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = `${w} × ${h}`;
    b.title = "Feldgröße eines offiziellen Felds 2026";
    b.addEventListener("click", () => {
      form.elements.width.value = w;
      form.elements.height.value = h;
    });
    return b;
  }));
  $("red-mode").replaceChildren(...s.red_modes.map((m) => new Option(RED_MODE_NAMES[m] || m, m)));
}

const sliderToSpeed = (v) => SPEED_MIN * Math.pow(SPEED_MAX / SPEED_MIN, v / 100);
const speedToSlider = (s) => 100 * Math.log(s / SPEED_MIN) / Math.log(SPEED_MAX / SPEED_MIN);
const fmtSpeed = (s) => (s < 10 ? s.toFixed(1) : Math.round(s)) + "×";

function updateControls() {
  const sel = $("maze-select");
  const names = control.mazes.join();
  if (sel.dataset.names !== names) {
    sel.replaceChildren(new Option("– auswählen –", ""), ...control.mazes.map((n) => new Option(n, n)));
    sel.dataset.names = names;
  }
  sel.value = control.mazes.includes(control.current) ? control.current : "";

  $("btn-start").disabled = control.running || control.finished;
  $("btn-start").textContent = control.started && !control.finished ? "Weiter" : "Start";
  $("btn-pause").disabled = !control.running;
  $("btn-step").disabled = control.finished;

  const red = $("red-mode");
  red.value = control.red_mode;
  red.disabled = control.started;
  red.title = control.started ? "Erst nach „Neu starten“ änderbar" : "";

  const slider = $("speed");
  if (document.activeElement !== slider) slider.value = speedToSlider(control.speed);
  $("speed-label").textContent = fmtSpeed(control.speed);
}

// --- Bedienung -----------------------------------------------------------------

$("btn-start").addEventListener("click", () => send({ cmd: "start" }));
$("btn-pause").addEventListener("click", () => send({ cmd: "pause" }));
$("btn-step").addEventListener("click", () => send({ cmd: "step" }));
$("btn-reset").addEventListener("click", () => send({ cmd: "reset" }));
$("btn-save").addEventListener("click", () => send({ cmd: "save" }));
$("red-mode").addEventListener("change", (e) => send({ cmd: "red_mode", value: e.target.value }));
$("maze-select").addEventListener("change", (e) => {
  if (e.target.value) send({ cmd: "load", maze: e.target.value });
});
$("gen-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const data = Object.fromEntries(new FormData(e.target));
  for (const k of ["width", "height", "black", "blue", "silver", "zones"]) data[k] = Number(data[k]);
  data.seed = data.seed.trim() === "" ? null : Number(data.seed);
  send({ cmd: "generate", ...data });
});
$("speed").addEventListener("input", (e) => {
  const speed = sliderToSpeed(Number(e.target.value));
  $("speed-label").textContent = fmtSpeed(speed);
  send({ cmd: "speed", value: speed });
});
$("show-truth").addEventListener("change", requestDraw);
window.addEventListener("resize", resize);
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => {
  readColors();
  requestDraw();
});

connect();
