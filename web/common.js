"use strict";
// Datei: common.js – Gemeinsames: Hilfsfunktionen, Verbindung, Farben, Kartenzeichnung (MapCanvas, PoseAnim)

const $ = (id) => document.getElementById(id);

const PAD = 10;
const FLOOR_NAMES = { normal: "weiß", black: "schwarz", blue: "blau", silver: "silber", red: "rot" };
const DIR_NAMES = ["Nord", "Ost", "Süd", "West"];
const DIR_STEP = [[0, 1], [1, 0], [0, -1], [-1, 0]];
const RED_MODE_NAMES = { zuletzt: "zuletzt erkunden", meiden: "meiden", normal: "normal erkunden" };
const SPEED_MIN = 0.5, SPEED_MAX = 100;
const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

let ws = null;
let view = "robot";

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
  if (!ok) robot.setLink(false);
}

function handle(msg) {
  if (msg.type === "settings") sim.applySettings(msg);
  else if (msg.type === "world") sim.setWorld(msg);
  else if (msg.type === "state") sim.setState(msg);
  else if (msg.type === "control") sim.setControl(msg);
  else if (msg.type === "robot_link") robot.setLink(msg.connected);
  else if (msg.type === "robot") robot.setState(msg);
}

// --- Farben --------------------------------------------------------------------

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

// --- Kartenzeichnung (für Roboter und Simulation) -------------------------------

class MapCanvas {
  constructor(canvas, draw) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.drawFn = draw;
    this.T = 40;
    this.bounds = null;     // {x0, y0, w, h} in Kacheln
    this.queued = false;
  }

  setBounds(x0, y0, w, h) {
    const b = this.bounds;
    if (b && b.x0 === x0 && b.y0 === y0 && b.w === w && b.h === h) return;
    this.bounds = { x0, y0, w, h };
    this.resize();
  }

  resize() {
    const b = this.bounds;
    const availW = this.canvas.parentElement.clientWidth;
    if (!b || availW === 0) return;     // Ansicht gerade versteckt
    const availH = Math.max(320, window.innerHeight - 260);
    this.T = Math.max(18, Math.floor(Math.min((availW - 2 * PAD) / b.w, (availH - 2 * PAD) / b.h, 84)));
    const w = b.w * this.T + 2 * PAD;
    const h = b.h * this.T + 2 * PAD;
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = w * dpr;
    this.canvas.height = h * dpr;
    this.canvas.style.width = w + "px";
    this.canvas.style.height = h + "px";
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.request();
  }

  request() {
    if (this.queued) return;
    this.queued = true;
    requestAnimationFrame((now) => {
      this.queued = false;
      if (!this.bounds) return;
      if (!colors) readColors();
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawFn(this, now);
    });
  }

  px(x) { return PAD + (x - this.bounds.x0) * this.T; }
  py(y) { return PAD + (this.bounds.y0 + this.bounds.h - 1 - y) * this.T; }
  // stufenlose Koordinaten in Kacheln, (0, 0) = Ecke unten links
  cx(x) { return PAD + (x - this.bounds.x0) * this.T; }
  cy(y) { return PAD + (this.bounds.y0 + this.bounds.h - y) * this.T; }

  beams(list) {
    const c = this.ctx;
    c.strokeStyle = colors.muted;
    c.lineWidth = 1;
    c.setLineDash([3, 3]);
    for (const b of list) {
      c.globalAlpha = b.echo ? 0.8 : 0.3;
      c.beginPath();
      c.moveTo(this.cx(b.x0), this.cy(b.y0));
      c.lineTo(this.cx(b.x1), this.cy(b.y1));
      c.stroke();
    }
    c.setLineDash([]);
    c.globalAlpha = 1;
  }

  tile(x, y, color, alpha = 1) {
    const c = this.ctx;
    c.globalAlpha = alpha;
    c.fillStyle = color;
    c.fillRect(this.px(x), this.py(y), this.T, this.T);
    c.globalAlpha = 1;
  }

  joints(tiles) {
    const c = this.ctx, T = this.T;
    c.strokeStyle = colors.truth;
    c.globalAlpha = 0.3;
    c.lineWidth = 1;
    for (const [x, y] of tiles) c.strokeRect(this.px(x) + 0.5, this.py(y) + 0.5, T - 1, T - 1);
    c.globalAlpha = 1;
  }

  label(x, y, text) {
    const c = this.ctx, T = this.T;
    c.fillStyle = colors.muted;
    c.font = `600 ${Math.round(T * 0.3)}px system-ui, sans-serif`;
    c.textAlign = "center";
    c.textBaseline = "middle";
    c.fillText(text, this.px(x) + T / 2, this.py(y) + T / 2);
  }

  frontier(list) {
    const c = this.ctx, T = this.T;
    c.strokeStyle = colors.frontier;
    c.globalAlpha = 0.55;
    c.lineWidth = 1.5;
    c.setLineDash([4, 3]);
    for (const [x, y] of list) c.strokeRect(this.px(x) + 5, this.py(y) + 5, T - 10, T - 10);
    c.setLineDash([]);
    c.globalAlpha = 1;
  }

  wall(x, y, dir) {
    const X = this.px(x), Y = this.py(y), T = this.T;
    const seg = {
      N: [X, Y, X + T, Y],
      S: [X, Y + T, X + T, Y + T],
      W: [X, Y, X, Y + T],
      E: [X + T, Y, X + T, Y + T],
    }[dir];
    const c = this.ctx;
    c.beginPath();
    c.moveTo(seg[0], seg[1]);
    c.lineTo(seg[2], seg[3]);
    c.stroke();
  }

  knownWalls(walls) {
    const c = this.ctx;
    c.lineCap = "round";
    c.strokeStyle = colors.wall;
    c.lineWidth = Math.max(3, this.T / 9);
    for (const w of walls) if (w.wall) this.wall(w.x, w.y, w.dir);
  }

  path(pose, planned) {
    if (!pose || planned.length < 2) return;
    const c = this.ctx, T = this.T;
    c.strokeStyle = colors.path;
    c.fillStyle = colors.path;
    c.lineWidth = Math.max(2, T / 16);
    c.lineJoin = "round";
    c.beginPath();
    c.moveTo(this.px(pose.x) + T / 2, this.py(pose.y) + T / 2);
    for (const [x, y] of planned.slice(1)) c.lineTo(this.px(x) + T / 2, this.py(y) + T / 2);
    c.stroke();
    const [ex, ey] = planned[planned.length - 1];
    c.beginPath();
    c.arc(this.px(ex) + T / 2, this.py(ey) + T / 2, T * 0.09, 0, Math.PI * 2);
    c.fill();
  }

  robot(pose, size = null) {
    const c = this.ctx, T = this.T, r = T * 0.3;
    const hl = size ? size.len * T / 2 : r, hw = size ? size.wid * T / 2 : r;   // halbe Länge / Breite
    c.save();
    c.translate(this.px(pose.x) + T / 2, this.py(pose.y) + T / 2);
    c.rotate(pose.angle * Math.PI / 2);
    c.fillStyle = colors.robot;
    c.beginPath();
    c.roundRect(-hw, -hl, 2 * hw, 2 * hl, r * 0.25);
    c.fill();
    c.fillStyle = colors.robotMark;
    c.beginPath();
    c.moveTo(0, -r * 0.7);
    c.lineTo(r * 0.5, r * 0.35);
    c.lineTo(-r * 0.5, r * 0.35);
    c.closePath();
    c.fill();
    c.restore();
  }
}

// Weiche Bewegung des Roboters zwischen zwei Posen {x, y, angle}
class PoseAnim {
  constructor() { this.from = null; this.to = null; this.start = 0; this.dur = 0; }

  set(target, dur) {
    if (!this.to) {
      this.from = this.to = target;
      this.dur = 0;
      return;
    }
    // Winkel ohne Sprung (z.B. West -> Nord)
    let angle = target.angle;
    while (angle - this.to.angle > 2) angle -= 4;
    while (angle - this.to.angle < -2) angle += 4;
    const to = { x: target.x, y: target.y, angle };
    if (to.x === this.to.x && to.y === this.to.y && to.angle === this.to.angle) return;
    this.from = this.pose(performance.now());
    // Großer Sprung (Pose in Kacheln und Vierteldrehungen): mehr als 3 cm oder 15° nicht überblenden,
    // sondern direkt springen, sonst schneidet das Bild bei hohem Tempo durch Wände
    if (Math.hypot(to.x - this.from.x, to.y - this.from.y) > 3 / 30 ||
        Math.abs(to.angle - this.from.angle) > 15 / 90) this.from = to;
    this.to = to;
    this.start = performance.now();
    this.dur = reducedMotion.matches ? 0 : dur;
  }

  progress(now) { return this.dur > 0 ? Math.min(1, (now - this.start) / this.dur) : 1; }

  pose(now) {
    if (!this.to) return null;
    const t = this.progress(now);
    const e = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    const lerp = (a, b) => a + (b - a) * e;
    return { x: lerp(this.from.x, this.to.x), y: lerp(this.from.y, this.to.y),
             angle: lerp(this.from.angle, this.to.angle) };
  }

  reset() { this.from = this.to = null; }
}

// --- Hilfsfunktionen -------------------------------------------------------------

function fmtTime(s) {
  return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

function fmtCm(v) {
  return v === null || v === undefined ? "kein Echo" : `${v.toFixed(1)} cm`;
}

function capitalize(s) {
  return s ? s.charAt(0).toUpperCase() + s.slice(1) : "–";
}

function setList(el, items) {
  el.replaceChildren(...items.map((text) => {
    const li = document.createElement("li");
    li.textContent = text;
    return li;
  }));
}

function setClock(prefix, time, limit) {
  $(`${prefix}time`).textContent = time === null ? "–" : fmtTime(time);
  $(`${prefix}limit`).textContent = fmtTime(limit);
  const ratio = time === null ? 0 : Math.min(1, time / limit);
  const fill = $(prefix === "st-" ? "timebar-fill" : "rb-timebar-fill");
  fill.style.width = `${ratio * 100}%`;
  fill.classList.toggle("low", ratio > 0.85);
}

function setSwatch(el, floor) {
  if (!colors) readColors();
  el.style.background = floor ? colors.floors[floor] : "transparent";
}

// Pose des Roboters aus einem Zustand des Roboterprogramms (Kachel + Bewegung seit Aktionsbeginn)
function framePose(msg) {
  const p = msg.pose, m = msg.motion || {};
  const cm = m.forward_cm || 0, deg = m.turn_deg || 0;
  const [dx, dy] = DIR_STEP[p.dir];
  const f = cm / 30;
  return { x: p.x + dx * f, y: p.y + dy * f, angle: p.dir + deg / 90 };
}

// Karte des echten Roboters (live und Aufzeichnung)
function drawRobotMap(m, s, pose) {
  const mp = s.map;
  for (const t of mp.tiles) {
    m.tile(t.x, t.y, colors.floors[t.floor], t.visited ? 1 : 0.6);
  }
  m.joints(mp.tiles.map((t) => [t.x, t.y]));
  m.label(0, 0, "S");
  m.frontier(mp.frontier);
  m.knownWalls(mp.walls);
  m.path(pose, mp.planned);
  if (pose) m.robot(pose);
}

function setRows(dl, rows) {
  const items = [];
  for (const [label, value, title] of rows) {
    const dt = document.createElement("dt");
    dt.textContent = label;
    const dd = document.createElement("dd");
    dd.textContent = value;
    if (title) dd.title = title;
    items.push(dt, dd);
  }
  dl.replaceChildren(...items);
}

