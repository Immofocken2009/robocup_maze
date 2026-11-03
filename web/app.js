"use strict";

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

  robot(pose) {
    const c = this.ctx, T = this.T, r = T * 0.3;
    c.save();
    c.translate(this.px(pose.x) + T / 2, this.py(pose.y) + T / 2);
    c.rotate(pose.angle * Math.PI / 2);
    c.fillStyle = colors.robot;
    c.beginPath();
    c.roundRect(-r, -r, 2 * r, 2 * r, r * 0.25);
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

// --- Ansicht: echter Roboter -------------------------------------------------------

const robot = {
  state: null,
  connected: false,
  anim: new PoseAnim(),
  map: null,

  init() {
    this.map = new MapCanvas($("rb-map"), (m, now) => this.draw(m, now));
    $("rb-red-mode").replaceChildren(
      ...Object.entries(RED_MODE_NAMES).map(([m, name]) => new Option(name, m)));
    $("rb-red-mode").addEventListener("change", (e) => send({ cmd: "robot_red_mode", value: e.target.value }));
    this.render();
  },

  setLink(connected) {
    this.connected = connected;
    this.render();
  },

  setState(msg) {
    this.state = msg;
    const p = msg.pose, m = msg.motion || {};
    const cm = m.forward_cm || 0, deg = m.turn_deg || 0;
    const [dx, dy] = DIR_STEP[p.dir];
    const f = cm / 30;
    this.anim.set({ x: p.x + dx * f, y: p.y + dy * f, angle: p.dir + deg / 90 }, 120);
    this.updateBounds();
    this.render();
    this.map.request();
  },

  updateBounds() {
    const s = this.state, mp = s.map;
    const xs = [0, s.pose.x], ys = [0, s.pose.y];
    for (const t of mp.tiles) { xs.push(t.x); ys.push(t.y); }
    for (const [x, y] of mp.frontier) { xs.push(x); ys.push(y); }
    const x0 = Math.min(...xs), y0 = Math.min(...ys);
    // mindestens 4 × 4 Kacheln, damit die Karte am Anfang nicht riesig wird
    const w = Math.max(4, Math.max(...xs) - x0 + 1), h = Math.max(4, Math.max(...ys) - y0 + 1);
    this.map.setBounds(x0, y0, w, h);
  },

  draw(m, now) {
    const s = this.state;
    if (!s) return;
    const mp = s.map;
    for (const t of mp.tiles) {
      m.tile(t.x, t.y, colors.floors[t.floor], t.visited ? 1 : 0.6);
    }
    m.joints(mp.tiles.map((t) => [t.x, t.y]));
    m.label(0, 0, "S");
    m.frontier(mp.frontier);
    m.knownWalls(mp.walls);
    const pose = this.anim.pose(now);
    m.path(pose, mp.planned);
    if (pose) m.robot(pose);
    if (this.anim.progress(now) < 1) m.request();
  },

  render() {
    const s = this.state;
    const live = this.connected && s;

    $("rb-status").textContent = !this.connected ? "Roboterprogramm nicht verbunden" : s ? s.status_text : "–";
    const red = $("rb-red-mode");
    if (s) red.value = s.red_mode;
    red.disabled = !live || !["start", "finished"].includes(s.status);
    red.title = red.disabled ? "Nur vor dem Lauf änderbar" : "";

    const warnings = [];
    if (s && s.error) warnings.push(s.error);
    if (s) warnings.push(...s.warnings);
    setList($("rb-warnings"), live ? warnings : []);

    if (!s) return;
    setClock("rb-", s.time, s.time_limit);

    $("rb-action").textContent = capitalize(s.action);
    $("rb-detail").textContent = s.detail || "";

    // Erkannt
    const walls = s.detected ? s.detected.walls : null;
    for (const side of ["front", "left", "right"]) {
      const el = $(`rd-${side}`);
      if (!walls) { el.textContent = "–"; el.className = ""; continue; }
      el.textContent = walls[side] ? "Wand" : "offen";
      el.className = walls[side] ? "strong" : "muted";
    }
    const floor = s.detected ? s.detected.floor : null;
    setSwatch($("rd-swatch"), floor);
    $("rd-floor").textContent = floor ? FLOOR_NAMES[floor] : "–";
    $("rd-pos").textContent = `${s.pose.x}, ${s.pose.y}, Blick ${DIR_NAMES[s.pose.dir]}`;
    $("rd-checkpoint").textContent = s.map.checkpoint ? s.map.checkpoint.join(", ") : "–";

    // Sensoren
    const se = s.sensors;
    if (se) {
      for (const side of ["front", "left", "right"]) {
        $(`rs-${side}`).textContent = fmtCm(se[side]);
        $(`rw-${side}`).classList.toggle("on", se[side] !== null && se[side] < s.wall_threshold);
      }
      setSwatch($("rs-swatch"), se.color);
      $("rs-color").textContent = se.color ? FLOOR_NAMES[se.color] : "nicht kalibriert";
      $("rs-rgbc").textContent = se.rgbc ? se.rgbc.join("  ") : "–";
      $("rs-enc").textContent = `${se.encoders[0]}°  /  ${se.encoders[1]}°`;
      $("rs-battery").textContent = `${se.battery.toFixed(1)} V`;
    }

    // Lauf
    $("rl-mode").textContent = capitalize(s.mode);
    $("rl-visited").textContent = s.stats ? `${s.stats.visited} Kacheln` : "–";
    $("rl-frontier").textContent = s.stats ? s.stats.frontier : "–";
    setList($("rb-events"), s.events.slice().reverse());
  },
};

// --- Ansicht: Simulation -------------------------------------------------------------

const sim = {
  world: null,      // das echte Labyrinth (kommt einmal pro geladenem Labyrinth)
  state: null,      // aktueller Stand der Simulation (kommt nach jeder Aktion)
  control: null,    // läuft / pausiert / fertig, Geschwindigkeit
  settingsLoaded: false,
  anim: new PoseAnim(),
  map: null,

  init() {
    this.map = new MapCanvas($("maze"), (m, now) => this.draw(m, now));
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
    $("show-truth").addEventListener("change", () => this.map.request());
  },

  setWorld(msg) {
    this.world = msg;
    this.state = null;
    this.anim.reset();
    this.updateWorldInfo();
    this.map.bounds = null;
    this.map.setBounds(0, 0, msg.width, msg.height);
  },

  setState(msg) {
    const r = msg.robot;
    this.anim.set({ x: r.x, y: r.y, angle: r.dir }, msg.anim_ms);
    this.state = msg;
    this.updatePanels();
    this.map.request();
  },

  setControl(msg) {
    this.control = msg;
    this.updateControls();
  },

  draw(m, now) {
    const world = this.world, state = this.state;
    if (!world) return;
    const showTruth = $("show-truth").checked;
    const key = (x, y) => `${x},${y}`;
    const known = new Map((state ? state.known_tiles : []).map((t) => [key(t.x, t.y), t]));

    // Kacheln: bekannte in ihrer Farbe, unbekannte grau (echter Boden blass, wenn eingeblendet)
    for (const t of world.tiles) {
      const k = known.get(key(t.x, t.y));
      m.tile(t.x, t.y, k ? colors.floors[k.floor] : colors.unknown);
      if (!k && showTruth && t.floor !== "normal") m.tile(t.x, t.y, colors.floors[t.floor], 0.3);
    }
    m.joints(world.tiles.map((t) => [t.x, t.y]));
    m.label(world.start[0], world.start[1], "S");
    if (state) m.frontier(state.frontier);

    // Wände: echte dünn, erkannte als Holzwand
    if (showTruth) {
      m.ctx.lineCap = "round";
      m.ctx.strokeStyle = colors.truth;
      m.ctx.lineWidth = 1.5;
      for (const t of world.tiles) for (const d of t.walls) m.wall(t.x, t.y, d);
    }
    if (state) m.knownWalls(state.known_walls);

    const pose = this.anim.pose(now);
    if (state) m.path(pose, state.planned);
    if (pose) m.robot(pose);
    if (this.anim.progress(now) < 1) m.request();
  },

  updatePanels() {
    const s = this.state;
    $("st-mode").textContent = capitalize(s.mode);
    setClock("st-", s.time, s.time_limit);
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
    setSwatch($("se-swatch"), s.sensors.color);
    $("se-color").textContent = FLOOR_NAMES[s.sensors.color];
    $("se-gyro").textContent = `${s.sensors.gyro}°`;
    setList($("events"), s.events.slice().reverse());
  },

  updateWorldInfo() {
    const world = this.world;
    const parts = [world.name];
    if (world.seed !== null && world.seed !== undefined) parts.push(`Seed ${world.seed}`);
    parts.push(`${world.width} × ${world.height}`, `${world.reachable} erreichbare Kacheln`);
    if (world.floating) parts.push(`${world.floating} davon floating`);
    $("maze-info").textContent = parts.join(", ") + (world.saved ? "" : " (nicht gespeichert)");
    $("btn-save").disabled = world.saved;
    setList($("notes"), world.notes);
  },

  applySettings(s) {
    if (this.settingsLoaded) return;
    this.settingsLoaded = true;
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
  },

  updateControls() {
    const control = this.control;
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
  },
};

const sliderToSpeed = (v) => SPEED_MIN * Math.pow(SPEED_MAX / SPEED_MIN, v / 100);
const speedToSlider = (s) => 100 * Math.log(s / SPEED_MIN) / Math.log(SPEED_MAX / SPEED_MIN);
const fmtSpeed = (s) => (s < 10 ? s.toFixed(1) : Math.round(s)) + "×";

// --- Ansicht wechseln ------------------------------------------------------------------

function showView(name) {
  view = name;
  for (const v of ["robot", "sim"]) {
    $(`view-${v}`).hidden = v !== name;
    $(`tab-${v}`).setAttribute("aria-selected", String(v === name));
  }
  try { localStorage.setItem("view", name); } catch (_) { /* ohne Speicher geht es auch */ }
  (name === "robot" ? robot : sim).map.resize();
}

$("tab-robot").addEventListener("click", () => showView("robot"));
$("tab-sim").addEventListener("click", () => showView("sim"));

window.addEventListener("resize", () => (view === "robot" ? robot : sim).map.resize());
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => {
  readColors();
  robot.map.request();
  sim.map.request();
});

robot.init();
sim.init();
let savedView = null;
try { savedView = localStorage.getItem("view"); } catch (_) { /* egal */ }
showView(savedView === "sim" ? "sim" : "robot");
connect();
