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
    this.anim.set(framePose(msg), 120);
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
    drawRobotMap(m, s, this.anim.pose(now));
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
      ["left", "right"].forEach((side, i) => {
        const p = se.motor_power[i];
        $(`rs-power-${side}`).textContent = p === null ? "aus" : `${p} %`;
        $(`rm-${side}`).style.width = `${p === null ? 0 : Math.min(100, Math.abs(p))}%`;
      });
      const bat = $("rb-battery");
      bat.textContent = `${se.battery.toFixed(2)} V`;
      bat.classList.toggle("low", s.battery_low);
    }

    // System: CPU-Temperatur und Lüfter
    const sy = s.system;
    if (sy) {
      const temp = sy.cpu_temp === null ? "–" : `${sy.cpu_temp.toFixed(1)} °C`;
      $("rb-cpu").textContent = sy.cpu_temp === null ? "–" : `${Math.round(sy.cpu_temp)} °C`;
      $("rb-cpu").classList.toggle("low", sy.cpu_hot);
      $("ry-temp").textContent = temp;
      $("ry-rpm").textContent = sy.fan_rpm === null ? "–" : sy.fan_rpm > 0 ? `${sy.fan_rpm} U/min` : "steht";
      $("ry-level").textContent = `${sy.fan_level} von ${sy.fan_levels}` +
        (sy.fan_percent === null ? "" : ` (${sy.fan_percent} %)`);
      $("ry-meter").style.width = `${sy.fan_percent || 0}%`;
      $("ry-control").textContent = sy.fan_control === "Python" ? "Python (hardware/fan.py)" : "Kernel";
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
    for (const id of ["noise", "slip"]) {
      $(id).addEventListener("change", (e) => send({ cmd: id, value: e.target.checked }));
    }
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
    this.anim.set(msg.robot, msg.anim_ms);
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
    if (state) m.beams(state.beams);
    if (pose) m.robot(pose, state ? state.robot_size : null);
    if (this.anim.progress(now) < 1) m.request();
  },

  updatePanels() {
    const s = this.state;
    $("st-mode").textContent = capitalize(s.mode);
    setClock("st-", s.time, s.time_limit);
    $("st-visited").textContent = `${s.stats.visited} von ${s.stats.reachable} Kacheln`;
    $("st-actions").textContent = s.stats.actions;
    $("st-now").textContent = capitalize(s.action);
    $("st-detail").textContent = s.detail || "";
    $("st-checkpoint").textContent = s.checkpoint ? `${s.checkpoint[0]}, ${s.checkpoint[1]}` : "–";

    for (const part of ["checkpoints", "blue", "reliability", "exit", "total"]) {
      $(`sc-${part}`).textContent = s.score[part];
    }
    $("sc-lop").textContent = s.score.lop ? `${s.score.lop}× Lack of Progress` : "";

    for (const side of ["front", "left", "right"]) {
      const v = s.sensors[side];
      $(`se-${side}`).textContent = fmtCm(v);
      $(`sw-${side}`).classList.toggle("on", v !== null && v < s.wall_threshold);
    }
    setSwatch($("se-swatch"), s.sensors.color);
    $("se-color").textContent = FLOOR_NAMES[s.sensors.color];
    $("se-enc").textContent = `${s.sensors.encoders[0]}°  /  ${s.sensors.encoders[1]}°`;
    $("se-skew").textContent = `${s.sensors.heading_error.toFixed(1)}°`;
    setList($("events"), s.events.slice().reverse());
  },

  updateWorldInfo() {
    const world = this.world;
    $("btn-save").disabled = world.saved;
    $("btn-save").title = world.saved ? "Schon gespeichert" : "Dieses Labyrinth speichern";
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

    $("noise").checked = control.noise;
    $("slip").checked = control.slip;

    const slider = $("speed");
    if (document.activeElement !== slider) slider.value = speedToSlider(control.speed);
    $("speed-label").textContent = fmtSpeed(control.speed);
  },
};

const sliderToSpeed = (v) => SPEED_MIN * Math.pow(SPEED_MAX / SPEED_MIN, v / 100);
const speedToSlider = (s) => 100 * Math.log(s / SPEED_MIN) / Math.log(SPEED_MAX / SPEED_MIN);
const fmtSpeed = (s) => (s < 10 ? s.toFixed(1) : Math.round(s)) + "×";

// --- Verlaufsdiagramm (Aufzeichnung) ------------------------------------------------------

const CHART_PANELS = [
  { title: "Ultraschall (cm)", lines: [["vorne", "ink"], ["links", "blue"], ["rechts", "wall"]], cap: 150 },
  { title: "Abweichung von der Mitte (cm)", lines: [["abweichung", "ink"]], zero: true },
  { title: "Motorbefehl (°/s)", lines: [["motor_links", "blue"], ["motor_rechts", "wall"]], zero: true },
];
const CHART_TITLE = 18, CHART_PLOT = 78, CHART_GAP = 14, CHART_AXIS = 22, CHART_LEFT = 46, CHART_RIGHT = 8;
const TIME_STEPS = [1, 2, 5, 10, 15, 30, 60, 120, 300, 600];

class Chart {
  constructor(canvas, onSeek) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.onSeek = onSeek;
    this.series = null;
    this.duration = 0;
    this.cursor = 0;
    this.w = 0;
    this.queued = false;
    let dragging = false;
    const seek = (e) => this.onSeek(this.tAt(e.clientX - canvas.getBoundingClientRect().left));
    canvas.addEventListener("pointerdown", (e) => { dragging = true; canvas.setPointerCapture(e.pointerId); seek(e); });
    canvas.addEventListener("pointermove", (e) => { if (dragging) seek(e); });
    canvas.addEventListener("pointerup", () => { dragging = false; });
    canvas.addEventListener("pointercancel", () => { dragging = false; });
  }

  setData(series, duration) {
    this.series = series;
    this.duration = duration;
    this.ranges = CHART_PANELS.map((panel) => {
      let lo = Infinity, hi = -Infinity;
      for (const [name] of panel.lines) {
        for (const [, v] of series[name] || []) {
          if (v === null) continue;
          lo = Math.min(lo, v);
          hi = Math.max(hi, v);
        }
      }
      if (lo > hi) return [0, 1];
      if (panel.zero) {
        const m = Math.max(Math.abs(lo), Math.abs(hi), 1);
        return [-m, m];
      }
      return [0, Math.max(1, Math.min(hi, panel.cap || hi))];
    });
    this.resize();
  }

  setCursor(t) {
    this.cursor = t;
    this.request();
  }

  plotW() { return Math.max(1, this.w - CHART_LEFT - CHART_RIGHT); }
  xAt(t) { return CHART_LEFT + (this.duration ? t / this.duration : 0) * this.plotW(); }
  tAt(x) { return Math.max(0, Math.min(this.duration, (x - CHART_LEFT) / this.plotW() * this.duration)); }

  resize() {
    const w = this.canvas.parentElement.clientWidth;
    if (!w) return;
    const h = CHART_PANELS.length * (CHART_TITLE + CHART_PLOT + CHART_GAP) + CHART_AXIS;
    const dpr = window.devicePixelRatio || 1;
    this.w = w;
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
    requestAnimationFrame(() => {
      this.queued = false;
      this.draw();
    });
  }

  draw() {
    const c = this.ctx;
    c.clearRect(0, 0, this.canvas.width, this.canvas.height);
    if (!this.series || !this.w) return;
    if (!colors) readColors();
    const ink = getComputedStyle(document.documentElement).getPropertyValue("--ink").trim();
    const lineColors = { ink, blue: colors.floors.blue, wall: colors.wall };
    c.font = "12px system-ui, sans-serif";
    c.textBaseline = "middle";
    let bottom = 0;
    CHART_PANELS.forEach((panel, i) => {
      const top = i * (CHART_TITLE + CHART_PLOT + CHART_GAP);
      const y0 = top + CHART_TITLE, y1 = y0 + CHART_PLOT;
      const [lo, hi] = this.ranges[i];
      const yAt = (v) => y1 - (Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo) * CHART_PLOT;
      bottom = y1;

      c.fillStyle = colors.muted;
      c.textAlign = "left";
      c.fillText(panel.title, CHART_LEFT, top + CHART_TITLE / 2);
      c.textAlign = "right";
      for (const v of panel.zero ? [hi, 0, lo] : [hi, lo]) c.fillText(fmtAxis(v), CHART_LEFT - 6, yAt(v));

      c.strokeStyle = colors.truth;
      c.lineWidth = 1;
      c.globalAlpha = 0.5;
      c.beginPath();
      const base = panel.zero ? yAt(0) : y1;
      c.moveTo(CHART_LEFT, Math.round(base) + 0.5);
      c.lineTo(CHART_LEFT + this.plotW(), Math.round(base) + 0.5);
      c.stroke();
      c.globalAlpha = 1;

      for (const [name, col] of panel.lines) {
        c.strokeStyle = lineColors[col];
        c.lineWidth = 1.25;
        c.beginPath();
        let open = false;
        for (const [t, v] of this.series[name] || []) {
          if (v === null) { open = false; continue; }
          const x = this.xAt(t), y = yAt(v);
          if (open) c.lineTo(x, y); else c.moveTo(x, y);
          open = true;
        }
        c.stroke();
      }
    });

    // Zeitachse
    const step = TIME_STEPS.find((s) => this.duration / s <= Math.max(2, this.plotW() / 80)) || 600;
    c.fillStyle = colors.muted;
    c.textAlign = "center";
    for (let t = 0; t <= this.duration + 1e-9; t += step) c.fillText(fmtTime(t), this.xAt(t), bottom + CHART_AXIS / 2 + 2);

    // Zeitpunkt
    const x = Math.round(this.xAt(this.cursor)) + 0.5;
    c.strokeStyle = ink;
    c.lineWidth = 1;
    c.beginPath();
    c.moveTo(x, CHART_TITLE);
    c.lineTo(x, bottom);
    c.stroke();
  }
}

function fmtAxis(v) {
  return Math.abs(v) >= 100 ? String(Math.round(v)) : String(Math.round(v * 10) / 10);
}

// --- Ansicht: Aufzeichnung -------------------------------------------------------------------

const CALC_TITLES = {
  fahrt: "Fahren (letzter Regeltakt)",
  seite: "Lenkung (letzte Seitenmessung)",
  vorne: "Abstand vorne, Haltepunkt",
  farbe: "Farbprüfung beim Fahren",
  "vorwärts": "Start der Fahrt",
  angekommen: "Angekommen",
  "schräglage": "Schrägstand am Kachelende",
  drehen: "Drehen",
  strecke: "Strecke nach Encoder",
  blockiert: "Ketten blockiert",
  "wände": "Wände gemessen",
  boden: "Boden gemessen",
  ergebnis: "Ergebnis der Fahrt",
  plan: "Planung",
  zeit: "Zeitschätzung",
};
const MODE_NAMES = { both: "beide Wände", left: "linke Wand", right: "rechte Wand", encoder: "Encoder" };
const KIND_NAMES = { io: "Hardware", calc: "Rechnung", flow: "Taster", state: "Zustand", log: "Meldung", end: "Ende" };
const OLD_S = 2;    // älter: Rechenschritt grau

const logView = {
  runs: [],
  name: null,
  sum: null,          // Übersicht der geöffneten Aufzeichnung
  data: null,         // alles zum gewählten Zeitpunkt
  t: 0,
  playing: false,
  speed: 1,
  lastTick: 0,
  busy: false,
  pending: false,
  loaded: false,
  map: null,
  chart: null,

  init() {
    this.map = new MapCanvas($("lg-map"), (m) => this.drawMap(m));
    this.chart = new Chart($("lg-chart"), (t) => this.seek(t));
    $("lg-run").addEventListener("change", (e) => this.open(e.target.value));
    $("lg-refresh").addEventListener("click", () => this.refresh());
    $("lg-play").addEventListener("click", () => (this.playing ? this.pause() : this.play()));
    $("lg-prev").addEventListener("click", () => this.jumpMark(-1));
    $("lg-next").addEventListener("click", () => this.jumpMark(1));
    $("lg-speed").addEventListener("change", (e) => { this.speed = Number(e.target.value); });
    $("lg-slider").addEventListener("input", (e) => this.seek(Number(e.target.value)));
    this.setDownload(null);
  },

  shown() {
    if (!this.loaded) {
      this.loaded = true;
      this.refresh();
    }
    this.chart.resize();
  },

  async refresh() {
    let data;
    try {
      data = await (await fetch("/api/runs")).json();
    } catch (_) {
      return;
    }
    this.runs = data.runs;
    const empty = this.runs.length === 0;
    $("lg-empty").hidden = !empty;
    $("lg-main").hidden = empty;
    $("lg-run").replaceChildren(...this.runs.map((r) => new Option(runLabel(r), r.name)));
    $("lg-run").disabled = empty;
    if (empty) {
      this.setDownload(null);
      return;
    }
    const same = this.runs.some((r) => r.name === this.name);
    const name = same ? this.name : this.runs[0].name;
    $("lg-run").value = name;
    await this.open(name, same);
  },

  async open(name, keepTime = false) {
    this.pause();
    let sum;
    try {
      const res = await fetch(`/api/runs/${encodeURIComponent(name)}`);
      if (!res.ok) throw new Error(await res.text());
      sum = await res.json();
    } catch (err) {
      $("lg-action").textContent = "Aufzeichnung lässt sich nicht lesen";
      $("lg-detail").textContent = String(err.message || err);
      return;
    }
    this.name = name;
    this.sum = sum;
    this.setDownload(name);
    $("lg-file").textContent = name;
    $("lg-slider").max = sum.duration;
    $("lg-duration").textContent = fmtSec(sum.duration);
    const b = sum.bounds;
    this.map.setBounds(b.x0, b.y0, Math.max(4, b.w), Math.max(4, b.h));
    this.map.resize();
    this.chart.setData(sum.series, sum.duration);
    this.renderInfo();
    this.seek(keepTime ? Math.min(this.t, sum.duration) : 0);
  },

  setDownload(name) {
    const a = $("lg-download");
    a.href = name ? `/api/runs/${encodeURIComponent(name)}/download` : "#";
    a.setAttribute("aria-disabled", String(!name));
  },

  seek(t) {
    if (!this.sum) return;
    this.t = Math.max(0, Math.min(this.sum.duration, t));
    $("lg-slider").value = this.t;
    $("lg-t").textContent = fmtSec(this.t);
    this.chart.setCursor(this.t);
    this.fetchAt();
  },

  // Immer nur eine Anfrage unterwegs; während sie läuft, zählt nur der neueste Zeitpunkt.
  async fetchAt() {
    if (this.busy) {
      this.pending = true;
      return;
    }
    this.busy = true;
    try {
      do {
        this.pending = false;
        const name = this.name;
        const res = await fetch(`/api/runs/${encodeURIComponent(name)}/at?t=${this.t}`);
        if (res.ok && name === this.name) {
          this.data = await res.json();
          this.render();
        }
      } while (this.pending);
    } catch (_) {
      /* beim nächsten Spulen wieder */
    }
    this.busy = false;
  },

  play() {
    if (!this.sum) return;
    if (this.t >= this.sum.duration) this.seek(0);
    this.playing = true;
    $("lg-play").textContent = "Pause";
    this.lastTick = performance.now();
    requestAnimationFrame((now) => this.tick(now));
  },

  pause() {
    this.playing = false;
    $("lg-play").textContent = "Abspielen";
  },

  tick(now) {
    if (!this.playing) return;
    const t = this.t + (now - this.lastTick) / 1000 * this.speed;
    this.lastTick = now;
    if (t >= this.sum.duration) {
      this.seek(this.sum.duration);
      this.pause();
      return;
    }
    this.seek(t);
    requestAnimationFrame((n) => this.tick(n));
  },

  jumpMark(dir) {
    if (!this.sum) return;
    const marks = this.sum.marks;
    const mark = dir < 0 ? marks.filter((m) => m.t < this.t - 0.05).pop()
                         : marks.find((m) => m.t > this.t + 0.001);
    if (mark) this.seek(mark.t);
  },

  drawMap(m) {
    const f = this.data && this.data.frame;
    if (f) drawRobotMap(m, f, framePose(f));
  },

  render() {
    const d = this.data, f = d.frame;
    $("lg-action").textContent = f ? capitalize(f.action) : "–";
    $("lg-detail").textContent = f ? f.detail || "" : "";
    $("lg-clock").textContent = f && typeof f.time === "number" ? fmtTime(f.time) : "–";
    this.renderInputs(d.io);
    this.renderCalc(d.calc);
    setList($("lg-events"), f ? f.events.slice().reverse() : []);
    this.renderRaw(d.raw, d.cursor);
    this.map.request();
  },

  renderInfo() {
    const s = this.sum, h = s.head, e = s.end;
    const rows = [
      ["Datum", h.date ? h.date.replace("T", " ") : "–"],
      ["Code", (h.git || "–") + (h.git_changed && h.git_changed.length ? `, nicht committet: ${h.git_changed.join(", ")}` : "")],
      ["Gefahrenzone", RED_MODE_NAMES[h.red_mode] || h.red_mode || "–"],
      ["Gangbreite am Start", typeof h.corridor_cm === "number" ? `${h.corridor_cm.toFixed(2)} cm` : "–"],
      ["Farbsensor", h.color_warning || "kalibriert"],
      ["Ultraschall", "us_warning" in h ? (h.us_warning || "kalibriert") : "–"],
      ["Ergebnis", e ? capitalize(e.mode) + (e.stopped ? ", mit K2 abgebrochen" : "") : "kein Ende (z.B. Strom weg)"],
      ["Besucht", e && typeof e.visited === "number" ? `${e.visited} Kacheln` : "–"],
      ["Dauer", fmtSec(s.duration)],
      ["Einträge", `${s.entries} (Hardware ${s.counts.io || 0}, Rechnung ${s.counts.calc || 0}, Zustand ${s.counts.state || 0})`],
      ["Vollständig", s.complete ? "ja" : "nein, mittendrin abgebrochen"],
    ];
    setRows($("lg-info"), rows.map(([k, v]) => [k, v]));
  },

  renderInputs(io) {
    const age = (rec) => (rec ? `vor ${fmtAge(this.t - rec.t)}` : "");
    const rows = [];
    for (const [side, label] of [["front", "vorne"], ["left", "links"], ["right", "rechts"]]) {
      const rec = io[`us.measure:${side}`];
      const echo = rec && typeof rec.ns === "number" ? ` · Echo ${(rec.ns / 1000).toFixed(0)} µs` : "";
      rows.push([`Ultraschall ${label}`, rec ? fmtCm(rec.r) + echo : "–", age(rec), rec]);
    }
    const enc = io["motors.positions"];
    rows.push(["Encoder L / R", enc ? `${enc.r[0].toFixed(1)}° / ${enc.r[1].toFixed(1)}°` : "–", age(enc), enc]);
    const raw = io["color.read_raw"];
    rows.push(["Farbsensor R G B C", raw ? raw.r.join("  ") : "–", age(raw), raw]);
    const cmds = ["motors.set_dps", "motors.stop", "motors.float"].map((k) => io[k]).filter(Boolean);
    const cmd = cmds.sort((a, b) => a.t - b.t).pop();
    let text = "–";
    if (cmd && cmd.src === "motors.set_dps") text = `L ${cmd.a[0].toFixed(1)} / R ${cmd.a[1].toFixed(1)} °/s`;
    else if (cmd && cmd.src === "motors.stop") text = "Stopp (gebremst)";
    else if (cmd) text = "stromlos";
    rows.push(["Motorbefehl", text, age(cmd), cmd]);
    const clock = io["clock.monotonic"];
    rows.push(["Uhr des Roboters", clock ? `${clock.r.toFixed(4)} s` : "–", age(clock), clock]);
    const key = io["robot._read_interrupt"];
    if (key) rows.push(["Taster", key.r === "stop" ? "K2 Stopp" : "K3 Lack of Progress", age(key), key]);
    setRows($("lg-inputs"), rows.map(([k, v, a, rec]) => [k, a ? `${v} · ${a}` : v, rec ? JSON.stringify(rec) : ""]));
  },

  renderCalc(calc) {
    const box = $("lg-calc");
    const kinds = [...Object.keys(CALC_TITLES), ...Object.keys(calc).filter((k) => !(k in CALC_TITLES))];
    const groups = [];
    for (const kind of kinds) {
      const rec = calc[kind];
      if (!rec) continue;
      const group = document.createElement("div");
      const ago = this.t - rec.t;
      if (ago > OLD_S) group.className = "old";
      const h = document.createElement("h3");
      h.textContent = CALC_TITLES[kind] || kind;
      const when = document.createElement("span");
      when.textContent = ` · vor ${fmtAge(ago)}`;
      h.append(when);
      const dl = document.createElement("dl");
      setRows(dl, Object.entries(rec)
        .filter(([k]) => !["k", "t", "what"].includes(k))
        .map(([k, v]) => [k.replace(/_/g, " "), k === "modus" ? MODE_NAMES[v] || fmtVal(v) : fmtVal(v), JSON.stringify(v)]));
      group.append(h, dl);
      groups.push(group);
    }
    box.replaceChildren(...groups);
  },

  renderRaw(raw, cursor) {
    const body = $("lg-raw").tBodies[0];
    const rows = raw.map((rec, i) => {
      const tr = document.createElement("tr");
      if (i === cursor) tr.className = "cur";
      else if (i > cursor) tr.className = "future";
      for (const [cls, text] of [["t", rec.t.toFixed(4)], ["k", KIND_NAMES[rec.k] || rec.k], ["v", describe(rec)]]) {
        const td = document.createElement("td");
        td.className = cls;
        td.textContent = text;
        tr.append(td);
      }
      tr.addEventListener("click", () => this.seek(rec.t));
      return tr;
    });
    body.replaceChildren(...rows);
    const cur = rows[Math.max(0, cursor)];
    const box = body.closest(".raw-box");
    if (cur && box) box.scrollTop = cur.offsetTop - box.clientHeight / 2 + cur.offsetHeight / 2;
  },
};

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

function fmtVal(v) {
  if (v === null || v === undefined) return "–";
  if (typeof v === "boolean") return v ? "ja" : "nein";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : String(Math.round(v * 1000) / 1000);
  if (Array.isArray(v)) return v.map(fmtVal).join(", ");
  if (typeof v === "object") return Object.entries(v).map(([k, x]) => `${k} ${fmtVal(x)}`).join(", ");
  return String(v);
}

function describe(rec) {
  const json = (v) => JSON.stringify(v);
  if (rec.k === "io") {
    const extra = Object.entries(rec).filter(([k]) => !["k", "t", "src", "a", "r"].includes(k))
      .map(([k, v]) => `  ${k}=${json(v)}`).join("");
    return `${rec.src}(${(rec.a || []).map(json).join(", ")}) → ${json(rec.r)}${extra}`;
  }
  if (rec.k === "calc") {
    return `${rec.what}: ` + Object.entries(rec).filter(([k]) => !["k", "t", "what"].includes(k))
      .map(([k, v]) => `${k}=${json(v)}`).join("  ");
  }
  if (rec.k === "state") return `${rec.action || "–"} · ${rec.detail || ""} · ${rec.mode || ""}`;
  if (rec.k === "flow") return `${rec.src} → ${json(rec.r)}`;
  if (rec.k === "log") return `${rec.level} ${rec.text}`;
  return Object.entries(rec).filter(([k]) => !["k", "t"].includes(k)).map(([k, v]) => `${k}=${json(v)}`).join("  ");
}

function fmtSec(s) {
  return `${Math.floor(s / 60)}:${(s % 60).toFixed(2).padStart(5, "0")}`;
}

function fmtAge(s) {
  return s < 1 ? `${Math.round(s * 1000)} ms` : `${s.toFixed(1)} s`;
}

function runLabel(run) {
  const m = run.name.match(/^(\d{4})-(\d\d)-(\d\d)_(\d\d)-(\d\d)-(\d\d)/);
  const when = m ? `${m[3]}.${m[2]}.${m[1]} ${m[4]}:${m[5]}:${m[6]}` : run.name;
  const size = run.size >= 1e6 ? `${(run.size / 1e6).toFixed(1)} MB` : `${Math.round(run.size / 1e3)} KB`;
  return `${when} · ${size}${run.growing ? " · läuft gerade" : ""}`;
}

// --- Ansicht wechseln ------------------------------------------------------------------

const VIEWS = { robot, sim, log: logView };

function showView(name) {
  view = name;
  for (const v of Object.keys(VIEWS)) {
    $(`view-${v}`).hidden = v !== name;
    $(`tab-${v}`).setAttribute("aria-selected", String(v === name));
  }
  try { localStorage.setItem("view", name); } catch (_) { /* ohne Speicher geht es auch */ }
  VIEWS[name].map.resize();
  if (name === "log") logView.shown();
}

for (const v of Object.keys(VIEWS)) $(`tab-${v}`).addEventListener("click", () => showView(v));

window.addEventListener("resize", () => {
  VIEWS[view].map.resize();
  if (view === "log") logView.chart.resize();
});
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => {
  readColors();
  robot.map.request();
  sim.map.request();
  logView.map.request();
  logView.chart.request();
});

robot.init();
sim.init();
logView.init();
let savedView = null;
try { savedView = localStorage.getItem("view"); } catch (_) { /* egal */ }
showView(savedView in VIEWS ? savedView : "robot");
connect();
