"use strict";
// Datei: view_sim.js – Ansicht „Simulation“

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

