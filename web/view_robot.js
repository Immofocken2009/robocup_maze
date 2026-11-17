"use strict";
// Datei: view_robot.js – Ansicht „Roboter“: Live-Karte des echten Roboters

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
      bat.textContent = se.battery === null ? "–" : `${se.battery.toFixed(2)} V`;
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

