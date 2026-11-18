"use strict";
// Datei: view_robot.js – Ansicht „Roboter“: Live-Karte des echten Roboters

// --- Ansicht: echter Roboter -------------------------------------------------------

const robot = {
  state: null,
  connected: false,
  anim: new PoseAnim(),
  map: null,

  init() {
    setInterval(() => {         // Kamerabild älter als 3 s: Hinweis statt eingefrorenem Bild
      if (this.cam.msg && performance.now() - this.cam.at > 3000) $("rc-status").textContent = "Kein aktuelles Kamerabild";
    }, 1000);
    const wrap = $("rb-cam-wrap");
    const toggle = () => {
      if (document.fullscreenElement) document.exitFullscreen();
      else if (wrap.requestFullscreen) wrap.requestFullscreen().catch(() => {});
    };
    $("rb-cam-full").addEventListener("click", toggle);
    $("rb-cam").addEventListener("dblclick", toggle);
    document.addEventListener("fullscreenchange", () => {
      $("rb-cam-full").hidden = !!document.fullscreenElement;
      this.drawCam();
    });
    this.map = new MapCanvas($("rb-map"), (m, now) => this.draw(m, now));
    $("rb-red-mode").replaceChildren(
      ...Object.entries(RED_MODE_NAMES).map(([m, name]) => new Option(name, m)));
    $("rb-red-mode").addEventListener("change", (e) => send({ cmd: "robot_red_mode", value: e.target.value }));
    this.render();
  },

  // --- Kamera: Bild und Einzeichnung (Pixelkoordinaten des Bildes) ---
  cam: { img: new Image(), msg: null, at: 0 },

  setCam(msg) {
    const c = this.cam;
    c.msg = msg;
    c.at = performance.now();
    c.img.onload = () => this.drawCam();
    c.img.src = "data:image/jpeg;base64," + msg.jpeg;
    const d = msg.det;
    $("rc-status").textContent = "Live";
    $("rc-bright").textContent = d.dark ? `${d.brightness} von 255 (zu dunkel, keine Kantensuche)` : `${d.brightness} von 255`;
    $("rc-vedges").textContent = d.vertical_edges.length
      ? d.vertical_edges.map((e) => `${e.bearing > 0 ? "+" : ""}${e.bearing}°`).join("  ") : "–";
    $("rc-lines").textContent = d.lines.length ? d.lines.map((l) => `${l.angle}°`).join("  ") : "–";
    $("rc-motion").textContent = `${d.motion} ${d.moving ? "(bewegt sich)" : "(steht)"}`;
    $("rc-swatch").style.background = `rgb(${d.patch.rgb.join(",")})`;
    $("rc-rgb").textContent = d.patch.rgb.join("  ");
    $("rc-fps").textContent = `${msg.fps} / ${msg.load_ms} ms`;
    const r = msg.rec;
    $("rc-rec").textContent = !r || r.mode === "aus" ? "aus" : r.on
      ? `läuft (${r.mode}): ${r.frames} Bilder` + (r.dropped ? `, ${r.dropped} verworfen` : "") : `bereit (${r.mode}), startet mit dem Lauf`;
  },

  drawCam() {
    const c = this.cam, canvas = $("rb-cam");
    if (!c.msg) return;
    const d = c.msg.det;
    // Im Vollbild feiner zeichnen (ganze Pixel je Bildpunkt), damit Linien und Schrift scharf bleiben
    const k = document.fullscreenElement
      ? Math.max(1, Math.floor(Math.min(screen.width / d.w, screen.height / d.h) * (window.devicePixelRatio || 1))) : 1;
    if (canvas.width !== d.w * k || canvas.height !== d.h * k) { canvas.width = d.w * k; canvas.height = d.h * k; }
    const g = canvas.getContext("2d");
    g.setTransform(k, 0, 0, k, 0, 0);
    g.drawImage(c.img, 0, 0, d.w, d.h);
    g.font = "12px system-ui, sans-serif";
    g.textBaseline = "top";
    const label = (text, x, y, color) => {
      const w = g.measureText(text).width + 6;
      const lx = Math.max(0, Math.min(d.w - w, x)), ly = Math.max(0, Math.min(d.h - 16, y));
      g.fillStyle = "rgba(0,0,0,.65)";
      g.fillRect(lx, ly, w, 15);
      g.fillStyle = color;
      g.fillText(text, lx + 3, ly + 1);
    };
    g.lineWidth = 2;
    // Boden/Wand-Linien
    for (const l of d.lines) {
      g.strokeStyle = "#ffd23f";
      g.beginPath(); g.moveTo(l.x0, l.y0); g.lineTo(l.x1, l.y1); g.stroke();
      label(`Boden/Wand ${l.angle}°`, (l.x0 + l.x1) / 2 - 30, (l.y0 + l.y1) / 2 + 4, "#ffd23f");
    }
    // senkrechte Kanten (Wandende, Ecke)
    for (const e of d.vertical_edges) {
      g.strokeStyle = "#4cc9f0";
      g.beginPath(); g.moveTo(e.x, e.y0); g.lineTo(e.x, e.y1); g.stroke();
      label(`Kante ${e.bearing > 0 ? "+" : ""}${e.bearing}°`, e.x + 3, e.y0, "#4cc9f0");
    }
    // Boden vor dem Roboter
    const p = d.patch;
    g.strokeStyle = "#fff"; g.lineWidth = 1;
    g.strokeRect(p.x0, p.y0, p.x1 - p.x0, p.y1 - p.y0);
    label("Boden", p.x0, p.y0 - 16, "#fff");
    // Hinweise oben links
    if (d.dark) label("zu dunkel", 4, 4, "#ff6b6b");
    label(d.moving ? "Bild bewegt sich" : "Bild steht", 4, d.dark ? 22 : 4, d.moving ? "#80ed99" : "#aaa");
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

