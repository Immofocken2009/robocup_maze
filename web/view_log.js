"use strict";
// Datei: view_log.js – Ansicht „Aufzeichnung“: Verlaufsdiagramm und Lauf-Wiedergabe

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
  "rückwärts": "Start der Fahrt rückwärts",
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

