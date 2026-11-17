"use strict";
// Datei: app.js – Ansicht wechseln, Start (Verdrahtung der Ansichten)

// --- Ansicht wechseln ------------------------------------------------------------------

const VIEWS = { robot, sim, log: logView, net, set: settings };

function showView(name) {
  view = name;
  for (const v of Object.keys(VIEWS)) {
    $(`view-${v}`).hidden = v !== name;
    $(`tab-${v}`).setAttribute("aria-selected", String(v === name));
  }
  try { localStorage.setItem("view", name); } catch (_) { /* ohne Speicher geht es auch */ }
  VIEWS[name].map.resize();
  if (name === "log") logView.shown();
  if (name === "net") net.shown(); else net.hide();
  if (name === "set") settings.shown();
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
net.init();
settings.init();
let savedView = null;
try { savedView = localStorage.getItem("view"); } catch (_) { /* egal */ }
showView(savedView in VIEWS ? savedView : "robot");
connect();
