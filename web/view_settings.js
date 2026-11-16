"use strict";
// Datei: view_settings.js – Ansicht „Einstellungen“: Lenkung

// --- Ansicht: Einstellungen (Lenkung) ---------------------------------------------------

const settings = {
  map: { resize() {} },
  data: null,

  init() {
    $("st-save").addEventListener("click", () => this.save());
    $("st-reset").addEventListener("click", () => this.send({ reset: true }, "Auf Standard zurückgesetzt."));
  },

  shown() { this.load(); },

  message(text) { setList($("st-messages"), text ? [text] : []); },

  async load() {
    try {
      this.data = await netApi("/api/settings");
      this.render();
    } catch (e) {
      this.message(e.message);
    }
  },

  render() {
    const form = $("st-form");
    form.textContent = "";
    for (const s of this.data.settings) {
      const row = document.createElement("div");
      row.className = "setting";
      const label = document.createElement("label");
      label.htmlFor = `st-${s.name}`;
      label.textContent = s.label;
      const help = document.createElement("p");
      help.className = "muted small";
      help.textContent = `${s.help} Standard: ${s.default} ${s.unit}.`;
      const number = document.createElement("input");
      number.type = "number";
      number.id = `st-${s.name}`;
      number.name = s.name;
      number.min = s.min; number.max = s.max; number.step = "any";
      number.value = s.value;
      const range = document.createElement("input");
      range.type = "range";
      range.min = s.min; range.max = s.max; range.step = (s.max - s.min) / 400;
      range.value = s.value;
      range.setAttribute("aria-label", s.label);
      number.addEventListener("input", () => { range.value = number.value; });
      range.addEventListener("input", () => { number.value = Math.round(range.value * 100) / 100; });
      const unit = document.createElement("span");
      unit.className = "muted small";
      unit.textContent = s.unit;
      const controls = document.createElement("div");
      controls.className = "controls";
      controls.append(range, number, unit);
      row.append(label, controls, help);
      form.append(row);
    }
    $("st-note").textContent = this.data.note;
  },

  async save() {
    const values = {};
    for (const s of this.data.settings) values[s.name] = Number($(`st-${s.name}`).value);
    await this.send({ values }, "Gespeichert.");
  },

  async send(body, ok) {
    try {
      this.data = await netApi("/api/settings", body);
      this.render();
      this.message(ok);
    } catch (e) {
      this.message(e.message);
    }
  },
};

