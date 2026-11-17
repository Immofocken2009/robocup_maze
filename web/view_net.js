"use strict";
// Datei: view_net.js – Ansicht „Netzwerk“: WLAN verwalten

// --- Ansicht: Netzwerk (WLAN verwalten) -------------------------------------------------

async function netApi(path, body) {
  let res;
  try {
    res = await fetch(path, body === undefined ? {} : {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
    });
  } catch (_) {
    throw Object.assign(new Error("Keine Antwort vom Pi (Verbindung unterbrochen?)"), { setup: false });
  }
  let data = null;
  try { data = await res.json(); } catch (_) { /* keine JSON-Antwort */ }
  if (!res.ok) {
    throw Object.assign(new Error((data && data.error) || `Fehler ${res.status}`), { setup: Boolean(data && data.setup) });
  }
  return data;
}

const net = {
  map: { resize() {} },   // keine Karte; showView() ruft resize() an jeder Ansicht auf
  status: null,           // Zustand vom Pi (LAN, WLAN, gespeicherte Netze)
  nets: null,             // Ergebnis der letzten Suche
  open: null,             // Netz, bei dem gerade das Passwortfeld offen ist
  busy: false,
  timer: null,
  messageFrom: "",        // welche Aktion die angezeigte Meldung ausgelöst hat

  init() {
    $("nt-scan").addEventListener("click", () => this.scan(true));
    $("nt-hidden").addEventListener("submit", (e) => {
      e.preventDefault();
      const form = e.target;
      const password = form.elements.password.value;
      form.elements.password.value = "";      // das Passwort bleibt nicht im Feld stehen
      this.connect({ ssid: form.elements.ssid.value, password, hidden: true });
    });
    $("nt-show-hidden").addEventListener("change", (e) => {
      $("nt-hidden").elements.password.type = e.target.checked ? "text" : "password";
    });
  },

  shown() {
    this.hide();
    this.refresh();
    this.scan(false);
    this.timer = setInterval(() => { if (!this.busy) this.refresh(); }, 5000);
  },

  hide() {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  },

  message(text, from = "") {
    this.messageFrom = text ? from : "";
    setList($("nt-messages"), text ? [text] : []);
  },

  setBusy(busy, headline) {
    this.busy = busy;
    for (const el of document.querySelectorAll("#view-net button, #view-net input")) el.disabled = busy;
    if (busy) $("nt-headline").textContent = headline;
  },

  async refresh() {
    try {
      const next = await netApi("/api/network");
      if (this.messageFrom === "refresh") this.message("");
      // Nur neu zeichnen, wenn sich etwas geändert hat; die Netzliste bleibt unberührt (ein getipptes Passwort bleibt stehen)
      const changed = JSON.stringify(next) !== JSON.stringify(this.status);
      this.status = next;
      if (changed) this.renderStatus();
    } catch (e) {
      this.message(e.message, "refresh");
    }
  },

  async scan(rescan) {
    const button = $("nt-scan");
    if (rescan) { button.disabled = true; button.textContent = "Suche …"; }
    try {
      const result = await netApi(`/api/network/scan${rescan ? "?rescan=1" : ""}`);
      this.nets = result.networks;
      if (result.warning) this.message(result.warning, "scan");
      else if (this.messageFrom === "scan") this.message("");
      this.renderNets();
    } catch (e) {
      this.message(e.message, "scan");
    } finally {
      button.disabled = this.busy;
      button.textContent = "Netze suchen";
    }
  },

  // body: {ssid, password?, hidden?, security?}; ohne Passwort für gespeicherte und offene Netze
  async connect(body) {
    if (this.busy) return;
    this.setBusy(true, `Verbinde mit ${body.ssid} …`);
    this.message("");
    try {
      this.status = await netApi("/api/network/connect", body);
      this.open = null;
    } catch (e) {
      this.message(e.message, "connect");
    } finally {
      this.setBusy(false);
      this.renderStatus();
      this.renderNets();
      this.scan(false);       // Liste neu laden: „gespeichert“ und „verbunden“ stimmen jetzt
    }
  },

  async forget(entry) {
    if (this.busy || !confirm(`„${entry.ssid}“ löschen?`)) return;
    this.setBusy(true, "Lösche …");
    this.message("");
    try {
      this.status = await netApi("/api/network/forget", { uuid: entry.uuid });
    } catch (e) {
      this.message(e.message, "forget");
    } finally {
      this.setBusy(false);
      this.renderStatus();
      this.scan(false);
    }
  },

  renderStatus() {
    const s = this.status;
    if (!s) return;
    const lan = s.ethernet, wifi = s.wifi;
    const lanUp = Boolean(lan && lan.connected), wifiUp = Boolean(wifi && wifi.connected);
    $("nt-headline").textContent = lanUp && wifiUp ? `LAN und WLAN ${s.ssid || ""}`.trim()
      : lanUp ? "LAN" : wifiUp ? `WLAN ${s.ssid || ""}`.trim() : "Kein Netz";
    setRows($("nt-info"), [
      ["LAN", !lan ? "kein Anschluss" : lanUp ? `verbunden · ${lan.ip || "–"}` : "nicht verbunden"],
      ["WLAN", !wifi ? "kein Adapter" : wifiUp ? `${s.ssid || "verbunden"} · ${wifi.ip || "–"}`
        : wifi.state.startsWith("connecting") ? "verbindet …" : "nicht verbunden"],
      ["Genutzt wird", s.preferred === "ethernet" ? "LAN" : s.preferred === "wifi" ? "WLAN" : "–"],
    ]);
    $("nt-note").hidden = lanUp;
    $("nt-setup").hidden = s.can_change;
    $("nt-setup-command").textContent = s.setup_command;

    const saved = s.saved;
    $("nt-saved-empty").hidden = saved.length > 0;
    $("nt-saved").replaceChildren(...saved.map((entry) => {
      const meta = entry.active ? "verbunden" : entry.managed ? "" : "System (netplan)";
      const buttons = [];
      if (!entry.active) buttons.push(this.button("Verbinden", () => this.connect({ ssid: entry.ssid })));
      if (entry.managed && !entry.active) buttons.push(this.button("Löschen", () => this.forget(entry)));
      return this.row(entry.ssid, meta, null, buttons, entry.active);
    }));
  },

  renderNets() {
    const list = this.nets || [];
    $("nt-empty").hidden = list.length > 0;
    $("nt-nets").replaceChildren(...list.map((n) => {
      const meta = [n.label, n.in_use ? "verbunden" : n.known ? "gespeichert" : ""].filter(Boolean).join(" · ");
      const buttons = [];
      if (!n.in_use) {
        const usable = n.security === "open" || n.security === "psk" || n.security === "sae";
        const b = this.button("Verbinden", () => {
          if (n.known || n.security === "open") this.connect({ ssid: n.ssid, security: n.security });
          else { this.open = this.open === n.ssid ? null : n.ssid; this.renderNets(); }
        });
        if (!usable) { b.disabled = true; b.title = "Unternehmens-WLAN und WEP werden nicht unterstützt"; }
        buttons.push(b);
      }
      const li = this.row(n.ssid, meta, n.signal, buttons, n.in_use);
      if (this.open === n.ssid) li.append(this.passwordForm(n));
      return li;
    }));
    const field = document.querySelector("#nt-nets input[type=password]");
    if (field) field.focus();
  },

  button(label, onClick) {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = label;
    b.addEventListener("click", onClick);
    return b;
  },

  // Eine Zeile der Liste: Name, Angaben, Signalstärke (0–100, optional), Knöpfe
  row(name, meta, signal, buttons, strong) {
    const li = document.createElement("li");
    li.className = "net";
    const head = document.createElement("div");
    head.className = "net-head";
    const title = document.createElement("span");
    title.className = "net-name" + (strong ? " strong" : "");
    title.textContent = name;
    head.append(title);
    if (meta) {
      const m = document.createElement("span");
      m.className = "muted small";
      m.textContent = meta;
      head.append(m);
    }
    if (signal !== null) {
      const meter = document.createElement("i");
      meter.className = "meter";
      meter.title = `Signal ${signal} %`;
      const fill = document.createElement("i");
      fill.style.width = `${Math.max(4, Math.min(100, signal))}%`;
      meter.append(fill);
      head.append(meter);
    }
    head.append(...buttons);
    li.append(head);
    return li;
  },

  passwordForm(n) {
    const form = document.createElement("form");
    form.className = "net-pass";
    const input = document.createElement("input");
    input.type = "password";
    input.placeholder = "Passwort";
    input.autocomplete = "new-password";
    input.required = true;
    const show = document.createElement("input");
    show.type = "checkbox";
    show.addEventListener("change", () => { input.type = show.checked ? "text" : "password"; });
    const showLabel = document.createElement("label");
    showLabel.className = "check";
    showLabel.append(show, " anzeigen");
    const go = document.createElement("button");
    go.type = "submit";
    go.className = "primary";
    go.textContent = "Verbinden";
    form.append(input, showLabel, go, this.button("Abbrechen", () => { this.open = null; this.renderNets(); }));
    form.addEventListener("submit", (e) => {
      e.preventDefault();
      const password = input.value;
      input.value = "";                       // das Passwort bleibt nicht im Feld stehen
      this.connect({ ssid: n.ssid, password, security: n.security });
    });
    return form;
  },
};

