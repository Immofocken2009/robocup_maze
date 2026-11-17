"""Netzwerk (Reiter „Netzwerk“): WLAN-Verwaltung über NetworkManager (nmcli), Pfade /api/network."""
import asyncio
import json
import os
import re

from aiohttp import web

import config
from web.api import _body, _json

# --- Netzwerk (Reiter „Netzwerk“) -----------------------------------------------------------
#
# WLAN über NetworkManager (nmcli, ohne Shell). Die Seite legt WLAN-Verbindungen an (Name robocup-<Netz>), startet
# und löscht sie. LAN behält Vorrang: Verbindungen der Seite bekommen Routenmetrik 600, LAN hat 100.
# Sie laufen ohne WLAN-Energiesparen (das kann die Live-Seite über WLAN verzögern).
# Ein Passwort steht nur während des Aufrufs in der Befehlszeile, nie in Meldungen oder im Log.
# Der Dienst läuft ohne angemeldete Sitzung: ohne robocup-network.rules (polkit) verweigert NetworkManager Änderungen.

NMCLI = "nmcli"
UUID = re.compile(r"^[0-9a-fA-F]{8}(-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}$")
SETUP_COMMAND = (f"sudo install -m 644 {config.PROJECT_DIR / 'robocup-network.rules'} "
                 "/etc/polkit-1/rules.d/50-robocup-network.rules")
NEEDED_PERMISSIONS = ("org.freedesktop.NetworkManager.settings.modify.system",
                      "org.freedesktop.NetworkManager.network-control",
                      "org.freedesktop.NetworkManager.wifi.scan")


class NetworkError(Exception):
    """Fehler mit Text für die Anzeige (enthält nie ein Passwort oder die Befehlszeile)."""

    def __init__(self, message: str, status: int = 400, setup: bool = False):
        super().__init__(message)
        self.status = status
        self.setup = setup


async def run_command(*command: str, timeout: float = config.NET_QUERY_S) -> tuple[int, str, str]:
    """Programm ohne Shell ausführen. Ergebnis: (Rückgabewert, Ausgabe, Fehlerausgabe); Meldungen englisch, UTF-8."""
    env = {k: v for k, v in os.environ.items() if k != "LANG" and not k.startswith("LC_")}
    env.update(LANG="C.UTF-8", LC_MESSAGES="C")
    try:
        proc = await asyncio.create_subprocess_exec(*command, stdin=asyncio.subprocess.DEVNULL,
                                                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                                                    env=env)
    except FileNotFoundError:
        raise NetworkError(f"{command[0]} ist nicht installiert", 503) from None
    try:
        out, err = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        raise NetworkError("Keine Antwort vom Netzwerk (Zeitüberschreitung)", 504) from None
    return proc.returncode, out.decode(errors="replace"), err.decode(errors="replace")


def split_terse(line: str) -> list[str]:
    """Eine Zeile aus „nmcli -t“ in Felder teilen. Doppelpunkt und Backslash im Text sind mit Backslash maskiert."""
    fields, current, i = [], [], 0
    while i < len(line):
        ch = line[i]
        if ch == "\\" and i + 1 < len(line):
            current.append(line[i + 1])
            i += 2
            continue
        if ch == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(ch)
        i += 1
    fields.append("".join(current))
    return fields


def fields_of(line: str, count: int) -> list[str]:
    return (split_terse(line) + [""] * count)[:count]


def explain(error: str, secret: str | None = None) -> NetworkError:
    """Fehlermeldung von nmcli in verständlichen Text übersetzen (das Passwort wird entfernt)."""
    text = " ".join(error.split())
    if secret:
        text = text.replace(secret, "***")
    low = text.lower()
    if "not authorized" in low or "insufficient privileges" in low or "authorization failed" in low:
        return NetworkError("Keine Berechtigung, das Netzwerk zu ändern. Einmal einrichten: " + SETUP_COMMAND,
                            403, setup=True)
    if "not running" in low or "could not create nmclient" in low:
        return NetworkError("NetworkManager läuft nicht", 503)
    if "secrets were required" in low or "no secrets" in low:
        return NetworkError("Passwort falsch oder fehlt")
    if "no network with ssid" in low:
        return NetworkError("Netz nicht gefunden (Name falsch oder zu weit weg)")
    if "timeout" in low or "timed out" in low:
        return NetworkError("Verbinden hat zu lange gedauert (Passwort falsch oder Netz zu weit weg)")
    if "no suitable device" in low or "device" in low and "not found" in low:
        return NetworkError("Kein passender WLAN-Adapter gefunden", 503)
    return NetworkError("Fehler von NetworkManager: " + (text[:200] or "unbekannt"))


def wifi_security(security: str) -> tuple[str, str]:
    """Spalte SECURITY aus der Suche: (Art, Anzeige). Art: open, psk (WPA/WPA2), sae (nur WPA3), enterprise, unsupported."""
    tokens = security.split()
    if not tokens or tokens == ["--"]:
        return "open", "offen"
    if "802.1X" in tokens:
        return "enterprise", security
    if "WEP" in tokens:
        return "unsupported", security
    if {"WPA1", "WPA2", "WPA"} & set(tokens):
        return "psk", security
    if "WPA3" in tokens:
        return "sae", security
    return "unsupported", security


def parse_networks(output: str, known: set[str]) -> list[dict]:
    """Ergebnis von „nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY device wifi list“: jedes Netz einmal (das stärkste),
    ohne versteckte (leerer Name), stärkste zuerst."""
    best: dict[str, dict] = {}
    for line in output.splitlines():
        in_use, ssid, signal, security = fields_of(line, 4)
        if not ssid:
            continue
        kind, label = wifi_security(security)
        entry = {"ssid": ssid, "signal": int(signal) if signal.isdigit() else 0, "security": kind,
                 "label": label, "in_use": in_use.strip() == "*", "known": ssid in known}
        old = best.get(ssid)
        if old is not None:
            entry["in_use"] = entry["in_use"] or old["in_use"]
            if old["signal"] >= entry["signal"]:
                old["in_use"] = entry["in_use"]
                continue
        best[ssid] = entry
    return sorted(best.values(), key=lambda n: (-n["in_use"], -n["signal"], n["ssid"].lower()))


def check_ssid(ssid) -> str:
    if not isinstance(ssid, str) or not ssid or len(ssid.encode()) > 32 or any(ord(c) < 32 or ord(c) == 127 for c in ssid):
        raise NetworkError("Netzname: 1 bis 32 Zeichen, ohne Steuerzeichen")
    return ssid


def check_password(password, kind: str) -> str:
    if not isinstance(password, str) or not password:
        raise NetworkError("Passwort fehlt")
    if kind == "psk":
        if not (8 <= len(password) <= 63 and all(32 <= ord(c) < 127 for c in password)) \
                and not re.fullmatch(r"[0-9a-fA-F]{64}", password):
            raise NetworkError("Passwort: 8 bis 63 Zeichen (oder 64 Hex-Zeichen), nur Buchstaben, Zahlen und Zeichen ohne Umlaute")
    elif len(password) > 128 or any(ord(c) < 32 or ord(c) == 127 for c in password):
        raise NetworkError("Passwort: höchstens 128 Zeichen, ohne Steuerzeichen")
    return password


class Network:
    """WLAN-Verwaltung. run: Programm ausführen (in Tests ersetzt, sie greifen nie auf das echte Netzwerk zu)."""

    def __init__(self, run=run_command):
        self.run = run
        self.lock = asyncio.Lock()      # Änderungen nacheinander

    async def nmcli(self, *args: str, timeout: float = config.NET_QUERY_S, secret: str | None = None) -> str:
        code, out, err = await self.run(NMCLI, *args, timeout=timeout)
        if code != 0:
            raise explain(err or out, secret)
        return out

    # --- Lesen -------------------------------------------------------------------

    async def devices(self) -> dict[str, dict]:
        out = await self.nmcli("-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status")
        found: dict[str, dict] = {}
        for line in out.splitlines():
            name, kind, state, connection = fields_of(line, 4)
            if kind in ("ethernet", "wifi") and kind not in found:
                found[kind] = {"name": name, "state": state, "connected": state == "connected",
                               "connection": connection or None, "ip": None}
        return found

    async def addresses(self) -> tuple[dict[str, str], str | None]:
        """IP-Adressen je Schnittstelle und die Schnittstelle mit der besten Standardroute (nur zur Anzeige)."""
        ips: dict[str, str] = {}
        preferred = None
        try:
            code, out, _ = await self.run("ip", "-j", "-4", "addr", "show", timeout=config.NET_QUERY_S)
            if code == 0:
                for item in json.loads(out):
                    for info in item.get("addr_info", []):
                        if info.get("family") == "inet":
                            ips.setdefault(item["ifname"], info["local"])
            code, out, _ = await self.run("ip", "-j", "route", "show", "default", timeout=config.NET_QUERY_S)
            routes = json.loads(out) if code == 0 else []
            if routes:
                preferred = min(routes, key=lambda r: r.get("metric", 0))["dev"]
        except (NetworkError, ValueError, KeyError, TypeError):
            pass
        return ips, preferred

    async def ssid_of(self, uuid: str) -> str:
        try:
            return (await self.nmcli("--escape", "no", "-g", "802-11-wireless.ssid", "connection", "show",
                                     "uuid", uuid)).rstrip("\n")
        except NetworkError:
            return ""

    async def saved(self) -> list[dict]:
        """Gespeicherte WLAN-Verbindungen. managed: von dieser Seite angelegt (nur die lassen sich löschen);
        die anderen (netplan) baut das System beim Neustart aus seiner Konfiguration neu."""
        out = await self.nmcli("-t", "-f", "UUID,NAME,TYPE,AUTOCONNECT,DEVICE", "connection", "show")
        rows = [r for r in (fields_of(line, 5) for line in out.splitlines()) if r[2] == "802-11-wireless"]
        ssids = await asyncio.gather(*(self.ssid_of(r[0]) for r in rows))
        return [{"uuid": uuid, "name": name, "ssid": ssid or name, "autoconnect": auto == "yes",
                 "device": device or None, "active": bool(device),
                 "managed": name.startswith(config.NET_WIFI_PREFIX)}
                for (uuid, name, _, auto, device), ssid in zip(rows, ssids)]

    async def can_change(self) -> bool:
        try:
            out = await self.nmcli("-t", "-f", "PERMISSION,VALUE", "general", "permissions")
        except NetworkError:
            return False
        values = dict(fields_of(line, 2) for line in out.splitlines())
        return all(values.get(p) == "yes" for p in NEEDED_PERMISSIONS)

    async def status(self) -> dict:
        devices = await self.devices()
        saved = await self.saved()
        ips, preferred_dev = await self.addresses()
        for device in devices.values():
            device["ip"] = ips.get(device["name"])
        wifi = devices.get("wifi")
        ssid = next((s["ssid"] for s in saved if wifi and s["device"] == wifi["name"]), None)
        return {"ethernet": devices.get("ethernet"), "wifi": wifi, "ssid": ssid,
                "preferred": next((k for k, d in devices.items() if d["name"] == preferred_dev), None),
                "saved": saved, "can_change": await self.can_change(), "setup_command": SETUP_COMMAND}

    async def scan(self, rescan: bool) -> dict:
        """Netze in der Nähe. Ohne rescan die Liste der letzten Suche (geht ohne Berechtigung, ist sofort da)."""
        known = {s["ssid"] for s in await self.saved()}
        args = ["-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi", "list", "--rescan"]
        warning = None
        try:
            out = await self.nmcli(*args, "yes" if rescan else "no", timeout=config.NET_SCAN_S)
        except NetworkError as e:
            if not rescan:
                raise
            warning = f"{e} (angezeigt wird die Liste der letzten Suche)"
            out = await self.nmcli(*args, "no")
        return {"networks": parse_networks(out, known), "rescanned": rescan and warning is None, "warning": warning}

    # --- Ändern ------------------------------------------------------------------

    async def add_profile(self, wifi: str, ssid: str, kind: str, password: str | None, hidden: bool) -> str:
        args = ["connection", "add", "type", "wifi", "ifname", wifi, "con-name", config.NET_WIFI_PREFIX + ssid,
                "ssid", ssid, "connection.autoconnect", "yes",
                "ipv4.route-metric", str(config.NET_WIFI_METRIC), "ipv6.route-metric", str(config.NET_WIFI_METRIC),
                "802-11-wireless.powersave", str(config.NET_WIFI_POWERSAVE)]
        if hidden:
            args += ["802-11-wireless.hidden", "yes"]
        if kind in ("psk", "sae"):
            args += ["wifi-sec.key-mgmt", "wpa-psk" if kind == "psk" else "sae", "wifi-sec.psk", password,
                     "wifi-sec.psk-flags", "0"]          # Passwort für alle Benutzer, damit es auch ohne Anmeldung verbindet
        out = await self.nmcli(*args, secret=password)
        match = re.search(r"\(([0-9a-fA-F-]{36})\)", out)
        if match:
            return match.group(1)
        found = [s["uuid"] for s in await self.saved() if s["managed"] and s["ssid"] == ssid]
        if not found:
            raise NetworkError("Verbindung angelegt, aber nicht gefunden")
        return found[-1]

    async def connect(self, ssid, password=None, hidden: bool = False, security: str | None = None) -> dict:
        """Mit einem Netz verbinden. Gibt es für den Namen schon eine Verbindung, wird sie gestartet (mit neuem
        Passwort, wenn die Seite sie angelegt hat). Sonst wird eine angelegt, bei Misserfolg wieder gelöscht."""
        check_ssid(ssid)
        if security in ("enterprise", "unsupported"):
            raise NetworkError("Dieses Netz (Unternehmens-WLAN oder WEP) wird nicht unterstützt")
        kind = security if security in ("open", "psk", "sae") else ("psk" if password else "open")
        async with self.lock:
            wifi = (await self.devices()).get("wifi")
            if wifi is None:
                raise NetworkError("Kein WLAN-Adapter gefunden", 503)
            existing = next((s for s in await self.saved() if s["ssid"] == ssid), None)
            created = existing is None
            if existing is None:
                if kind != "open":
                    check_password(password, kind)
                uuid = await self.add_profile(wifi["name"], ssid, kind, password, hidden)
            else:
                uuid = existing["uuid"]
                if password and existing["managed"]:
                    check_password(password, kind if kind != "open" else "psk")
                    await self.nmcli("connection", "modify", "uuid", uuid, "wifi-sec.psk", password,
                                     "wifi-sec.psk-flags", "0", secret=password)
            try:
                await self.nmcli("-w", str(config.NET_CONNECT_S), "connection", "up", "uuid", uuid, "ifname",
                                 wifi["name"], timeout=config.NET_CONNECT_S + 10, secret=password)
            except NetworkError:
                if created:         # falsches Passwort soll keine Verbindung zurücklassen, die ständig neu versucht
                    try:
                        await self.nmcli("connection", "delete", "uuid", uuid)
                    except NetworkError:
                        pass
                raise
        return await self.status()

    async def forget(self, uuid) -> dict:
        """Eine von der Seite angelegte, gerade nicht aktive WLAN-Verbindung löschen."""
        if not isinstance(uuid, str) or not UUID.match(uuid):
            raise NetworkError("Ungültige Verbindung")
        async with self.lock:
            entry = next((s for s in await self.saved() if s["uuid"] == uuid), None)
            if entry is None:
                raise NetworkError("Verbindung nicht gefunden", 404)
            if not entry["managed"]:
                raise NetworkError("Systemverbindung (netplan): lässt sich nur in der Systemkonfiguration ändern")
            if entry["active"]:
                raise NetworkError("Gerade verbunden: erst mit einem anderen Netz verbinden oder das Kabel einstecken")
            await self.nmcli("connection", "delete", "uuid", uuid)
        return await self.status()


async def _network(request: web.Request, call) -> web.Response:
    try:
        return _json(await call(request.app["network"]))
    except NetworkError as e:
        return _json({"error": str(e), "setup": e.setup}, e.status)


async def network_status(request: web.Request) -> web.Response:
    return await _network(request, lambda net: net.status())


async def network_scan(request: web.Request) -> web.Response:
    return await _network(request, lambda net: net.scan(request.query.get("rescan") == "1"))


async def network_connect(request: web.Request) -> web.Response:
    data = await _body(request)
    return await _network(request, lambda net: net.connect(
        data.get("ssid"), data.get("password") or None, data.get("hidden") is True, data.get("security")))


async def network_forget(request: web.Request) -> web.Response:
    data = await _body(request)
    return await _network(request, lambda net: net.forget(data.get("uuid")))


def add_network_routes(app: web.Application):
    app.router.add_get("/api/network", network_status)
    app.router.add_get("/api/network/scan", network_scan)
    app.router.add_post("/api/network/connect", network_connect)
    app.router.add_post("/api/network/forget", network_forget)
