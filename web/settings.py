"""Einstellungen (Reiter „Einstellungen“): Lenkwerte lesen und speichern, Pfade /api/settings."""
from aiohttp import web

import config
from web.api import _json

def settings_state() -> dict:
    saved = config.read_settings()
    return {"settings": [{"name": name, "label": label, "unit": unit, "min": low, "max": high, "help": text,
                          "default": getattr(config, name), "value": saved.get(name, getattr(config, name))}
                         for name, (label, unit, low, high, text) in config.STEER_SETTINGS.items()],
            "note": "Gilt ab der nächsten Fahrt des Roboters, auch mitten im Lauf."}


async def settings_get(_request):
    return _json(settings_state())


async def settings_save(request):
    """Body: {"values": {Name: Zahl}} speichert (nur Abweichungen vom Standard), {"reset": true} stellt alles zurück."""
    try:
        body = await request.json()
    except ValueError:
        return _json({"error": "Ungültige Anfrage"}, 400)
    if not isinstance(body, dict):
        return _json({"error": "Ungültige Anfrage"}, 400)
    saved = {}
    if not body.get("reset"):
        values = body.get("values")
        if not isinstance(values, dict):
            return _json({"error": "Ungültige Anfrage"}, 400)
        for name, (label, unit, low, high, _text) in config.STEER_SETTINGS.items():
            if name not in values:
                continue
            v = values[name]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not low <= v <= high:
                return _json({"error": f"{label}: bitte eine Zahl von {low:g} bis {high:g} {unit} angeben"}, 400)
            if float(v) != float(getattr(config, name)):
                saved[name] = float(v)
    try:
        config.write_settings(saved)
    except OSError as exc:
        return _json({"error": f"Speichern nicht möglich: {exc.strerror or exc}"}, 500)
    return _json(settings_state())


def add_settings_routes(app: web.Application):
    app.router.add_get("/api/settings", settings_get)
    app.router.add_post("/api/settings", settings_save)
