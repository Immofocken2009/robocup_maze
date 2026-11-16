"""Gemeinsame Helfer der JSON-Schnittstelle (/api/...): Antwort als JSON, Anfrage als JSON-Objekt lesen."""
import json
import math

from aiohttp import web


def _finite(value):
    """NaN/unendlich gibt es in JSON nicht: durch null ersetzen."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_finite(v) for v in value]
    return value


def _json(data, status: int = 200) -> web.Response:
    return web.Response(text=json.dumps(_finite(data), ensure_ascii=False), content_type="application/json",
                        status=status)


async def _body(request: web.Request) -> dict:
    try:
        data = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text="JSON erwartet") from None
    if not isinstance(data, dict):
        raise web.HTTPBadRequest(text="JSON-Objekt erwartet")
    return data
