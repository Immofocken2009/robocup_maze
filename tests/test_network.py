"""Tests der Netzwerk-Verwaltung der Webseite (web/server.py, mit nachgebautem nmcli, nie das echte Netzwerk)."""
import json
import re
import unittest

import config
from web.server import (NEEDED_PERMISSIONS, SETUP_COMMAND, Network, NetworkError, add_network_routes,
                        parse_networks, split_terse, wifi_security)


class FakeNm:
    """Ersetzt web.server.run_command: antwortet wie nmcli/ip aus einer Tabelle und merkt sich alle Aufrufe."""

    def __init__(self):
        self.calls: list[tuple] = []
        self.profiles = {      # uuid -> (Name, Netzname, Gerät)
            "aebdb66f-503b-318b-aaca-428849272844": ("netplan-wlan0-Immoh", "Immoh", ""),
        }
        self.permissions = "yes"
        self.rescan_error = None
        self.fail_up = False
        self.wifi_state = "disconnected"
        self.scan_text = ("*:Heim\\:Netz:80:WPA2\n :Heim\\:Netz:50:WPA2\n :Gast:40:WPA2 WPA3\n"
                          " ::30:WPA2\n :Offen:20:\n :Firma:60:WPA2 802.1X\n :Nur3:55:WPA3\n")

    async def __call__(self, *command, timeout=15):
        self.calls.append(command)
        if command[0] == "ip":
            if "addr" in command:
                return 0, json.dumps([{"ifname": "eth0", "addr_info": [{"family": "inet", "local": "10.0.0.5"}]}]), ""
            return 0, json.dumps([{"dst": "default", "dev": "eth0", "metric": 100}]), ""
        args = list(command[1:])
        if args[:2] == ["-t", "-f"] and args[2] == "DEVICE,TYPE,STATE,CONNECTION":
            return 0, f"eth0:ethernet:connected:netplan-eth0\nwlan0:wifi:{self.wifi_state}:\nlo:loopback:unmanaged:\n", ""
        if args[:2] == ["-t", "-f"] and args[2] == "UUID,NAME,TYPE,AUTOCONNECT,DEVICE":
            rows = [f"{u}:{n.replace(':', chr(92) + ':')}:802-11-wireless:yes:{d}"      # wie nmcli: Doppelpunkt maskiert
                    for u, (n, _, d) in self.profiles.items()]
            return 0, "\n".join(["11111111-2222-3333-4444-555555555555:netplan-eth0:802-3-ethernet:yes:eth0"] + rows), ""
        if args[:3] == ["--escape", "no", "-g"]:
            return 0, self.profiles[args[-1]][1] + "\n", ""
        if args[2:3] == ["PERMISSION,VALUE"]:
            return 0, "".join(f"{p}:{self.permissions}\n" for p in NEEDED_PERMISSIONS), ""
        if "wifi" in args and "list" in args:
            if args[-1] == "yes" and self.rescan_error:
                return 1, "", self.rescan_error
            return 0, self.scan_text, ""
        if args[:2] == ["connection", "add"]:
            uuid = f"00000000-0000-0000-0000-{len(self.profiles):012d}"
            self.profiles[uuid] = (args[args.index("con-name") + 1], args[args.index("ssid") + 1], "")
            return 0, f"Connection '{self.profiles[uuid][0]}' ({uuid}) successfully added.\n", ""
        if "up" in args:
            if self.fail_up:
                return 4, "", "Error: Connection activation failed: (7) Secrets were required, but not provided."
            uuid = args[args.index("uuid") + 1]
            name, ssid, _ = self.profiles[uuid]
            self.profiles[uuid] = (name, ssid, "wlan0")
            return 0, "Connection successfully activated\n", ""
        if args[:2] == ["connection", "delete"]:
            del self.profiles[args[args.index("uuid") + 1]]
            return 0, "deleted\n", ""
        if args[:2] == ["connection", "modify"]:
            return 0, "", ""
        raise AssertionError(f"unerwarteter Aufruf: {command}")

    def commands(self, word: str) -> list[tuple]:
        return [c for c in self.calls if word in c]


class NetworkTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.nm = FakeNm()
        self.net = Network(self.nm)

    def test_split_terse_unescapes_colon_and_backslash(self):
        self.assertEqual(split_terse("a\\:b:c\\\\d:"), ["a:b", "c\\d", ""])

    def test_security_kinds(self):
        for text, kind in [("", "open"), ("--", "open"), ("WPA2", "psk"), ("WPA1 WPA2", "psk"),
                           ("WPA2 WPA3", "psk"), ("WPA3", "sae"), ("WPA2 802.1X", "enterprise"), ("WEP", "unsupported")]:
            self.assertEqual(wifi_security(text)[0], kind, text)

    def test_parse_networks_dedupes_and_sorts(self):
        nets = parse_networks(self.nm.scan_text, known={"Gast"})
        self.assertEqual([n["ssid"] for n in nets], ["Heim:Netz", "Firma", "Nur3", "Gast", "Offen"])  # Name mit Doppelpunkt, stärkstes zuerst, ohne leeren Namen
        self.assertTrue(nets[0]["in_use"] and nets[0]["signal"] == 80)
        self.assertTrue(next(n for n in nets if n["ssid"] == "Gast")["known"])

    async def test_status(self):
        status = await self.net.status()
        self.assertEqual(status["ethernet"]["ip"], "10.0.0.5")
        self.assertEqual(status["preferred"], "ethernet")
        self.assertFalse(status["wifi"]["connected"])
        self.assertTrue(status["can_change"])
        self.assertEqual([s["managed"] for s in status["saved"]], [False])
        self.nm.permissions = "auth"
        self.assertFalse((await self.net.status())["can_change"])

    async def test_connect_new_secured_network(self):
        status = await self.net.connect("Heim:Netz", "geheim1234", security="psk")
        add = self.nm.commands("add")[0]
        self.assertIn("ipv4.route-metric", add)
        self.assertEqual(add[add.index("ipv4.route-metric") + 1], str(config.NET_WIFI_METRIC))
        self.assertEqual(add[add.index("wifi-sec.key-mgmt") + 1], "wpa-psk")
        self.assertEqual(add[add.index("wifi-sec.psk-flags") + 1], "0")
        self.assertEqual(add[add.index("802-11-wireless.powersave") + 1], str(config.NET_WIFI_POWERSAVE))
        self.assertEqual(add[add.index("con-name") + 1], config.NET_WIFI_PREFIX + "Heim:Netz")
        self.assertTrue(any(s["ssid"] == "Heim:Netz" and s["managed"] for s in status["saved"]))

    async def test_connect_variants(self):
        await self.net.connect("Offen", None, security="open")
        self.assertNotIn("wifi-sec.key-mgmt", self.nm.commands("add")[0])
        await self.net.connect("Nur3", "passwort", security="sae")
        self.assertEqual(self.nm.commands("add")[1][self.nm.commands("add")[1].index("wifi-sec.key-mgmt") + 1], "sae")
        await self.net.connect("Versteckt", "geheim1234", hidden=True)
        add = self.nm.commands("add")[2]
        self.assertEqual(add[add.index("802-11-wireless.hidden") + 1], "yes")

    async def test_failed_connect_removes_new_profile_and_hides_password(self):
        self.nm.fail_up = True
        before = set(self.nm.profiles)
        with self.assertRaises(NetworkError) as ctx:
            await self.net.connect("Heim:Netz", "geheim1234", security="psk")
        self.assertEqual(set(self.nm.profiles), before)
        self.assertNotIn("geheim1234", str(ctx.exception))
        self.assertIn("Passwort", str(ctx.exception))

    async def test_existing_system_profile_is_activated_not_duplicated(self):
        await self.net.connect("Immoh", None)
        self.assertEqual(self.nm.commands("add"), [])
        self.assertEqual(len(self.nm.commands("up")), 1)

    async def test_invalid_input_never_reaches_nmcli(self):
        for ssid, password in [("", None), ("x" * 33, None), ("a\nb", None), ("Netz", "kurz"), ("Netz", "ü" * 10)]:
            with self.assertRaises(NetworkError, msg=(ssid, password)):
                await self.net.connect(ssid, password, security="psk" if password else "open")
        with self.assertRaises(NetworkError):
            await self.net.connect("Firma", "geheim1234", security="enterprise")
        self.assertEqual(self.nm.commands("add"), [])

    async def test_forget_only_what_the_page_created(self):
        await self.net.connect("Gast", "geheim1234", security="psk")
        mine = next(u for u, (n, _, _) in self.nm.profiles.items() if n.startswith(config.NET_WIFI_PREFIX))
        with self.assertRaises(NetworkError) as ctx:        # gerade verbunden
            await self.net.forget(mine)
        self.assertIn("verbunden", str(ctx.exception))
        self.nm.profiles[mine] = (self.nm.profiles[mine][0], "Gast", "")
        await self.net.forget(mine)
        self.assertNotIn(mine, self.nm.profiles)
        with self.assertRaises(NetworkError) as ctx:        # netplan-Profil
            await self.net.forget("aebdb66f-503b-318b-aaca-428849272844")
        self.assertIn("netplan", str(ctx.exception))
        with self.assertRaises(NetworkError):
            await self.net.forget("../etc")
        self.assertEqual(len(self.nm.commands("delete")), 1)

    async def test_permission_denied_explains_setup(self):
        async def denied(*command, timeout=15):
            return 1, "", "Error: Failed to add/activate new connection: Insufficient privileges"
        with self.assertRaises(NetworkError) as ctx:
            await Network(denied).connect("Heim", "geheim1234", security="psk")
        self.assertTrue(ctx.exception.setup)
        self.assertEqual(ctx.exception.status, 403)
        self.assertIn(SETUP_COMMAND, str(ctx.exception))

    async def test_scan_falls_back_to_cached_list(self):
        self.nm.rescan_error = "Error: Scanning not allowed immediately following previous scan."
        result = await self.net.scan(rescan=True)
        self.assertFalse(result["rescanned"])
        self.assertIn("letzten Suche", result["warning"])
        self.assertTrue(result["networks"])
        self.assertTrue((await self.net.scan(rescan=False))["networks"])

    async def test_http_api(self):
        from aiohttp import web
        from aiohttp.test_utils import TestClient, TestServer
        app = web.Application()
        app["network"] = self.net
        add_network_routes(app)
        async with TestClient(TestServer(app)) as client:
            reply = await client.get("/api/network")
            self.assertEqual((await reply.json())["ethernet"]["ip"], "10.0.0.5")
            reply = await client.post("/api/network/connect", json={"ssid": "", "password": None})
            self.assertEqual(reply.status, 400)
            self.assertIn("Netzname", (await reply.json())["error"])
            reply = await client.post("/api/network/connect", data="kein json")
            self.assertEqual(reply.status, 400)
            reply = await client.post("/api/network/forget", json={"uuid": "aebdb66f-503b-318b-aaca-428849272844"})
            self.assertEqual(reply.status, 400)
            reply = await client.get("/api/network/scan")
            self.assertEqual(reply.status, 200)
            self.assertEqual(self.nm.commands("add"), [])

    def test_polkit_rule_grants_exactly_what_the_code_needs(self):
        rules = (config.PROJECT_DIR / "robocup-network.rules").read_text()
        granted = set(re.findall(r'"(org\.freedesktop\.NetworkManager\.[\w.-]+)"', rules))
        self.assertEqual(granted, set(NEEDED_PERMISSIONS))
        self.assertIn('subject.user == "immo"', rules)
        service = (config.PROJECT_DIR / "robocup-web.service").read_text()
        self.assertIn("User=immo", service)

