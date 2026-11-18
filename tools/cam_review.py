"""Kamerabilder eines Laufs sparsam durchsehen (ohne jedes Bild einzeln anzuschauen).

  python3 -m tools.cam_review stats [Lauf]            Tabelle: je Bild Zeit, Aktion, Helligkeit, Kanten, Linien, Bewegung (nur Text)
  python3 -m tools.cam_review sheet [Lauf] [--events] [--n 12] [--cols 4] [--out datei.jpg]
                                                      Kontaktbogen: viele Bilder verkleinert in EINEM Bild, mit Zeit, Aktion und Einzeichnung
                                                      --events: Bilder um Ereignisse (Rutscht, Schleift, Kein Fortschritt, Rückzug, Blockiert),
                                                      sonst gleichmäßig über den Lauf verteilt
  python3 -m tools.cam_review frame [Lauf] ZEIT       ein Bild (Sekunden im Lauf) in voller Größe als Datei
Lauf = Pfad der .jsonl.gz-Datei oder weglassen (neuester). Ausgabedateien liegen in logs/review/ (nicht in Git).
"""
import argparse
import sys
from pathlib import Path

import config
from hardware.run_log import list_runs, read_run
from vision.recording import cam_path

EVENT_CALCS = ("rutscht", "schleift", "kein_fortschritt", "rueckzug", "encoder_ausreisser")
OUT_DIR = config.PROJECT_DIR / "logs" / "review"


class Run:
    """Aufzeichnung mit Kamerabildern: Bildeinträge, Aktionen (aus dem Zustand) und Ereignisse."""

    def __init__(self, path: Path):
        self.path = path
        self.records, self.complete = read_run(path)
        self.frames = [r for r in self.records if r["k"] == "cam"]
        self.cam_file = cam_path(path)
        self.states = [(r["t"], (r["msg"].get("action") or "")) for r in self.records
                       if r["k"] == "state" and r["msg"].get("action") is not None]
        self.events = [(r["t"], r["what"]) for r in self.records if r["k"] == "calc" and r["what"] in EVENT_CALCS]
        self.events += [(t, "blockiert") for t, a in self.states if "lockiert" in a]

    def action_at(self, t: float) -> str:
        last = ""
        for ts, action in self.states:
            if ts > t:
                break
            last = action
        return last

    def frame_near(self, t: float) -> dict | None:
        return min(self.frames, key=lambda f: abs(f["ct"] - t), default=None)

    def image(self, frame: dict):
        from PIL import Image
        import io
        with open(self.cam_file, "rb") as f:
            f.seek(frame["off"])
            return Image.open(io.BytesIO(f.read(frame["len"]))).convert("RGB")


def stats(run: Run) -> str:
    lines = ["   t(s)  Bild  Helligk.  Kanten  Linien  Bewegung  Aktion"]
    for i, f in enumerate(run.frames):
        d = f["det"]
        lines.append(f"{f['ct']:7.1f}  {i:4d}  {d['brightness']:8.1f}  {len(d['vertical_edges']):6d}  {len(d['lines']):6d}  "
                     f"{d['motion']:8.2f}  {run.action_at(f['ct'])[:40]}")
    if run.events:
        lines.append("Ereignisse: " + ", ".join(f"{t:.1f}s {what}" for t, what in run.events))
    return "\n".join(lines)


def pick(run: Run, n: int, events: bool) -> list[tuple[dict, str]]:
    """Bilder für den Kontaktbogen: (Bild, Beschriftung)."""
    if not run.frames:
        return []
    chosen: dict[int, str] = {}
    if events:
        for t, what in run.events:
            f = run.frame_near(t)
            if f is not None:
                chosen.setdefault(run.frames.index(f), what)
        rest = n - len(chosen)
    else:
        rest = n
    if rest > 0 and not events:
        step = max(1, len(run.frames) // rest)
        for i in range(0, len(run.frames), step):
            chosen.setdefault(i, "")
    items = sorted(chosen.items())[:n]
    return [(run.frames[i], label) for i, label in items]


def draw(img, det: dict, text: str):
    from PIL import ImageDraw
    g = ImageDraw.Draw(img)
    k = img.width / det["w"]
    for l in det["lines"]:
        g.line([l["x0"] * k, l["y0"] * k, l["x1"] * k, l["y1"] * k], fill=(255, 210, 63), width=2)
    for e in det["vertical_edges"]:
        g.line([e["x"] * k, e["y0"] * k, e["x"] * k, e["y1"] * k], fill=(76, 201, 240), width=2)
    g.rectangle([0, 0, img.width, 14], fill=(0, 0, 0))
    g.text((3, 1), text, fill=(255, 255, 255))
    return img


def sheet(run: Run, n: int, cols: int, events: bool, out: Path) -> Path:
    from PIL import Image
    items = pick(run, n, events)
    if not items:
        raise SystemExit("Keine Kamerabilder in dieser Aufzeichnung")
    tw = 400
    th = int(tw * 0.75)
    rows = (len(items) + cols - 1) // cols
    canvas = Image.new("RGB", (cols * tw, rows * th), (30, 30, 30))
    for i, (f, label) in enumerate(items):
        tile = run.image(f).resize((tw, th))
        action = run.action_at(f["ct"])
        draw(tile, f["det"], f"{f['ct']:.1f}s {label} {action}"[:60])
        canvas.paste(tile, ((i % cols) * tw, (i // cols) * th))
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out, "JPEG", quality=80)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("stats", "sheet", "frame"))
    ap.add_argument("args", nargs="*")
    ap.add_argument("--events", action="store_true")
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    rest = list(a.args)
    t_arg = rest.pop() if a.command == "frame" and rest else None
    path = Path(rest[0]) if rest else (list_runs() or [None])[0]
    if path is None or not path.exists():
        raise SystemExit("Keine Aufzeichnung gefunden")
    run = Run(path)
    if a.command == "stats":
        print(stats(run))
    elif a.command == "sheet":
        print(sheet(run, a.n, a.cols, a.events, Path(a.out) if a.out else OUT_DIR / f"{path.name.split('.')[0]}_sheet.jpg"))
    else:
        f = run.frame_near(float(t_arg)) if t_arg else None
        if f is None:
            raise SystemExit("Zeit angeben, z.B.: frame <Lauf> 123.4")
        out = Path(a.out) if a.out else OUT_DIR / f"{path.name.split('.')[0]}_{f['ct']:.1f}s.jpg"
        out.parent.mkdir(parents=True, exist_ok=True)
        run.image(f).save(out, "JPEG", quality=90)
        print(out)


if __name__ == "__main__":
    sys.exit(main())
