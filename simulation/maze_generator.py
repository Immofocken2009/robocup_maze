"""Zufällige Labyrinthe nach dem Regelwerk RoboCupJunior Rescue Maze 2026 (eine Ebene).

Umgesetzt:
- Kacheln mit Wänden auf den Kanten, Randwand (3.1)
- Gänge mit Sackgassen und Schleifen, Foyers = offene Räume (3.3.4)
- freistehende Wände, dadurch Floating Tiles (3.3.1)
- Start beliebig im Feld (3.3.5)
- schwarze, blaue, silberne Kacheln (3.2); schwarze nur so, dass alles andere erreichbar bleibt (5.3.7)
- Gefahrenzone: komplett ummauert, rote Eingangskachel, blockiert nichts, Start nicht darin (3.5)

Gleiche Einstellungen + gleicher Seed ergeben immer dasselbe Labyrinth.
"""
import random
from collections import deque
from dataclasses import dataclass

from navigation.maze_map import Dir, Pos, step

Edge = tuple[Pos, Pos]


@dataclass
class _Zone:
    tiles: set[Pos]
    entrance: Pos   # rote Kachel (in der Zone)
    outside: Pos    # Nachbarkachel außerhalb, über die man hineinkommt


class _Grid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.all = {(x, y) for x in range(width) for y in range(height)}
        self.walls: set[Edge] = set()
        for p in self.all:
            for q in self.neighbors(p):
                self.walls.add(_edge(p, q))

    def neighbors(self, p: Pos) -> list[Pos]:
        return [q for q in (step(p, d) for d in Dir) if q in self.all]

    def open(self, a: Pos, b: Pos):
        self.walls.discard(_edge(a, b))

    def is_open(self, a: Pos, b: Pos) -> bool:
        return _edge(a, b) not in self.walls


def _edge(a: Pos, b: Pos) -> Edge:
    return (a, b) if a < b else (b, a)


def generate(width: int, height: int, black: int = 2, blue: int = 2, silver: int = 3,
             zones: int = 1, seed: int | None = None) -> tuple[dict, list[str]]:
    """Erzeugt ein Labyrinth im JSON-Format von simulation/mazes/. Gibt (Daten, Warnungen) zurück."""
    if seed is None:
        seed = random.randrange(1_000_000)
    rng = random.Random(seed)
    grid = _Grid(width, height)
    warnings: list[str] = []

    zone_list = _place_zones(grid, zones, rng, warnings)
    zone_tiles = set().union(*(z.tiles for z in zone_list))
    main = grid.all - zone_tiles

    _carve_maze(grid, main, rng)
    for zone in zone_list:
        _carve_maze(grid, zone.tiles, rng)
        grid.open(zone.entrance, zone.outside)
    _add_loops(grid, main, rng)
    foyers = _carve_foyers(grid, main, rng)
    _add_island(grid, foyers, rng)

    start = rng.choice(sorted(main))
    start_dir = rng.choice(list(Dir))
    floors = {p: "normal" for p in grid.all}
    for zone in zone_list:
        floors[zone.entrance] = "red"

    # Sonderkacheln nur außerhalb der Gefahrenzone und nicht auf dem Start
    candidates = sorted(main - {start})
    rng.shuffle(candidates)
    placed = _place_black(grid, floors, candidates, black, start)
    if placed < black:
        warnings.append(f"Nur {placed} von {black} schwarzen Kacheln passen, ohne Wege zu blockieren")
    free = [p for p in candidates if floors[p] == "normal"]
    for kind, count in (("silver", silver), ("blue", blue)):
        if count > len(free):
            warnings.append(f"Zu wenig Platz: nur {len(free)} statt {count} Kacheln '{kind}'")
        for p in free[:count]:
            floors[p] = kind
        free = free[count:]

    tiles = []
    for (x, y) in sorted(grid.all, key=lambda p: (-p[1], p[0])):
        walls = "".join(d.name for d in Dir
                        if step((x, y), d) not in grid.all
                        or not grid.is_open((x, y), step((x, y), d)))
        tiles.append({"x": x, "y": y, "walls": walls, "floor": floors[(x, y)]})
    data = {
        "width": width, "height": height,
        "start": list(start), "start_dir": start_dir.name,
        "seed": seed,
        "params": {"black": black, "blue": blue, "silver": silver, "zones": zones},
        "tiles": tiles,
    }
    return data, warnings


def _place_zones(grid: _Grid, count: int, rng: random.Random, warnings: list[str]) -> list[_Zone]:
    zones: list[_Zone] = []
    taken: set[Pos] = set()
    for _ in range(count):
        for _attempt in range(200):
            zw, zh = rng.choice((2, 3)), rng.choice((2, 3))
            if zw >= grid.width or zh >= grid.height:
                continue
            x0, y0 = rng.randrange(grid.width - zw + 1), rng.randrange(grid.height - zh + 1)
            rect = {(x, y) for x in range(x0, x0 + zw) for y in range(y0, y0 + zh)}
            near = rect | {q for p in rect for q in grid.neighbors(p)}
            if near & taken:
                continue   # Zonen sollen sich nicht berühren
            main = grid.all - taken - rect
            if len(main) < max(6, len(grid.all) // 2) or not _connected(grid, main):
                continue   # Rest des Feldes muss groß genug und zusammenhängend bleiben
            doors = [(p, q) for p in sorted(rect) for q in grid.neighbors(p) if q in main]
            entrance, outside = rng.choice(doors)
            zones.append(_Zone(rect, entrance, outside))
            taken |= rect
            break
        else:
            warnings.append("Gefahrenzone passt nicht ins Feld")
            break
    return zones


def _connected(grid: _Grid, area: set[Pos]) -> bool:
    if not area:
        return False
    first = next(iter(area))
    seen, todo = {first}, [first]
    while todo:
        for q in grid.neighbors(todo.pop()):
            if q in area and q not in seen:
                seen.add(q)
                todo.append(q)
    return seen == area


def _carve_maze(grid: _Grid, area: set[Pos], rng: random.Random):
    """Gänge: zufällige Tiefensuche, jede Kachel wird genau einmal angebunden (keine Schleifen)."""
    start = rng.choice(sorted(area))
    seen, stack = {start}, [start]
    while stack:
        p = stack[-1]
        options = [q for q in grid.neighbors(p) if q in area and q not in seen]
        if not options:
            stack.pop()
            continue
        q = rng.choice(options)
        grid.open(p, q)
        seen.add(q)
        stack.append(q)


def _add_loops(grid: _Grid, area: set[Pos], rng: random.Random, chance: float = 0.12):
    """Einzelne Wände entfernen: es entstehen Schleifen und freistehende Wände."""
    for a, b in sorted(grid.walls):
        if a in area and b in area and rng.random() < chance:
            grid.open(a, b)


def _carve_foyers(grid: _Grid, area: set[Pos], rng: random.Random) -> list[tuple[int, int, int, int]]:
    """Offene Räume: alle Innenwände eines Rechtecks entfernen. Erster Raum möglichst 3x3."""
    count = len(area) // 28
    foyers = []
    for i in range(count):
        sizes = [(3, 3)] if i == 0 else [(2, 2), (3, 2), (2, 3)]
        for _attempt in range(100):
            w, h = rng.choice(sizes)
            x0, y0 = rng.randrange(grid.width - w + 1), rng.randrange(grid.height - h + 1)
            rect = {(x, y) for x in range(x0, x0 + w) for y in range(y0, y0 + h)}
            if not rect <= area or any(_overlaps(rect, f) for f in foyers):
                continue
            for p in rect:
                for q in grid.neighbors(p):
                    if q in rect:
                        grid.open(p, q)
            foyers.append((x0, y0, w, h))
            break
    return foyers


def _overlaps(rect: set[Pos], foyer: tuple[int, int, int, int]) -> bool:
    x0, y0, w, h = foyer
    return any(x0 <= x < x0 + w and y0 <= y < y0 + h for x, y in rect)


def _add_island(grid: _Grid, foyers: list[tuple[int, int, int, int]], rng: random.Random):
    """In einem 3x3-Raum 1-2 Wände an der Mittelkachel aufstellen. Ihre Enden berühren
    keine andere Wand, sie stehen also frei (Insel)."""
    for x0, y0, w, h in foyers:
        if w >= 3 and h >= 3:
            center = (x0 + 1, y0 + 1)
            for d in rng.sample(list(Dir), rng.choice((1, 2))):
                grid.walls.add(_edge(center, step(center, d)))
            return


def _place_black(grid: _Grid, floors: dict[Pos, str], candidates: list[Pos],
                 count: int, start: Pos) -> int:
    """Schwarze Kacheln nur dort, wo danach alle anderen Kacheln noch erreichbar sind."""
    placed = 0
    for p in candidates:
        if placed == count:
            break
        if floors[p] != "normal":
            continue
        floors[p] = "black"
        if _all_reachable(grid, floors, start):
            placed += 1
        else:
            floors[p] = "normal"
    return placed


def _all_reachable(grid: _Grid, floors: dict[Pos, str], start: Pos) -> bool:
    seen, todo = {start}, deque([start])
    while todo:
        p = todo.popleft()
        for q in grid.neighbors(p):
            if q not in seen and floors[q] != "black" and grid.is_open(p, q):
                seen.add(q)
                todo.append(q)
    return len(seen) == sum(1 for f in floors.values() if f != "black")
