"""Wegplanung: Dijkstra über (Kachel, Blickrichtung).

Ergebnis ist eine Liste von Aktionen: "F" (vorwärts), "L" (links drehen), "R" (rechts drehen).
"""
import heapq
import itertools

import config
from navigation.maze_map import Dir, MazeMap, Pos, TileType, step

State = tuple[Pos, Dir]


def plan_path(maze: MazeMap, pos: Pos, heading: Dir, targets: set[Pos]) -> list[str] | None:
    """Günstigster Weg zu einer der Zielkacheln, oder None wenn keine erreichbar ist.

    Gefahren wird nur über besuchte Kacheln. Eine unbesuchte Kachel darf nur
    das Ziel selbst sein (Grenzkachel).
    """
    start: State = (pos, heading)
    best = {start: 0.0}
    came_from: dict[State, tuple[State, str]] = {}
    counter = itertools.count()   # Gleichstand: zuerst gefundene Zustände zuerst
    heap = [(0.0, next(counter), start)]

    while heap:
        cost, _, state = heapq.heappop(heap)
        if cost > best[state]:
            continue
        p, h = state
        if p in targets:
            return _actions(came_from, state)
        for action, nxt, step_cost in _moves(maze, p, h, targets):
            new_cost = cost + step_cost
            if new_cost < best.get(nxt, float("inf")):
                best[nxt] = new_cost
                came_from[nxt] = (state, action)
                heapq.heappush(heap, (new_cost, next(counter), nxt))
    return None


def _moves(maze: MazeMap, pos: Pos, heading: Dir, targets: set[Pos]):
    if maze.is_passable(pos, heading):
        nxt = step(pos, heading)
        if nxt in maze.visited or nxt in targets:
            cost = config.PLAN_COST_FORWARD
            if maze.kind(nxt) is TileType.BLUE:   # blau ist nur bekannt, wenn schon besucht
                cost += config.PLAN_COST_BLUE_REVISIT
            yield "F", (nxt, heading), cost
    yield "L", (pos, heading.left()), config.PLAN_COST_TURN
    yield "R", (pos, heading.right()), config.PLAN_COST_TURN


def _actions(came_from: dict[State, tuple[State, str]], state: State) -> list[str]:
    actions = []
    while state in came_from:
        state, action = came_from[state]
        actions.append(action)
    actions.reverse()
    return actions


def follow(pos: Pos, heading: Dir, actions: list[str]) -> list[State]:
    """Alle Zustände, die beim Abfahren der Aktionen durchlaufen werden (inkl. Start)."""
    states = [(pos, heading)]
    for a in actions:
        if a == "F":
            pos = step(pos, heading)
        elif a == "L":
            heading = heading.left()
        else:
            heading = heading.right()
        states.append((pos, heading))
    return states


def estimate_time(maze: MazeMap, pos: Pos, heading: Dir, actions: list[str]) -> float:
    """Geschätzte Fahrzeit in Sekunden für eine Aktionsliste."""
    total = 0.0
    for a, (p, _) in zip(actions, follow(pos, heading, actions)[1:]):
        if a == "F":
            total += config.MOVE_TIME_S
            if maze.kind(p) is TileType.BLUE:
                total += config.BLUE_WAIT_S
        else:
            total += config.TURN_TIME_S
    return total
