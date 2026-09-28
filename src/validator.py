from parser import Connection, Map, ZoneType


# A move that is forbidden at one turn: (from_zone, to_zone, turn), where
# turn is the turn in which the move would be printed.
Cut = tuple[str, str, int]


class Violation:
    """First rule broken by a schedule.

    For link violations, `crossings` counts the drones that started
    crossing `connection` at `turn`, per direction (from, to).
    """

    def __init__(
        self,
        message: str,
        turn: int,
        connection: str | None = None,
        crossings: dict[tuple[str, str], int] | None = None,
    ) -> None:
        self.message: str = message
        self.turn: int = turn
        self.connection: str | None = connection
        self.crossings: dict[tuple[str, str], int] = crossings or {}

    def cut(self) -> Cut | None:
        """Move to forbid so the solver avoids this violation.

        Only a link used in both directions can be repaired this way:
        forbid the direction carrying fewer drones. Anything else means
        the model itself is wrong, so there is no cut.
        """
        if len(self.crossings) < 2:
            return None
        u, v = min(self.crossings, key=lambda d: (self.crossings[d], d))
        return (u, v, self.turn)

    def __str__(self) -> str:
        return f"turn {self.turn}: {self.message}"


class Validator:
    """Replays printed output turn by turn and checks every rule.

    It knows nothing about the flow network, so it catches modelling
    mistakes as well as bugs. Link capacity counts every drone that
    starts crossing a connection in a turn, both directions together.
    A drone finishing a transit frees the connection for the same
    turn, like drones leaving a zone.
    """

    def __init__(self, mp: Map) -> None:
        self.map: Map = mp
        self.conns: dict[str, Connection] = {
            f"{c.h1}-{c.h2}": c for c in mp.conns
        }
        self.conn_between: dict[frozenset[str], str] = {
            frozenset((c.h1, c.h2)): name for name, c in self.conns.items()
        }

    def validate(self, lines: list[str]) -> Violation | None:
        """Return the first violation in `lines`, or None if valid."""
        mp = self.map
        hubs = mp.hubs
        n = mp.nb_drones
        # A zone name, or (connection, target zone) while in flight.
        pos: dict[int, str | tuple[str, str]] = {
            d: mp.start_hub for d in range(1, n + 1)
        }
        for turn, line in enumerate(lines, start=1):
            moved: set[int] = set()
            crossings: dict[str, dict[tuple[str, str], int]] = {}
            for tok in line.split():
                try:
                    did_s, dest = tok[1:].split("-", 1)
                    d = int(did_s)
                except ValueError:
                    return Violation(f"malformed move {tok}", turn)
                if not tok.startswith("D") or not 1 <= d <= n:
                    return Violation(f"unknown drone in {tok}", turn)
                if d in moved:
                    return Violation(f"D{d} moves twice", turn)
                moved.add(d)
                cur = pos[d]
                if cur == mp.end_hub:
                    return Violation(f"D{d} moves after delivery", turn)
                if isinstance(cur, tuple):
                    if dest != cur[1]:
                        return Violation(f"{tok} leaves transit", turn)
                    pos[d] = dest
                    continue
                if dest in self.conns:
                    c = self.conns[dest]
                    if cur not in (c.h1, c.h2):
                        return Violation(f"{tok} not from {cur}", turn)
                    target = c.h2 if cur == c.h1 else c.h1
                    if hubs[target].metadata.zone != ZoneType.RESTRICTED:
                        return Violation(f"{tok} toward non-restricted",
                                         turn)
                    pos[d] = (dest, target)
                else:
                    if dest not in hubs:
                        return Violation(f"unknown zone in {tok}", turn)
                    zone = hubs[dest].metadata.zone
                    if zone in (ZoneType.BLOCKED, ZoneType.RESTRICTED):
                        return Violation(f"{tok} enters {zone.value} zone",
                                         turn)
                    if dest not in hubs[cur].conns:
                        return Violation(f"{tok}: no link from {cur}", turn)
                    target = dest
                    pos[d] = dest
                name = self.conn_between[frozenset((cur, target))]
                per_dir = crossings.setdefault(name, {})
                per_dir[(cur, target)] = per_dir.get((cur, target), 0) + 1

            for d, p in pos.items():
                if isinstance(p, tuple) and d not in moved:
                    return Violation(f"D{d} waits on {p[0]}", turn)
            for name, per_dir in crossings.items():
                used = sum(per_dir.values())
                cap = self.conns[name].max_link_cap
                if used > cap:
                    return Violation(f"link {name} used by {used} > {cap}",
                                     turn, name, per_dir)
            occupancy: dict[str, int] = {}
            for p in pos.values():
                if isinstance(p, str):
                    occupancy[p] = occupancy.get(p, 0) + 1
            for zone_name, count in occupancy.items():
                if zone_name in (mp.start_hub, mp.end_hub):
                    continue
                cap = hubs[zone_name].metadata.max_drones
                if count > cap:
                    return Violation(f"{zone_name} holds {count} > {cap}",
                                     turn)
        if any(p != mp.end_hub for p in pos.values()):
            return Violation("drones not delivered", len(lines))
        return None
