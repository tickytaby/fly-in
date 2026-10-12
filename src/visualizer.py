import zlib

from parser import Map, ZoneType
from solver import Route


class ColorPalette:
    """Turns free-form map color names into 256-color ANSI codes.

    Known names use a fixed table. Any other name gets a stable color
    picked from its CRC32, so the same name always looks the same.
    """

    NAMED: dict[str, int] = {
        "red": 196, "darkred": 88, "maroon": 124, "crimson": 161,
        "orange": 208, "gold": 220, "yellow": 226, "lime": 118,
        "green": 34, "darkgreen": 22, "olive": 100, "teal": 30,
        "cyan": 51, "turquoise": 44, "blue": 33, "navy": 25,
        "indigo": 54, "purple": 129, "violet": 177, "magenta": 201,
        "pink": 218, "salmon": 209, "coral": 203, "brown": 130,
        "tan": 180, "beige": 230, "khaki": 186, "lavender": 183,
        "white": 255, "silver": 250, "gray": 245, "grey": 245,
        # Pure black is invisible on dark terminals.
        "black": 242,
    }
    FALLBACK: tuple[int, ...] = (
        196, 208, 226, 118, 51, 33, 129, 201, 214, 159, 177, 48,
    )
    RAINBOW: tuple[int, ...] = (196, 208, 226, 46, 51, 33, 129)

    def __init__(self, enabled: bool) -> None:
        self.enabled: bool = enabled

    def code(self, color: str | None) -> int | None:
        """256-color code for a color name, None for no color."""
        if not color:
            return None
        key = color.lower()
        if key in self.NAMED:
            return self.NAMED[key]
        return self.FALLBACK[zlib.crc32(key.encode()) % len(self.FALLBACK)]

    def paint(
        self,
        text: str,
        color: str | None = None,
        bold: bool = False,
        dim: bool = False,
    ) -> str:
        """Wrap `text` in escape codes (unchanged when disabled)."""
        if not self.enabled or not text:
            return text
        style = (["1"] if bold else []) + (["2"] if dim else [])
        if color and color.lower() == "rainbow":
            return "".join(
                self._wrap(ch, style + [f"38;5;{self.RAINBOW[i % 7]}"])
                for i, ch in enumerate(text)
            )
        c = self.code(color)
        return self._wrap(text, style + ([f"38;5;{c}"] if c is not None
                                         else []))

    @staticmethod
    def _wrap(text: str, codes: list[str]) -> str:
        if not codes:
            return text
        return f"\033[{';'.join(codes)}m{text}\033[0m"


class Cell:
    """A piece of output text, kept with its visible width."""

    def __init__(self, plain: str, painted: str) -> None:
        self.plain: str = plain
        self.painted: str = painted


class TerminalVisualizer:
    """Colored per-turn view of a solved schedule.

    Every turn shows the required move line (colored by destination
    zone), then the state of every zone: occupancy against max_drones,
    drones in flight toward restricted zones, and delivery progress.
    """

    FULL = "●"
    EMPTY = "○"
    # Zones holding more than this show "n/cap" instead of a bar.
    BAR_LIMIT = 8

    def __init__(
        self,
        mp: Map,
        routes: list[Route],
        palette: ColorPalette,
        width: int = 100,
    ) -> None:
        self.map: Map = mp
        self.routes: list[Route] = routes
        self.palette: ColorPalette = palette
        self.width: int = width
        self.makespan: int = max((r.arrival for r in routes), default=0)
        self.zones: list[str] = [
            name for name in mp.hubs
            if name not in (mp.start_hub, mp.end_hub)
        ]

    def _color(self, zone: str) -> str | None:
        return self.map.hubs[zone].metadata.color

    def _position(self, route: Route, t: int) -> str:
        """Where a drone is at turn t (the end zone once delivered)."""
        pos = route.positions
        return pos[t] if t < len(pos) else self.map.end_hub

    def _moves(self, t: int) -> list[tuple[int, str, str]]:
        """(drone id, printed destination, zone it heads to) at turn t."""
        moves: list[tuple[int, str, str]] = []
        for i, route in enumerate(self.routes, start=1):
            pos = route.positions
            if t < len(pos) and pos[t] != pos[t - 1]:
                # On a connection, color by the restricted zone ahead.
                target = pos[t] if pos[t] in self.map.hubs else pos[t + 1]
                moves.append((i, pos[t], target))
        return moves

    def render_moves(self, t: int) -> str:
        """The required output line for turn t, colored per zone."""
        return " ".join(
            f"D{i}-" + self.palette.paint(label, self._color(target),
                                          bold=label not in self.map.hubs)
            for i, label, target in self._moves(t)
        )

    def _zone_cell(self, zone: str, drones: int) -> Cell:
        meta = self.map.hubs[zone].metadata
        p = self.palette
        mark = {ZoneType.PRIORITY: "★", ZoneType.RESTRICTED: "!",
                ZoneType.BLOCKED: "✗"}.get(meta.zone, "")
        blocked = meta.zone == ZoneType.BLOCKED
        name = p.paint(zone + mark, meta.color, bold=mark == "!",
                       dim=blocked)
        if blocked:
            return Cell(zone + mark, name)
        cap = meta.max_drones
        if cap > self.BAR_LIMIT:
            bar = f"{drones}/{cap}"
            return Cell(f"{zone}{mark} {bar}", f"{name} {bar}")
        full = self.FULL * drones
        empty = self.EMPTY * (cap - drones)
        return Cell(
            f"{zone}{mark} {full}{empty}",
            f"{name} {p.paint(full, meta.color)}{p.paint(empty, dim=True)}",
        )

    def _grid(self, cells: list[Cell], indent: str = "  ") -> list[str]:
        """Lay cells out in aligned columns that fit the width."""
        if not cells:
            return []
        col = max(len(c.plain) for c in cells) + 2
        per_row = max(1, (self.width - len(indent)) // col)
        rows: list[str] = []
        for r in range(0, len(cells), per_row):
            row = cells[r:r + per_row]
            rows.append(indent + "".join(
                c.painted + " " * (col - len(c.plain)) for c in row
            ).rstrip())
        return rows

    def render_state(self, t: int) -> list[str]:
        """Zone occupancy, drones in flight and progress at turn t."""
        mp = self.map
        p = self.palette
        occupancy: dict[str, int] = {}
        flying: list[str] = []
        for i, route in enumerate(self.routes, start=1):
            here = self._position(route, t)
            if here in mp.hubs:
                occupancy[here] = occupancy.get(here, 0) + 1
            else:
                target = route.positions[t + 1]
                flying.append(f"D{i} {here} → "
                              + p.paint(target, self._color(target)))
        n = mp.nb_drones
        start = p.paint(mp.start_hub, self._color(mp.start_hub))
        end = p.paint(mp.end_hub, self._color(mp.end_hub))
        lines = [
            f"  {start}: {occupancy.get(mp.start_hub, 0)} waiting    "
            f"{end}: {occupancy.get(mp.end_hub, 0)}/{n} delivered"
        ]
        lines += self._grid([self._zone_cell(z, occupancy.get(z, 0))
                             for z in self.zones])
        if flying:
            lines.append("  in flight: " + ", ".join(flying))
        return lines

    def render_turn(self, t: int) -> list[str]:
        """Header, move line and state block for turn t."""
        header = self.palette.paint(f"── Turn {t} ".ljust(40, "─"), dim=True)
        return [header, self.render_moves(t)] + self.render_state(t)

    def render_summary(self) -> list[str]:
        """Makespan plus the subject's optional secondary metrics."""
        n = len(self.routes)
        moves_per_turn = [len(self._moves(t))
                          for t in range(1, self.makespan + 1)]
        # One turn per zone change: restricted moves count 2.
        move_turns = sum(moves_per_turn)
        avg_arrival = sum(r.arrival for r in self.routes) / n if n else 0
        avg_moving = move_turns / self.makespan if self.makespan else 0
        bold = self.palette.paint(f"{self.makespan} turns", bold=True)
        return [
            self.palette.paint("── Summary ".ljust(40, "─"), dim=True),
            f"  makespan:               {bold} for {n} drones",
            f"  average turns / drone:  {avg_arrival:.2f}",
            f"  total movement turns:   {move_turns}",
            f"  drones moving per turn: {avg_moving:.2f} average, "
            f"{max(moves_per_turn, default=0)} max",
        ]

    def render(self) -> list[str]:
        """Full visual output: every turn, then the summary."""
        lines: list[str] = []
        for t in range(1, self.makespan + 1):
            lines += self.render_turn(t)
        return lines + self.render_summary()
