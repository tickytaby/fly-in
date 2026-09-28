from flow_network import FlowNetwork
from parser import Map, ZoneType
from validator import Cut


# Arrival at turn t costs t * BIG. BIG must stay fixed once edges exist
# and only needs to exceed the movement cost of any route (~2 per turn).
BIG = 10**6

WAIT_COST = 0
MOVE_COST = 2
PRIORITY_COST = 1
# Cost of each of the two legs of a move into a restricted zone.
RESTRICTED_LEG_COST = 2


class TimeExpandedGraph(FlowNetwork):
    """Time-expanded flow network of a map, grown one layer at a time.

    Node keys:
        ("in", zone, t) / ("out", zone, t): zone copy at turn t, split so
            the in -> out edge carries the zone capacity. The end zone
            only has an "in" node, which drains into the sink.
        ("transit", u, v, t): drone sitting on connection u-v at turn t
            on its way to restricted zone v. No wait edge: it must
            arrive at turn t + 1.
        ("sink",): super sink collecting every end-zone arrival.

    `cuts` lists moves (from, to, turn) that must not be created, turn
    being the turn the move is printed in (see Violation.cut).
    """

    def __init__(self, mp: Map, cuts: set[Cut] | None = None) -> None:
        super().__init__()
        self.map: Map = mp
        self.cuts: set[Cut] = cuts or set()
        self.start: str = mp.start_hub
        self.end: str = mp.end_hub
        # "Unlimited" capacity: flow never exceeds the number of drones.
        self.unlimited: int = mp.nb_drones
        self.zones: list[str] = [
            name
            for name, hub in mp.hubs.items()
            if hub.metadata.zone != ZoneType.BLOCKED or name == self.start
        ]
        usable = set(self.zones)
        # Directed moves (u, v, link capacity), both ways per connection.
        self.moves: list[tuple[str, str, int]] = []
        self.conn_name: dict[tuple[str, str], str] = {}
        for c in mp.conns:
            if c.h1 not in usable or c.h2 not in usable:
                continue
            name = f"{c.h1}-{c.h2}"
            for u, v in ((c.h1, c.h2), (c.h2, c.h1)):
                self.moves.append((u, v, c.max_link_cap))
                self.conn_name[(u, v)] = name
        self.horizon: int = -1
        self.sink: int = self.node(("sink",))
        self.add_layer(0)
        self.source: int = self.ids[("in", self.start, 0)]

    def _is_restricted(self, zone: str) -> bool:
        return self.map.hubs[zone].metadata.zone == ZoneType.RESTRICTED

    def _move_cost(self, zone: str) -> int:
        if self.map.hubs[zone].metadata.zone == ZoneType.PRIORITY:
            return PRIORITY_COST
        return MOVE_COST

    def _capacity(self, zone: str) -> int:
        if zone == self.start:
            return self.unlimited
        return self.map.hubs[zone].metadata.max_drones

    def _entry(self, zone: str, t: int) -> int:
        """Node a drone enters when it reaches `zone` at turn t."""
        return self.node(("in", zone, t))

    def _exit(self, zone: str, t: int) -> int | None:
        """Node a drone leaves `zone` from at turn t, if it exists."""
        return self.ids.get(("out", zone, t))

    def add_layer(self, t: int) -> None:
        """Add every node and edge whose head lies in layer t.

        Layers must be added in order (t == horizon + 1).
        """
        if t != self.horizon + 1:
            raise ValueError(f"layer {t} added out of order")
        self.horizon = t

        # Zone copies at turn t.
        layer_zones = [self.start] if t == 0 else self.zones
        for z in layer_zones:
            zin = self._entry(z, t)
            if z == self.end:
                self.add_edge(zin, self.sink, self.unlimited, t * BIG)
            else:
                zout = self.node(("out", z, t))
                self.add_edge(zin, zout, self._capacity(z), 0)
        if t == 0:
            return

        # Wait edges from layer t - 1.
        for z in self.zones:
            prev = self._exit(z, t - 1)
            if prev is not None:
                self.add_edge(prev, self._entry(z, t), self.unlimited, WAIT_COST)

        for u, v, link_cap in self.moves:
            if not self._is_restricted(v):
                # Normal / priority move: leave u at t-1, reach v at t.
                if (u, v, t) in self.cuts:
                    continue
                prev = self._exit(u, t - 1)
                if prev is not None:
                    self.add_edge(prev, self._entry(v, t), link_cap, self._move_cost(v))
            elif t >= 2:
                # Restricted move: leave u at t-2, sit on the
                # connection at t-1, reach v at t.
                if (u, v, t - 1) in self.cuts:
                    continue
                prev = self._exit(u, t - 2)
                if prev is not None:
                    transit = self.node(("transit", u, v, t - 1))
                    self.add_edge(prev, transit, link_cap, RESTRICTED_LEG_COST)
                    self.add_edge(
                        transit, self._entry(v, t), link_cap, RESTRICTED_LEG_COST
                    )

    def build(self, horizon: int) -> None:
        """Add layers until the horizon reaches `horizon`."""
        while self.horizon < horizon:
            self.add_layer(self.horizon + 1)
