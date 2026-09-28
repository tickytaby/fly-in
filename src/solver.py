from flow_network import Edge
from parser import Map
from time_expanded import TimeExpandedGraph
from validator import Cut, Validator


class Route:
    """Where one drone is at every turn, from turn 0 to its arrival.

    positions[t] is a zone name, or a connection name while the drone
    is in flight toward a restricted zone.
    """

    def __init__(self, positions: list[str]) -> None:
        self.positions: list[str] = positions

    @property
    def arrival(self) -> int:
        """Turn at which the drone reaches the end zone."""
        return len(self.positions) - 1

    @property
    def departure(self) -> int:
        """First turn at which the drone leaves the start zone."""
        for t in range(1, len(self.positions)):
            if self.positions[t] != self.positions[0]:
                return t
        raise RuntimeError("route never leaves the start zone")


class SSPSolver:
    """Successive Shortest Paths on a growing time-expanded network."""

    # Safety net for the validate -> cut -> re-solve loop in `solve`.
    MAX_CUTS = 50

    def __init__(
        self, mp: Map, verbose: bool = False, cuts: set[Cut] | None = None
    ) -> None:
        self.map: Map = mp
        self.verbose: bool = verbose
        self.graph: TimeExpandedGraph = TimeExpandedGraph(mp, cuts)
        self.scheduled: int = 0
        # Number of augmenting paths that cancelled earlier flow.
        self.cancellations: int = 0
        self.routes: list[Route] = []

    @classmethod
    def solve(cls, mp: Map, verbose: bool = False) -> "SSPSolver":
        """Solve `mp` and return a solver whose schedule is validated.

        The flow model cannot stop two drones from crossing a link in
        opposite directions when one of them enters a restricted zone.
        If the validator finds such a crossing, that move is cut from
        the graph and everything is solved again.

        Raises:
            Exception: if the end is unreachable or no valid schedule
                could be produced.
        """
        validator = Validator(mp)
        cuts: set[Cut] = set()
        while True:
            solver = cls(mp, verbose, cuts)
            solver.run()
            violation = validator.validate(solver.output_lines())
            if violation is None:
                return solver
            cut = violation.cut()
            if cut is None or cut in cuts or len(cuts) >= cls.MAX_CUTS:
                raise Exception(f"Error: invalid schedule, {violation}")
            if verbose:
                print(f"[ssp] {violation} -> cutting {cut}, re-solving")
            cuts.add(cut)

    def _lower_bound(self) -> int:
        """Earliest turn a single drone can reach the end (Dijkstra)."""
        try:
            _, _, cost = self.map.run_dijkstra()
        except KeyError:
            raise Exception(f"Error: end zone '{self.map.end_hub}' is unreachable")
        return cost

    def _augment_once(self) -> bool:
        """Find and push one cheapest augmenting path.

        Returns:
            False if the sink is unreachable in the current network.
        """
        g = self.graph
        dist, via = g.spfa(g.source)
        if dist[g.sink] == float("inf"):
            return False
        path = g.path_to(via, g.sink)
        backward = [e for e in path if not e.original]
        if backward:
            self.cancellations += 1
            if self.verbose:
                undone = ", ".join(
                    f"{g.describe(e.to)}->{g.describe(e.tail)}" for e in backward
                )
                print(f"[ssp] drone {self.scheduled + 1} cancels: {undone}")
        g.augment(path)
        return True

    def run(self) -> list[Route]:
        """Schedule every drone and return one route per drone.

        Raises:
            Exception: if the end zone cannot be reached.
        """
        g = self.graph
        g.build(self._lower_bound())
        limit = 2 * len(g.zones)
        while self.scheduled < self.map.nb_drones:
            if self._augment_once():
                self.scheduled += 1
                continue
            if self.scheduled == 0 and g.horizon > limit:
                raise Exception(f"Error: end zone '{self.map.end_hub}' is unreachable")
            g.add_layer(g.horizon + 1)
            if self.verbose:
                print(f"[ssp] horizon grown to {g.horizon}")
        self.routes = self._decompose()
        return self.routes

    def _decompose(self) -> list[Route]:
        """Split the final flow into one route per drone."""
        g = self.graph
        remaining: dict[int, int] = {
            e.idx: e.flow for e in g.edges if e.original and e.flow > 0
        }
        routes: list[Route] = []
        for _ in range(self.scheduled):
            positions: list[str] = []
            node = g.source
            while node != g.sink:
                self._record(node, positions)
                edge = self._next_edge(node, remaining)
                remaining[edge.idx] -= 1
                node = edge.to
            routes.append(Route(positions))
        routes.sort(key=lambda r: (r.arrival, r.departure))
        return routes

    def _next_edge(self, node: int, remaining: dict[int, int]) -> Edge:
        for e in self.graph.adj[node]:
            if e.original and remaining.get(e.idx, 0) > 0:
                return e
        raise RuntimeError(f"flow decomposition stuck at {self.graph.describe(node)}")

    def _record(self, node: int, positions: list[str]) -> None:
        """Append the position a node stands for, if it opens a turn."""
        key = self.graph.keys[node]
        if key[0] == "in":
            positions.append(str(key[1]))
        elif key[0] == "transit":
            positions.append(self.graph.conn_name[(str(key[1]), str(key[2]))])

    @property
    def makespan(self) -> int:
        """Latest arrival turn over all drones."""
        return max((r.arrival for r in self.routes), default=0)

    def output_lines(self) -> list[str]:
        """One line per turn listing every drone movement of that turn."""
        lines: list[str] = []
        for t in range(1, self.makespan + 1):
            moves: list[str] = []
            for i, route in enumerate(self.routes, start=1):
                pos = route.positions
                if t < len(pos) and pos[t] != pos[t - 1]:
                    moves.append(f"D{i}-{pos[t]}")
            lines.append(" ".join(moves))
        return lines
