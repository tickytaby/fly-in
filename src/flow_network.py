from collections import deque


NodeKey = tuple[str | int, ...]


class Edge:
    """One arc of the residual network.

    Every edge is created together with a partner running the opposite
    way. The original (forward) edge starts with the real capacity and
    cost, the partner (backward) edge starts with capacity 0 and the
    negated cost. Pushing one unit moves one unit of capacity from an
    edge to its partner.
    """

    __slots__ = ("idx", "to", "cap", "cost", "partner", "original")

    def __init__(self, idx: int, to: int, cap: int, cost: int, original: bool) -> None:
        self.idx: int = idx
        self.to: int = to
        self.cap: int = cap
        self.cost: int = cost
        self.partner: "Edge" = self
        self.original: bool = original

    @property
    def tail(self) -> int:
        """Node this edge leaves from (the head of its partner)."""
        return self.partner.to

    @property
    def flow(self) -> int:
        """Units currently pushed through an original edge."""
        return self.partner.cap if self.original else 0


class FlowNetwork:
    """Integer-node residual network with readable node keys.

    Nodes are ints. `ids` maps a readable key such as ("in", zone, t)
    to its id, and `keys` maps ids back to keys so any node can be
    printed.
    """

    def __init__(self) -> None:
        self.ids: dict[NodeKey, int] = {}
        self.keys: list[NodeKey] = []
        self.adj: list[list[Edge]] = []
        self.edges: list[Edge] = []

    def node(self, key: NodeKey) -> int:
        """Return the id of `key`, creating the node if needed."""
        nid = self.ids.get(key)
        if nid is None:
            nid = len(self.keys)
            self.ids[key] = nid
            self.keys.append(key)
            self.adj.append([])
        return nid

    def add_edge(self, u: int, v: int, cap: int, cost: int) -> Edge:
        """Add edge u -> v and its residual partner v -> u.

        Returns:
            The original (forward) edge.
        """
        fwd = Edge(len(self.edges), v, cap, cost, True)
        bwd = Edge(len(self.edges) + 1, u, 0, -cost, False)
        fwd.partner = bwd
        bwd.partner = fwd
        self.edges.append(fwd)
        self.edges.append(bwd)
        self.adj[u].append(fwd)
        self.adj[v].append(bwd)
        return fwd

    def describe(self, nid: int) -> str:
        """Readable name of a node id."""
        return ":".join(str(k) for k in self.keys[nid])

    def dump(self) -> str:
        """All original edges, one per line, for debugging."""
        lines: list[str] = []
        for e in self.edges:
            if e.original:
                lines.append(
                    f"{self.describe(e.tail)} -> {self.describe(e.to)}"
                    f" cap={e.cap} cost={e.cost} flow={e.flow}"
                )
        return "\n".join(lines)

    def spfa(self, source: int) -> tuple[list[float], list[Edge | None]]:
        """Shortest paths from `source` over edges with capacity > 0.

        Queue-based Bellman-Ford, safe with the negative costs of
        backward edges (as long as there is no negative cycle, which
        SSP guarantees).

        Returns:
            (dist, via) where dist[v] is the cheapest cost to reach v
            (inf if unreachable) and via[v] is the edge used to reach v.
        """
        n = len(self.keys)
        inf = float("inf")
        dist: list[float] = [inf] * n
        via: list[Edge | None] = [None] * n
        queued: list[bool] = [False] * n
        dist[source] = 0
        queue: deque[int] = deque([source])
        queued[source] = True
        while queue:
            u = queue.popleft()
            queued[u] = False
            du = dist[u]
            for e in self.adj[u]:
                if e.cap <= 0:
                    continue
                nd = du + e.cost
                if nd < dist[e.to]:
                    dist[e.to] = nd
                    via[e.to] = e
                    if not queued[e.to]:
                        queued[e.to] = True
                        queue.append(e.to)
        return dist, via

    def path_to(self, via: list[Edge | None], target: int) -> list[Edge]:
        """Edges from the SPFA source to `target`, in travel order."""
        path: list[Edge] = []
        e = via[target]
        while e is not None:
            path.append(e)
            e = via[e.tail]
        path.reverse()
        return path

    @staticmethod
    def augment(path: list[Edge], amount: int = 1) -> None:
        """Push `amount` units along `path`."""
        for e in path:
            e.cap -= amount
            e.partner.cap += amount
