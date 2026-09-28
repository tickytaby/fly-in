from pathlib import Path

import pytest

from flow_network import FlowNetwork
from parser import Map
from solver import SSPSolver
from time_expanded import BIG, TimeExpandedGraph
from validator import Validator

LINEAR = "./maps/easy/01_linear_path.txt"

# Drone 1's cheapest route (priority m) takes x or y at turn 2; drone 2
# can only arrive at turn 3 by pushing drone 1 onto the other branch.
TRAP = """nb_drones: 2
start_hub: s 0 0
end_hub: t 4 0
hub: m 1 0 [zone=priority]
hub: a 1 1
hub: x 2 1
hub: y 2 -1
connection: s-m
connection: s-a
connection: m-x
connection: m-y
connection: a-x
connection: x-t
connection: y-t
"""

RESTRICTED = """nb_drones: 3
start_hub: s 0 0
end_hub: t 2 0
hub: r 1 0 [zone=restricted]
connection: s-r
connection: r-t
"""


def load(tmp_path: Path, text: str) -> Map:
    p = tmp_path / "map.txt"
    p.write_text(text)
    return Map.get_from_file(str(p))


def solve(mp: Map) -> SSPSolver:
    solver = SSPSolver(mp)
    solver.run()
    # The raw model output must already be valid, without any cuts.
    violation = Validator(mp).validate(solver.output_lines())
    assert violation is None, str(violation)
    return solver


# Step 1: network container

def test_edges_created_in_pairs() -> None:
    net = FlowNetwork()
    a, b, c = net.node(("a",)), net.node(("b",)), net.node(("c",))
    assert net.node(("a",)) == a
    e1 = net.add_edge(a, b, 3, 5)
    e2 = net.add_edge(b, c, 1, -2)
    for e, tail, head in ((e1, a, b), (e2, b, c)):
        assert e.original and not e.partner.original
        assert e.partner.partner is e
        assert e.tail == tail and e.to == head
        assert e.partner.to == tail and e.partner.cap == 0
        assert e.partner.cost == -e.cost
    FlowNetwork.augment([e1, e2])
    assert (e1.cap, e1.flow, e1.partner.cap) == (2, 1, 1)
    assert (e2.cap, e2.flow) == (0, 1)


# Step 2: add_layer

def edge_set(g: TimeExpandedGraph) -> set[tuple[str, str, int, int]]:
    return {(g.describe(e.tail), g.describe(e.to), e.cap, e.cost)
            for e in g.edges if e.original}


def test_linear_layers_match_hand_drawing() -> None:
    g = TimeExpandedGraph(Map.get_from_file(LINEAR))
    assert edge_set(g) == {("in:start:0", "out:start:0", 2, 0)}
    g.add_layer(1)
    g.add_layer(2)
    layer2 = {e for e in edge_set(g) if e[1].endswith(":2") or
              (e[1] == "sink" and e[0] == "in:goal:2")}
    assert layer2 == {
        ("in:start:2", "out:start:2", 2, 0),
        ("in:waypoint1:2", "out:waypoint1:2", 1, 0),
        ("in:waypoint2:2", "out:waypoint2:2", 1, 0),
        ("in:goal:2", "sink", 2, 2 * BIG),
        ("out:start:1", "in:start:2", 2, 0),
        ("out:waypoint1:1", "in:waypoint1:2", 2, 0),
        ("out:waypoint2:1", "in:waypoint2:2", 2, 0),
        ("out:start:1", "in:waypoint1:2", 1, 2),
        ("out:waypoint1:1", "in:start:2", 1, 2),
        ("out:waypoint1:1", "in:waypoint2:2", 1, 2),
        ("out:waypoint2:1", "in:waypoint1:2", 1, 2),
        ("out:waypoint2:1", "in:goal:2", 1, 2),
    }
    with pytest.raises(ValueError):
        g.add_layer(5)


def test_restricted_move_uses_transit(tmp_path: Path) -> None:
    g = TimeExpandedGraph(load(tmp_path, RESTRICTED))
    g.build(2)
    edges = edge_set(g)
    assert ("out:s:0", "transit:s:r:1", 1, 2) in edges
    assert ("transit:s:r:1", "in:r:2", 1, 2) in edges
    # No one-turn move into a restricted zone, no waiting in transit.
    assert not any(e[0].startswith("out:s") and e[1].startswith("in:r")
                   for e in edges)
    assert not any(e[0].startswith("transit") and e[1].startswith("transit")
                   for e in edges)


# Step 3: SPFA

def test_spfa_on_fresh_network() -> None:
    mp = Map.get_from_file(LINEAR)
    g = TimeExpandedGraph(mp)
    _, _, lower = mp.run_dijkstra()
    g.build(lower)
    dist, via = g.spfa(g.source)
    # 3 normal moves of cost 2 each.
    assert dist[g.sink] == lower * BIG + 3 * 2
    path = g.path_to(via, g.sink)
    assert path[0].tail == g.source and path[-1].to == g.sink


# Step 4: SSP loop

@pytest.mark.parametrize("n", [1, 2, 5])
def test_linear_arrivals(tmp_path: Path, n: int) -> None:
    text = Path(LINEAR).read_text().replace("nb_drones: 2",
                                            f"nb_drones: {n}")
    solver = solve(load(tmp_path, text))
    assert [r.arrival for r in solver.routes] == [3 + i for i in range(n)]


def test_trap_needs_cancellation(tmp_path: Path) -> None:
    solver = solve(load(tmp_path, TRAP))
    assert solver.cancellations >= 1
    assert [r.arrival for r in solver.routes] == [3, 3]


def test_restricted_spacing(tmp_path: Path) -> None:
    solver = solve(load(tmp_path, RESTRICTED))
    assert [r.arrival for r in solver.routes] == [3, 4, 5]
    assert solver.routes[0].positions == ["s", "s-r", "r", "t"]
    assert solver.output_lines() == [
        "D1-s-r", "D1-r D2-s-r", "D1-t D2-r D3-s-r", "D2-t D3-r", "D3-t",
    ]


def test_unreachable_end(tmp_path: Path) -> None:
    text = RESTRICTED.replace("hub: r 1 0 [zone=restricted]",
                              "hub: r 1 0 [zone=blocked]")
    with pytest.raises(Exception, match="unreachable"):
        SSPSolver(load(tmp_path, text)).run()


# Step 5: decomposition, output, and the subject's benchmarks

@pytest.mark.parametrize("path,target", [
    ("./maps/easy/01_linear_path.txt", 6),
    ("./maps/easy/02_simple_fork.txt", 8),
    ("./maps/easy/03_basic_capacity.txt", 6),
    ("./maps/medium/01_dead_end_trap.txt", 12),
    ("./maps/medium/02_circular_loop.txt", 15),
    ("./maps/medium/03_priority_puzzle.txt", 12),
    ("./maps/hard/01_maze_nightmare.txt", 30),
    ("./maps/hard/02_capacity_hell.txt", 35),
    ("./maps/hard/03_ultimate_challenge.txt", 45),
    ("./maps/challenger/01_the_impossible_dream.txt", 45),
])
def test_maps_valid_and_on_target(path: str, target: int) -> None:
    mp = Map.get_from_file(path)
    solver = solve(mp)
    assert len(solver.routes) == mp.nb_drones
    assert len(solver.output_lines()) == solver.makespan <= target
