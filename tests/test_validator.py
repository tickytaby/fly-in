from pathlib import Path

import pytest

from parser import Map
from solver import SSPSolver
from time_expanded import TimeExpandedGraph
from validator import Validator, Violation

LINEAR = "./maps/easy/01_linear_path.txt"
CAPACITY = "./maps/easy/03_basic_capacity.txt"

# a and b both lead to the goal and share a cap-1 link.
SQUARE = """nb_drones: 2
start_hub: s 0 0
end_hub: g 2 0
hub: a 1 1
hub: b 1 -1
connection: s-a
connection: s-b
connection: a-b
connection: a-g
connection: b-g
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


def check(mp: Map, lines: list[str]) -> Violation | None:
    return Validator(mp).validate(lines)


# Validator

def test_valid_output_passes() -> None:
    mp = Map.get_from_file(LINEAR)
    lines = ["D1-waypoint1", "D1-waypoint2 D2-waypoint1",
             "D1-goal D2-waypoint2", "D2-goal"]
    assert check(mp, lines) is None


def test_swap_on_cap1_link_is_a_repairable_violation(tmp_path: Path) -> None:
    mp = load(tmp_path, SQUARE)
    v = check(mp, ["D1-a D2-b", "D1-b D2-a", "D1-g D2-g"])
    assert v is not None and v.turn == 2 and v.connection == "a-b"
    assert v.crossings == {("a", "b"): 1, ("b", "a"): 1}
    assert v.cut() == ("a", "b", 2)


def test_same_direction_over_link_capacity() -> None:
    mp = Map.get_from_file(LINEAR)
    v = check(mp, ["D1-waypoint1 D2-waypoint1"])
    assert v is not None and "link start-waypoint1" in v.message
    # One direction only: the model is wrong, a cut would not help.
    assert v.cut() is None


def test_zone_over_capacity() -> None:
    mp = Map.get_from_file(CAPACITY)
    v = check(mp, ["D1-bottleneck D2-bottleneck D3-bottleneck"])
    assert v is not None and "bottleneck holds 3 > 2" in v.message
    assert v.cut() is None


@pytest.mark.parametrize("lines,message", [
    (["D1-r"], "enters restricted zone"),
    (["D1-s-r", ""], "D1 waits on s-r"),
    (["D1-s-r", "D1-s"], "leaves transit"),
    (["D1-s-r", "D1-r", "D1-t"], "drones not delivered"),
    (["D1-t"], "no link from s"),
    (["D4-s-r"], "unknown drone"),
])
def test_rule_violations(tmp_path: Path, lines: list[str],
                         message: str) -> None:
    v = check(load(tmp_path, RESTRICTED), lines)
    assert v is not None and message in v.message


# Cuts

def test_cut_removes_normal_move() -> None:
    mp = Map.get_from_file(LINEAR)
    g = TimeExpandedGraph(mp, {("start", "waypoint1", 1)})
    g.build(2)
    assert not any(g.describe(e.tail) == "out:start:0"
                   and g.describe(e.to) == "in:waypoint1:1"
                   for e in g.edges if e.original)
    solver = SSPSolver(mp, cuts={("start", "waypoint1", 1)})
    solver.run()
    assert [r.arrival for r in solver.routes] == [4, 5]


def test_cut_removes_restricted_entry(tmp_path: Path) -> None:
    mp = load(tmp_path, RESTRICTED)
    solver = SSPSolver(mp, cuts={("s", "r", 1)})
    solver.run()
    assert [r.arrival for r in solver.routes] == [4, 5, 6]
    assert solver.routes[0].positions == ["s", "s", "s-r", "r", "t"]


def test_solve_cuts_and_resolves(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []

    def fake_validate(self: Validator, lines: list[str]) -> Violation | None:
        calls.append(1)
        if len(calls) == 1:
            return Violation("fake crossing", 1, "start-waypoint1",
                             {("start", "waypoint1"): 1,
                              ("waypoint1", "start"): 1})
        return None

    monkeypatch.setattr(Validator, "validate", fake_validate)
    solver = SSPSolver.solve(Map.get_from_file(LINEAR))
    assert len(calls) == 2
    assert solver.graph.cuts == {("start", "waypoint1", 1)}
    assert [r.arrival for r in solver.routes] == [4, 5]


def test_solve_refuses_unrepairable(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(Validator, "validate",
                        lambda self, lines: Violation("broken", 1))
    with pytest.raises(Exception, match="invalid schedule"):
        SSPSolver.solve(Map.get_from_file(LINEAR))


def test_solve_needs_no_cut_on_provided_maps() -> None:
    for path in sorted(Path("maps").glob("*/0*.txt")):
        solver = SSPSolver.solve(Map.get_from_file(str(path)))
        assert solver.graph.cuts == set(), path
