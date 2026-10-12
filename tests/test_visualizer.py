import sys
from pathlib import Path

import pytest

import main
from parser import Map
from solver import SSPSolver
from visualizer import ColorPalette, TerminalVisualizer

LINEAR = "./maps/easy/01_linear_path.txt"

RESTRICTED = """nb_drones: 2
start_hub: s 0 0
end_hub: t 2 0
hub: r 1 0 [zone=restricted color=rainbow]
connection: s-r
connection: r-t
"""


def visual(mp: Map, colors: bool = False) -> TerminalVisualizer:
    solver = SSPSolver.solve(mp)
    return TerminalVisualizer(mp, solver.routes, ColorPalette(colors))


# Colors

def test_known_color_names() -> None:
    p = ColorPalette(True)
    assert p.code("red") == 196
    assert p.code("Gold") == p.code("gold") == 220
    assert p.code(None) is None
    assert p.paint("x", "red") == "\033[38;5;196mx\033[0m"


def test_unknown_color_is_stable() -> None:
    p = ColorPalette(True)
    code = p.code("unicorn_sparkle")
    assert code in ColorPalette.FALLBACK
    assert all(ColorPalette(True).code("unicorn_sparkle") == code
               for _ in range(3))


def test_rainbow_colors_each_letter() -> None:
    painted = ColorPalette(True).paint("abc", "rainbow")
    assert painted.count("\033[38;5;") == 3


def test_disabled_palette_leaves_text_alone() -> None:
    p = ColorPalette(False)
    assert p.paint("text", "red", bold=True, dim=True) == "text"


# Turn rendering

@pytest.mark.parametrize("path", sorted(Path("maps").glob("*/0*.txt")))
def test_uncolored_moves_match_required_format(path: Path) -> None:
    mp = Map.get_from_file(str(path))
    solver = SSPSolver.solve(mp)
    vis = TerminalVisualizer(mp, solver.routes, ColorPalette(False))
    assert [vis.render_moves(t) for t in range(1, solver.makespan + 1)] \
        == solver.output_lines()


def test_state_shows_occupancy() -> None:
    vis = visual(Map.get_from_file(LINEAR))
    state = "\n".join(vis.render_state(2))
    assert "start: 0 waiting" in state
    assert "goal: 0/2 delivered" in state
    assert "waypoint1 ●" in state and "waypoint2 ●" in state
    assert "goal: 1/2 delivered" in "\n".join(vis.render_state(3))


def test_state_shows_drones_in_flight(tmp_path: Path) -> None:
    p = tmp_path / "map.txt"
    p.write_text(RESTRICTED)
    vis = visual(Map.get_from_file(str(p)))
    state = vis.render_state(1)
    assert state[-1] == "  in flight: D1 s-r → r"
    assert "r! ○" in "\n".join(state)


def test_summary_metrics() -> None:
    summary = "\n".join(visual(Map.get_from_file(LINEAR)).render_summary())
    assert "4 turns for 2 drones" in summary
    assert "average turns / drone:  3.50" in summary
    assert "total movement turns:   6" in summary


# Entry point

def test_main_plain_output(monkeypatch: pytest.MonkeyPatch,
                           capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr(sys, "argv", ["main.py", LINEAR, "--plain"])
    main.main()
    assert capsys.readouterr().out.splitlines() == [
        "D1-waypoint1", "D1-waypoint2 D2-waypoint1",
        "D1-goal D2-waypoint2", "D2-goal",
    ]


def test_main_reports_errors(monkeypatch: pytest.MonkeyPatch,
                             capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr(sys, "argv", ["main.py", "missing.txt"])
    with pytest.raises(SystemExit) as exit_info:
        main.main()
    assert exit_info.value.code == 1
    assert "Failed reading file" in capsys.readouterr().out
