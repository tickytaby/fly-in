from pathlib import Path

import pytest

from parser import Map


def test_neg_nb_drones() -> None:
    with pytest.raises(Exception, match="nb_drones must be a positive int"):
        Map.get_from_file("./maps/easy/neg_nb_drones.txt")


def test_nbdrones_not_first() -> None:
    with pytest.raises(Exception,
                       match="nb_drones defined not on first line"):
        Map.get_from_file("./maps/easy/bad_01_linear_path.txt")


def test_mult_starts() -> None:
    with pytest.raises(Exception, match="Multiple start_hubs"):
        Map.get_from_file("./maps/easy/multiple_starts.txt")


def test_mult_ends() -> None:
    with pytest.raises(Exception, match="Multiple end_hubs"):
        Map.get_from_file("./maps/easy/multiple_ends.txt")


def test_error_reports_file_line_number(tmp_path: Path) -> None:
    p = tmp_path / "map.txt"
    p.write_text("# comment\n"
                 "nb_drones: 2\n"
                 "start_hub: s 0 0\n"
                 "end_hub: g 1 0\n"
                 "connection: s-x\n")
    with pytest.raises(Exception, match=r"Line #5 ->"):
        Map.get_from_file(str(p))
