import argparse
import os
import shutil
import sys

from parser import Map
from solver import SSPSolver
from visualizer import ColorPalette, TerminalVisualizer


def main() -> None:
    """Solve the map given on the command line and print the turns.

    In a terminal the output is visual: colored move lines plus the
    state of every zone per turn. With --plain, or when the output is
    redirected, only the required move lines are printed.
    """
    args_parser = argparse.ArgumentParser(
        description="Route drones from start to end in the fewest turns."
    )
    args_parser.add_argument("map_file", help="path to the map file")
    args_parser.add_argument("--plain", action="store_true",
                             help="print only the required move lines")
    args_parser.add_argument("-v", "--verbose", action="store_true",
                             help="log cancellations, growth and cuts")
    args = args_parser.parse_args()
    try:
        mp = Map.get_from_file(args.map_file)
        solver = SSPSolver.solve(mp, verbose=args.verbose)
    except Exception as e:
        print(e)
        sys.exit(1)
    if args.plain or not sys.stdout.isatty():
        for line in solver.output_lines():
            print(line)
        return
    palette = ColorPalette(enabled="NO_COLOR" not in os.environ)
    width = shutil.get_terminal_size((100, 24)).columns
    for line in TerminalVisualizer(mp, solver.routes, palette,
                                   width).render():
        print(line)


if __name__ == "__main__":
    main()
