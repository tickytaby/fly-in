import sys

from parser import Map
from solver import SSPSolver


def main() -> None:
    """Solve the map given on the command line and print the turns."""
    if len(sys.argv) < 2:
        print("Usage: python src/main.py <map_file> [-v]")
        sys.exit(1)
    try:
        mp = Map.get_from_file(sys.argv[1])
        solver = SSPSolver.solve(mp, verbose="-v" in sys.argv[2:])
    except Exception as e:
        print(e)
        sys.exit(1)
    for line in solver.output_lines():
        print(line)
    print(f"Makespan: {solver.makespan} turns for {mp.nb_drones} drones")


if __name__ == "__main__":
    main()
