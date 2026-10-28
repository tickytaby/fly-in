*This project has been created as part of the 42 curriculum by tickytaby.*

# Fly-in

## Description

Fly-in routes a fleet of drones from a start zone to an end zone through a
network of zones, in as few simulation turns as possible.

The network is read from a map file. Zones have a type (`normal`,
`priority`, `restricted`, `blocked`), a capacity (`max_drones`) and an
optional color. Connections are bidirectional and have a capacity
(`max_link_capacity`). Every turn each drone may move to a neighbouring zone,
start a 2-turn flight toward a restricted zone, or wait, and no zone or
connection may ever hold more drones than its capacity.

The program:

- parses and validates the map, stopping with a clear error (line number and
  cause) on any malformed input;
- computes a schedule that minimises the total number of turns (makespan);
- checks that schedule against every movement and capacity rule;
- prints it in the required `D<ID>-<zone>` format, with a colored
  turn-by-turn view of the network when run in a terminal.

The project is written in Python 3, fully object-oriented and type-checked
with mypy. No graph library is used: the flow network, shortest-path search
and Dijkstra are implemented from scratch.

## Instructions

### Requirements

- Python 3.11 or later (pinned in `.python-version`)
- [uv](https://docs.astral.sh/uv/) is recommended. Without it, `make install`
  falls back to `venv` + `pip`.

### Installation

```sh
make install
```

### Running

```sh
make run                                   # default map: maps/easy/01_linear_path.txt
make run MAP=maps/hard/02_capacity_hell.txt
make run MAP=maps/hard/02_capacity_hell.txt ARGS=--plain
```

Or directly:

```sh
.venv/bin/python src/main.py <map_file> [--plain] [-v]
```

| Option | Effect |
|---|---|
| `--plain` | Print only the required move lines, one per turn. This is also the default whenever stdout is not a terminal (pipe, file redirect). |
| `-v`, `--verbose` | Log solver internals: flow cancellations, horizon growth and validator cuts. |
| `NO_COLOR=1` (env) | Keep the visual layout but disable ANSI colors. |

### Other Makefile rules

| Rule | Effect |
|---|---|
| `make debug` | Run the program under `pdb`. |
| `make lint` | `flake8 .` and `mypy` with the flags required by the subject. |
| `make lint-strict` | `flake8 .` and `mypy . --strict`. |
| `make test` | Run the unit tests with pytest. |
| `make clean` | Remove `__pycache__`, `.mypy_cache` and `.pytest_cache`. |

### Output format

One line per turn listing every move of that turn. A drone heading to a
restricted zone first appears on the connection, then on the zone the next
turn. Example on `maps/medium/03_priority_puzzle.txt` (5 drones, 6 turns):

```
D1-fast_junction D2-start-slow_path1
D1-fast_path D2-slow_path1 D3-fast_junction D4-start-slow_path1
D1-merge_point D2-slow_path2 D3-fast_path D4-slow_path1 D5-fast_junction
D1-goal D2-merge_point D3-merge_point D4-slow_path2 D5-fast_path
D2-goal D3-goal D4-merge_point D5-merge_point
D4-goal D5-goal
```

## Algorithm and implementation

### Why a flow problem

Drones are interchangeable: nobody cares *which* drone takes which route, only
that all of them arrive as early as possible. That turns a multi-agent path
finding problem into a **flow problem**: drones are units of flow, zones and
connections are pipes with a capacity, the start is the source and the end is
the sink.

Time matters, though (a zone holding a drone at turn 3 may be free at turn 4),
so the flow is computed on a **time-expanded network**.

### Time-expanded network (`src/time_expanded.py`)

Every zone is copied once per turn. A drone's trajectory through space *and*
time then becomes an ordinary path in a static, acyclic graph.

- **Node splitting:** each zone copy at turn `t` is an `in` node and an `out`
  node joined by an edge of capacity `max_drones`. This is how zone capacity
  is enforced. The start zone has unlimited capacity, and the end zone drains
  directly into a super sink.
- **Wait edges:** `out(z, t-1) -> in(z, t)` let a drone stay in place.
- **Move edges:** `out(u, t-1) -> in(v, t)` with capacity `max_link_capacity`.
- **Restricted zones:** the move goes through a `transit(u, v, t-1)` node that
  has *no* wait edge, so a drone on the connection must arrive the next turn.
  This is how the 2-turn cost and the "cannot wait on a connection" rule are
  encoded.
- **Blocked zones** are simply not created.

Edge costs encode the objective:

| Edge | Cost |
|---|---|
| arrival at the end zone at turn `t` | `t × 10⁶` |
| move to a normal zone | 2 |
| move to a priority zone | 1 |
| each of the two legs toward a restricted zone | 2 |
| waiting | 0 |

The huge arrival cost makes the solver minimise arrival times first. Movement
costs only break ties: among equally fast schedules it prefers fewer moves and
priority zones.

### Min-cost flow by Successive Shortest Paths (`src/solver.py`, `src/flow_network.py`)

Drones are added one at a time. For each drone the solver finds the cheapest
augmenting path from source to sink in the **residual network** and pushes
one unit of flow along it.

- Residual (backward) edges have negative cost, so paths are found with
  **SPFA**, a queue-based Bellman-Ford that handles negative edges.
- An augmenting path may use backward edges. That *cancels* part of an
  earlier drone's route and reroutes it, so routing decisions are never
  greedy or final. With `-v`, these cancellations are logged.
- SSP on a time-expanded network produces an *earliest-arrival flow*, so the
  resulting makespan is optimal for the model.

### Growing the horizon instead of guessing it

The number of turns `T` is unknown in advance, so the network is grown
progressively instead of being built for a guessed upper bound:

1. **Dijkstra** on the static graph, with turn costs (restricted = 2), gives
   the earliest turn a single drone can arrive. That is a lower bound on `T`,
   and the network is first built up to that layer.
2. SSP schedules as many drones as fit. When no augmenting path exists, one
   more layer (turn) is added and SSP continues.
3. Flow already pushed stays valid when a layer is added, so **paths are
   never recomputed from scratch**. Each new layer only pays for the drones
   it makes feasible.

If no drone can reach the end after `2 × (number of zones)` layers, the end
zone is reported as unreachable.

### Flow decomposition

Once all drones are routed, the total flow is split into one path per drone
by walking source to sink along edges that carry flow. Each path is turned
into a `Route`, the drone's position at every turn. Routes are sorted by
arrival, so `D1` is always the first drone delivered.

### Validation and cuts (`src/validator.py`)

The flow model gives each direction of a connection its own capacity, so it
does not formally forbid two drones crossing the same connection in opposite
directions in one turn. In practice min-cost flow never does this: drones are
interchangeable, so both drones waiting reaches the same state at zero cost.
It was never observed on the provided maps or on ~20,000 randomly generated
ones. To guard against it anyway, and against any other modelling mistake,
the printed output is replayed
by an **independent validator** that knows nothing about the flow network.
It checks adjacency, blocked and restricted rules, transit rules, zone and
link capacities, and that every drone is delivered.

When it finds a link used in both directions over its capacity, it **cuts**
the weaker direction at that turn from the graph and the solver re-solves.
Any other violation is a hard error, so an invalid schedule is never printed.

### Adaptability

The same model handles every topology. Disjoint paths, shared bottlenecks,
dead ends, loops, priority shortcuts and restricted detours all come out of
the min-cost flow, with no map-specific strategy or hand-tuned heuristic.
Distribution across multiple paths, strategic waiting and deadlock avoidance
are consequences of the flow formulation rather than special cases.

### Complexity

Let `Z` be the number of zones, `C` the number of connections, `T` the
makespan and `D` the number of drones.

- **Network size:** `V = O(T · Z)` nodes and `E = O(T · (Z + C))` edges.
- **Time:** one SPFA per drone, plus failed searches when the horizon grows.
  The worst case is `O((D + T) · V · E)`. In practice SPFA on this nearly
  acyclic graph runs close to linear in `E`. Every provided map, including
  the challenger, is solved in under 0.3 s.
- **Memory:** `O(V + E)`. Growing the horizon from the lower bound means no
  layer beyond the final makespan is ever built.
- **Number of drones:** `D` only adds augmentations; it does not enlarge the
  network beyond what `T` requires. A map with 500 drones on a single link
  is handled without issue.

### Parser (`src/parser.py`)

The parser reads the map line by line inside a context manager. Data is
stored in Pydantic models (`Map`, `Hub`, `HubMetadata`, `Connection`). It
rejects, with the line number and the cause:

- a missing, non-integer or non-positive `nb_drones`, or one not on the first
  line;
- a missing or duplicated start or end hub, and duplicate zone names;
- names containing dashes, and non-integer coordinates;
- malformed or unknown metadata, duplicate metadata keys, invalid zone types
  and non-positive capacities;
- connections to undefined zones and duplicate connections (`a-b` / `b-a`).

## Visual representation

When stdout is a terminal, each turn is rendered as follows:

```
── Turn 2 ──────────────────────────────
D1-fast_path D2-slow_path1 D3-fast_junction D4-start-slow_path1
  start: 1 waiting    goal: 0/5 delivered
  slow_path1! ●      slow_path2 ○       fast_junction★ ●○  fast_path ●  merge_point ○○○
  in flight: D4 start-slow_path1 → slow_path1
```

- **Move line:** the required output line. Each destination is painted in
  its zone's map color, and in-flight connections are shown in bold.
- **Zone colors:** taken from the map's `color=` metadata. Common names map to
  fixed 256-color codes. Any other name gets a stable color derived from a
  hash, so arbitrary color names still work. `rainbow` is painted letter by
  letter.
- **Zone type markers:** `★` for priority, `!` for restricted, `✗` for
  blocked (dimmed).
- **Occupancy bars:** one `●` per drone in the zone and one `○` per free
  slot, so capacity pressure is visible at a glance. Zones with more than 8
  slots show `n/cap` instead.
- **Start and end counters:** drones still waiting, and drones delivered out
  of the total.
- **In flight:** the drones currently on a connection toward a restricted
  zone, and where they will land next turn.
- **Summary:** the makespan, then the subject's secondary metrics: average
  turns per drone, total movement turns and drones moving per turn (average
  and max).

Together these make it easy to see *why* the schedule looks the way it does:
where the bottlenecks are, when drones wait, and how the flow spreads over
parallel paths. The rows wrap to the terminal width.

The move line of every turn is the required output line, colored. The lines
below it add the zone states. `--plain`, or redirecting the output, prints
only the uncolored move lines, which is handy for scripts and validators.

## Performance

Results on the provided maps:

| Map | Drones | Turns | Target |
|---|---|---|---|
| easy/01_linear_path | 2 | 4 | ≤ 6 |
| easy/02_simple_fork | 4 | 4 | ≤ 8 |
| easy/03_basic_capacity | 4 | 4 | ≤ 6 |
| medium/01_dead_end_trap | 5 | 8 | ≤ 12 |
| medium/02_circular_loop | 6 | 10 | ≤ 15 |
| medium/03_priority_puzzle | 5 | 6 | ≤ 12 |
| hard/01_maze_nightmare | 8 | 13 | ≤ 30 |
| hard/02_capacity_hell | 12 | 16 | ≤ 35 |
| hard/03_ultimate_challenge | 15 | 26 | ≤ 45 |
| challenger/01_the_impossible_dream | 25 | **43** | record 45 |

Every target is met or beaten, and the challenger map beats the 45-turn
reference record.

## Resources

### References

- L. R. Ford Jr., D. R. Fulkerson, *Constructing Maximal Dynamic Flows from
  Static Flows*, Operations Research, 1958. This paper introduces
  time-expanded networks.
- D. Gale, *Transient Flows in Networks*, Michigan Mathematical Journal, 1959.
  This paper covers earliest-arrival flows.
- M. Skutella, *An Introduction to Network Flows over Time*, 2009.
- J. Edmonds, R. M. Karp, *Theoretical Improvements in Algorithmic Efficiency
  for Network Flow Problems*, Journal of the ACM, 1972.
- T. H. Cormen et al., *Introduction to Algorithms* (CLRS), chapters on
  single-source shortest paths and maximum flow.
- [cp-algorithms: Minimum-cost flow](https://cp-algorithms.com/graph/min_cost_flow.html)
- [cp-algorithms: Dijkstra](https://cp-algorithms.com/graph/dijkstra.html)
  and [Bellman-Ford / SPFA](https://cp-algorithms.com/graph/bellman_ford.html)
- [Python `heapq` documentation](https://docs.python.org/3/library/heapq.html)
- [Pydantic](https://docs.pydantic.dev/), [mypy](https://mypy.readthedocs.io/)
  and [flake8](https://flake8.pycqa.org/) documentation

### Use of AI

AI (Claude) was used as a learning and review aid:

- **Theory:** explaining flow concepts (residual networks, augmenting paths,
  min-cost flow, time-expanded networks), which I then wrote up in my own
  notes and checked against the references above.
- **Review:** checking the project against the subject requirements and
  probing the parser with edge-case maps.
- **Documentation:** drafting this README from my notes and the code.

The algorithm design and the code were written, tested and reviewed by me,
and I can explain every part of it.
