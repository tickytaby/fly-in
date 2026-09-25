Small intro to graph theory:
  - An augmenting path is a simple path from the source to the sink in the residual network.
    - It consists of edges with positive residual capacity.
    - Flow is increased along this path by the minimum residual capacity (bottleneck) of the edges in the path.
    - Ford-Fulkerson theorem: A flow is max iff there are no augmenting paths in the residual network

  - The running picture: DRONES ARE WATER, CONNECTIONS ARE PIPES
  
  THE PROBLEM ITSELF:
    - Makespan -> Total time until the last agent finishes. If 5 drones arrive at turns 3, 4, 4, 6, 9 -> makespan is 9.
      - This is our main metric.
    - Sum of costs, the total of all arrival times (26 here), which is close to the spec's
        "average turns per drone" metric.
    - MAPF (Multi-Agent Path Finding) -> The general family of this problem: many agents on a graph, moving in turns,
      not allowed to collide.
    - Anonymous (unlabeled) agents -> Agents are interchangeable, so nobody cares which drone takes which route.
      -> Anonymous agents is what turns our problem from MAPF to a FLOW PROBLEM

  FLOW BASICS:
    - Network -> A graph where every edge has a CAPACITY: how much can pass through it. Like pipes, each with a max throughput.
    - Source and Sink -> Where flow enters (start zone) and where it leaves (end zone).
      - A SUPER SINK is an extra artificial node that several real endpoints all drain into, like all the end-zone copies in the time-expanded graph.
    - Flow -> An assignment of how much goes through each edge.
      - A VALID FLOW respects two rules:
        1. No edge carries more than its capacity
        2. Flow conservation: at every node except source and sink, what comes in equals what goes out. Stuff doesn't appear or disappear mid-pipe.
    - Flow value -> How much reaches the sink in total. For you, that's how many drones get delivered.
    - Max flow -> Largest flow value possible. The question it answers: "How many drones can this network carry at a given time?"

  IMPROVING A FLOW:
    - Residual network (residual graph) -> A second view of the network that shows what changes are still possible given the current flow:
      For each edge it records:
        1. A FORWARD EDGE with remaining capacity: "You can still send this much more through here".
        2. A BACKWARD EDGE with capacity equal to the current flow: "You can undo this much of what was sent".
    - Augmenting path -> A source-to-sink path in the residual network. Pushing flow along it increases the total flow value,
      even if some of its steps go along backward edges and cancel earlier flow.
    - Bottleneck -> Smallest remaining capacity along an augmenting path, which is how much you can push along it.
    - Cancellation -> Using a backward edge.
    - Cut / min cut -> A cut splits the nodes into a source side and a sink side. Its capacity is the total capacity of edges
      crossing from source side to sink side: the narrowest "wall" the flow must pass through.
    - Max-flow min-cut theorem -> Max flow = min cut. Equivalently, a flow is max exactly when no augmenting paths exist.
      This is why "keep augmenting until you can't" is correct.

  THE MAX FLOW ALGOS:
    - Ford-Fulkerson -> Repeatedly find any augmenting path and push flow along it. Correct, but it can take many rounds
      depending on which paths it happens to pick.
    - Edmonds-Karp -> Ford-Fulkerson where you always pick the augmenting path with the fewest edges, found by BFS. "Shortest"
      here means # of edges, and it's chosen purely to guarantee the algo finishes quickly.

  ADDING COSTS:
    - Cost -> Price per unit of flow on each edge. For us, it's based on the turn and zone type. Backward edges have negative cost,
      because cancelling flow refunds what you paid.
    - Min-cost flow -> Among all flows of a given value, find the cheapest.
    - Successive Shortest Paths (SSP) -> Augment along the cheapest augmenting path each round, where "shortest" now means
      lowest total cost. This gives a min-cost flow. It's what we're building.
    - Bellmand-Ford -> A shortest-path algo that handles negative edges by repeatedly relaxing every edge. Slower than Dijkstra
      but safe for resiudal networks.
    - SPFA -> A faster-in-practice variant of Bellman-Ford that only re-checks nodes whose distance just changed.
    - Potentials (Johnson's trick) -> A way of adjusting edge costs so none are negative, without changing which path is cheapest.
      This allows us to use Dijkstra with your heap on the residual network. It's a speed upgrade for later (maybe)

  TIME EXPANDED MODELLING:
    - Time-expanded network -> Copy every zone once per turn, and draw edges only forward in time. A drone's route through space
      and time becomes an ordinary path.
    - Layer -> All the copies for one turn t.
    - Horizon -> The last layer currently built.
    - Node splitting -> Replace a node with an in-node and an out-node joined by one edge, whose capacity then acts
      as the node's capacity. It's how we encode max_drones.
    - Wait edge -> An edge from a zone at turn t to the same zone at turn t+1 meaning "stay put"
    - Transit node -> The extra node for a drone mid-flight toward a restricted zone. It has no wait edge, so the drone
      must continue.
    - DAG -> A directed graph with no cycles. The time-expanded network is one, because time only moves forward.

  RESULTS:
    - Flow decomposition -> Turn the final flow into individual paths, one per drone, by repeatedly walking from source
      to sink along edges carrying flow.
    - Earliest arrival flow (Gale) -> A flow where, for every turn t, as many units as possible arrived by t. SSP on
      the time-expanded graph produces one.
    - Quickest flow / quickest transshipment -> The problem of getting a given amount of flow through in the shortest
      time, which is minimizing makespan in flow language.

  THE APPROACH:
    - SSP approach on a time-expanded graph -> gives us earliest arrival flow. This approach is both optimal and complete.
      - Optimal meaning it always finds the best answer, and complete because it always finds some answer if one exists.

For the solver:
  - We will take a time-expanded graph approach with progressive calculations of augmenting paths.
  - Before implementing a time-expanded graph, we need to first have a starting point and/or bounds for the time dimension.
  - We will take a progressive approach which should be a good balance between limiting both calculations and memory usage. Will be the lowest memory usage approach, with extra calculations -> Assume T to be the exact number of turns to get all the drones from start to end. We start our time expanded graph at t, calculated as Dijkstra from start to end with turn costs. The resulting t is the fastest a single drone can possibly arrive, defining our lower bound for makespan to be at least t. Extra computations will be exactly (T - t) * cost_of_algo. Furthermore, we will not need to recompute all previously valid augmenting paths when adding one more turn to the simulation, as valid paths up to t will still be valid for t + 1. This will limit the "cost_of_algo" to the computational cost of finding extra paths that become feasible with t+1 w.r.t. t.

To find the lower bound for simulation turns t: we will be using Dijkstra's algo on the static graph (not time-aware)

Dijkstra:
  Imagine a map of towns connected by roads, each road with a length. You light a fire in your home town, and its spreads
  along every road at the same steady speed.

  How the simulation works:
    - There exist two kind of towns:
      1. Settled towns: the fire has reached here, their distance from origin is final.
      2. Frontier towns: neighbors of settled towns. You have a best guess so far for each, based on the roads
         you've seen
    - Then repeat one step:
      1. Pick the frontier town with the smallest guess. That's the next place the fire reaches, so mark it settled.
      2. Look at its roads. For each neighbor, ask: "is going through this town shorter than my current guess for that neighbor?"
         If yes, update the guess.
    - Stop when the goal is settled.

TODO:
  - Implement the time-expanded graph.
  - SSP
