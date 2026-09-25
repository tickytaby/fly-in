Small intro to graph theory:
  - An augmenting path is a simple path from the source to the sink in the residual network.
    - It consists of edges with positive residual capacity.
    - Flow is increased along this path by the minimum residual capacity (bottleneck) of the edges in the path.
    - Ford-Fulkerson theorem: A flow is max iff there are no augmenting paths in the residual network

For the solver:
  - We will take a time-expanded graph approach with progressive calculations of augmenting paths.
  - Before implementing a time-expanded graph, we need to first have a starting point and/or bounds for the time dimension.
  - We will take a progressive approach which should be a good balance between limiting both calculations and memory usage. Will be the lowest memory usage approach, with extra calculations -> Assume T to be the exact number of turns to get all the drones from start to end. We start our time expanded graph at t, calculated as Dijkstra from start to end with turn costs. The resulting t is the fastest a single drone can possibly arrive, defining our lower bound for makespan to be at least t. Extra computations will be exactly (T - t) * cost_of_algo. Furthermore, we will not need to recompute all previously valid augmenting paths when adding one more turn to the simulation, as valid paths up to t will still be valid for t + 1. This will limit the "cost_of_algo" to the computational cost of finding extra paths that become feasible with t+1 w.r.t. t.
