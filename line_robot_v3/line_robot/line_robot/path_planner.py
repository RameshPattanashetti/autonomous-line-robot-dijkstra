#!/usr/bin/env python3
"""
path_planner.py — Dijkstra shortest-path pre-planner
=====================================================
ROS2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic

All coordinates are in the ODOMETRY frame, which has its origin
at the robot spawn point (-3.5, 0 in world frame).
Formula:  odom_x = world_x - SPAWN_X,  odom_y = world_y - SPAWN_Y

GRAPH LAYOUT (odom frame)
-------------------------
Two routes between junction A and junction B:

  START(0,0) -n0- junc_a(2.7,0.45)
                    |              \
               main branch      shortcut branch
               n2(4.3,-0.45)    sc_mid(4.95,0.55)
               n3(5.7,0.45)          |
               junc_b(7.2,-0.35) <---+
                    |
                n6(8.8,0.2) - END(10.0,0.0)

Obstacle positions in odom frame:
  obs1: world(-0.05,-0.03) => odom(3.45,-0.03)  — on main branch
  obs2: world(4.55,-0.06)  => odom(8.05,-0.06)  — on tail, after merge
"""

import heapq
import math
from typing import Dict, List, Tuple

# Robot spawn in world frame (from simulation.launch.py)
SPAWN_X = -3.5
SPAWN_Y =  0.0

def _w2o(wx: float, wy: float):
    """Convert world-frame coords to odom-frame coords."""
    return (wx - SPAWN_X, wy - SPAWN_Y)

# ── Nodes in ODOM frame ───────────────────────────────────────────────
NODES: Dict[str, Tuple[float, float]] = {
    "start":  _w2o(-3.50,  0.00),   # (0.00,  0.00)
    "n0":     _w2o(-2.20,  0.00),   # (1.30,  0.00)
    "junc_a": _w2o(-0.80,  0.45),   # (2.70,  0.45)
    # main zig-zag branch
    "n2":     _w2o( 0.80, -0.45),   # (4.30, -0.45)  near obs1
    "n3":     _w2o( 2.20,  0.45),   # (5.70,  0.45)
    "n4":     _w2o( 3.70, -0.35),   # (7.20, -0.35)  == junc_b
    # shortcut branch
    "sc_mid": _w2o( 1.45,  0.55),   # (4.95,  0.55)  clears obs1
    # shared tail
    "junc_b": _w2o( 3.70, -0.35),   # (7.20, -0.35)
    "n6":     _w2o( 5.30,  0.20),   # (8.80,  0.20)
    "end":    _w2o( 6.50,  0.00),   # (10.00, 0.00)
}

# Obstacle centres in ODOM frame + collision radii
OBSTACLES = [
    (_w2o(-0.05, -0.03), 0.07),   # obs1 — blue cylinder
    (_w2o( 4.55, -0.06), 0.08),   # obs2 — red cylinder
]

OBSTACLE_PENALTY        = 50.0
OBSTACLE_INFLUENCE_RADIUS = 0.50  # m

EDGES: List[Tuple[str, str]] = [
    ("start",  "n0"),
    ("n0",     "junc_a"),
    # main branch
    ("junc_a", "n2"),
    ("n2",     "n3"),
    ("n3",     "n4"),
    # shortcut branch
    ("junc_a", "sc_mid"),
    ("sc_mid", "junc_b"),
    # shared tail (n4 and junc_b are the same point)
    ("junc_b", "n6"),
    ("n6",     "end"),
]


def _node_penalty(name: str) -> float:
    x, y = NODES[name]
    total = 0.0
    for (ox, oy), r in OBSTACLES:
        d = math.dist((x, y), (ox, oy))
        if d < r + OBSTACLE_INFLUENCE_RADIUS:
            total += OBSTACLE_PENALTY * (1.0 - d / (r + OBSTACLE_INFLUENCE_RADIUS))
    return total


def _build_graph() -> Dict[str, List[Tuple[str, float]]]:
    graph: Dict[str, List[Tuple[str, float]]] = {n: [] for n in NODES}
    for a, b in EDGES:
        dist    = math.dist(NODES[a], NODES[b])
        cost_ab = dist + _node_penalty(b)
        cost_ba = dist + _node_penalty(a)
        graph[a].append((b, cost_ab))
        graph[b].append((a, cost_ba))
    return graph


def dijkstra(source: str = "start", target: str = "end") -> Tuple[List[str], float]:
    graph = _build_graph()
    dist  = {n: float("inf") for n in NODES}
    prev: Dict[str, str] = {}
    dist[source] = 0.0
    heap = [(0.0, source)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == target:
            break
        for v, w in graph[u]:
            alt = dist[u] + w
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(heap, (alt, v))

    path, cur = [], target
    while cur in prev:
        path.append(cur)
        cur = prev[cur]
    path.append(source)
    path.reverse()
    return path, dist[target]


def plan_waypoints() -> Tuple[List[Tuple[float, float]], float, str]:
    """
    Run Dijkstra and return:
      waypoints  — list of (x, y) in ODOM frame
      total_cost — Dijkstra cost
      route_name — 'SHORTCUT' or 'MAIN LINE'
    """
    node_path, cost = dijkstra()
    waypoints  = [NODES[n] for n in node_path]
    route_name = "SHORTCUT" if "sc_mid" in node_path else "MAIN LINE"
    return waypoints, cost, route_name


if __name__ == "__main__":
    wps, cost, name = plan_waypoints()
    print(f"Chosen route : {name}")
    print(f"Total cost   : {cost:.3f}")
    print("Waypoints (odom frame):")
    for i, (x, y) in enumerate(wps):
        print(f"  [{i:2d}]  odom({x:6.2f}, {y:6.2f})  "
              f"world({x+SPAWN_X:6.2f}, {y+SPAWN_Y:6.2f})")
