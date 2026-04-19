"""
A simple A* implementation for grid-based pathfinding.
"""

import heapq
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from src.grid import get_neighbors

Position = Tuple[int, int]
HeuristicFunction = Callable[[Position, Position], float]


def reconstruct_path(
    came_from: Dict[Position, Position],
    current: Position
) -> List[Position]:
    """
    Reconstruct the final path by walking backward from the goal.
    """
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(
    grid: List[List[int]],
    start: Position,
    goal: Position,
    heuristic: HeuristicFunction
) -> Dict[str, object]:
    """
    Run A* search on a grid.

    Returns a dictionary with:
        - path: list of positions from start to goal, or None
        - path_cost: total number of steps, or None
        - nodes_expanded: number of nodes expanded
        - runtime: execution time in seconds
    """
    start_time = time.perf_counter()

    # Priority queue holds (f_score, node)
    open_heap: List[Tuple[float, Position]] = []
    heapq.heappush(open_heap, (0, start))

    came_from: Dict[Position, Position] = {}

    # g(n): cost from start to node
    g_score: Dict[Position, float] = {start: 0}

    # f(n) = g(n) + h(n)
    f_score: Dict[Position, float] = {start: heuristic(start, goal)}

    closed_set: Set[Position] = set()
    nodes_expanded = 0

    while open_heap:
        _, current = heapq.heappop(open_heap)

        # Skip if already processed
        if current in closed_set:
            continue

        nodes_expanded += 1

        # Goal found
        if current == goal:
            runtime = time.perf_counter() - start_time
            path = reconstruct_path(came_from, current)

            return {
                "path": path,
                "path_cost": len(path) - 1,  # each move costs 1
                "nodes_expanded": nodes_expanded,
                "runtime": runtime,
            }

        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + 1  # uniform move cost

            # Update if this path to neighbor is better
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

                heapq.heappush(open_heap, (f_score[neighbor], neighbor))

    # No path found
    runtime = time.perf_counter() - start_time
    return {
        "path": None,
        "path_cost": None,
        "nodes_expanded": nodes_expanded,
        "runtime": runtime,
    }