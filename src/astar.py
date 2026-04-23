"""
A* implementation for grid-based pathfinding.
"""

import time
from typing import Callable, Dict, List, Set, Tuple

from src.grid import get_neighbors

Position = Tuple[int, int]
HeuristicFunction = Callable[[Position, Position], float]


def reconstruct_path(
    came_from: Dict[Position, Position],
    current: Position
) -> List[Position]:
    """
    We reconstruct the path by following parent pointers
    backward from the goal node to the start node.
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
    This runs A* search on a grid using a simple list-based open set.

    The function returns a dictionary with:
        - path: a list of positions from start to goal, or None
        - path_cost: the total number of steps, or None
        - nodes_expanded: the total number of nodes expanded
        - runtime: the execution time in seconds
    """
    start_time = time.perf_counter()

    # This stores the nodes discovered but not yet expanded
    open_list: List[Position] = [start]

    # This stores the nodes that already fully expanded
    closed_set: Set[Position] = set()

    # This stores the parent pointers for path reconstruction
    came_from: Dict[Position, Position] = {}

    # g(n): actual cost from start to node
    g_score: Dict[Position, float] = {start: 0}

    # f(n) = g(n) + h(n)
    f_score: Dict[Position, float] = {start: heuristic(start, goal)}

    nodes_expanded = 0

    while open_list:
        # This finds the node in the open list with the lowest f-score
        current = min(open_list, key=lambda node: f_score.get(node, float("inf")))

        # If the goal is reached, we reconstruct and return the path
        if current == goal:
            runtime = time.perf_counter() - start_time
            path = reconstruct_path(came_from, current)

            return {
                "path": path,
                "path_cost": len(path) - 1,
                "nodes_expanded": nodes_expanded,
                "runtime": runtime,
            }

        # Move the current node from the open list to the closed set
        open_list.remove(current)
        closed_set.add(current)
        nodes_expanded += 1

        # Explore the neighbors of the current node
        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + 1  # each move costs 1

            # If the neighbor is new, we add it to the open list
            if neighbor not in open_list:
                open_list.append(neighbor)

            # If this path is not better, we skip it
            elif tentative_g >= g_score.get(neighbor, float("inf")):
                continue

            # This is the best path found so far to this neighbor
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

    # If no path found
    runtime = time.perf_counter() - start_time
    return {
        "path": None,
        "path_cost": None,
        "nodes_expanded": nodes_expanded,
        "runtime": runtime,
    }
