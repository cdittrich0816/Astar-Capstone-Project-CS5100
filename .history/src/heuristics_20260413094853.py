"""
Heuristic functions for A* search.

These estimate the remaining cost from a node to the goal.
"""

import math
from typing import Tuple

Position = Tuple[int, int]


def manhattan(a: Position, b: Position) -> float:
    """
    Manhattan distance:
    Best for 4-direction movement on a grid.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Position, b: Position) -> float:
    """
    Euclidean distance:
    Straight-line distance between two points.
    """
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)