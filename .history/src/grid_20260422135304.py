"""
Grid helper functions for A* experiments.
"""

from typing import List, Tuple

Grid = List[List[int]]
Position = Tuple[int, int]


def is_in_bounds(grid: Grid, pos: Position) -> bool:
    """
    Check if a position is inside the grid.
    """
    rows = len(grid)
    cols = len(grid[0])
    r, c = pos
    return 0 <= r < rows and 0 <= c < cols


def is_walkable(grid: Grid, pos: Position) -> bool:
    """
    Check if a position is not blocked.
    Assumes:
        0 = open cell
        1 = wall
    """
    r, c = pos
    return grid[r][c] == 0


def get_neighbors(grid: Grid, pos: Position) -> List[Position]:
    """
    Return valid 4-direction neighbors (up, down, left, right).
    """
    r, c = pos
    candidates = [
        (r - 1, c),  # up
        (r + 1, c),  # down
        (r, c - 1),  # left
        (r, c + 1),  # right
    ]

    neighbors = []
    for neighbor in candidates:
        if is_in_bounds(grid, neighbor) and is_walkable(grid, neighbor):
            neighbors.append(neighbor)

    return neighbors


def print_grid(
    grid: Grid,
    start: Position | None = None,
    goal: Position | None = None,
    path: List[Position] | None = None
) -> None:
    """
    This prints the grid in a simple readable format.

    Symbols:
        S = start
        G = goal
        * = path
        # = wall
        . = open space
    """
    path_set = set(path) if path else set()

    for r in range(len(grid)):
        row_symbols = []
        for c in range(len(grid[0])):
            pos = (r, c)

            if pos == start:
                row_symbols.append("S")
            elif pos == goal:
                row_symbols.append("G")
            elif pos in path_set:
                row_symbols.append("*")
            elif grid[r][c] == 1:
                row_symbols.append("#")
            else:
                row_symbols.append(".")
        print(" ".join(row_symbols))