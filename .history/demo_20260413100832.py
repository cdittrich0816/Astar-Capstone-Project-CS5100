"""
Simple demo script for showing A* on one predefined grid.
"""

from experiments.predefined_grids import GRIDS
from src.astar import astar
from src.grid import print_grid
from src.heuristics import manhattan, euclidean


def run_demo(grid_name: str = "small_easy") -> None:
    """
    Run A* with both heuristics on one chosen grid and print results.
    """
    if grid_name not in GRIDS:
        raise ValueError(f"Grid '{grid_name}' not found in predefined grids.")

    grid_info = GRIDS[grid_name]
    grid = grid_info["grid"]
    start = grid_info["start"]
    goal = grid_info["goal"]

    heuristics = {
        "Manhattan": manhattan,
        "Euclidean": euclidean,
    }

    print("=" * 60)
    print(f"Demo Grid: {grid_name}")
    print("=" * 60)
    print("\nOriginal Grid:")
    print_grid(grid, start=start, goal=goal)

    for heuristic_name, heuristic_fn in heuristics.items():
        result = astar(grid, start, goal, heuristic_fn)

        print("\n" + "-" * 60)
        print(f"Heuristic: {heuristic_name}")
        print("-" * 60)

        if result["path"] is None:
            print("No path found.")
            print(f"Nodes Expanded: {result['nodes_expanded']}")
            print(f"Runtime: {result['runtime']:.6f} seconds")
        else:
            print("Path found.")
            print(f"Path Cost: {result['path_cost']}")
            print(f"Nodes Expanded: {result['nodes_expanded']}")
            print(f"Runtime: {result['runtime']:.6f} seconds")
            print("\nGrid with Path:")
            print_grid(grid, start=start, goal=goal, path=result["path"])


if __name__ == "__main__":
    run_demo("small_easy")