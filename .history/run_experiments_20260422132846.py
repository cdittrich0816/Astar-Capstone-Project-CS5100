"""
Run A* experiments on predefined grids using different heuristics.
This saves results to a CSV file.
"""

import csv
import os

from experiments.predefined_grids import GRIDS
from src.astar import astar
from src.heuristics import manhattan, euclidean, weighted_manhattan

RESULTS_PATH = "results/raw_results.csv"


def run_all_experiments():
    """
    This runs A* on every predefined grid using all the selected heuristics.
    A list of result dictionaries is returned.
    """
    heuristics = {
        "manhattan": manhattan,
        "euclidean": euclidean,
        "weighted_manhattan": weighted_manhattan,
    }

    results = []

    for grid_name, grid_info in GRIDS.items():
        grid = grid_info["grid"]
        start = grid_info["start"]
        goal = grid_info["goal"]

        for heuristic_name, heuristic_fn in heuristics.items():
            result = astar(grid, start, goal, heuristic_fn)

            row = {
                "grid_name": grid_name,
                "heuristic": heuristic_name,
                "path_found": result["path"] is not None,
                "path_cost": result["path_cost"],
                "nodes_expanded": result["nodes_expanded"],
                "runtime": result["runtime"],
            }

            results.append(row)

    return results


def save_results_to_csv(results, filepath):
    """
    This save experiment results to a CSV file.
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    fieldnames = [
        "grid_name",
        "heuristic",
        "path_found",
        "path_cost",
        "nodes_expanded",
        "runtime",
    ]

    with open(filepath, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


def print_results(results):
    """
    Print results in a readable format.
    """
    print("\nExperiment Results")
    print("-" * 75)

    for row in results:
        print(
            f"Grid: {row['grid_name']:<18} "
            f"Heuristic: {row['heuristic']:<18} "
            f"Path Found: {str(row['path_found']):<5} "
            f"Path Cost: {str(row['path_cost']):<4} "
            f"Nodes Expanded: {row['nodes_expanded']:<4} "
            f"Runtime: {row['runtime']:.6f}s"
        )


if __name__ == "__main__":
    results = run_all_experiments()
    save_results_to_csv(results, RESULTS_PATH)
    print_results(results)

    print(f"\nResults saved to: {RESULTS_PATH}")