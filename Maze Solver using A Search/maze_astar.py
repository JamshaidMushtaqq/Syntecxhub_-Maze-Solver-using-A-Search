"""
Maze Solver using A* Search
---------------------------
- Grid/maze: '#' = wall, '.' = open cell, 'S' = start, 'G' = goal
- Each open cell is a node; neighbours are 4-directional (up/down/left/right)
- A* uses f(n) = g(n) + h(n)
      g(n) = cost from start to n
      h(n) = heuristic estimate from n to goal (Manhattan or Euclidean)
- Returns the shortest path, or None if the goal is unreachable
"""

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

Pos = Tuple[int, int]  # (row, col)


# ---------------------------------------------------------------- Maze model
class Maze:
    def __init__(self, layout: List[str]):
        self.grid = [list(row) for row in layout]
        self.rows = len(self.grid)
        self.cols = len(self.grid[0]) if self.rows else 0
        self.start: Optional[Pos] = None
        self.goal: Optional[Pos] = None

        for r in range(self.rows):
            if len(self.grid[r]) != self.cols:
                raise ValueError("All maze rows must have the same length")
            for c in range(self.cols):
                if self.grid[r][c] == "S":
                    self.start = (r, c)
                elif self.grid[r][c] == "G":
                    self.goal = (r, c)

        if self.start is None or self.goal is None:
            raise ValueError("Maze must contain both 'S' (start) and 'G' (goal)")

    def is_walkable(self, pos: Pos) -> bool:
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r][c] != "#"

    def neighbors(self, pos: Pos) -> List[Pos]:
        r, c = pos
        candidates = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        return [p for p in candidates if self.is_walkable(p)]


# ---------------------------------------------------------------- Heuristics
def manhattan(a: Pos, b: Pos) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Pos, b: Pos) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


HEURISTICS = {"manhattan": manhattan, "euclidean": euclidean}


# ---------------------------------------------------------------- A* search
def astar(maze: Maze, heuristic: str = "manhattan"):
    """
    Returns (path, explored_nodes)
      path: list of positions from start to goal, or None if unreachable
      explored_nodes: set of nodes expanded during the search (for visualization)
    """
    h = HEURISTICS[heuristic]
    start, goal = maze.start, maze.goal

    # Priority queue entries: (f, h, counter, node). Counter breaks ties safely.
    counter = 0
    open_heap = [(h(start, goal), h(start, goal), counter, start)]
    came_from: Dict[Pos, Pos] = {}
    g_score: Dict[Pos, float] = {start: 0}
    closed: Set[Pos] = set()

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return reconstruct_path(came_from, current), closed

        for nxt in maze.neighbors(current):
            tentative_g = g_score[current] + 1  # each step costs 1
            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                f = tentative_g + h(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (f, h(nxt, goal), counter, nxt))

    return None, closed  # open list empty -> goal unreachable


def reconstruct_path(came_from: Dict[Pos, Pos], current: Pos) -> List[Pos]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------------------------------------------------------------- Visualization
def render(maze: Maze, path: Optional[List[Pos]], explored: Set[Pos]) -> str:
    """Console view: '*' = final path, 'o' = explored but not on path."""
    path_set = set(path) if path else set()
    lines = []
    for r in range(maze.rows):
        row = []
        for c in range(maze.cols):
            p = (r, c)
            ch = maze.grid[r][c]
            if ch in ("S", "G", "#"):
                row.append(ch)
            elif p in path_set:
                row.append("*")
            elif p in explored:
                row.append("o")
            else:
                row.append(".")
        lines.append(" ".join(row))
    return "\n".join(lines)


def plot(maze: Maze, path: Optional[List[Pos]], explored: Set[Pos], title: str = ""):
    """Optional matplotlib plot (only used if matplotlib is installed)."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError:
        print("matplotlib not installed; skipping plot (pip install matplotlib)")
        return

    # 0 open, 1 wall, 2 explored, 3 path, 4 start, 5 goal
    data = [[0] * maze.cols for _ in range(maze.rows)]
    for r in range(maze.rows):
        for c in range(maze.cols):
            if maze.grid[r][c] == "#":
                data[r][c] = 1
    for r, c in explored:
        data[r][c] = 2
    for r, c in (path or []):
        data[r][c] = 3
    data[maze.start[0]][maze.start[1]] = 4
    data[maze.goal[0]][maze.goal[1]] = 5

    cmap = ListedColormap(["white", "black", "#bfdcff", "#ffd54f", "green", "red"])
    plt.imshow(data, cmap=cmap, vmin=0, vmax=5)
    plt.xticks([]); plt.yticks([])
    plt.title(title)
    plt.show()


# ---------------------------------------------------------------- Demo
def solve_and_report(name: str, layout: List[str], heuristic: str = "manhattan"):
    maze = Maze(layout)
    path, explored = astar(maze, heuristic)

    print(f"\n=== {name} (heuristic: {heuristic}) ===")
    if path is None:
        print("No path found: goal is unreachable.")
    else:
        print(f"Shortest path length: {len(path) - 1} steps")
        print(f"Nodes explored: {len(explored)}")
        print("Path:", path)
    print(render(maze, path, explored))
    return maze, path, explored


if __name__ == "__main__":
    solvable = [
        "S . . # . . . .",
        ". # . # . # # .",
        ". # . . . . # .",
        ". # # # # . # .",
        ". . . . # . . .",
        "# # # . # # # .",
        ". . . . . . . G",
    ]
    solvable = [row.replace(" ", "") for row in solvable]

    unreachable = [
        "S . # . .",
        ". . # . .",
        "# # # . .",
        ". . . . G",
    ]
    unreachable = [row.replace(" ", "") for row in unreachable]

    for hname in ("manhattan", "euclidean"):
        solve_and_report("Solvable maze", solvable, hname)

    solve_and_report("Unreachable maze", unreachable)

    # Uncomment for a graphical plot:
    # m, p, e = solve_and_report("Plot demo", solvable)
    # plot(m, p, e, "A* Maze Solver")
