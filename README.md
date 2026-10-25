# Maze Solver using A* Search

A Python implementation of the A* search algorithm that finds the shortest path through a grid-based maze. It supports both Manhattan and Euclidean heuristics, handles unreachable goals gracefully, and visualizes the search in the console (with an optional matplotlib plot).

## Features

- Represents a maze as a grid of nodes (start, goal, walls, open cells)
- A* search with a selectable heuristic: **Manhattan** or **Euclidean**
- Returns the shortest path between start and goal
- Detects and reports unreachable goals (no path exists)
- Console visualization of explored nodes and the final path
- Optional graphical plot using matplotlib

## Maze Format

The maze is a list of strings. Each character represents one cell:

| Symbol | Meaning            |
|--------|--------------------|
| `S`    | Start position     |
| `G`    | Goal position      |
| `#`    | Wall (blocked)     |
| `.`    | Open cell          |

Example:

```
S . . # . . . .
. # . # . # # .
. # . . . . # .
. # # # # . # .
. . . . # . . .
# # # . # # # .
. . . . . . . G
```

## How It Works

1. Every open cell is a **node**. Each node is connected to its open neighbors (up, down, left, right), and every move costs 1.
2. A* evaluates nodes using:

   ```
   f(n) = g(n) + h(n)
   ```

   - `g(n)`: actual cost from the start to node `n`
   - `h(n)`: estimated cost from `n` to the goal (the heuristic)
3. A priority queue (`heapq`) always expands the node with the lowest `f` value first.
4. When the goal is reached, the path is rebuilt by following parent links back to the start.
5. If the priority queue becomes empty before the goal is found, the goal is unreachable and `None` is returned.

## Heuristics

- **Manhattan distance:** `|x1 - x2| + |y1 - y2|` - best suited to grids with 4-directional movement.
- **Euclidean distance:** `sqrt((x1 - x2)^2 + (y1 - y2)^2)` - straight-line distance.

Both heuristics are admissible (they never overestimate the true cost), so A* is guaranteed to return an optimal path.

## Requirements

- Python 3.8 or higher
- `matplotlib` (optional, only needed for the graphical plot)

```
pip install matplotlib
```

## Usage

Run the demo:

```
python maze_astar.py
```

Use it in your own code:

```python
from maze_astar import Maze, astar, render

layout = [
    "S..#",
    ".#..",
    "...G",
]

maze = Maze(layout)
path, explored = astar(maze, heuristic="manhattan")  # or "euclidean"

if path is None:
    print("No path found: goal is unreachable.")
else:
    print("Shortest path length:", len(path) - 1)
    print(render(maze, path, explored))
```

To show the graphical plot, call:

```python
from maze_astar import plot
plot(maze, path, explored, "A* Maze Solver")
```

## Sample Output

```
=== Solvable maze (heuristic: manhattan) ===
Shortest path length: 13 steps
Nodes explored: 14
S . . # . . . .
* # . # . # # .
* # . . . . # .
* # # # # . # .
* * * * # . . .
# # # * # # # .
. . . * * * * G
```

Legend for console output:

- `*` - cell on the final shortest path
- `o` - cell explored by A* but not on the final path
- `#` - wall
- `S` / `G` - start / goal

Unreachable example:

```
=== Unreachable maze (heuristic: manhattan) ===
No path found: goal is unreachable.
```

## Heuristic Comparison

On the sample solvable maze, both heuristics found the same shortest path length (13 steps), but Manhattan explored fewer nodes:

| Heuristic  | Path Length | Nodes Explored |
|------------|-------------|----------------|
| Manhattan  | 13          | 14             |
| Euclidean  | 13          | 23             |

## Project Structure

```
.
├── maze_astar.py   # Maze model, A* search, heuristics, visualization, demo
└── README.md       # Project documentation
```

## Possible Improvements

- Diagonal movement support
- Weighted cells (different movement costs)
- Random maze generator
- Step-by-step animated visualization
- Loading mazes from text files
