# -------------------------------------------------------------------------------------
# STEP 1: Import libraries and define constants
# -------------------------------------------------------------------------------------
import heapq, time, random, os
from itertools import permutations
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.animation import FuncAnimation, PillowWriter

ROWS, COLS = 12, 24

# -------------------------------------------------------------------------------------
# STEP 2: Define the maze layout
# -1 = Start, 9 = Goal, 1 = Wall, 0 = Free space
# -------------------------------------------------------------------------------------
maze = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,-1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,1,1,0,0,0,1,0,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,1,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,1,1,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,1,0,0,0,1,0,1],
    [1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]

# -------------------------------------------------------------------------------------
# STEP 3: Node class for A* pathfinding
# -------------------------------------------------------------------------------------
class Node:
    def __init__(self, row, col, g, h, parent=None):
        self.row, self.col, self.g, self.h = row, col, g, h
        self.f = g + h
        self.parent = parent
    def __lt__(self, other): return self.f < other.f

# -------------------------------------------------------------------------------------
# STEP 4: Heuristic function (Manhattan distance)
# -------------------------------------------------------------------------------------
def heuristic(r1, c1, r2, c2):
    return abs(r1 - r2) + abs(c1 - c2)

# -------------------------------------------------------------------------------------
# STEP 5: Validate a move in the maze
# -------------------------------------------------------------------------------------
def is_valid(r, c):
    return 0 <= r < ROWS and 0 <= c < COLS and maze[r][c] != 1

# -------------------------------------------------------------------------------------
# STEP 6: Reconstruct path from A* search
# -------------------------------------------------------------------------------------
def reconstruct_path(node):
    path = []
    while node.parent:
        path.append((node.row, node.col))
        node = node.parent
    return path[::-1]

# -------------------------------------------------------------------------------------
# STEP 7: A* search between two points
# -------------------------------------------------------------------------------------
def astar(start, goal):
    sr, sc = start
    gr, gc = goal
    open_set = []
    heapq.heappush(open_set, Node(sr, sc, 0, heuristic(sr, sc, gr, gc)))
    closed = [[False]*COLS for _ in range(ROWS)]
    expanded = 0

    while open_set:
        current = heapq.heappop(open_set)
        r, c = current.row, current.col
        if closed[r][c]: continue
        closed[r][c] = True
        expanded += 1

        if (r, c) == (gr, gc):
            return reconstruct_path(current), expanded

        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = r + dr, c + dc
            if is_valid(nr, nc) and not closed[nr][nc]:
                g = current.g + 1
                h = heuristic(nr, nc, gr, gc)
                heapq.heappush(open_set, Node(nr, nc, g, h, current))
    return [], expanded

# -------------------------------------------------------------------------------------
# STEP 8: Multi-goal path planning using A* (try all permutations)
# -------------------------------------------------------------------------------------
def multi_target_path(start, goals):
    best_path, best_order, min_cost, total_expanded = [], [], float('inf'), 0
    for perm in permutations(goals):
        current, full_path, cost, expanded_total = start, [], 0, 0
        for goal in perm:
            segment, expanded = astar(current, goal)
            if not segment: break
            full_path += segment
            cost += len(segment)
            expanded_total += expanded
            current = goal
        if full_path and cost < min_cost:
            best_path = full_path
            best_order = perm
            min_cost = cost
            total_expanded = expanded_total
    return best_path, best_order, min_cost, total_expanded

# -------------------------------------------------------------------------------------
# STEP 9: Randomly place 2–4 goal points
# - Uses a fixed seed (random.seed(480)) for consistent results
# - You can change or remove the seed for different target sets
# -------------------------------------------------------------------------------------
def set_random_targets(min_targets=2, max_targets=4):
    random.seed(168)  # Change this seed if you want different results
    for i in range(ROWS):
        for j in range(COLS):
            if maze[i][j] == 9:
                maze[i][j] = 0
    count = random.randint(min_targets, max_targets)
    empty = [(i, j) for i in range(ROWS) for j in range(COLS) if maze[i][j] == 0]
    random.shuffle(empty)
    for i in range(count):
        maze[empty[i][0]][empty[i][1]] = 9

# -------------------------------------------------------------------------------------
# STEP 10: Locate start and goal cells from maze
# -------------------------------------------------------------------------------------
def find_start_and_goals():
    start, goals = None, []
    for i in range(ROWS):
        for j in range(COLS):
            if maze[i][j] == -1: start = (i, j)
            elif maze[i][j] == 9: goals.append((i, j))
    return start, goals

# -------------------------------------------------------------------------------------
# STEP 11: Visualize maze with matplotlib
# -------------------------------------------------------------------------------------
def plot_maze(maze, save_path=None, title="Maze", cell_labels=True):
    cmap = ListedColormap(['limegreen', 'white', 'red', 'blue', 'gold'])
    value_map = {1: 0, 0: 1, -1: 2, 9: 3, 2: 4}
    grid = [[value_map.get(cell, 0) for cell in row] for row in maze]
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(grid, cmap=cmap)
    for i in range(ROWS):
        for j in range(COLS):
            if cell_labels:
                ax.text(j, i, str(i * COLS + j + 1), ha='center', va='center', fontsize=6)
    ax.set_xticks(np.arange(COLS))
    ax.set_yticks(np.arange(ROWS))
    ax.set_xticklabels([]), ax.set_yticklabels([])
    ax.set_xticks(np.arange(-.5, COLS, 1), minor=True)
    ax.set_yticks(np.arange(-.5, ROWS, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.3)
    plt.title(title)
    plt.tight_layout()
    if save_path: plt.savefig(save_path, dpi=150); plt.close()
    else: plt.show()

# -------------------------------------------------------------------------------------
# STEP 12: Mark path on the maze using value = 2
# -------------------------------------------------------------------------------------
def mark_path(path):
    for r, c in path:
        if maze[r][c] == 0:
            maze[r][c] = 2

# -------------------------------------------------------------------------------------
# STEP 13: Print maze in ASCII (optional debug)
# -------------------------------------------------------------------------------------

# def print_maze():
#     for row in maze:
#         print(" ".join('S' if v == -1 else 'G' if v == 9 else '#' if v == 1 else '*' if v == 2 else '.' for v in row))

# -------------------------------------------------------------------------------------
# STEP 14: Create GIF using matplotlib animation
# -------------------------------------------------------------------------------------
def create_gif_from_path(maze, path, gif_name="robot_path.gif"):
    fig, ax = plt.subplots(figsize=(12, 6))
    cmap = ListedColormap(['limegreen', 'white', 'red', 'blue', 'gold'])
    value_map = {1: 0, 0: 1, -1: 2, 9: 3, 2: 4}
    temp_maze = [row[:] for row in maze]
    frames = []

    for r, c in path:
        if temp_maze[r][c] == 0:
            temp_maze[r][c] = 2
        frames.append([row[:] for row in temp_maze])

    def update(i):
        ax.clear()
        grid = [[value_map.get(cell, 0) for cell in row] for row in frames[i]]
        ax.imshow(grid, cmap=cmap)
        ax.set_xticks([]), ax.set_yticks([])
        ax.set_title(f"Step {i + 1}")

    anim = FuncAnimation(fig, update, frames=len(frames), interval=50)

    # 👇 Save GIF silently, then close the figure (prevents pop-up)
    anim.save(gif_name, writer=PillowWriter(fps=1.5))
    plt.close(fig)
    print(f"✅ GIF saved: {gif_name}")

# -------------------------------------------------------------------------------------
# STEP 15: Main execution block
# -------------------------------------------------------------------------------------
 
if __name__ == "__main__":
    print("🚀 Starting Robot Path Planning Project...")

    set_random_targets()
    start, goals = find_start_and_goals()
   

    t1 = time.time()
    path, order, cost, expanded = multi_target_path(start, goals)
    t2 = time.time()

    if path:
        create_gif_from_path(maze, path)
        mark_path(path)
        plot_maze(maze, save_path="final_maze.png", title="Final Maze")
        print("✅ PNG saved: final_maze.png")

    print("\nMaze with Final Path (*):")
    plot_maze(maze=maze)

    # print_maze()

    print("--- Path Planner  Information ---")
    print(f"Start: {start}")
    print(f"Target order: {order}")
    print(f"Total path length: {cost}")
    print(f"Optimal path: {[start] + path}")

    cell_path = [start] + path
    cell_numbers = [r * COLS + c + 1 for r, c in cell_path]
    print(f"Optimal path cell numbers: {cell_numbers}")

    with open("optimal_path.txt", "w") as f:
        f.write("Optimal Path (coordinates):\n")
        f.write(str(cell_path) + "\n\n")
        f.write("Optimal Path (cell numbers):\n")
        f.write(str(cell_numbers) + "\n\n")
        f.write(f"Time Taken: {t2 - t1:.4f} s \n\n")
        f.write(f"Nodes expanded: {expanded} \n\n")
        f.write(f"Computation time: {t2 - t1:.4f} s \n\n")
        f.write(f"Number of targets: {len(goals)} \n\n")
     

    print("✅ Path saved to optimal_path.txt")
    print(f"Nodes expanded: {expanded}")
    print(f"Computation time: {t2 - t1:.4f} s")
    print(f"Number of targets: {len(goals)}")
    print("🎉 Robot path planning complete!")