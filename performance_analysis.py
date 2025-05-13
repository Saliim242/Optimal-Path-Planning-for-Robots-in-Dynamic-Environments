# -------------------------------------------------------------------------------------
# performance_analysis.py
# - Runs the path planner 20 times
# - Measures execution time for each run
# - Plots execution time as a performance chart
# -------------------------------------------------------------------------------------

import time
import matplotlib.pyplot as plt
from main import (
    maze, ROWS, COLS,
    set_random_targets,
    find_start_and_goals,
    multi_target_path
)

execution_times = []

print("📊 Starting Performance Analysis (20 runs)...")

# -------------------------------------------------------------------------------------
# STEP 1: Run the path planner 20 times and record execution time
# -------------------------------------------------------------------------------------
for run in range(1, 21):
    set_random_targets()
    start, goals = find_start_and_goals()

    print(f"🔁 Run {run}/20 started...")
    t1 = time.time()
    path, order, cost, expanded = multi_target_path(start, goals)
    t2 = time.time()

    exec_time = t2 - t1
    execution_times.append(exec_time)
    print(f"✅ Run {run} completed in {exec_time:.4f} seconds")

# -------------------------------------------------------------------------------------
# STEP 2: Plot performance chart
# -------------------------------------------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(range(1, 21), execution_times, marker='o', linestyle='-', color='blue')
plt.title("Path Planning Execution Time Over 20 Runs")
plt.xlabel("Run Number")
plt.ylabel("Execution Time (seconds)")
plt.grid(True)
plt.tight_layout()
plt.savefig("performance_analysis.png", dpi=150)
plt.show()

print("📈 Performance chart saved as performance_analysis.png")
print("✅ Performance analysis complete.")
