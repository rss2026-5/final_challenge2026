"""
Benchmark: KDTree RRT* vs brute-force RRT*

Runs both planners on the real stata_basement map across multiple
(start, goal) pairs and reports planning time, iterations used,
tree size, and path cost.

Usage:
    python3 benchmark_rrt.py
"""

import time
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import KDTree
from PIL import Image
import yaml
import os

# ── Map loading ────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_YAML   = os.path.join(SCRIPT_DIR, "maps/stata_basement.yaml")

def load_map(yaml_path):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), cfg["image"])
    img = np.array(Image.open(img_path).convert("L"))
    res = cfg["resolution"]
    ox, oy = cfg["origin"][0], cfg["origin"][1]
    occupied_thresh = cfg.get("occupied_thresh", 0.65)
    negate = cfg.get("negate", 0)
    img_f = img.astype(float) / 255.0
    if negate:
        img_f = 1.0 - img_f
    # ROS convention: low probability = free, high = occupied
    obstacle_mask = img_f < (1.0 - occupied_thresh)
    return obstacle_mask, res, ox, oy

# ── Shared geometry helpers ────────────────────────────────────────────────────

def make_helpers(dilated, res, width, height):
    def is_free(u, v):
        if u < 0 or v < 0 or u >= width or v >= height:
            return False
        return not dilated[v, u]

    def dist(a, b):
        return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

    def connectible(m, n):
        N = max(abs(n[0]-m[0]), abs(n[1]-m[1]))
        if N == 0:
            return True
        ts = np.linspace(0, 1, N + 1)
        us = np.round(ts*n[0] + (1-ts)*m[0]).astype(int)
        vs = np.round(ts*n[1] + (1-ts)*m[1]).astype(int)
        in_bounds = (us >= 0) & (vs >= 0) & (us < width) & (vs < height)
        if not in_bounds.all():
            return False
        return not dilated[vs, us].any()

    return is_free, dist, connectible

# ── RRT* variants ──────────────────────────────────────────────────────────────

class RRTNode:
    __slots__ = ["pos", "parent", "cost"]
    def __init__(self, pos, parent=None, cost=0.0):
        self.pos    = pos
        self.parent = parent
        self.cost   = cost


def rrt_star_kdtree(start_px, goal_px, dilated, width, height,
                    max_iter=5000, step_size=40, goal_bias=0.15,
                    goal_tol=25, rewire_radius=60, rebuild_every=50):
    is_free, dist, connectible = make_helpers(dilated, None, width, height)

    start_node = RRTNode(pos=start_px, cost=0.0)
    tree = [start_node]
    positions = [list(start_px)]
    kdtree = KDTree(positions)
    goal_node = None
    iters_used = 0

    for i in range(max_iter):
        iters_used += 1
        sample = goal_px if np.random.random() < goal_bias else (
            np.random.randint(0, width-1), np.random.randint(0, height-1))
        if not is_free(*sample):
            continue

        if i % rebuild_every == 0:
            kdtree = KDTree(positions)

        _, idx = kdtree.query(sample)
        nearest = tree[idx]

        delta = (sample[0]-nearest.pos[0], sample[1]-nearest.pos[1])
        d = dist(delta, (0, 0))
        next_pos = sample if d <= step_size else (
            round(nearest.pos[0] + delta[0]/d*step_size),
            round(nearest.pos[1] + delta[1]/d*step_size))

        if not is_free(*next_pos) or not connectible(nearest.pos, next_pos):
            continue

        neighbor_idxs = kdtree.query_ball_point(next_pos, rewire_radius)
        neighbors = [tree[j] for j in neighbor_idxs if connectible(tree[j].pos, next_pos)]

        best_parent = nearest
        best_cost   = nearest.cost + dist(nearest.pos, next_pos)
        for n in neighbors:
            c = n.cost + dist(n.pos, next_pos)
            if c < best_cost:
                best_parent, best_cost = n, c

        new_node = RRTNode(pos=next_pos, parent=best_parent, cost=best_cost)
        tree.append(new_node)
        positions.append(list(next_pos))

        for n in neighbors:
            nc = new_node.cost + dist(next_pos, n.pos)
            if nc < n.cost:
                n.parent, n.cost = new_node, nc

        if dist(next_pos, goal_px) < goal_tol and connectible(next_pos, goal_px):
            gc = new_node.cost + dist(next_pos, goal_px)
            if goal_node is None or gc < goal_node.cost:
                goal_node = RRTNode(pos=goal_px, parent=new_node, cost=gc)
            break

    path_len = None
    if goal_node is not None:
        path_len = goal_node.cost
    return goal_node is not None, iters_used, len(tree), path_len


def rrt_star_kdtree_lazy(start_px, goal_px, dilated, width, height,
                         max_iter=5000, step_size=40, goal_bias=0.15,
                         goal_tol=25, rewire_radius=60, rebuild_every=1000):
    """KDTree variant that rebuilds only every 1000 iterations."""
    return rrt_star_kdtree(
        start_px, goal_px, dilated, width, height,
        max_iter=max_iter, step_size=step_size, goal_bias=goal_bias,
        goal_tol=goal_tol, rewire_radius=rewire_radius,
        rebuild_every=rebuild_every)


def rrt_star_brute(start_px, goal_px, dilated, width, height,
                   max_iter=5000, step_size=40, goal_bias=0.15,
                   goal_tol=25, rewire_radius=60):
    is_free, dist, connectible = make_helpers(dilated, None, width, height)

    start_node = RRTNode(pos=start_px, cost=0.0)
    tree = [start_node]
    positions = np.array([list(start_px)], dtype=float)
    goal_node = None
    iters_used = 0

    for i in range(max_iter):
        iters_used += 1
        sample = goal_px if np.random.random() < goal_bias else (
            np.random.randint(0, width-1), np.random.randint(0, height-1))
        if not is_free(*sample):
            continue

        # brute-force nearest neighbor
        dists_to_sample = np.linalg.norm(positions - np.array(sample), axis=1)
        idx = int(np.argmin(dists_to_sample))
        nearest = tree[idx]

        delta = (sample[0]-nearest.pos[0], sample[1]-nearest.pos[1])
        d = dist(delta, (0, 0))
        next_pos = sample if d <= step_size else (
            round(nearest.pos[0] + delta[0]/d*step_size),
            round(nearest.pos[1] + delta[1]/d*step_size))

        if not is_free(*next_pos) or not connectible(nearest.pos, next_pos):
            continue

        # brute-force rewire neighborhood
        dists_to_new = np.linalg.norm(positions - np.array(next_pos), axis=1)
        neighbor_idxs = np.where(dists_to_new <= rewire_radius)[0]
        neighbors = [tree[j] for j in neighbor_idxs if connectible(tree[j].pos, next_pos)]

        best_parent = nearest
        best_cost   = nearest.cost + dist(nearest.pos, next_pos)
        for n in neighbors:
            c = n.cost + dist(n.pos, next_pos)
            if c < best_cost:
                best_parent, best_cost = n, c

        new_node = RRTNode(pos=next_pos, parent=best_parent, cost=best_cost)
        tree.append(new_node)
        positions = np.vstack([positions, list(next_pos)])

        for n in neighbors:
            nc = new_node.cost + dist(next_pos, n.pos)
            if nc < n.cost:
                n.parent, n.cost = new_node, nc

        if dist(next_pos, goal_px) < goal_tol and connectible(next_pos, goal_px):
            gc = new_node.cost + dist(next_pos, goal_px)
            if goal_node is None or gc < goal_node.cost:
                goal_node = RRTNode(pos=goal_px, parent=new_node, cost=gc)
            break

    path_len = goal_node.cost if goal_node is not None else None
    return goal_node is not None, iters_used, len(tree), path_len


# ── Benchmark runner ───────────────────────────────────────────────────────────

def world_to_pixel(x, y, ox, oy, res):
    u = int((x - ox) / res)
    v = int((y - oy) / res)
    return (u, v)

# (start_world, goal_world) pairs using real stata_basement free cells
SCENARIOS = [
    # short:  ~20m apart
    ((44.5, 58.2), (55.4, 53.9)),
    # medium: ~50m apart
    ((32.6, 64.7), (79.2, 105.1)),
    # long:   ~80m apart
    ((26.6, 91.4), (106.8, 53.5)),
]

MAX_ITER    = 50000
TRIALS      = 3
DILATION    = 10


def run_scenario(label, start_px, goal_px, dilated, width, height):
    results = {"KDTree(50)": [], "KDTree(1000)": [], "Brute-force": []}

    for _ in range(TRIALS):
        np.random.seed(None)

        t0 = time.perf_counter()
        found, iters, size, cost = rrt_star_kdtree(
            start_px, goal_px, dilated, width, height, max_iter=MAX_ITER)
        results["KDTree(50)"].append(
            (time.perf_counter()-t0, iters, size, cost if found else float("nan")))

        t0 = time.perf_counter()
        found, iters, size, cost = rrt_star_kdtree_lazy(
            start_px, goal_px, dilated, width, height, max_iter=MAX_ITER)
        results["KDTree(1000)"].append(
            (time.perf_counter()-t0, iters, size, cost if found else float("nan")))

        t0 = time.perf_counter()
        found, iters, size, cost = rrt_star_brute(
            start_px, goal_px, dilated, width, height, max_iter=MAX_ITER)
        results["Brute-force"].append(
            (time.perf_counter()-t0, iters, size, cost if found else float("nan")))

    def fmt(vals, fmt_str=".2f"):
        arr = np.array(vals, dtype=float)
        return f"{np.nanmean(arr):{fmt_str}} ± {np.nanstd(arr):{fmt_str}}"

    col = 20
    print(f"\n{'─'*75}")
    print(f"Scenario: {label}  (start={start_px}, goal={goal_px})")
    print(f"{'─'*75}")
    print(f"{'Metric':<22} {'KDTree(50)':>{col}} {'KDTree(1000)':>{col}} {'Brute-force':>{col}}")
    print(f"{'─'*75}")
    for metric, idx, fstr in [("Time (s)", 0, ".2f"), ("Iterations", 1, ".0f"),
                               ("Tree size", 2, ".0f"), ("Path cost (px)", 3, ".1f")]:
        row = {k: fmt([r[idx] for r in v], fstr) for k, v in results.items()}
        print(f"{metric:<22} {row['KDTree(50)']:>{col}} {row['KDTree(1000)']:>{col}} {row['Brute-force']:>{col}}")

    base = np.nanmean([r[0] for r in results["Brute-force"]])
    for name in ["KDTree(50)", "KDTree(1000)"]:
        mean_t = np.nanmean([r[0] for r in results[name]])
        print(f"\n  {name} speedup vs brute-force: {base/mean_t:.2f}x")


def main():
    print("Loading map...")
    obstacle_mask, res, ox, oy = load_map(MAP_YAML)
    height, width = obstacle_mask.shape
    dilated = binary_dilation(obstacle_mask, iterations=DILATION)
    print(f"Map: {width}x{height} px, res={res}m/px")
    print(f"Free cells: {np.sum(~dilated)} / {width*height}")
    print(f"\nRunning {TRIALS} trials per scenario, max_iter={MAX_ITER}")

    for i, (start_w, goal_w) in enumerate(SCENARIOS):
        start_px = world_to_pixel(*start_w, ox, oy, res)
        goal_px  = world_to_pixel(*goal_w,  ox, oy, res)

        def is_free(u, v):
            return 0 <= u < width and 0 <= v < height and not dilated[v, u]

        if not is_free(*start_px) or not is_free(*goal_px):
            print(f"\nScenario {i+1}: start or goal in obstacle — skipping "
                  f"(start={start_px} free={is_free(*start_px)}, "
                  f"goal={goal_px} free={is_free(*goal_px)})")
            continue

        run_scenario(f"#{i+1}", start_px, goal_px, dilated, width, height)

    print(f"\n{'═'*60}")
    print("Done.")


if __name__ == "__main__":
    main()
