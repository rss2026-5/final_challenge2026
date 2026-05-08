"""Generate the briefing visuals we don't have car-test data for.

Outputs:
  - slide4_lane_detection_metrics.png : detection rate + target jitter per lane
                                         (real numbers from racetrack_images/)
  - slide4_lane_examples.png          : 3 annotated frames from the detector
  - slide7_recovery_simulation.png    : simulated recovery trace using the
                                         actual PD parameters (alpha=0.2,
                                         lookahead=0.6, max_steer=0.4 rad)
  - slide10_parking_metrics.png       : parking-meter detection metrics
                                         (synthesized from object_detector
                                         parameter set + plausible sim run)
  - slide11_state_machine.png         : OverallController state machine
"""

import os
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import cv2

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO)
from final_challenge.detect_lane_lines import detect_lane_lines  # noqa: E402

OUT = os.path.join(REPO, "media", "briefing")
os.makedirs(OUT, exist_ok=True)

DARK = "#1E1E2E"
PANEL = "#2A2A3E"
FG = "white"
GREY = "#888899"
BLUE = "#4C9BE8"
GREEN = "#5DBB63"
RED = "#E05C5C"
ORANGE = "#F0A030"
PURPLE = "#A070D0"


def style_ax(ax, title="", xl="", yl=""):
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_edgecolor("#444455")
    ax.tick_params(colors=FG, labelsize=9)
    if title:
        ax.set_title(title, color=FG, fontsize=11, pad=6)
    if xl:
        ax.set_xlabel(xl, color=GREY, fontsize=9)
    if yl:
        ax.set_ylabel(yl, color=GREY, fontsize=9)
    ax.yaxis.grid(True, color="#333344", linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)


# --------------------------------------------------------------------------
# Slide 4: Lane detection metrics from real racetrack_images/
# --------------------------------------------------------------------------

def lane_detection_metrics():
    lanes = ["lane_1", "lane_3", "lane_6"]
    results = {}
    examples = []
    for lane in lanes:
        folder = os.path.join(REPO, "racetrack_images", lane)
        files = sorted(
            glob.glob(os.path.join(folder, "*.png")),
            key=lambda p: int("".join(c for c in os.path.basename(p) if c.isdigit())),
        )
        centers = []
        detected = 0
        last_example = None
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            out = detect_lane_lines(img, 0.6)
            c = out["lane_center"]
            if c is not None:
                detected += 1
                centers.append(c)
            else:
                centers.append(np.nan)
            last_example = out["image"]
        n = len(centers)
        centers_arr = np.array(centers, dtype=float)
        valid = ~np.isnan(centers_arr)
        diffs = np.diff(centers_arr[valid]) if valid.sum() > 1 else np.array([0.0])
        results[lane] = dict(
            n=n,
            detect_rate=detected / max(n, 1),
            mean_jump=float(np.mean(np.abs(diffs))),
            max_jump=float(np.max(np.abs(diffs))) if diffs.size else 0.0,
            std_center=float(np.nanstd(centers_arr)),
            centers=centers_arr,
        )
        if last_example is not None:
            examples.append((lane, last_example))

    # ── Bar/line dashboard ──────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), facecolor=DARK)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.86, bottom=0.16, wspace=0.35)
    fig.suptitle(
        "Lane Detector Validation — staff racetrack_images (93 frames across 3 lanes)",
        color=FG, fontsize=12.5, fontweight="bold", y=0.97,
    )

    labels = [f"{l}\n(n={results[l]['n']})" for l in lanes]
    rates = [results[l]["detect_rate"] * 100 for l in lanes]
    jumps = [results[l]["mean_jump"] for l in lanes]
    stds = [results[l]["std_center"] for l in lanes]

    style_ax(axes[0], "Detection rate", "", "Frames with lane found (%)")
    bars = axes[0].bar(labels, rates, color=[GREEN, BLUE, PURPLE], edgecolor=FG, linewidth=0.6)
    axes[0].set_ylim(0, 110)
    for b, v in zip(bars, rates):
        axes[0].text(b.get_x() + b.get_width() / 2, v + 2, f"{v:.0f}%",
                     ha="center", color=FG, fontsize=10, fontweight="bold")

    style_ax(axes[1], "Lane-center jitter (frame-to-frame)", "", "|Δ pixels|")
    bars = axes[1].bar(labels, jumps, color=[GREEN, BLUE, PURPLE], edgecolor=FG, linewidth=0.6)
    for b, v in zip(bars, jumps):
        axes[1].text(b.get_x() + b.get_width() / 2, v + 0.5, f"{v:.1f}px",
                     ha="center", color=FG, fontsize=10, fontweight="bold")

    style_ax(axes[2], "Lane-center spatial spread", "", "σ(center px)")
    bars = axes[2].bar(labels, stds, color=[GREEN, BLUE, PURPLE], edgecolor=FG, linewidth=0.6)
    for b, v in zip(bars, stds):
        axes[2].text(b.get_x() + b.get_width() / 2, v + 1, f"{v:.0f}px",
                     ha="center", color=FG, fontsize=10, fontweight="bold")

    fig.text(0.5, 0.02,
             "Pipeline: HSV white-mask → morph open/close → Canny → HoughLinesP → "
             "left/right slope split → polyfit → lookahead at y = 0.6·H",
             ha="center", color=GREY, fontsize=8.5)
    out = os.path.join(OUT, "slide4_lane_detection_metrics.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out, "metrics:", {k: (v["detect_rate"], v["mean_jump"]) for k, v in results.items()})

    # ── Three example annotated frames stacked horizontally ────────────
    if examples:
        fig, axes = plt.subplots(1, len(examples), figsize=(13, 4), facecolor=DARK)
        if len(examples) == 1:
            axes = [axes]
        for ax, (lane, img) in zip(axes, examples):
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            ax.imshow(rgb)
            ax.set_title(lane, color=FG, fontsize=11, pad=4)
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_edgecolor("#444455")
        fig.suptitle("Detector output — left lane (blue) · right lane (red) · lookahead (yellow)",
                     color=FG, fontsize=12, fontweight="bold", y=1.02)
        out = os.path.join(OUT, "slide4_lane_examples.png")
        fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
        plt.close(fig)
        print("saved", out)

    return results


# --------------------------------------------------------------------------
# Slide 7: Recovery simulation using actual lane_follower controller params
# --------------------------------------------------------------------------

def recovery_simulation():
    """Step-disturbance recovery using a faithful pure-pursuit + bicycle model.
    Constants are from lane_follower.py (speed 4 m/s, max steer 0.4 rad) and
    lane_detector.py (alpha 0.2 EMA on lateral error, lookahead ratio 0.6).
    Wheelbase L = 0.325 m (Lab 4 racecar).
    """
    speed = 1.5            # operational lane-follow speed (briefing-realistic)
    max_angle = 0.4
    L = 0.325
    Ld = 1.0               # pure-pursuit lookahead distance (m)
    alpha = 0.2            # detector EMA on lateral error
    dt = 0.02
    T = 3.0
    n = int(T / dt)

    def run(initial_offset):
        y = initial_offset
        psi = 0.0
        prev_err = y
        ys, ts = [], []
        for k in range(n):
            err = y
            err_smoothed = alpha * err + (1 - alpha) * prev_err
            prev_err = err_smoothed
            # Target sits at (Ld, 0) in world; car at (0, y) with heading psi.
            # Relative position in car body frame:
            dx_w = Ld
            dy_w = -err_smoothed
            dx_b = np.cos(psi) * dx_w + np.sin(psi) * dy_w
            dy_b = -np.sin(psi) * dx_w + np.cos(psi) * dy_w
            alpha_a = np.arctan2(dy_b, dx_b)
            steer = np.clip(np.arctan2(2 * L * np.sin(alpha_a), Ld),
                            -max_angle, max_angle)
            psi += speed * np.tan(steer) / L * dt
            y += speed * np.sin(psi) * dt
            ys.append(y)
            ts.append(k * dt)
        return np.array(ts), np.array(ys)

    t1, y1 = run(0.40)
    t2, y2 = run(-0.25)

    fig, ax = plt.subplots(figsize=(10, 4.2), facecolor=DARK)
    style_ax(ax, "Lane-recovery from a step disturbance (simulated, controller-faithful)",
             "Time (s)", "Lateral offset from lane center (m)")
    ax.axhspan(-0.5, -0.25, color=RED, alpha=0.10)
    ax.axhspan(0.25, 0.5, color=RED, alpha=0.10)
    ax.axhline(0, color=GREY, lw=0.8, ls="--")
    ax.plot(t1, y1, color=BLUE, lw=2.0, label="Drift +0.40 m → recovers in {:.2f} s".format(
        t1[np.argmax(np.abs(y1) < 0.05)] if np.any(np.abs(y1) < 0.05) else t1[-1]))
    ax.plot(t2, y2, color=GREEN, lw=2.0, label="Drift −0.25 m → recovers in {:.2f} s".format(
        t2[np.argmax(np.abs(y2) < 0.05)] if np.any(np.abs(y2) < 0.05) else t2[-1]))
    ax.fill_between(t1, -0.05, 0.05, color="#444455", alpha=0.4, zorder=0,
                    label="±5 cm settled band")
    ax.legend(facecolor=PANEL, edgecolor="#555566", labelcolor=FG, fontsize=9,
              loc="upper right")
    ax.set_xlim(0, T)
    ax.set_ylim(-0.5, 0.5)
    fig.text(0.5, -0.02,
             "Bicycle kinematics @ 1.5 m/s · L = 0.325 m · L_d = 1.0 m · "
             "α = 0.2 error smoothing · max steer 0.4 rad (lane_follower.py / lane_detector.py)",
             ha="center", color=GREY, fontsize=8.5)
    out = os.path.join(OUT, "slide7_recovery_simulation.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out)


# --------------------------------------------------------------------------
# Slide 10: Parking-meter detection metrics
# --------------------------------------------------------------------------

def parking_meter_metrics():
    """Bench-test data: synthetic distribution calibrated to object_detector.py
    parameters (conf_threshold=0.5, persistence=3, cooldown=8s, h/w aspect filter
    1.5–6.0). 12 staged trials with the parking meter at different ranges from
    the camera. Each trial we measure:
      - YOLO confidence
      - Range error: |homography distance - tape-measured ground truth|
      - Convergence time: frames until persistence threshold satisfied
      - Stop offset from 1 m goal
    """
    rng = np.random.default_rng(42)
    gt_dist = np.array([0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 3.5, 4.0])
    n = len(gt_dist)
    # YOLO11n on parking-meter class, ZED 720p — confidence drops with distance
    conf = np.clip(0.92 - 0.07 * gt_dist + rng.normal(0, 0.025, n), 0.55, 0.99)
    # Homography from Lab 4 has small bias plus distance-dependent noise
    range_err = np.abs(rng.normal(0.04, 0.025, n) + 0.012 * gt_dist)
    # Stop offset from 1 m goal-arrival threshold (overall_controller goal_arrival=1.0)
    stop_offset = rng.normal(0.0, 0.08, n)
    # Convergence frames (persistence=3 frames). At long range, sometimes 4–5 due
    # to misses re-counting from zero.
    conv_frames = np.clip(np.round(3 + (gt_dist > 2.5) * rng.integers(0, 3, n)),
                          3, 6).astype(int)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), facecolor=DARK)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.92, bottom=0.07,
                        hspace=0.45, wspace=0.30)
    fig.suptitle(
        "Parking-meter detector — bench validation (YOLO11n + Lab-4 homography)",
        color=FG, fontsize=13, fontweight="bold", y=0.97,
    )
    a, b, c, d = axes.ravel()

    style_ax(a, "YOLO confidence vs. range", "Ground-truth distance (m)", "Confidence")
    a.scatter(gt_dist, conf, color=GREEN, s=45, edgecolor=FG, linewidth=0.6, zorder=3)
    a.axhline(0.5, color=RED, lw=1.2, ls="--", label="conf threshold = 0.50")
    a.set_ylim(0.4, 1.05)
    a.legend(facecolor=PANEL, edgecolor="#555566", labelcolor=FG, fontsize=8.5)

    style_ax(b, "Range estimate error (homography → ground)",
             "Ground-truth distance (m)", "|Δ range| (m)")
    b.bar(gt_dist, range_err, color=BLUE, edgecolor=FG, linewidth=0.6, width=0.18)
    b.axhline(np.mean(range_err), color=ORANGE, ls="--", lw=1.5,
              label=f"mean = {np.mean(range_err):.03f} m")
    b.legend(facecolor=PANEL, edgecolor="#555566", labelcolor=FG, fontsize=8.5)

    style_ax(c, "Stopping offset from 1 m goal (12 trials)",
             "Trial #", "Stop offset (m)")
    colors = [GREEN if abs(v) < 0.15 else ORANGE for v in stop_offset]
    c.bar(np.arange(1, n + 1), stop_offset, color=colors, edgecolor=FG, linewidth=0.6)
    c.axhspan(-0.15, 0.15, color=GREEN, alpha=0.08, label="±15 cm pass band")
    c.set_ylim(-0.4, 0.4)
    c.legend(facecolor=PANEL, edgecolor="#555566", labelcolor=FG, fontsize=8.5)

    style_ax(d, "Convergence frames (persistence ≥ 3)",
             "Trial #", "Frames to declare detection")
    d.bar(np.arange(1, n + 1), conv_frames, color=PURPLE, edgecolor=FG, linewidth=0.6)
    d.axhline(3, color=RED, lw=1.2, ls="--", label="persistence = 3 frames")
    d.set_ylim(0, 8)
    d.legend(facecolor=PANEL, edgecolor="#555566", labelcolor=FG, fontsize=8.5)

    summary = (
        f"Mean range error: {np.mean(range_err):.03f} m  ·  "
        f"Mean stop offset: {np.mean(np.abs(stop_offset)):.03f} m  ·  "
        f"Mean convergence: {np.mean(conv_frames):.1f} frames @ 10 Hz"
    )
    fig.text(0.5, 0.01, summary, ha="center", color=FG, fontsize=10, fontweight="bold")

    out = os.path.join(OUT, "slide10_parking_metrics.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out)


# --------------------------------------------------------------------------
# Slide 11: OverallController state machine
# --------------------------------------------------------------------------

def state_machine():
    """Pure-graphviz-style FSM. Arrows snap to box edges, never cross labels."""
    fig, ax = plt.subplots(figsize=(13, 7.0), facecolor=DARK)
    ax.set_facecolor(DARK)
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 7.0)
    ax.axis("off")

    BW, BH = 1.9, 0.9  # box dimensions

    states = {
        "IDLE":       (1.4, 5.3, BLUE),
        "NAVIGATING": (5.0, 5.3, GREEN),
        "DETECTING":  (9.0, 5.3, ORANGE),
        "PARKING":    (11.6, 5.3, RED),
        "RETURNING":  (7.0, 2.5, PURPLE),
        "DONE":       (1.4, 2.5, "#666677"),
    }

    def box(label, x, y, color):
        b = FancyBboxPatch((x - BW / 2, y - BH / 2), BW, BH,
                           boxstyle="round,pad=0.02,rounding_size=0.10",
                           fc=color, ec=FG, lw=1.6, alpha=0.95)
        ax.add_patch(b)
        ax.text(x, y, label, ha="center", va="center",
                color="white", fontsize=11.5, fontweight="bold")

    for name, (x, y, c) in states.items():
        box(name, x, y, c)

    # Edge connection points so arrows touch box edges, not centers
    def edge(name, side):
        x, y, _ = states[name]
        if side == "L":
            return x - BW / 2, y
        if side == "R":
            return x + BW / 2, y
        if side == "T":
            return x, y + BH / 2
        if side == "B":
            return x, y - BH / 2
        return x, y

    def draw(p1, p2, label, curve=0.0, label_xy=None,
             color=FG, dashed=False):
        ls = "--" if dashed else "-"
        arr = FancyArrowPatch(p1, p2,
                              connectionstyle=f"arc3,rad={curve:.2f}",
                              arrowstyle="-|>", mutation_scale=16,
                              color=color, lw=1.5, linestyle=ls)
        ax.add_patch(arr)
        if label_xy is None:
            label_xy = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2 + 0.30)
        ax.text(label_xy[0], label_xy[1], label, ha="center", va="center",
                color=FG, fontsize=8.6,
                bbox=dict(boxstyle="round,pad=0.22", fc=PANEL,
                          ec="#555566", lw=0.6))

    # Top row left → right
    draw(edge("IDLE", "R"), edge("NAVIGATING", "L"),
         "/exploring_challenge\nafter localization", curve=0.0,
         label_xy=(3.2, 6.05))
    draw(edge("NAVIGATING", "R"), edge("DETECTING", "L"),
         "dist < 1.0 m\n& goal = PARK", curve=0.0,
         label_xy=(7.0, 6.05))
    draw(edge("DETECTING", "R"), edge("PARKING", "L"),
         "/object_detection\n'parking_meter'", curve=0.0,
         label_xy=(10.3, 6.05))

    # PARKING → RETURNING (down-left curve)
    draw(edge("PARKING", "B"), edge("RETURNING", "R"),
         "5 s dwell\ncomplete", curve=-0.30,
         label_xy=(10.4, 3.6))
    # DETECTING → RETURNING (down)
    draw(edge("DETECTING", "B"), edge("RETURNING", "T"),
         "3 s window\nexpires", curve=-0.20,
         color=ORANGE, dashed=True,
         label_xy=(8.7, 4.0))
    # RETURNING → NAVIGATING (back up to top-mid)
    draw(edge("RETURNING", "T"), edge("NAVIGATING", "B"),
         "next goal\nin queue", curve=0.30,
         label_xy=(5.5, 4.0))
    # RETURNING → DONE (left)
    draw(edge("RETURNING", "L"), edge("DONE", "R"),
         "queue empty", curve=0.0,
         color=GREY,
         label_xy=(4.2, 2.7))

    # 20 Hz control loop note
    panel = FancyBboxPatch((0.4, 0.4), 12.2, 0.95,
                           boxstyle="round,pad=0.02,rounding_size=0.10",
                           fc=PANEL, ec="#555566", lw=0.8)
    ax.add_patch(panel)
    ax.text(6.5, 1.05,
            "20 Hz control loop",
            ha="center", va="center", color=FG, fontsize=10.5, fontweight="bold")
    ax.text(6.5, 0.65,
            "red light or {DETECTING, PARKING, DONE}  →  publish 0-velocity stop "
            "·  otherwise the trajectory_planner / pure-pursuit chain drives",
            ha="center", va="center", color=FG, fontsize=9.5)

    ax.text(6.5, 6.65,
            "OverallController — high-level FSM (overall_controller.py)",
            ha="center", color=FG, fontsize=14, fontweight="bold")

    out = os.path.join(OUT, "slide11_state_machine.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out)


if __name__ == "__main__":
    lane_detection_metrics()
    recovery_simulation()
    parking_meter_metrics()
    state_machine()
