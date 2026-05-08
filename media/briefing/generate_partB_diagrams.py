"""Generate the two new Part B design diagrams.

Outputs:
  slide10_parking_pipeline.png    — replaces the fabricated metrics chart
  slide11_traffic_safety.png      — for the new traffic-light + safety slide
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Wedge, Circle, Rectangle

OUT = os.path.dirname(os.path.abspath(__file__))

DARK   = "#1E1E2E"
PANEL  = "#2A2A3E"
FG     = "white"
GREY   = "#888899"
BLUE   = "#4C9BE8"
GREEN  = "#5DBB63"
RED    = "#E05C5C"
ORANGE = "#F0A030"
PURPLE = "#A070D0"


def block(ax, x, y, w, h, label, sub, color):
    b = FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                       boxstyle="round,pad=0.02,rounding_size=0.10",
                       fc=color, ec=FG, lw=1.4, alpha=0.95)
    ax.add_patch(b)
    ax.text(x, y + 0.10, label, ha="center", va="center",
            color="white", fontsize=10.5, fontweight="bold")
    ax.text(x, y - 0.18, sub, ha="center", va="center",
            color="white", fontsize=8.3)


def harrow(ax, x1, x2, y, color=FG):
    arr = FancyArrowPatch((x1, y), (x2, y),
                          arrowstyle="-|>", mutation_scale=14,
                          color=color, lw=1.5)
    ax.add_patch(arr)


# ─────────────────────────────────────────────────────────
# Slide 10 — Parking-meter pipeline
# ─────────────────────────────────────────────────────────
def parking_pipeline():
    fig, ax = plt.subplots(figsize=(14.5, 5.0), facecolor=DARK)
    ax.set_facecolor(DARK)
    ax.set_xlim(0, 14.5)
    ax.set_ylim(0, 5.0)
    ax.axis("off")

    # 6 boxes spaced for ample text room
    BW, BH = 2.10, 0.95
    ys = 3.5
    xs = [1.30, 3.70, 6.10, 8.50, 10.90, 13.30]

    # Order matches object_detector.py code path:
    # YOLO → aspect filter → homography → closest-forward → persistence → cooldown
    blocks = [
        ("ZED RGB",          "/zed image",                  BLUE),
        ("YOLO11n",          "conf ≥ 0.5 · iou 0.7\nclass: parking meter", PURPLE),
        ("Aspect filter",    "h/w ∈ [1.5, 6]",              ORANGE),
        ("Homography",       "Lab-4, 5 points",             BLUE),
        ("Closest forward",  "argmin distance",             RED),
        ("Persistence",      "3 frames · 8 s cooldown",     GREEN),
    ]
    for x, (lbl, sub, c) in zip(xs, blocks):
        block(ax, x, ys, BW, BH, lbl, sub, c)
    for i in range(len(xs) - 1):
        harrow(ax, xs[i] + BW / 2, xs[i + 1] - BW / 2, ys)

    # Stop-logic strip (deterministic facts only)
    strip = FancyBboxPatch((0.4, 0.55), 13.7, 1.65,
                           boxstyle="round,pad=0.02,rounding_size=0.10",
                           fc=PANEL, ec="#555566", lw=0.8)
    ax.add_patch(strip)
    ax.text(7.25, 1.85, "OverallController stop logic",
            ha="center", va="center", color=FG, fontsize=11.5, fontweight="bold")
    ax.text(7.25, 1.30,
            "pose-to-goal < 1.0 m   ⇒   PARKING   ⇒   0 velocity for 5.0 s   ⇒   RETURNING",
            ha="center", va="center", color=FG, fontsize=10.5)
    ax.text(7.25, 0.85,
            "annotated frame saved to  ~/parking_captures/  on every announce",
            ha="center", va="center", color=GREY, fontsize=9.5, style="italic")

    ax.text(7.25, 4.65, "Parking-meter detection & stop pipeline",
            ha="center", color=FG, fontsize=14, fontweight="bold")

    out = os.path.join(OUT, "slide10_parking_pipeline.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out)


# ─────────────────────────────────────────────────────────
# Slide 11 (new) — Traffic light + safety controller
# ─────────────────────────────────────────────────────────
def traffic_safety():
    fig, axes = plt.subplots(1, 2, figsize=(14.0, 6.4), facecolor=DARK,
                             gridspec_kw=dict(width_ratios=[1.15, 1.0]))
    fig.suptitle("Reactive subsystems — red light + forward-cone safety",
                 color=FG, fontsize=14, fontweight="bold", y=0.97)

    # ── Left: red-light pipeline + hysteresis ──────────────────────────
    axL = axes[0]
    axL.set_facecolor(DARK)
    axL.set_xlim(0, 7.5)
    axL.set_ylim(0, 6.0)
    axL.axis("off")
    axL.text(3.75, 5.6, "Red-light detector", ha="center",
             color=FG, fontsize=12, fontweight="bold")

    BW, BH = 1.55, 0.85
    row1_y = 4.4
    row2_y = 2.65
    xs1 = [1.0, 2.85, 4.7, 6.55]
    xs2 = [1.0, 2.85, 4.7, 6.55]
    p1 = [
        ("HSV mask",   "hue ∈ [0,10] ∪\n[170,180]",     RED),
        ("ROI",        "rows 30%–90%\n(light on pedestal)",        BLUE),
        ("Morph 5×5",  "open → close",                  ORANGE),
        ("Blob filter","area ≥ 80 px\ncircularity ≥ 0.6", PURPLE),
    ]
    for x, (lbl, sub, c) in zip(xs1, p1):
        block(axL, x, row1_y, BW, BH, lbl, sub, c)
    for i in range(len(xs1) - 1):
        harrow(axL, xs1[i] + BW / 2, xs1[i + 1] - BW / 2, row1_y)

    # Hysteresis state machine: False (default, left) ⇄ True (right)
    sm_left_x  = 2.0
    sm_right_x = 5.5
    block(axL, sm_left_x,  row2_y, BW + 0.2, BH, "is_red = False",
          "publishes  False", GREEN)
    block(axL, sm_right_x, row2_y, BW + 0.2, BH, "is_red = True",
          "publishes  True", RED)

    # Down arrow from blob filter to state machine
    arr = FancyArrowPatch((xs1[-1], row1_y - BH / 2),
                          (sm_right_x, row2_y + BH / 2),
                          arrowstyle="-|>", mutation_scale=14,
                          color=FG, lw=1.5,
                          connectionstyle="arc3,rad=-0.30")
    axL.add_patch(arr)
    axL.text(6.55, 3.55, "feeds\nhit counter",
             ha="left", va="center", color=FG, fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.18", fc=PANEL, ec="#555566", lw=0.6))

    # forward arrow (False → True): top curve
    arr = FancyArrowPatch((sm_left_x  + (BW + 0.2) / 2, row2_y + 0.18),
                          (sm_right_x - (BW + 0.2) / 2, row2_y + 0.18),
                          connectionstyle="arc3,rad=-0.45",
                          arrowstyle="-|>", mutation_scale=12,
                          color=RED, lw=1.4)
    axL.add_patch(arr)
    axL.text((sm_left_x + sm_right_x) / 2, row2_y + 0.95,
             "3 consecutive\nred frames", ha="center", va="center",
             color=FG, fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.18", fc=PANEL, ec="#555566", lw=0.6))

    # back arrow (True → False): bottom curve
    arr = FancyArrowPatch((sm_right_x - (BW + 0.2) / 2, row2_y - 0.18),
                          (sm_left_x  + (BW + 0.2) / 2, row2_y - 0.18),
                          connectionstyle="arc3,rad=-0.45",
                          arrowstyle="-|>", mutation_scale=12,
                          color=GREEN, lw=1.4)
    axL.add_patch(arr)
    axL.text((sm_left_x + sm_right_x) / 2, row2_y - 0.95,
             "5 consecutive\nclear frames", ha="center", va="center",
             color=FG, fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.18", fc=PANEL, ec="#555566", lw=0.6))

    axL.text(3.75, 0.45,
             "Asymmetric hysteresis biases toward safe stops "
             "(easier to engage red than to release it)",
             ha="center", color=GREY, fontsize=9, style="italic")

    # ── Right: forward-cone safety controller ─────────────────────────
    axR = axes[1]
    axR.set_facecolor(DARK)
    axR.set_xlim(-3.0, 5.0)
    axR.set_ylim(-3.5, 4.5)
    axR.set_aspect("equal")
    axR.axis("off")
    axR.text(1.0, 4.0, "Forward-cone safety controller", ha="center",
             color=FG, fontsize=12, fontweight="bold")

    # Car body
    car = Rectangle((-0.55, -0.25), 1.10, 0.50, fc=BLUE, ec=FG, lw=1.4)
    axR.add_patch(car)
    axR.text(0, 0, "car", ha="center", va="center", color=FG, fontsize=8)

    # Cone ±22.5°
    cone = Wedge(center=(0.55, 0), r=4.2,
                 theta1=-22.5, theta2=22.5,
                 facecolor=ORANGE, alpha=0.25, edgecolor=ORANGE, lw=1.5)
    axR.add_patch(cone)
    axR.plot([0.55, 0.55 + 4.2 * np.cos(np.radians(22.5))],
             [0, 4.2 * np.sin(np.radians(22.5))], color=ORANGE, lw=1.0, ls="--")
    axR.plot([0.55, 0.55 + 4.2 * np.cos(np.radians(-22.5))],
             [0, 4.2 * np.sin(np.radians(-22.5))], color=ORANGE, lw=1.0, ls="--")
    axR.text(2.6, 1.4, "±22.5° cone\n(forward only)", color=ORANGE, fontsize=9.5,
             ha="center")

    # Range threshold line at, say, 1.5 m for illustration
    threshold_r = 1.6
    arc = Wedge(center=(0.55, 0), r=threshold_r,
                theta1=-22.5, theta2=22.5,
                facecolor=RED, alpha=0.20, edgecolor=RED, lw=1.0)
    axR.add_patch(arc)
    axR.text(1.4, -0.6, "stop zone\nd < max(0.3 m,\nv·0.5 s)",
             color=RED, fontsize=8.8, ha="center")

    # Adjacent-lane car (illustrative obstacle outside cone)
    other = Rectangle((-1.5, 1.6), 1.0, 0.45, fc=GREY, ec=FG, lw=1.0)
    axR.add_patch(other)
    axR.text(-1.0, 1.83, "adj. lane", ha="center", va="center", color=FG, fontsize=8)
    axR.annotate("ignored\n(outside cone)", xy=(-1.0, 1.5), xytext=(-2.6, 0.5),
                 color=GREY, fontsize=8.5,
                 arrowprops=dict(arrowstyle="->", color=GREY))

    axR.text(1.0, -2.7,
             "TTC = 0.5 s   ·   floor = 0.3 m\n"
             "10th-percentile range (rejects noisy returns)\n"
             "0.2 m release hysteresis",
             ha="center", color=FG, fontsize=10,
             bbox=dict(boxstyle="round,pad=0.30", fc=PANEL, ec="#555566", lw=0.8))

    out = os.path.join(OUT, "slide11_traffic_safety.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("saved", out)


if __name__ == "__main__":
    parking_pipeline()
    traffic_safety()
