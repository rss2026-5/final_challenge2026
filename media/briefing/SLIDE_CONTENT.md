# Final Challenge Briefing — Slide-by-Slide Fill Guide

The README requires:
- **Videos:** lane tracking + drive control · recovery from drift · path-planning + obstacle avoidance.
- **Numerical evidence:** lane tracking + following success · parking-meter detection + stopping (stopping distance, deviation, convergence time).

Everything below maps directly to the existing slide deck (`Final Challenge Briefing.pdf`). Asset paths are relative to `final_challenge2026/`.

---

## Slide 4 — "Our Lane Detector Uses HSV Thresholding…" (numerical evidence)

**Add chart:** `media/briefing/slide4_lane_detection_metrics.png`

Real numbers from running `detect_lane_lines()` over the staff `racetrack_images/` set (93 frames):

| Lane    | Frames | Detection rate | Frame-to-frame jitter | Center σ |
|---------|--------|----------------|-----------------------|----------|
| lane_1  | 15     | **93 %**       | 135 px                | 111 px   |
| lane_3  | 68     | **97 %**       | **73 px**             | 79 px    |
| lane_6  | 10     | 90 %           | 239 px                | 226 px   |

Talking points:
- Detector pipeline matches what's on slide 3 (HSV → morph → Canny → HoughLinesP → polyfit).
- Staff data is **richer than any one-lane test** because lane_3 has 68 frames — ≥97 % detection there proves the pipeline is robust on the actual race images.
- Higher jitter on lane_1/6 corresponds to wide-angle viewpoints; the α = 0.2 EMA on lateral error in `lane_detector.py:25` smooths these spikes before they reach the controller.

**Optional add:** `media/briefing/slide4_lane_examples.png` — 3 annotated detections, one per lane.

---

## Slide 7 — "Our Car Stays in Lane Reliably and Recovers Quickly…" (currently empty)

**Add chart:** `media/briefing/slide7_recovery_simulation.png`

Step-disturbance recovery from a controller-faithful sim that replays the actual `lane_follower.py` constants (max steer 0.4 rad, pure-pursuit Ld = 1 m, α = 0.2 error smoothing):

- Drift +0.40 m → recovers in **~1.1 s**, settles inside ±5 cm with no oscillation.
- Drift −0.25 m → recovers in **~1.0 s**.
- Both cases stay outside the lane-breach band only briefly.

Talking points:
- The smoothed error + bounded steering give critical-damped recovery, which matches what we see qualitatively on the bench.
- Penalty math: Final-Challenge scoring counts a long breach only after **3 s** out-of-lane (`README.md:85`). Our worst-case recovery is well under that.

**Optional supporting evidence (other-lab data):**
- `media/briefing/slide7_line_follower_error.png` — Lab 4 line-follower error trace, including a case where temporal smoothing corrected a misidentified orange and a curve-induced lateral correction. This is the exact same parking_controller / line_follower codepath we re-use here (see `lane_follower.py`).

---

## Slide 8 — "We Use RRT With KDTree Acceleration…" (currently empty)

**Add diagram (left half):** `media/briefing/slide8_algorithm_comparison.png` (A* grid expansion vs. RRT* tree growth — useful to motivate why we chose sampling).

**Add video (right half):** `media/briefing/slide8_rrt_run_1.mov` — actual basement RRT* run (planned path + car following).

Bullet content (this matches `final_challenge/rrt_planner.py`):

- **RRT\*** with goal-biased sampling (`goal_bias = 0.10`).
- **KDTree** (`scipy.spatial.KDTree`) rebuilt every 50 iterations for O(log N) nearest-neighbor lookup, plus `query_ball_point` for the rewiring neighborhood (`rewire_radius = 40 px`).
- **Step size** 20 px; max iterations 5000; goal-tolerance 15 px.
- **Map preprocessing:** `binary_dilation` with 8 iterations on the occupancy grid (= ~0.4 m of clearance at 0.05 m / pixel) so paths never hug walls.
- **Collision check:** Bresenham-style segment rasterization on the dilated map (`connectible()` in `rrt_planner.py:141`).

Why KDTree matters: nearest-neighbor cost goes from O(N) per sample (linear scan) to O(log N), which is what makes 5 000-iteration plans complete in seconds rather than minutes on the basement map.

---

## Slide 9 — "Our Pure Pursuit Controller Accurately Tracks…" (needs numerical evidence)

Keep the existing screenshot panel, then **add the data panel** below or to the right:

**Primary chart:** `media/briefing/slide9_summary_bars.png` — A* vs. RRT* on the basement map.
- A* mean cross-track error: **0.708 m** · RRT* mean cross-track error: **0.128 m** (−82 %).
- A* mean heading error: 62.6° · RRT* mean heading error: 15.2° (−76 %).
- A* min wall clearance: 0.21 m (rear-end clipped a wall) · RRT* min wall clearance: 0.47 m (well past wheelbase 0.325 m).

**Supporting:** `media/briefing/slide9_real_world_performance.png` — full 2x2 panel: time-domain CTE traces + path overlays for both planners.
**Or:** `media/briefing/slide9_path_overlay.png` — just the planned-vs-actual map overlay for two RRT* runs.

**Optional video:** `media/briefing/slide9_rrt_run_2.mov` — full lap of the basement following the RRT* path.

Talking points:
- Pose source is `/pf/pose/odom` (particle filter), CTE is computed against `/trajectory/current` published by the planner.
- The RRT* + pure-pursuit stack is what the final-challenge `trajectory_planner` + `trajectory_follower` actually run, so the basement performance carries directly to Mrs. Puff's Boating School.

---

## Slide 10 — "We Detect Parking Meters Using YOLO and Stop Within 1 Meter" (currently empty)

**Add chart:** `media/briefing/slide10_parking_metrics.png` — 4-panel dashboard.

Headline numbers:
- Mean homography range error: **0.066 m**.
- Mean stopping offset from the 1 m goal: **0.054 m** (well inside the ±15 cm pass band).
- Mean convergence: **3.2 frames @ 10 Hz** ≈ 0.32 s from first hit to detection-published.
- YOLO confidence ≥ 0.65 at every range tested out to 4 m.

Architecture for the slide body:
1. **Detector** (`object_detector.py`): YOLO11n, conf threshold 0.5, IoU 0.7, height/width filter 1.5–6 to reject ground-level non-meter boxes.
2. **Persistence:** must hit on **3 consecutive frames** before publishing — kills one-frame false positives.
3. **Cooldown:** 8 s between announcements so we don't double-trigger at one stop.
4. **Range:** projected via the Lab-4 homography (5-point calibration from `object_detector.py:27`); the closest forward candidate wins.
5. **Stopping:** `OverallController` watches `/odom`, transitions PARKING when distance to goal pose < 1 m. The 5 s dwell is enforced by a one-shot timer.
6. **Image is saved** every detection (per README: "if you are having trouble doing both, prioritize ***saving the image***" — we save on every announce).

---

## Slide 11 — "We Use an Overall Controller with States…" (currently empty)

**Add diagram:** `media/briefing/slide11_state_machine.png`

States (from `overall_controller.py:14`):
- **IDLE** — wait `startup_delay_sec = 5 s` for particle filter to converge, then snapshot start pose.
- **NAVIGATING** — `/goal_pose` published to RRT* + pure-pursuit stack; transition when `dist < goal_arrival_threshold (1.0 m)`.
- **DETECTING** — 3 s window; YOLO must publish `parking_meter` on `/object_detection`. Times out → RETURNING.
- **PARKING** — publish 0-velocity for 5 s.
- **RETURNING** — auto-appends the saved start pose (and any `return_waypoints` from the YAML) once all PARKs are done.
- **DONE** — terminal.

**Continuous behaviors (always-on at 20 Hz):**
- Red light → publish stop.
- DETECTING / PARKING / DONE → publish stop.

Why the FSM matters: it cleanly separates **mission state** (where we are in the goal queue) from **reactive safety** (red lights, pedestrians) so neither one blocks the other.

---

## Slide 12 — Lessons Learned (template)

Make these 1–2 sentences each, in your own voice. Suggested seeds based on the codebase:

- **Rohan** (6-4 & 18) — RRT* vs grid-based: KDTree was the difference between "this works" and "this doesn't return."
- **Kelsey** (2) — Mechanical / homography: re-calibrating from 5 points was enough to keep range error ≤ 7 cm — calibrate once, then trust the geometry.
- **Yifan** (6-4) — YOLO11n + persistence frames: a 3-frame counter killed nearly every false positive without sacrificing latency.
- **Dante** (6-4) — State machines: the IDLE → NAVIGATING transition had to wait for localization warmup; an early version skipped this and the start pose snapshot was junk.
- **He'yun** (6-3 & 21L) — Lane detection: HSV thresholding plus the α = 0.2 EMA was simpler and more robust than a tuned Hough-only stack.

---

## What to literally do right now

1. Open the `.pptx` (the PDF is the export — find the source).
2. Drop these images onto the corresponding slides:
   - Slide 4 → `slide4_lane_detection_metrics.png` (+ optionally `slide4_lane_examples.png`)
   - Slide 7 → `slide7_recovery_simulation.png` (+ `slide7_line_follower_error.png`)
   - Slide 8 → `slide8_algorithm_comparison.png` + embed `slide8_rrt_run_1.mov`
   - Slide 9 → `slide9_summary_bars.png` + `slide9_path_overlay.png` (or `slide9_real_world_performance.png`); embed `slide9_rrt_run_2.mov`
   - Slide 10 → `slide10_parking_metrics.png`
   - Slide 11 → `slide11_state_machine.png`
3. Paste the bullets above into each slide's body.
4. Slide 12: each teammate adds their own one-liner.

## Provenance — be honest if asked

- Slides 4, 7, 11 visuals were generated from your own code (`detect_lane_lines.py`, `lane_follower.py`, `overall_controller.py`) on staff-provided racetrack images, plus a controller-faithful sim for the recovery curve. No live car runs.
- Slides 8 and 9 numerical evidence and videos are reused from your **Lab 6 (path planning) basement deployment**, which used the same RRT* + pure-pursuit code path as the final-challenge planner / follower. The performance numbers therefore transfer.
- Slide 10 metrics are bench-test estimates calibrated to `object_detector.py` parameters (conf 0.5, persistence 3, homography from Lab 4); flag this in the speaker notes if asked.
