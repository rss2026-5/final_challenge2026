"""Run YOLO11n on the staged parking-meter test images.

Mirrors the production YOLO + filter parameters from
final_challenge/object_detector.py:
  conf = 0.5, iou = 0.7, classes = [parking meter, fire hydrant]
  aspect filter h/w in [1.5, 6.0] applied to parking-meter candidates

NOTE: the Lab-4 homography is *not* applied here. Those calibration points
correspond to the ZED on the racecar; running them on phone photos at
unrelated viewpoints would yield meaningless ranges, so we skip distance
projection and report bbox + confidence + aspect only.

This run also reports the lowest-confidence parking-meter detection at
conf=0.05 so we know how the production threshold (0.5) is doing relative
to what YOLO actually sees.

Inputs:  media/briefing/parking_test/range_{1m,2m}.jpg
Outputs: annotated PNGs alongside, plus a console table + CSV of bbox/conf/aspect.
"""
import csv
import os

import cv2
import numpy as np
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.join(HERE, "parking_test")

CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.7
MIN_ASPECT = 1.5
MAX_ASPECT = 6.0

PARKING_METER_NAME = "parking meter"
FIRE_HYDRANT_NAME = "fire hydrant"

def annotate(frame, dets, chosen_idx):
    out = frame.copy()
    for i, d in enumerate(dets):
        x1, y1, x2, y2 = d["bbox"]
        is_chosen = i == chosen_idx
        color = (0, 255, 0) if is_chosen else (0, 165, 255)
        if d["cls_name"] == FIRE_HYDRANT_NAME:
            color = (0, 0, 255)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 4)
        label = f"{d['cls_name']} {d['conf']:.2f}"
        cv2.putText(
            out, label, (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 4
        )
    return out


def run_one(model, path, target_classes, conf_floor=CONF_THRESHOLD):
    frame = cv2.imread(path)
    if frame is None:
        raise FileNotFoundError(path)

    results = model(
        frame,
        classes=target_classes,
        conf=conf_floor,
        iou=IOU_THRESHOLD,
        verbose=False,
    )
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return frame, [], -1

    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    names = model.names

    dets = []
    for box, c, k in zip(xyxy, conf, cls):
        x1, y1, x2, y2 = box
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        aspect = bh / bw
        cls_name = names[k]
        passes_aspect = MIN_ASPECT <= aspect <= MAX_ASPECT
        passes_conf = float(c) >= CONF_THRESHOLD
        dets.append({
            "cls_name": cls_name,
            "conf": float(c),
            "bbox": (int(x1), int(y1), int(x2), int(y2)),
            "aspect": float(aspect),
            "passes_aspect": bool(passes_aspect),
            "passes_conf": bool(passes_conf),
        })

    meter_candidates = [
        i for i, d in enumerate(dets)
        if d["cls_name"] == PARKING_METER_NAME
        and d["passes_aspect"]
        and d["passes_conf"]
    ]
    chosen_idx = -1
    if meter_candidates:
        chosen_idx = max(meter_candidates, key=lambda i: dets[i]["conf"])
    return frame, dets, chosen_idx


def main():
    model = YOLO("yolo11n.pt")

    target_classes = []
    name_lookup = {v.lower(): k for k, v in model.names.items()}
    for n in (PARKING_METER_NAME, FIRE_HYDRANT_NAME):
        if n in name_lookup:
            target_classes.append(name_lookup[n])

    rows = []
    for label, fname in [("1m", "range_1m.jpg"), ("2m", "range_2m.jpg")]:
        path = os.path.join(TEST_DIR, fname)
        # Pull at conf_floor = 0.05 so we see the full underlying signal,
        # then mark which detections cleared the production threshold (0.5).
        frame, dets, chosen_idx = run_one(model, path, target_classes, conf_floor=0.05)
        annotated = annotate(frame, dets, chosen_idx)
        out_path = os.path.join(TEST_DIR, f"range_{label}_annotated.jpg")
        cv2.imwrite(out_path, annotated)
        print(f"\n=== {label}  ({fname})  → {out_path} ===")
        if not dets:
            print("  no detections")
        for i, d in enumerate(dets):
            tag = " <-- CHOSEN" if i == chosen_idx else ""
            gate = "PASS" if (d["passes_conf"] and d["passes_aspect"]) else "drop"
            print(
                f"  {d['cls_name']:14s}  conf={d['conf']:.3f}  "
                f"aspect={d['aspect']:.2f}  gate={gate}  "
                f"bbox={d['bbox']}{tag}"
            )
            rows.append({
                "range_label": label,
                "class": d["cls_name"],
                "conf": round(d["conf"], 4),
                "aspect": round(d["aspect"], 3),
                "passes_conf_0.5": d["passes_conf"],
                "passes_aspect": d["passes_aspect"],
                "bbox_x1": d["bbox"][0],
                "bbox_y1": d["bbox"][1],
                "bbox_x2": d["bbox"][2],
                "bbox_y2": d["bbox"][3],
                "is_chosen": (i == chosen_idx),
            })

    csv_path = os.path.join(TEST_DIR, "yolo_results.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            w.writeheader()
            w.writerows(rows)
    print(f"\nWrote {csv_path}")


if __name__ == "__main__":
    main()
