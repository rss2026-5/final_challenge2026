#!/usr/bin/env python3
"""
Offline analyzer for a Part B perception test bag.

Usage (on the car):
    python3 analyze_bag.py ~/test_bags/phase2_<timestamp>

Reads the rosbag2 sqlite directly (no rclpy needed) and prints:
  - Per-topic message rate
  - Object-detection event timeline (when 'parking_meter' was published)
  - Traffic-light state transitions
  - Selected debug-image frames extracted as PNGs to OUTDIR

It also dumps representative debug frames so you can eyeball whether the
detectors fired on the correct things, without needing rqt_image_view.
"""
import os
import sys
import sqlite3
import struct
from collections import defaultdict
from pathlib import Path

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


def find_db3(bag_dir):
    bag_dir = Path(bag_dir).expanduser()
    if bag_dir.is_file() and bag_dir.suffix == ".db3":
        return bag_dir
    candidates = list(bag_dir.glob("*.db3"))
    if not candidates:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    return candidates[0]


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_bag.py <bag_dir_or_db3>")
        sys.exit(1)

    db_path = find_db3(sys.argv[1])
    outdir = Path(sys.argv[1]).expanduser() / "extracted_frames"
    outdir.mkdir(exist_ok=True)
    print(f"Reading {db_path}")
    print(f"Output frames -> {outdir}")
    print()

    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()

    cur.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cur.fetchall()}

    cur.execute("SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages")
    t_min, t_max, n_total = cur.fetchone()
    duration = (t_max - t_min) / 1e9
    print(f"Bag duration: {duration:.1f}s  total messages: {n_total}")
    print()

    # Per-topic counts and rate.
    print(f"{'Topic':<45} {'Type':<32} {'Count':>6} {'Hz':>6}")
    print("-" * 95)
    for tid, (name, type_str) in topics.items():
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
        count = cur.fetchone()[0]
        hz = count / duration if duration > 0 else 0.0
        print(f"{name:<45} {type_str:<32} {count:>6} {hz:>6.1f}")
    print()

    name_to_id = {name: tid for tid, (name, _) in topics.items()}

    # Object detection events.
    obj_id = name_to_id.get("/object_detection")
    if obj_id is not None and HAS_RCLPY:
        print("=== /object_detection events ===")
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (obj_id,),
        )
        msg_cls = get_message("std_msgs/msg/String")
        for ts, blob in cur.fetchall():
            t_rel = (ts - t_min) / 1e9
            try:
                m = deserialize_message(blob, msg_cls)
                print(f"  t={t_rel:6.2f}s  data='{m.data}'")
            except Exception as e:
                print(f"  t={t_rel:6.2f}s  <decode err: {e}>")
        print()

    # Traffic light transitions.
    tl_id = name_to_id.get("/traffic_light/state")
    if tl_id is not None and HAS_RCLPY:
        print("=== /traffic_light/state transitions ===")
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tl_id,),
        )
        msg_cls = get_message("std_msgs/msg/Bool")
        prev = None
        for ts, blob in cur.fetchall():
            t_rel = (ts - t_min) / 1e9
            try:
                m = deserialize_message(blob, msg_cls)
                marker = " (CHANGE)" if prev is not None and m.data != prev else ""
                print(f"  t={t_rel:6.2f}s  data={m.data}{marker}")
                prev = m.data
            except Exception as e:
                print(f"  t={t_rel:6.2f}s  <decode err: {e}>")
        print()

    # Extract debug image frames at fixed times.
    if HAS_RCLPY:
        try:
            import numpy as np
            import cv2
            img_cls = get_message("sensor_msgs/msg/Image")

            def extract_at_times(topic_name, times_rel, prefix):
                tid = name_to_id.get(topic_name)
                if tid is None:
                    return
                for t_rel in times_rel:
                    target_ts = t_min + int(t_rel * 1e9)
                    cur.execute(
                        "SELECT timestamp, data FROM messages "
                        "WHERE topic_id=? ORDER BY ABS(timestamp - ?) LIMIT 1",
                        (tid, target_ts),
                    )
                    row = cur.fetchone()
                    if row is None:
                        continue
                    ts, blob = row
                    try:
                        img = deserialize_message(blob, img_cls)
                        # Convert to numpy
                        arr = np.frombuffer(img.data, dtype=np.uint8)
                        if img.encoding == "bgr8":
                            arr = arr.reshape((img.height, img.width, 3))
                        elif img.encoding == "rgb8":
                            arr = arr.reshape((img.height, img.width, 3))[:, :, ::-1]
                        else:
                            arr = arr.reshape((img.height, img.width, -1))
                        actual_t = (ts - t_min) / 1e9
                        out = outdir / f"{prefix}_{actual_t:06.2f}s.png"
                        cv2.imwrite(str(out), arr)
                        print(f"  wrote {out.name}  (target t={t_rel:.1f}s, actual t={actual_t:.2f}s)")
                    except Exception as e:
                        print(f"  failed to extract t={t_rel:.1f}s: {e}")

            print("=== Extracting debug frames every 10s ===")
            sample_times = [i * 10 for i in range(int(duration // 10) + 1)]
            extract_at_times("/object_detector/debug_image", sample_times, "obj")
            extract_at_times("/traffic_light/debug_image", sample_times, "tl")
            print()

            # Also extract a frame near each /object_detection publish.
            if obj_id is not None:
                cur.execute(
                    "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                    (obj_id,),
                )
                fire_times = [(ts - t_min) / 1e9 for (ts,) in cur.fetchall()]
                if fire_times:
                    print("=== Extracting frames at object_detection fire times ===")
                    extract_at_times("/object_detector/debug_image", fire_times, "obj_fire")
                    print()

            # And frames near each traffic_light state transition.
            if tl_id is not None:
                cur.execute(
                    "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                    (tl_id,),
                )
                tl_times = [(ts - t_min) / 1e9 for (ts,) in cur.fetchall()]
                if tl_times:
                    print("=== Extracting frames at traffic_light transitions ===")
                    extract_at_times("/traffic_light/debug_image", tl_times, "tl_trans")
                    print()
        except Exception as e:
            print(f"image extraction failed: {e}")

    if not HAS_RCLPY:
        print("rclpy not importable in this env — counts only, no message decoding.")

    print("Done.")


if __name__ == "__main__":
    main()
