#!/usr/bin/env python3
"""Validate avoidance-zone task markers in a recorded waypoint CSV."""

import argparse
import csv
import sys

from static_avoid.static_obstacle_planner import ReferencePath, extract_avoidance_zones


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--start-task", default="avoid_start")
    parser.add_argument("--end-task", default="avoid_end")
    args = parser.parse_args()

    points = []
    tasks = []
    frames = set()
    with open(args.csv_path, "r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise RuntimeError("CSV must contain x and y columns")
        for line_number, row in enumerate(reader, start=2):
            try:
                points.append((float(row["x"]), float(row["y"])))
            except (TypeError, ValueError):
                raise RuntimeError("Invalid x/y value at CSV line {}".format(line_number))
            tasks.append(str(row.get("task", "none") or "none").strip().lower())
            frame = str(row.get("frame_id", "") or "").strip()
            if frame:
                frames.add(frame)

    if len(points) < 2:
        raise RuntimeError("Route contains fewer than two points")
    if len(frames) > 1:
        raise RuntimeError("Route mixes multiple frame_id values: {}".format(sorted(frames)))

    reference = ReferencePath(points)
    zones = extract_avoidance_zones(
        reference,
        points,
        tasks,
        args.start_task.strip().lower(),
        args.end_task.strip().lower(),
    )
    if not zones:
        raise RuntimeError("No complete avoidance zone was found")

    print("route: {}".format(args.csv_path))
    print("frame: {}".format(next(iter(frames)) if frames else "<missing>"))
    print("points: {}".format(len(points)))
    print("length_m: {:.3f}".format(reference.length))
    for index, (start_s, end_s) in enumerate(zones, start=1):
        print("zone_{}: start_s={:.3f} end_s={:.3f} length={:.3f}".format(index, start_s, end_s, end_s - start_s))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        sys.exit(2)
