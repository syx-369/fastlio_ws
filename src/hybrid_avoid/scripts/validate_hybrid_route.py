#!/usr/bin/env python3

import argparse
import csv
import os
import sys

from hybrid_avoid.hybrid_planner import ReferencePath, extract_avoidance_zones


def main():
    parser = argparse.ArgumentParser(description="Validate a hybrid_avoid CSV route")
    parser.add_argument("csv_path")
    arguments = parser.parse_args()
    if not os.path.isfile(arguments.csv_path):
        print("ERROR: file does not exist: {}".format(arguments.csv_path), file=sys.stderr)
        return 2
    points, tasks, frames = [], [], set()
    with open(arguments.csv_path, "r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            try:
                points.append((float(row["x"]), float(row["y"])))
                tasks.append(str(row.get("task", "none") or "none").strip().lower())
                if row.get("frame"):
                    frames.add(row["frame"].strip())
            except (ValueError, TypeError, KeyError):
                continue
    try:
        reference = ReferencePath(points)
        zones = extract_avoidance_zones(reference, points, tasks)
    except ValueError as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    if not zones:
        print("ERROR: no avoid_start/avoid_end zone", file=sys.stderr)
        return 1
    print("route: {}".format(arguments.csv_path))
    print("frame: {}".format(",".join(sorted(frames)) if frames else "unspecified"))
    print("points: {}".format(len(points)))
    print("length_m: {:.3f}".format(reference.length))
    for index, zone in enumerate(zones, 1):
        print("zone_{}: start_s={:.3f} end_s={:.3f} length={:.3f}".format(index, zone[0], zone[1], zone[1] - zone[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
