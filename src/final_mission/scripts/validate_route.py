#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛前航迹校验。不启动任何节点，纯静态检查 CSV。

用法：
    rosrun final_mission validate_route.py <csv_path> [--rounds 2]

检查项（按严重程度）：

ERROR（必须修，否则跑不通）
  1. 缺列、行内容非法
  2. avoid_start / avoid_end 不成对或嵌套
  3. 任务点落在避障区内（A* 重规划与精确停靠会打架）
  4. 七点位数量与轮次不符（决赛两轮需各出现 2 次）
  5. 同一轮内 piper_stop 顺序错乱（必须 1→2→3→4→5→6→7）

WARN（能跑但有风险）
  6. 相邻任务点间距过小（< goal_reached_dist×2，停靠会互相干扰）
  7. 任务点朝向与前一航点朝向差异过大（对齐时会大幅自转，伸臂有碰撞风险）
  8. 避障区长度异常（过短说明标记打得太近）
"""

import argparse
import csv
import math
import sys


REQUIRED_COLUMNS = {"seq", "x", "y", "yaw", "task"}
ARM_SEQUENCE = [
    "piper_stop_1", "piper_stop_2", "piper_stop_3", "piper_stop_4",
    "piper_stop_5", "piper_stop_6", "piper_stop_7",
]
ZONE_MARKERS = ("avoid_start", "avoid_end")


def external_name(task):
    task = (task or "").strip()
    if not task.lower().startswith("ext:"):
        return None
    return task[4:].strip() or None


def load(csv_path):
    rows = []
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = REQUIRED_COLUMNS.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("缺少必需列：%s" % ", ".join(sorted(missing)))
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append({
                    "line": line_number,
                    "seq": int(row["seq"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw": float(row["yaw"]),
                    "task": (row.get("task") or "none").strip(),
                })
            except (TypeError, ValueError) as exc:
                raise ValueError("第 %d 行非法：%s" % (line_number, exc))
    return rows


def check(rows, rounds, goal_reached_dist=0.05, yaw_warn=0.10):
    errors = []
    warnings = []

    # ---- 避障区配对 ----
    zones = []
    open_start = None
    for index, row in enumerate(rows):
        marker = row["task"].lower()
        if marker == "avoid_start":
            if open_start is not None:
                errors.append(
                    "第 %d 行 avoid_start 嵌套（上一个在第 %d 行未闭合）"
                    % (row["line"], rows[open_start]["line"])
                )
            open_start = index
        elif marker == "avoid_end":
            if open_start is None:
                errors.append("第 %d 行 avoid_end 没有对应的 avoid_start" % row["line"])
            else:
                zones.append((open_start, index))
                open_start = None
    if open_start is not None:
        errors.append(
            "第 %d 行 avoid_start 没有对应的 avoid_end" % rows[open_start]["line"]
        )

    # ---- 任务点收集 ----
    task_points = [
        (index, row) for index, row in enumerate(rows)
        if row["task"] not in ("", "none") and row["task"].lower() not in ZONE_MARKERS
    ]

    # ---- 任务点是否落在避障区内 ----
    for index, row in task_points:
        for start, end in zones:
            if start <= index <= end:
                errors.append(
                    "任务点 seq=%d (%s) 落在避障区 seq=%d..%d 内：A* 重规划"
                    "会与精确停靠冲突，请重录使两者分离"
                    % (row["seq"], row["task"], rows[start]["seq"], rows[end]["seq"])
                )

    # ---- 七点位数量与顺序 ----
    # rounds<=0 表示不检查七点位（用于校验预赛航迹这类不含机械臂停靠的路线）。
    arm_hits = [
        (index, row, external_name(row["task"]))
        for index, row in task_points
        if external_name(row["task"]) in ARM_SEQUENCE
    ]
    if rounds > 0:
        counts = {name: 0 for name in ARM_SEQUENCE}
        for _, _, name in arm_hits:
            counts[name] += 1
        for name in ARM_SEQUENCE:
            if counts[name] != rounds:
                errors.append(
                    "%s 出现 %d 次，期望 %d 次（决赛两轮取货卸货）"
                    % (name, counts[name], rounds)
                )

        expected = ARM_SEQUENCE * rounds
        actual = [name for _, _, name in arm_hits]
        if actual != expected and len(actual) == len(expected):
            errors.append(
                "七点位顺序错乱。期望 %s，实际 %s"
                % (" -> ".join(expected), " -> ".join(actual))
            )
    elif arm_hits:
        warnings.append(
            "已用 --rounds 0 跳过七点位检查，但航迹里有 %d 个 piper_stop 点。"
            % len(arm_hits)
        )

    # ---- 相邻任务点间距 ----
    min_gap = goal_reached_dist * 2.0
    for i in range(len(task_points) - 1):
        _, a = task_points[i]
        _, b = task_points[i + 1]
        gap = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        if gap < min_gap:
            warnings.append(
                "任务点 seq=%d 与 seq=%d 间距仅 %.3fm（< %.3fm），停靠会互相干扰"
                % (a["seq"], b["seq"], gap, min_gap)
            )

    # ---- 任务点朝向突变 ----
    for index, row in task_points:
        if index == 0:
            continue
        previous = rows[index - 1]
        diff = abs(math.atan2(
            math.sin(row["yaw"] - previous["yaw"]),
            math.cos(row["yaw"] - previous["yaw"]),
        ))
        if diff > yaw_warn:
            warnings.append(
                "任务点 seq=%d 朝向与前一点差 %.3frad(%.1f°)，对齐时会大幅自转；"
                "机械臂侧伸时有碰撞风险"
                % (row["seq"], diff, math.degrees(diff))
            )

    # ---- 改动 2026-07-30：红绿灯必须排在机械臂任务之前 ----
    # 相机装在机械臂上。红绿灯在七点位之前时机械臂在零位（相机前视），能看到灯。
    # 若排到取放段之后，一旦某轮抓取失败，机械臂会停在 PICK_SCAN
    # （piper_task_node.py:32，joint1=-1.530rad=-87.7°，侧伸），相机朝侧面，
    # 灯根本不在视野里 —— 而且这个失败是静默的：视觉一直报 none，
    # 总控等满 traffic_timeout 后按 pass 策略放行，等于闯红灯。
    light_indices = [
        index for index, row in task_points
        if external_name(row["task"]) == "traffic_light"
    ]
    arm_indices = [index for index, _, _ in arm_hits]
    if light_indices and arm_indices:
        first_arm = min(arm_indices)
        for index in light_indices:
            if index > first_arm:
                warnings.append(
                    "ext:traffic_light 排在机械臂任务点之后（seq=%d 在 seq=%d 之后）。"
                    "抓取失败时机械臂停在侧伸姿态，车载相机看不到灯。"
                    "建议把红绿灯点录在所有 piper_stop 之前"
                    % (rows[index]["seq"], rows[first_arm]["seq"])
                )

    # ---- 避障区长度 ----
    for start, end in zones:
        length = 0.0
        for i in range(start, end):
            length += math.hypot(
                rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"]
            )
        if length < 1.0:
            warnings.append(
                "避障区 seq=%d..%d 弧长仅 %.2fm，标记可能打得太近"
                % (rows[start]["seq"], rows[end]["seq"], length)
            )

    return errors, warnings, zones, task_points


def main():
    parser = argparse.ArgumentParser(description="决赛航迹校验")
    parser.add_argument("csv_path")
    parser.add_argument("--rounds", type=int, default=2,
                        help="取货卸货轮次，决赛为 2；填 0 跳过七点位检查")
    parser.add_argument("--goal-reached-dist", type=float, default=0.05)
    args = parser.parse_args()

    try:
        rows = load(args.csv_path)
    except (OSError, ValueError) as exc:
        print("[ERROR] %s" % exc)
        return 1

    if not rows:
        print("[ERROR] CSV 为空")
        return 1

    errors, warnings, zones, task_points = check(
        rows, args.rounds, args.goal_reached_dist
    )

    total_length = 0.0
    for i in range(len(rows) - 1):
        total_length += math.hypot(
            rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"]
        )

    print("=" * 64)
    print("航迹：%s" % args.csv_path)
    print("航点数 %d，总弧长 %.2fm，平均间距 %.3fm"
          % (len(rows), total_length, total_length / max(len(rows) - 1, 1)))
    print("避障区 %d 个，任务点 %d 个" % (len(zones), len(task_points)))
    print("-" * 64)
    for start, end in zones:
        print("  避障区 seq=%d..%d" % (rows[start]["seq"], rows[end]["seq"]))
    for _, row in task_points:
        print("  任务点 seq=%-5d %s" % (row["seq"], row["task"]))
    print("-" * 64)

    for message in warnings:
        print("[WARN ] %s" % message)
    for message in errors:
        print("[ERROR] %s" % message)

    print("=" * 64)
    if errors:
        print("结论：不可用，先修 %d 个 ERROR。" % len(errors))
        return 1
    if warnings:
        print("结论：可用，但有 %d 个 WARN 需确认。" % len(warnings))
        return 0
    print("结论：校验全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
