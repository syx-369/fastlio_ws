#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用 Intel RealSense D435i + Ultralytics YOLO 进行交通信号灯实时检测，
并在检测到信号灯后进一步判断当前亮起的是红灯、黄灯还是绿灯。

默认策略：
1. 用你已有的 YOLO 权重检测交通信号灯位置
2. 在检测框内部做 HSV 颜色分析
3. 结合交通灯常见排布（竖直：红-黄-绿；水平：红-黄-绿）判断当前灯态
4. 可选显示检测框中心的深度距离

适用场景：
- 你当前的 YOLO 权重只能检测“交通信号灯”这个目标，但不能区分红/黄/绿
- 你希望在不重新训练三分类模型的前提下，先检测再判断灯色

运行示例：
python realtime_detect_d435i_yolo_traffic_light_state.py \
    --weights runs/detect/train/weights/best.pt \
    --conf 0.4 \
    --imgsz 640 \
    --device 0

如果你的模型本身就直接把类别分成 red / yellow / green，也可以直接使用，程序会优先读取类别名。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


@dataclass
class DepthResult:
    distance_m: Optional[float]
    center_xy: Tuple[int, int]


@dataclass
class LightStateResult:
    state: str
    score: float
    layout: str
    debug_scores: Dict[str, float]


class RealsenseTrafficLightDetector:
    def __init__(
        self,
        weights: str,
        conf: float = 0.4,
        iou: float = 0.45,
        imgsz: int = 640,
        device: str = "0",
        width: int = 640,
        height: int = 480,
        fps: int = 60,
        depth_window: int = 5,
        show_depth: bool = True,
        min_saturation: int = 70,
        min_value: int = 90,
        min_state_score: float = 0.010,
        min_state_ratio: float = 1.15,
        target_classes: str = "",
    ) -> None:
        self.weights = weights
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.depth_window = max(1, depth_window)
        self.show_depth = show_depth
        self.min_saturation = int(min_saturation)
        self.min_value = int(min_value)
        self.min_state_score = float(min_state_score)
        self.min_state_ratio = float(min_state_ratio)
        self.target_classes = self._parse_target_classes(target_classes)

        self.model = YOLO(self.weights)
        self.names = self.model.names

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        self.depth_scale = None

        self._setup_camera()

    @staticmethod
    def _parse_target_classes(target_classes: str) -> set[str]:
        if not target_classes:
            return set()
        return {item.strip().lower() for item in target_classes.split(",") if item.strip()}

    def _setup_camera(self) -> None:
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        has_rgb = False
        for sensor in device.sensors:
            sensor_name = sensor.get_info(rs.camera_info.name)
            if sensor_name == "RGB Camera":
                has_rgb = True
                break

        if not has_rgb:
            raise RuntimeError("未检测到 RGB Camera，D435i 无法输出彩色图像。")

        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def _get_aligned_frames(self) -> Tuple[np.ndarray, rs.depth_frame]:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            raise RuntimeError("未成功获取对齐后的深度帧或彩色帧。")

        color_image = np.asanyarray(color_frame.get_data())
        return color_image, aligned_depth_frame

    def _robust_depth_at_box_center(
        self,
        depth_frame: rs.depth_frame,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> DepthResult:
        cx = max(0, min((x1 + x2) // 2, self.width - 1))
        cy = max(0, min((y1 + y2) // 2, self.height - 1))

        if not self.show_depth:
            return DepthResult(distance_m=None, center_xy=(cx, cy))

        depth_image = np.asanyarray(depth_frame.get_data())
        half = self.depth_window // 2

        x_start = max(0, cx - half)
        x_end = min(self.width, cx + half + 1)
        y_start = max(0, cy - half)
        y_end = min(self.height, cy + half + 1)

        patch = depth_image[y_start:y_end, x_start:x_end]
        valid = patch[patch > 0]

        if valid.size == 0:
            distance = depth_frame.get_distance(cx, cy)
            if distance <= 0:
                return DepthResult(distance_m=None, center_xy=(cx, cy))
            return DepthResult(distance_m=float(distance), center_xy=(cx, cy))

        median_depth_m = float(np.median(valid) * self.depth_scale)
        if median_depth_m <= 0:
            return DepthResult(distance_m=None, center_xy=(cx, cy))

        return DepthResult(distance_m=median_depth_m, center_xy=(cx, cy))

    @staticmethod
    def _normalize_class_name(name: str) -> str:
        return name.strip().lower().replace("-", " ").replace("_", " ")

    def _class_name_from_id(self, cls_id: int) -> str:
        if isinstance(self.names, dict):
            return str(self.names.get(int(cls_id), cls_id))
        if isinstance(self.names, list) and 0 <= int(cls_id) < len(self.names):
            return str(self.names[int(cls_id)])
        return str(cls_id)

    def _match_target_class(self, class_name: str, cls_id: int) -> bool:
        if not self.target_classes:
            return True
        norm_name = self._normalize_class_name(class_name)
        return norm_name in self.target_classes or str(int(cls_id)) in self.target_classes

    def _state_from_class_name(self, class_name: str) -> Optional[str]:
        name = self._normalize_class_name(class_name)

        if any(token in name for token in ["red", "红"]):
            return "red"
        if any(token in name for token in ["yellow", "amber", "黄"]):
            return "yellow"
        if any(token in name for token in ["green", "绿"]):
            return "green"
        return None

    def _state_text(self, state: str) -> str:
        mapping = {
            "red": "RED",
            "yellow": "YELLOW",
            "green": "GREEN",
            "unknown": "UNKNOWN",
        }
        return mapping.get(state, state.upper())

    def _state_terminal_text(self, state: str) -> str:
        mapping = {
            "red": "红灯",
            "yellow": "黄灯",
            "green": "绿灯",
            "unknown": "未知灯色",
        }
        return mapping.get(state, state)

    def _state_draw_color(self, state: str) -> Tuple[int, int, int]:
        if state == "red":
            return (0, 0, 255)
        if state == "yellow":
            return (0, 255, 255)
        if state == "green":
            return (0, 255, 0)
        return (255, 255, 255)

    def _safe_crop(self, image: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> np.ndarray:
        x1 = max(0, min(x1, image.shape[1] - 1))
        y1 = max(0, min(y1, image.shape[0] - 1))
        x2 = max(0, min(x2, image.shape[1]))
        y2 = max(0, min(y2, image.shape[0]))
        if x2 <= x1 or y2 <= y1:
            return np.empty((0, 0, 3), dtype=image.dtype)
        return image[y1:y2, x1:x2]

    def _make_color_masks(self, hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        hsv_blur = cv2.GaussianBlur(hsv, (5, 5), 0)
        h = hsv_blur[:, :, 0]
        s = hsv_blur[:, :, 1]
        v = hsv_blur[:, :, 2]

        bright_mask = (s >= self.min_saturation) & (v >= self.min_value)

        red_mask = (((h <= 10) | (h >= 160)) & bright_mask)
        yellow_mask = ((h >= 15) & (h <= 40) & bright_mask)
        green_mask = ((h >= 40) & (h <= 95) & bright_mask)

        return red_mask, yellow_mask, green_mask, v

    @staticmethod
    def _score_color(mask: np.ndarray, region_mask: np.ndarray, value_channel: np.ndarray) -> float:
        masked = mask & region_mask
        region_area = max(int(region_mask.sum()), 1)
        if masked.sum() == 0:
            return 0.0

        pixel_fraction = float(masked.sum()) / float(region_area)
        mean_brightness = float(value_channel[masked].mean()) / 255.0
        return pixel_fraction * mean_brightness

    def _judge_state_from_roi(self, roi_bgr: np.ndarray) -> LightStateResult:
        if roi_bgr.size == 0:
            return LightStateResult(
                state="unknown",
                score=0.0,
                layout="unknown",
                debug_scores={"red": 0.0, "yellow": 0.0, "green": 0.0},
            )

        roi_h, roi_w = roi_bgr.shape[:2]
        if roi_h < 8 or roi_w < 8:
            return LightStateResult(
                state="unknown",
                score=0.0,
                layout="unknown",
                debug_scores={"red": 0.0, "yellow": 0.0, "green": 0.0},
            )

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        red_mask, yellow_mask, green_mask, value_channel = self._make_color_masks(hsv)

        yy, xx = np.indices((roi_h, roi_w))
        one_third_h = roi_h / 3.0
        one_third_w = roi_w / 3.0

        vertical_regions = {
            "red": yy < one_third_h,
            "yellow": (yy >= one_third_h) & (yy < 2.0 * one_third_h),
            "green": yy >= 2.0 * one_third_h,
        }
        horizontal_regions = {
            "red": xx < one_third_w,
            "yellow": (xx >= one_third_w) & (xx < 2.0 * one_third_w),
            "green": xx >= 2.0 * one_third_w,
        }

        color_masks = {
            "red": red_mask,
            "yellow": yellow_mask,
            "green": green_mask,
        }

        global_scores: Dict[str, float] = {}
        whole_region = np.ones((roi_h, roi_w), dtype=bool)
        for color_name, color_mask in color_masks.items():
            global_scores[color_name] = self._score_color(color_mask, whole_region, value_channel)

        vertical_scores: Dict[str, float] = {}
        horizontal_scores: Dict[str, float] = {}
        for color_name, color_mask in color_masks.items():
            vertical_scores[color_name] = (
                0.80 * self._score_color(color_mask, vertical_regions[color_name], value_channel)
                + 0.20 * global_scores[color_name]
            )
            horizontal_scores[color_name] = (
                0.80 * self._score_color(color_mask, horizontal_regions[color_name], value_channel)
                + 0.20 * global_scores[color_name]
            )

        vertical_total = sum(vertical_scores.values())
        horizontal_total = sum(horizontal_scores.values())

        if vertical_total >= horizontal_total:
            chosen_layout = "vertical"
            final_scores = vertical_scores
        else:
            chosen_layout = "horizontal"
            final_scores = horizontal_scores

        sorted_items = sorted(final_scores.items(), key=lambda item: item[1], reverse=True)
        best_state, best_score = sorted_items[0]
        second_score = sorted_items[1][1] if len(sorted_items) > 1 else 0.0

        if best_score < self.min_state_score:
            return LightStateResult(
                state="unknown",
                score=best_score,
                layout=chosen_layout,
                debug_scores=final_scores,
            )

        if second_score > 0 and best_score / max(second_score, 1e-6) < self.min_state_ratio:
            return LightStateResult(
                state="unknown",
                score=best_score,
                layout=chosen_layout,
                debug_scores=final_scores,
            )

        return LightStateResult(
            state=best_state,
            score=best_score,
            layout=chosen_layout,
            debug_scores=final_scores,
        )

    def _draw_result(
        self,
        image: np.ndarray,
        box: np.ndarray,
        conf: float,
        class_name: str,
        depth_result: DepthResult,
        light_state_result: LightStateResult,
    ) -> None:
        x1, y1, x2, y2 = box.astype(int)
        x1 = max(0, min(x1, self.width - 1))
        y1 = max(0, min(y1, self.height - 1))
        x2 = max(0, min(x2, self.width - 1))
        y2 = max(0, min(y2, self.height - 1))

        state_text = self._state_text(light_state_result.state)
        box_color = self._state_draw_color(light_state_result.state)

        label = f"{class_name} {conf:.2f} | {state_text}"
        if depth_result.distance_m is not None:
            label += f" | {depth_result.distance_m:.3f} m"

        cv2.rectangle(image, (x1, y1), (x2, y2), box_color, 2)

        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        text_w, text_h = text_size
        text_y = y1 - 10 if y1 - 10 > 10 else y1 + text_h + 10

        cv2.rectangle(
            image,
            (x1, text_y - text_h - 8),
            (x1 + text_w + 8, text_y + 4),
            box_color,
            -1,
        )
        cv2.putText(
            image,
            label,
            (x1 + 4, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

        cx, cy = depth_result.center_xy
        cv2.circle(image, (cx, cy), 4, (255, 0, 255), -1)

    def run(self) -> None:
        prev_time = time.time()

        try:
            while True:
                color_image, aligned_depth_frame = self._get_aligned_frames()

                results = self.model.predict(
                    source=color_image,
                    conf=self.conf,
                    iou=self.iou,
                    imgsz=self.imgsz,
                    device=self.device,
                    verbose=False,
                )

                display_image = color_image.copy()
                detected_count = 0

                if results and len(results) > 0 and results[0].boxes is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    confs = results[0].boxes.conf.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy().astype(int)

                    for box, score, cls_id in zip(boxes, confs, classes):
                        class_name = self._class_name_from_id(int(cls_id))
                        if not self._match_target_class(class_name, int(cls_id)):
                            continue

                        x1, y1, x2, y2 = box.astype(int)
                        roi = self._safe_crop(color_image, x1, y1, x2, y2)

                        state_from_cls = self._state_from_class_name(class_name)
                        if state_from_cls is not None:
                            light_state_result = LightStateResult(
                                state=state_from_cls,
                                score=1.0,
                                layout="class_name",
                                debug_scores={
                                    "red": 1.0 if state_from_cls == "red" else 0.0,
                                    "yellow": 1.0 if state_from_cls == "yellow" else 0.0,
                                    "green": 1.0 if state_from_cls == "green" else 0.0,
                                },
                            )
                        else:
                            light_state_result = self._judge_state_from_roi(roi)

                        depth_result = self._robust_depth_at_box_center(
                            aligned_depth_frame, x1, y1, x2, y2
                        )
                        terminal_message = (
                            f"[检测结果] {self._state_terminal_text(light_state_result.state)}"
                            f" | 类别: {class_name}"
                            f" | 置信度: {float(score):.2f}"
                            f" | 灯态分数: {light_state_result.score:.3f}"
                        )
                        if depth_result.distance_m is not None:
                            terminal_message += f" | 距离: {depth_result.distance_m:.3f} m"
                        print(terminal_message, flush=True)

                        self._draw_result(
                            display_image,
                            box,
                            float(score),
                            class_name,
                            depth_result,
                            light_state_result,
                        )
                        detected_count += 1

                curr_time = time.time()
                fps_value = 1.0 / max(curr_time - prev_time, 1e-6)
                prev_time = curr_time

                cv2.putText(
                    display_image,
                    f"FPS: {fps_value:.1f}",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display_image,
                    f"Detections: {detected_count}",
                    (15, 62),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display_image,
                    "Press q or ESC to quit",
                    (15, self.height - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow("D435i + YOLO Traffic Light State Detection", display_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i + YOLO 交通信号灯实时检测与灯态判断")
    parser.add_argument("--weights", type=str, default="traffic light.pt", help="YOLO 权重路径，例如 best.pt")
    parser.add_argument("--conf", type=float, default=0.4, help="置信度阈值")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="推理尺寸")
    parser.add_argument("--device", type=str, default="0", help="推理设备，如 0 / cpu")
    parser.add_argument("--width", type=int, default=640, help="相机宽度")
    parser.add_argument("--height", type=int, default=480, help="相机高度")
    parser.add_argument("--fps", type=int, default=60, help="相机帧率")
    parser.add_argument("--depth-window", type=int, default=5, help="深度中值窗口大小，建议奇数")
    parser.add_argument(
        "--hide-depth",
        action="store_true",
        help="不显示距离信息，只显示检测框和灯态",
    )
    parser.add_argument(
        "--min-saturation",
        type=int,
        default=70,
        help="HSV 中 S 的最小阈值，越大越严格",
    )
    parser.add_argument(
        "--min-value",
        type=int,
        default=90,
        help="HSV 中 V 的最小阈值，越大越偏向高亮灯区域",
    )
    parser.add_argument(
        "--min-state-score",
        type=float,
        default=0.010,
        help="红/黄/绿判定最低分数阈值，过低会判为 UNKNOWN",
    )
    parser.add_argument(
        "--min-state-ratio",
        type=float,
        default=1.15,
        help="第一名与第二名颜色得分比，过小会判为 UNKNOWN",
    )
    parser.add_argument(
        "--target-classes",
        type=str,
        default="",
        help="只处理指定类别，支持类别名或类别id，多个用逗号分隔，例如 'traffic light,0'",
    )
    return parser.parse_args()



def main() -> int:
    args = parse_args()

    try:
        detector = RealsenseTrafficLightDetector(
            weights=args.weights,
            conf=args.conf,
            iou=args.iou,
            imgsz=args.imgsz,
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            depth_window=args.depth_window,
            show_depth=not args.hide_depth,
            min_saturation=args.min_saturation,
            min_value=args.min_value,
            min_state_score=args.min_state_score,
            min_state_ratio=args.min_state_ratio,
            target_classes=args.target_classes,
        )
        detector.run()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
