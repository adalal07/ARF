import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


class YOLOFaceDetector:
    def __init__(
        self,
        weights_path: str = 'yolov12n-face.pt',
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.5,
        imgsz: int = 640,
        bbox_margin: float = 0.05,
        device: str | None = None,
    ):
        self.model = YOLO(weights_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.imgsz = imgsz
        self.bbox_margin = bbox_margin
        self.device = device

    def _expand_box(self, box, width, height):
        x1, y1, x2, y2 = box
        w = x2 - x1
        h = y2 - y1
        margin_x = w * self.bbox_margin
        margin_y = h * self.bbox_margin
        x1 = max(0, x1 - margin_x)
        y1 = max(0, y1 - margin_y)
        x2 = min(width - 1, x2 + margin_x)
        y2 = min(height - 1, y2 + margin_y)
        return int(x1), int(y1), int(x2), int(y2)

    def detect_faces(self, image: np.ndarray):
        if image is None or image.size == 0:
            raise ValueError("Input image/frame is empty")

        results = self.model.predict(
            source=image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

        result = results[0]
        boxes = []
        if result.boxes is not None:
            for xyxy, conf in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                x1, y1, x2, y2 = xyxy
                boxes.append(
                    {
                        "bbox": (float(x1), float(y1), float(x2), float(y2)),
                        "confidence": float(conf),
                    }
                )

        if not boxes:
            return False, None, None, []

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        overlay = image.copy()

        for box in boxes:
            x1, y1, x2, y2 = self._expand_box(box["bbox"], image.shape[1], image.shape[0])
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, thickness=-1)

        heatmap = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(image, 0.7, heatmap, 0.3, 0)
        return True, mask, overlay, boxes


def use_yolo_face_detector(image: np.ndarray, **kwargs):
    detector = YOLOFaceDetector(**kwargs)
    return detector.detect_faces(image)


