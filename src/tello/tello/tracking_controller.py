import cv2
import numpy as np
from faceDetection import use_mediapipe_mask
from faceDetection_yolo import use_yolo_face_detector

class FaceTracker:
    def __init__(self, target_size=0.08):
        self.target_size = target_size

    def compute_errors(self, mask, detections=None):
        """
        Given a binary face mask, compute:
        - horizontal error (normalized)
        - vertical error (normalized)
        - size error (desired area - current area)
        """

        H, W = mask.shape

        def _area(det):
            x1, y1, x2, y2 = det["bbox"]
            return (x2 - x1) * (y2 - y1)

        if detections:
            best = max(detections, key=_area)
            x1, y1, x2, y2 = best["bbox"]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            area = _area(best) / (W * H)
        else:
            ys, xs = np.where(mask > 0)
            if len(xs) == 0:
                return None

            cx = np.mean(xs)
            cy = np.mean(ys)
            w = xs.max() - xs.min()
            h = ys.max() - ys.min()
            area = (w * h) / (W * H)   

        nx = (cx - W/2) / (W/2)
        ny = (cy - H/2) / (H/2)

        size_error = self.target_size - area

        return {
            "nx": float(nx),
            "ny": float(ny),
            "size_error": float(size_error),
            "centroid": (float(cx), float(cy)),
            "area": float(area)
        }

def main():
    cap = cv2.VideoCapture(0) 
    tracker = FaceTracker(target_size=0.08)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        has_face, mask, overlay, detections = use_yolo_face_detector(frame)

        if has_face:
            errors = tracker.compute_errors(mask, detections=detections)

            if errors is not None:
                nx = errors["nx"]
                ny = errors["ny"]
                size_err = errors["size_error"]

                print(f"nx={nx:.3f}, ny={ny:.3f}, size_err={size_err:.3f}")

                cx, cy = errors["centroid"]
                vis = overlay.copy()
                cv2.circle(vis, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                cv2.imshow("Tracking Overlay", vis)
            else:
                print("Face lost")
                cv2.imshow("Tracking Overlay", frame)

        else:
            print("No face detected")
            cv2.imshow("Tracking Overlay", frame)

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()