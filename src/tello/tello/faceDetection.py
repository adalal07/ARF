import cv2
import mediapipe as mp
import numpy as np


class FaceMaskExtractor:
    def __init__(self, model_path):
        BaseOptions = mp.tasks.BaseOptions
        FaceLandmarker = mp.tasks.vision.FaceLandmarker
        FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        self.FaceLandmarker = FaceLandmarker

        self.options = FaceLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.IMAGE,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=False
        )
        self.FACE_OUTLINE = [
            10, 338, 297, 332, 284, 251, 389, 356,
            454, 323, 361, 288, 397, 365, 379, 378,
            400, 377, 152, 148, 176, 149, 150, 136,
            172, 58, 132, 93, 234, 127, 162, 21,
            54, 103, 67, 109
        ]

    def _outline_mask(self, image, landmarks):
        h, w, _ = image.shape
        pts = []

        for idx in self.FACE_OUTLINE:
            lm = landmarks[idx]
            x, y = int(lm.x * w), int(lm.y * h)
            pts.append([x, y])

        pts = np.array(pts, np.int32)

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        return mask

    def extract_mask(self, image):
        """Return (detectedFace, image, mask, overlay, landmarks)."""
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        with self.FaceLandmarker.create_from_options(self.options) as landmarker:
            result = landmarker.detect(mp_image)

        if not result.face_landmarks:
            return False, None, None, None, None

        landmarks = result.face_landmarks[0]
        mask = self._outline_mask(image, landmarks)

        heatmap = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(image, 0.7, heatmap, 0.3, 0)

        return True, image, mask, overlay, landmarks
def use_mediapipe_mask(img):
    extractor = FaceMaskExtractor(
        model_path="face_landmarker.task"
    )
    hasimage, image, mask, overlay, landmarks = extractor.extract_mask(img)
    return hasimage, mask, overlay, landmarks
   
if __name__ == "__main__":
    #img_path = "/Users/harshapolavaram/Desktop/Photo on 21-11-25 at 10.31 PM.jpg"
    #img_path="/Users/harshapolavaram/Desktop/huggingface-color.png"
    img = cv2.imread("test_image.png")
    hasimage, mask, overlay, landmarks = use_mediapipe_mask(img)
    if hasimage:
        cv2.imshow("Mask", mask)
        cv2.imshow("Overlay", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
