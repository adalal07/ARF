import cv2
import numpy as np

camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 320],
                          [0,    0,   1]], dtype=float)

dist_coeffs = np.zeros((5, 1))
marker_length = 0.05  # meters

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

half = marker_length / 2.0
marker_obj_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)


def process_frame(frame):
    """
    Detects ArUco markers in a frame and computes their pose.
    
    Returns:
        detections     → list of dictionaries containing:
            {
                "id": marker ID,
                "rvec": rvec,
                "tvec": tvec,
                "R": rotation matrix,
                "T": 4x4 transform matrix
            }
    """
    frame_out = frame.copy()
    detections = []

    # Detect markers
    corners, ids, _ = detector.detectMarkers(frame_out)

    if ids is None:
        return detections

    for marker_corners, marker_id in zip(corners, ids):
        img_points = marker_corners[0].astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            marker_obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success:
            continue
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        detections.append({
            "id": int(marker_id[0]),
            "rvec": rvec,
            "tvec": tvec,
            "R": R,
            "T": T
        })

    return detections


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error opening camera")
        exit()

    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed")
            break

        detections = process_frame(frame)

        for det in detections:
            print(f"\nMarker {det['id']} detected")
            print("Translation:", det["tvec"].ravel())
            print("Rotation matrix:\n", det["R"])
            print("Transform matrix:\n", det["T"])

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()