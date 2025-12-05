import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Helper to count extended fingers
def count_fingers(hand_landmarks):
    tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky tips
    finger_count = 0

    # Wrist y coordinate (reference)
    wrist_y = hand_landmarks.landmark[0].y

    for tip_id in tips[1:]:  # Skip thumb for now (special case)
        tip_y = hand_landmarks.landmark[tip_id].y
        lower_joint_y = hand_landmarks.landmark[tip_id - 2].y
        if tip_y < lower_joint_y:  
            finger_count += 1

    # Thumb detection (left/right hand flips this)
    thumb_tip_x = hand_landmarks.landmark[4].x
    thumb_joint_x = hand_landmarks.landmark[3].x
    if thumb_tip_x > thumb_joint_x:
        finger_count += 1

    return finger_count

def classify_gesture(finger_count):
    if finger_count == 0:
        return "Fist"
    if finger_count == 1:
        return "Pointing"
    if finger_count == 5:
        return "Open Hand"
    return f"{finger_count} fingers"

# Camera capture
cap = cv2.VideoCapture(0)

with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
) as hands:

    while True:
        success, frame = cap.read()
        if not success:
            continue

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                fingers = count_fingers(hand_landmarks)
                gesture = classify_gesture(fingers)

                print(gesture)

                cv2.putText(frame, gesture, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)

        cv2.imshow("Hand Gesture Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()