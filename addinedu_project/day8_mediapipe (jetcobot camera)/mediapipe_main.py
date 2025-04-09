import cv2
import time
from hand_module import handDetector  # 개별로 Github에서 가져와서 가져옴!

# 제스처 인식기 초기화
detector = handDetector()

# 캠 열기
cap = cv2.VideoCapture('/dev/jetcocam0')

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break

    # 손 인식 및 랜드마크 표시
    frame, hand_img = detector.findHands(frame, draw=True)

    # 제스처 인식
    gesture = ""
    if detector.lmList:
        gesture = detector.get_gesture()
        cv2.putText(frame, f'Gesture: {gesture}', (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

    # FPS 표시
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame, f'FPS: {int(fps)}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 윈도우 출력
    combined_frame = detector.frame_combine(frame, hand_img)
    cv2.imshow("Hand Gesture Detection", combined_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

