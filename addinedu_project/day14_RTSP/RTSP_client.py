import cv2

# RTSP 스트리밍 주소
rtsp_url = 'rtsp://192.168.0.167:8554/video'

# 비디오 캡처 객체 생성
cap = cv2.VideoCapture(rtsp_url)

# 연결이 성공했는지 확인
if not cap.isOpened():
    print("RTSP 스트리밍을 열 수 없습니다!")
    exit()

while True:
    # 영상 한 프레임을 읽어오기
    ret, frame = cap.read()

    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 프레임을 윈도우에 표시
    cv2.imshow('RTSP Stream', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 캡처 객체 해제
cap.release()
cv2.destroyAllWindows()

