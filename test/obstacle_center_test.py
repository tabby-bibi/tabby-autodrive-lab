import cv2
import numpy as np

# 모듈 임포트
from camera.camera_controller import CameraManager

# 임계값 거리 같은건 여기선 테스트용이므로 생략
# 장애물 중심 계산 함수
def get_obstacle_center(frame):
    """
    프레임에서 장애물 후보를 찾아 중심 x좌표(cx) 반환
    간단히 밝기/색상 기준으로 검출
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # 밝은 배경이면 THRESH_BINARY

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # 가장 큰 컨투어 선택
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # 프레임에 표시
            cv2.circle(frame, (cx, cy), 10, (0,0,255), -1)
            cv2.drawContours(frame, [largest_contour], -1, (0,255,0), 2)
            return cx
    # 검출 안되면 화면 중앙 반환
    h, w, _ = frame.shape
    return w // 2

def main():
    cap = CameraManager() # Camera manager 객체 생성
    if not cap.isOpened():
        print("카메라 열기 실패")
        return

    FRAME_CENTER = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2)

    while True:
        ret, frame = cap.capture_frame()
        if not ret:
            continue

        cx = get_obstacle_center(frame)

        # 화면 중앙 표시
        cv2.line(frame, (FRAME_CENTER,0), (FRAME_CENTER, frame.shape[0]), (255,0,0), 2)
        # 장애물 중심 표시 이미 get_obstacle_center 안에서 표시됨

        cv2.putText(frame, f"Obstacle CX: {cx}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

        cv2.imshow("Obstacle Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # q 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
