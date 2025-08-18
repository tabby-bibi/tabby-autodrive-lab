# 센서 import
from .hc_sr04p_sensor import HCSR04Sensor
from camera.camera_controller import CameraManager

# 패키지 import
import logging
import time

# import
import cv2

# TODO: 이거 나중에 constants.py로 분리
TRIG_PIN = 23
ECHO_PIN = 24


class ObjectDetector:
    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN):
        try:
            # 센서 초기화
            self.sensor1 = HCSR04Sensor(trig_pin, echo_pin)  # HC-SR04P 초음파 센서
            # 카메라 초기화
            self.camera = CameraManager()
        except Exception as e:
            print(f"Error while Initializing Object Detector : {e}")

    def detect_obstacles(self, threshold_cm=20, loop_delay=0.05):
        logging.info("장애물 감지 시작")
        try:
            self.camera.start()
        except Exception as e:
            logging.error("카메라 시작 실패: %s", e)
            return

        try:
            while True:
                # 초음파 거리 측정
                try:
                    d_ultra = self.sensor1.get_distance()
                except Exception as e:
                    logging.warning("초음파 센서 실패: %s", e)
                    d_ultra = None

                logging.debug("[HC-SR04P] %s cm", d_ultra)

                obstacle = (d_ultra is not None and d_ultra < threshold_cm)

                if obstacle:
                    logging.info("⚠ 장애물 감지됨")
                    print("object detected")

                    # === 카메라 프레임 분석 ===
                    frame = self.camera.capture_frame()
                    if frame is not None:
                        cx = self._get_obstacle_center(frame)  # TODO: OpenCV 객체 인식으로 구현 가능
                        frame_center = frame.shape[1] // 2  # 가로 중앙값

                        if cx < frame_center:
                            logging.info("➡ 장애물이 왼쪽 → 오른쪽으로 회피")
                            print("right turn")
                        else:
                            logging.info("➡ 장애물이 오른쪽 → 왼쪽으로 회피")
                            print("left turn")

                        time.sleep(0.5)
                        print("forward")

                time.sleep(loop_delay)

        except KeyboardInterrupt:
            logging.info("사용자 중단")
        finally:
            try:
                self.camera.stop()
            except Exception:
                pass
            logging.info("감지 로직 종료")

    def _get_obstacle_center(self, frame):
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
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                return cx
        # 검출 안되면 화면 중앙 반환
        h, w, _ = frame.shape
        return w // 2