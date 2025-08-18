# 센서 import
from .hc_sr04p_sensor import HCSR04Sensor
from camera.camera_controller import CameraManager

# 패키지 import
import logging
import time

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

                    if self.motor:
                        self.motor.stop()
                        time.sleep(0.3)

                        # === 카메라 프레임 분석 ===
                        frame = self.camera.capture_frame()
                        cx = self._get_obstacle_center(frame)  # TODO: OpenCV 등으로 구현
                        frame_center = frame.shape[1] // 2  # 가로 중앙값

                        if cx < frame_center:
                            logging.info("➡ 장애물이 왼쪽 → 오른쪽으로 회피")
                            self.motor.turn_right()
                        else:
                            logging.info("➡ 장애물이 오른쪽 → 왼쪽으로 회피")
                            self.motor.turn_left()

                        time.sleep(0.5)
                        self.motor.forward()

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
        프레임에서 장애물 중심 x좌표(cx) 반환
        → 임시로 화면 정중앙 반환, 추후 OpenCV 객체 인식으로 구현 가능
        """
        h, w, _ = frame.shape
        return w // 2

