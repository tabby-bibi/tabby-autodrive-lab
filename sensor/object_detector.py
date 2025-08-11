# 센서 import
from .hc_sr04p_sensor import HCSR04Sensor
from .vl53l0x_sensor import VL53L0XSensor
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
           self.sensor2 = VL53L0XSensor()  # VL53L0X 레이저 거리 센서

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
                # 카메라 프레임은 필요할 때만 캡처 (비용이 큼)
                # image_frame = self.camera.capture_frame()

                try:
                    d_ultra = self.sensor1.get_distance()
                except Exception as e:
                    logging.warning("초음파 센서 실패: %s", e)
                    d_ultra = None


                logging.debug("[HC-SR04P] %s cm, [VL53L0X] %s cm", d_ultra, d_lidar)

                # 안전 비교: None 체크
                obstacle = ((d_ultra is not None and d_ultra < threshold_cm))


                if obstacle:
                    logging.info("⚠ 장애물 감지됨")
                    print("object detected")
                    # TODO: 정지/회피 명령 실행

                time.sleep(loop_delay)

        except KeyboardInterrupt:
            logging.info("사용자 중단")
        finally:
            try:
                self.camera.stop()
            except Exception:
                pass
            logging.info("감지 로직 종료")
