# 센서 import
from hc_sr04p_sensor import HCSR04Sensor
from vl50l0x_sensor import VL53L0XSensor
from camera.camera_controller import CameraManager

# TODO: 이거 나중에 constants.py로 분리
TRIG_PIN = 23
ECHO_PIN = 24


class ObjectDetector:
    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN):
        # 센서 초기화
        self.sensor1 = HCSR04Sensor(trig_pin, echo_pin)  # HC-SR04P 초음파 센서
        self.sensor2 = VL53L0XSensor()  # VL53L0X 레이저 거리 센서

        # 카메라 초기화
        self.camera = CameraManager()

    def detect_obstacles(self):
        print("====== 장애물 감지 로직 시작 ======")

        # 카메라 시작
        self.camera.start()

        try:
            while True:
                # 카메라 프레임 캡처
                image_frame = self.camera.capture_frame()

                # 센서 데이터 읽기
                distance_ultrasonic = self.sensor1.get_distance()
                distance_lidar = self.sensor2.get_distance()

                # 거리 정보 출력
                print(f"[HC-SR04P] Distance: {distance_ultrasonic} cm")
                print(f"[VL53L0X] Distance: {distance_lidar} cm")

                # TODO: 이미지 + 거리 데이터 처리 로직
                # 예: 특정 거리 이하이면 멈춤 또는 경고
                if distance_ultrasonic < 20 or distance_lidar < 20:
                    print("⚠ 장애물 감지됨! 정지 또는 회피 로직 실행")

        except KeyboardInterrupt:
            print("사용자에 의해 중단됨.")

        finally:
            self.camera.stop()
            print("====== 장애물 감지 로직 종료 ======")
