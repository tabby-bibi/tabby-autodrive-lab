'''
초음파 센서를 사용하여 좌우 장애물 감지하는 코드입니다 - 센서 하나 사용하는 기준으로 작성됨
# TODO : 센서 두개 할지 고민해봐야함
'''
class HCSR04Sensor:
    def __init__(self):
        pass  # GPIO 제어는 외부에서 처리

    def calculate_distance(self, pulse_duration_sec: float) -> float:
        """
        초음파 펄스 지속시간(초)을 받아서 cm 단위 거리로 변환.
        """
        SPEED_OF_SOUND_CM_PER_SEC = 34300  # cm/s
        distance_cm = (pulse_duration_sec * SPEED_OF_SOUND_CM_PER_SEC) / 2
        return distance_cm