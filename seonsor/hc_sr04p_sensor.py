import RPi.GPIO as GPIO
# TODO : 개발 환경 ( ex : macOs ) 에 따른 라이브러리 설정 해야 함 일단은 그냥 에러 그냥 둘거임 
import time

# TODO : GPIO 핀 설정등 어떻게 할지 정해야 함
class HCSR04Sensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, False)
        time.sleep(2)  # 센서 안정화 시간

    def get_distance(self):
        # 트리거에 10us 펄스 신호 발생
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        pulse_start = time.time()
        pulse_end = time.time()

        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()

        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 34300 / 2  # cm 단위 거리 계산

        return distance

    def cleanup(self):
        GPIO.cleanup()
