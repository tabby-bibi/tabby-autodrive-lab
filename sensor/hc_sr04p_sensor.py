import RPi.GPIO as GPIO
import time

class HCSR04Sensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, False)
        time.sleep(2)

    def get_distance(self, timeout=0.02, samples=3):
        distances = []
        for _ in range(samples):
            # 트리거 펄스
            GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, False)

            start_time = time.time()
            stop_time = time.time()

            # Echo 핀 HIGH 대기 (timeout 추가)
            while GPIO.input(self.echo_pin) == 0:
                start_time = time.time()
                if start_time - stop_time > timeout:
                    return None

            # Echo 핀 LOW 대기
            while GPIO.input(self.echo_pin) == 1:
                stop_time = time.time()
                if stop_time - start_time > timeout:
                    return None

            # 거리 계산
            pulse_duration = stop_time - start_time
            distance = (pulse_duration * 34300) / 2
            distances.append(distance)

            time.sleep(0.05)  # 샘플 간 간격

        # 평균값 반환
        return sum(distances) / len(distances) if distances else None

    def cleanup(self):
        GPIO.cleanup()
