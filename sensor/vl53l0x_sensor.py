import time
import board
import busio
import adafruit_vl53l0x


class VL53L0XSensor:
    def __init__(self, i2c=None):
        # I2C 인스턴스가 없으면 기본 보드 SCL, SDA로 초기화
        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c
        self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)

    def get_distance(self):
        """
        현재 센서가 측정한 거리(mm)를 반환합니다.
        """
        return self.sensor.range

    def continuous_measure(self, interval_sec=0.5):
        """
        거리 값을 지정한 간격(interval_sec)으로 계속 출력합니다.
        종료하려면 KeyboardInterrupt(Ctrl+C)를 누르세요.
        """
        print("거리 측정 시작 (종료하려면 Ctrl+C)")
        try:
            while True:
                dist = self.get_distance()
                print(f"Distance: {dist} mm")
                time.sleep(interval_sec)
        except KeyboardInterrupt:
            print("측정 종료")
