"""
프로젝트 실행 진입점 파일입니다.
"""
from sensor.object_detector import ObjectDetector

# HC-SR04P 핀 번호
TRIG_PIN = 23
ECHO_PIN = 24

def main():
    print(" ====== main 시작 ====== ")

    # ObjectDetector 인스턴스 생성
    detector = ObjectDetector(
        image=None,          # 이미지 객체(카메라 모듈 쓰면 CameraManager에서 받음)
        trig_pin=TRIG_PIN,
        echo_pin=ECHO_PIN
    )

    # 장애물 감지 실행
    try:
        detector.detect_obstacles()
    except KeyboardInterrupt:
        print("\n사용자 종료 요청 감지됨")

    print(" ====== main 끝남 ====== ")


if __name__ == '__main__':
    main()
