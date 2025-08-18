"""
프로젝트 실행 진입점 파일
- 차선 인식 + 장애물 회피 통합
"""
import cv2
import logging
import time

from sensor.object_detector import ObjectDetector
from lane_detection.lane_detector import region_of_interest, compute_histogram

# HC-SR04P 핀 번호
TRIG_PIN = 23
ECHO_PIN = 24

# 파라미터
THRESHOLD_CM = 20   # 장애물 거리 임계값
FRAME_WIDTH = 640
FRAME_CENTER = FRAME_WIDTH // 2


def main():
    print(" ====== main 시작 ====== ")

    # ObjectDetector 인스턴스 생성
    detector = ObjectDetector(
        trig_pin=TRIG_PIN,
        echo_pin=ECHO_PIN
    )

    try:
        detector.camera.start()
        logging.info("카메라 시작 완료")

        while True:
            # 카메라 프레임 획득
            frame = detector.camera.capture_frame()
            if frame is None:
                continue

            # 초음파 거리 측정
            try:
                d_ultra = detector.sensor1.get_distance()
            except Exception as e:
                logging.warning("초음파 센서 읽기 실패: %s", e)
                d_ultra = None

            # 장애물 감지 여부
            obstacle = (d_ultra is not None and d_ultra < THRESHOLD_CM)

            if obstacle:
                # === 회피 모드 ===
                logging.info("⚠ 장애물 감지됨: %s cm", d_ultra)
                cx = detector._get_obstacle_center(frame)  # ObjectDetector 내부 함수
                if cx < FRAME_CENTER:
                    logging.info("➡ 장애물이 왼쪽 → 오른쪽으로 회피")
                    detector.motor.turn_right()
                else:
                    logging.info("➡ 장애물이 오른쪽 → 왼쪽으로 회피")
                    detector.motor.turn_left()

                time.sleep(0.5)
                detector.motor.forward()

            else:
                # === 차선 인식 기반 주행 모드 ===
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                roi = region_of_interest(gray)  # 하단 ROI
                hist = compute_histogram(roi)
                lane_center = int(hist.argmax())
                deviation = lane_center - FRAME_CENTER

                if deviation < -30:
                    logging.debug("차선 왼쪽 → 좌회전")
                    detector.motor.turn_left()
                elif deviation > 30:
                    logging.debug("차선 오른쪽 → 우회전")
                    detector.motor.turn_right()
                else:
                    logging.debug("차선 중앙 → 직진")
                    detector.motor.forward()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n사용자 종료 요청 감지됨")

    finally:
        detector.camera.stop()
        print(" ====== main 끝남 ====== ")


if __name__ == '__main__':
    main()