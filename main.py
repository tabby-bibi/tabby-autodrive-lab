"""
프로젝트 실행 진입점 파일
- 차선 인식 + 장애물 회피 통합
"""
import cv2
import logging
import time

from sensor.object_detector import ObjectDetector
from lane_detection.lane_detector import region_of_interest, compute_histogram

# ==== 메인에서만 쓰는 경량 파라미터(Detector 내부 상수와 충돌 없음) ====
FRAME_WIDTH = 640
FRAME_CENTER = FRAME_WIDTH // 2

# 차선 주행 튜닝 (필요 시 조정)
LANE_TURN_DEADZONE = 30   # |deviation| <= 이 값이면 직진
LOOP_DELAY_S = 0.05       # 메인 루프 주기 (Detector 내부와 별개)

def main():
    print(" ====== main 시작 ====== ")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # 개선된 ObjectDetector (상수는 클래스 내부에 포함되어 있음)
    detector = ObjectDetector()

    try:
        detector.camera.start()
        logging.info("카메라 시작 완료")

        while True:
            # 1) 카메라 프레임 획득
            frame = detector.camera.capture_frame()
            if frame is None:
                time.sleep(LOOP_DELAY_S)
                continue

            # 2) 초음파(필터 적용) 측정 + 장애물 여부(히스테리시스)
            d_ultra = detector.read_distance_filtered()
            obstacle = detector.is_obstacle(d_ultra)

            if obstacle:
                # === 회피 모드 ===
                logging.info("⚠ 장애물 감지됨: %s", f"{d_ultra:.1f} cm" if d_ultra is not None else "None")
                direction = detector.plan_avoidance(frame)

                if direction == "right":
                    print("right turn")
                    # motor.turn_right()
                else:
                    print("left turn")
                    # motor.turn_left()

                time.sleep(detector.AVOID_COOLDOWN_S)
                print("forward")
                # motor.forward()

            else:
                # === 차선 인식 기반 주행 모드 ===
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                roi = region_of_interest(gray)  # 하단 ROI
                hist = compute_histogram(roi)
                lane_center = int(hist.argmax())  # ROI 내 차선의 x-중심
                deviation = lane_center - FRAME_CENTER

                if deviation < -LANE_TURN_DEADZONE:
                    logging.debug("차선 왼쪽 → 좌회전")
                    print("left turn")
                    # motor.turn_left()
                elif deviation > LANE_TURN_DEADZONE:
                    logging.debug("차선 오른쪽 → 우회전")
                    print("right turn")
                    # motor.turn_right()
                else:
                    logging.debug("차선 중앙 → 직진")
                    print("forward")
                    # motor.forward()

            # 3) 루프 주기
            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        print("\n사용자 종료 요청 감지됨")

    finally:
        detector.camera.stop()
        print(" ====== main 끝남 ======")

if __name__ == '__main__':
    main()