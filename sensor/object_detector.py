import logging
import time
from collections import deque

import cv2
import numpy as np

from sensor.hc_sr04p_sensor import HCSR04Sensor
from camera.camera_controller import CameraManager
from constants import (
    TRIG_PIN, ECHO_PIN,
    THRESHOLD_CM, MARGIN_CM,
    ULTRA_MIN_CM, ULTRA_MAX_CM, JUMP_REJECT_CM,
    SAMPLES, ECHO_TIMEOUT_S,
    LOOP_DELAY_S, AVOID_COOLDOWN_S,
    BLACK_H_MIN, BLACK_H_MAX, BLACK_S_MAX, BLACK_V_MAX,
    MIN_CONTOUR_AREA, USE_MORPH, MORPH_KERNEL,
)

class ObjectDetector:

    """
    초음파(이상치 필터) + 카메라(검정색 마스크)로 장애물 판단 및 좌/우 회피
    """

    def read_distance_filtered(self):
        """초음파 값(이상치/점프 거부/중앙값) 필터 적용 후 반환"""
        return self._read_ultrasonic_filtered()

    def is_obstacle(self, d_cm):
        """히스테리시스로 장애물 여부 판정(True/False)"""
        return self._apply_hysteresis(d_cm)

    def plan_avoidance(self, frame):
        """
        프레임에서 '검정' 장애물 중심을 찾고,
        회피 방향을 'left' 또는 'right'로 알려줌
        """
        if frame is None:
            return "left"  # 프레임 없으면 기본 좌회전

        cx, area = self._get_black_center_and_area(frame)
        frame_center = frame.shape[1] // 2
        if area is None or area < self.MIN_CONTOUR_AREA:
            return "left"  # 검출 불확실 → 보수적으로 좌회전
        return "right" if cx < frame_center else "left"


    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN,
                 on_turn_left=None, on_turn_right=None, on_forward=None):
        # HW
        self.sensor = HCSR04Sensor(trig_pin, echo_pin)
        self.camera = CameraManager()
        # Motor callbacks
        self.on_turn_left = on_turn_left or (lambda: print("left turn"))
        self.on_turn_right = on_turn_right or (lambda: print("right turn"))
        self.on_forward = on_forward or (lambda: print("forward"))

        # States
        self._obstacle_state = False
        self._last_valid_cm = None
        self._recent_cm = deque(maxlen=5)  # 통계용

    def start(self):
        logging.info("장애물 감지 시작")
        self.camera.start()

    def stop(self):
        try:
            self.camera.stop()
        except Exception:
            pass
        logging.info("감지 로직 종료")

    def detect_loop(self):
        try:
            self.start()
            while True:
                d_ultra = self._read_ultrasonic_filtered()
                logging.debug("[ULTRA] %s", f"{d_ultra:.1f} cm" if d_ultra is not None else "None")

                obstacle = self._apply_hysteresis(d_ultra)

                if obstacle:
                    logging.info("⚠ 장애물 감지 (ultra=%.1f cm)", d_ultra if d_ultra else -1)
                    self._avoid_with_camera_black()
                    time.sleep(AVOID_COOLDOWN_S)
                else:
                    self.on_forward()

                time.sleep(LOOP_DELAY_S)
        except KeyboardInterrupt:
            logging.info("사용자 중단")
        finally:
            self.stop()

    # ---------- Ultrasonic Filtering ----------

    def _read_ultrasonic_filtered(self):
        """
        1) None → 무시(직전값 유지)
        2) 범위 밖(>ULTRA_MAX_CM) → 무시(800cm 등)
        3) 너무 가까움(<ULTRA_MIN_CM) → 즉시 장애물로 인식되도록 해당 값 반환
        4) 점프 거부: 이전 유효값 대비 JUMP_REJECT_CM 이상 차이면 무시
        5) 최근값 deque에 넣고 중앙값으로 완화
        """
        try:
            raw = self.sensor.get_distance(timeout=ECHO_TIMEOUT_S, samples=SAMPLES)
        except Exception as e:
            logging.warning("초음파 센서 실패: %s", e)
            raw = None

        # 1) 미측정
        if raw is None:
            logging.debug("ULTRA: None (skip, keep last)")
            return self._last_valid_cm

        # 2) 범위 밖
        if raw > ULTRA_MAX_CM:
            logging.debug("ULTRA: %.1fcm > ULTRA_MAX → reject", raw)
            return self._last_valid_cm

        # 3) 너무 가까움 → 즉시 장애물로 쓰도록 그대로 채택
        if raw < ULTRA_MIN_CM:
            logging.debug("ULTRA: %.1fcm < ULTRA_MIN → immediate obstacle", raw)
            self._push_recent(raw)
            self._last_valid_cm = raw
            return raw

        # 4) 점프 거부
        if self._last_valid_cm is not None:
            if abs(raw - self._last_valid_cm) >= JUMP_REJECT_CM:
                logging.debug("ULTRA jump reject: raw=%.1f last=%.1f diff>=%.1f",
                              raw, self._last_valid_cm, JUMP_REJECT_CM)
                return self._last_valid_cm

        # 5) 중앙값 완화
        self._push_recent(raw)
        median_cm = self._median_recent()
        self._last_valid_cm = median_cm
        return median_cm

    def _push_recent(self, val):
        self._recent_cm.append(val)

    def _median_recent(self):
        arr = list(self._recent_cm)
        return np.median(arr) if arr else None

    # ---------- Obstacle Decision (Hysteresis) ----------

    def _apply_hysteresis(self, d_cm):
        if d_cm is None:
            return self._obstacle_state
        if not self._obstacle_state and d_cm <= THRESHOLD_CM:
            self._obstacle_state = True
        elif self._obstacle_state and d_cm >= (THRESHOLD_CM + MARGIN_CM):
            self._obstacle_state = False
        return self._obstacle_state

    # ---------- Vision: Black Object ----------

    def _avoid_with_camera_black(self):
        frame = self.camera.capture_frame()
        if frame is None:
            logging.warning("카메라 프레임 없음 → 기본 회피(좌회전)")
            self.on_turn_left()
            return

        cx, area = self._get_black_center_and_area(frame)
        frame_center = frame.shape[1] // 2

        if area is None or area < MIN_CONTOUR_AREA:
            logging.info("검정 컨투어 미약/없음(area=%s) → 기본 회피(좌회전)", area)
            self.on_turn_left()
            return

        if cx < frame_center:
            logging.info("⬅ 검정 장애물 중심이 좌측 → 우회전")
            self.on_turn_right()
        else:
            logging.info("➡ 검정 장애물 중심이 우측 → 좌회전")
            self.on_turn_left()

    def _get_black_center_and_area(self, frame):
        """
        HSV에서 V 낮은 영역을 '검정'으로 간주. 모폴로지로 노이즈 제거.
        반환: (cx, area) / 없으면 (frame_center, None)
        """
        h, w = frame.shape[:2]
        frame_center = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([BLACK_H_MIN, 0, 0], dtype=np.uint8)
        upper = np.array([BLACK_H_MAX, BLACK_S_MAX, BLACK_V_MAX], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        if USE_MORPH:
            k = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return frame_center, None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            # 디버깅 오버레이(원하면 주석 해제)
            # cv2.circle(frame, (cx, int(M["m01"]/M["m00"])), 8, (0, 0, 255), -1)
            # cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
            return cx, area
        return frame_center, area
