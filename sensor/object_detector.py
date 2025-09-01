import logging
import time
from collections import deque

import cv2
import numpy as np

from sensor.hc_sr04p_sensor import HCSR04Sensor
from camera.camera_controller import CameraManager

# ---------- 하드코딩 상수 ----------
TRIG_PIN = 23
ECHO_PIN = 24
THRESHOLD_CM = 30
MARGIN_CM = 5
ULTRA_MIN_CM = 2
ULTRA_MAX_CM = 400
JUMP_REJECT_CM = 50
SAMPLES = 5
ECHO_TIMEOUT_S = 0.04
LOOP_DELAY_S = 0.1
AVOID_COOLDOWN_S = 0.5

BLACK_H_MIN = 0
BLACK_H_MAX = 180
BLACK_S_MAX = 255
BLACK_V_MAX = 50

MIN_CONTOUR_AREA = 300
USE_MORPH = True
MORPH_KERNEL = (5, 5)
# -----------------------------------

class ObjectDetector:

    """
    초음파(이상치 필터) + 카메라(검정색 마스크)로 장애물 판단 및 좌/우 회피
    """

    def read_distance_filtered(self):
        return self._read_ultrasonic_filtered()

    def is_obstacle(self, d_cm):
        return self._apply_hysteresis(d_cm)

    def plan_avoidance(self, frame):
        if frame is None:
            return "left"
        cx, area = self._get_black_center_and_area(frame)
        frame_center = frame.shape[1] // 2
        if area is None or area < MIN_CONTOUR_AREA:
            return "left"
        return "right" if cx < frame_center else "left"

    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN,
                 on_turn_left=None, on_turn_right=None, on_forward=None):
        self.sensor = HCSR04Sensor(trig_pin, echo_pin)
        self.camera = CameraManager()
        self.on_turn_left = on_turn_left or (lambda: print("left turn"))
        self.on_turn_right = on_turn_right or (lambda: print("right turn"))
        self.on_forward = on_forward or (lambda: print("forward"))
        self._obstacle_state = False
        self._last_valid_cm = None
        self._recent_cm = deque(maxlen=5)

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
        try:
            raw = self.sensor.get_distance(timeout=ECHO_TIMEOUT_S, samples=SAMPLES)
        except Exception as e:
            logging.warning("초음파 센서 실패: %s", e)
            raw = None

        if raw is None:
            return self._last_valid_cm
        if raw > ULTRA_MAX_CM:
            return self._last_valid_cm
        if raw < ULTRA_MIN_CM:
            self._push_recent(raw)
            self._last_valid_cm = raw
            return raw
        if self._last_valid_cm is not None and abs(raw - self._last_valid_cm) >= JUMP_REJECT_CM:
            return self._last_valid_cm

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
            return cx, area
        return frame_center, area
