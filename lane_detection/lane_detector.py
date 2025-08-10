"""
lane_utils.py
차선 인식용 유틸 함수들 (ROI, 히스토그램, 시각화)
"""

import cv2
import numpy as np

def region_of_interest(img):
    """
    이미지(그레이스케일 또는 엣지 이미지)의 하단 절반만 마스크.
    """
    height, width = img.shape
    mask = np.zeros_like(img)
    polygon = np.array([[
        (0, height),
        (0, int(height * 0.5)),
        (width, int(height * 0.5)),
        (width, height)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(img, mask)

def compute_histogram(img):
    """
    x축(가로) 방향으로 픽셀 값을 합산해서 1D 히스토그램 반환.
    """
    return np.sum(img, axis=0)

def draw_histogram(hist, width=640, height=200):
    """
    히스토그램을 가시화한 이미지 반환.
    - hist 길이가 width와 다르면 선형 보간해서 맞춤.
    """
    hist = np.asarray(hist)
    # 필요하면 너비에 맞추어 보간
    if hist.size != width:
        hist_scaled = np.interp(np.linspace(0, hist.size - 1, width),
                                np.arange(hist.size),
                                hist)
    else:
        hist_scaled = hist

    hist_img = np.zeros((height, width, 3), dtype=np.uint8)
    max_val = np.max(hist_scaled)
    if max_val == 0:
        return hist_img

    hist_norm = (hist_scaled / max_val) * (height - 1)  # 0..height-1 범위
    for x in range(width):
        h = int(hist_norm[x])
        cv2.line(hist_img, (x, height - 1), (x, height - 1 - h), (255, 255, 255), 1)

    return hist_img
