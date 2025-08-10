'''
    차선 인식 기능을 담당하는 코드입니다.
'''

import cv2
import numpy as np
from picamera2 import Picamera2
from car_controller import CarController

'''
Custom camera setup to avoid exposure issues on Raspberry Pi camera.
This script uses manual exposure and white balance settings.
'''

# Set Region of Interest (ROI)
def region_of_interest(img):
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

# Compute histogram of pixel intensities along the x-axis
def compute_histogram(img):
    return np.sum(img, axis=0)

# Visualize histogram as an image
def draw_histogram(hist):
    hist_img = np.zeros((200, 640, 3), dtype=np.uint8)
    max_val = np.max(hist)
    if max_val == 0:
        return hist_img

    hist_norm = (hist / max_val) * 200  # Normalize
    for x in range(len(hist)):
