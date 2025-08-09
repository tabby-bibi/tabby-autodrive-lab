import torch 
import torch.nn as nn
import torchvision.transforms as transforms
import cv2
import numpy as np
import time
import pigpio
from PIL import Image
from picamera2 import Picamera2
import csv
import os
from datetime import datetime

# === 설정 ===
SERVO_PIN = 18
IN1 = 12
IN2 = 13
PWM_SPEED = 110
MODEL_PATH = "model_regression.pth"
IMAGE_SIZE = (160, 120)
SAVE_DIR = "drive_log"

os.makedirs(SAVE_DIR, exist_ok=True)

# === 로그 파일 ===
csv_file = open(os.path.join(SAVE_DIR, "log.csv"), "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["timestamp", "angle", "image_filename"])

# === 영상 녹화 초기화 ===
timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
video_path = os.path.join(SAVE_DIR, f"drive_{timestamp_str}.avi")
fourcc = cv2.VideoWriter_fourcc(*"XVID")
video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (680, 480))

# === GPIO 초기화 ===
pi = pigpio.pi()
pi.set_mode(IN1, pigpio.OUTPUT)
pi.set_mode(IN2, pigpio.OUTPUT)
pi.set_mode(SERVO_PIN, pigpio.OUTPUT)

def set_servo_angle(angle):
    angle = max(0, min(180, angle))
    pulsewidth = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(SERVO_PIN, pulsewidth)

def motor_forward(speed=PWM_SPEED):
    pi.set_PWM_dutycycle(IN1, speed)
    pi.set_PWM_dutycycle(IN2, 0)

def motor_stop():
    pi.set_PWM_dutycycle(IN1, 0)
    pi.set_PWM_dutycycle(IN2, 0)

# === 모델 정의 ===
class RegressionCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2), nn.ReLU(),
            nn.Conv2d(16, 32, 5, stride=2), nn.ReLU(),
            nn.Conv2d(32, 64, 5, stride=2), nn.ReLU()
        )
        self.flatten = nn.Flatten()
        self.fc = nn.Sequential(
            nn.Linear(64 * 17 * 12, 100), nn.ReLU(),
            nn.Linear(100, 1)
        )

    def forward(self, x):
        x = self.conv(x)
        x = self.flatten(x)
        x = self.fc(x)
        return x

# === 모델 로드 ===
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = RegressionCNN().to(device)
model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
model.eval()

# === 이미지 전처리 ===
transform = transforms.Compose([
    transforms.Resize((160, 120)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
])

# === 카메라 초기화 ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (680, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.configure("preview")
picam2.start()

# === 주행 시작 ===
print("주행 시작 (Ctrl+C 또는 'q'로 종료)")

try:
    while True:
        # 프레임 캡처
        frame = picam2.capture_array()
        pil_image = Image.fromarray(frame)
        input_tensor = transform(pil_image).unsqueeze(0).to(device)

        # 모델 예측
        with torch.no_grad():
            output = model(input_tensor)
            norm_angle = output.item()
            angle = int(norm_angle * 180)
            angle = max(0, min(180, angle))

        # 조향 & 모터 전진
        set_servo_angle(angle)
        motor_forward()

        # 영상 저장
        video_writer.write(frame)

        # 이미지 파일 저장
        img_filename = f"frame_{int(time.time() * 1000)}.jpg"
        img_path = os.path.join(SAVE_DIR, img_filename)
        cv2.imwrite(img_path, frame)

        # 로그 기록
        csv_writer.writerow([datetime.now().isoformat(), angle, img_filename])

        # 화면 표시
        cv2.putText(frame, f"Steering: {angle}", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.imshow("Live Drive", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("수동 종료")
            break

except KeyboardInterrupt:
    print("Ctrl+C 종료")

finally:
    print("안전 종료 중...")
    motor_stop()
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
    picam2.stop()
    video_writer.release()
    csv_file.close()
    cv2.destroyAllWindows()
    print("로그와 영상이 저장되었습니다.")
