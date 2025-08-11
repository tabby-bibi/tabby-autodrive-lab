'''
    카메라 초기화 및 설정 코드 파일 입니다.
'''
import cv2
from picamera2 import picamera2

class CameraManager:
    def __init__(self, resolution=(640, 480), fps=15, exposure_time=10000, colour_gains=(1.5, 1.5)):
        self.picam2 = picamera2()
        self.config = self.picam2.create_preview_configuration(
            main={"format": "BGR888", "size": resolution},
            controls={"FrameRate": fps}
        )
        self.picam2.configure(self.config)

        # Manual exposure and white balance
        self.picam2.set_controls({
            "AwbEnable": False,
            "AeEnable": False,
            "ExposureTime": exposure_time,
            "AnalogueGain": 1.0,
            "ColourGains": colour_gains
        })

    def start(self):
        try:
            print(" ====== camera started ====== ")
            self.picam2.start()
        except Exception as e:
            print(f"Error while starting camera: {e}")

    def stop(self):
        print(" ====== camera stopped ====== ")
        self.picam2.stop()

    def capture_frame(self):
        try:
            frame = self.picam2.capture_array()
            if frame is None:
                print("Warning: Captured frame is None.")
            return frame
        except Exception as e:
            print(f"Error capturing frame: {e}")
            return None

    def release(self):
        self.stop()
        cv2.destroyAllWindows()

    def __del__(self):
        self.release()
