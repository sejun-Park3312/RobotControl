import threading
import cv2
from Vision import Vision


VS = Vision()
lock = VS.lock
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
