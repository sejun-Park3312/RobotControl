from Vision import Vision
import threading
import time
from ProjectPath import PROJECT_PATH

VS = Vision()
VisionThread = threading.Thread(target=VS.Tracking, daemon=True)
VisionThread.start()

start = time.time()
while time.time() - start < 10 :
    time.sleep(1)