import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision

VS = Vision()
VisionThread = threading.Thread(target=VS.Tracking, daemon=True)
VisionThread.start()

start = time.time()
while time.time() - start < 10 :
    time.sleep(1)