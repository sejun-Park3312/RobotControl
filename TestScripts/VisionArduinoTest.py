import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino

VS = Vision()
VisionThread = threading.Thread(target=VS.Tracking, daemon=True)
VisionThread.start()

AD = Arduino()
AD.ManualPWM()
