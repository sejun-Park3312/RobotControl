import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino

VS = Vision()
AD = Arduino()


ArduinoThread = threading.Thread(target=AD.ManualPWM, daemon=True)
ArduinoThread.start()


VS.Tracking()