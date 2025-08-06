import numpy as np
import threading
import time
import cv2
from ProjectPath import PROJECT_PATH
from ClassFiles.RobotController import RobotController
from ClassFiles.Vision import Vision


VS = Vision()
lock = VS.lock
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

RC = RobotController()
RC.Controller()

