import numpy as np
import threading
import time
import cv2
from RealTimeData_Recorder import RealTimeData_Recorder
from Vision import Vision


VS = Vision()
lock = VS.lock
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

Running = True
currtime = time.time()
while VS.Running:

    if time.time() -currtime > 5:
        with lock:
            Pose = VS.Position
            pos = [round(Pose[0]*1000,2), round(Pose[1]*1000,2), round(Pose[2]*1000,2)]
        print(f"Vision:{pos}")
        currtime = time.time()

RD = RealTimeData_Recorder()
RD.DefineData("Robot_XYZ", {'x', 'y', 'z'})
RD.DefineData("Vision_XYZ", {'x', 'y', 'z'})

