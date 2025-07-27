import threading
import time
import cv2
import threading
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.Control import Control


VS = Vision()
AD = Arduino()
CT = Control()

lock = VS.lock
Thread_Vision = threading.Thread(target=VS.Tracking, daemon=True)
Thread_Vision.start()

Running = True
StartTime = time.time()
while Running:

    if cv2.waitKey(1) == 27 or VS.Running == False:
        AD.Running = False
        AD.Disconnect()
        Running = False
        print("Test Stopped!")
        break

    time.sleep(CT.SamplingTime)
    with lock:
        CT.Z_Target = VS.Z
        PWM = CT.Get_PWM()

    AD.Send_PWM(PWM)
    print(f"PWM:{PWM}")

AD.Disconnect()
print("TotalSystem Ended!")
