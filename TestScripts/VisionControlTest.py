import threading
import time
import cv2
import threading
from ProjectPath import PROJECT_PATH
from simple_pid import PID
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.Control import Control


VS = Vision()
AD = Arduino()
CT = Control()
CT.alpha = 1
CT.Kp = 1e-1/2
CT.Kd = 1e-2
CT.Ki = 0
CT.pid = PID(Kp=CT.Kp, Kd=CT.Kd, Ki=CT.Ki, setpoint = 0)

lock = VS.lock
Thread_Vision = threading.Thread(target=VS.Tracking, daemon=True)
Thread_Vision.start()

Running = True
StartTime = time.time()
while Running:

    if cv2.waitKey(1) == 27 or VS.Running == False:
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
