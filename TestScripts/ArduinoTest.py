import threading
import time
import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino

AD = Arduino()
ArduinoThread = threading.Thread(target=AD.ManualPWM, daemon=True)
ArduinoThread.start()

while AD.Running:
    if keyboard.is_pressed('esc'):
        AD.Disconnect()
        break