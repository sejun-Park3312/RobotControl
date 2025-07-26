import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino

AD = Arduino()
AD.ManualPWM()
