import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.MagnetFieldSensor import MagnetFieldSensor
from ClassFiles.Arduino import Arduino
from ClassFiles.ManualController import ManualController

# Preprocessing
GDX = MagnetFieldSensor()
AD = Arduino()
MC = ManualController()

def Measure():
    print("Sensing...")
    Data = GDX.Measure()
    print(f"{[round(x,3) for x in Data]} [mT]")
    print("")

def PWM(Value):
    AD.Send_PWM([Value, Value, Value])
    print(f"PWM : {Value}")
    print("")

# Define Handle
Handle = {"GDX": GDX,
          'AD': AD,
          'Sense': Measure,
          'PWM': PWM}

MC.AddHandle(Handle)

if GDX.Connect == True:
    MC.Start()
else:
    print("Failed to Start Control...")


print("THE END!")
