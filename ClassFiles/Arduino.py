import serial
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH

class Arduino:
    def __init__(self):
        print("Arduino Connecting...")
        self.Running = True
        self.ArduinoSerial = self.ArduinoSerial = serial.Serial('/dev/ttyUSB0', 115200)
        self.PWM_OnOff = False
        time.sleep(2)
        print("Arduino Connected!")
        print("")
        print("")


    def Send_PWM(self, PWM):
        if self.Running == True:
            self.ArduinoSerial.write(f"{PWM}\n".encode())
        else:
            self.Disconnect()


    def Disconnect(self):
        if self.Running:
            self.Running = False
            self.ArduinoSerial.write(f"{0}\n".encode())
            self.ArduinoSerial.write(b"999\n")
            self.ArduinoSerial.flush()
            self.ArduinoSerial.close()
            print("Arduino Disconnected!")
            print("")
            print("")


    def ManualPWM(self):

        def PWM_On(key):
            if key == keyboard.Key.space:
                self.PWM_OnOff = True

        def PWM_Off(key):
            if key == keyboard.Key.space:
                self.PWM_OnOff = False
            elif key == keyboard.Key.esc:
                self.Disconnect()
                return False  # 리스너 종료

        listener = keyboard.Listener(on_press=PWM_On, on_release=PWM_Off)
        listener.start()

        PWM = 200
        OnOff = False
        try:
            while self.Running:
                # ESC 누르면 종료
                if self.PWM_OnOff:
                    self.Send_PWM(PWM)
                    print("On")
                else:
                    self.Send_PWM(0)
                    print("Off")

                time.sleep(50/1000)
        except KeyboardInterrupt:
            self.Disconnect()
