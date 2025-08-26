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
        self.ManualPWM_Value = [255, 255, 255]
        time.sleep(2)
        print("Arduino Connected!")
        print("")
        print("")


    def Send_PWM(self, PWM_list):
        """
        PWM_list: [pwm_px, pwm_nx, pwm_center]
        """
        if self.Running:
            # 리스트를 문자열 "val1,val2,val3\n"로 변환
            msg = ",".join(str(int(pwm)) for pwm in PWM_list) + "\n"
            self.ArduinoSerial.write(msg.encode())
        else:
            self.Disconnect()


    def Disconnect(self):
        if self.Running:
            self.Running = False
            self.Send_PWM([0,0,0])
            self.ArduinoSerial.write(b"999,999,999\n")
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

        PWM = self.ManualPWM_Value
        OnOff = False
        try:
            while self.Running:
                # ESC 누르면 종료
                if self.PWM_OnOff:
                    self.Send_PWM(PWM)
                    print(f"On:{PWM}")
                else:
                    self.Send_PWM([0,0,0])
                    print("Off")

                time.sleep(50/1000)
        except KeyboardInterrupt:
            self.Disconnect()
