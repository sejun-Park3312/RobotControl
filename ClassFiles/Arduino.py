import serial
import time
from pynput import keyboard


class Arduino:
    def __init__(self):
        print("Arduino Connecting...")
        self.Running = True

        try:
            self.ArduinoSerial = serial.Serial('/dev/ttyUSB0', 115200)  # 포트 및 속도 설정
            time.sleep(2)
            if self.ArduinoSerial.isOpen():
                print("Arduino Connected!")
            else:
                print("Failed to connect to Arduino.")
                self.Running = False
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            self.Running = False

        self.PWM_OnOff = False
        self.PWM_Value = 255
        print("")

    def Send_PWM(self, PWM_list):
        """
        PWM_list: [pwm_px, pwm_nx, pwm_center]
        """
        if self.Running:
            msg = ",".join(str(int(pwm)) for pwm in PWM_list) + "\n"
            self.ArduinoSerial.write(msg.encode())
        else:
            self.Disconnect()

    def Disconnect(self):
        if self.Running:
            self.Running = False
            self.Send_PWM([0, 0, 0])
            self.ArduinoSerial.write(b"999,999,999\n")
            self.ArduinoSerial.flush()
            self.ArduinoSerial.close()
            print("Arduino Disconnected!")
            print("")

    def ManualPWM(self, Value):
        self.PWM_Value = Value

        def PWM_On(key):
            if key == keyboard.Key.space:
                self.PWM_OnOff = True
            elif key == keyboard.Key.up:
                self.PWM_Value += 1
                print(f"PWM : {self.PWM_Value}")
            elif key == keyboard.Key.down:
                self.PWM_Value -= 1
                print(f"PWM : {self.PWM_Value}")

        def PWM_Off(key):
            if key == keyboard.Key.space:
                self.PWM_OnOff = False
                return

            try:
                if key.char == 'q':  # 'q'로 종료
                    self.Send_PWM([0, 0, 0])
                    self.Running = False
                    return False  # 리스너 종료
            except AttributeError:
                pass

        listener = keyboard.Listener(on_press=PWM_On, on_release=PWM_Off)
        listener.start()

        try:
            while self.Running:
                if self.PWM_OnOff:
                    PWM = [self.PWM_Value, self.PWM_Value, self.PWM_Value]
                    self.Send_PWM(PWM)
                else:
                    self.Send_PWM([0, 0, 0])

                time.sleep(50 / 1000)
        except KeyboardInterrupt:
            self.Send_PWM([0, 0, 0])
            self.Disconnect()

        self.Running = True

# import serial
# import time
# from pynput import keyboard
# from ProjectPath import PROJECT_PATH
# from ClassFiles.Control_Water import Control_Water
#
# class Arduino:
#     def __init__(self):
#         print("Arduino Connecting...")
#         self.Running = True
#         self.ArduinoSerial = self.ArduinoSerial = serial.Serial('/dev/ttyUSB0', 115200)
#         self.PWM_OnOff = False
#         self.PWM_Value = 255
#         time.sleep(2)
#         print("Arduino Connected!")
#         print("")
#         print("")
#
#
#     def Send_PWM(self, PWM_list):
#         """
#         PWM_list: [pwm_px, pwm_nx, pwm_center]
#         """
#         if self.Running:
#             # 리스트를 문자열 "val1,val2,val3\n"로 변환
#             msg = ",".join(str(int(pwm)) for pwm in PWM_list) + "\n"
#             # print(msg)
#             self.ArduinoSerial.write(msg.encode())
#         else:
#             self.Disconnect()
#
#
#
#     def Disconnect(self):
#         if self.Running:
#             self.Running = False
#             self.Send_PWM([0,0,0])
#             self.ArduinoSerial.write(b"999,999,999\n")
#             self.ArduinoSerial.flush()
#             self.ArduinoSerial.close()
#             print("Arduino Disconnected!")
#             print("")
#             print("")
#
#
#     def ManualPWM(self, Value):
#
#         self.PWM_Value = Value
#         def PWM_On(key):
#             if key == keyboard.Key.space:
#                 self.PWM_OnOff = True
#             elif key == keyboard.Key.up:
#                 self.PWM_Value  = self.PWM_Value + 1
#                 print(f"PWM : {self.PWM_Value}")
#             elif key == keyboard.Key.down:
#                 self.PWM_Value  = self.PWM_Value - 1
#                 print(f"PWM : {self.PWM_Value}")
#
#         def PWM_Off(key):
#             if key == keyboard.Key.space:
#                 self.PWM_OnOff = False
#                 return
#
#             # 2️⃣ 일반 키 처리
#             try:
#                 if key.char == 'q':  # 일반 문자 키
#                     self.Send_PWM([0, 0, 0])
#                     self.Running = False
#                     return False  # 리스너 종료
#             except AttributeError:
#                 pass
#
#         listener = keyboard.Listener(on_press=PWM_On, on_release=PWM_Off)
#         listener.start()
#
#         try:
#             while self.Running:
#                 # ESC 누르면 종료
#                 if self.PWM_OnOff:
#                     PWM = [self.PWM_Value, self.PWM_Value, self.PWM_Value]
#                     self.Send_PWM(PWM)
#                     # print(f"On:{PWM}")
#                 else:
#                     self.Send_PWM([0,0,0])
#                     # print("Off")
#
#                 time.sleep(50/1000)
#         except KeyboardInterrupt:
#             self.Send_PWM([0,0,0])
#
#         self.Running = True
#
#
#
