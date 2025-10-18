import time

from openpyxl.chart.data_source import NumData

from gdx import gdx
from ProjectPath import PROJECT_PATH

class MagnetFieldSensor():
    def __init__(self):
        self.Sensor = gdx.gdx()

        # GDX-3MG 013000L6/GDX-3MG 015009F6
        self.Sensor.open(connection='ble', device_to_open="GDX-3MG 015009F6")
        if self.Sensor.ble_open:
            self.Connect = True
            print("Connected!")
            print("")
        else:
             self.Connect = False
             print("Connection Failed...")
             print("")
             return
        self.Sensor.select_sensors([1,2,3])

        # MeasuringTime[sec]
        self.MeasureTime = 2
        self.MeasureTimeGap = 0.01
        self.WaitingTime = 1
        time.sleep(3)


    def Measure(self):

        # print("Sensing...")
        time.sleep(self.WaitingTime)
        self.Sensor.start(self.MeasureTimeGap*1000)
        data = []
        NumData = 0
        curr_time = time.time()
        while time.time() - curr_time < self.MeasureTime:
            Measured = self.Sensor.read()
            if Measured:
                data.append(Measured)
                NumData += 1
            time.sleep(self.MeasureTimeGap)

        self.Sensor.stop()
        avg_data = [sum(m[i] for m in data) / NumData for i in range(3)]
        # print(f"Data : {[round(x,3) for x in avg_data]}")
        # print("")

        return avg_data

