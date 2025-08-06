import numpy as np
import os
from ProjectPath import PROJECT_PATH
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

class Calibration:
    def __init__(self):
        self.degree = 3
        self.poly = PolynomialFeatures(self.degree)
        self.models = [LinearRegression() for _ in range(3)]  # x, y, z
        self.fit()



    def fit(self):
        # Paths...
        Calibration_path = os.path.dirname(os.path.abspath(__file__)) # .../RobotControl/ClassFiles
        Project_path = os.path.dirname(Calibration_path) # .../RobotControl
        Data_path = os.path.join(Project_path, 'Data', 'Calibration_Data/Ref_Data') # .../RobotControl/Data

        Vision_XYZ = np.load(Data_path + '/Vision_XYZ.npy')
        Robot_xyz = np.load(Data_path + '/Robot_XYZ.npy')

        X_poly = self.poly.fit_transform(Vision_XYZ)
        for i in range(3):  # x, y, z에 대해 각각 회귀
            self.models[i].fit(X_poly, Robot_xyz[:, i])



    def CorrectPosition(self, Vision_XYZ):
        """보간된 로봇 위치 예측"""
        X_poly = self.poly.transform(Vision_XYZ)
        corrected = np.column_stack([model.predict(X_poly) for model in self.models])
        return corrected
