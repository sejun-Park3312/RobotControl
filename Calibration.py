import numpy as np
import os
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from ProjectPath import PROJECT_PATH

class Calibration:
    def __init__(self):
        self.degree = 3
        self.poly = PolynomialFeatures(self.degree)
        self.models = [LinearRegression() for _ in range(3)]  # x, y, z
        self.fit()



    def fit(self):

        # Calibration.py location: ~/PycharmProjects/RobotControl
        Base_Dir = os.path.dirname(os.path.abspath(__file__))
        Data_Path = os.path.join(Base_Dir, 'Calibration_Data')

        Vision_XYZ = np.load(Data_Path + '/Vision_XYZ.npy')
        Robot_xyz = np.load(Data_Path + '/Robot_XYZ.npy')

        X_poly = self.poly.fit_transform(Vision_XYZ)
        for i in range(3):  # x, y, z에 대해 각각 회귀
            self.models[i].fit(X_poly, Robot_xyz[:, i])



    def CorrectPosition(self, Vision_XYZ):
        """보간된 로봇 위치 예측"""
        X_poly = self.poly.transform(Vision_XYZ)
        corrected = np.column_stack([model.predict(X_poly) for model in self.models])
        return corrected
