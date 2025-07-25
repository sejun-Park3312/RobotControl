import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

class PolynomialInterpolator3D:
    def __init__(self, degree=3):
        self.degree = degree
        self.poly = PolynomialFeatures(degree)
        self.models = [LinearRegression() for _ in range(3)]  # x, y, z

    def fit(self, vision_xyz, robot_xyz):
        """보간 모델 학습"""
        X_poly = self.poly.fit_transform(vision_xyz)
        for i in range(3):  # x, y, z에 대해 각각 회귀
            self.models[i].fit(X_poly, robot_xyz[:, i])

    def predict(self, vision_xyz):
        """보간된 로봇 위치 예측"""
        X_poly = self.poly.transform(vision_xyz)
        predicted = np.column_stack([model.predict(X_poly) for model in self.models])
        return predicted
