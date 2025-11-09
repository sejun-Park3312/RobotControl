import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
import DR_init
# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "a0509_SJ_Custom"
# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

class PathMaker:
    def __init__(self):
        self.CurveNum = 7

    def Cal_CurvePoints(self, StartPoint, EndPoint, CurveCenter, Axis):
        r_1 = StartPoint - CurveCenter
        r_2 = EndPoint - CurveCenter
        theta = np.arccos(np.dot(r_1, r_2)/np.linalg.norm(r_1)/np.linalg.norm(r_2))
        n = self.CurveNum -1
        CurvePoints = np.zeros((n + 1, 3), dtype=float)
        for i in range(n + 1):
            rotm = (R.from_rotvec(Axis * theta / n * i)).as_matrix()
            CurvePoints[i] = rotm @ r_1 + CurveCenter
        return CurvePoints, theta


    def Rel2Abs(self, StartPoint, RelMotion):
        np_StartPoint = np.array(StartPoint, dtype=float)
        np_AbsPoints = np.zeros([len(RelMotion)+1,np_StartPoint.shape[0]])
        np_AbsPoints[0] = np_StartPoint
        PrevPoint = np_StartPoint
        for i in range(len(RelMotion)):
            np_AbsPoints[i+1] = np.array(RelMotion[i], dtype=float) + PrevPoint
            PrevPoint = np_AbsPoints[i+1]

        AbsPoints = np_AbsPoints.tolist()
        return AbsPoints


    def Abs2Rel(self, AbsPoints):

        RelPoints = np.diff(AbsPoints, axis=0)

        return RelPoints


    def RoundPath(self, RelMotionList, Radius, PlotOption = False):

        Points = self.Rel2Abs([0,0,0], RelMotionList)
        StartPoint = np.array([0,0,0], dtype=float)
        Pvec = np.array(RelMotionList, dtype=float)

        LinePoints = []
        CurvePoints = []
        thetaList = []
        for i in range(Pvec.shape[0] - 1):
            u_1 = Pvec[i] / np.linalg.norm(Pvec[i])
            u_2 = Pvec[i + 1] / np.linalg.norm(Pvec[i + 1])
            u_r = (-u_1 + u_2) / np.linalg.norm(-u_1 + u_2)
            n = np.cross(u_1, u_2)/np.linalg.norm(np.cross(u_1, u_2))

            theta = np.arccos(np.dot(-u_1, u_2)) / 2
            r_center = Points[i + 1] + u_r * Radius[i]/np.sin(theta)
            p_1r = (np.linalg.norm(Pvec[i]) - Radius[i]/np.tan(theta)) * u_1 + Points[i]
            p_2r = Radius[i]/np.tan(theta) * u_2 + Points[i + 1]

            np_CurvePoints, theta = self.Cal_CurvePoints(p_1r, p_2r, r_center, n)
            thetaList.append(theta)
            CurvePoints.append(np_CurvePoints.tolist())

        LineStart = Points[0]
        for i in range(len(CurvePoints)):
            LineEnd = np.array(CurvePoints[i][0])

            np_LinePoints = np.linspace(LineStart, LineEnd, 2)
            LinePoints.append(np_LinePoints.tolist())
            LineStart = np.array(CurvePoints[i][-1])
        np_LinePoints = np.linspace(LineStart, Points[-1], 2)
        LinePoints.append(np_LinePoints.tolist())

        TotalPoints = LinePoints[0]
        for i in range(len(CurvePoints)):
            TotalPoints = TotalPoints + CurvePoints[i][1:-1] + LinePoints[i + 1]

        if PlotOption:
            self.plot_3d_points(TotalPoints, connect=True)

        return LinePoints, CurvePoints, thetaList


    def MakeBlendOrder(self, LinePoints, CurvePoinst, thetaList):

        nL = len(LinePoints)
        nC = len(CurvePoinst)
        posx()




    def plot_3d_points(self, points, connect=False, point_color='red', line_color='blue'):
        """
        points: list of [x, y, z], 예: [[x1,y1,z1], [x2,y2,z2], ...]
        connect: True이면 점들을 순서대로 선으로 연결
        point_color: 점 색
        line_color: 선 색
        """
        # 리스트 → np 배열
        points = np.array(points)
        if points.shape[1] != 3:
            points = points[:,0:3]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 점 그리기
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=point_color, s=50)

        # 선으로 연결
        if connect:
            ax.plot(points[:, 0], points[:, 1], points[:, 2], c=line_color)

        # 축 라벨
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Points')

        # 같은 비율로 축 설정
        max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                              points[:, 1].max() - points[:, 1].min(),
                              points[:, 2].max() - points[:, 2].min()]).max() / 2.0

        mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
        mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
        mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.show()
