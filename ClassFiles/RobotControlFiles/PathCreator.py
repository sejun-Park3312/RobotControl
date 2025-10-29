import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev


class PathCreator:
    def __init__(self):
        print("Path Creator")
        self.LineNum = 5
        self.CurveNum = 20


    def Cal_CurvePoints(self, StartPoint, EndPoint, CurveCenter, Axis):
        r_1 = StartPoint - CurveCenter
        r_2 = EndPoint - CurveCenter
        theta = np.arccos(np.dot(r_1, r_2)/np.linalg.norm(r_1)/np.linalg.norm(r_2))
        n = self.CurveNum
        CurvePoints = np.zeros((n + 1, 3), dtype=float)
        for i in range(n + 1):
            rotm = (R.from_rotvec(Axis * theta / n * i)).as_matrix()
            CurvePoints[i] = rotm @ r_1 + CurveCenter
        return CurvePoints


    def Abs_RoundPath(self, StartPoint, RelMotions, Radius, PlotOption = False):

        Points = self.Rel2Abs(StartPoint, RelMotions)
        StartPoint = np.array([StartPoint], dtype=float)
        Pvec = np.array(RelMotions, dtype=float)

        LinePoints = []
        CurvePoints = []
        for i in range(Pvec.shape[0] - 1):
            u_1 = Pvec[i] / np.linalg.norm(Pvec[i])
            u_2 = Pvec[i + 1] / np.linalg.norm(Pvec[i + 1])
            u_r = (-u_1 + u_2) / np.linalg.norm(-u_1 + u_2)
            n = np.cross(u_1, u_2)/np.linalg.norm(np.cross(u_1, u_2))

            theta = np.arccos(np.dot(-u_1, u_2)) / 2
            r_center = Points[i + 1] + u_r * Radius[i]/np.sin(theta)
            p_1r = (np.linalg.norm(Pvec[i]) - Radius[i]/np.tan(theta)) * u_1 + Points[i]
            p_2r = Radius[i]/np.tan(theta) * u_2 + Points[i + 1]

            np_CurvePoints = self.Cal_CurvePoints(p_1r, p_2r, r_center, n)
            CurvePoints.append(np_CurvePoints.tolist())

        LineStart = Points[0]
        for i in range(len(CurvePoints)):
            LineEnd = np.array(CurvePoints[i][0])

            np_LinePoints = np.linspace(LineStart, LineEnd, self.LineNum)
            LinePoints.append(np_LinePoints.tolist())
            LineStart = np.array(CurvePoints[i][-1])
        np_LinePoints = np.linspace(LineStart, Points[-1], self.LineNum)
        LinePoints.append(np_LinePoints.tolist())

        TotalPoints = LinePoints[0]
        for i in range(len(CurvePoints)):
            TotalPoints = TotalPoints + CurvePoints[i][1:-1] + LinePoints[i + 1]

        if PlotOption:
            self.plot_3d_points(TotalPoints, connect=True)

        return TotalPoints


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



    def Cal_RelativeMotion(self, Points):

        RelPoints = np.diff(np.array(Points, dtype=float), axis = 0)
        u = RelPoints/np.linalg.norm(RelPoints, axis = 1)[:, np.newaxis]
        u_list = np.zeros([len(Points), 3])
        u_list[0] = u[0]
        u_list[-1] = u[-1]
        RotAngle = np.zeros([len(Points)-1,1])
        for i in range(len(Points)-2):
            u_list[i+1] = (u[i] + u[i+1])/np.linalg.norm(u[i] + u[i+1])
            RotAngle[i] = (np.arcsin(np.dot(np.cross(u_list[i], u_list[i+1]), np.array([0,0,1]))))
        RotAngle[-1] = (np.arcsin(np.dot(np.cross(u_list[-2], u_list[-1]), np.array([0,0,1]))))

        RotAngle = RotAngle * 180/np.pi
        RelMotions = np.concatenate((RelPoints, RotAngle), axis = 1)

        return RelMotions


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
            raise ValueError("points 리스트는 3차원 좌표 [x,y,z]여야 합니다.")

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

    def linear_resample(self, points, num_points=90):
        """
        선형 보간 후 곡선 길이 기준 등간격으로 샘플링
        ------------------------------------------------
        points: np.ndarray, shape (N,D) - 원본 점
        num_points: int - 등간격으로 뽑을 점 개수

        return: np.ndarray, shape (num_points, D)
        """
        points = np.asarray(points)
        n_points, dim = points.shape

        # 1️⃣ 각 구간 길이
        diffs = np.diff(points, axis=0)
        seg_lengths = np.linalg.norm(diffs, axis=1)
        cumlen = np.hstack([[0], np.cumsum(seg_lengths)])
        total_length = cumlen[-1]

        # 2️⃣ 등간격 거리 생성
        target_distances = np.linspace(0, total_length, num_points)

        # 3️⃣ 선형 보간
        resampled = np.zeros((num_points, dim))
        for d in range(dim):
            resampled[:, d] = np.interp(target_distances, cumlen, points[:, d])

        return resampled


    def plot_columns(self, data, x=None, labels=None, title=None):
        """
        n×m 배열을 받아 m개의 그래프를 각각 그리는 함수

        Parameters
        ----------
        data : np.ndarray
            n×m 크기의 배열
        x : np.ndarray, optional
            x축 데이터 (기본값: 0 ~ n-1)
        labels : list of str, optional
            각 그래프의 라벨 리스트
        title : str, optional
            전체 그래프 제목
        """
        data = np.asarray(data)
        n, m = data.shape

        if x is None:
            x = np.arange(n)

        plt.figure(figsize=(8, 5))

        for i in range(m):
            label = labels[i] if labels and i < len(labels) else f"col {i + 1}"
            plt.plot(x, data[:, i], label=label)

        plt.xlabel("Index")
        plt.ylabel("Value")
        if title:
            plt.title(title)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()


    def MakeTrajectory(self, RelativeMotionList, Radius, PlotOption = False):

        StartPoint = [0, 0, 0]
        PointList = RelativeMotionList
        TotalPoints = self.Abs_RoundPath(StartPoint, PointList, Radius, False)
        SampledPoints = self.linear_resample(TotalPoints, 50)

        RelMotions = self.Cal_RelativeMotion(SampledPoints)
        AbsMotions = self.Rel2Abs([0, 0, 0, 0], RelMotions)

        if PlotOption:
            self.plot_3d_points(SampledPoints)
            self.plot_columns(AbsMotions)

        return AbsMotions
