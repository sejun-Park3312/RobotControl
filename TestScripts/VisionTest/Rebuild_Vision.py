from ClassFiles.Vision import Vision
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
import cv2
import numpy as np
import time

VS = Vision()
Rebuilt_VisionData = RealTimeData_Recorder()

DataName = "Rebuilt_VisionData"
Rebuilt_VisionData.DefineData(DataName, ["X", "Y", "Z"])


def Get_Center(frame, prev_center):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 50, 50])
    upper = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) == 0:
        return None

    # 모든 contour 중심 계산
    centers = []
    for c in cnts:
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centers.append(np.array([cx, cy], dtype=np.float64))

    # 이전 프레임이 있다면 → 가장 가까운 blob 선택
    if prev_center is not None and len(centers) > 1:
        dists = [np.linalg.norm(c - prev_center) for c in centers]
        best = centers[np.argmin(dists)]  # 이전 위치에 가장 가까운 것 선택
    else:
        # 첫 프레임은 가장 큰 contour
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        best = np.array([
            int(M["m10"] / M["m00"]),
            int(M["m01"] / M["m00"])
        ], dtype=np.float64)

    return best


def Get_Position(Cam1, Cam2, PrevCenter):

    Position = []

    ret1 = Cam1[0]
    frame1 = Cam1[1]
    ret2 = Cam2[0]
    frame2 = Cam2[1]

    prev_center1 = PrevCenter[0]
    prev_center2 = PrevCenter[1]

    # Undistorting
    frame1_undist = cv2.undistort(frame1, VS.K1, VS.D1)
    frame2_undist = cv2.undistort(frame2, VS.K2, VS.D2)

    # Cutting ROI
    roi_frame1 = frame1_undist[VS.ROI_1[1]:VS.ROI_1[1] + VS.ROI_1[3],
                 VS.ROI_1[0]:VS.ROI_1[0] + VS.ROI_1[2]]
    roi_frame2 = frame2_undist[VS.ROI_2[1]:VS.ROI_2[1] + VS.ROI_2[3],
                 VS.ROI_2[0]:VS.ROI_2[0] + VS.ROI_2[2]]

    # Get Center
    pt1 = Get_Center(roi_frame1, prev_center1)
    pt2 = Get_Center(roi_frame2, prev_center2)

    if pt1 is not None and pt2 is not None:
        # Original Frame tuple
        pt1_orig = np.array((pt1[0] + VS.ROI_1[0], pt1[1] + VS.ROI_1[1]), dtype=np.float64)
        pt2_orig = np.array((pt2[0] + VS.ROI_2[0], pt2[1] + VS.ROI_2[1]), dtype=np.float64)

        # Triangulation
        pts4d = cv2.triangulatePoints(VS.P1, VS.P2, pt1_orig.T, pt2_orig.T)
        pts3d = (pts4d / pts4d[3])[:3].flatten()
        x_Cam1, y_Cam1, z_Cam1 = pts3d.flatten()

        P_Cam1 = np.array([[x_Cam1], [y_Cam1], [z_Cam1], [1]])
        P_World = VS.T_World2Cam1 @ P_Cam1

        if VS.Calibration_ONOFF:
            CorrectedPosition = VS.Calibration.CorrectPosition([[P_World[0][0], P_World[1][0], P_World[2][0]]])
            x = CorrectedPosition[0][0]
            y = CorrectedPosition[0][1]
            z = CorrectedPosition[0][2]
        else:
            x = P_World[0][0]
            y = P_World[1][0]
            z = P_World[2][0]
        Position = [x, y, z]

        # Visualization
        if pt1 is not None:
            cv2.circle(frame1_undist, tuple(pt1_orig.astype(int)), 5, (0, 255, 0), -1)
        if pt2 is not None:
            cv2.circle(frame2_undist, tuple(pt2_orig.astype(int)), 5, (0, 255, 0), -1)

        if VS.GUI:
            # Show XYZ
            cv2.putText(frame1_undist, f"3D: X={x * 1000:.2f} Y={y * 1000:.2f} Z={z * 1000:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show ROI
            cv2.rectangle(frame1_undist, (VS.ROI_1[0], VS.ROI_1[1]),
                          (VS.ROI_1[0] + VS.ROI_1[2], VS.ROI_1[1] + VS.ROI_1[3]), (255, 0, 0), 2)
            cv2.rectangle(frame2_undist, (VS.ROI_2[0], VS.ROI_2[1]),
                          (VS.ROI_2[0] + VS.ROI_2[2], VS.ROI_2[1] + VS.ROI_2[3]), (255, 0, 0), 2)

            # Show Image
            cv2.imshow("Camera 1", frame1_undist)
            cv2.imshow("Camera 2", frame2_undist)
            cv2.waitKey(1)

        PrevCenter = [pt1, pt2]
        # Return
        return Position, PrevCenter

    return None, None

PrevCenter = [None, None]
cap1 = cv2.VideoCapture('../../Data/History/1109_water_PNU/PNU_CAM1.mp4')
cap2 = cv2.VideoCapture('../../Data/History/1109_water_PNU/PNU_CAM2.mp4')
StartTime = time.time()

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    if not ret1:
        break

    Cam1 = [ret1, frame1]
    Cam2 = [ret2, frame2]
    Position, PrevCenter = Get_Position(Cam1, Cam2, PrevCenter)
    if Position is not None and PrevCenter is not None:
        Rebuilt_VisionData.AppendData(DataName, Position)

    time.sleep(5/1000)


# Save Data
Rebuilt_VisionData.Data[DataName]["Time"]["StartTime"] = StartTime
Rebuilt_VisionData.SaveData(DataName, DataName, SavePath='Results')