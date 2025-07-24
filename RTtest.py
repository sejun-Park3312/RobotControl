from RobotTransformation import RobotTransformation
RT = RobotTransformation()
T = RT.Pose2Tmatrix([0,0,0,0,0,0])
print(T)

T_b2r = RT.Pose2Tmatrix(RT.RefPose)
print(T_b2r)
print()


T_r2E = RT.Pose2Tmatrix([529.0, 104.5, 541.5, 0,0,0])
print(T_r2E)
print()

T_r2E = RT.Pose2Tmatrix([1.8788476904927492e-13, 34.5, 1452.5, 3.552713678800501e-15, 1.2722218874358041e-14, 7.888609052210118e-31])
print(T_r2E)
print()


T_b2E = T_b2r @ T_r2E
print(T_b2E)
print()

(121.0, 34.5, 1331.5, 45.0, 90.0, 45.0)