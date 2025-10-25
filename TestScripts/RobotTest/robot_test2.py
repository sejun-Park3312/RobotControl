#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import threading, sys
import numpy as np
from scipy.interpolate import CubicSpline

sys.dont_write_bytecode = True
sys.path.append("/home/msrl/catkin_ws/src/doosan-robot/common/imp")

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "a0509_custom3"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

# -----------------------------
# ROS 콜백 및 종료
# -----------------------------
def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1
    if msgRobotState_cb.count % 100 == 0:
        rospy.loginfo("________ ROBOT STATUS ________")
        print("current_posj:", msg.current_posj)
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()

# -----------------------------
# 스플라인 샘플링 함수
# -----------------------------
def sample_path(path_points, num_samples=90):
    """
    path_points: Nx3 array of (x, y, z)
    return: list of posx objects
    """
    t = np.arange(len(path_points))
    cs_x = CubicSpline(t, path_points[:,0])
    cs_y = CubicSpline(t, path_points[:,1])
    cs_z = CubicSpline(t, path_points[:,2])

    ts = np.linspace(0, len(path_points)-1, num_samples)
    sampled_points = np.stack([cs_x(ts), cs_y(ts), cs_z(ts)], axis=1)

    # posx 객체로 변환 (rx, ry, rz는 0,180,0 고정)
    return [posx(p[0], p[1], p[2], 0, 180, 0) for p in sampled_points]

# -----------------------------
# 메인
# -----------------------------
if __name__ == "__main__":
    rospy.init_node('single_robot_spline_py')
    rospy.on_shutdown(shutdown)

    set_robot_mode  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)

    # ROS subscriber 쓰레드
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True
    t1.start()

    # 로봇 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 기본 속도/가속도
    set_velx(30, 20)
    set_accx(60, 40)

    # -----------------------------
    # 임의 path 정의 (3D)
    # -----------------------------
    path_points = np.array([
        [400, 500, 800],
        [420, 520, 780],
        [450, 540, 750],
        [480, 580, 720],
        [500, 600, 700]
    ])

    # 스플라인 샘플링
    sampled_posx = sample_path(path_points, num_samples=90)

    # -----------------------------
    # 연속 이동
    # -----------------------------
    while not rospy.is_shutdown():
        movesx(sampled_posx, vel=50, acc=100)  # task-space 연속 이동
        rospy.sleep(1)  # 반복 시 잠깐 쉬어주기

    print("good bye!")
