## <<Terminal Preprocessing>>
## <Start Gazebo with ROS>
# cd ~/catkin_ws
# source devel/setup.bash
# roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509

#!/usr/bin/env python3
import rospy
from dsr_msgs.srv import MoveJoint, MoveHome, GetRobotState
from dsr_msgs.srv import GetCurrentPose

def wait_until_motion_done():
    rospy.wait_for_service('/dsr01a0509/system/get_robot_state')
    rospy.wait_for_service('/dsr01a0509/system/get_current_pose')
    get_state = rospy.ServiceProxy('/dsr01a0509/system/get_robot_state', GetRobotState)
    get_pose_srv = rospy.ServiceProxy('/dsr01a0509/system/get_current_pose', GetCurrentPose)

    while True:
        res = get_state()
        pose = get_pose_srv(1)

        # robot_state: 1 = Idle, 2 = Moving 등등 상황에 맞게
        if res.robot_state == 1:
            break
        print(pose.pos)
        rospy.sleep(0.1)  # 100ms 주기로 확인

def main():
    rospy.init_node('simple_move_gazebo')
    rospy.wait_for_service('/dsr01a0509/motion/move_home')
    rospy.wait_for_service('/dsr01a0509/motion/move_joint')


    move_home = rospy.ServiceProxy('/dsr01a0509/motion/move_home', MoveHome)
    move_joint = rospy.ServiceProxy('/dsr01a0509/motion/move_joint', MoveJoint)

    res_home = move_home()
    print("Move Home result:", res_home)

    wait_until_motion_done()  # 홈 완료될 때까지 대기!
    rospy.sleep(2)

    joint_pose = [0, 0, 90, 0, 90, 0]
    vel = 30
    acc = 30
    time = 0
    radius = 0
    mode = 0
    blendType = 0
    syncType = 0

    res_joint = move_joint(joint_pose, vel, acc, time, radius, mode, blendType, syncType)

    wait_until_motion_done()  # 조인트 이동 완료될 때까지 대기!

if __name__ == '__main__':
    main()


