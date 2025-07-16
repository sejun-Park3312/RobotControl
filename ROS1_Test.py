#!/usr/bin/env python3

import rospy
from dsr_msgs.srv import MoveJoint

def main():
    rospy.init_node('simple_move_client')
    rospy.wait_for_service('/dsr01/motion/move_joint')
    try:
        move_joint = rospy.ServiceProxy('/dsr01/motion/move_joint', MoveJoint)
        # 6개 관절 각도(deg), 가속도, 속도
        resp = move_joint([0, 0, 90, 0, 90, 0], 20, 30)
        print("MoveJoint Response:", resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    main()
