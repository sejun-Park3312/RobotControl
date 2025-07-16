import rospy
from dsr_msgs.srv import GetCurrentPose
from dsr_msgs.srv import GetRobotState


def get_ee_position(get_pose_srv):
    try:
        resp = get_pose_srv()
        return resp.pos
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to get EE position: {e}")
        return None, None
resp.space_type
def main():
    rospy.init_node('robot_controller')

    # 서비스 준비
    rospy.wait_for_service('/dsr01a0509/system/get_current_pose')
    get_pose_srv = rospy.ServiceProxy('/dsr01a0509/system/get_current_pose', GetCurrentPose)

    # 예: 로봇 움직임 후 EE 위치 확인
    # move_robot()  # 기존 움직임 함수 호출

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        pos, rotm = get_ee_position(get_pose_srv)
        if pos:
            rospy.loginfo(f"Current EE Position: {pos}")
        rate.sleep()

if __name__ == '__main__':
    main()