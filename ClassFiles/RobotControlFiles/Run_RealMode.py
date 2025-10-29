from GazeboSimulator import GazeboSimulator

## <<you should connect Robot IP first>>
## <<Connect LAN>>
# ip addr show (보통 enp(유선)/enx(어댑터)로 시작한다함)

## <<어댑터 사용했을 경우>>
# enxa0cec8ac637d/enx00e04f82fbd0
# sudo ip addr flush dev enxa0cec8ac637d
# sudo ip addr add 192.168.0.100/24 dev enxa0cec8ac637d
# sudo ip link set enxa0cec8ac637d up

## <<그냥 유선연결>>
# sudo ip addr flush dev enp68s0
# sudo ip addr add 192.168.0.100/24 dev enp68s0
# sudo ip link set enp68s0 up

# sudo ethtool -s enxa0cec8ac637d speed 100 duplex full autoneg off
# ping 192.168.0.181

Gazebo = GazeboSimulator()
Gazebo.SJ_World = 'Setup3'
Gazebo.SJ_Trj = 'PNU'
Gazebo.RealMode()

# RC = RobotController()
# RC.launcher_name = Gazebo.launcher_name
# RC.launcher_model = Gazebo.launcher_model
# RC.Ready()
#
# RC.Velocity = [20, 20]
# RC.Acceleration = [20, 20]
# RC.InitJoint = [3.179781198501587, 15.046290397644043, 95.11274719238281, -1.387559109389258e-06, 69.84093475341797, 3.1798245906829834]
# RC.InitPose = [450, 25, 400]
# RC.GetController()
#
# RC.EndController()