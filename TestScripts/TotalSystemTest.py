import threading
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.TotalSystem import TotalSystem
from ClassFiles.GazeboSimulator import GazeboSimulator

# Gazebo = GazeboSimulator()
# Gazebo.launcher_name = "SJ_Custom" # or single_robot_gazebo/SJ_Custom
# Gazebo.launcher_model = "a0509_custom" # or a0509/a0509_Calibration/a0509_custom
# Gazebo.RealMode()

TS = TotalSystem()
TS.RC.launcher_model = "a0509_custom"
TS.RC.Ready()


## << Parameter Setting >>
## --------------------------------------------------------------------
# PID Gain
TS.CT.Kp = 1e-1/3
TS.CT.Kd = 1e-2
TS.CT.Ki = 0



## << Start! >>
## --------------------------------------------------------------------
TotalSystem_Thread = threading.Thread(target=TS.Start, daemon=True)
TotalSystem_Thread.start()

banner = "\n Waiting Your Order..."
locals_dict = {'MoveJoint': TS.RC.Move_Joint,
               'MoveRel': TS.RC.Move_Rel,
               'MoveAbs': TS.RC.Move_Abs,
               'GetPose': TS.RC.Get_Pose,
               'GetJoint': TS.RC.Get_Joint,
               'HomePose': TS.RC.Move_Home,
               'InitPose': TS.RC.Init_Pose,
               'Wait': TS.RC.Wait,

               'Z_Target': TS.Print_ZTarget,
               'Z_System': TS.Print_ZSystem,
               'Z_Error': TS.Print_ZError,
               'PWM': TS.Print_PWM,
               'TS': TS}
code.interact(banner=banner, local=locals_dict)


## << Closing... >>
## --------------------------------------------------------------------
TS.VS.Running = False
TS.RC.Running = False
TS.Running = False

TotalSystem_Thread.join()
TS.SaveResults()
TS.AD.Disconnect()

print(".")
print(".")
print(".")
print("")
print("THE END!")






## <<you should connect Robot IP first>>
## <<Connect LAN>>
# ip addr show (보통 enp(유선)/enx(어댑터)로 시작한다함)

## <<어댑터 사용했을 경우>>
# sudo ip addr flush dev enx00e04f82fbd0
# sudo ip addr add 192.168.0.100/24 dev enx00e04f82fbd0
# sudo ip link set enx00e04f82fbd0 up

## <<그냥 유선연결>>
# sudo ip addr flush dev enp68s0
# sudo ip addr add 192.168.0.100/24 dev enp68s0
# sudo ip link set enp68s0 up