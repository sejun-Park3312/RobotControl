import threading
from ProjectPath import PROJECT_PATH
from ClassFiles.TotalSystem import TotalSystem

## <<Before Starting, Open Gazebo>>
## <<Copy and Paste the Following Commands into Terminal!!>>
## ---------------------------------------------
# cd ~/catkin_ws
# source devel/setup.bash
# roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509 sim:=true
# roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom sim:=true
# roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom mode:=real host:=192.168.0.181 port:=12345
## ---------------------------------------------

## <<Connection LAN>>

# sudo ip addr add 192.168.0.100/24 dev enp68s0
# sudo ip link set enp68s0 up; ping 192.168.0.181

# sudo ip addr add 192.168.0.100/24 dev enx00e04f82fbd0
# sudo ip link set enx00e04f82fbd0 up
# ping 192.168.0.181

TS = TotalSystem()
TS.RC.modelName = "a0509_custom"
TS.CT.Kp = 1e-1/3
TS.CT.Kd = 1e-2
TS.CT.Ki = 0

TotalSystem_Thread = threading.Thread(target=TS.Start, daemon=True)
TotalSystem_Thread.start()

TS.TotalSystemController()
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


