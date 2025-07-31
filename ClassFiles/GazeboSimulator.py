import rospy
import subprocess
import time
from ProjectPath import PROJECT_PATH

class GazeboSimulator:
    def __init__(self):
        self.launcher_model = "a0509_custom" # or a0509/a0509_custom/a0509_Calibration
        self.launcher_name = "SJ_Custom" # or single_robot_gazebo/SJ_Custom

        # Virtual Mode
        self.EmulatorModel = "a0509"

        # Real Mode
        self.host = "192.168.0.181" # 실제 로봇의 IP주소
        self.port = "12345"
        self.LAN_Name = "enp68s0" # or enx00e04f82fbd0/enp68s0



    def VirtualMode(self):
        print("Preparing Virtual Mode")
        print("---------------------------")
        print("")

        print("Opening Docker Emulator...")
        # Close and Remove Existing Docker
        subprocess.run("docker ps -q | xargs -r docker stop", shell=True, executable="/bin/bash")
        subprocess.run("docker ps -aq | xargs -r docker rm", shell=True, executable="/bin/bash")

        # Open Emulator(Choose Right Robot Model DRCF!!)
        Docker_Msg = (
            "docker run -it --rm "
            "--name dsr01_emulator "
            "-e ROBOT_ID=dsr01 "
            "-e ROBOT_MODEL=a0509 "
            "-p 12345:12345 "
            "doosanrobot/dsr_emulator:3.0.1"
        )
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', Docker_Msg + '; exec bash'])
        time.sleep(5)
        print("Emulator Opened!")
        print("")

        # Open Gazebo
        print("Opening Gazebo...")
        Gazebo_Msg = ("cd ~/catkin_ws; "
                      " source devel/setup.bash; "
                      " roslaunch dsr_launcher " + self.launcher_name + ".launch model:=" + self.launcher_model)
        subprocess.Popen(['gnome-terminal','--','bash', '-c', Gazebo_Msg + '; exec bash'])
        print("Gazebo Opened!")
        print("")

        print("---------------------------")
        print("Virtual Mode Ready!")
        print("")
        print("")



    def RealMode(self):
        print("Preparing Virtual Mode")
        print("---------------------------")
        print("")

        # Open Gazebo
        print("Opening Gazebo...")
        Gazebo_Msg = ("cd ~/catkin_ws; "
                      " source devel/setup.bash; "
                      " roslaunch dsr_launcher " + self.launcher_name + ".launch model:=" + self.launcher_model + " mode:=real host:=" + self.host + " port:=" + self.port)
        subprocess.Popen(['gnome-terminal','--','bash', '-c', Gazebo_Msg + '; exec bash'])
        print("Gazebo Opened!")
        print("")

        print("---------------------------")
        print("Real Mode Ready!")
        print("")
        print("")





## <<Before Starting, Open Gazebo>>
## <<Copy and Paste the Following Commands into Terminal!!>>


    ### <<< Virtual Mode>>>
    ## ---------------------------------------------

        ## <<Open Emulator>>
            #  docker ps -q | xargs -r docker stop
            #  docker ps -aq | xargs -r docker rm
            #  docker run -it --rm \
            #   --name dsr01_emulator \
            #   -e ROBOT_ID=dsr01 \
            #   -e ROBOT_MODEL=a0509 \
            #   -p 12345:12345 \
            #   doosanrobot/dsr_emulator:3.0.1

        ## <<Open Gazebo>>
            # cd ~/catkin_ws
            #  source devel/setup.bash
            # roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509
            # roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom

    ## ---------------------------------------------


    ### <<<Real Mode>>>
    ## ---------------------------------------------

        ## <<Connect LAN>>
            # ip addr show (보통 enp(유선)/enx(어댑터)로 시작한다함)
            # sudo ip addr add 192.168.0.100/24 dev enx00e04f82fbd0
            # sudo ip link set enx00e04f82fbd0 up
            # ping 192.168.0.181

        ## <<Open Gazebo>>
            # cd ~/catkin_ws
            #  source devel/setup.bash
            # roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509 mode:=real host:=192.168.0.181 port:=12345
            # roslaunch dsr_launcher SJ_Custom.launch model:=a0509_Calibration mode:=real host:=192.168.0.181 port:=12345

    ## ---------------------------------------------
