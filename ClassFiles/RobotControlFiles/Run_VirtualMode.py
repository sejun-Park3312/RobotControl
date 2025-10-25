from GazeboSimulator import GazeboSimulator

Gazebo = GazeboSimulator()
Gazebo.SJ_World = 'Setup2'
Gazebo.VirtualMode()

# RC = RobotController()
# RC.launcher_name = Gazebo.launcher_name
# RC.launcher_model = Gazebo.launcher_model
# RC.Ready()
# RC.GetController()
