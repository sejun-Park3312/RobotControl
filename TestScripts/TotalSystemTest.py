import threading
from ProjectPath import PROJECT_PATH
from ClassFiles.TotalSystem import TotalSystem

TS = TotalSystem()
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
