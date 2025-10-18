import code
from ProjectPath import PROJECT_PATH

class ManualController():
    def __init__(self):
        self.Handle = None

    def AddHandle(self, Handle):
        self.Handle = Handle

    def Start(self):
        print("Preparing Controller...")
        banner = "\n Waiting Your Order..."
        code.interact(banner=banner, local=self.Handle)
        print("Controller Closed!")


class Action():
    def __init__(self):
        self.Action = None
        self.Key = None

