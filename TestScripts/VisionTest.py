import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision

# 먼저 연결된 웹캠의 번호 확인 후 Vision에 주기
# v4l2-ctl --list-devices
# sudo apt install v4l-utils (위에 설치 필요 시)
VS = Vision()
VS.Tracking()