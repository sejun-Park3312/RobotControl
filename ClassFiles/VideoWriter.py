import threading
import time
from ClassFiles.Vision import Vision
import cv2

class VideoWriter:
    def __init__(self, Vision):
        self.VS = Vision
        self.Running = True

        self.Cam1_Buffer = []
        self.Cam2_Buffer = []
        self.FPS = 20


    def Capture(self):
        while self.VS.Running:
            with self.VS.lock:
                Cam1_Img = self.VS.Cam1_Img
                Cam2_Img = self.VS.Cam2_Img

            if Cam1_Img is not None:
                self.Cam1_Buffer.append(Cam1_Img)
            if Cam2_Img is not None:
                self.Cam2_Buffer.append(Cam2_Img)

            time.sleep(1/self.FPS)


    def SaveVideo(self, Filename):
        print("Saving Video...")
        # Cam1
        Frames_1 = self.Cam1_Buffer
        if Frames_1:
            h, w, c = Frames_1[0].shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(Filename + '_1.mp4', fourcc, self.FPS, (w, h))
            for f in Frames_1:
                out.write(f)
            out.release()

        # Cam2
        Frames_2 = self.Cam2_Buffer
        if Frames_2:
            h, w, c = Frames_2[0].shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(Filename + '_2.mp4', fourcc, self.FPS, (w, h))
            for f in Frames_2:
                out.write(f)
            out.release()

        print("Video Saved!")