import cv2
import numpy as np
from pupil_apriltags import Detector
from copy import copy
import asyncio
import time

data = {"THETA"     : 0.0, # The robot's measured angle, [rad]
        "THETA_DOT" : 0.0, # The robot's calculated rotation rate, [rad/s]
        "TIME"      : 0.0} # Time of the last measurement, [ns]

data_global = {} # Dictionary keyed by AprilTag indices
data_storage = {}

start_time = time.time_ns()

class Camera(object):
    def __init__(self, 
                 frame_width, 
                 frame_height, 
                 fps, 
                 video_source,
                 save_video = None,
                 framesPerSave = 10 
                 ):
        # Camera Settings
        self.video_source = video_source
        self.save_video = save_video
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.cap = cv2.VideoCapture(self.video_source)
        self.cap.set(6, self.fourcc) # setting MJPG codec
        self.cap.set(3, frame_width)
        self.cap.set(4, frame_height)
        # self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # disable autofocus
        # self.cap.set(28, 500) # set focus level
        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))
        self.framesPerSave = framesPerSave
        self.currentFrame = 0
        self.cap.set(5,fps) # fps
        self.fps = self.cap.get(5)
        if self.save_video is not None: # Save video to file
            size = (int(self.frame_width), int(self.frame_height))
            self.out = cv2.VideoWriter(self.save_video, self.fourcc, self.fps, size) # Use save_video as the filename

    def restart(self):
        self.cap.release()
        self.cap = cv2.VideoCapture(self.video_source)
        self.cap.set(6, self.fourcc) # setting MJPG codec
        self.cap.set(3, self.frame_width)
        self.cap.set(4, self.frame_height)
        self.cap.set(5, self.fps)

    def capture_frame(self):
        self.ret, self.frame = self.cap.read()
        if self.save_video is not None:
            if (self.currentFrame == self.framesPerSave-1):
                self.out.write(self.frame)
                self.currentFrame = 0
            else:
                self.currentFrame = self.currentFrame + 1
        return [self.ret, self.frame]

    def show_frame(self):
        cv2.imshow('frame', self.frame)

    def close(self):
        self.cap.release()
        if self.save_video is not None:
            self.out.release()
        cv2.destroyAllWindows()

detector = Detector(families="tag36h11", nthreads=4, quad_sigma=0.5) # Used to have a quad_blur here, might need later

cameraNumber = 1
fps = 30
frameWidth = int(1920/2)
frameHeight = int(1080/2)
showVideo = True
lineLength = 30

def detectTags(myCamera):
    gray = cv2.cvtColor(myCamera.frame, cv2.COLOR_BGR2GRAY) 
    detections = detector.detect(gray) 
    return detections

def getMeasurement(detection, myCamera):
    center = detection.center 
    center_top = (detection.corners[2]+detection.corners[3])/2 
    theta = np.mod(3*np.pi-np.arctan2(center_top[1]-center[1],center_top[0]-center[0]),2*np.pi)
    drawingTheta = np.mod(np.arctan2(center_top[1]-center[1],center_top[0]-center[0])+np.pi,2*np.pi)
    cv2.line(myCamera.frame, (int(center[0]),int(center[1])), (int(center[0]+lineLength*np.cos(drawingTheta)),int(center[1]+lineLength*np.sin(drawingTheta))), (0,255,0), 2)
    cv2.circle(myCamera.frame, (int(center[0]),int(center[1])), 5, (0,0,255), -1)
    number = str(detection.tag_id)
    current_time = time.time_ns()
    if number in data_global.keys():
        dt = current_time - data_global[number]["TIME"]
        dtheta = theta - data_global[number]["THETA"]
        if dtheta < -np.pi: # This would probably be where the circle wraps
            dtheta = dtheta + 2*np.pi
        elif dtheta < 0: # This might be due to weird, near-zero rate stuff
            dtheta = 0
        data_global[number]["THETA"] = theta
        data_global[number]["THETA_DOT"] = dtheta/dt
        data_global[number]["TIME"] = time.time_ns() - start_time
        data_storage[number] = np.append(data_storage[number],np.fromiter(data_global[number].values(),dtype=float).reshape(1,-1),axis=0)
    else:
        data_global[number] = data.copy()
        data_global[number]["THETA"] = theta
        data_global[number]["TIME"] = time.time_ns() - start_time
        data_storage[number] = np.fromiter(data_global[number].values(),dtype=float).reshape(1,-1)
    return

async def main():
    myCamera = Camera(video_source=cameraNumber,frame_height=frameHeight,frame_width=frameWidth,fps=fps) 
    while True:
        try:
            cam_time = time.time()

            myCamera.ret = False
            while not myCamera.ret:
                myCamera.capture_frame()
                if not myCamera.ret:
                    await asyncio.sleep(1.0)
                    myCamera.restart()
            
            detections = detectTags(myCamera)
            for result in detections:
                getMeasurement(result,myCamera)

            myCamera.show_frame()
            print(data_global)

            if cv2.waitKey(1) & 0xFF == ord('q'): 
                print("Exiting by pressing q!")
                for key in data_global.keys():
                    print("Saving {}".format(key))
                    filename = "Data/Save_" + key
                    np.save(filename,data_storage[key])
                break

            # print("Camera FPS: {}".format(1/(time.time() - cam_time)))

        except asyncio.exceptions.CancelledError:
            print("Exiting by asyncio error!")
            break
    myCamera.close()

asyncio.run(main())