import cv2
from pupil_apriltags import Detector
import numpy as np
import asyncio
import data
import time

class camera(object):
    def __init__(self, 
                 positions_active_queue: asyncio.Queue,
                 positions_history_queue: asyncio.Queue,
                 stop_event: asyncio.Event,
                 frame_width = None, 
                 frame_height = None, 
                 fps = None, 
                 video_source = None,
                 save_video = None,
                 framesPerSave = None,
                 mmperpix = 1.0,
                 ):
        self.positions_active_queue = positions_active_queue
        self.positions_history_queue = positions_history_queue
        self.stop_event = stop_event
        
        self.positions = data.data_positions()

        # Camera Settings
        if video_source is not None:
            self.video_source = video_source
        else:
            self.video_source = 0
        self.save_video = save_video
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.cap = cv2.VideoCapture(self.video_source)
        if frame_width is not None:
            self.cap.set(3, frame_width)
        self.frame_width = int(self.cap.get(3))
        if frame_height is not None:
            self.cap.set(4, frame_height)
        self.frame_height = int(self.cap.get(4))
        if fps is not None:
            self.cap.set(5,fps) # fps
        self.fps = self.cap.get(5)
        self.cap.set(6, self.fourcc) # setting MJPG codec
        # self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # disable autofocus
        # self.cap.set(cv2.CAP_PROP_FOCUS, 50) # set focus level
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0) # disable autoexposure
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, -11) # set exposure
        if framesPerSave is not None:
            self.framesPerSave = framesPerSave
        else:
            self.framesPerSave = 10
        self.currentFrame = 0
        
        # Saving
        if self.save_video is not None: # Save video to file
            self.save_video = save_video
            size = (int(self.frame_width), int(self.frame_height))
            self.out = cv2.VideoWriter(self.save_video, self.fourcc, self.fps, size) # Use save_video as the filename

        # AprilTags
        self.detector = Detector(families="tag36h11", nthreads=4, quad_sigma=0.5) # Used to have a quad_blur here, might need later
        self.detections = []
        self.linelength = 30
        self.mmperpix = mmperpix # Conversion to real units 

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

    def detect_tags(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) 
        self.detections = self.detector.detect(gray) 

    def _positions_from_detection(self,
                                  detection):
        center = detection.center 
        center_top = (detection.corners[2]+detection.corners[3])/2 
        theta = np.mod(3*np.pi-np.arctan2(center_top[1]-center[1],center_top[0]-center[0]),2*np.pi)
        drawtheta = np.mod(np.arctan2(center_top[1]-center[1],center_top[0]-center[0])+np.pi,2*np.pi)
        # center[0] = center[0] - self.frame_width/2
        # center[1] = -center[1] + self.frame_height/2
        return [center[0], center[1], theta, drawtheta]

    def get_measurements(self):
        measurements = {}
        for detection in self.detections:
            number_str = str(detection.tag_id)
            measurements[number_str] = {}
            x, y, theta, drawtheta = self._positions_from_detection(detection)
            cv2.line(self.frame, (int(x),int(y)), (int(x+self.linelength*np.cos(drawtheta)),int(y+self.linelength*np.sin(drawtheta))), (0,255,0), 2)
            cv2.circle(self.frame, (int(x),int(y)), 5, (0,0,255), -1)
            measurements[number_str]["X"] = (x - self.frame_width/2) * self.mmperpix
            measurements[number_str]["Y"] = (-y + self.frame_height/2) * self.mmperpix
            measurements[number_str]["THETA"] = theta
        return measurements

    def close(self):
        self.cap.release()
        if self.save_video is not None:
            self.out.release()
        cv2.destroyAllWindows()

    async def update_camera(self):
        self.capture_frame()
        self.detect_tags()
        measurements = self.get_measurements()
        self.show_frame()
        if len(measurements.keys()) > 0:
            positions_dict = self.positions.fill(measurements)
            if self.positions_active_queue.empty():
                await self.positions_active_queue.put(positions_dict)
            if self.positions_history_queue.empty():
                await self.positions_history_queue.put(positions_dict)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_event.set()
            self.close()

    # This function should actually be run as a task and includes a restriction on how often the updates will actually run.
    async def update(self,
                     min_time: float = 0.02
                     ):
        while not self.stop_event.is_set():
            await asyncio.gather(asyncio.sleep(min_time),self.update_camera())