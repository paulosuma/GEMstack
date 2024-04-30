from ...state import AllState,VehicleState,AgentState
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from .agent_detection import AgentDetector
from .sign_detection import SignDetector

from ultralytics import YOLO
import cv2
import numpy as np
from typing import Dict
import multiprocessing
import copy
import time


class Detector(Component):
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

        self.camera_image = None
        self.lidar_point_cloud = None
        
        self.object_states = {}

    def rate(self):
        return settings.get('perception.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['states']
    
    def initialize(self):
        # use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        
        # use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)
    
    def image_callback(self, image : cv2.Mat):
        self.camera_image = image

    def lidar_callback(self, point_cloud: np.ndarray):
        self.lidar_point_cloud = point_cloud
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.camera_image is None or self.lidar_point_cloud is None:
            return {}   # no image data or lidar data yet
        
        # debugging
        # self.save_data()

        agent_detector = AgentDetector(vehicle, self.camera_image, self.lidar_point_cloud)
        sign_detector = SignDetector(vehicle, self.camera_image, self.lidar_point_cloud)

        mp_dict = multiprocessing.Manager().dict() # to get states from the detection modules

        def run_agent_detection():
            mp_dict['agents'] = agent_detector.detect_agents()

        def run_sign_detection():
            mp_dict['signs'] = sign_detector.detect_signs()

        p1 = multiprocessing.Process(target=run_agent_detection)
        p2 = multiprocessing.Process(target=run_sign_detection)

        t1 = time.time()

        # start the detection processes
        p1.start()
        p2.start()

        # wait for them complete
        p1.join()
        p2.join()

        t2 = time.time()

        print('Detection time:', t2 - t1)

        self.object_states = mp_dict['agents'] + mp_dict['signs']

        # incorporate agent tracking for demo?

        return self.object_states
