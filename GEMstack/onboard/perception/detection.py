from ...state import AllState,VehicleState,AgentState
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from .agent_detection import AgentDetector

from ultralytics import YOLO
try:
    from sensor_msgs.msg import CameraInfo
    import rospy
except ImportError:
    pass
import cv2
import numpy as np
from typing import Dict
import threading
import copy
import time


class Detector(Component):
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        
        self.camera_info_sub = None
        self.camera_info = None
        self.camera_image = None
        self.lidar_point_cloud = None
        
        self.agent_states = {}
        self.agent_counter = 0

    def rate(self):
        return settings.get('perception.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        # use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        
        # use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)
        
        # subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber('/zed2/zed_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
    
    def image_callback(self, image : cv2.Mat):
        self.camera_image = image

    # Uncomment before running on the vehicle
    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.lidar_point_cloud = point_cloud
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if [data for data in [self.camera_info, self.camera_image, self.lidar_point_cloud] if data is None]:
            return {}   # no image data or lidar data or camera info yet
        
        # debugging
        # self.save_data()

        agent_detector = AgentDetector(vehicle, self.camera_info, 
                                       self.camera_image, self.lidar_point_cloud)

        t1 = time.time()
        detected_agents = agent_detector.detect_agents()

        t2 = time.time()
        agent_states, agent_counter = agent_detector.track_agents(detected_agents, self.agent_states, 
                                                                  self.agent_counter, self.rate())
        t3 = time.time()

        print('Agent detection time:', t2 - t1)
        print('Agent shape estimation and tracking time:', t3 - t2)

        self.agent_states = agent_states
        self.agent_counter = agent_counter

        return agent_states
