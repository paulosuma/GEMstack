from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import numpy as np
from typing import Dict,Tuple, List
import time
from numpy.linalg import inv




class PedestrianTracker(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.xrange = (2.3959036, 5.8143473)
        self.yrange = (-2.0247698, 4.0374074)
        
    def rate(self):
        return 0.5 # once every 2 seconds
    
    def state_inputs(self):
        return ['agents']
    
    def state_outputs(self):
        return ['tracked_agents'] 
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
        pass
        # initialize Kalman tracker

    # takes in current detected agents, performs kalman tracking and adds to list of past tracked detections
    def update(self, agents : List[AgentState]) -> List[List[AgentState]]:
        pass
        # return 8 most recent frames of tracked agents.
        
    def cleanup(self):
        # clean up kalman tracke