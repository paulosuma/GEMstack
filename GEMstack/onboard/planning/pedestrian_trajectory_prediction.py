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




class PedestrianTrajPrediction(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.xrange = (2.3959036, 5.8143473)
        self.yrange = (-2.0247698, 4.0374074)
        
    def rate(self):
        return 0.5 # once every 2 seconds
    
    def state_inputs(self):
        return ['past_agent_states']
    
    def state_outputs(self):
        return ['predicted_trajectories']
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
       pass

    # takes in the agent states of the past 8 frames and returns the predicted trajectories of the agents in the next 12 frames
    def update(self, past_agent_states : List[AgentState]) -> Dict[str,AgentState]:
        pass
        # write data to file which model will read

        # run the traj prediction model on data

        # read data from model's output file

        # convert data to AgentState objects
        
        # return data
        
    def cleanup(self):
        # clean up subprocess which runs the model.
        pass