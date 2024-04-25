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
import subprocess
import sys
import time

def start_model_process(model_path, env_path):
    python_path = '/root/miniconda3/envs/AgentFormer/bin/python'
    return subprocess.Popen(
        [python_path, model_path],# ["conda", "run", "-n", env_path, "python", model_path],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        encoding="utf-8",
    )

def run_model(process):
    process.stdin.write("GET PREDICTIONS" + "\n")
    process.stdin.flush()

    while True:
        start_up_message = process.stdout.readline().strip()
        print(start_up_message)
        if start_up_message == "READY":
            break
    return


class PedestrianTrajPrediction(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.model_process  = start_model_process("agentformer_process.py", 'AgentFormer')
        self.frame_rate = 2.5
        
    def rate(self):
        return 0.5 # once every 2 seconds
    
    def state_inputs(self):
        return ['tracking_frames']
    
    def state_outputs(self):
        return ['predicted_trajectories']
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
       pass

    def self.convert_data_from_model_output(file)

    # takes in the agent states of the past 8 frames and returns the predicted trajectories of the agents in the next 12 frames
    def update(self, past_agent_states : List[str]) -> Dict[str,AgentState]:
        cur_time = time.time()
        # parse past_agent states flip the x and y coordinates
        for state in past_agent_states:
            # flip 4th and 2nd word in the state string
            state = state.split()
            state[2], state[4] = state[4], state[2]
            state = ' '.join(state)
            # output to a file
        

            
        # write data to file which model will read

        # run the traj prediction model on data
        run_model(self.model_process)

        # read data from model's output file
        self.convert_data_from_model_output(file)

        # output frame 7/2.5 -> time  + cur_time = detection_time

        # convert data to AgentState objects make sure to convert the frames to time(which will add to the AgentPose object)
        
        # return data
        pass
        
    def cleanup(self):
        # clean up subprocess which runs the model.
        self.model_process.stdin.close()
        self.model_process.wait()
        pass