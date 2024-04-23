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
from collections import defaultdict

# 8 frames used to base future trajectories off of (current frame plus previous 7)
NUM_PREV_FRAMES = 7
NUM_FUTURE_FRAMES = 12

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

    def estimate_velocity(self, past_values):
        # estimate velocity from past few frames
        pass
    
    # def convert_data_from_model_output(self, file_name) -> Dict[List[AgentState]]:
    # Changing signature of model since we changed the output format of AgentFormer to return the actual tensor
    def convert_data_from_model_output(self, sample_model_3D, frame) -> List[Dict[List[AgentState]]]:
        # # read the file and convert the data to AgentState objects
        # agent_dict = defaultdict(list) # Key: Pedestrian ID | Value: List of AgentStates for each future frame
        # Commenting out the code that reads lines from a file because we changed the output format of AgentFormer to return the actual tensor
        # file = open(file_name, 'r')
        # read through file line by lie
        # for line in file.readlines():
        #     # split the 
        #     frame_id, ped_id, x, y = line.split(' ')
        #     # convert to floats
        #     frame_id, ped_id, x, y = float(frame_id), float(ped_id), float(x), float(y)
        #     x,y = y,x # flip the x and y coordinates back to normal
        #     # convert the frame_id to time
        #     time = frame_id/self.frame_rate + self.cur_time
        #     # create an AgentState object
        #     pose = ObjectPose(t=time, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.START)

        #     print ('pose xy:', x, y)

        #     # dimensions of a pedestrian(not accurate)
        #     l = 1
        #     w = 1
        #     h = 1.7
        #     dims = (w, h, l) 
        #     #velocity = estimate velocity from past few frames
        #     agent_state = AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)
        #     agent_dict[ped_id].append(agent_state)
        #     # add the agent state to the list of agent states

        agent_list = []
        # sample_model_3D: 5 x ped_id x 12 x 2
        for traj in range(sample_model_3D.shape[0]):
            # Create the dictionary of pedestrian-future AgentState lists for the current trajectory
            agent_dict = defaultdict(list) # Key: Pedestrian ID | Value: List of AgentStates for each future frame
            for ped_id in range(sample_model_3D.shape[1]):
                for future_frame_id in range(sample_model_3D.shape[2]):
                    # flip the x- and y-coordinates back to normal
                    x = sample_model_3D[traj][ped_id][future_frame_id][1]
                    y = sample_model_3D[traj][ped_id][future_frame_id][0]

                    # convert the frame_id to time
                    frame_id = frame + future_frame_id + 1
                    time = frame_id / self.frame_rate + self.cur_time

                    # create an AgentState object
                    pose = ObjectPose(t=time, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.START)

                    print("pose xy: ", x, y)

                    # dimensions of a pedestrian (not accurate)
                    l = 1
                    w = 1
                    h = 1.7
                    dims = (w, h, l)
                    # velocity = esimate velocity from past few frames
                    agent_state = AgentState(pose=pose, dimension=dims, outline=None, type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)
                    agent_dict[ped_id].append(agent_state)
            agent_list.append(agent_dict)
        return agent_dict
            

    # takes in the agent states of the past 8 frames and returns the predicted trajectories of the agents in the next 12 frames
    # outputs dictionary where key is the sampler id and value is the list of agent states for the next 12 frames
    def update(self, past_agent_states : List[str]) -> List[Dict[List[AgentState]]]:
        self.cur_time = time.time()
        # parse past_agent states flip the x and y coordinates
        for state in past_agent_states:
            # flip 4th and 2nd word in the state string
            state = state.split()
            state[2], state[4] = state[4], state[2]
            state = ' '.join(state)
            # output to a file
        

            
        # write data to file which model will read

        # run the traj prediction model on data
        sample_model_3D, frame = run_model(self.model_process)

        # read data from model's output file
        # self.convert_data_from_model_output(file)

        # output frame 7/2.5 -> time  + cur_time = detection_time

        # convert data to AgentState objects make sure to convert the frames to time(which will add to the AgentPose object)
        agent_list = self.convert_data_from_model_output(sample_model_3D, frame)
        
        # return data
        return agent_list
        
    def cleanup(self):
        # clean up subprocess which runs the model.
        self.model_process.stdin.close()
        self.model_process.wait()
        pass