from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .point_cloud_manipulation import transform_point_cloud

from ultralytics import YOLO
import numpy as np
from typing import Dict
import threading
import copy


class AgentDetector():
    """Detects and tracks other agents."""
    def __init__(self, vehicle : VehicleState, 
                 camera_info, camera_image, lidar_point_cloud):
        self.vehicle = vehicle
        
        self.camera_info = camera_info
        self.camera_image = camera_image
        self.lidar_point_cloud = lidar_point_cloud
        
        self.detector = YOLO(settings.get('perception.agent_detection.model'))

    def box_to_agent(self, bbox_xywh, bbox_cls):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        
        Uses the image, the camera intrinsics, the lidar point cloud, 
        and the calibrated camera / lidar poses to get a good estimate 
        of the other agent's pose (in vehicle frame) and dimensions.
        """

        # Obtain point clouds in image frame and vehicle frame
        pcd_image_pixels, pcd_vehicle_frame = transform_point_cloud(
            self.lidar_point_cloud, np.array(self.camera_info.P).reshape(3,4), 
            self.camera_info.width, self.camera_info.height
        )

        x, y, w, h = bbox_xywh
        # print('Bbox: [{.2f}, {.2f}, {.2f}, {.2f}]'.format(x,y,w,h))

        # tolerance for calibration-related errors
        tolerance = settings.get('perception.agent_detection.tolerance_factor') 
        indices = [i for i in range(len(pcd_image_pixels)) if 
                    (x - tolerance * w/2) <= pcd_image_pixels[i][0] <= (x + tolerance * w/2) and 
                    (y - tolerance * h/2) <= pcd_image_pixels[i][1] <= (y + tolerance * h/2)]
        points = [pcd_vehicle_frame[idx] for idx in indices]   # in vehicle frame

        # Estimate center and dimensions
        center = np.mean(points, axis=0)
        dimensions = np.max(points, axis=0) - np.min(points, axis=0)

        # For a PhysicalObject, 
        #   origin is at the object's center in the x-y plane and at the bottom in the z axis
        pose = ObjectPose(t=0, x=center[0], y=center[1], z=center[2] - dimensions[2]/2, 
                          yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
        
        # set type based on class id
        type_dict = {
            '0': AgentEnum.PEDESTRIAN,
            '1': AgentEnum.BICYCLIST,
            '2': AgentEnum.CAR,
            '7': AgentEnum.MEDIUM_TRUCK if dimensions[2] <= 2.0 else AgentEnum.LARGE_TRUCK
        }
        return AgentState(pose=pose, dimensions=dimensions, outline=None, 
                          type=type_dict[str(bbox_cls)], activity=AgentActivityEnum.STOPPED, 
                          velocity=(0,0,0), yaw_rate=0)

    def detect_agents(self):
        yolo_class_ids = [
            0,  # person
            1,  # bicycle
            2,  # car
            7   # truck
        ]
        detection_result = self.detector(self.camera_image, classes=yolo_class_ids, verbose=False)
        bbox_locations = detection_result[0].boxes.xywh.tolist()
        bbox_classes = detection_result[0].boxes.cls.tolist()

        detected_agents = []
        for i in range(len(bbox_locations)):
            agent = self.box_to_agent(bbox_locations[i], bbox_classes[i])
            detected_agents.append(agent)
        
        return detected_agents
    
    def deduplication(self, agent, prev_states):
        """ For dedupliction:
        - Check if agent was detected before using the previous states.
        - If seen before, 
            return the dict key corresponding to the previous agent matching the current one. 
        """
        
        polygon = agent.polygon_parent()

        for key in prev_states:
            prev_agent = prev_states[key]
            prev_polygon = prev_agent.polygon_parent()

            if collisions.polygon_intersects_polygon_2d(polygon, prev_polygon):
                return key
        
        return None

    def track_agents(self, detected_agents, prev_states, counter, rate):
        """ Given a list of detected agents, updates the state of the agents.
        - Keep track of which agents were detected before.
        - For each agent, assign appropriate ids and estimate velocities.
        """
        dt = 1 / rate # time between updates

        states = {}

        for agent in detected_agents:
            prev_key = self.deduplication(agent, prev_states)

            if prev_key is None: # new agent
                # velocity of a new agent is 0 by default
                states['agent_' + str(counter)] = agent
                counter += 1
            else:
                prev_agent = self.prev_states[prev_key]
                prev_pose = prev_agent.pose

                # absolute vel = vel w.r.t vehicle + vehicle velocity 
                v_x = (agent.pose.x - prev_pose.x) / dt + self.vehicle.v
                v_y = (agent.pose.y - prev_pose.y) / dt
                v_z = (agent.pose.z - prev_pose.z) / dt

                if any([v_x, v_y, v_z]):
                    agent.activity = AgentActivityEnum.MOVING
                    agent.velocity = (v_x, v_y, v_z)
            
                states[prev_key] = agent            
        
        return states, counter


class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['agents']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('agent_detector',self.agent_callback, AgentState)
    
    def agent_callback(self, name : str, agent : AgentState):
        with self.lock:
            self.agents[name] = agent

    def update(self) -> Dict[str,AgentState]:
        with self.lock:
            return copy.deepcopy(self.agents)
