from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import collisions, transforms
from ..interface.gem import GEMInterface
from ..component import Component
from .pcd_manipulation import project_point_cloud

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


class AgentDetector(Component):
    """Detects and tracks other agents."""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        
        self.camera_info_sub = None
        self.camera_info = None
        self.camera_image = None
        self.lidar_point_cloud = None
        
        self.T_lidar = np.eye(4)
        self.T_lidar[:3,:3] = np.array(settings.get('vehicle.calibration.top_lidar.rotation'))
        self.T_lidar[:3,3] = np.array(settings.get('vehicle.calibration.top_lidar.position'))
        self.T_camera = np.eye(4)
        self.T_camera[:3,:3] = np.array(settings.get('vehicle.calibration.front_camera.rotation'))
        self.T_camera[:3,3] = np.array(settings.get('vehicle.calibration.front_camera.rgb_position'))
        
        self.detector = YOLO(settings.get('perception.agent_detection.model'))
        
        self.counter = 0
        self.prev_agent_states = {}

    def rate(self):
        return settings.get('perception.agent_detection.rate')
    
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

        t1 = time.time()
        detected_agents = self.detect_agents()

        t2 = time.time()
        agent_states = self.track_agents(vehicle, detected_agents)
        t3 = time.time()

        print('Detection time:', t2 - t1)
        print('Shape estimation and tracking time:', t3 - t2)

        self.prev_agent_states = agent_states
        return agent_states

    def box_to_agent(self, bbox_xywh, bbox_cls):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        
        Uses the image, the camera intrinsics, the lidar point cloud, 
        and the calibrated camera / lidar poses to get a good estimate 
        of the other agent's pose and dimensions.
        """

        # Obtain point clouds in image frame and vehicle frame
        pcd_image_pixels, pcd_vehicle_frame = project_point_cloud(
            self.lidar_point_cloud, self.T_lidar, self.T_camera, 
            np.array(self.camera_info.P).reshape(3,4), 
            self.camera_image.width, self.camera_info.height
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

        # Create the agent state with the estimated position (in vehicle frame) and dimensions
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
    
    def deduplication(self, agent):
        """ For dedupliction:
        - Check if agent was detected before using prev_agent_states.
        - If seen before, 
            return the dict key corresponding to the previous agent matching the current one. 
        """
        
        polygon = agent.polygon_parent()

        for key in self.prev_agent_states:
            prev_agent = self.prev_agent_states[key]
            prev_polygon = prev_agent.polygon_parent()

            if collisions.polygon_intersects_polygon_2d(polygon, prev_polygon):
                return key
        
        return None

    def track_agents(self, vehicle : VehicleState, detected_agents):
        """ Given a list of detected agents, updates the state of the agents.
        - Keep track of which agents were detected before.
        - For each agent, assign appropriate ids and estimate velocities.
        """

        dt = 1 / self.rate() # time between updates

        agent_states = {}

        for agent in detected_agents:
            prev_agent_key = self.deduplication(agent)

            if prev_agent_key is None: # new agent
                # velocity of a new agent is 0 by default
                agent_states['agent_' + str(self.counter)] = agent
                self.counter += 1
            else:
                prev_agent = self.prev_agent_states[prev_agent_key]
                prev_pose = prev_agent.pose

                # absolute vel = vel w.r.t vehicle + vehicle velocity 
                v_x = (agent.pose.x - prev_pose.x) / dt + vehicle.v
                v_y = (agent.pose.y - prev_pose.y) / dt
                v_z = (agent.pose.z - prev_pose.z) / dt

                if any([v_x, v_y, v_z]):
                    agent.activity = AgentActivityEnum.MOVING
                    agent.velocity = (v_x, v_y, v_z)
            
                agent_states[prev_agent_key] = agent            
        
        return agent_states


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
