from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,SignState,SignEnum,SignalLightEnum,SignalLightEnum,CrossingGateEnum,SignalLightState,CrossingGateState,Sign
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .point_cloud_manipulation import transform_point_cloud
from .object_detection import ObjectDetector

from ultralytics import YOLO
import numpy as np
from typing import Dict
import threading
import copy

class SignLightDetector(ObjectDetector):
    
    def __init__(self, vehicle : VehicleState, 
                 camera_info, camera_image, lidar_point_cloud):
        detector = YOLO(settings.get('perception.sign_light_detection.model'))
        super().__init__(vehicle, camera_info, camera_image, lidar_point_cloud, detector)
    
    def detect_agents(self):
        yolo_class_ids = [
            3,  #do_not_turn_l
            4,  #do_not_turn_r
            5,  #do_not_u_turn
            7,  # green_light
            9,  #no_parking
            11, #ped_crossing
            13, #railway_crossing
            14, #red_light
            15, #stop
            20  #yellow_light
        ]
        
        detected_objects, bbox_classes = super().detect_objects(yolo_class_ids)

        detected_agents = []
        for i in range(len(detected_objects)):
            agent = self.object_to_agent(detected_objects[i], bbox_classes[i])
            detected_agents.append(agent)
        
        return detected_agents
    
    def object_to_agent(self, detected_object, bbox_cls):
        """Creates a 3D agent state from a PhysicalObject."""

        # set agent type based on class id
        type_dict = {
            '3': SignEnum.NO_LEFT_TURN,
            '4': SignEnum.NO_RIGHT_TURN,
            '5': SignEnum.NO_U_TURN,
            '7': SignalLightEnum.GREEN,
            '9': SignEnum.NO_PARKING,
            '11': SignEnum.PEDESTRIAN_CROSSING,
            '13': SignEnum.RAILROAD_CROSSING,
            '14': SignalLightEnum.RED,
            '15': SignEnum.STOP_SIGN,
            '20': SignalLightEnum.YELLOW
        }
        
        return Sign(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                          type=type_dict[str(bbox_cls)], entities=[])
    
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
        self.vehicle_interface.subscribe_sensor('sign_light_detector',self.agent_callback, Sign)
    
    def agent_callback(self, name : str, agent : Sign):
        with self.lock:
            self.agents[name] = agent

    def update(self) -> Dict[str,Sign]:
        with self.lock:
            return copy.deepcopy(self.agents)