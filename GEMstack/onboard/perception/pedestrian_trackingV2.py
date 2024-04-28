from ...state import (
    ObjectPose,
    ObjectFrameEnum,
    AgentState,
    AgentEnum,
    AgentActivityEnum,
)
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2

try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass

import numpy as np
from typing import Dict, List
from .kalman_tracker import KalmanTracker  # Behavior Prediction Team


class PedestrianTracker(Component):
    def __init__(
        self,
        kalman_config_files=["GEMstack/onboard/perception/2D_bounding_box_config.py"],
        kalman_classes = [AgentEnum.PEDESTRIAN],
        detection_file_name="GEMstack/onboard/prediction/tracking_results.txt",
        test=False,
        write_all=False,
        write_limit=8
    ):
        self.kalman_trackers = [KalmanTracker(config_file_path=kalman_config_file) for kalman_config_file in kalman_config_files]
        self.kalman_classes = kalman_classes
        self.tracking_results = {ag_class:{} for ag_class in kalman_classes}
        self.current_frame = 0
        self.write_limit= write_limit
        
        self.detection_file_name = detection_file_name
        self.write_all = write_all

        f = open(self.detection_file_name, "w")
        f.close()


        # For Testing
        self.test = test
        if self.test:
            self.detection_file_name = detection_file_name
            self.write_all = write_all
            f = open(self.detection_file_name, "w")
            f.close()

    """Base class for top-level components in the execution stack."""

    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 2.5

    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ["detected_agents"]

    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ["tracking_frames"]

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True

    def initialize(self):
        """Initialize the component. This is called once before the first
        update."""
        pass

    def cleanup(self):
        """Cleans up resources used by the component. This is called once after
        the last update."""
        pass

    def update(self, detected_agents: List[AgentState]):
        print("Updating with detected, ", detected_agents)
        """Update the component."""
        self.track_agents(detected_agents=detected_agents)
        return self.tracking_results

    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug(item, value)

    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug_event(label)
            
    def track_agents(self, detected_agents: List[AgentState]):
        """Given a list of detected agents, updates the state of the agents using a Kalman Tracker."""
        all_tracked_objects = {}
        for i, kalman_tracker in enumerate(self.kalman_trackers):
            detections = []
            for agent in detected_agents:
                if agent.type == self.kalman_classes[i]:
                    x, y, z = agent.pose.x, agent.pose.y, agent.pose.z
                    w, h, l = agent.dimensions
                    detections.append(np.array([x, y, w, l]))

            kalman_agent_states, matches = kalman_tracker.update_pedestrian_tracking(
                detections
            )
            tracking_results = {}
            for pid in kalman_agent_states:
                ag_state = kalman_agent_states[pid]
                tracking_results[pid] = AgentState(
                    pose=ObjectPose(
                        t=0,
                        x=ag_state[0],
                        y=ag_state[1],
                        z=0,
                        yaw=0,
                        pitch=0,
                        roll=0,
                        frame=ObjectFrameEnum.CURRENT,
                    ),
                    dimensions=(ag_state[2], ag_state[3], 1.5),
                    velocity=(ag_state[4], ag_state[5], 0),
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    yaw_rate=0,
                    outline=None,
                )
            all_tracked_objects[self.kalman_classes[i]] = tracking_results
        
        self.update_track_history(all_tracked_objects)

        if self.test:
            return tracking_results, matches


    def update_track_history(
        self, all_ag_dict: Dict[AgentEnum, Dict[int, AgentState]]
    ):
        for ag_class, ag_dict in all_ag_dict.items():
            for pid in sorted(ag_dict):
                if self.current_frame in self.tracking_results[ag_class]:
                    self.tracking_results[ag_class][self.current_frame][pid] = ag_dict[pid]
                else:
                    self.tracking_results[ag_class][self.current_frame] = {pid: ag_dict[pid]}
            if not self.write_all:
                # Remove old tracking information
                self.tracking_results[ag_class].pop(self.current_frame - self.write_limit, None)
        
        self.current_frame += 1