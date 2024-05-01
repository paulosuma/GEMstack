from dataclasses import replace
import math
import numpy as np
from typing import List
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.physical_object import ObjectFrameEnum,ObjectPose,convert_xyhead
from ...knowledge.vehicle.geometry import front2steer,steer2front
from ...mathutils.signal import OnlineLowPassFilter
from ..interface.gem import GEMInterface
from ..component import Component
from ..interface.gem_hardware import GNSSReading, VioslamReading

class GNSSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.gnss_speed = reading.speed
        self.status = reading.status
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        #print("INS status",self.status)

        # vehicle gnss heading (yaw) in radians
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        localxy = transforms.rotate2d(self.location,-self.yaw_offset) #rotate gps about the origin
        gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        vehicle_pose_global = replace(self.gnss_pose,
                                      t=self.vehicle_interface.time(),
                                      x=center_xyhead[0],   
                                      y=center_xyhead[1],
                                      yaw=center_xyhead[2])

        readings = self.vehicle_interface.get_reading()
        raw = readings.to_state(vehicle_pose_global)

        #filtering speed
        # filt_vel     = self.speed_filter(raw.v)
        raw.v = self.gnss_speed
        return raw
    

class VIOSlamEstimator(Component):
    """Looks at the Vioslam reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'Vioslam' not in vehicle_interface.sensors():
            raise RuntimeError("Vioslam sensor not available")
        vehicle_interface.subscribe_sensor('Vioslam',self.vio_slam_callback,ObjectPose)
        self.Vioslam_pose = None
        self.cam_location = settings.get('vehicle.calibration.front_camera.center_position')[:2]
        self.P_c_cam = -1 * np.array(self.cam_location)
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get vioslam information
    def vio_slam_callback(self, reading : VioslamReading):
        self.Vioslam_pose = reading.pose
        self.status = reading.status
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.Vioslam_pose is not None
    
    def rotation_mat(self, yaw):
        return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])

    def update(self) -> VehicleState:
        if self.Vioslam_pose is None:
            return
        # reference point is located at the center of rear axle
        center_x, center_y = np.array([self.Vioslam_pose.x, self.Vioslam_pose.y]) + \
                        self.rotation_mat(self.Vioslam_pose.yaw) @ self.P_c_cam
        print("VIO pose_start x, y, yaw = ", center_x, center_y, self.Vioslam_pose.yaw)
        
        self.Vioslam_pose.x=center_x
        self.Vioslam_pose.y=center_y

        readings = self.vehicle_interface.get_reading()
        raw = readings.to_state(self.Vioslam_pose)

        #filtering speed
        raw.v = self.gnss_speed
        #filt_vel     = self.speed_filter(raw.v)
        #raw.v = filt_vel
        return raw
        


class OmniscientStateEstimator(Component):
    """A state estimator used for the simulator which provides perfect state information"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.fake_gnss_callback)
        self.vehicle_state = None

    # Get GNSS information
    def fake_gnss_callback(self, vehicle_state):
        self.vehicle_state = vehicle_state
    
    def rate(self):
        return 50.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.vehicle_state is not None

    def update(self) -> VehicleState:
        return self.vehicle_state
    

#alias, will be deprecated by end of February
FakeStateEstimator = OmniscientStateEstimator