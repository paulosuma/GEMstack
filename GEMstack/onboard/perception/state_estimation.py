from dataclasses import replace
import math
from typing import List
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.physical_object import ObjectFrameEnum,ObjectPose,convert_xyhead
from ...knowledge.vehicle.geometry import front2steer,steer2front
from ..interface.gem import GEMInterface
from ..component import Component

class GNSSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        vehicle_interface.subscribe_gnss(self.inspva_callback)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')

    # Get GNSS information
    def inspva_callback(self, inspva_msg):
        self.gnss_pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    x=inspva_msg.longitude,
                                    y=inspva_msg.latitude,
                                    z=inspva_msg.height,
                                    yaw=inspva_msg.azimuth,  #heading from north in degrees
                                    roll=inspva_msg.roll,
                                    pitch=inspva_msg.pitch,
                                    )
        print("INS status",inspva_msg.status)
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        localxy = transforms.rotate2d(self.location,-self.yaw_offset)
        gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        vehicle_pose_global = replace(self.gnss_pose,
                                      t=self.vehicle_interface.time(),
                                      x=center_xyhead[0],
                                      y=center_xyhead[1],
                                      yaw=center_xyhead[2])

        readings = self.vehicle_interface.get_reading()
        return readings.to_state(vehicle_pose_global)



class FakeStateEstimator(Component):
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        vehicle_interface.subscribe_gnss(self.fake_gnss_callback)
        self.vehicle_state = None

    # Get GNSS information
    def fake_gnss_callback(self, vehicle_state):
        self.vehicle_state = vehicle_state
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.vehicle_state is not None

    def update(self) -> VehicleState:
        return self.vehicle_state