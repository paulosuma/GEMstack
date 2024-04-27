from dataclasses import replace
import math
from typing import List
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.physical_object import ObjectFrameEnum,ObjectPose,convert_xyhead
from ...knowledge.vehicle.geometry import front2steer,steer2front
from ...mathutils.signal import OnlineLowPassFilter
from ..interface.gem import GEMInterface
from ..component import Component
from ..interface.gem_hardware import GNSSReading
import math

# Radius of the Earth in meters
earth_radius = 6371000  

#Adapted from Jsuan's code, written by Enguang
class IMUStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'imu' not in vehicle_interface.sensors():
            #Add holonomic constraint and IMU estimation
            #TODO, test if this is for physically uninstalled gnss condition
            #Or indoor unfixed satellite?
            #Note the Septentrio IMU would work indoor, even when GNSS signal isn't available
            raise RuntimeError("IMU sensor not available")
        
        #add gnss_callback
        vehicle_interface.subscribe_sensor('imu',self.imu_callback)
        self.imu_pose = None
        self.accleration = None
        self.angular_vel = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get IMU information
    def imu_callback(self, reading):
        #read the accleration and angular velocity here
        self.linear_acceleration.x = reading.linear_acceleration.x
        self.linear_acceleration.y = reading.linear_acceleration.y
        self.linear_acceleration.z = reading.linear_acceleration.z
        self.angular_velocity.x = reading.angular_velocity.x
        self.angular_velocity.y = reading.angular_velocity.y
        self.angular_velocity.z = reading.angular_velocity.z

        self.angular_vel = reading.pose
        self.status = reading.status
    
    def rate(self):
        return 40.0 #Up to 50Hz
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        print("INS status",self.status) #Should be 'ok'

        #Do double integration here
        #Do integration from last known good GNSS location


        # vehicle gnss heading (yaw) in radians
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

        #readings belong to GEMVehicleReading class
        #combine speed, steering, left/right signal etc with vehicle_pose_global
        readings = self.vehicle_interface.get_reading()

        #raw is VehicleState type
        raw = readings.to_state(vehicle_pose_global)

        #filtering speed
        filt_vel = self.speed_filter(raw.v)
        raw.v = filt_vel
        return raw


#Bug fixed by Enguang
class GNSSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            #Add holonomic constraint and IMU estimation
            #TODO, test if this is for physically uninstalled gnss condition
            #Or indoor unfixed satellite?
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss', self.gnss_callback, GNSSReading)
        
        #add gnss_callback
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        self.gnss_pose = None
        #set initial gnss location and yaw
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.status = reading.status
    
    def rate(self):
        return 10.0 #acturally up to 50 Hz
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        #print("INS status",self.status) #Should be 'ok'


        # vehicle gnss heading (yaw) in radians
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

        print('pose X Y yaw', vehicle_pose_global.x, vehicle_pose_global.y, vehicle_pose_global.yaw)
        print('GNSS pose X Y yaw', vehicle_pose_global.x, vehicle_pose_global.y, vehicle_pose_global.yaw)
        #readings belong to GEMVehicleReading class
        #combine speed, steering, left/right signal etc with vehicle_pose_global
        readings = self.vehicle_interface.get_reading()

        #raw is VehicleState type
        raw = readings.to_state(vehicle_pose_global)

        #filtering speed
        filt_vel = self.speed_filter(raw.v)
        raw.v = filt_vel
        return raw

class EKSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            #Add holonomic constraint and IMU estimation
            #TODO, test if this is for physically uninstalled gnss condition
            #Or indoor unfixed satellite?
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss', self.gnss_callback, GNSSReading)
        
        #add gnss_callback
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        self.gnss_pose = None
        #set initial gnss location and yaw
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.status = reading.status
    
    def rate(self):
        return 10.0 #acturally up to 50 Hz
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        #print("INS status",self.status) #Should be 'ok'


        # vehicle gnss heading (yaw) in radians
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

        print('pose X Y yaw', vehicle_pose_global.x, vehicle_pose_global.y, vehicle_pose_global.yaw)
        print('GNSS pose X Y yaw', vehicle_pose_global.x, vehicle_pose_global.y, vehicle_pose_global.yaw)
        #readings belong to GEMVehicleReading class
        #combine speed, steering, left/right signal etc with vehicle_pose_global
        readings = self.vehicle_interface.get_reading()

        #raw is VehicleState type
        raw = readings.to_state(vehicle_pose_global)

        #filtering speed
        filt_vel = self.speed_filter(raw.v)
        raw.v = filt_vel
        return raw
    
    def initialize_converter(init_lon, init_lat):
        """ Store initial longitude and latitude converted to degrees """
        init_lon = math.degrees(init_lon)
        init_lat = math.degrees(init_lat)
        return init_lon, init_lat
    
    def to_cartesian(init_lon, init_lat, lon, lat, yaw):
        """ Convert geographical coordinates to Cartesian coordinates """
        # Convert input coordinates from radians to degrees
        lon = math.degrees(lon)
        lat = math.degrees(lat)

        # Calculate differences in coordinates
        dlat = math.radians(lat - init_lat)
        dlon = math.radians(lon - init_lon)
        a = init_lat * math.pi / 180

        # Convert differences in latitude and longitude to distances in meters
        dx = dlon * earth_radius * math.cos(a)
        dy = dlat * earth_radius

        # Return the differences as new (x, y) coordinates and the unchanged yaw
        return dx, dy, yaw




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