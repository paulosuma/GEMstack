from ...state import AllState,VehicleState,PhysicalObject,ObjectPose,ObjectFrameEnum
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .point_cloud_manipulation import transform_point_cloud
from .pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

from ultralytics import YOLO
import numpy as np


class ObjectDetector():
    def __init__(self, vehicle : VehicleState, camera_info, 
                 camera_image, lidar_point_cloud, detector):
        self.vehicle = vehicle
        
        self.camera_info = camera_info
        self.camera_image = camera_image
        self.lidar_point_cloud = lidar_point_cloud
        
        self.detector = detector

    def box_to_object(self, bbox_xywh):
        """Creates a PhysicalObject from a (x,y,w,h) bounding box."""

        x, y, w, h = bbox_xywh
        # print('Bbox: [{.2f}, {.2f}, {.2f}, {.2f}]'.format(x,y,w,h))

        """
        Uses the image, the camera intrinsics, the lidar point cloud, 
        and the calibrated camera / lidar poses to get a good estimate 
        of the object's pose (in vehicle frame) and dimensions.
        """
        '''
        # Obtain point clouds in image frame and vehicle frame
        pcd_image_pixels, pcd_vehicle_frame = transform_point_cloud(
            self.lidar_point_cloud, np.array(self.camera_info.P).reshape(3,4), 
            self.camera_info.width, self.camera_info.height
        )

        # tolerance for calibration-related errors
        tolerance = settings.get('perception.object_detection.tolerance_factor') 
        indices = [i for i in range(len(pcd_image_pixels)) if 
                    (x - tolerance * w/2) <= pcd_image_pixels[i][0] <= (x + tolerance * w/2) and 
                    (y - tolerance * h/2) <= pcd_image_pixels[i][1] <= (y + tolerance * h/2)]
        points = [pcd_vehicle_frame[idx] for idx in indices]   # in vehicle frame
        '''

        # Obtain 3d world coordinates for all pixels in the image
        handler = PixelWise3DLidarCoordHandler()
        all_points = handler.get3DCoord(self.camera_image, self.lidar_point_cloud) # img ht x img width x 3

        # Filter points in bbox
        points = []
        for i in range(x - w/2, x + w/2 + 1):
            for j in range(y - h/2, y + h/2 + 1):
                points.append(all_points[j][i])

        # Estimate center and dimensions
        center = np.mean(points, axis=0)
        dimensions = np.max(points, axis=0) - np.min(points, axis=0)
        
        # For a PhysicalObject, 
        #   origin is at the object's center in the x-y plane and at the bottom in the z axis
        pose = ObjectPose(t=0, x=center[0], y=center[1], z=center[2] - dimensions[2]/2, 
                          yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
        
        return PhysicalObject(pose=pose, dimensions=tuple(dimensions))

    def detect_objects(self, class_ids):
        detection_result = self.detector(self.camera_image, classes=class_ids, verbose=False)
        bbox_locations = detection_result[0].boxes.xywh.tolist()
        bbox_classes = detection_result[0].boxes.cls.tolist()

        detected_objects = []
        for i in range(len(bbox_locations)):
            detected_object = self.box_to_object(bbox_locations[i])
            detected_objects.append(detected_object)
        
        return detected_objects, bbox_classes
    
    def deduplication(self, obj : PhysicalObject, prev_states):
        """ For dedupliction:
        - Check if object was detected before using the previous states.
        - If seen before, 
            return the dict key corresponding to the previous object matching the current one. 
        """
        
        polygon = obj.polygon_parent()

        for key in prev_states:
            prev_obj = prev_states[key]
            prev_polygon = prev_obj.polygon_parent()

            if collisions.polygon_intersects_polygon_2d(polygon, prev_polygon):
                return key
        
        return None
