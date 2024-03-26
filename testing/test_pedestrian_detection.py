#needed to import GEMstack from top level directory
import sys
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pedestrian_detection import PedestrianDetector, lidar_to_image, lidar_to_vehicle, filter_lidar_by_range
from GEMstack.onboard.interface.gem import GEMInterface
# from GEMstack.utils.mpl_visualization import plot_object
from GEMstack.utils.klampt_visualization import plot_object
from GEMstack.state import VehicleState
import numpy as np
import pathlib
from ultralytics import YOLO
import matplotlib.pyplot as plt

# for klampt
from klampt import vis
from klampt.math import so3,se3
from klampt.vis.colorize import colorize
from klampt import PointCloud,Geometry3D
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points
import cv2
import os
import numpy as np
import math
import time

OUTPUT_DIR = 'output'

def klampt_vis(zed_image, lidar_point_cloud, depth):
    
    zed_K = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
    zed_K = np.array(zed_K).reshape((3, 3))
    zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
    zed_w = 1280
    zed_h = 720


    lidar_xform = se3.identity()
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    data = {}
    def load_and_show_scan(idx, depth=depth):
        pc = numpy_convert.from_numpy(lidar_point_cloud,'PointCloud')
        pc = colorize(pc,'z','plasma')
        data['lidar'] = Geometry3D(pc)

        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        pc = image_to_points(depth,zed_image,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='PointCloud')

        data['zed'] = Geometry3D(pc)
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        vis.add('lidar',data['lidar'])
        vis.add('zed',data['zed'])

    data['index'] = 1
    def increment_index():
        data['index'] += 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] -= 1
            return
    def decrement_index():
        data['index'] -= 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] += 1
            return
    def print_xforms():
        print("lidar:")
        print("rotation:",so3.ndarray(lidar_xform[0]))
        print("position:",lidar_xform[1])
        print("zed:")
        print("rotation:",so3.ndarray(zed_xform[0]))
        print("position:",zed_xform[1])


    vis.addAction(increment_index,"Increment index",'=')
    vis.addAction(decrement_index,"Decrement index",'-')
    vis.addAction(print_xforms,'Print transforms','p')
    load_and_show_scan(1)
    vis.add('zed_xform',zed_xform)
    vis.add('lidar_xform',lidar_xform)
    vis.edit('zed_xform')
    vis.edit('lidar_xform')
    vis.show()
    while vis.shown():
        lidar_xform = vis.getItemConfig('lidar_xform')
        lidar_xform = lidar_xform[:9],lidar_xform[9:]
        zed_xform = vis.getItemConfig('zed_xform')
        zed_xform = zed_xform[:9],zed_xform[9:]
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        time.sleep(0.02)
    vis.kill()

class TestHelper:
    def __init__(self, ped_detector, point_cloud, zed_image, depth):
        self.ped_detector = ped_detector
        self.point_cloud = point_cloud
        self.zed_image = zed_image
        self.depth = depth

    def test_lidar_to_image(self):
        print ('\nTest function lidar_to_image()...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.ped_detector.extrinsic, 
                                           self.ped_detector.intrinsic)
        vis = self.zed_image.copy()
        for proj_pt in point_cloud_image:
            color = (0, 255, 0) # green
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, 'lidar_to_image.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)

    def test_update(self):
        print ('\nTest function update()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        self.ped_detector.update(VehicleState.zero())

    def test_detect_agents(self):
        print ('\nTest function detect_agents()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        self.ped_detector.detect_agents()

    def test_box_to_agent(self):
        print ('\nTest function box_to_agent()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        
        yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8n.pt')
        detector = YOLO(yolo_path)
        detection_result = detector(self.zed_image,verbose=False)
        
        #TODO: create boxes from detection result
        pedestrian_boxes = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
           class_id = int(box.cls[0].item())
           if class_id == 0: # class 0 stands for pedestrian
               bbox = box.xywh[0].tolist()
               pedestrian_boxes.append(bbox)
    
        # Only keep lidar point cloud that lies in roi area for agents
        point_cloud_lidar = filter_lidar_by_range(ped_detector.point_cloud, 
                                                  ped_detector.xrange, 
                                                  ped_detector.yrange)
        
        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(point_cloud_lidar, ped_detector.extrinsic, ped_detector.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = lidar_to_vehicle(point_cloud_lidar, ped_detector.T_lidar2_Gem)

        # Find agents
        detected_agents = []
        print ('Detected {} persons'.format(len(pedestrian_boxes)))
        for i,b in enumerate(pedestrian_boxes):
            agent = ped_detector.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            plot_object('agent', agent)
        klampt_vis(self.zed_image, point_cloud_lidar, self.depth)
          


if __name__=='__main__':
    gem_interface = GEMInterface()
    ped_detector = PedestrianDetector(gem_interface)
    
    data_idx = 1
    point_cloud = np.load(os.path.join(abs_path, f'../data/step1_rgb_image_stereo_depth/lidar{data_idx}.npz'))
    point_cloud = point_cloud['arr_0']

    zed_image = cv2.imread(os.path.join(abs_path, f'../data/step1_rgb_image_stereo_depth/color{data_idx}.png'))
    depth = cv2.imread(os.path.join(abs_path, f'../data/step1_rgb_image_stereo_depth/depth{data_idx}.tif'), cv2.IMREAD_UNCHANGED)
    
    test_helper = TestHelper(ped_detector, point_cloud, zed_image, depth)
    
    # test_helper.test_lidar_to_image()
    # test_helper.test_detect_agents()
    test_helper.test_box_to_agent()
    # test_helper.test_update()
    
    print ('\nDone!')
    