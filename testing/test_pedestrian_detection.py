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
from GEMstack.state import VehicleState, AgentEnum
import numpy as np
import pathlib
from ultralytics import YOLO
import matplotlib.pyplot as plt
import random

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

import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--test_target', '-t', type=str, required=False, 
                    choices=['fuse', 'lidar_to_image', 'lidar_to_image_dbscan',
                             'detect_agents', 'box_to_agent', 'update'])
parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--data_idx', '-i', type=int, default=1)

args = parser.parse_args()

OUTPUT_DIR = args.output_dir

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

import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

colors = []
colors += [(i, 0, 255) for i in range(100, 256, 25)]
colors += [(i, 255, 0) for i in range(100, 256, 25)]
colors += [(0, i, 255) for i in range(100, 256, 25)]
colors += [(255, i, 0) for i in range(100, 256, 25)]
colors += [(255, 0, i) for i in range(100, 256, 25)]
colors += [(0, 255, i) for i in range(100, 256, 25)]
seed = 19
random.seed(seed)
random.shuffle(colors)

id2str = {
    0: "pedestrian",
    2: "car",
    11: "stop sign"
}
    

class TestHelper:
    def __init__(self, ped_detector, point_cloud, zed_image, depth):
        self.ped_detector = ped_detector
        self.yolo_detector = ped_detector.detector
        self.point_cloud = point_cloud
        self.zed_image = zed_image
        self.depth = depth        
    
    def test_fuse_lidar_image(self):
        detection_result = self.yolo_detector(self.zed_image,verbose=False)
        
        #TODO: create boxes from detection result
        boxes = []
        target_ids = [0, 2, 11]
        bbox_ids = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id in target_ids: # class 0 stands for pedestrian
                bbox_ids.append(class_id)
                bbox = box.xywh[0].tolist()
                boxes.append(bbox)
            
        # Only keep lidar point cloud that lies in roi area for agents
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        epsilon = 0.09  # Epsilon parameter for DBSCAN
        
        # for epsilon in np.linspace(0.01, 0.2, 10):
        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)

        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        self.ped_detector.extrinsic, 
                                        self.ped_detector.intrinsic)

        vis = self.zed_image.copy()
        for i in range(len(boxes)):
            box = boxes[i]
            id = bbox_ids[i]
            print ("id:", id)
            
            x,y,w,h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2
            
            # Filter. Get the idxs of point cloud that belongs to the agent
            idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
                            (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
            agent_image_pc = point_cloud_image[idxs]
            agent_pc_3D = filtered_point_cloud[idxs]
            agent_clusters = clusters[idxs]
            
            # draw bbox
            color =  (255, 0, 255)
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)

            # Get unique elements and their counts
            unique_elements, counts = np.unique(agent_clusters, return_counts=True)
            max_freq = np.max(counts)
            label_cluster = unique_elements[counts == max_freq]
            
            # filter again
            idxs = agent_clusters == label_cluster
            agent_image_pc = agent_image_pc[idxs]
            agent_clusters = agent_clusters[idxs]
            agent_pc_3D = agent_pc_3D[idxs]
            
            # calulate depth
            depth = np.mean( (agent_pc_3D[:, 0] ** 2 + agent_pc_3D[:, 1] ** 2) ** 0.5 ) # euclidean dist
            
            # draw
            text = str(id2str[id])
            text += f" | depth: {depth:.2f}"
            text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            p1 = (left_up[0], left_up[1] - text_size[1])
            cv2.rectangle(vis, (p1[0] - 2 // 2, p1[1] - 2 - baseline), (p1[0] + text_size[0], p1[1] + text_size[1]),
                        color, -1)
            cv2.putText(vis, text, (p1[0], p1[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, 8)
            
            # draw point cloud
            for proj_pt, cluster in zip(agent_image_pc, agent_clusters):
                if cluster != label_cluster:
                    continue
                color = colors[cluster % len(colors)]
                radius = 1
                center = int(proj_pt[0]), int(proj_pt[1])
                vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
            
            
        # output
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'fuse_{epsilon}_{args.data_idx}.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)
        
    def test_lidar_to_image_dbscan(self):
        print ('\nTest function lidar_to_image_dbscan()...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        epsilon = 0.09  # Epsilon parameter for DBSCAN
        
        # for epsilon in np.linspace(0.01, 0.2, 10):
        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)

        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        self.ped_detector.extrinsic, 
                                        self.ped_detector.intrinsic)
    
        vis = self.zed_image.copy()
        for proj_pt, cluster in zip(point_cloud_image, clusters):
            color = colors[cluster % len(colors)]
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'lidar_to_image_{epsilon}.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)
        
    def test_lidar_to_image(self, framenum=1):
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
        output_path = os.path.join(OUTPUT_DIR, f'lidar_to_image{framenum}.png')
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

    def test_box_to_agent(self, framenum=1):
        print ('\nTest function box_to_agent()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        

        detection_result = self.yolo_detector(self.zed_image,verbose=False)
        
        bbox_image = self.zed_image.copy()
        
        #TODO: create boxes from detection result
        pedestrian_boxes = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
           class_id = int(box.cls[0].item())
           if class_id == 0: # class 0 stands for pedestrian
               bbox = box.xywh[0].tolist()
               pedestrian_boxes.append(bbox)
 
               # draw bbox
               x,y,w,h = bbox
               xmin, xmax = x - w/2, x + w/2
               ymin, ymax = y - h/2, y + h/2
               bbox_image = cv2.rectangle(bbox_image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2) 
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'bbox_image{framenum}.png')
        print ('Output image with bbox result:', output_path)
        cv2.imwrite(output_path, bbox_image)
    
        # Only keep lidar point cloud that lies in roi area for agents
        point_cloud_lidar = filter_lidar_by_range(ped_detector.point_cloud, 
                                                  ped_detector.xrange, 
                                                  ped_detector.yrange)
        
        # Perform DBSCAN clustering
        epsilon = 0.09  # Epsilon parameter for DBSCAN
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(point_cloud_lidar)

        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(point_cloud_lidar, ped_detector.extrinsic, ped_detector.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = lidar_to_vehicle(point_cloud_lidar, ped_detector.T_lidar2_Gem)

        # Find agents
        detected_agents = []
        print ('Detected {} persons'.format(len(pedestrian_boxes)))
        for i,b in enumerate(pedestrian_boxes):
            agent = ped_detector.box_to_agent(b, point_cloud_image, point_cloud_image_world)#, clusters)
        #     plot_object('agent', agent)
        # klampt_vis(self.zed_image, point_cloud_lidar, self.depth)
        
    def test_track_agents(self, framenum=80):
        for i in range(1, framenum):
            lidar_fn = os.path.join(args.src_dir, f'lidar{i}.npz')
            image_fn = os.path.join(args.src_dir, f'color{i}.png')
            depth_fn = os.path.join(args.src_dir, f'depth{i}.tif')
        
            point_cloud = np.load(lidar_fn)['arr_0']
            image = cv2.imread(image_fn)
            depth = cv2.imread(depth_fn)
            self.depth = depth

            
            self.ped_detector.test_set_data(image, point_cloud)

            detected_agents, detection_result = self.ped_detector.detect_agents(test=True)
            
            detected_pedestrians = [x for x in detected_agents if x.type==AgentEnum.PEDESTRIAN]
            
            current_agent_states, matches = self.ped_detector.track_agents(None,detected_agents, test=True)
            rev_matches = {v:k for k,v in matches.items()}

            
            bbox_image = image.copy()
            
            #TODO: create boxes from detection result
            pedestrian_boxes = []
            detected_ped_id = 0
            for box in detection_result[0].boxes: # only one image, so use index 0 of result
                class_id = int(box.cls[0].item())
                if class_id == 0: # class 0 stands for pedestrian
                    bbox = box.xywh[0].tolist()
                    pedestrian_boxes.append(bbox)

                    pid = rev_matches[detected_ped_id]
                    ag_state = current_agent_states[pid]
        
                    # draw bbox
                    x,y,w,h = bbox
                    xmin, xmax = x - w/2, x + w/2
                    ymin, ymax = y - h/2, y + h/2
                    
                    # What our program measured before kalman
                    m = detected_pedestrians[detected_ped_id]
                    
                    bbox_image = cv2.rectangle(bbox_image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2) 
                    bbox_image = cv2.putText(
                        img = bbox_image,
                        # text = f"PID:{pid}, XY:{round(ag_state.pose.x, 2)},{round(ag_state.pose.y, 2)}, VELXY:{round(ag_state.velocity[0], 2)},{round(ag_state.velocity[1], 2)}",
                        text = f"PID:{pid}, XY:{round(ag_state.pose.x, 2)},{round(ag_state.pose.y, 2)}, mXY:{round(m.pose.x, 2)},{round(m.pose.y, 2)}",

                        org = (int(xmin) - 300, int(ymax)-10),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale = 0.5,
                        color = (0, 0, 255),
                        thickness = 2
                    )
                    detected_ped_id += 1
            
            pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
            output_path = os.path.join(OUTPUT_DIR, f'bbox_image{i}.png')
            print ('Output image with bbox result:', output_path)
            cv2.imwrite(output_path, bbox_image)
          


if __name__=='__main__':
    extrinsic = [[-0.00519, -0.99997, 0.005352, 0.1627], 
                 [-0.0675, -0.00499, -0.9977, -0.03123], 
                 [0.99771, -0.00554, -0.06743, -0.7284],
                 [0,       0 ,             0 ,      1]]
    
    gem_interface = GEMInterface()
    ped_detector = PedestrianDetector(gem_interface, extrinsic)
    
    
    test_helper = TestHelper(ped_detector, None, None, None)
    
    test_helper.test_track_agents(framenum=10)
    # load data
    # all_data = []
    # for i in range(1, 30):
    #     lidar_fn = os.path.join(args.src_dir, f'lidar{i}.npz')
    #     image_fn = os.path.join(args.src_dir, f'color{i}.png')
    #     depth_fn = os.path.join(args.src_dir, f'depth{i}.tif')
    
    #     point_cloud = np.load(lidar_fn)['arr_0']
    #     image = cv2.imread(image_fn)
    #     depth = cv2.imread(depth_fn)

    
    #     test_helper = TestHelper(ped_detector, point_cloud, image, depth)

    #     test_helper.test_box_to_agent(i)
    
    # if args.test_target == 'fuse':
    #     test_helper.test_fuse_lidar_image()
    # if args.test_target == 'lidar_to_image':
    #     test_helper.test_lidar_to_image()
    # if args.test_target == 'lidar_to_image_dbscan':
    #     test_helper.test_lidar_to_image_dbscan()
    # if args.test_target == 'detect_agents':
    #     test_helper.test_detect_agents()
    # if args.test_target == 'box_to_agent':
    #     test_helper.test_box_to_agent()
    # if args.test_target == 'update':
    #     test_helper.test_update()


    
    print ('\nDone!')
    