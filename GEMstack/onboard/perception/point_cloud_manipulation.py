from ...utils import settings
import numpy as np
from typing import Tuple


T_lidar = np.eye(4)
T_lidar[:3,:3] = np.array(settings.get('vehicle.calibration.top_lidar.rotation'))
T_lidar[:3,3] = np.array(settings.get('vehicle.calibration.top_lidar.position'))

T_camera = np.eye(4)
T_camera[:3,:3] = np.array(settings.get('vehicle.calibration.front_camera.rotation'))
T_camera[:3,3] = np.array(settings.get('vehicle.calibration.front_camera.rgb_position'))


def project_point_cloud(pcd, T):
    """Projects a point cloud in one frame to another frame.

    Args:
        pcd: 3D point cloud
        T: 4x4 transformation matrix (from old to new frame)

    Returns:
        pcd_new_frame: 3D point cloud in the new frame
    """
    
    pcd_temp = np.hstack((pcd, np.ones((pcd.shape[0], 1))))
    pcd_temp = pcd_temp.transpose()
    
    pcd_new_frame = (T @ pcd_temp).transpose()[:, :3]
    return pcd_new_frame


def project_point_cloud_camera_to_image(pcd : np.ndarray, P : np.ndarray, xrange : Tuple[int,int], 
                                        yrange : Tuple[int,int]) -> Tuple[np.ndarray,np.ndarray]:
    """Projects a point cloud in the camera frame to a 2D image using a intrinsic matrix P.
    
    Returns:
        - pcd_image_pixels: an Nx2 array of (u,v) visible image coordinates
        - indices: an array of N indices of visible points into the original point cloud
    """

    pcd_with_ids = np.hstack((pcd, np.arange(len(pcd)).reshape(-1, 1)))
    pcd_fwd = pcd_with_ids[pcd_with_ids[:, 2] > 0]
    
    pxform = pcd_fwd[:,:3].dot(P[:3,:3].T) + P[:3,3]
    uv = (pxform[:,0:2].T / pxform[:,2]).T
    inds = np.logical_and(np.logical_and(uv[:,0] >= xrange[0], uv[:,0] < xrange[1]),
                          np.logical_and(uv[:,1] >= yrange[0], uv[:,1] < yrange[1]))
    
    pcd_image_pixels = uv[inds]
    indices = pcd_fwd[inds,3].astype(int)
    
    return pcd_image_pixels, indices


def transform_point_cloud(pcd, camera_P, image_w, image_h):
    """Transforms a point cloud in the lidar frame to the vehicle and the image frames.

    Args:
        pcd: 3D point cloud in the lidar frame
        camera_P: intrinsic matrix
        image_w: image width
        image_h: image height

    Returns:
        pcd_image_pixels: image coordinates corresponding to the lidar points
        pcd_vehicle_frame: 3D point cloud in the vehicle frame
    """

    # Transform LiDAR points to camera frame
    T_lidar_to_camera = np.linalg.inv(T_camera) @ T_lidar
    pcd_camera_frame = transform_point_cloud(pcd, T_lidar_to_camera)

    # Project the 3D points in the camera frame onto the 2D image
    pcd_image_pixels, indices = project_point_cloud_camera_to_image(
        pcd_camera_frame, camera_P, (0, image_w), (0, image_h)
    )

    # Convert the visible points from camera frame to vehicle frame
    pcd_camera_frame = pcd_camera_frame[indices] 
    pcd_vehicle_frame = transform_point_cloud(pcd_camera_frame, T_camera)

    return pcd_image_pixels, pcd_vehicle_frame
