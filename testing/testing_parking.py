import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
from ultralytics import YOLO

sobel_kernel_size = 3
sobel_min_threshold = 90
conf_val = 0.85

MODEL_WEIGHT_PATH = 'GEMstack/knowledge/detection/parking_spot_detection.pt'
model = YOLO(MODEL_WEIGHT_PATH)
bbox_id_counter = 1
bridge = CvBridge()

from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

# Initialize handler
handler = PixelWise3DLidarCoordHandler()

def get_midpoint(p1, p2):
    return ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)

def draw_center_line(canvas):
    height, width = canvas.shape[:2]  # Get the shape of the image
    mid_x = width // 2  # Calculate the midpoint of width
    mid_y = height // 2  # Calculate the midpoint of height
        # Draw the middle line
    cv2.line(canvas, (mid_x, 0), (mid_x, height), (255, 0, 0), 2)
    return canvas

# acute angle between midpoint box line and middle of the screen line
def calculate_angle(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    angle_radians = np.arctan2(dy, dx)
    return angle_radians

def draw_equidistant_line(points, canvas):
    # Sort the points based on their y-values 
    points = sorted(points, key=lambda x: x[1])

    # Assume the first and last points after sorting belong to the longest lines
    p1, p2 = points[0], points[2]  # top points of the box
    p3, p4 = points[1], points[3]  # bottom points of the box

    # Calculate midpoints
    top_mid = get_midpoint(p1, p2)
    bottom_mid = get_midpoint(p3, p4)

    # Draw the red midpoint line
    cv2.line(canvas, top_mid, bottom_mid, (0, 0, 255), 2)  

    # Calculate the midpoint of the red line
    red_line_midpoint = get_midpoint(top_mid, bottom_mid)

    # Print the coordinate of the midpoint of the red line
    print(f"Midpoint of the red line: {red_line_midpoint}")
    
    # Calculate the angle of the red line relative to the horizontal in radians
    angle_red = calculate_angle(top_mid, bottom_mid)
    # Compute the positive difference from π/2 radians (90 degrees in radians)
    angle_between = abs(np.pi/2 - angle_red)  # Ensure the result is always positive
    print(f"Angle between the red and blue lines: {angle_between} radians")

    return canvas


def get_rotated_box_points(x, y, width, height, angle):
    rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                          [width / 2, height / 2], [-width / 2, height / 2]])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
    return np.int0(rotated_rectangle)

def empty_detect(img):
    global model, bbox_id_counter
    results = model(img)
    for box, conf in zip(results[0].obb, results[0].obb.conf):
        class_id = int(box.cls[0].item())
        confidence = float(conf.item())
        if class_id == 0 and confidence >= conf_val:
            x, y, w, h, r = box.xywhr[0].tolist()
            return (bbox_id_counter, x, y, w, h, r)
    return None

# sanity checking

def perpendicular_distance(point, line_start, line_end):
    # Calculate the distance of a point to a line defined by two points (line_start and line_end)
    num = abs((line_end[1] - line_start[1]) * point[0] - (line_end[0] - line_start[0]) * point[1] + line_end[0] * line_start[1] - line_end[1] * line_start[0])
    den = np.sqrt((line_end[1] - line_start[1])**2 + (line_end[0] - line_start[0])**2)
    return num / den

def ros_PointCloud2_to_numpy(pc2_msg, want_rgb=False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])

    if want_rgb:
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError(
                "PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0], 6))
        xyzrgb[:, :3] = xyzpack[:, :3]
        for i, x in enumerate(xyzpack):
            rgb = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # r,g,b values in the 0-255 range
            xyzrgb[i, 3:] = (r, g, b)
        return xyzrgb
    else:
        return np.array(list(gen), dtype=np.float32)[:, :3]

class ImageProcessorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/oak/rgb/modified_image", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)
        self.image = None
        self.point_cloud = None

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("CvBridge Error:", e)
            return

        if self.point_cloud is None:
            return  # Wait until the point cloud is available

        try:
            numpy_point_cloud = ros_PointCloud2_to_numpy(self.point_cloud)  # Convert PointCloud2 to numpy array
        except Exception as e:
            print(f"PointCloud conversion error: {e}")
            return

        # Get 3D coordinates
        coord_3d_map = handler.get3DCoord(cv_image, numpy_point_cloud)

        # Print 3D coordinates
        print(f"Sample 3D coordinate: {coord_3d_map[100][100]}")

        # Continue with your existing image processing operations
        canvas = cv_image.copy()
        bbox_info = empty_detect(cv_image)
        if bbox_info:
            id, x, y, w, h, r = bbox_info
            points = get_rotated_box_points(x, y, w, h, -r)

            buffer = 20
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)
            x_min = max(x_min - buffer, 0)
            y_min = max(y_min - buffer, 0)
            x_max = min(x_max + buffer, cv_image.shape[1])
            y_max = min(y_max + buffer, cv_image.shape[0])
            cropped_image = cv_image[y_min:y_max, x_min:x_max]

            gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel_size)
            sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel_size)
            sobel = cv2.magnitude(sobelx, sobely)

            _, sobel_thresholded = cv2.threshold(sobel, sobel_min_threshold, 255, cv2.THRESH_BINARY)

            # Apply the green mask on detected lines in the original image within the bounding box
            mask = (sobel_thresholded > 0)
            canvas[y_min:y_max, x_min:x_max][mask] = [0, 255, 0]  # Green mask on detected lines

            canvas = draw_equidistant_line(points, canvas)
            canvas = draw_center_line(canvas)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except CvBridgeError as e:
            print("CvBridge Error during modified image publishing:", e)


if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")