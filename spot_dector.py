import cv2
import numpy as np

def point_line_distance(point, segment):
    p = np.array(point)
    a = np.array(segment[0])
    b = np.array(segment[1])

    # Project vector ap onto ab to find the closest point
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab) / np.dot(ab, ab)

    # Find the closest point on the segment to the point
    t = max(0, min(1, t)) # Ensure t is within the bounds of the line segment
    closest_point = a + t * ab
    return np.linalg.norm(p - closest_point)

def find_perpendicular_point(x1, y1, x2, y2, x3, y3):
    dx = x2 - x1
    dy = y2 - y1

    if dx == 0:
        return x1, y3 # vertical
    
    m1 = dy / dx
    b1 = y1 - m1 * x1
    
    # The line through (x3, y3) and perpendicular to the original line
    if m1 == 0:
        return x3, y1 # horizontal
    m2 = -1 / m1
    b2 = y3 - m2 * x3
    
    # Intersection point of the two lines
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    return int(x), int(y)


def find_dirction_vector(x1, x2, y1, y2):
  dx = x2 - x1
  dy = y2 - y1

  length = np.sqrt(dx**2 + dy**2)

  unit_dx = dx / length
  unit_dy = dy / length
  return unit_dx, unit_dy

def convert_to_pixels(object_size, object_distance, focal_length):
    # Assuming object_size is in centimeters and focal_length is in pixels
    image_size = (object_size / object_distance) * focal_length
    return image_size

def find_spot(frame, agent_position, vehicle_length):
    image = cv2.imread(frame)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blurred, 100, 200)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=20)

    # Filter lines
    curbside_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if abs(angle) > 30 and abs(angle) < 150:  # Adjust angle threshold as needed
            curbside_lines.append(line)

    # Variables to keep track of the closest line
    min_distance = float('inf')
    closest_line = None

    for line in curbside_lines:
        x1, y1, x2, y2 = line[0]
        segment = ((x1, y1), (x2, y2))
        distance = point_line_distance(agent_position, segment)
        if distance < min_distance:
            min_distance = distance
            closest_line = line

    if closest_line is not None:
        # Pedestrian Position
        cv2.circle(image, agent_position, 5, (0, 0, 255), -1)

        cv2.line(image, (closest_line[0][0], closest_line[0][1]), (closest_line[0][2], closest_line[0][3]), (0, 255, 0), 2)
        x4, y4 = find_perpendicular_point(closest_line[0][0], closest_line[0][1], closest_line[0][2], closest_line[0][3], agent_position[0], agent_position[1])

        # TODO: Include depth
        # focal_length = mtx[0, 0]
        # object_distance = 2000
        # object_size = 100  # cm

        # pixels = convert_to_pixels(object_size, object_distance, focal_length)

        # Calculate two points that are 100 pixels apart from picked point
        unit_dx, unit_dy = find_dirction_vector(closest_line[0][0], closest_line[0][2], closest_line[0][1], closest_line[0][3])
        x5, y5 = int(x4 + vehicle_length * unit_dx), int(y4 + vehicle_length * unit_dy)
        x6, y6 = int(x4 - vehicle_length * unit_dx), int(y4 - vehicle_length * unit_dy)

        cv2.line(image, (x5, y5), (x5 - 200, y5), (0, 255, 0), 2)
        cv2.line(image, (x6, y6), (x6 - 150, y6), (0, 255, 0), 2)

    cv2.imshow('Parking Spots', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    agent_position = (300, 200)
    vehicle_length = 100
    frame = 'curb.jpeg'
    find_spot(frame, agent_position, vehicle_length)

    agent_position = (1500, 800)
    vehicle_length = 100
    frame = 'curb3.jpeg'
    find_spot(frame, agent_position, vehicle_length)