''' 
#######################################################################
# ----- Room Cleaning Planner Geometry Related Utility Functions -----
#######################################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# ----- Imports -----
# ======================================================
from matplotlib.path import Path
import math
from geometry_msgs.msg import Quaternion

# ======================================================
# ----- Functions -----
# ======================================================

def point_in_polygon(px, py, polygon):
    """
    polygon: list of (x, y) pixel coordinates
    """
    poly = Path(polygon)
    return poly.contains_point((px, py))

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def angle_between(v1, v2):
    """
    v1, v2: (x, y) vectors
    """
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

    if mag1 == 0 or mag2 == 0:
        return 0.0

    cos_angle = max(min(dot / (mag1 * mag2), 1.0), -1.0)
    return math.acos(cos_angle)

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q