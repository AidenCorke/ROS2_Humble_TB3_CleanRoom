''' 
##############################################################
# ----- Room Cleaning Planner RViz Visualization Class  -----
##############################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# ----- Imports -----
# ======================================================
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ======================================================
# ----- Class -----
# ======================================================
class RVizMarkers():
    def __init__(self, node):
        self.node = node
        self.room_marker_pub = node.create_publisher(Marker, 'room_polygon', 10)
        self.waypoint_pub = node.create_publisher(MarkerArray, 'cleaning_waypoints', 10)
        self.path_pub = node.create_publisher(Marker, 'cleaning_path', 10)


    # -------------------------------------------------------------------
    # --- Visualize the room ---
    # -------------------------------------------------------------------
    def publish_room_polygon(self, corners):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Convert pixel → world coordinates
        for px, py in corners + [corners[0]]:
            wx, wy = self.mp.pixel_to_world(px, py)
            p = Point(x=wx, y=wy, z=0.0)
            marker.points.append(p)

        self.room_marker_pub.publish(marker)


    # -------------------------------------------------------------------
    # --- Visualize the waypoints of the path ---
    # -------------------------------------------------------------------
    def publish_waypoints(self, waypoints):
        ma = MarkerArray()

        for i, (x, y) in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 0.5
            m.color.b = 1.0
            m.color.a = 1.0
            m.pose.position.x = x
            m.pose.position.y = y
            ma.markers.append(m)

        self.waypoint_pub.publish(ma)


    # -------------------------------------------------------------------
    # --- Visualize the path  ---
    # -------------------------------------------------------------------
    def publish_path(self, waypoints):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in waypoints:
            p = Point(x=x, y=y, z=0.0)
            marker.points.append(p)

        self.path_pub.publish(marker)