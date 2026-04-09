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
#from clean_room_server.utils import map_utils

# ======================================================
# ----- Class -----
# ======================================================
class RVizMarkers():
    def __init__(self, node, map_utils):
        self.node = node
        self.map_utils = map_utils

        self.room_marker_pub = node.create_publisher(Marker, 'room_polygon', 10)
        self.waypoint_pub = node.create_publisher(MarkerArray, 'cleaning_waypoints', 10)
        self.path_pub = node.create_publisher(Marker, 'cleaning_path', 10)


    # -------------------------------------------------------------------
    # --- Visualize the room ---
    # -------------------------------------------------------------------
    def publish_room_polygon(self, corners_px):
        """This function extract corner points and publishes them as marker called "room_polygon"."""
        # -- Setup marker parameters --
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "room_polygon"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0

        # -- Extract corners --
        for px, py in corners_px + [corners_px[0]]:
            wx, wy = self.map_utils.pixel_to_world(px, py)
            marker.points.append(Point(x=wx, y=wy, z=0.0))

        # -- Publish room polygon --
        self.room_marker_pub.publish(marker)


    # -------------------------------------------------------------------
    # --- Visualize the waypoints of the path ---
    # -------------------------------------------------------------------
    def publish_waypoints(self, waypoints):
        """This function extracts all waypoints and publishes them to "cleaning_waypoints"."""
        # -- Define marker array --
        ma = MarkerArray()

        # -- Loop through each waypoint --
        for i, (px, py) in enumerate(waypoints):
            wx, wy = self.map_utils.pixel_to_world(px,py)
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
            m.pose.position.x = wx
            m.pose.position.y = wy
            ma.markers.append(m)

        # -- Publish waypoints --
        self.waypoint_pub.publish(ma)


    # -------------------------------------------------------------------
    # --- Visualize the path  ---
    # -------------------------------------------------------------------
    def publish_path(self, waypoints):
        """This function extracts and publishes the planned cleaning path as a marker in topic "cleaning_path"."""
        # -- Setup marker parameters --
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # -- Loop through each waypoint --
        for px, py in waypoints:
            wx, wy = self.map_utils.pixel_to_world(px,py)
            p = Point(x=wx, y=wy, z=0.0)
            marker.points.append(p)

        # -- Publish path marker --
        self.path_pub.publish(marker)

    # -------------------------------------------------------------------
    # --- Clear old visualizations  ---
    # -------------------------------------------------------------------
    def clear(self):
        """This function clears existing cleaning visualizations."""
        # -- Publish empty marker array of waypoints --
        self.waypoint_pub.publish(MarkerArray())

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        self.room_marker_pub.publish(clear_marker)
        self.path_pub.publish(clear_marker)        