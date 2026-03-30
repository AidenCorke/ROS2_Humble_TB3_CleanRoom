''' 
#######################################################
Room Cleaning Planner Node
#######################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# --- Imports ---
# ======================================================
# ROS2 Imports
import rclpy
import yaml
import os

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python import get_package_share_directory


# Custom Imports
from clean_room_interfaces.srv import CleanRoom
from clean_room.planner_core.mapping.map_processor import MapProcessor
from clean_room.planner_core.planning.room_cleaning_path_planner import RoomPathPlanner
from clean_room.planner_core.visualizing.rviz_markers import RVizMarkers

from clean_room.utils.map_utils import MapUtils
from clean_room.utils.geometry_utils import point_in_polygon


# ======================================================
# ----- Node Setup -----
# ======================================================
class RoomCleaningPlanner(Node):
  
    # Node constructor
    def __init__(self):
        super().__init__('room_cleaning_planner')   # Node name
        # Declare parameters
        self.declare_parameter('rooms_config', '/dev/null')
        self.declare_parameter('map_yaml', '/dev/null')

        rooms_path = self.get_parameter('rooms_config').value
        map_yaml_path = self.get_parameter('map_yaml').value

        # Load room config
        if rooms_path == "/dev/null":
            self.get_logger().error("rooms_path parameter not set")
            return        
        else:
            with open(rooms_path, 'r') as f:
                self.room_config = yaml.safe_load(f)['rooms']

        # Load map metadata
        if rooms_path == "/dev/null":
            self.get_logger().error("maps_path parameter not set")
            return         
        else:
            self.map_utils = MapUtils(map_yaml_path)

        # Load map image
        self.mp = MapProcessor(self.map_utils.map_image_path)

        # Subscribe amcl_pose to collect pose of robot
        self.current_pose = None
        self.pose_sub = self.create_subscription(PoseStamped,
                                                 '/amcl_pose',
                                                 self.pose_callback,
                                                 10
                                                 )

        # Create Nav2 action clients
        self.follow_wp_client = ActionClient(self, FollowWaypoints, 'navigate_to_wp')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        '''     
        # Clean room service
        self.clean_room_srv = self.create_service(CleanRoom,
                                                  'clean_room',
                                                  self.clean_room_callback,
                                                  )
        '''

        # Path planning visualization function calls
        self.markers = RVizMarkers(self)

        self.get_logger().info("Room Cleaning Planner Node initialized.")

    # ===================================================================
    # ----- Service / Client Functions -----
    # ===================================================================
    # -------------------------------------------------------------------
    # --- Service callback ---
    # -------------------------------------------------------------------
    def clean_room_callback(self, request, response):
        room_name = request.room_name

        try:
            self.clean_room(room_name)
            response.accepted = True
            response.message = f"Started cleaning {room_name}"
        except Exception as e:
            response.accepted = False
            response.message = f"Failed: {str(e)}"

        return response
    


    # ===================================================================
    # ----- Map/Waypoint Functions -----
    # ===================================================================
    # -------------------------------------------------------------------
    # --- Current room detection ---
    # -------------------------------------------------------------------
    def detect_current_room(self):
        """ This function detects and returns the room the robot is currently in using current pose and defined room boundaries."""
        # If no pose is present break out and return None
        if self.current_pose is None:
            return None

        # Convert robot pose to pixel coordinates
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        px, py = self.map_utils.world_to_pixel(x, y)

        # Check each room polygon
        for room_name, params in self.room_config.items():
            if point_in_polygon((px, py, params['corners'])):
                return room_name

        return None

    
    # -------------------------------------------------------------------
    # --- Load in room configs ---
    # -------------------------------------------------------------------
    def load_room_config(self):
        """ This function loads room configurations from a yaml file."""
        pkg_path = get_package_share_directory('room_cleaning_planner')
        config_path = os.path.join(pkg_path, 'config', 'rooms.yaml')

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        self.room_config = data['rooms']


    # -------------------------------------------------------------------
    # --- Extract room parameters ---
    # -------------------------------------------------------------------
    def get_room_params(self, room_name):
        """ This function extracts room parameters from loaded in config files."""

        # If room doesnt exist throw an error
        if room_name not in self.room_config:
            raise ValueError(f"Room '{room_name}' not found in config")

        params = self.room_config[room_name]
        return (
            params['corners'],
            params['entry_point'],
            params['mesh_resolution'],
            params['inflation_radius']
        )   

    # -------------------------------------------------------------------
    # --- Collect pose ---
    # -------------------------------------------------------------------
    def pose_callback(self, msg):
        """ This function collects the robots current pose."""
        self.current_pose = msg.pose


    # -------------------------------------------------------------------
    # --- Find closest waypoint to start pose ---
    # -------------------------------------------------------------------
    def find_closest_waypoint(self, waypoints):
        """ This function finds the closest waypoint to its current location"""

        if self.current_pose is None:   # If no current pose found break function and return node 0
            return 0

        px = self.current_pose.position.x
        py = self.current_pose.position.y

        # Compute distances between waypoints and current position
        dists = [(i, (wx - px)**2 + (wy - py)**2)
                 for i, (wx, wy) in enumerate(waypoints)]

        return min(dists, key=lambda x: x[1])[0]


    # -------------------------------------------------------------------
    # --- Waypoint conversion ---
    # -------------------------------------------------------------------
    def waypoint_to_pose(self, x, y):
        """ This function converts waypoints to pose."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0  # facing forward
        return pose     




    # ===================================================================
    # ----- Nav2 Functions -----
    # ===================================================================
    # -------------------------------------------------------------------
    # --- Send waypoints to Nav2 ---
    # -------------------------------------------------------------------
    def navigate_to_wp(self, ordered_waypoints):
        """ This function sends all waypoint to Nav2 FollowWaypoint client."""

        poses = []
        for x, y in ordered_waypoints:  # Loop through all waypoints
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.follow_wp_client.wait_for_server()                 # Wait for response from server
        future = self.follow_wp_client.send_goal_async(goal)    # Send goal async
        future.add_done_callback(self._follow_wp_response)      # Create a callback for when done


    # -------------------------------------------------------------------
    # --- Send point to Nav2 ---
    # -------------------------------------------------------------------
    def navigate_to_point(self, x, y):
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0   # Face forward

        goal.pose = pose

        self.nav_client.wait_for_server()                   # Wait for response from server
        future = self.nav_client.send_goal_async(goal)      # Send goal async
        future.add_done_callback(self._nav_goal_response)   # Ad callback for when goal done                


    # -------------------------------------------------------------------
    # --- Nav2 client response handling - waypoints ---
    # -------------------------------------------------------------------
    def _follow_wp_response(self, future):
        """ This function handles responses from Nav2 client if goal was accepted or not."""

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Waypoint navigation goal rejected!")
            return

        self.get_logger().info("Waypoint navigation goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_wp_result)   


    # -------------------------------------------------------------------
    # --- Nav2 client response handling - point ---
    # -------------------------------------------------------------------
    def _nav_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigate to room goal rejected")
            return

        self.get_logger().info("Navigate to room goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)
  

    # -------------------------------------------------------------------
    # --- Nav2 client completion message update - waypoints ---
    # -------------------------------------------------------------------
    def _follow_wp_result(self, future):
        self.get_logger().info("Clean room navigation has completed.")


    # -------------------------------------------------------------------
    # --- Nav2 client completion message update - point ---
    # -------------------------------------------------------------------
    def _nav_result(self, future):
        self.get_logger().info("Navigation to room completed")



    # ===================================================================
    # ----- Main Cleaning Logic -----
    # ===================================================================
    def clean_room(self, room_name):
        """ This function exectutes cleaning room planning and sends resulting path to Nav2 client for execution."""

        corners = self.room_config[room_name]['corners']
        entry = self.room_config[room_name]['entry_point']
        mesh_res = self.room_config[room_name]['mesh_resolution']
        inflation = self.room_config[room_name]['inflation_radius']

        # Visualize room polygon
        self.markers.publish_room_polygon(corners, self.map_utils.pixel_to_world)

        # Room detection and navigation
        current_room = self.detect_current_room()
        if current_room != room_name:
            self.get_logger().info(f"Robot in {current_room}, navigating to {room_name} now...")
            xe, ye = self.map_utils.pixel_to_world(entry[0],entry[1])
            self.navigate_to_point(xe,ye)   # Navigate to designated entry point
            return
        self.get_logger().info(f"Currently in {room_name}, beginning cleaning...")

        # Path Planning
        planner = RoomPathPlanner(self.mp,
                                  corners,
                                  mesh_res,
                                  inflation)

        waypoints = planner.generate_waypoints()    # Generate waypoints
        start_idx = self.find_closest_waypoint(waypoints)                   # Find starting waypoint
        path_indices = planner.compute_full_path(start_index=start_idx)     # Compute optimal path
        ordered_waypoints = [waypoints[i] for i in path_indices]            # Convert waypoints numbers to coordinates
        
        # Visualization of path in RViz
        self.markers.publish_room_polygon(corners)
        self.markers.publish_waypoints(waypoints)
        self.markers.publish_path(ordered_waypoints)

        # Execute cleaning path
        self.execute_waypoints(ordered_waypoints) # Send waypoints to Nav2             


def main(args=None):
    rclpy.init(args=args)
    node = RoomCleaningPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        