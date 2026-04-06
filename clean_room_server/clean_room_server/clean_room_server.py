''' 
#######################################################
Room Cleaning Action Server Node
#######################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# --- Imports ---
# ======================================================
# - ROS2 Imports -
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import PoseStamped, Point
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

# - Standard Imports -
import math
import yaml

# - Custom Imports -
from clean_room_interfaces.action import CleanRoom
from clean_room_server.planner.cleaning_manager import CleaningManager
from clean_room_server.utils.geometry_utils import point_in_polygon, yaw_to_quaternion
from clean_room_server.utils.map_utils import MapUtils
from clean_room_server.planner.visualizing.rviz_markers import RVizMarkers


# - Evaluation Imports -
from evaluator_interfaces.srv import StartCleaning
from std_srvs.srv import Trigger

# ======================================================
# --- Node ---
# ======================================================
class CleanRoomServer(Node):

    def __init__(self):
        super().__init__('clean_room_server')
        # -- Parameters --
        # Declare parameters
        self.declare_parameter('rooms_config', '/dev/null')
        self.declare_parameter('map_yaml', '/dev/null')

        # Extract parameters
        self.rooms_path = self.get_parameter('rooms_config').value
        self.map_yaml_path = self.get_parameter('map_yaml').value

        # Warning if parameters aren't loaded correct
        if self.rooms_path == "/dev/null":
            self.get_logger().error("rooms_path parameter not set")
        if self.map_yaml_path == "/dev/null":
            self.get_logger().error("maps_path parameter not set")

        # -- Action Server Node --
        # create action server node
        self._action_server = ActionServer(self,                                    # Node
                                           CleanRoom,                               # Action type
                                           'clean_room',                            # Action name
                                           execute_callback=self.execute_callback,  # Action execution
                                           cancel_callback=self.cancel_callback     # Action canceling
                                           )
        # -- Pose Listener --
        # Create a buffer and listener to TF in order to get current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        
        # -- Nav2 Class -- 
        # Create Nav2 BasicNavigator instance
        self.navigator = BasicNavigator(node_name='clean_room_navigator')   
        self.navigator.waitUntilNav2Active()    # Wait for Nav2 to fully launch

        # -- Cleaning Logic Class --
        self.manager = CleaningManager(self.rooms_path, self.map_yaml_path)   

        # -- Evaluation Services --
        self.start_eval_cli = self.create_client(StartCleaning, 'start_eval')
        self.start_eval_req = StartCleaning.Request()
        self.stop_eval_cli = self.create_client(Trigger, 'stop_eval')
        self.stop_eval_req = Trigger.Request()

        # -- Visualization --
        self.map_utils = MapUtils(self.map_yaml_path)
        self.viz = RVizMarkers(self, self.map_utils)

    # -------------------------------------------------------------------
    # --- Cancel response recieving ---
    # -------------------------------------------------------------------
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.navigator.cancelTask()
        return CancelResponse.ACCEPT


    # -------------------------------------------------------------------
    # --- Detect current pose ---
    # -------------------------------------------------------------------
    def get_current_pose(self,time_out=1):
        """This function extracts the current pose of the robot from TF, will try for 1 second before ."""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < time_out * 1e9:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(seconds=0)
                )
                pose = PoseStamped()
                pose.header = trans.header
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation

                return pose
            
            except Exception as e:
                warned = False
                while not warned:
                    self.get_logger().warn(f"Current pose lookup failed: {e}")    
                    warned = True
        # Time out reached and pose couldnt be extracted
        self.get_logger().warn("TF pose lookup timed out.")
        return None

    # -------------------------------------------------------------------
    # --- Detect current room ---
    # -------------------------------------------------------------------
    def detect_current_room(self):
        """ This function detects and returns the room the robot is currently in using current pose and defined room boundaries."""

        # Get current pose
        current_pose = self.get_current_pose()

        # Convert robot pose to pixel coordinates
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        px, py = self.manager.map_utils.world_to_pixel(x, y)

        # Checks
        #self.get_logger().info(f"x: {x}  px: {px}")
        #self.get_logger().info(f"y: {y}  py: {py}")

        # Check each room polygon
        for room_name, params in self.manager.room_config.items():
            if point_in_polygon(px, py, params['corners']):
                return room_name

        return None


    # -------------------------------------------------------------------
    # --- Navigate to entry point of room ---
    # -------------------------------------------------------------------
    def navigate_to_entry_point(self, room_name):
        """This function sends a nav2 request to navigate to the entry point of the given room using the simple commander basic navigator class."""
        # Extract entry point from param files
        params = self.manager.room_config[room_name]
        entry_px, entry_py = params["entry_point"]
        xw, yw = self.manager.map_utils.pixel_to_world(entry_px, entry_py)

        # Set the initial (current) pose
        #current_pose = self.get_current_pose()
        #self.navigator.setInitialPose(current_pose)

        # Wait for navigation to fully activate, since autostarting nav2
        #self.navigator.waitUntilNav2Active()

        # Load map file
        #self.navigator.changeMap(self.get_parameter('map_yaml').value)

        # Obtain global and local costmaps
        #self.global_costmap = self.navigator.getGlobalCostmap()
        #self.local_costmap = self.navigator.getLocalCostmap()

        # Create waypoint goal for navigator
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(xw)
        goal.pose.position.y = float(yw)
        goal.pose.orientation.w = 1.0       # Face forward
        goal.pose.orientation.z = 0.0       

        self.get_logger().info(f"Navigating to entry point of {room_name}...")

        nav_to_point = self.navigator.goToPose(goal)    # Send goal
        i = 0
        while not self.navigator.isTaskComplete():
            
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.get_logger().info('Distance remaining: '
                                       + '{:.2f}'.format(feedback.distance_remaining)
                                       + ' meters.'
                                       )            

        # -- Results --    
        result = self.navigator.getResult()  # Extract result
        # Task succeeded
        if result == TaskResult.SUCCEEDED:                                  
            self.get_logger().info('Reached entry point.')

        # Task canceled
        elif result == TaskResult.CANCELED:                                 
            self.get_logger().info('Navigation to entry point canceled!')

        # Task failed
        elif result == TaskResult.FAILED:                                   
            (error_code, error_msg) = self.navigator.getTaskError()
            self.get_logger().warn(f'Entry point navigation failed!{error_code}:{error_msg}')

        # Weird result
        else:                                                              
            self.get_logger().warn('Entry point navigation has an invalid return status!')        
               
        return True


    # -------------------------------------------------------------------
    # --- Navigate through cleaning path ---
    # -------------------------------------------------------------------
    def navigate_waypoints(self, ordered_waypoints, params):
        """This function sends a list of (x, y) waypoints to Nav2 FollowWaypoints."""
        # Load room config
        #with open(self.rooms_path, 'r') as f:
        #    data = yaml.safe_load(f)
        #room = data["room_name"]
        #corners_pix = room['corners']
        
        

        # -- Convert poses into readable goals for nav2 --        
        goal_poses = []
        total = len(ordered_waypoints)

        # Loop through all waypoints adding to pose
        for i, (px, py) in enumerate(ordered_waypoints):
            wx, wy = self.manager.map_utils.pixel_to_world(px,py)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)

            # Compute orientation toward next waypoint
            if i < len(ordered_waypoints) - 1:
                px_n, py_n = ordered_waypoints[i+1]
                wx_n, wy_n = self.manager.map_utils.pixel_to_world(px_n,py_n)
                yaw = math.atan2(wy_n - wy, wx_n - wx)
            else:
                yaw = 0.0  # final waypoint

            pose.pose.orientation = yaw_to_quaternion(yaw)
            goal_poses.append(pose)

        # -- Extract room corners for evaluation service --
        # Loop through each corner 
        boundary_points = []
        for px,py in params['corners']:
            # Convert from pixel to world
            wx, wy = self.manager.map_utils.pixel_to_world(px,py)

            # Store in point message
            point = Point()
            point.x = wx
            point.y = wy
            #point.z = 0

            # Add to list
            boundary_points.append(point)

        # Send evaluation request
        self.start_eval_req.boundary_points = boundary_points
        self.start_eval_cli.call_async(self.start_eval_req)

        # -- Send goal to Nav2 --
        nav_to_wp = self.navigator.followWaypoints(goal_poses)
        wp_past = 5 # Random number
        time_start = self.get_clock().now()
        time_elapsed = 0 # Initialize elapsed time

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            wp_current = feedback.current_waypoint
            time_elapsed = (self.get_clock().now() - time_start)
            if feedback and wp_current != wp_past:  # Loops everytime waypoint is updated
                self.get_logger().info('Executing current waypoint: '
                                       + str(feedback.current_waypoint + 1)
                                       + '/'
                                       + str(len(goal_poses))
                                    )
                #self.get_logger().info('Cleaning Time on Waypoint: ' + str(time_elapsed) + " seconds")
                wp_past = wp_current
            
            # Cancel navigation if elapsed time exceeds 1200 seconds
            if time_elapsed > Duration(seconds=1200):
                self.navigator.cancelTask()
                self.start_eval_cli.destroy()
                self.stop_eval_cli.destroy()


        # -- Results --    
        result = self.navigator.getResult()  # Extract result
        self.stop_eval_cli.call_async(self.stop_eval_req)   # send eval stop request

        # Task succeeded
        if result == TaskResult.SUCCEEDED:                                  
            self.get_logger().info('Cleaning completed successfully!')

        # Task canceled
        elif result == TaskResult.CANCELED:                                 
            self.get_logger().info('Cleaning was cancelled!')
        
        # Task failed
        elif result == TaskResult.FAILED:                                   
            (error_code, error_msg) = self.navigator.getTaskError()
            self.get_logger().warn(f'Cleaning failed!{error_code}:{error_msg}')

        # Weird result
        else:
            self.get_logger().warn('Goal has an invalid return status!')        

        return True



    # -------------------------------------------------------------------
    # --- Execute action ---
    # -------------------------------------------------------------------
    async def execute_callback(self, goal_handle):
        # -- Setup --
        room_name = goal_handle.request.room_name  # Extract room name from goal
        self.get_logger().info(f"Cleaning requested for room: {room_name}") # Publish request recieved
        self.room_name = room_name
        params = self.manager.room_config[self.room_name]

        # -- Visualize Room Polygon -- 
        self.viz.publish_room_polygon(params["corners"])

        # -- Room detection and navigation to entry point if needed --
        current_room = self.detect_current_room()
        if current_room != room_name:
            self.get_logger().info(f"Robot currently in {current_room}.")
            success = self.navigate_to_entry_point(room_name)   # Navigate to designated entry point
            
            # If it fails to reach the entry point
            if not success:
                result = CleanRoom.Result()
                result.success = False
                result.message = f"Failed to reach entry point"
                goal_handle.abort()
                return result
            
        self.get_logger().info(f"Currently in {room_name}, beginning cleaning...")

        # -- Cleaning Operations --
        self.get_logger().info("Starting planning cleaning path...")
        current_pose = self.get_current_pose()
        ordered_waypoints = self.manager.cleaning_path(room_name,
                                                       current_pose,
                                                       )
        self.get_logger().info("Completed planning cleaning path.")
        
        # -- Visualize cleaning path --
        self.viz.publish_waypoints(ordered_waypoints)
        self.viz.publish_path(ordered_waypoints)
        

        # Send navigation request with created path
        success = self.navigate_waypoints(ordered_waypoints,params) # wait for it to complete before proceeding
        if not success: # If it fails in cleaning navigation
            result = CleanRoom.Result()
            result.success = False
            result.message = "Failed during waypoint navigation"
            goal_handle.abort()
            return result


        # -- Result message handling --
        result = CleanRoom.Result()
        result.success = True
        result.message = f"Finished cleaning {room_name} successfully"
        goal_handle.succeed()
        self.viz.clear() # clear old path visualizations

        return result




# ======================================================
# --- Main Function ---
# ======================================================
def main(args=None):
    rclpy.init(args=args)

    node = CleanRoomServer()
    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Clean Room Server...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
