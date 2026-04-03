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
# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action._follow_waypoints import FollowWaypoints_FeedbackMessage
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


# Standard Imports
import asyncio
import math

# Custom Imports
from clean_room_interfaces.action import CleanRoom
from clean_room_server.planner.cleaning_manager import CleaningManager
from clean_room_server.utils.geometry_utils import point_in_polygon, yaw_to_quaternion
#from clean_room_operations.clean_room_operations.utils.geometry_utils import point_in_polygon
#from clean_room_operations.clean_room_operations.cleaning_manager import CleaningManager

# ======================================================
# --- Node ---
# ======================================================
class CleanRoomServer(Node):

    def __init__(self):
        super().__init__('clean_room_server')

        # Declare parameters
        self.declare_parameter('rooms_config', '/dev/null')
        self.declare_parameter('map_yaml', '/dev/null')

        # Extract parameters
        rooms_path = self.get_parameter('rooms_config').value
        map_yaml_path = self.get_parameter('map_yaml').value

        # Warning if parameters aren't loaded correct
        if rooms_path == "/dev/null":
            self.get_logger().error("rooms_path parameter not set")
        if map_yaml_path == "/dev/null":
            self.get_logger().error("maps_path parameter not set")

        # create action server node
        self._action_server = ActionServer(self,                                    # Node
                                           CleanRoom,                               # Action type
                                           'clean_room',                            # Action name
                                           execute_callback=self.execute_callback,  # Action execution
                                           cancel_callback=self.cancel_callback     # Action canceling
                                           )

        # Subscribe to robot's current pose
        self.current_pose = None
        self.pose_sub = self.create_subscription(PoseStamped,
                                                 '/amcl_pose',
                                                 self.pose_callback,
                                                 10
                                                 )

        # Initialize cleaning logic class
        self.manager = CleaningManager(rooms_path, map_yaml_path)

    # -------------------------------------------------------------------
    # --- Collect pose ---
    # -------------------------------------------------------------------
    def pose_callback(self, msg):
        """ This function collects the robots current pose."""
        self.current_pose = msg.pose


    # -------------------------------------------------------------------
    # --- Cancel response recieving ---
    # -------------------------------------------------------------------
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    # -------------------------------------------------------------------
    # --- Detect current room ---
    # -------------------------------------------------------------------
    def detect_current_room(self):
        """ This function detects and returns the room the robot is currently in using current pose and defined room boundaries."""
        # If no pose is present break out and return None
        if self.current_pose is None:
            return None

        # Convert robot pose to pixel coordinates
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        px, py = self.manager.map_utils.world_to_pixel(x, y)

        # Check each room polygon
        for room_name, params in self.manager.room_config.items():
            if point_in_polygon((px, py, params['corners'])):
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

        # Create Nav2 BasicNavigator instance
        navigator = BasicNavigator()

        # Set the initial (current) pose
        navigator.setInitialPose(self.current_pose)

        # Wait for navigation to fully activate, since autostarting nav2
        navigator.waitUntilNav2Active()

        # Load map file
        #navigator.changeMap(self.get_parameter('map_yaml').value)

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

        nav_start = self.navigator.get_clock().now()    # Time of nav start

        nav_to_point = navigator.goToPose(goal)

        while not navigator.isTaskComplete(task=nav_to_point):
            
            i += 1
            feedback = navigator.getFeedback(task=nav_to_point)
            if feedback and i % 5 == 0:
                print( 'Estimated time of arrival: '
                    + '{:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                print(
                    'Distance remaining: '
                    + '{:.2f}'.format(feedback.distance_remaining)
                    + ' meters.'
                )            
                
                # Cancel navigation is it takes 10 minutes
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()
            

        # -- Results --    
        result = navigator.getResult()  # Extract result
        # Task succeeded
        if result == TaskResult.SUCCEEDED:                                  
            self.get_logger().info('Reached entry point.')

        # Task canceled
        elif result == TaskResult.CANCELED:                                 
            self.get_logger().info('Navigation to entry point canceled!')

        # Task failed
        elif result == TaskResult.FAILED:                                   
            (error_code, error_msg) = navigator.getTaskError()
            self.get_logger().warn(f'Entry point navigation failed!{error_code}:{error_msg}')

        # Weird result
        else:                                                              
            self.get_logger().warn('Entry point navigation has an invalid return status!')        
        
        navigator.lifecycleShutdown()   # Shutdown navigator instance

        
        return True


    # -------------------------------------------------------------------
    # --- Navigate through cleaning path ---
    # -------------------------------------------------------------------
    def navigate_waypoints(self, ordered_waypoints):
        """This function sends a list of (x, y) waypoints to Nav2 FollowWaypoints."""

        # -- Initialize navigator instance --
        navigator = BasicNavigator()                    # Initialize navigator class instance
        navigator.setInitialPose(self.current_pose)     # Set the initial (current) pose
        navigator.waitUntilNav2Active()                 # Wait for nav2 to fully activate, since autostarting nav2

        # -- Convert poses into readable goals for nav2 --        
        goal_poses = []
        total = len(ordered_waypoints)

        # Loop through all waypoints adding to pose
        for i, (x, y) in enumerate(ordered_waypoints):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            # Compute orientation toward next waypoint
            if i < len(ordered_waypoints) - 1:
                nx, ny = ordered_waypoints[i+1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0  # final waypoint

            pose.pose.orientation = yaw_to_quaternion(yaw)
            goal_poses.append(pose)

        nav_start = navigator.get_clock().now()
        nav_to_wp = navigator.followWaypoints(goal_poses)

        while not navigator.isTaskComplete(task=nav_to_wp):
            
            i = i + 1
            feedback = navigator.getFeedback(task=nav_to_wp)
            if feedback and i % 5 == 0:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(goal_poses))
                )
                now = navigator.get_clock().now()           
                    
                # Cancel navigation is it takes 10 minutes
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()
            
        # -- Results --    
        result = navigator.getResult()  # Extract result

        # Task succeeded
        if result == TaskResult.SUCCEEDED:                                  
            self.get_logger().info('Cleaning completed successfully!')

        # Task canceled
        elif result == TaskResult.CANCELED:                                 
            self.get_logger().info('Cleaning was cancelled!')
        
        # Task failed
        elif result == TaskResult.FAILED:                                   
            (error_code, error_msg) = navigator.getTaskError()
            self.get_logger().warn(f'Cleaning failed!{error_code}:{error_msg}')

        # Weird result
        else:
            self.get_logger().warn('Goal has an invalid return status!')        
        
        navigator.lifecycleShutdown()   # Shutdown navigator instance once done

        return True


    # -------------------------------------------------------------------
    # --- Execute action ---
    # -------------------------------------------------------------------
    async def execute_callback(self, goal_handle):
        
        room_name = goal_handle.request.room_name  # Extract room name from goal
        self.get_logger().info(f"Cleaning requested for room: {room_name}") # Publish request recieved

        # Room detection and navigation to entry point if needed
        current_room = self.detect_current_room()
        if current_room != room_name:
            self.get_logger().info(f"Robot navigating to {room_name} now...")
            success = await self.navigate_to_entry_point(room_name)   # Navigate to designated entry point
            
            # If it fails to reach the entry point
            if not success:
                result = CleanRoom.Result()
                result.success = False
                result.message = f"Failed to reach entry point"
                goal_handle.abort()
                return result
            
        self.get_logger().info(f"Currently in {room_name}, beginning cleaning...")


        # Create path from cleaning operations
        ordered_waypoints = self.manager.cleaning_path(room_name,
                                                       self.current_pose,
                                                       )
        self.get_logger().info("Starting cleaning path generation...")

        # Send navigation request with created path
        success = await self.navigate_waypoints(ordered_waypoints) # wait for it to complete before proceeding
        if not success: # If it fails in cleaning navigation
            result = CleanRoom.Result()
            result.success = False
            result.message = "Failed during waypoint navigation"
            goal_handle.abort()
            return result


        # Result message handling
        result = CleanRoom.Result()
        result.success = True
        result.message = f"Finished cleaning {room_name} successfully"
        goal_handle.succeed()
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
