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

        # Create Nav2 action clients
        self.follow_wp_client = ActionClient(self,
                                             FollowWaypoints,
                                             'follow_waypoints'
                                             )
        self.nav_client = ActionClient(self,
                                       NavigateToPose,
                                       'navigate_to_pose'
                                       )

        # Subscribe to feedback topic from nav2
        self.current_wp_index = 0
        self.wp_feedback_sub = self.create_subscription(
            FollowWaypoints_FeedbackMessage,
            '/follow_waypoints/_action/feedback',
            self.wp_feedback_callback,
            10
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
    async def navigate_to_entry_point(self, room_name):
        """This function sends a nav2 request to navigate to the entry point of the given room."""
        # Extract entry point from param files
        params = self.manager.room_config[room_name]
        entry_px, entry_py = params["entry_point"]
        xw, yw = self.manager.map_utils.pixel_to_world(entry_px, entry_py)

        # Initialize goal for Nav2
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xw)
        pose.pose.position.y = float(yw)
        pose.pose.orientation.w = 1.0       # Face forward
        goal.pose = pose

        self.get_logger().info(f"Navigating to entry point of {room_name}...")

        # Wait to send goal
        self.nav_client.wait_for_server()  # Wait for server to be free
        future = self.nav_client.send_goal_async(goal)  # Send async goal request
        goal_handle = await future

        # Rejection handling
        if not goal_handle.accepted:
            self.get_logger().error("Entry point navigation rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        await result_future

        self.get_logger().info("Reached entry point")
        return True


    # -------------------------------------------------------------------
    # --- Navigate through cleaning path ---
    # -------------------------------------------------------------------
    async def navigate_waypoints(self, ordered_waypoints):
        """This function sends a list of (x, y) waypoints to Nav2 FollowWaypoints."""
        poses = []
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
            poses.append(pose)

        # Initialize goal
        goal = FollowWaypoints.Goal()
        goal.poses = poses

        # Wait for a server to be available
        self.get_logger().info(f"Sending {len(poses)} waypoints to FollowWaypoints")
        self.follow_wp_client.wait_for_server(10)
        self.get_logger().info("FollowWaypoints server is available")

        future = self.follow_wp_client.send_goal_async(goal)
        goal_handle = await future  # Wait for goal
        
        # Rejection handling
        if not goal_handle.accepted:
            self.get_logger().error("Waypoint navigation goal rejected")
            return False

        # Wait for result
        self.get_logger().info(f"Goal accepted: {goal_handle.accepted}")
        result_future = goal_handle.get_result_async()

        # Update feedback message
        feedback_msg = CleanRoom.Feedback() # Create variable for storing feedback in form of action
        feedback_msg.status = "Cleaning in progress..."


        # ------------- make into timer fct?
        while not result_future.done(): # Loops while the result isnt finished
            progress = (self.current_wp_index / total) * 100
            feedback_msg.progress = float(progress)
            goal_handle.publish_feedback(feedback_msg) # <-------------- error here
            await asyncio.sleep(0.5)

        result = await result_future
        self.get_logger().info(f"FollowWaypoints result: {result.result}")
        self.get_logger().info("Waypoint navigation completed")
        return True


    # -------------------------------------------------------------------
    # --- Execute action ---
    # -------------------------------------------------------------------
    def wp_feedback_callback(self, msg):
        self.current_wp_index = msg.feedback.current_waypoint


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
