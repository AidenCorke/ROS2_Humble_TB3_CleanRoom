''' 
#######################################################
Room Cleaning Action Client Node
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
from rclpy.action import ActionClient

# Custom Imports
from clean_room_interfaces.action import CleanRoom


# ======================================================
# --- Node ---
# ======================================================
class CleanRoomClient(Node):

    def __init__(self):
        super().__init__('clean_room_client')

        # Create a action client node
        self._client = ActionClient(self,           # Node
                                    CleanRoom,      # Action type
                                    'clean_room')   # Action name


    # -------------------------------------------------------------------
    # --- Send Goal to Server ---
    # -------------------------------------------------------------------
    def send_goal(self,room_name):
        """This sends a single goal message to server."""

        # -- Goal Message --
        goal_msg = CleanRoom.Goal()     # Initialize goal message
        goal_msg.room_name = room_name  # Store goal into message
        
        # -- Sending Goal --
        self._client.wait_for_server()  # Wait for a server

        # Send goal async - identify feedback callback function
        self._send_goal_future = self._client.send_goal_async(goal_msg,
                                                              feedback_callback=self.feedback_callback
                                                              )
        # Calls on client to run goal_response_callback once goal future is resolved
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    # -------------------------------------------------------------------
    # --- Goal response handling ---
    # -------------------------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()   # Define goal based on future value

        # If goal gets rejected send message and stop
        if not goal_handle.accepted:
            self.get_logger().info("Cleaning room goal was rejected :(")
            return
        
        # If goal gets accespted send message and continue
        self.get_logger().info("Cleaning room goal was accepted :)")

        # Call on node to send goal once server is available async
        self._get_result_future = goal_handle.get_result_async()

        # Call on node to send a result msg once future is closed
        self._get_result_future.add_done_callback(self.get_result_callback)

    # -------------------------------------------------------------------
    # --- Feedback response ---
    # -------------------------------------------------------------------
    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback  # Extract feedback from message

        # Publish feedback
        self.get_logger().info(f"Progress: {fb.progress}% | Status: {fb.status}")


    # -------------------------------------------------------------------
    # --- Result response ---
    # -------------------------------------------------------------------
    def get_result_callback(self, future):
        result = future.result().result # Extract result from message

        # Publish result message
        self.get_logger().info(f"Result: success={result.success}, message='{result.message}'")

        # Shutdown node
        #rclpy.shutdown()



# ======================================================
# --- Main Function ---
# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = CleanRoomClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
