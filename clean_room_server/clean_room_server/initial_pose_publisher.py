import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Publish once after a short delay
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]


        self.pub.publish(msg)
        self.get_logger().info("Initial pose published.")
        self.timer.cancel()

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
