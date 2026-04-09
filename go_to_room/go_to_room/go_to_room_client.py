#!/usr/bin/env python3
"""
go_to_room_client.py — sends a room goal and waits for result.

MODES
-----
  --mode pub     (default) Publish to /go_to_room, wait on /go_to_room_result.
                 No feedback; simple and dependency-free.

  --mode action  Use the /go_to_room_action ROS2 action server.
                 Prints live feedback (room, waypoints, distance, elapsed).
                 Requires go_to_room_interfaces to be built.

USAGE
-----
  ros2 run go_to_room go_to_room_client <room> [--mode pub|action]

  Rooms: hallway  living_room  library  bedroom
         kitchen  pantry  dining_room
"""

import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)
from std_msgs.msg import String

RELIABLE = QoSProfile(
    durability  = QoSDurabilityPolicy.VOLATILE,
    reliability = QoSReliabilityPolicy.RELIABLE,
    history     = QoSHistoryPolicy.KEEP_LAST,
    depth       = 10)

VALID_ROOMS = {
    'hallway', 'living_room', 'library', 'bedroom',
    'kitchen', 'pantry', 'dining_room',
}


# ══════════════════════════════════════════════════════════════════════════════
# Pub/Sub client (original behaviour)
# ══════════════════════════════════════════════════════════════════════════════
class GoToRoomClientPubSub(Node):
    """Publish a room name, wait for a result string on /go_to_room_result."""

    def __init__(self):
        super().__init__('go_to_room_client')
        self._result = None
        self._pub = self.create_publisher(String, '/go_to_room', RELIABLE)
        self.create_subscription(
            String, '/go_to_room_result',
            lambda m: setattr(self, '_result', m.data),
            RELIABLE)

    def send_goal(self, room: str, timeout: float = 180.0) -> bool:
        self.get_logger().info(f"[PubSub] Sending goal: '{room}'")

        # Wait for the node subscriber to connect (up to 5 s)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pub.get_subscription_count() > 0:
                break
        else:
            self.get_logger().warn(
                'No subscriber on /go_to_room — sending anyway')

        msg = String()
        msg.data = room
        self._pub.publish(msg)
        time.sleep(0.1)

        self.get_logger().info('[PubSub] Goal sent — waiting for result …')

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._result is not None:
                ok = self._result == 'succeeded'
                self.get_logger().info(
                    f"[PubSub] Result: {'SUCCEEDED ✓' if ok else 'FAILED ✗'}")
                return ok

        self.get_logger().error(f'[PubSub] Timeout — no result in {timeout:.0f}s')
        return False


# ══════════════════════════════════════════════════════════════════════════════
# Action client
# ══════════════════════════════════════════════════════════════════════════════
def _run_action_client(room: str, timeout: float = 180.0) -> bool:
    """
    Send a goal to /go_to_room_action and stream feedback until done.
    Returns True on success, False on failure/cancel/timeout.
    """
    try:
        from rclpy.action import ActionClient
        from cleaner_interfaces.action import GoToRoom
    except ImportError as e:
        print(f'\n[Action] ERROR: {e}')
        print('Build go_to_room_interfaces first, or use --mode pub.\n')
        return False

    class _ActionClientNode(Node):
        def __init__(self):
            super().__init__('go_to_room_action_client')
            self._client  = ActionClient(self, GoToRoom, '/go_to_room_action')
            self._done    = False
            self._success = False

        def send_goal(self, room_name):
            self.get_logger().info('[Action] Waiting for server …')
            if not self._client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(
                    '[Action] Server not available after 10 s')
                return False

            goal_msg = GoToRoom.Goal()
            goal_msg.room_name = room_name

            self.get_logger().info(f"[Action] Sending goal: '{room_name}'")
            self._send_goal_future = self._client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_cb)
            self._send_goal_future.add_done_callback(self._goal_response_cb)

        def _goal_response_cb(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('[Action] Goal REJECTED by server')
                self._done    = True
                self._success = False
                return
            self.get_logger().info('[Action] Goal ACCEPTED — navigating …')
            self._result_future = goal_handle.get_result_async()
            self._result_future.add_done_callback(self._result_cb)

        def _feedback_cb(self, feedback_msg):
            fb = feedback_msg.feedback
            print(
                f'\r[Action] Room: {fb.current_room:<12} | '
                f'WP: {fb.waypoints_total - fb.waypoints_remaining}/'
                f'{fb.waypoints_total} | '
                f'Dist: {fb.distance_remaining:5.2f}m | '
                f'Elapsed: {fb.elapsed_seconds:6.1f}s',
                end='', flush=True)

        def _result_cb(self, future):
            print()  # newline after feedback line
            result = future.result().result
            self._success = result.success
            self.get_logger().info(
                f'[Action] Result: {result.final_status.upper()} '
                f'(success={result.success})')
            self._done = True

    rclpy.init()
    node = _ActionClientNode()
    node.send_goal(room)

    deadline = time.time() + timeout
    while not node._done and rclpy.ok():
        if time.time() > deadline:
            node.get_logger().error(
                f'[Action] Timeout — no result in {timeout:.0f}s')
            node._success = False
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    success = node._success
    node.destroy_node()
    rclpy.shutdown()
    return success


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='Send a go_to_room goal via pub/sub or action.')
    parser.add_argument(
        'room',
        help='Target room name. '
             'Valid: hallway living_room library bedroom kitchen pantry dining_room')
    parser.add_argument(
        '--mode', choices=['pub', 'action'], default='pub',
        help='Interface to use: pub (default) or action')
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='Seconds to wait for a result (default: 180)')

    # Support bare positional + optional flag without requiring argparse to see
    # the room as a sub-command.
    args = parser.parse_args()

    room = args.room.strip().lower().replace(' ', '_')
    if room not in VALID_ROOMS:
        parser.error(
            f"Unknown room '{room}'. "
            f"Valid rooms: {', '.join(sorted(VALID_ROOMS))}")

    if args.mode == 'action':
        ok = _run_action_client(room, timeout=args.timeout)
        sys.exit(0 if ok else 1)

    # ── Pub/Sub mode ──────────────────────────────────────────────────────
    rclpy.init()
    client = GoToRoomClientPubSub()
    ok = client.send_goal(room, timeout=args.timeout)
    client.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
