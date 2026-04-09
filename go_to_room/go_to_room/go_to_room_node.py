#!/usr/bin/env python3
"""
go_to_room_node.py — Final Project
===================================================
Navigates TurtleBot3 to named rooms using nav2_simple_commander.
  BasicNavigator - Go to Pose and recovery

Supports TWO interfaces:
  1. Pub/Sub  — publish room name to /go_to_room, result on /go_to_room_result
  2. Action   — rclpy action server on /go_to_room_action  (GoToRoom action)
               gives live feedback (current_room, waypoint progress, distance)
"""

import math
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ── Custom action import (generated from GoToRoom.action) ─────────────────────
# Action definition (place in action/GoToRoom.action):
#   string room_name
#   ---
#   bool success
#   string final_status
#   ---
#   string current_room
#   int32  waypoints_total
#   int32  waypoints_remaining
#   float32 distance_remaining
#   float32 elapsed_seconds
try:
    from cleaner_interfaces.action import GoToRoom
    _ACTION_AVAILABLE = True
except ImportError:
    _ACTION_AVAILABLE = False
    import warnings
    warnings.warn(
        "go_to_room_interfaces not found — Action server disabled. "
        "Only pub/sub mode will work.",
        stacklevel=1,
    )

# ── QoS profiles ──────────────────────────────────────────────────────────────
TRANSIENT = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

RELIABLE = QoSProfile(
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)


# ── Helpers ───────────────────────────────────────────────────────────────────
def _pose(nav, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


# ── Map data ──────────────────────────────────────────────────────────────────
GOALS = {
    'hallway':     ( 1.0,   0.50),
    'living_room': (-2.5,   3.50),
    'library':     (-6.5,   3.00),
    'bedroom':     (-6.5,  -0.50),
    'kitchen':     ( 4.5,   3.00),
    'dining_room': ( 5.5,  -1.45),
    'pantry':      ( 1.0,   3.00),
}

ROOM_BOUNDS = {
    'hallway':     {'x': (-5.10,  7.40), 'y': (-0.20,  0.90)},
    'living_room': {'x': (-5.10,  0.00), 'y': ( 0.90,  5.30)},
    'library':     {'x': (-7.40, -5.10), 'y': ( 0.90,  5.30)},
    'bedroom':     {'x': (-7.40, -5.10), 'y': (-3.75,  0.90)},
    'kitchen':     {'x': ( 2.25,  7.40), 'y': ( 0.90,  5.30)},
    'dining_room': {'x': ( 4.90,  7.40), 'y': (-5.10, -0.20)},
    'pantry':      {'x': ( 0.00,  2.15), 'y': ( 0.90,  5.30)},
}

VALID_ROOMS = set(ROOM_BOUNDS.keys())

ROUTES = {
    ('hallway', 'living_room'):  [(-2.0, 1.0), (-2.5, 2.5)],
    ('hallway', 'library'):      [(-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0)],
    ('hallway', 'bedroom'):      [(-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0), (-6.25,  1.25)],
    ('hallway', 'kitchen'):      [( 3.0, 0.5), ( 3.2, 2.7)],
    ('hallway', 'dining_room'):  [( 6.0, 0.5)],
    ('hallway', 'pantry'):       [( 3.0, 0.5), ( 2.4, 4.5)],

    ('living_room', 'hallway'):      [(-2.5, 1.2), (-2.0, 1.0)],
    ('living_room', 'library'):      [(-2.5, 1.2), (-5.0, 1.2), (-5.5, 2.5)],
    ('living_room', 'bedroom'):      [(-2.5, 1.2), (-5.0, 1.2), (-6.25, 1.25)],
    ('living_room', 'kitchen'):      [(-2.5, 1.2), (-1.0, 0.5), ( 3.0, 0.5), ( 3.2, 2.7)],
    ('living_room', 'dining_room'):  [(-2.5, 1.2), (-1.0, 0.5), ( 3.0, 0.5), ( 6.0, 0.5)],
    ('living_room', 'pantry'):       [(-2.5, 1.2), (-1.0, 0.5), ( 3.0, 0.5), ( 2.4, 4.5)],

    ('library', 'hallway'):      [(-5.5, 2.5), (-5.0, 1.2), (-2.0, 1.0)],
    ('library', 'living_room'):  [(-5.5, 2.5), (-5.0, 1.2), (-2.5, 1.2)],
    ('library', 'bedroom'):      [(-6.25, 1.1)],
    ('library', 'kitchen'):      [(-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 3.0, 0.5), ( 3.2, 2.7)],
    ('library', 'dining_room'):  [(-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 3.0, 0.5), ( 6.0, 0.5)],
    ('library', 'pantry'):       [(-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 3.0, 0.5), ( 2.4, 4.5)],

    ('bedroom', 'hallway'):      [(-6.25, 1.25), (-5.0, 1.2), (-2.0, 1.0)],
    ('bedroom', 'living_room'):  [(-6.25, 1.25), (-5.0, 1.2), (-2.5, 1.2)],
    ('bedroom', 'library'):      [(-6.25, 1.1)],
    ('bedroom', 'kitchen'):      [(-6.25, 1.25), (-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 3.0, 0.5), ( 3.2, 2.7)],
    ('bedroom', 'dining_room'):  [(-6.25, 1.25), (-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 6.0, 0.5)],
    ('bedroom', 'pantry'):       [(-6.25, 1.25), (-5.0, 3.0), (-4.0, 2.0), (-2.0, 1.0), ( 3.0, 0.5), ( 2.4, 4.5)],

    ('kitchen', 'hallway'):      [( 3.2, 2.7), ( 3.0, 0.5)],
    ('kitchen', 'living_room'):  [( 3.2, 2.7), ( 3.0, 0.5), (-1.0, 0.5), (-2.5, 2.0)],
    ('kitchen', 'library'):      [( 3.2, 2.7), ( 3.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0)],
    ('kitchen', 'bedroom'):      [( 3.2, 2.7), ( 3.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0), (-6.25, 1.25)],
    ('kitchen', 'dining_room'):  [( 4.8, 1.8), ( 6.0, 0.5)],
    ('kitchen', 'pantry'):       [( 3.2, 2.7), ( 2.4, 4.5)],

    ('dining_room', 'hallway'):      [( 6.0, 0.5)],
    ('dining_room', 'living_room'):  [( 6.0, 0.5), ( 3.0, 0.5), (-1.0, 0.5), (-2.5, 2.0)],
    ('dining_room', 'library'):      [( 6.0, 0.5), ( 3.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0)],
    ('dining_room', 'bedroom'):      [( 6.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0), (-6.25,  1.25)],
    ('dining_room', 'kitchen'):      [( 6.0, 0.5), ( 4.8, 1.8)],
    ('dining_room', 'pantry'):       [( 6.0, 0.5), ( 3.2, 2.7), ( 2.4, 4.5)],

    ('pantry', 'hallway'):      [( 2.4, 4.5), ( 3.0, 0.5)],
    ('pantry', 'living_room'):  [( 2.4, 4.5), ( 3.0, 0.5), (-2.0, 1.0),  (-2.5, 2.5)],
    ('pantry', 'library'):      [( 2.4, 4.5), ( 3.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0)],
    ('pantry', 'bedroom'):      [( 2.4, 4.5), ( 3.0, 0.5), (-2.0, 1.0), (-4.0, 2.0), (-5.0, 3.0), (-6.25,  1.25)],
    ('pantry', 'kitchen'):      [( 2.4, 4.5), ( 3.2, 2.7)],
    ('pantry', 'dining_room'):  [( 2.4, 4.5), ( 3.2, 2.7), ( 6.0, 0.5)],
}


# ── Node ──────────────────────────────────────────────────────────────────────
class GoToRoomNode(Node):

    def __init__(self):
        super().__init__('go_to_room_node')

        # Parameters
        self.declare_parameter('initial_x',   -2.00)
        self.declare_parameter('initial_y',    0.50)
        self.declare_parameter('initial_yaw',  0.00)
        self.declare_parameter('max_retries',  3)

        self._nav = None

        # AMCL pose
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._amcl = None
        self._amcl_lock = threading.Lock()
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            lambda m: self._set_amcl(m.pose.pose), 10)

        # ── Pub/Sub interface ──────────────────────────────────────────────
        self._goal_q = queue.Queue()
        self.create_subscription(
            String, '/go_to_room',
            lambda m: self._goal_q.put(('pubsub', m.data.strip(), None)),
            RELIABLE)

        self._result_pub = self.create_publisher(String, '/go_to_room_result', RELIABLE)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Action interface ───────────────────────────────────────────────
        if _ACTION_AVAILABLE:
            self._action_server = ActionServer(
                self,
                GoToRoom,
                '/go_to_room_action',
                execute_callback=self._action_execute_cb,
                goal_callback=self._action_goal_cb,
                cancel_callback=self._action_cancel_cb,
            )
            self.get_logger().info('Action server /go_to_room_action READY')
        else:
            self._action_server = None
            self.get_logger().warn('Action server disabled (interfaces not built)')

        # Nav loop runs in background thread
        threading.Thread(
            target=self._nav_loop, daemon=True, name='nav_loop'
        ).start()

    # ── AMCL helpers ──────────────────────────────────────────────────────
    def _set_amcl(self, pose):
        with self._amcl_lock:
            self._amcl = pose

    def _get_room(self):
        with self._amcl_lock:
            p = self._amcl
        if p is None:
            return None
        for name, b in ROOM_BOUNDS.items():
            if b['x'][0] <= p.position.x <= b['x'][1] and \
               b['y'][0] <= p.position.y <= b['y'][1]:
                return name
        return None

    def _set_initial_pose(self):
        ix  = self.get_parameter('initial_x').value
        iy  = self.get_parameter('initial_y').value
        iyw = self.get_parameter('initial_yaw').value
        pose = _pose(self._nav, ix, iy, iyw)
        self._nav.setInitialPose(pose)
        self.get_logger().info(f'Initial pose set: ({ix:.2f}, {iy:.2f})')

    # ── Single-pose navigator (core primitive) ───────────────────────────────
    def _go_to_pose(self, x, y, feedback_cb=None, cancel_check=None,
                    wp_index=None, wp_total=None):
        """
        Navigate to a single (x, y) coordinate using goToPose().
        Each waypoint is independent — failure here does not affect others.

        Returns True on success, False on stall/failure/cancel.
        """
        pose = _pose(self._nav, x, y)
        label = f'({x:.2f}, {y:.2f})'
        if wp_index is not None:
            label = f'WP {wp_index}/{wp_total} {label}'
        self.get_logger().info(f'  goToPose → {label}')
        self._nav.goToPose(pose)

        start       = time.time()
        elapsed     = 0.0
        last_dist   = None
        stall_since = None

        while not self._nav.isTaskComplete():
            if cancel_check and cancel_check():
                self.get_logger().info(f'  Cancel — aborting {label}')
                self._nav.cancelTask()
                return False

            fb      = self._nav.getFeedback()
            elapsed = time.time() - start

            if fb:
                dist = fb.distance_remaining

                self.get_logger().info(
                    f'  {label} | dist={dist:.2f}m | t={elapsed:.1f}s',
                    throttle_duration_sec=2.0)

                if feedback_cb:
                    total     = wp_total or 1
                    remaining = total - (wp_index or 0) + 1
                    feedback_cb(total, remaining, dist, elapsed)

                # Close enough override (intermediate waypoints only)
                if dist < 0.15 and elapsed > 20.0:
                    self.get_logger().info(f'  [Override] Close enough at {label}')
                    self._nav.cancelTask()
                    return True

                # Stall detection: no progress > 0.05m in 8s
                if last_dist is None or (last_dist - dist) > 0.05:
                    last_dist   = dist
                    stall_since = time.time()
                elif stall_since and time.time() - stall_since > 8.0:
                    self.get_logger().warn(
                        f'  [Stall] No progress at {label} — aborting')
                    self._nav.cancelTask()
                    return False

            time.sleep(0.1)

        result = self._nav.getResult()
        ok     = (result == TaskResult.SUCCEEDED)
        self.get_logger().info(
            f'  {label} → {"OK" if ok else "FAIL"} | t={elapsed:.1f}s')
        return ok

    # ── High-level navigation logic ────────────────────────────────────────
    def _navigate(self, dst, feedback_cb=None, cancel_check=None):
        """
        Navigate to `dst` one waypoint at a time using goToPose().

        Each corridor waypoint is driven independently so a failure at any
        single point triggers a per-waypoint backup+retry rather than
        aborting the entire route.  The final GOALS[dst] coordinate is
        always the last pose sent.
        """
        cur     = self._get_room()
        retries = int(self.get_parameter('max_retries').value)

        # Determine corridor waypoints
        if cur and (cur, dst) in ROUTES:
            corridor = list(ROUTES[(cur, dst)])
            self.get_logger().info(
                f'  Direct route: {cur} → {dst} ({len(corridor)} corridor wps)')
        else:
            self.get_logger().info(
                '  Position unknown / no direct route — recovering to hallway')
            recovered = False
            for entry in [(1.0, 0.5), (-2.0, 0.5), (0.0, 0.5)]:
                if self._go_to_pose(*entry, feedback_cb=feedback_cb,
                                    cancel_check=cancel_check):
                    recovered = True
                    break
                self._backup()

            if not recovered:
                self.get_logger().error('  Recovery failed — cannot navigate')
                return False

            cur = self._get_room()
            corridor = list(ROUTES.get((cur, dst),
                                       ROUTES.get(('hallway', dst), [])))
            if not corridor and dst != cur:
                self.get_logger().error(f'No route to {dst} after recovery')
                return False
            self.get_logger().info(
                f'  Post-recovery route to {dst} ({len(corridor)} wps)')

        # Full waypoint list = corridor points + final room goal
        all_wps = corridor + [GOALS[dst]]
        total   = len(all_wps)
        self.get_logger().info(
            f'  Driving {total} waypoints individually (corridor + goal)')

        # Drive each waypoint independently
        for i, (x, y) in enumerate(all_wps, start=1):
            is_final = (i == total)
            wp_retries = retries if is_final else 2  # more retries on final goal

            for attempt in range(1, wp_retries + 2):
                if attempt > 1:
                    self.get_logger().info(
                        f'  WP {i}/{total} retry {attempt-1}/{wp_retries} — backing up')
                    self._backup()

                ok = self._go_to_pose(
                    x, y,
                    feedback_cb=feedback_cb,
                    cancel_check=cancel_check,
                    wp_index=i,
                    wp_total=total)

                if cancel_check and cancel_check():
                    return False

                if ok:
                    break  # this waypoint done, move to next
            else:
                # Exhausted retries on this waypoint
                self.get_logger().error(
                    f'  WP {i}/{total} ({x:.2f},{y:.2f}) failed after ' +
                    f'{wp_retries} retries — aborting route')
                return False

        self.get_logger().info(f'  All {total} waypoints reached — arrived at {dst}')
        return True

    def _stop_robot(self):
        stop_msg = Twist()
        for _ in range(10):
            self._cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)

    def _backup(self, duration: float = 2.0, speed: float = -0.1):
        """Reverse for `duration` seconds to escape a stuck position."""
        self.get_logger().info(f'  [Backup] Reversing for {duration}s ...')
        msg = Twist()
        msg.linear.x = speed
        deadline = time.time() + duration
        while time.time() < deadline:
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        self._stop_robot()
        self.get_logger().info('  [Backup] Done')

    # ── Action server callbacks ────────────────────────────────────────────
    def _action_goal_cb(self, goal_request):
        key = goal_request.room_name.strip().lower().replace(' ', '_')
        if key not in VALID_ROOMS:
            self.get_logger().warn(
                f'[Action] Rejecting invalid room: "{key}"')
            return GoalResponse.REJECT
        self.get_logger().info(f'[Action] Accepting goal: {key}')
        return GoalResponse.ACCEPT

    def _action_cancel_cb(self, goal_handle):
        self.get_logger().info('[Action] Cancel requested')
        return CancelResponse.ACCEPT

    async def _action_execute_cb(self, goal_handle):
        """Runs in the executor's thread pool — safe to block."""
        key = goal_handle.request.room_name.strip().lower().replace(' ', '_')
        self.get_logger().info(f'[Action] Executing goal: {key}')

        feedback_msg = GoToRoom.Feedback()

        def feedback_cb(total, remaining, dist, elapsed):
            feedback_msg.current_room         = self._get_room() or 'unknown'
            feedback_msg.waypoints_total       = total
            feedback_msg.waypoints_remaining   = remaining
            feedback_msg.distance_remaining    = dist
            feedback_msg.elapsed_seconds       = elapsed
            goal_handle.publish_feedback(feedback_msg)

        def cancel_check():
            return goal_handle.is_cancel_requested

        # Block here until navigation finishes (or is cancelled)
        success = self._navigate(key, feedback_cb=feedback_cb,
                                 cancel_check=cancel_check)

        result = GoToRoom.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success      = False
            result.final_status = 'canceled'
            self.get_logger().info(f'[Action] Goal {key} CANCELED')
        elif success:
            self._stop_robot()
            goal_handle.succeed()
            result.success      = True
            result.final_status = 'succeeded'
            self.get_logger().info(f'[Action] Goal {key} SUCCEEDED')
        else:
            goal_handle.abort()
            result.success      = False
            result.final_status = 'failed'
            self.get_logger().info(f'[Action] Goal {key} FAILED')

        return result

    # ── Pub/Sub nav loop (background thread) ──────────────────────────────
    def _nav_loop(self):
        self.get_logger().info('Nav thread: initializing (waiting 10s)...')
        time.sleep(10.0)

        self._nav = BasicNavigator()

        self.get_logger().info('Nav thread: waiting for Nav2...')
        try:
            self._nav.waitUntilNav2Active()
        except Exception as e:
            self.get_logger().error(f'Nav2 failed to activate: {e}')
            return

        self._set_initial_pose()

        self.get_logger().info('Nav thread: waiting for AMCL to converge...')
        for i in range(30):
            if not rclpy.ok():
                return
            time.sleep(1.0)
            if self._get_room() is not None:
                self.get_logger().info(
                    f'AMCL converged! Robot in: {self._get_room()}')
                break
            if i % 5 == 0:
                self.get_logger().info('  Still waiting for AMCL pose...')

        cur_room = self._get_room()
        self.get_logger().info(
            f'SYSTEM READY — location: {cur_room if cur_room else "UNKNOWN"}')

        # Main command loop (pub/sub only)
        while rclpy.ok():
            try:
                # Items in queue: ('pubsub', room_name, None)
                source, room_name, _ = self._goal_q.get(timeout=1.0)
            except queue.Empty:
                continue

            key = room_name.strip().lower().replace(' ', '_')

            if key not in VALID_ROOMS:
                self.get_logger().error(f'[PubSub] Invalid room: "{key}"')
                self._result_pub.publish(String(data='failed'))
                continue

            self.get_logger().info(f'[PubSub] 🚀 NAVIGATING TO: {key.upper()}')
            success = self._navigate(key)

            if success:
                self._stop_robot()

            status = 'succeeded' if success else 'failed'
            self._result_pub.publish(String(data=status))
            self.get_logger().info(
                f'[PubSub] 🏁 {key.upper()} → {status.upper()}')


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    node = GoToRoomNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
