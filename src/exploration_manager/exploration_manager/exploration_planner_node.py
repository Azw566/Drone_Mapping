"""
exploration_planner_node.py  (one instance per drone)

Receives frontier cluster assignments from the coordinator and navigates
the drone toward the assigned frontier centroid.

Subscribed
  /{ns}/frontiers/list          drone_interfaces/FrontierList   (own frontiers)
  /{ns}/lio_sam/mapping/odometry nav_msgs/Odometry               (current ENU pose)

Published
  /{ns}/goal_pose               geometry_msgs/Point              (ENU map frame)
  /{ns}/drone_state             drone_interfaces/DroneState      (for coordinator)

Service server
  /{ns}/assign_frontier         drone_interfaces/AssignFrontier
    — the coordinator calls this to hand a frontier centroid to this drone.
    — The planner accepts unless it is in a non-interruptible state.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from drone_interfaces.msg import FrontierList, DroneState
from drone_interfaces.srv import AssignFrontier
from px4_msgs.msg import BatteryStatus

_STATUS_IDLE      = 'idle'
_STATUS_EXPLORING = 'exploring'


class ExplorationPlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_planner')

        self.declare_parameter('drone_ns', 'd1')
        self.declare_parameter('goal_radius', 0.8)

        ns   = self.get_parameter('drone_ns').get_parameter_value().string_value
        self._ns          = ns
        self._goal_radius = self.get_parameter('goal_radius').get_parameter_value().double_value

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_goal  = self.create_publisher(Point, f'/{ns}/goal_pose', 10)
        self._pub_state = self.create_publisher(DroneState, f'/{ns}/drone_state', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        # LIO-SAM publishes odometry with BEST_EFFORT reliability — match it.
        _best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry, f'/{ns}/lio_sam/mapping/odometry', self._odom_cb,
            _best_effort_qos)
        self.create_subscription(
            BatteryStatus, f'/{ns}/fmu/out/battery_status',
            self._battery_cb, _best_effort_qos)

        # ── Service server ───────────────────────────────────────────────────
        self._srv = self.create_service(
            AssignFrontier, f'/{ns}/assign_frontier', self._assign_cb)

        # ── State ────────────────────────────────────────────────────────────
        self._pos_enu  = [0.0, 0.0, 0.0]   # current ENU position
        self._goal     = None               # current goal (geometry_msgs/Point)
        self._status   = _STATUS_IDLE
        self._battery  = 100.0

        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(f'[{ns}] ExplorationPlanner ready')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos_enu = [p.x, p.y, p.z]

    def _battery_cb(self, msg: BatteryStatus):
        if msg.remaining < 0.0:
            return  # -1 = unknown; keep last value
        self._battery = max(0.0, min(100.0, msg.remaining * 100.0))

    def _assign_cb(self, req: AssignFrontier.Request,
                   res: AssignFrontier.Response) -> AssignFrontier.Response:
        """Coordinator hands us a frontier centroid to explore."""
        self._goal = req.frontier_centroid
        self._status = _STATUS_EXPLORING
        # Republish immediately so the offboard controller picks it up
        self._pub_goal.publish(self._goal)
        self.get_logger().info(
            f'[{self._ns}] Assigned frontier: '
            f'({self._goal.x:.1f}, {self._goal.y:.1f})')
        res.accepted = True
        res.reason   = ''
        return res

    # ── Tick ───────────────────────────────────────────────────────────────
    def _tick(self):
        if self._status == _STATUS_EXPLORING and self._goal is not None:
            # Re-publish goal at each tick so late-joining controller gets it
            self._pub_goal.publish(self._goal)
            if self._at_goal():
                self.get_logger().info(
                    f'[{self._ns}] Reached frontier — idle')
                self._goal   = None
                self._status = _STATUS_IDLE

    def _at_goal(self) -> bool:
        if self._goal is None:
            return False
        dx = self._pos_enu[0] - self._goal.x
        dy = self._pos_enu[1] - self._goal.y
        return math.sqrt(dx*dx + dy*dy) < self._goal_radius

    # ── DroneState ─────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = DroneState()
        msg.drone_id        = self._ns
        msg.status          = self._status
        msg.battery_percent = float(self._battery)

        msg.current_pose.header.stamp    = self.get_clock().now().to_msg()
        msg.current_pose.header.frame_id = f'{self._ns}/map'
        msg.current_pose.pose.position.x = float(self._pos_enu[0])
        msg.current_pose.pose.position.y = float(self._pos_enu[1])
        msg.current_pose.pose.position.z = float(self._pos_enu[2])

        if self._goal:
            msg.current_goal.x = float(self._goal.x)
            msg.current_goal.y = float(self._goal.y)
            msg.current_goal.z = float(self._goal.z)

        self._pub_state.publish(msg)


def main():
    rclpy.init()
    node = ExplorationPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
