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
from geometry_msgs.msg import Point
from drone_interfaces.msg import FrontierList, DroneState
from drone_interfaces.srv import AssignFrontier
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition

_STATUS_IDLE      = 'idle'
_STATUS_EXPLORING = 'exploring'
_STATUS_TAKING_OFF = 'taking_off'


class ExplorationPlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_planner')

        self.declare_parameter('drone_ns', 'd1')
        self.declare_parameter('goal_radius', 0.8)
        self.declare_parameter('hover_alt', 3.0)
        self.declare_parameter('hover_ready_tolerance_m', 0.5)
        self.declare_parameter('tick_period_s', 0.5)
        self.declare_parameter('state_publish_period_s', 1.0)
        self.declare_parameter('goal_republish_period_s', 2.0)

        ns   = self.get_parameter('drone_ns').get_parameter_value().string_value
        self._ns          = ns
        self._goal_radius = self.get_parameter('goal_radius').get_parameter_value().double_value
        self._hover_alt = self.get_parameter('hover_alt').get_parameter_value().double_value
        self._hover_ready_tolerance_m = (
            self.get_parameter('hover_ready_tolerance_m').get_parameter_value().double_value)
        self._tick_period_s = self.get_parameter('tick_period_s').get_parameter_value().double_value
        self._state_publish_period_s = (
            self.get_parameter('state_publish_period_s').get_parameter_value().double_value)
        self._goal_republish_period_s = (
            self.get_parameter('goal_republish_period_s').get_parameter_value().double_value)

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
        self.create_subscription(
            VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position',
            self._local_pos_cb, _best_effort_qos)

        # ── Service server ───────────────────────────────────────────────────
        self._srv = self.create_service(
            AssignFrontier, f'/{ns}/assign_frontier', self._assign_cb)

        # ── State ────────────────────────────────────────────────────────────
        self._pos_enu  = [0.0, 0.0, 0.0]   # current ENU position
        self._goal     = None               # current goal (geometry_msgs/Point)
        self._status   = _STATUS_TAKING_OFF
        self._battery  = 100.0
        self._last_goal_publish_s = None
        self._px4_z_ned = 0.0
        self._px4_xy_valid = False
        self._px4_z_valid = False
        self._ready_logged = False

        self.create_timer(self._state_publish_period_s, self._publish_state)
        self.create_timer(self._tick_period_s, self._tick)

        self.get_logger().info(f'[{ns}] ExplorationPlanner ready')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos_enu = [p.x, p.y, p.z]

    def _battery_cb(self, msg: BatteryStatus):
        if msg.remaining < 0.0:
            return  # -1 = unknown; keep last value
        self._battery = max(0.0, min(100.0, msg.remaining * 100.0))

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._px4_z_ned = msg.z
        self._px4_xy_valid = msg.xy_valid
        self._px4_z_valid = msg.z_valid

    def _assign_cb(self, req: AssignFrontier.Request,
                   res: AssignFrontier.Response) -> AssignFrontier.Response:
        """Coordinator hands us a frontier centroid to explore."""
        if not self._flight_ready():
            res.accepted = False
            res.reason = 'vehicle not ready for exploration'
            return res
        self._goal = req.frontier_centroid
        self._status = _STATUS_EXPLORING
        # Publish state immediately so monitors see 'exploring' before the next
        # planner tick can transition back to 'idle'.
        self._publish_state()
        # Publish immediately so the offboard controller picks the assignment up.
        self._publish_goal()
        self.get_logger().info(
            f'[{self._ns}] Assigned frontier: '
            f'({self._goal.x:.1f}, {self._goal.y:.1f})')
        res.accepted = True
        res.reason   = ''
        return res

    # ── Tick ───────────────────────────────────────────────────────────────
    def _tick(self):
        if self._status != _STATUS_EXPLORING:
            ready = self._flight_ready()
            next_status = _STATUS_IDLE if ready else _STATUS_TAKING_OFF
            if next_status != self._status:
                self._status = next_status
            if ready and not self._ready_logged:
                self.get_logger().info(f'[{self._ns}] Ready for frontier assignments')
                self._ready_logged = True

        if self._status == _STATUS_EXPLORING and self._goal is not None:
            if self._at_goal():
                self.get_logger().info(
                    f'[{self._ns}] Reached frontier — idle')
                self._goal   = None
                self._status = _STATUS_IDLE
                self._last_goal_publish_s = None
                return

            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (self._last_goal_publish_s is None or
                    (now_s - self._last_goal_publish_s) >= self._goal_republish_period_s):
                self._publish_goal()

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

    def _publish_goal(self):
        if self._goal is None:
            return
        self._pub_goal.publish(self._goal)
        self._last_goal_publish_s = self.get_clock().now().nanoseconds * 1e-9

    def _flight_ready(self) -> bool:
        if not (self._px4_xy_valid and self._px4_z_valid):
            return False
        target_z_ned = -self._hover_alt
        return abs(self._px4_z_ned - target_z_ned) <= self._hover_ready_tolerance_m


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
