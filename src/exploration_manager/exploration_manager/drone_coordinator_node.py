"""
drone_coordinator_node.py  (singleton — one instance for all drones)

Collects FrontierList from all drones, selects the best frontier per idle
drone, and calls each drone's AssignFrontier service.

Selection criteria
  score = cluster_size / max(1, distance_from_drone_to_centroid)

Safety
  A frontier centroid is not assigned to drone A if another drone B is
  already heading toward a centroid within safety_radius metres.

Subscribed
  /{ns}/frontiers/list    drone_interfaces/FrontierList  (per drone)
  /{ns}/drone_state       drone_interfaces/DroneState    (per drone)

Service clients
  /{ns}/assign_frontier   drone_interfaces/AssignFrontier
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty
from drone_interfaces.msg import FrontierList, DroneState
from drone_interfaces.srv import AssignFrontier

_STATUS_IDLE = 'idle'


class DroneCoordinatorNode(Node):
    def __init__(self):
        super().__init__('drone_coordinator')

        self.declare_parameter('drone_namespaces', ['d1', 'd2'])
        self.declare_parameter('safety_radius', 3.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('no_frontier_timeout', 30.0)

        namespaces = (self.get_parameter('drone_namespaces')
                      .get_parameter_value().string_array_value)
        self._safety_radius = (self.get_parameter('safety_radius')
                                .get_parameter_value().double_value)
        self._low_bat_threshold = (self.get_parameter('low_battery_threshold')
                                   .get_parameter_value().double_value)
        self._no_frontier_timeout = (self.get_parameter('no_frontier_timeout')
                                     .get_parameter_value().double_value)
        self._namespaces = list(namespaces)

        # Per-drone state
        self._states:    dict[str, DroneState]    = {}
        self._frontiers: dict[str, FrontierList]  = {}
        self._assign_clients: dict[str, object] = {}
        self._land_pubs:          dict[str, object] = {}
        self._last_frontier_time: dict[str, float]  = {}
        self._mission_complete = False

        cbg = ReentrantCallbackGroup()

        for ns in self._namespaces:
            self.create_subscription(
                DroneState, f'/{ns}/drone_state',
                lambda msg, n=ns: self._state_cb(n, msg), 10)
            self.create_subscription(
                FrontierList, f'/{ns}/frontiers/list',
                lambda msg, n=ns: self._frontier_cb(n, msg), 10)
            self._assign_clients[ns] = self.create_client(
                AssignFrontier, f'/{ns}/assign_frontier',
                callback_group=cbg)
            self._land_pubs[ns] = self.create_publisher(Empty, f'/{ns}/cmd/land', 10)
            self._last_frontier_time[ns] = float('inf')

        # Latched publisher for mission-complete signal
        _latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_mission_complete = self.create_publisher(Bool, '/mission_complete', _latch)

        # Coordination loop at 1 Hz
        self.create_timer(1.0, self._coordinate, callback_group=cbg)

        self.get_logger().info(
            f'DroneCoordinator managing: {self._namespaces}')

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _state_cb(self, ns: str, msg: DroneState):
        self._states[ns] = msg

    def _frontier_cb(self, ns: str, msg: FrontierList):
        self._frontiers[ns] = msg
        self._last_frontier_time[ns] = self.get_clock().now().nanoseconds * 1e-9

    # ── Coordination ───────────────────────────────────────────────────────
    def _coordinate(self):
        if self._mission_complete:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9

        # Low-battery check: trigger landing for any drone below threshold
        for ns, state in self._states.items():
            bat = state.battery_percent
            if 0.0 < bat < self._low_bat_threshold:
                self.get_logger().warn(
                    f'[coordinator] {ns} low battery ({bat:.1f}%) → landing')
                self._land_pubs[ns].publish(Empty())

        # Mission-complete check
        all_reported = len(self._states) == len(self._namespaces)
        all_idle = all_reported and all(
            s.status == _STATUS_IDLE for s in self._states.values())
        no_frontiers = all(
            self._last_frontier_time[ns] != float('inf') and
            (now_s - self._last_frontier_time[ns]) > self._no_frontier_timeout
            for ns in self._namespaces)
        if all_idle and no_frontiers:
            self._mission_complete = True
            self.get_logger().info('[coordinator] Mission complete — all frontiers exhausted')
            self._pub_mission_complete.publish(Bool(data=True))
            for ns in self._namespaces:
                self._land_pubs[ns].publish(Empty())
            return

        # Build list of currently claimed goal centroids (one per non-idle drone)
        claimed: list[Point] = []
        for ns, state in self._states.items():
            if state.status != _STATUS_IDLE:
                claimed.append(state.current_goal)

        for ns in self._namespaces:
            state = self._states.get(ns)
            if state is None or state.status != _STATUS_IDLE:
                continue  # drone not idle — skip

            # Skip drones with low battery (they are landing)
            if state.battery_percent < self._low_bat_threshold:
                continue

            best = self._pick_best_frontier(ns, state, claimed)
            if best is None:
                continue

            if not self._assign_clients[ns].service_is_ready():
                continue

            req = AssignFrontier.Request()
            req.drone_id           = ns
            req.frontier_centroid  = best
            future = self._assign_clients[ns].call_async(req)
            future.add_done_callback(
                lambda f, n=ns, g=best: self._assignment_done(n, g, f))

            # Optimistically claim this goal now so a second idle drone
            # does not race to the same frontier in this same tick
            claimed.append(best)

    def _pick_best_frontier(self,
                            ns: str,
                            state: DroneState,
                            claimed: list[Point]) -> Point | None:
        """Score all known frontiers and return the best unclaimed one."""
        drone_x = state.current_pose.pose.position.x
        drone_y = state.current_pose.pose.position.y

        best_score   = -1.0
        best_centroid = None

        for src_ns, fl in self._frontiers.items():
            for centroid, size in zip(fl.centroids, fl.sizes):
                # Skip if another drone is already heading here
                if any(self._dist(centroid, c) < self._safety_radius
                       for c in claimed):
                    continue

                d = max(1.0, math.sqrt(
                    (centroid.x - drone_x)**2 +
                    (centroid.y - drone_y)**2))
                score = size / d

                if score > best_score:
                    best_score    = score
                    best_centroid = centroid

        return best_centroid

    @staticmethod
    def _dist(a: Point, b: Point) -> float:
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    def _assignment_done(self, ns: str, goal: Point, future):
        try:
            res = future.result()
            if res.accepted:
                self.get_logger().info(
                    f'[coordinator] {ns} accepted frontier '
                    f'({goal.x:.1f}, {goal.y:.1f})')
            else:
                self.get_logger().warn(
                    f'[coordinator] {ns} rejected: {res.reason}')
        except Exception as e:
            self.get_logger().error(f'[coordinator] assign call failed: {e}')


def main():
    rclpy.init()
    node = DroneCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
