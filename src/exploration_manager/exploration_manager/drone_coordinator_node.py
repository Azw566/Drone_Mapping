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
  /poi/detections         drone_interfaces/ArucoDetection (optional)

Service clients
  /{ns}/assign_frontier   drone_interfaces/AssignFrontier
"""

from dataclasses import dataclass
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty
from drone_interfaces.msg import FrontierList, DroneState, ArucoDetection
from drone_interfaces.srv import AssignFrontier

_STATUS_IDLE = 'idle'
_STATUS_EXPLORING = 'exploring'


@dataclass
class _SearchTarget:
    tag_id: int
    point: Point
    label: str
    last_assigned_s: float = -1.0


@dataclass
class _InterestPoint:
    point: Point
    label: str
    last_assigned_s: float = -1.0
    visited: bool = False


class DroneCoordinatorNode(Node):
    def __init__(self):
        super().__init__('drone_coordinator')

        self.declare_parameter('drone_namespaces', ['d1', 'd2'])
        self.declare_parameter('safety_radius', 3.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('no_frontier_timeout', 30.0)
        self.declare_parameter('max_assignment_distance', 15.0)
        self.declare_parameter('min_assignment_distance', 1.5)
        self.declare_parameter('idle_completion_dwell_s', 5.0)
        self.declare_parameter('enable_poi_search', False)
        self.declare_parameter('required_tag_ids', '')
        self.declare_parameter('poi_search_targets', '')
        self.declare_parameter('poi_search_cooldown_s', 30.0)
        self.declare_parameter('poi_search_leg_timeout_s', 45.0)
        self.declare_parameter('poi_search_preempts_frontiers', False)
        self.declare_parameter('drone_spawn_positions', '')
        self.declare_parameter('poi_reference_tag_positions', '')
        self.declare_parameter('max_tag_offset_refine_delta_m', 1.5)
        self.declare_parameter('enable_interest_points', False)
        self.declare_parameter('interest_points', '')
        self.declare_parameter('interest_points_preempt_frontiers', False)
        self.declare_parameter('interest_point_cooldown_s', 10.0)
        self.declare_parameter('require_interest_points_for_completion', False)

        namespaces = (self.get_parameter('drone_namespaces')
                      .get_parameter_value().string_array_value)
        self._safety_radius = (self.get_parameter('safety_radius')
                                .get_parameter_value().double_value)
        self._low_bat_threshold = (self.get_parameter('low_battery_threshold')
                                   .get_parameter_value().double_value)
        self._no_frontier_timeout = (self.get_parameter('no_frontier_timeout')
                                     .get_parameter_value().double_value)
        self._max_assignment_distance = (self.get_parameter('max_assignment_distance')
                                         .get_parameter_value().double_value)
        self._min_assignment_distance = (self.get_parameter('min_assignment_distance')
                                         .get_parameter_value().double_value)
        self._idle_completion_dwell_s = (self.get_parameter('idle_completion_dwell_s')
                                         .get_parameter_value().double_value)
        self._enable_poi_search = (self.get_parameter('enable_poi_search')
                                   .get_parameter_value().bool_value)
        self._poi_search_cooldown_s = (self.get_parameter('poi_search_cooldown_s')
                                       .get_parameter_value().double_value)
        self._poi_search_leg_timeout_s = (self.get_parameter('poi_search_leg_timeout_s')
                                          .get_parameter_value().double_value)
        self._poi_search_preempts_frontiers = (
            self.get_parameter('poi_search_preempts_frontiers')
            .get_parameter_value().bool_value)
        self._max_tag_offset_refine_delta_m = (
            self.get_parameter('max_tag_offset_refine_delta_m')
            .get_parameter_value().double_value)
        self._enable_interest_points = (
            self.get_parameter('enable_interest_points')
            .get_parameter_value().bool_value)
        self._interest_points_preempt_frontiers = (
            self.get_parameter('interest_points_preempt_frontiers')
            .get_parameter_value().bool_value)
        self._interest_point_cooldown_s = (
            self.get_parameter('interest_point_cooldown_s')
            .get_parameter_value().double_value)
        self._require_interest_points_for_completion = (
            self.get_parameter('require_interest_points_for_completion')
            .get_parameter_value().bool_value)
        self._namespaces = list(namespaces)
        self._required_tag_ids = self._parse_required_tag_ids(
            self.get_parameter('required_tag_ids').get_parameter_value().string_value)
        self._poi_search_targets = self._parse_search_targets(
            self.get_parameter('poi_search_targets').get_parameter_value().string_value)
        self._interest_points = self._parse_interest_points(
            self.get_parameter('interest_points').get_parameter_value().string_value)
        self._spawn_world_by_ns = self._parse_namespaced_points(
            self.get_parameter('drone_spawn_positions').get_parameter_value().string_value)
        self._tag_world_positions = self._parse_tag_points(
            self.get_parameter('poi_reference_tag_positions').get_parameter_value().string_value)

        # Per-drone state
        self._states:    dict[str, DroneState]    = {}
        self._frontiers: dict[str, FrontierList]  = {}
        self._assign_clients: dict[str, object] = {}
        self._land_pubs:          dict[str, object] = {}
        self._last_frontier_time: dict[str, float]  = {}
        self._mission_complete = False
        self._exploration_started = False
        self._no_assignable_since = None
        self._found_tags: set[int] = set()
        self._active_poi_search_by_ns: dict[str, _SearchTarget] = {}
        self._tag_last_assigned_s: dict[int, float] = {}
        self._map_offset_by_ns: dict[str, tuple[float, float]] = {}
        self._tag_offset_by_ns: dict[str, dict[int, tuple[float, float]]] = {}
        self._search_reassignment_due: set[str] = set()
        self._search_reassignment_anchor_by_ns: dict[str, Point] = {}
        self._timed_out_search_target_by_ns: dict[str, _SearchTarget] = {}
        self._active_interest_point_by_ns: dict[str, _InterestPoint] = {}
        self._last_status_by_ns: dict[str, str] = {}

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
            # Initialise to current ROS time so the no-frontier timeout is
            # measured from node start, not from the epoch.  This prevents the
            # coordinator from declaring mission-complete before the first
            # frontier message ever arrives (which would happen with 0.0 because
            # now_s >> no_frontier_timeout right from the first tick).
            # If a drone's map pipeline never starts, this timestamp simply ages
            # out after no_frontier_timeout seconds from node startup, which is
            # the desired "don't block forever" behaviour.
            self._last_frontier_time[ns] = self.get_clock().now().nanoseconds * 1e-9

        if self._enable_poi_search:
            self.create_subscription(
                ArucoDetection, '/poi/detections', self._poi_cb, 10)

        # Latched publisher for mission-complete signal
        _latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_mission_complete = self.create_publisher(Bool, '/mission_complete', _latch)

        # Coordination loop at 1 Hz
        self.create_timer(1.0, self._coordinate, callback_group=cbg)

        search_summary = ''
        if self._enable_poi_search:
            search_summary = (
                f' | poi_search tags={sorted(self._required_tag_ids)} '
                f'targets={len(self._poi_search_targets)}')
        if self._enable_interest_points:
            search_summary += (
                f' | interest_points={len(self._interest_points)} '
                f'preempt={self._interest_points_preempt_frontiers}')
        self.get_logger().info(
            f'DroneCoordinator managing: {self._namespaces}{search_summary}')

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _state_cb(self, ns: str, msg: DroneState):
        previous_status = self._last_status_by_ns.get(ns)
        self._states[ns] = msg
        self._last_status_by_ns[ns] = msg.status
        self._maybe_bootstrap_map_offset(ns, msg)
        if msg.status == _STATUS_IDLE and previous_status != _STATUS_IDLE:
            interest_point = self._active_interest_point_by_ns.pop(ns, None)
            if interest_point is not None and not interest_point.visited:
                interest_point.visited = True
                self.get_logger().info(
                    f'[coordinator] {ns} visited interest point {interest_point.label}')
        if msg.status == _STATUS_IDLE:
            self._active_poi_search_by_ns.pop(ns, None)
            self._search_reassignment_anchor_by_ns.pop(ns, None)
            self._timed_out_search_target_by_ns.pop(ns, None)

    def _frontier_cb(self, ns: str, msg: FrontierList):
        self._frontiers[ns] = msg
        self._last_frontier_time[ns] = self.get_clock().now().nanoseconds * 1e-9

    def _poi_cb(self, msg: ArucoDetection):
        tag_id = int(msg.tag_id)
        self._found_tags.add(tag_id)
        ns = str(msg.detected_by).strip()
        self._maybe_refine_map_offset_from_tag(ns, tag_id, msg)
        for drone_ns, target in list(self._active_poi_search_by_ns.items()):
            if target.tag_id == tag_id:
                self._search_reassignment_anchor_by_ns[drone_ns] = (
                    self._copy_point(self._search_target_point_in_map(drone_ns, target)))
                self._active_poi_search_by_ns.pop(drone_ns, None)
                self._timed_out_search_target_by_ns.pop(drone_ns, None)
                if self._required_tag_ids - self._found_tags:
                    self._search_reassignment_due.add(drone_ns)

    # ── Coordination ───────────────────────────────────────────────────────
    def _coordinate(self):
        if self._mission_complete:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        missing_tags = self._required_tag_ids - self._found_tags

        for ns, target in list(self._active_poi_search_by_ns.items()):
            if ns in self._search_reassignment_due:
                continue
            if target.tag_id not in missing_tags:
                continue
            if target.last_assigned_s < 0.0:
                continue
            if (now_s - target.last_assigned_s) < self._poi_search_leg_timeout_s:
                continue
            self._search_reassignment_due.add(ns)
            self._search_reassignment_anchor_by_ns[ns] = (
                self._copy_point(self._search_target_point_in_map(ns, target)))
            self._timed_out_search_target_by_ns[ns] = target
            self.get_logger().info(
                f'[coordinator] {ns} POI search tag {target.tag_id} timed out '
                f'— trying an alternate viewpoint')

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
        all_frontiers_reported = len(self._frontiers) == len(self._namespaces)
        all_frontiers_empty = all_frontiers_reported and all(
            len(self._frontiers[ns].centroids) == 0 for ns in self._namespaces)
        # A drone counts as "no frontiers" if:
        #   (a) it has sent at least one frontier message but the last one was
        #       more than no_frontier_timeout seconds ago, OR
        #   (b) its map pipeline never started — we give it no_frontier_timeout
        #       seconds from node start (now_s − 0.0) before declaring it done,
        #       which prevents a broken drone from blocking mission completion
        #       indefinitely.
        no_frontiers = all(
            (now_s - self._last_frontier_time[ns]) > self._no_frontier_timeout
            for ns in self._namespaces)
        no_assignable_work = all_idle and all_reported and all(
            self._pick_best_frontier(ns, self._states[ns], [], now_s) is None and
            self._pick_best_poi_search_target(ns, self._states[ns], [], now_s) is None and
            self._pick_best_interest_point(ns, self._states[ns], [], now_s) is None
            for ns in self._namespaces)
        if no_assignable_work and self._exploration_started:
            if self._no_assignable_since is None:
                self._no_assignable_since = now_s
        else:
            self._no_assignable_since = None
        idle_completion_ready = (
            self._exploration_started and
            self._no_assignable_since is not None and
            (now_s - self._no_assignable_since) >= self._idle_completion_dwell_s)
        all_required_tags_found = (
            not self._enable_poi_search or
            not self._required_tag_ids or
            not missing_tags)
        all_interest_points_complete = (
            not self._enable_interest_points or
            not self._require_interest_points_for_completion or
            all(point.visited for point in self._interest_points))
        if all_idle and all_required_tags_found and all_interest_points_complete and (
                all_frontiers_empty or no_frontiers or idle_completion_ready):
            self._mission_complete = True
            self.get_logger().info('[coordinator] Mission complete — all work exhausted')
            self._pub_mission_complete.publish(Bool(data=True))
            for ns in self._namespaces:
                self._land_pubs[ns].publish(Empty())
            return

        # Build list of currently claimed goal centroids (one per non-idle drone)
        claimed_by_ns: dict[str, Point] = {}
        claimed_search_tag_ids = {
            target.tag_id
            for other_ns, target in self._active_poi_search_by_ns.items()
            if other_ns not in self._search_reassignment_due
        }
        claimed_interest_labels = {
            point.label
            for other_ns, point in self._active_interest_point_by_ns.items()
            if other_ns in self._states and self._states[other_ns].status != _STATUS_IDLE
        }
        for ns, state in self._states.items():
            if state.status != _STATUS_IDLE and ns not in self._search_reassignment_due:
                claimed_by_ns[ns] = state.current_goal

        for ns in self._namespaces:
            state = self._states.get(ns)
            search_reassign_due = ns in self._search_reassignment_due
            if state is None:
                continue
            if state.status != _STATUS_IDLE and not search_reassign_due:
                continue  # drone not idle — skip

            # Skip drones with low battery (they are landing)
            if state.battery_percent < self._low_bat_threshold:
                continue

            other_claimed = [
                point for other_ns, point in claimed_by_ns.items()
                if other_ns != ns
            ]
            assignment_label = 'frontier'
            search_target = None
            interest_point = None
            best = None
            retry_target = self._timed_out_search_target_by_ns.get(ns)
            prefer_poi_search = (
                self._enable_poi_search and
                (self._poi_search_preempts_frontiers or search_reassign_due) and
                bool(missing_tags)
            )
            prefer_interest_points = (
                self._enable_interest_points and
                self._interest_points_preempt_frontiers)
            if prefer_interest_points:
                interest_point = self._pick_best_interest_point(
                    ns,
                    state,
                    other_claimed,
                    now_s,
                    claimed_labels=claimed_interest_labels,
                )
                if interest_point is not None:
                    best = self._interest_point_in_map(ns, interest_point)
                    interest_point.last_assigned_s = now_s
                    assignment_label = f'interest point {interest_point.label}'
            if best is None and prefer_poi_search:
                search_target = self._pick_best_poi_search_target(
                    ns,
                    state,
                    other_claimed,
                    now_s,
                    retry_target=retry_target,
                    claimed_tag_ids=claimed_search_tag_ids,
                )
                if search_target is not None:
                    best = self._search_target_point_in_map(ns, search_target)
                    search_target.last_assigned_s = now_s
                    assignment_label = f'POI search tag {search_target.tag_id}'
            if best is None:
                best = self._pick_best_frontier(ns, state, other_claimed, now_s)
            if best is None and not prefer_poi_search:
                search_target = self._pick_best_poi_search_target(
                    ns,
                    state,
                    other_claimed,
                    now_s,
                    claimed_tag_ids=claimed_search_tag_ids,
                )
                if search_target is not None:
                    best = self._search_target_point_in_map(ns, search_target)
                    search_target.last_assigned_s = now_s
                    assignment_label = f'POI search tag {search_target.tag_id}'
            if best is None and not prefer_interest_points:
                interest_point = self._pick_best_interest_point(
                    ns,
                    state,
                    other_claimed,
                    now_s,
                    claimed_labels=claimed_interest_labels,
                )
                if interest_point is not None:
                    best = self._interest_point_in_map(ns, interest_point)
                    interest_point.last_assigned_s = now_s
                    assignment_label = f'interest point {interest_point.label}'
            if best is None:
                if search_reassign_due:
                    self._search_reassignment_due.discard(ns)
                    self._search_reassignment_anchor_by_ns.pop(ns, None)
                    self._timed_out_search_target_by_ns.pop(ns, None)
                continue

            if not self._assign_clients[ns].service_is_ready():
                continue

            req = AssignFrontier.Request()
            req.drone_id           = ns
            req.frontier_centroid  = best
            future = self._assign_clients[ns].call_async(req)
            future.add_done_callback(
                lambda f, n=ns, g=best, l=assignment_label, t=search_target, i=interest_point:
                    self._assignment_done(n, g, l, t, i, f))

            # Optimistically claim this goal now so a second idle drone
            # does not race to the same frontier in this same tick
            claimed_by_ns[ns] = best
            if search_target is not None:
                claimed_search_tag_ids.add(search_target.tag_id)
            if interest_point is not None:
                claimed_interest_labels.add(interest_point.label)

    def _pick_best_frontier(self,
                            ns: str,
                            state: DroneState,
                            claimed: list[Point],
                            now_s: float) -> Point | None:
        """Score all known frontiers and return the best unclaimed one."""
        drone_x = state.current_pose.pose.position.x
        drone_y = state.current_pose.pose.position.y

        best_score   = -1.0
        best_centroid = None

        for src_ns, fl in self._frontiers.items():
            if (now_s - self._last_frontier_time.get(src_ns, 0.0)) > self._no_frontier_timeout:
                continue
            for centroid, size in zip(fl.centroids, fl.sizes):
                # Skip if another drone is already heading here
                if any(self._dist(centroid, c) < self._safety_radius
                       for c in claimed):
                    continue

                d = math.sqrt(
                    (centroid.x - drone_x)**2 +
                    (centroid.y - drone_y)**2)
                if self._min_assignment_distance > 0.0 and d < self._min_assignment_distance:
                    continue
                d = max(1.0, d)
                if self._max_assignment_distance > 0.0 and d > self._max_assignment_distance:
                    continue
                score = size / d

                if score > best_score:
                    best_score    = score
                    best_centroid = centroid

        return best_centroid

    def _pick_best_interest_point(self,
                                  ns: str,
                                  state: DroneState,
                                  claimed: list[Point],
                                  now_s: float,
                                  claimed_labels: set[str] | None = None
                                  ) -> _InterestPoint | None:
        if not self._enable_interest_points or not self._interest_points:
            return None
        if claimed_labels is None:
            claimed_labels = set()

        drone_x = state.current_pose.pose.position.x
        drone_y = state.current_pose.pose.position.y
        best_interest_point = None
        best_distance = None
        for interest_point in self._interest_points:
            if interest_point.visited:
                continue
            if interest_point.label in claimed_labels:
                continue
            if (now_s - interest_point.last_assigned_s) < self._interest_point_cooldown_s:
                continue
            map_point = self._interest_point_in_map(ns, interest_point)
            if any(self._dist(map_point, c) < self._safety_radius for c in claimed):
                continue
            distance = math.hypot(
                map_point.x - drone_x,
                map_point.y - drone_y,
            )
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_interest_point = interest_point

        return best_interest_point

    def _pick_best_poi_search_target(self,
                                     ns: str,
                                     state: DroneState,
                                     claimed: list[Point],
                                     now_s: float,
                                     retry_target: _SearchTarget | None = None,
                                     claimed_tag_ids: set[int] | None = None) -> _SearchTarget | None:
        if not self._enable_poi_search or not self._poi_search_targets:
            return None

        missing_tags = self._required_tag_ids - self._found_tags
        if not missing_tags:
            return None
        if claimed_tag_ids is None:
            claimed_tag_ids = set()

        anchor = self._search_reassignment_anchor_by_ns.get(ns)
        if anchor is None:
            drone_x = state.current_pose.pose.position.x
            drone_y = state.current_pose.pose.position.y
        else:
            drone_x = anchor.x
            drone_y = anchor.y
        anchor_side = self._side_bucket(drone_x)
        active_tag_ids = {
            target.tag_id
            for other_ns, target in self._active_poi_search_by_ns.items()
            if other_ns != ns
        }

        if retry_target is not None and retry_target.tag_id in missing_tags:
            retry_pick = self._pick_search_target_for_tag(
                ns,
                retry_target.tag_id,
                drone_x,
                drone_y,
                claimed,
                now_s,
                anchor_side,
                allow_opposite_side=True,
                exclude_label=retry_target.label,
            )
            if retry_pick is not None:
                return retry_pick[0]

        for skip_active_tags in (True, False):
            side_passes = [True] if anchor_side == 0 else [False, True]
            for allow_opposite_side in side_passes:
                best_priority = None
                best_target = None
                for tag_id in sorted(missing_tags):
                    if skip_active_tags and tag_id in active_tag_ids:
                        continue
                    if tag_id in claimed_tag_ids and (
                            retry_target is None or tag_id != retry_target.tag_id):
                        continue

                    candidate = self._pick_search_target_for_tag(
                        ns,
                        tag_id,
                        drone_x,
                        drone_y,
                        claimed,
                        now_s,
                        anchor_side,
                        allow_opposite_side=allow_opposite_side,
                    )
                    if candidate is None:
                        continue
                    nearest_target, nearest_distance = candidate

                    tag_world_point = self._tag_world_positions.get(tag_id)
                    tag_side = self._side_bucket(
                        tag_world_point.x if tag_world_point is not None else nearest_target.point.x)
                    side_priority = 0
                    if anchor_side != 0:
                        if tag_side == anchor_side:
                            side_priority = 2
                        elif tag_side == 0:
                            side_priority = 1
                    last_assigned_s = self._tag_last_assigned_s.get(tag_id, -1.0)
                    priority = (
                        side_priority,
                        1 if last_assigned_s < 0.0 else 0,
                        0.0 if last_assigned_s < 0.0 else (now_s - last_assigned_s),
                        -nearest_distance,
                        -float(tag_id),
                    )
                    if best_priority is None or priority > best_priority:
                        best_priority = priority
                        best_target = nearest_target

                if best_target is not None:
                    return best_target

        return None

    def _pick_search_target_for_tag(self,
                                    ns: str,
                                    tag_id: int,
                                    drone_x: float,
                                    drone_y: float,
                                    claimed: list[Point],
                                    now_s: float,
                                    anchor_side: int,
                                    allow_opposite_side: bool,
                                    exclude_label: str | None = None
                                    ) -> tuple[_SearchTarget, float] | None:
        nearest_target = None
        nearest_distance = None
        for target in self._poi_search_targets:
            if target.tag_id != tag_id:
                continue
            if exclude_label is not None and target.label == exclude_label:
                continue
            if (now_s - target.last_assigned_s) < self._poi_search_cooldown_s:
                continue
            map_point = self._search_target_point_in_map(ns, target)
            if any(self._dist(map_point, c) < self._safety_radius for c in claimed):
                continue
            candidate_side = self._side_bucket(map_point.x)
            if (anchor_side != 0 and not allow_opposite_side and
                    candidate_side not in (0, anchor_side)):
                continue

            distance = math.hypot(
                map_point.x - drone_x,
                map_point.y - drone_y,
            )
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_target = target

        if nearest_target is None or nearest_distance is None:
            return None
        return nearest_target, nearest_distance

    def _search_target_point_in_map(self, ns: str, target: _SearchTarget) -> Point:
        point = Point()
        offset = self._map_offset_by_ns.get(ns)
        if offset is None:
            point.x = target.point.x
            point.y = target.point.y
        else:
            point.x = target.point.x + offset[0]
            point.y = target.point.y + offset[1]
        point.z = target.point.z
        return point

    def _interest_point_in_map(self, ns: str, interest_point: _InterestPoint) -> Point:
        point = Point()
        offset = self._map_offset_by_ns.get(ns)
        if offset is None:
            point.x = interest_point.point.x
            point.y = interest_point.point.y
        else:
            point.x = interest_point.point.x + offset[0]
            point.y = interest_point.point.y + offset[1]
        point.z = interest_point.point.z
        return point

    def _maybe_bootstrap_map_offset(self, ns: str, msg: DroneState):
        if ns in self._map_offset_by_ns:
            return
        spawn = self._spawn_world_by_ns.get(ns)
        if spawn is None:
            return
        pose = msg.current_pose.pose.position
        self._map_offset_by_ns[ns] = (
            float(pose.x - spawn.x),
            float(pose.y - spawn.y),
        )
        self.get_logger().info(
            f'[coordinator] {ns} map offset bootstrapped '
            f'Δx={self._map_offset_by_ns[ns][0]:.2f} '
            f'Δy={self._map_offset_by_ns[ns][1]:.2f}')

    def _maybe_refine_map_offset_from_tag(self, ns: str, tag_id: int, msg: ArucoDetection):
        if not ns:
            return
        world_point = self._tag_world_positions.get(tag_id)
        if world_point is None:
            return
        observed = msg.world_pose.pose.position
        observed_offset = (
            float(observed.x - world_point.x),
            float(observed.y - world_point.y),
        )
        current_offset = self._map_offset_by_ns.get(ns)
        if current_offset is not None and self._max_tag_offset_refine_delta_m > 0.0:
            delta = math.hypot(
                observed_offset[0] - current_offset[0],
                observed_offset[1] - current_offset[1],
            )
            if delta > self._max_tag_offset_refine_delta_m:
                self.get_logger().warn(
                    f'[coordinator] Ignoring tag {tag_id} offset update for {ns} '
                    f'(Δ={delta:.2f} m > {self._max_tag_offset_refine_delta_m:.2f} m)')
                return
        per_ns = self._tag_offset_by_ns.setdefault(ns, {})
        per_ns[tag_id] = observed_offset
        samples = list(per_ns.values())
        avg_dx = sum(sample[0] for sample in samples) / len(samples)
        avg_dy = sum(sample[1] for sample in samples) / len(samples)
        self._map_offset_by_ns[ns] = (avg_dx, avg_dy)
        self.get_logger().info(
            f'[coordinator] {ns} map offset refined from tag {tag_id} '
            f'→ Δx={avg_dx:.2f} Δy={avg_dy:.2f}')

    def _defer_active_search_retry(self, ns: str):
        active_target = self._active_poi_search_by_ns.get(ns)
        if active_target is None:
            return
        active_target.last_assigned_s = self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _dist(a: Point, b: Point) -> float:
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    @staticmethod
    def _copy_point(src: Point) -> Point:
        point = Point()
        point.x = float(src.x)
        point.y = float(src.y)
        point.z = float(src.z)
        return point

    @staticmethod
    def _side_bucket(x: float) -> int:
        if x > 2.0:
            return 1
        if x < -2.0:
            return -1
        return 0

    @staticmethod
    def _parse_required_tag_ids(raw: str) -> set[int]:
        return {
            int(token.strip()) for token in raw.split(',')
            if token.strip()
        }

    def _parse_search_targets(self, raw: str) -> list[_SearchTarget]:
        targets = []
        for idx, token in enumerate(raw.split(';')):
            token = token.strip()
            if not token:
                continue
            parts = [part.strip() for part in token.split(':')]
            if len(parts) not in (3, 4):
                self.get_logger().warn(
                    f'[coordinator] Ignoring malformed poi_search_targets entry: {token}')
                continue
            try:
                tag_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3]) if len(parts) == 4 else 0.0
            except ValueError:
                self.get_logger().warn(
                    f'[coordinator] Ignoring non-numeric poi_search_targets entry: {token}')
                continue
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            targets.append(_SearchTarget(tag_id=tag_id, point=point, label=f'target_{idx}'))
        return targets

    def _parse_interest_points(self, raw: str) -> list[_InterestPoint]:
        interest_points = []
        for idx, token in enumerate(raw.split(';')):
            token = token.strip()
            if not token:
                continue
            parts = [part.strip() for part in token.split(':')]
            if len(parts) == 3:
                label = f'interest_{idx}'
                coord_parts = parts
            elif len(parts) == 4:
                label = parts[0] or f'interest_{idx}'
                coord_parts = parts[1:]
            else:
                self.get_logger().warn(
                    f'[coordinator] Ignoring malformed interest_points entry: {token}')
                continue
            try:
                x = float(coord_parts[0])
                y = float(coord_parts[1])
                z = float(coord_parts[2])
            except ValueError:
                self.get_logger().warn(
                    f'[coordinator] Ignoring non-numeric interest_points entry: {token}')
                continue
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            interest_points.append(_InterestPoint(point=point, label=label))
        return interest_points

    def _parse_namespaced_points(self, raw: str) -> dict[str, Point]:
        points = {}
        for token in raw.split(';'):
            token = token.strip()
            if not token:
                continue
            parts = [part.strip() for part in token.split(':')]
            if len(parts) != 3:
                self.get_logger().warn(
                    f'[coordinator] Ignoring malformed drone_spawn_positions entry: {token}')
                continue
            ns = parts[0]
            try:
                x = float(parts[1])
                y = float(parts[2])
            except ValueError:
                self.get_logger().warn(
                    f'[coordinator] Ignoring non-numeric drone_spawn_positions entry: {token}')
                continue
            point = Point()
            point.x = x
            point.y = y
            points[ns] = point
        return points

    def _parse_tag_points(self, raw: str) -> dict[int, Point]:
        points = {}
        for token in raw.split(';'):
            token = token.strip()
            if not token:
                continue
            parts = [part.strip() for part in token.split(':')]
            if len(parts) != 3:
                self.get_logger().warn(
                    f'[coordinator] Ignoring malformed poi_reference_tag_positions entry: {token}')
                continue
            try:
                tag_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
            except ValueError:
                self.get_logger().warn(
                    f'[coordinator] Ignoring non-numeric poi_reference_tag_positions entry: {token}')
                continue
            point = Point()
            point.x = x
            point.y = y
            points[tag_id] = point
        return points

    def _assignment_done(self,
                         ns: str,
                         goal: Point,
                         label: str,
                         search_target: _SearchTarget | None,
                         interest_point: _InterestPoint | None,
                         future):
        try:
            res = future.result()
            if res.accepted:
                self._exploration_started = True
                if search_target is not None:
                    self._tag_last_assigned_s[search_target.tag_id] = search_target.last_assigned_s
                    self._active_poi_search_by_ns[ns] = search_target
                else:
                    self._active_poi_search_by_ns.pop(ns, None)
                if interest_point is not None:
                    self._active_interest_point_by_ns[ns] = interest_point
                else:
                    self._active_interest_point_by_ns.pop(ns, None)
                self._search_reassignment_due.discard(ns)
                self._search_reassignment_anchor_by_ns.pop(ns, None)
                self._timed_out_search_target_by_ns.pop(ns, None)
                self.get_logger().info(
                    f'[coordinator] {ns} accepted {label} '
                    f'({goal.x:.1f}, {goal.y:.1f})')
            else:
                if search_target is not None:
                    search_target.last_assigned_s = -1.0
                    self._defer_active_search_retry(ns)
                if interest_point is not None:
                    interest_point.last_assigned_s = -1.0
                self._active_interest_point_by_ns.pop(ns, None)
                self._search_reassignment_due.discard(ns)
                self._search_reassignment_anchor_by_ns.pop(ns, None)
                self._timed_out_search_target_by_ns.pop(ns, None)
                self.get_logger().warn(
                    f'[coordinator] {ns} rejected: {res.reason}')
        except Exception as e:
            if search_target is not None:
                search_target.last_assigned_s = -1.0
                self._defer_active_search_retry(ns)
            if interest_point is not None:
                interest_point.last_assigned_s = -1.0
            self._active_interest_point_by_ns.pop(ns, None)
            self._search_reassignment_due.discard(ns)
            self._search_reassignment_anchor_by_ns.pop(ns, None)
            self._timed_out_search_target_by_ns.pop(ns, None)
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
