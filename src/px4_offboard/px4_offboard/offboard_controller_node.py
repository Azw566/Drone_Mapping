"""
offboard_controller_node.py

Arm → switch to OFFBOARD → takeoff to hover altitude → fly to
exploration goals sent by the coordinator.

State machine
  IDLE         boot state; waits for xy_valid + z_valid
  PRE_ARM      publish OffboardControlMode + hold setpoint for 2 s
               (PX4 requires 2 s of continuous setpoints before OFFBOARD switch)
  SWITCHING    request OFFBOARD mode, retry every 1 s
  ARMING       OFFBOARD confirmed; send ARM command, retry every 2 s
  TAKING_OFF   climb via VELOCITY setpoint (-1.5 m/s NED z) until near HOVER_ALT
               (velocity mode avoids the aggressive position-error spool-up from ground)
  HOVER        at altitude, position setpoints; waiting for goal_pose
  EXPLORING    flying toward current goal (position setpoints)

Goals are received as geometry_msgs/Point in the LIO-SAM map frame (ENU).
They are converted to NED for the TrajectorySetpoint.

Published topics
  /{ns}/fmu/in/offboard_control_mode
  /{ns}/fmu/in/trajectory_setpoint
  /{ns}/fmu/in/vehicle_command
  /{ns}/drone_state   (drone_interfaces/DroneState)

Subscribed topics
  /{ns}/fmu/out/vehicle_status_v1
  /{ns}/fmu/out/vehicle_local_position
  /{ns}/goal_pose   (geometry_msgs/Point, ENU map frame)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

_NAN = float('nan')

# State labels
_IDLE        = 'idle'
_PRE_ARM     = 'pre_arm'
_SWITCHING   = 'switching'   # waiting for OFFBOARD mode confirmation
_ARMING      = 'arming'      # OFFBOARD confirmed; now arm
_OFFBOARD    = 'offboard'
_TAKING_OFF  = 'taking_off'
_HOVER       = 'hover'
_EXPLORING   = 'exploring'
_INSPECTING  = 'inspecting'
_LANDING     = 'landing'

# Config
HOVER_ALT_M      = 1.5    # metres above takeoff (NED z = -HOVER_ALT_M)
GOAL_RADIUS_M    = 0.5    # consider goal reached when within this distance
SEARCH_GOAL_RADIUS_M = GOAL_RADIUS_M
PRE_ARM_SECS     = 2.0    # seconds of publishing before arming
TAKEOFF_VEL_MS   = 1.5    # vertical climb speed during TAKING_OFF (m/s, NED -z)
MAX_EXPLORING_STEP_M = 0.08  # cap each horizontal setpoint update to ~0.8 m/s @ 10 Hz
FACE_GOAL_YAW = False
GOAL_SCAN_ENABLED = False
GOAL_SCAN_DURATION_S = 8.0
GOAL_SCAN_YAWSPEED_RAD_S = 0.6
HOVER_READY_TOLERANCE_M = 0.5
INSPECTION_ALTITUDE_M = HOVER_ALT_M


class OffboardControllerNode(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        self.declare_parameter('drone_ns', 'd1')
        self.declare_parameter('hover_alt', HOVER_ALT_M)
        self.declare_parameter('hover_ready_tolerance_m', HOVER_READY_TOLERANCE_M)
        self.declare_parameter('goal_radius_m', GOAL_RADIUS_M)
        self.declare_parameter('search_goal_radius_m', SEARCH_GOAL_RADIUS_M)
        self.declare_parameter('max_exploring_step_m', MAX_EXPLORING_STEP_M)
        self.declare_parameter('inspection_altitude_m', INSPECTION_ALTITUDE_M)
        self.declare_parameter('face_goal_yaw', FACE_GOAL_YAW)
        self.declare_parameter('goal_scan_enabled', GOAL_SCAN_ENABLED)
        self.declare_parameter('goal_scan_duration_s', GOAL_SCAN_DURATION_S)
        self.declare_parameter('goal_scan_yawspeed_rad_s', GOAL_SCAN_YAWSPEED_RAD_S)

        ns   = self.get_parameter('drone_ns').get_parameter_value().string_value
        self._ns = ns
        self._hover_alt = self.get_parameter('hover_alt').get_parameter_value().double_value
        self._hover_ready_tolerance_m = (
            self.get_parameter('hover_ready_tolerance_m').get_parameter_value().double_value)
        self._goal_radius_m = (
            self.get_parameter('goal_radius_m').get_parameter_value().double_value)
        self._search_goal_radius_m = (
            self.get_parameter('search_goal_radius_m').get_parameter_value().double_value)
        self._max_exploring_step_m = (
            self.get_parameter('max_exploring_step_m').get_parameter_value().double_value)
        self._inspection_altitude_m = (
            self.get_parameter('inspection_altitude_m').get_parameter_value().double_value)
        self._face_goal_yaw = (
            self.get_parameter('face_goal_yaw').get_parameter_value().bool_value)
        self._goal_scan_enabled = (
            self.get_parameter('goal_scan_enabled').get_parameter_value().bool_value)
        self._goal_scan_duration_s = (
            self.get_parameter('goal_scan_duration_s').get_parameter_value().double_value)
        self._goal_scan_yawspeed_rad_s = (
            self.get_parameter('goal_scan_yawspeed_rad_s').get_parameter_value().double_value)

        # PX4 QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        lio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_ocm = self.create_publisher(
            OffboardControlMode, f'/{ns}/fmu/in/offboard_control_mode', px4_qos)
        self._pub_sp  = self.create_publisher(
            TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', px4_qos)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, f'/{ns}/fmu/in/vehicle_command', px4_qos)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, f'/{ns}/fmu/out/vehicle_status_v1',
            self._status_cb, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position',
            self._pos_cb, px4_qos)
        self.create_subscription(
            Odometry, f'/{ns}/lio_sam/mapping/odometry',
            self._lio_cb, lio_qos)
        self.create_subscription(
            Point, f'/{ns}/goal_pose', self._goal_cb, 10)
        self.create_subscription(
            Empty, f'/{ns}/cmd/land', self._land_cb, 10)

        # ── State ────────────────────────────────────────────────────────────
        self._state      = _IDLE
        self._nav_state  = -1
        self._arm_state  = 1   # 1 = DISARMED
        self._pos_ned    = [0.0, 0.0, 0.0]   # current NED position from PX4
        self._home_ned   = None               # NED position at first position fix
        self._lio_pos_enu = None              # current ENU position from LIO-SAM
        self._goal_enu   = None               # latest goal in ENU map frame
        self._pre_arm_start = None
        self._arm_last_attempt = None         # last time arm command was sent
        self._px4_us     = 0                  # latest PX4 timestamp (µs) from VehicleLocalPosition
        self._auto_start_enabled = True       # only auto-arm once per process
        self._inspect_started_s = None
        self._inspection_altitude_override_m = None
        # ── Main control timer (10 Hz) ───────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'[{ns}] OffboardController ready '
            f'(hover={self._hover_alt} m, hover_tol={self._hover_ready_tolerance_m:.2f} m, '
            f'goal_radius={self._goal_radius_m:.2f} m, '
            f'search_goal_radius={self._search_goal_radius_m:.2f} m, '
            f'max_step={self._max_exploring_step_m:.2f} m, '
            f'inspection_alt={self._inspection_altitude_m:.2f} m, '
            f'face_goal_yaw={self._face_goal_yaw}, goal_scan={self._goal_scan_enabled})')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _status_cb(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state
        self._arm_state = msg.arming_state

    def _pos_cb(self, msg: VehicleLocalPosition):
        self._px4_us  = msg.timestamp          # track PX4 clock to stamp our outgoing msgs
        self._pos_ned = [msg.x, msg.y, msg.z]
        if self._home_ned is None and msg.xy_valid and msg.z_valid:
            self._home_ned = [msg.x, msg.y, msg.z]
            self.get_logger().info(
                f'[{self._ns}] Home NED: {self._home_ned}')

    def _lio_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._lio_pos_enu = [p.x, p.y, p.z]

    def _land_cb(self, msg: Empty):
        if self._state != _IDLE:
            self.get_logger().info(f'[{self._ns}] Land command → LANDING')
            self._state = _LANDING

    def _goal_cb(self, msg: Point):
        """Accept a new goal in ENU map frame."""
        self._goal_enu = [msg.x, msg.y, msg.z]
        self._inspection_altitude_override_m = None
        if self._state == _HOVER:
            self._state = _EXPLORING
            self.get_logger().info(
                f'[{self._ns}] → EXPLORING goal ENU({msg.x:.1f},{msg.y:.1f})')

    # ── Control loop ──────────────────────────────────────────────────────
    def _control_loop(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if self._state == _IDLE:
            if self._auto_start_enabled and self._home_ned is not None:
                self._state = _PRE_ARM
                self._pre_arm_start = now
                self._auto_start_enabled = False
                self.get_logger().info(f'[{self._ns}] → PRE_ARM')

        elif self._state == _PRE_ARM:
            self._publish_ocm()
            self._publish_hold_setpoint()
            elapsed = (now - self._pre_arm_start).nanoseconds * 1e-9
            if elapsed >= PRE_ARM_SECS:
                self._state = _SWITCHING
                self._arm_last_attempt = now
                self.get_logger().info(f'[{self._ns}] → SWITCHING (request OFFBOARD mode)')
                self._send_offboard_command()

        elif self._state == _SWITCHING:
            # Step 1: keep publishing OCM+setpoints and retry OFFBOARD mode switch
            # until PX4 confirms nav_state == OFFBOARD (14).
            self._publish_ocm()
            self._publish_hold_setpoint()
            if self._nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._state = _ARMING
                self._arm_last_attempt = now
                self.get_logger().info(f'[{self._ns}] → ARMING (OFFBOARD confirmed, now arm)')
                self._send_arm_command()
            else:
                if (now - self._arm_last_attempt).nanoseconds * 1e-9 >= 1.0:
                    self._arm_last_attempt = now
                    self._send_offboard_command()

        elif self._state == _ARMING:
            # Step 2: OFFBOARD mode is active; retry arm until armed.
            self._publish_ocm()
            self._publish_hold_setpoint()
            if self._arm_state == 2:   # ARMED
                self._state = _TAKING_OFF
                self.get_logger().info(f'[{self._ns}] → TAKING_OFF (armed in OFFBOARD)')
            else:
                if (now - self._arm_last_attempt).nanoseconds * 1e-9 >= 2.0:
                    self._arm_last_attempt = now
                    self._send_arm_command()

        elif self._state == _OFFBOARD:
            # Legacy fallback (shouldn't be reached with new flow)
            self._publish_ocm()
            self._publish_hold_setpoint()
            self._state = _TAKING_OFF
            self.get_logger().info(f'[{self._ns}] → TAKING_OFF')

        elif self._state == _TAKING_OFF:
            # Use velocity setpoints to avoid aggressive position-error spool-up
            # from the ground. PX4 climbs at a steady rate regardless of altitude error.
            self._publish_ocm(velocity_mode=True)
            self._publish_takeoff_setpoint()
            if self._at_altitude():
                self._state = _HOVER
                self.get_logger().info(f'[{self._ns}] → HOVER')

        elif self._state == _HOVER:
            self._publish_ocm()
            self._publish_hover_setpoint()

        elif self._state == _EXPLORING:
            self._publish_ocm()
            self._publish_goal_setpoint()
            if self._goal_enu is not None and self._at_goal():
                if self._goal_scan_enabled and self._goal_scan_duration_s > 0.0:
                    self._state = _INSPECTING
                    self._inspect_started_s = now_s
                    self._inspection_altitude_override_m = self._goal_inspection_altitude_m()
                    self.get_logger().info(f'[{self._ns}] Goal reached → INSPECTING')
                else:
                    self._state = _HOVER
                    self.get_logger().info(f'[{self._ns}] Goal reached → HOVER')
                    self._inspection_altitude_override_m = None
                self._goal_enu = None

        elif self._state == _INSPECTING:
            self._publish_ocm()
            self._publish_inspection_setpoint(now_s)
            if (self._inspect_started_s is not None and
                    (now_s - self._inspect_started_s) >= self._goal_scan_duration_s):
                self._state = _HOVER
                self._inspect_started_s = None
                self._inspection_altitude_override_m = None
                self.get_logger().info(f'[{self._ns}] Inspection complete → HOVER')

        elif self._state == _LANDING:
            self._publish_ocm(velocity_mode=True)
            self._publish_landing_setpoint()
            if self._pos_ned[2] > -0.3:   # within 30 cm of ground (NED z → 0)
                if self._arm_state == 1:   # DISARMED
                    self._state    = _IDLE
                    self._goal_enu = None
                    self._inspect_started_s = None
                    self._inspection_altitude_override_m = None
                    self.get_logger().info(f'[{self._ns}] Landed → IDLE')
                elif (self._arm_last_attempt is None or
                      (now - self._arm_last_attempt).nanoseconds * 1e-9 >= 1.0):
                    self._arm_last_attempt = now
                    self._send_disarm_command()

    # ── Timestamp helper ──────────────────────────────────────────────────
    def _now_us(self) -> int:
        """Return a timestamp in PX4 microseconds.

        We prefer the latest PX4 clock value (from VehicleLocalPosition).
        Using PX4's own clock prevents stale-setpoint rejection during
        XRCE-DDS timesync drift — the most common cause of OFFBOARD loss
        in high-load Gazebo SITL runs.
        """
        if self._px4_us > 0:
            return self._px4_us
        return self.get_clock().now().nanoseconds // 1000

    # ── PX4 command helpers ────────────────────────────────────────────────
    def _publish_ocm(self, velocity_mode: bool = False):
        """Publish OffboardControlMode.

        velocity_mode=True  → velocity control (used during TAKING_OFF)
        velocity_mode=False → position control (PRE_ARM, HOVER, EXPLORING)
        """
        msg = OffboardControlMode()
        msg.timestamp    = self._now_us()
        msg.position     = not velocity_mode
        msg.velocity     = velocity_mode
        msg.acceleration = False
        self._pub_ocm.publish(msg)

    def _publish_hold_setpoint(self):
        """Hold current NED position."""
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        if self._home_ned:
            msg.position = [float(self._home_ned[0]),
                            float(self._home_ned[1]),
                            float(self._home_ned[2])]
        else:
            msg.position = [0.0, 0.0, 0.0]
        msg.yaw = _NAN  # hold current heading — don't rotate to north
        self._pub_sp.publish(msg)

    def _publish_takeoff_setpoint(self):
        """Climb at a steady velocity (velocity mode) to avoid aggressive spool-up from ground."""
        msg = TrajectorySetpoint()
        msg.timestamp  = self._now_us()
        # Position must be NaN when using velocity control mode
        msg.position   = [_NAN, _NAN, _NAN]
        # NED: negative z = upward.  Hold x/y at 0 (no horizontal drift).
        msg.velocity   = [0.0, 0.0, -TAKEOFF_VEL_MS]
        msg.yaw        = _NAN  # hold current heading — yaw rotation saturates motors
        self._pub_sp.publish(msg)

    def _publish_hover_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        p = self._pos_ned
        msg.position = [float(p[0]), float(p[1]), -self._hover_alt]
        msg.yaw = _NAN  # hold current heading
        self._pub_sp.publish(msg)

    def _publish_goal_setpoint(self):
        goal_ned = self._goal_setpoint_ned()
        if goal_ned is None:
            self._publish_hover_setpoint()
            return
        search_goal_active = self._goal_requests_tag_scan()
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(goal_ned[0]),
                        float(goal_ned[1]),
                        float(goal_ned[2])]
        msg.yaw = self._yaw_to_goal() if (self._face_goal_yaw and search_goal_active) else _NAN
        msg.yawspeed = 0.5 if (self._face_goal_yaw and search_goal_active) else 0.0
        self._pub_sp.publish(msg)

    def _publish_inspection_setpoint(self, now_s: float):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        p = self._pos_ned
        inspection_altitude_m = (
            self._inspection_altitude_override_m
            if self._inspection_altitude_override_m is not None
            else self._inspection_altitude_m
        )
        msg.position = [float(p[0]), float(p[1]), -inspection_altitude_m]
        msg.yaw = self._inspection_yaw(now_s)
        msg.yawspeed = float(self._goal_scan_yawspeed_rad_s)
        self._pub_sp.publish(msg)

    def _publish_landing_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position  = [_NAN, _NAN, _NAN]
        msg.velocity  = [0.0, 0.0, 0.8]   # NED +z = downward, 0.8 m/s
        msg.yaw       = _NAN
        self._pub_sp.publish(msg)

    def _send_disarm_command(self):
        msg = VehicleCommand()
        msg.timestamp     = self._now_us()
        msg.command       = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1        = 0.0
        msg.from_external = True
        self._pub_cmd.publish(msg)

    def _send_arm_command(self):
        msg = VehicleCommand()
        msg.timestamp      = self._now_us()
        msg.command        = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1         = 1.0   # arm
        msg.from_external  = True
        self._pub_cmd.publish(msg)

    def _send_offboard_command(self):
        msg = VehicleCommand()
        msg.timestamp     = self._now_us()
        msg.command       = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1        = 1.0   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2        = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.from_external = True
        self._pub_cmd.publish(msg)

    # ── Geometry helpers ───────────────────────────────────────────────────
    def _at_altitude(self) -> bool:
        return abs(self._pos_ned[2] - (-self._hover_alt)) < self._hover_ready_tolerance_m

    def _at_goal(self) -> bool:
        if self._goal_enu is None or self._lio_pos_enu is None:
            return False
        dx = self._lio_pos_enu[0] - self._goal_enu[0]
        dy = self._lio_pos_enu[1] - self._goal_enu[1]
        goal_radius_m = (
            self._search_goal_radius_m
            if self._goal_requests_tag_scan()
            else self._goal_radius_m
        )
        return math.sqrt(dx*dx + dy*dy) < goal_radius_m

    def _yaw_to_goal(self) -> float:
        goal_delta_ned = self._goal_delta_ned()
        if goal_delta_ned is None:
            return _NAN
        dx = goal_delta_ned[0]
        dy = goal_delta_ned[1]
        return math.atan2(dy, dx)

    def _goal_delta_ned(self):
        if self._goal_enu is None or self._lio_pos_enu is None:
            return None
        dx_enu = self._goal_enu[0] - self._lio_pos_enu[0]
        dy_enu = self._goal_enu[1] - self._lio_pos_enu[1]
        return [dy_enu, dx_enu]

    def _goal_setpoint_ned(self):
        goal_delta_ned = self._goal_delta_ned()
        if goal_delta_ned is None:
            return None
        dist = math.hypot(goal_delta_ned[0], goal_delta_ned[1])
        if dist > self._max_exploring_step_m and dist > 1e-6:
            scale = self._max_exploring_step_m / dist
            goal_delta_ned = [
                goal_delta_ned[0] * scale,
                goal_delta_ned[1] * scale,
            ]
        return [
            self._pos_ned[0] + goal_delta_ned[0],
            self._pos_ned[1] + goal_delta_ned[1],
            -self._hover_alt,
        ]

    def _goal_inspection_altitude_m(self) -> float:
        if self._goal_enu is None:
            return self._inspection_altitude_m
        goal_altitude_m = float(self._goal_enu[2])
        if goal_altitude_m > 0.0:
            return goal_altitude_m
        return self._inspection_altitude_m

    def _goal_requests_tag_scan(self) -> bool:
        if self._goal_enu is None:
            return False
        return float(self._goal_enu[2]) > 0.0

    def _inspection_yaw(self, now_s: float) -> float:
        if self._inspect_started_s is None or self._goal_scan_duration_s <= 0.0:
            return _NAN
        progress = max(
            0.0,
            min(1.0, (now_s - self._inspect_started_s) / self._goal_scan_duration_s),
        )
        return -math.pi + (2.0 * math.pi * progress)



def main():
    rclpy.init()
    node = OffboardControllerNode()
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
