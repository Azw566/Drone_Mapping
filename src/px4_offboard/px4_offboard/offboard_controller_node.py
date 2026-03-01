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
_LANDING     = 'landing'

# Config
HOVER_ALT_M      = 3.0    # metres above takeoff (NED z = -HOVER_ALT_M)
GOAL_RADIUS_M    = 0.5    # consider goal reached when within this distance
PRE_ARM_SECS     = 2.0    # seconds of publishing before arming
TAKEOFF_VEL_MS   = 1.5    # vertical climb speed during TAKING_OFF (m/s, NED -z)


class OffboardControllerNode(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        self.declare_parameter('drone_ns', 'd1')
        self.declare_parameter('hover_alt', HOVER_ALT_M)

        ns   = self.get_parameter('drone_ns').get_parameter_value().string_value
        self._ns = ns
        self._hover_alt = self.get_parameter('hover_alt').get_parameter_value().double_value

        # PX4 QoS
        px4_qos = QoSProfile(
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
            Point, f'/{ns}/goal_pose', self._goal_cb, 10)
        self.create_subscription(
            Empty, f'/{ns}/cmd/land', self._land_cb, 10)

        # ── State ────────────────────────────────────────────────────────────
        self._state      = _IDLE
        self._nav_state  = -1
        self._arm_state  = 1   # 1 = DISARMED
        self._pos_ned    = [0.0, 0.0, 0.0]   # current NED position from PX4
        self._home_ned   = None               # NED position at first position fix
        self._goal_enu   = None               # latest goal in ENU map frame
        self._goal_ned   = None               # goal converted to NED
        self._pre_arm_start = None
        self._arm_last_attempt = None         # last time arm command was sent
        self._px4_us     = 0                  # latest PX4 timestamp (µs) from VehicleLocalPosition
        # ── Main control timer (10 Hz) ───────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(f'[{ns}] OffboardController ready (hover={self._hover_alt} m)')

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

    def _land_cb(self, msg: Empty):
        if self._state != _IDLE:
            self.get_logger().info(f'[{self._ns}] Land command → LANDING')
            self._state = _LANDING

    def _goal_cb(self, msg: Point):
        """Accept a new goal in ENU map frame."""
        self._goal_enu = [msg.x, msg.y, msg.z]
        # Convert ENU → NED
        self._goal_ned = [msg.y, msg.x, -self._hover_alt]
        if self._state == _HOVER:
            self._state = _EXPLORING
            self.get_logger().info(
                f'[{self._ns}] New goal ENU({msg.x:.1f},{msg.y:.1f}) '
                f'→ NED({self._goal_ned[0]:.1f},{self._goal_ned[1]:.1f})')

    # ── Control loop ──────────────────────────────────────────────────────
    def _control_loop(self):
        now = self.get_clock().now()

        if self._state == _IDLE:
            if self._home_ned is not None:
                self._state = _PRE_ARM
                self._pre_arm_start = now
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
            if self._goal_ned is not None and self._at_goal():
                self._state = _HOVER
                self._goal_ned = None
                self._goal_enu = None
                self.get_logger().info(f'[{self._ns}] Goal reached → HOVER')

        elif self._state == _LANDING:
            self._publish_ocm(velocity_mode=True)
            self._publish_landing_setpoint()
            if self._pos_ned[2] > -0.3:   # within 30 cm of ground (NED z → 0)
                self._send_disarm_command()
                self._state    = _IDLE
                self._goal_ned = None
                self._goal_enu = None
                self.get_logger().info(f'[{self._ns}] Landed → IDLE')

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
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(self._goal_ned[0]),
                        float(self._goal_ned[1]),
                        float(self._goal_ned[2])]
        msg.yaw = self._yaw_to_goal()
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
        return abs(self._pos_ned[2] - (-self._hover_alt)) < 0.3

    def _at_goal(self) -> bool:
        if self._goal_ned is None:
            return False
        dx = self._pos_ned[0] - self._goal_ned[0]
        dy = self._pos_ned[1] - self._goal_ned[1]
        return math.sqrt(dx*dx + dy*dy) < GOAL_RADIUS_M

    def _yaw_to_goal(self) -> float:
        if self._goal_ned is None:
            return 0.0
        dx = self._goal_ned[0] - self._pos_ned[0]
        dy = self._goal_ned[1] - self._pos_ned[1]
        return math.atan2(dy, dx)



def main():
    rclpy.init()
    node = OffboardControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
