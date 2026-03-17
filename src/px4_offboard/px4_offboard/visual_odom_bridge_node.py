"""
visual_odom_bridge_node.py

Converts LIO-SAM nav_msgs/Odometry (ENU / ROS convention) into
px4_msgs/VehicleOdometry (NED / Hamiltonian quaternion) and publishes it
to the uXRCE-DDS topic that PX4's EKF2 listens on.

Frame conventions
  LIO-SAM output  : ENU  — x=East, y=North, z=Up   ; quaternion (x,y,z,w) ROS
  PX4 input       : NED  — x=North, y=East,  z=Down ; quaternion (w,x,y,z) Hamiltonian

ENU → NED position:
  ned_x =  enu_y   (North  = ENU-y)
  ned_y =  enu_x   (East   = ENU-x)
  ned_z = -enu_z   (Down   = -ENU-z)

ENU → NED quaternion:
  Apply the world-frame rotation q_R = [w=0, x=√2/2, y=√2/2, z=0].
  q_NED = q_R ⊗ q_ENU
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition

# √2 / 2
_SQRT2_2 = math.sqrt(2.0) / 2.0

# Quaternion (w,x,y,z) representing a 180° rotation around the axis (1/√2, 1/√2, 0)
# — this is the frame rotation from ENU to NED.
_QR_W = 0.0
_QR_X = _SQRT2_2
_QR_Y = _SQRT2_2
_QR_Z = 0.0

_NAN = float('nan')
_DEBUG_SAMPLES = 5


def _hamilton(q1w, q1x, q1y, q1z, q2w, q2x, q2y, q2z):
    """Hamilton product q1 ⊗ q2, returns (w, x, y, z)."""
    return (
        q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z,
        q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y,
        q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x,
        q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w,
    )


class VisualOdomBridgeNode(Node):
    def __init__(self):
        super().__init__('visual_odom_bridge')

        self.declare_parameter('drone_ns', 'd1')
        ns = self.get_parameter('drone_ns').get_parameter_value().string_value

        odom_topic  = f'/{ns}/lio_sam/mapping/odometry'
        vio_topic   = f'/{ns}/fmu/in/vehicle_visual_odometry'

        # PX4 uses BEST_EFFORT / VOLATILE for all fmu topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # LIO-SAM publishes odometry with BEST_EFFORT — match it
        lio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._px4_us = 0   # latest PX4 clock value (µs), updated from vehicle_local_position
        self._msg_count = 0
        self._debug_count = 0

        self.pub = self.create_publisher(VehicleOdometry, vio_topic, px4_qos)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._cb, lio_qos)
        self.create_subscription(
            VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position',
            self._lp_cb, px4_qos)

        self.get_logger().info(
            f'[{ns}] VisualOdomBridge: {odom_topic} → {vio_topic}')

    def _lp_cb(self, msg: VehicleLocalPosition):
        """Track PX4's internal clock from vehicle_local_position timestamps."""
        self._px4_us = msg.timestamp

    def _now_us(self) -> int:
        """Return a timestamp in PX4 microseconds.

        Prefer PX4's own clock (read from VehicleLocalPosition) to avoid
        stale-message rejection caused by Gazebo/ROS2 clock drift under load.
        Falls back to the ROS2 clock before the first VehicleLocalPosition arrives.
        """
        if self._px4_us > 0:
            return self._px4_us
        return self.get_clock().now().nanoseconds // 1000

    def _cb(self, msg: Odometry):
        self._msg_count += 1
        if self._msg_count == 1 or self._msg_count % 50 == 0:
            self.get_logger().info(
                f'[{self.get_parameter("drone_ns").get_parameter_value().string_value}] '
                f'VIO cb #{self._msg_count}, px4_us={self._px4_us}')

        # ── Position ENU → NED ───────────────────────────────────────────────
        ex = msg.pose.pose.position.x
        ey = msg.pose.pose.position.y
        ez = msg.pose.pose.position.z

        # ── Orientation ENU (ROS x,y,z,w) → NED (Hamiltonian w,x,y,z) ──────
        rx = msg.pose.pose.orientation.x
        ry = msg.pose.pose.orientation.y
        rz = msg.pose.pose.orientation.z
        rw = msg.pose.pose.orientation.w

        # The simpler ENU→NED rotation is the last known-good path for PX4
        # EKF2 fusion in this repo. The extra FLU→FRD body flip regressed
        # horizontal validity during bootstrap.
        nw, nx, ny, nz = _hamilton(
            _QR_W, _QR_X, _QR_Y, _QR_Z,
            rw, rx, ry, rz,
        )

        # ── Linear velocity ENU → NED ────────────────────────────────────────
        vx_e = msg.twist.twist.linear.x
        vy_e = msg.twist.twist.linear.y
        vz_e = msg.twist.twist.linear.z

        out = VehicleOdometry()
        out.timestamp        = self._now_us()
        out.timestamp_sample = out.timestamp
        out.pose_frame       = VehicleOdometry.POSE_FRAME_NED

        out.position = [ ey,  ex, -ez]   # NED from ENU
        out.q        = [nw, nx, ny, nz]  # Hamiltonian (w,x,y,z)

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity       = [ vy_e,  vx_e, -vz_e]

        # Keep PX4's own noise tuning, but advertise that the odometry is valid.
        out.position_variance    = [_NAN, _NAN, _NAN]
        out.orientation_variance = [_NAN, _NAN, _NAN]
        out.velocity_variance    = [_NAN, _NAN, _NAN]
        out.reset_counter        = 0
        out.quality              = 100

        self.pub.publish(out)
        if self._debug_count < _DEBUG_SAMPLES:
            self._debug_count += 1
            self.get_logger().info(
                f'[{self.get_parameter("drone_ns").get_parameter_value().string_value}] '
                f'pub VIO#{self._debug_count} ts={out.timestamp} sample={out.timestamp_sample} '
                f'pose_frame={out.pose_frame} vel_frame={out.velocity_frame} '
                f'pos={list(out.position)} vel={list(out.velocity)} q={list(out.q)} '
                f'pos_var={list(out.position_variance)} '
                f'ori_var={list(out.orientation_variance)} '
                f'vel_var={list(out.velocity_variance)} quality={out.quality}'
            )


def main():
    rclpy.init()
    node = VisualOdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if 'Unable to convert call argument to Python object' not in str(exc):
            raise
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
