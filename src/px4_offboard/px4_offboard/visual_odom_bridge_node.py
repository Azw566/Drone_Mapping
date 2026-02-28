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
  Apply rotation q_R = [w=0, x=√2/2, y=√2/2, z=0] (Hamilton)
  q_NED = q_R ⊗ q_ENU
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

# √2 / 2
_SQRT2_2 = math.sqrt(2.0) / 2.0

# Quaternion (w,x,y,z) representing a 180° rotation around the axis (1/√2, 1/√2, 0)
# — this is the frame rotation from ENU to NED.
_QR_W = 0.0
_QR_X = _SQRT2_2
_QR_Y = _SQRT2_2
_QR_Z = 0.0

_NAN = float('nan')


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

        self.pub = self.create_publisher(VehicleOdometry, vio_topic, px4_qos)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._cb, lio_qos)

        self.get_logger().info(
            f'[{ns}] VisualOdomBridge: {odom_topic} → {vio_topic}')

    def _cb(self, msg: Odometry):
        # ── Position ENU → NED ───────────────────────────────────────────────
        ex = msg.pose.pose.position.x
        ey = msg.pose.pose.position.y
        ez = msg.pose.pose.position.z

        # ── Orientation ENU (ROS x,y,z,w) → NED (Hamiltonian w,x,y,z) ──────
        rx = msg.pose.pose.orientation.x
        ry = msg.pose.pose.orientation.y
        rz = msg.pose.pose.orientation.z
        rw = msg.pose.pose.orientation.w

        # q_NED = q_R ⊗ q_ENU  (both in Hamiltonian w,x,y,z)
        nw, nx, ny, nz = _hamilton(_QR_W, _QR_X, _QR_Y, _QR_Z,
                                    rw,     rx,     ry,     rz)

        # ── Linear velocity ENU → NED ────────────────────────────────────────
        vx_e = msg.twist.twist.linear.x
        vy_e = msg.twist.twist.linear.y
        vz_e = msg.twist.twist.linear.z

        out = VehicleOdometry()
        out.timestamp        = self.get_clock().now().nanoseconds // 1000
        out.timestamp_sample = out.timestamp
        out.pose_frame       = VehicleOdometry.POSE_FRAME_NED

        out.position = [ ey,  ex, -ez]   # NED from ENU
        out.q        = [nw, nx, ny, nz]  # Hamiltonian (w,x,y,z)

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity       = [ vy_e,  vx_e, -vz_e]

        # Leave variances at NaN so PX4 uses its own tuning
        out.position_variance    = [_NAN, _NAN, _NAN]
        out.orientation_variance = [_NAN, _NAN, _NAN]
        out.velocity_variance    = [_NAN, _NAN, _NAN]

        self.pub.publish(out)


def main():
    rclpy.init()
    node = VisualOdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
