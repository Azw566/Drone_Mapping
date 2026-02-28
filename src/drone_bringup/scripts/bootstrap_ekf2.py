#!/usr/bin/env python3
"""
bootstrap_ekf2.py

Publishes synthetic VehicleOdometry at origin (NED) to both drones for
DURATION_S seconds so that PX4 EKF2 (EKF2_EV_CTRL=15, vision-only) can
initialise and set xy_valid=true.

Once EKF2 has converged the offboard controller will detect xy_valid=true,
set home_ned, and proceed to PRE_ARM → ARM → OFFBOARD → TAKEOFF automatically.
Real LIO-SAM data from the visual_odom_bridge will take over seamlessly.

Usage:
  python3 bootstrap_ekf2.py [duration_seconds]   default 15
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import VehicleOdometry

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
RATE_HZ    = 30.0


class EkfBootstrapper(Node):
    def __init__(self):
        super().__init__('ekf2_bootstrapper')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pubs = [
            self.create_publisher(VehicleOdometry,
                                  '/d1/fmu/in/vehicle_visual_odometry', px4_qos),
            self.create_publisher(VehicleOdometry,
                                  '/d2/fmu/in/vehicle_visual_odometry', px4_qos),
        ]

        self._start   = self.get_clock().now()
        self._timer   = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f'EKF2 bootstrap: publishing synthetic odometry for {DURATION_S:.0f} s …')

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if elapsed >= DURATION_S:
            self.get_logger().info('Bootstrap done — shutting down.')
            self._timer.cancel()
            rclpy.shutdown()
            return

        ts = self.get_clock().now().nanoseconds // 1000

        msg = VehicleOdometry()
        msg.timestamp        = ts
        msg.timestamp_sample = ts
        msg.pose_frame       = VehicleOdometry.POSE_FRAME_NED

        # Identity pose at origin
        msg.position = [0.0, 0.0, 0.0]
        msg.q        = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z

        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity       = [0.0, 0.0, 0.0]

        # Tight variances so EKF trusts this measurement
        msg.position_variance    = [0.1, 0.1, 0.1]
        msg.orientation_variance = [0.1, 0.1, 0.1]
        msg.velocity_variance    = [0.1, 0.1, 0.1]

        for pub in self._pubs:
            pub.publish(msg)


def main():
    rclpy.init()
    node = EkfBootstrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
