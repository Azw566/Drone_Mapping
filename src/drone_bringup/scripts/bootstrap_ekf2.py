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
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
RATE_HZ    = 30.0
DEBUG_SAMPLES = 5
class EkfBootstrapper(Node):
    def __init__(self):
        super().__init__('ekf2_bootstrapper')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._px4_us = {'d1': 0, 'd2': 0}
        self._ready = {'d1': False, 'd2': False}
        self._pubs = {}
        self._debug_count = {'d1': 0, 'd2': 0}
        for ns in ('d1', 'd2'):
            self._pubs[ns] = self.create_publisher(
                VehicleOdometry, f'/{ns}/fmu/in/vehicle_visual_odometry', px4_qos)
            self.create_subscription(
                VehicleLocalPosition,
                f'/{ns}/fmu/out/vehicle_local_position',
                lambda msg, drone_ns=ns: self._local_pos_cb(drone_ns, msg),
                px4_qos,
            )

        self._start   = self.get_clock().now()
        self._timer   = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f'EKF2 bootstrap: publishing synthetic odometry for {DURATION_S:.0f} s …')

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if all(self._ready.values()):
            self.get_logger().info('Bootstrap done — both PX4 local positions are valid.')
            self._timer.cancel()
            rclpy.shutdown()
            return

        if elapsed >= DURATION_S:
            self.get_logger().info(
                'Bootstrap timed out — shutting down with ready='
                f'{self._ready}')
            self._timer.cancel()
            rclpy.shutdown()
            return
        ros_ts = self.get_clock().now().nanoseconds // 1000
        for ns, pub in self._pubs.items():
            ts = self._px4_us[ns] if self._px4_us[ns] > 0 else ros_ts

            msg = VehicleOdometry()
            msg.timestamp        = ts
            msg.timestamp_sample = ts
            msg.pose_frame       = VehicleOdometry.POSE_FRAME_NED

            # Use the same neutral heading as the last known-good bootstrap.
            # The bridge will hand over to live LIO-SAM orientation once EV
            # fusion is up; a synthetic yaw offset here can prevent EKF2 from
            # ever declaring heading good on the ground.
            msg.position = [0.0, 0.0, 0.0]
            msg.q        = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z

            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            msg.velocity       = [0.0, 0.0, 0.0]

            # Tight variances so EKF trusts this measurement.
            msg.position_variance    = [0.1, 0.1, 0.1]
            msg.orientation_variance = [0.1, 0.1, 0.1]
            msg.velocity_variance    = [0.1, 0.1, 0.1]
            msg.reset_counter        = 0
            msg.quality              = 100

            pub.publish(msg)
            if self._debug_count[ns] < DEBUG_SAMPLES:
                self._debug_count[ns] += 1
                self.get_logger().info(
                    f'[{ns}] bootstrap VIO#{self._debug_count[ns]} '
                    f'ts={msg.timestamp} sample={msg.timestamp_sample} '
                    f'pose_frame={msg.pose_frame} vel_frame={msg.velocity_frame} '
                    f'pos={list(msg.position)} vel={list(msg.velocity)} q={list(msg.q)} '
                    f'pos_var={list(msg.position_variance)} '
                    f'ori_var={list(msg.orientation_variance)} '
                    f'vel_var={list(msg.velocity_variance)} quality={msg.quality}'
                )

    def _local_pos_cb(self, ns: str, msg: VehicleLocalPosition):
        self._px4_us[ns] = msg.timestamp
        self._ready[ns] = bool(
            msg.xy_valid and msg.v_xy_valid and msg.heading_good_for_control)


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
