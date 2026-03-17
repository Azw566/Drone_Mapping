#!/usr/bin/env python3
"""
px4_health_monitor.py

Compact PX4 health logger for one drone namespace. This is meant for SITL
debugging inside launch files where the normal `ros2 topic echo` path is
awkward because PX4 uses BEST_EFFORT QoS and loopback-only DDS settings.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    BatteryStatus,
    EstimatorAidSource1d,
    EstimatorAidSource2d,
    EstimatorAidSource3d,
    EstimatorStatusFlags,
    FailsafeFlags,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Px4HealthMonitor(Node):
    def __init__(self):
        super().__init__('px4_health_monitor')
        self.declare_parameter('drone_ns', 'd1')
        self._ns = self.get_parameter('drone_ns').get_parameter_value().string_value

        self._vlp = None
        self._est = None
        self._failsafe = None
        self._status = None
        self._battery = None
        self._vio_in = None
        self._vio_in_count = 0
        self._aid_ev_pos = None
        self._aid_ev_vel = None
        self._aid_ev_yaw = None
        self._last_summary = None

        self.create_subscription(
            VehicleLocalPosition,
            f'/{self._ns}/fmu/out/vehicle_local_position',
            self._vlp_cb,
            PX4_QOS,
        )
        self.create_subscription(
            EstimatorStatusFlags,
            f'/{self._ns}/fmu/out/estimator_status_flags',
            self._est_cb,
            PX4_QOS,
        )
        self.create_subscription(
            FailsafeFlags,
            f'/{self._ns}/fmu/out/failsafe_flags',
            self._failsafe_cb,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            f'/{self._ns}/fmu/out/vehicle_status_v1',
            self._status_cb,
            PX4_QOS,
        )
        self.create_subscription(
            BatteryStatus,
            f'/{self._ns}/fmu/out/battery_status',
            self._battery_cb,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleOdometry,
            f'/{self._ns}/fmu/in/vehicle_visual_odometry',
            self._vio_in_cb,
            PX4_QOS,
        )
        self.create_subscription(
            EstimatorAidSource2d,
            f'/{self._ns}/fmu/out/estimator_aid_src_ev_pos',
            self._aid_ev_pos_cb,
            PX4_QOS,
        )
        self.create_subscription(
            EstimatorAidSource3d,
            f'/{self._ns}/fmu/out/estimator_aid_src_ev_vel',
            self._aid_ev_vel_cb,
            PX4_QOS,
        )
        self.create_subscription(
            EstimatorAidSource1d,
            f'/{self._ns}/fmu/out/estimator_aid_src_ev_yaw',
            self._aid_ev_yaw_cb,
            PX4_QOS,
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'[{self._ns}] PX4 health monitor ready')

    def _vlp_cb(self, msg: VehicleLocalPosition):
        self._vlp = msg

    def _est_cb(self, msg: EstimatorStatusFlags):
        self._est = msg

    def _failsafe_cb(self, msg: FailsafeFlags):
        self._failsafe = msg

    def _status_cb(self, msg: VehicleStatus):
        self._status = msg

    def _battery_cb(self, msg: BatteryStatus):
        self._battery = msg

    def _vio_in_cb(self, msg: VehicleOdometry):
        self._vio_in = msg
        self._vio_in_count += 1

    def _aid_ev_pos_cb(self, msg: EstimatorAidSource2d):
        self._aid_ev_pos = msg

    def _aid_ev_vel_cb(self, msg: EstimatorAidSource3d):
        self._aid_ev_vel = msg

    def _aid_ev_yaw_cb(self, msg: EstimatorAidSource1d):
        self._aid_ev_yaw = msg

    def _tick(self):
        summary = self._build_summary()
        if summary != self._last_summary:
            self._last_summary = summary
            self.get_logger().info(summary)

    def _build_summary(self) -> str:
        if self._vlp is None:
            return f'[{self._ns}] waiting for vehicle_local_position'

        pos = (
            self._fmt(self._vlp.x),
            self._fmt(self._vlp.y),
            self._fmt(self._vlp.z),
        )
        status_bits = [
            f'xy={int(bool(self._vlp.xy_valid))}',
            f'z={int(bool(self._vlp.z_valid))}',
            f'vxy={int(bool(self._vlp.v_xy_valid))}',
            f'vz={int(bool(self._vlp.v_z_valid))}',
            f'heading={int(bool(self._vlp.heading_good_for_control))}',
            f'eph={self._fmt(self._vlp.eph)}',
            f'evh={self._fmt(self._vlp.evh)}',
            f'pos=({pos[0]},{pos[1]},{pos[2]})',
        ]

        if self._status is not None:
            status_bits.extend([
                f'nav={self._status.nav_state}',
                f'arm={self._status.arming_state}',
            ])

        if self._est is not None:
            status_bits.extend([
                f'tilt_align={int(bool(getattr(self._est, "cs_tilt_align", False)))}',
                f'ev_pos={int(bool(getattr(self._est, "cs_ev_pos", False)))}',
                f'ev_vel={int(bool(getattr(self._est, "cs_ev_vel", False)))}',
                f'ev_yaw={int(bool(getattr(self._est, "cs_ev_yaw", False)))}',
                f'ev_hgt={int(bool(getattr(self._est, "cs_ev_hgt", False)))}',
                f'baro={int(bool(getattr(self._est, "cs_baro_hgt", False)))}',
                f'mag={int(bool(getattr(self._est, "cs_mag_hdg", False)))}',
                f'yaw_align={int(bool(getattr(self._est, "cs_yaw_align", False)))}',
            ])

        if self._vio_in is not None:
            status_bits.extend([
                f'vio_in={self._vio_in_count}',
                f'vio_ts={self._vio_in.timestamp}',
                f'vio_sample={self._vio_in.timestamp_sample}',
            ])

        if self._aid_ev_pos is not None:
            status_bits.extend([
                f'evpos_fused={int(bool(self._aid_ev_pos.fused))}',
                f'evpos_rej={int(bool(self._aid_ev_pos.innovation_rejected))}',
                f'evpos_tlast={self._aid_ev_pos.time_last_fuse}',
            ])

        if self._aid_ev_vel is not None:
            status_bits.extend([
                f'evvel_fused={int(bool(self._aid_ev_vel.fused))}',
                f'evvel_rej={int(bool(self._aid_ev_vel.innovation_rejected))}',
                f'evvel_tlast={self._aid_ev_vel.time_last_fuse}',
            ])

        if self._aid_ev_yaw is not None:
            status_bits.extend([
                f'evyaw_fused={int(bool(self._aid_ev_yaw.fused))}',
                f'evyaw_rej={int(bool(self._aid_ev_yaw.innovation_rejected))}',
                f'evyaw_tlast={self._aid_ev_yaw.time_last_fuse}',
            ])

        if self._failsafe is not None:
            status_bits.extend([
                f'fs_loc={int(bool(self._failsafe.local_position_invalid))}',
                f'fs_vel={int(bool(self._failsafe.local_velocity_invalid))}',
                f'fs_alt={int(bool(self._failsafe.local_altitude_invalid))}',
                f'fs_gcs={int(bool(self._failsafe.gcs_connection_lost))}',
                f'fs_offb={int(bool(self._failsafe.offboard_control_signal_lost))}',
                f'fs_home={int(bool(self._failsafe.home_position_invalid))}',
            ])

        if self._battery is not None:
            remaining = getattr(self._battery, 'remaining', math.nan)
            warning = getattr(self._battery, 'warning', -1)
            status_bits.extend([
                f'bat={self._fmt(remaining)}',
                f'bat_warn={warning}',
            ])

        return f'[{self._ns}] ' + '  '.join(status_bits)

    @staticmethod
    def _fmt(value) -> str:
        if isinstance(value, float) and not math.isfinite(value):
            return 'nan'
        if isinstance(value, float):
            return f'{value:.2f}'
        return str(value)


def main():
    rclpy.init()
    node = Px4HealthMonitor()
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
