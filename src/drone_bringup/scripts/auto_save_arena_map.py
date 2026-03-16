#!/usr/bin/env python3
"""
Save the explored arena map automatically when /mission_complete is published.
"""

import argparse
from datetime import datetime
import os
import signal

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

from save_arena_map import MapSaverNode


def _default_output_path() -> str:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.expanduser(f'~/.ros/maps/arena_map_{stamp}.png')


class MissionCompleteMapSaver(MapSaverNode):
    def __init__(self, output_path: str, save_delay_s: float, exit_after_save: bool):
        super().__init__(output_path)
        self._save_delay_s = max(0.0, save_delay_s)
        self._exit_after_save = exit_after_save
        self._save_requested = False
        self._done = False
        self._save_timer = None

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, '/mission_complete', self._mission_complete_cb, latch_qos)
        self.get_logger().info(
            f'Waiting for /mission_complete to save map → {self._output_path}')

    @property
    def done(self) -> bool:
        return self._done

    def _mission_complete_cb(self, msg: Bool):
        if not msg.data or self._save_requested:
            return
        self._save_requested = True
        self.get_logger().info(
            f'Mission complete received — saving map in {self._save_delay_s:.1f}s')
        self._save_timer = self.create_timer(self._save_delay_s, self._save_once)

    def _save_once(self):
        if self._save_timer is not None:
            self._save_timer.cancel()
            self._save_timer = None
        self.save()
        if self._exit_after_save:
            self._done = True


def main():
    parser = argparse.ArgumentParser(description='Auto-save arena map on mission completion')
    parser.add_argument(
        '--out',
        default='',
        help='Output PNG path (default: ~/.ros/maps/arena_map_<timestamp>.png)')
    parser.add_argument(
        '--save-delay',
        type=float,
        default=5.0,
        help='Seconds to wait after /mission_complete before saving')
    parser.add_argument(
        '--keep-running',
        action='store_true',
        help='Stay alive after saving instead of exiting')
    args = parser.parse_args()

    output_path = args.out or _default_output_path()

    rclpy.init()
    node = MissionCompleteMapSaver(
        output_path=output_path,
        save_delay_s=args.save_delay,
        exit_after_save=not args.keep_running,
    )
    shutdown_requested = False

    def _handle_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, _handle_shutdown)
    signal.signal(signal.SIGTERM, _handle_shutdown)

    try:
        while rclpy.ok() and not node.done and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.5)
        if shutdown_requested and not node.done:
            node.get_logger().info('Shutdown signal received — saving current map before exit')
            node.save()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C — saving current map before exit')
        node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
