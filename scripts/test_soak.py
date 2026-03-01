#!/usr/bin/env python3
"""
test_soak.py

Sustained-load test for the exploration stack.

Runs exploration_planner + drone_coordinator for DURATION_S (default 300 s =
5 min) while a synthetic "drone" continuously explores one frontier after
another.  Checks that:

  • Neither subprocess crashes at any point
  • The coordinator keeps assigning goals (no silent stall > STALL_TIMEOUT_S)
  • Every goal_pose assignment is followed by an idle state within CYCLE_TIMEOUT_S
  • The /d1/drone_state publisher keeps arriving (no DDS dropout > DROPOUT_TIMEOUT_S)

Usage (source workspace first)
  python3 scripts/test_soak.py              # 5-minute run
  python3 scripts/test_soak.py --duration 60   # quick 1-minute run
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from drone_interfaces.msg import FrontierList, DroneState

_GREEN  = '\033[32m'
_RED    = '\033[31m'
_YELLOW = '\033[33m'
_RESET  = '\033[0m'

PASS = f'{_GREEN}PASS{_RESET}'
FAIL = f'{_RED}FAIL{_RESET}'
INFO = f'{_YELLOW}INFO{_RESET}'

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_DURATION_S  = 300   # 5 minutes
CYCLE_TIMEOUT_S     = 15.0  # max time for one full assign → reach → idle cycle
STALL_TIMEOUT_S     = 15.0  # coordinator must assign a goal within this long
DROPOUT_TIMEOUT_S   = 10.0  # drone_state must arrive at least this often
NODE_STARTUP_S      = 4.0

# Frontier positions cycled through (x, y)
FRONTIER_WAYPOINTS = [
    ( 5.0,  3.0),
    ( 8.0, -2.0),
    (-3.0,  6.0),
    ( 1.0, -5.0),
    ( 6.0,  7.0),
    (-4.0, -3.0),
]


# ── Monitor node ──────────────────────────────────────────────────────────────
class SoakMonitorNode(Node):
    def __init__(self):
        super().__init__('test_soak')

        _be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_odom     = self.create_publisher(
            Odometry, '/d1/lio_sam/mapping/odometry', _be)
        self._pub_frontier = self.create_publisher(
            FrontierList, '/d1/frontiers/list', 10)

        self.last_goal_time   = None   # time of last /d1/goal_pose
        self.last_state_time  = None   # time of last /d1/drone_state
        self.last_status      = None
        self.cycles_completed = 0
        self.errors: list[str] = []
        self._lock = threading.Lock()

        self.create_subscription(Point,      '/d1/goal_pose',   self._goal_cb,  10)
        self.create_subscription(DroneState, '/d1/drone_state', self._state_cb, 10)

    def _goal_cb(self, msg: Point):
        with self._lock:
            self.last_goal_time = time.monotonic()

    def _state_cb(self, msg: DroneState):
        with self._lock:
            now = time.monotonic()
            if self.last_status == 'exploring' and msg.status == 'idle':
                self.cycles_completed += 1
            self.last_status     = msg.status
            self.last_state_time = now

    def publish_frontier(self, x: float, y: float, size: float = 25.0):
        msg = FrontierList()
        msg.header.frame_id = 'd1/map'
        msg.header.stamp = self.get_clock().now().to_msg()
        p = Point(); p.x = x; p.y = y; p.z = 0.0
        msg.centroids.append(p)
        msg.sizes.append(size)
        self._pub_frontier.publish(msg)

    def publish_odom(self, x: float, y: float):
        msg = Odometry()
        msg.header.frame_id = 'd1/map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 3.0
        msg.pose.pose.orientation.w = 1.0
        self._pub_odom.publish(msg)


def start_node(package: str, executable: str, *extra_args) -> subprocess.Popen:
    cmd = ['ros2', 'run', package, executable, '--ros-args'] + list(extra_args)
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def kill_proc(p: subprocess.Popen):
    try:
        p.kill()
    except ProcessLookupError:
        pass
    try:
        p.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass


def purge_nodes(*names: str):
    for name in names:
        subprocess.run(['pkill', '-9', '-f', name], capture_output=True)
    time.sleep(0.5)


# ── Soak driver thread ────────────────────────────────────────────────────────
def drive_exploration(node: SoakMonitorNode, stop_event: threading.Event,
                      stall_timeout: float, cycle_timeout: float):
    """
    Continuously cycles through FRONTIER_WAYPOINTS, simulating the drone
    flying to each assigned goal.  Records errors in node.errors.
    """
    wp_idx = 0

    # Wait until the planner is publishing drone_state
    t0 = time.monotonic()
    while node.last_state_time is None:
        if time.monotonic() - t0 > NODE_STARTUP_S * 2:
            node.errors.append('Timed out waiting for first drone_state message')
            stop_event.set()
            return
        time.sleep(0.1)

    while not stop_event.is_set():
        wx, wy = FRONTIER_WAYPOINTS[wp_idx % len(FRONTIER_WAYPOINTS)]
        wp_idx += 1

        # ── Publish frontier, wait for goal assignment ────────────────────────
        goal_before = node.last_goal_time
        stall_start = time.monotonic()
        while not stop_event.is_set():
            node.publish_frontier(wx, wy)
            time.sleep(0.3)
            if node.last_goal_time != goal_before:
                break
            if time.monotonic() - stall_start > stall_timeout:
                node.errors.append(
                    f'STALL: no goal assigned within {stall_timeout}s '
                    f'(cycle {node.cycles_completed + 1})')
                stop_event.set()
                return

        if stop_event.is_set():
            break

        # ── Simulate drone arriving at the waypoint ───────────────────────────
        # Guard: wait for planner to acknowledge the assignment (status →
        # 'exploring') before watching for the return to 'idle'.  Without this
        # the leftover 'idle' from the previous cycle fires immediately.
        seen_exploring = False
        cycle_start = time.monotonic()
        while not stop_event.is_set():
            node.publish_odom(wx, wy)
            time.sleep(0.1)
            s = node.last_status
            if s == 'exploring':
                seen_exploring = True
            if seen_exploring and s == 'idle':
                break
            if time.monotonic() - cycle_start > cycle_timeout:
                node.errors.append(
                    f'TIMEOUT: drone did not go idle within {cycle_timeout}s '
                    f'after reaching waypoint ({wx:.1f}, {wy:.1f}), '
                    f'cycle {node.cycles_completed + 1}')
                stop_event.set()
                return


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Soak test for exploration stack')
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION_S,
                        help='Duration in seconds (default: 300)')
    args = parser.parse_args()

    print('\n══════════════════════════════════════════════')
    print('  test_soak.py')
    print(f'  Duration: {args.duration}s')
    print('══════════════════════════════════════════════\n')

    purge_nodes('exploration_planner', 'drone_coordinator')

    rclpy.init()
    node = SoakMonitorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    procs = []
    stop_event = threading.Event()

    try:
        # ── Start nodes ───────────────────────────────────────────────────────
        print(f'[{INFO}] Starting exploration_planner and drone_coordinator …')
        procs.append(start_node(
            'exploration_manager', 'exploration_planner',
            '-p', 'drone_ns:=d1',
            '-p', 'goal_radius:=0.8',
        ))
        procs.append(start_node(
            'exploration_manager', 'drone_coordinator',
            '-p', 'drone_namespaces:=[d1]',
            '-p', 'safety_radius:=0.5',
            '-p', 'no_frontier_timeout:=60.0',
            '-p', 'low_battery_threshold:=5.0',
        ))
        time.sleep(NODE_STARTUP_S)

        if not all(p.poll() is None for p in procs):
            print(f'[{FAIL}] A node crashed during startup.')
            sys.exit(1)

        # ── Start driver thread ───────────────────────────────────────────────
        driver = threading.Thread(
            target=drive_exploration,
            args=(node, stop_event, STALL_TIMEOUT_S, CYCLE_TIMEOUT_S),
            daemon=True,
        )
        driver.start()

        # ── Main monitoring loop ──────────────────────────────────────────────
        t_start = time.monotonic()
        t_report = t_start
        while time.monotonic() - t_start < args.duration and not stop_event.is_set():
            now = time.monotonic()

            # Check for subprocess crashes
            for p in procs:
                if p.poll() is not None:
                    node.errors.append(f'Subprocess {p.args[3]} exited unexpectedly '
                                       f'(returncode={p.returncode})')
                    stop_event.set()

            # Check for drone_state dropout
            if (node.last_state_time is not None and
                    now - node.last_state_time > DROPOUT_TIMEOUT_S):
                node.errors.append(
                    f'DDS DROPOUT: no drone_state for {DROPOUT_TIMEOUT_S}s')
                stop_event.set()

            # Progress report every 30 s
            if now - t_report >= 30:
                elapsed = now - t_start
                print(f'[{INFO}] {elapsed:.0f}s elapsed — '
                      f'{node.cycles_completed} cycles complete — '
                      f'no errors so far')
                t_report = now

            time.sleep(1.0)

        elapsed = time.monotonic() - t_start
        stop_event.set()
        driver.join(timeout=3)

        # ── Results ───────────────────────────────────────────────────────────
        print(f'\n──────────────────────────────────────────────')
        print(f'  Duration run:     {elapsed:.1f}s')
        print(f'  Cycles completed: {node.cycles_completed}')
        print(f'  Errors:           {len(node.errors)}')
        print(f'──────────────────────────────────────────────')

        crashed = [p for p in procs if p.poll() is not None]

        if node.errors:
            for err in node.errors:
                print(f'  [{FAIL}] {err}')
            print(f'\n[{FAIL}] Soak test FAILED after {elapsed:.1f}s\n')
            sys.exit(1)

        if crashed:
            print(f'[{FAIL}] {len(crashed)} subprocess(es) crashed.')
            sys.exit(1)

        if node.cycles_completed == 0:
            print(f'[{FAIL}] No exploration cycles completed.')
            sys.exit(1)

        print(f'[{PASS}] Soak test passed — '
              f'{node.cycles_completed} cycles in {elapsed:.1f}s, no errors.\n')

    finally:
        stop_event.set()
        for p in procs:
            kill_proc(p)
        purge_nodes('exploration_planner', 'drone_coordinator')
        rclpy.shutdown()
        spin_thread.join(timeout=3)


if __name__ == '__main__':
    main()
