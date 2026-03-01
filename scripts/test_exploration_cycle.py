#!/usr/bin/env python3
"""
test_exploration_cycle.py

Integration test for the full frontier-assign → navigate → reach-goal → idle
→ reassign cycle, without Gazebo, PX4, or LIO-SAM.

What is tested
──────────────
  1. Frontier published → coordinator assigns it → planner publishes goal_pose
  2. Simulated odometry at goal → planner detects arrival → drone_state = idle
  3. New frontier published → coordinator reassigns → second goal_pose received

Nodes started as subprocesses
  • exploration_planner  (drone_ns=d1)
  • drone_coordinator    (drone_namespaces=['d1'], safety_radius=0.5)

Mocked by this test
  • /d1/lio_sam/mapping/odometry  (nav_msgs/Odometry)
  • /d1/frontiers/list            (drone_interfaces/FrontierList)

Observed by this test
  • /d1/goal_pose                 (geometry_msgs/Point)
  • /d1/drone_state               (drone_interfaces/DroneState)

Usage (source workspace first)
  python3 scripts/test_exploration_cycle.py
"""

import os
import signal
import subprocess
import sys
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from drone_interfaces.msg import FrontierList, DroneState

# ── Colours ──────────────────────────────────────────────────────────────────
_GREEN  = '\033[32m'
_RED    = '\033[31m'
_YELLOW = '\033[33m'
_RESET  = '\033[0m'

PASS = f'{_GREEN}PASS{_RESET}'
FAIL = f'{_RED}FAIL{_RESET}'
INFO = f'{_YELLOW}INFO{_RESET}'

# ── Timeouts ─────────────────────────────────────────────────────────────────
NODE_STARTUP_S    = 4.0   # time to let subprocesses initialise
ASSIGN_TIMEOUT_S  = 8.0   # coordinator must assign within this time
IDLE_TIMEOUT_S    = 8.0   # planner must go idle after goal reached
REASSIGN_TIMEOUT_S = 8.0  # coordinator must reassign after second frontier

# Two frontier centroids used across the two assignment cycles
FRONTIER_1 = (5.0,  3.0)
FRONTIER_2 = (8.0, -2.0)


# ── Test node ─────────────────────────────────────────────────────────────────
class TestNode(Node):
    def __init__(self):
        super().__init__('test_exploration_cycle')

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

        self.goals:  list[tuple[float, float, float]] = []  # (x, y, monotonic_time)
        self.states: list[str] = []

        self.create_subscription(Point,      '/d1/goal_pose',   self._goal_cb,  10)
        self.create_subscription(DroneState, '/d1/drone_state', self._state_cb, 10)

    def _goal_cb(self, msg: Point):
        self.goals.append((msg.x, msg.y, time.monotonic()))
        self.get_logger().info(f'[test] goal_pose  → ({msg.x:.1f}, {msg.y:.1f})')

    def _state_cb(self, msg: DroneState):
        if not self.states or self.states[-1] != msg.status:
            self.states.append(msg.status)
            self.get_logger().info(f'[test] drone_state → {msg.status}')

    # ── Helpers ───────────────────────────────────────────────────────────────
    def publish_frontier(self, x: float, y: float, size: float = 25.0):
        msg = FrontierList()
        msg.header.frame_id = 'd1/map'
        msg.header.stamp = self.get_clock().now().to_msg()
        p = Point(); p.x = x; p.y = y; p.z = 0.0
        msg.centroids.append(p)
        msg.sizes.append(size)
        self._pub_frontier.publish(msg)

    def publish_odom(self, x: float, y: float, z: float = 3.0):
        msg = Odometry()
        msg.header.frame_id = 'd1/map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = 1.0
        self._pub_odom.publish(msg)


# ── Helpers ───────────────────────────────────────────────────────────────────
def wait_for(fn, timeout_s: float, interval: float = 0.05) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if fn():
            return True
        time.sleep(interval)
    return False


def assert_true(condition: bool, message: str):
    status = PASS if condition else FAIL
    print(f'  [{status}] {message}')
    if not condition:
        sys.exit(1)


def start_node(package: str, executable: str, *extra_args) -> subprocess.Popen:
    cmd = ['ros2', 'run', package, executable, '--ros-args'] + list(extra_args)
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,   # own process group → kill whole tree
    )


def kill_proc(p: subprocess.Popen):
    """Kill the ros2-run wrapper and its spawned Python child."""
    # SIGKILL the wrapper
    try:
        p.kill()
    except ProcessLookupError:
        pass
    try:
        p.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass


def purge_nodes(*names: str):
    """pkill -9 any processes whose argv contains one of the given names."""
    for name in names:
        subprocess.run(['pkill', '-9', '-f', name],
                       capture_output=True)
    time.sleep(0.5)


# ── Main test ─────────────────────────────────────────────────────────────────
def main():
    print('\n══════════════════════════════════════════════')
    print('  test_exploration_cycle.py')
    print('══════════════════════════════════════════════\n')

    # Kill any leftover nodes from previous runs before starting
    purge_nodes('exploration_planner', 'drone_coordinator')

    rclpy.init()
    node = TestNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    procs = []
    try:
        # ── Start nodes under test ────────────────────────────────────────────
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
        assert_true(all(p.poll() is None for p in procs),
                    'Both nodes are running after startup')

        # Drain any stale DDS messages that arrived during startup
        node.goals.clear()
        node.states.clear()
        time.sleep(0.5)

        # ── Cycle 1: publish frontier 1, wait for a goal near that frontier ────
        # Only count goals that arrive AFTER we start publishing (timestamp
        # filter) so DDS-buffered goals from a previous run are ignored.
        print(f'\n[{INFO}] Cycle 1 — publishing frontier at {FRONTIER_1}')
        gx = gy = None
        t_publish_start = time.monotonic()
        t0 = t_publish_start
        while time.monotonic() - t0 < ASSIGN_TIMEOUT_S:
            node.publish_frontier(*FRONTIER_1)
            time.sleep(0.3)
            for gx_, gy_, gt in node.goals:
                if gt < t_publish_start:
                    continue   # stale — ignore
                if math.sqrt((gx_ - FRONTIER_1[0])**2 +
                             (gy_ - FRONTIER_1[1])**2) < 2.0:
                    gx, gy = gx_, gy_
                    break
            if gx is not None:
                break

        assert_true(gx is not None,
                    f'goal_pose near frontier_1 received within {ASSIGN_TIMEOUT_S}s')

        # ── Simulate drone arriving at the goal ───────────────────────────────
        # Wait for 'exploring' first (planner publishes DroneState at 2 Hz, so
        # the leftover 'idle' from boot could otherwise fire immediately).
        print(f'\n[{INFO}] Simulating drone arrival at goal ({gx:.1f}, {gy:.1f})')
        seen_exploring = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < IDLE_TIMEOUT_S:
            node.publish_odom(gx, gy)
            time.sleep(0.1)
            s = node.states[-1] if node.states else ''
            if s == 'exploring':
                seen_exploring = True
            if seen_exploring and s == 'idle':
                break

        assert_true(seen_exploring,
                    'drone_state reached exploring before idle')
        assert_true(node.states and node.states[-1] == 'idle',
                    f'drone_state transitions to idle within {IDLE_TIMEOUT_S}s after reaching goal')

        # ── Cycle 2: publish frontier 2, verify a goal near that frontier ────
        print(f'\n[{INFO}] Cycle 2 — publishing frontier at {FRONTIER_2}')
        gx2 = gy2 = None
        t_cycle2_start = time.monotonic()
        t0 = t_cycle2_start
        while time.monotonic() - t0 < REASSIGN_TIMEOUT_S:
            node.publish_frontier(*FRONTIER_2)
            time.sleep(0.3)
            for gx_, gy_, gt in node.goals:
                if gt < t_cycle2_start:
                    continue   # stale — ignore
                if math.sqrt((gx_ - FRONTIER_2[0])**2 +
                             (gy_ - FRONTIER_2[1])**2) < 2.0:
                    gx2, gy2 = gx_, gy_
                    break
            if gx2 is not None:
                break

        assert_true(gx2 is not None,
                    f'second goal_pose near frontier_2 received within {REASSIGN_TIMEOUT_S}s')

        print(f'\n[{PASS}] All assertions passed — exploration cycle works end-to-end.\n')

    finally:
        for p in procs:
            kill_proc(p)
        purge_nodes('exploration_planner', 'drone_coordinator')
        rclpy.shutdown()
        spin_thread.join(timeout=3)


if __name__ == '__main__':
    main()
