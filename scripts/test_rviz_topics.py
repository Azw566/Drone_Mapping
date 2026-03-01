#!/usr/bin/env python3
"""
test_rviz_topics.py

Verifies that every topic referenced in multi_drone.rviz is being actively
published with the expected message type.

Requires the full simulation stack to be running:
  ros2 launch drone_bringup full_stack.launch.py

Usage (source workspace first)
  python3 scripts/test_rviz_topics.py
  python3 scripts/test_rviz_topics.py --timeout 15   # wait longer for topics
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

# ── Colours ───────────────────────────────────────────────────────────────────
_GREEN  = '\033[32m'
_RED    = '\033[31m'
_YELLOW = '\033[33m'
_CYAN   = '\033[36m'
_RESET  = '\033[0m'

PASS    = f'{_GREEN}PASS{_RESET}'
FAIL    = f'{_RED}FAIL{_RESET}'
SKIP    = f'{_YELLOW}SKIP{_RESET}'
WARN    = f'{_YELLOW}WARN{_RESET}'

# ── RViz display class → ROS2 message type mapping ────────────────────────────
_DISPLAY_TYPE_MAP = {
    'rviz_default_plugins/Map':         'nav_msgs/msg/OccupancyGrid',
    'rviz_default_plugins/PointCloud2': 'sensor_msgs/msg/PointCloud2',
    'rviz_default_plugins/Path':        'nav_msgs/msg/Path',
    'rviz_default_plugins/MarkerArray': 'visualization_msgs/msg/MarkerArray',
    'rviz_default_plugins/Image':       'sensor_msgs/msg/Image',
    'rviz_default_plugins/TF':          None,    # no single topic
    'rviz_default_plugins/Grid':        None,    # no topic
}


def parse_rviz(rviz_path: Path) -> list[dict]:
    """
    Parse multi_drone.rviz manually (no PyYAML dependency needed since the
    file uses a simple repeated structure).

    Returns a list of dicts with keys:
        name, class_, topic, enabled
    """
    displays = []
    current: dict | None = None

    for raw_line in rviz_path.read_text().splitlines():
        line = raw_line.strip()

        if line.startswith('- Class:'):
            if current and current.get('topic'):
                displays.append(current)
            cls = line.split(':', 1)[1].strip()
            current = {'class_': cls, 'name': '', 'topic': None, 'enabled': True}

        elif current is not None:
            if line.startswith('Name:'):
                current['name'] = line.split(':', 1)[1].strip()
            elif line.startswith('Value:') and 'topic' not in line.lower():
                # Topic value line (child of "Topic:" block)
                val = line.split(':', 1)[1].strip()
                if val.startswith('/'):
                    current['topic'] = val
            elif line.startswith('Enabled:'):
                current['enabled'] = line.split(':', 1)[1].strip().lower() == 'true'

    if current and current.get('topic'):
        displays.append(current)

    return displays


def ros2_topic_list(timeout_s: float) -> set[str]:
    """Return the set of currently active topics."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=timeout_s)
        return set(result.stdout.strip().splitlines())
    except subprocess.TimeoutExpired:
        return set()


def ros2_topic_type(topic: str, timeout_s: float = 5.0) -> str | None:
    """Return the message type for a topic, or None on failure."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'type', topic],
            capture_output=True, text=True, timeout=timeout_s)
        t = result.stdout.strip()
        return t if t else None
    except subprocess.TimeoutExpired:
        return None


def ros2_topic_hz(topic: str, sample_time: float = 3.0) -> float | None:
    """
    Estimate topic publish rate by counting messages for sample_time seconds.
    Returns messages/s or None if no messages arrived.
    """
    import threading

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.serialization import deserialize_message
    except ImportError:
        return None

    # Use ros2 topic hz via subprocess (simpler, no type import needed)
    try:
        proc = subprocess.Popen(
            ['ros2', 'topic', 'hz', '--window', '10', topic],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        time.sleep(sample_time)
        proc.terminate()
        out, _ = proc.communicate(timeout=2)
        for line in out.splitlines():
            if 'average rate' in line.lower():
                parts = line.split(':')
                if len(parts) >= 2:
                    try:
                        return float(parts[1].strip().split()[0])
                    except ValueError:
                        pass
        return None
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Seconds to wait for topics to appear (default: 10)')
    parser.add_argument('--check-hz', action='store_true',
                        help='Also sample publish rate (adds ~3s per topic)')
    args = parser.parse_args()

    rviz_path = (Path(__file__).parent.parent /
                 'src/drone_bringup/rviz/multi_drone.rviz')

    print('\n══════════════════════════════════════════════════════════')
    print('  test_rviz_topics.py')
    print(f'  Config: {rviz_path}')
    print('══════════════════════════════════════════════════════════\n')

    if not rviz_path.exists():
        print(f'[{FAIL}] RViz config not found: {rviz_path}')
        sys.exit(1)

    displays = parse_rviz(rviz_path)
    print(f'Parsed {len(displays)} display(s) with topics from RViz config.\n')

    # ── Wait for the ROS2 graph to be available ────────────────────────────────
    print(f'[{WARN}] Waiting up to {args.timeout}s for topics to appear …')
    t0 = time.monotonic()
    active_topics: set[str] = set()
    while time.monotonic() - t0 < args.timeout:
        active_topics = ros2_topic_list(5.0)
        if active_topics:
            break
        time.sleep(1.0)

    if not active_topics:
        print(f'[{FAIL}] No topics found. Is the full simulation stack running?\n'
              f'        Launch it with: ros2 launch drone_bringup full_stack.launch.py\n')
        sys.exit(1)

    print(f'  {len(active_topics)} active topics found.\n')

    # ── Check each display ────────────────────────────────────────────────────
    col_w = 40
    header = (f'  {"Display":<22} {"Topic":<{col_w}} '
              f'{"Present":<10} {"Type OK":<10} {"Enabled":<8}')
    print(header)
    print('  ' + '─' * (len(header) - 2))

    failures = []
    warnings = []

    for d in displays:
        topic   = d['topic']
        enabled = d['enabled']
        cls     = d['class_']
        name    = d['name']
        expected_type = _DISPLAY_TYPE_MAP.get(cls)

        present = topic in active_topics

        if not present:
            actual_type = None
            type_ok     = None
        else:
            actual_type = ros2_topic_type(topic)
            if expected_type is None:
                type_ok = None  # no expectation
            elif actual_type is None:
                type_ok = False
            else:
                type_ok = actual_type == expected_type

        # Format row
        topic_display = topic if len(topic) <= col_w else '…' + topic[-(col_w-1):]
        present_str = f'{_GREEN}yes{_RESET}' if present else f'{_RED}NO{_RESET}'
        if type_ok is True:
            type_str = f'{_GREEN}ok{_RESET}'
        elif type_ok is False:
            type_str = f'{_RED}MISMATCH{_RESET}'
        else:
            type_str = f'{_YELLOW}—{_RESET}'
        enabled_str = 'yes' if enabled else f'{_YELLOW}off{_RESET}'

        print(f'  {name:<22} {topic_display:<{col_w}} '
              f'{present_str:<19} {type_str:<19} {enabled_str}')

        if not present:
            severity = failures if enabled else warnings
            severity.append(f'{"[ENABLED] " if enabled else "[disabled]"}'
                             f'{topic} — not published')
        elif type_ok is False:
            failures.append(f'{topic} — expected {expected_type}, got {actual_type}')

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    if warnings:
        print(f'Disabled-but-missing topics (informational):')
        for w in warnings:
            print(f'  [{WARN}] {w}')
        print()

    if failures:
        print(f'Failures:')
        for f in failures:
            print(f'  [{FAIL}] {f}')
        print(f'\n[{FAIL}] {len(failures)} topic(s) missing or have wrong type.\n')
        sys.exit(1)

    print(f'[{PASS}] All enabled RViz topics are present with correct types.\n')


if __name__ == '__main__':
    main()
