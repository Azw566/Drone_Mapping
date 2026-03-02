#!/usr/bin/env python3
"""
show_map_result.py

Captures a snapshot of the completed maze map and all discovered POIs, then
saves a PNG report and prints a text summary.

Run AFTER (or during) the full-stack mission:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 scripts/show_map_result.py

Options:
    --output PATH   Where to save the PNG  (default: /tmp/maze_map_result.png)
    --timeout N     Seconds to wait for topics  (default: 15)
"""

import argparse
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

try:
    from drone_interfaces.msg import ArucoDetection
    _HAS_INTERFACES = True
except ImportError:
    _HAS_INTERFACES = False
    print('[WARN] drone_interfaces not found — POI data will be skipped.')

try:
    import matplotlib
    matplotlib.use('Agg')           # headless / no display needed
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False


# ── ROS2 node that collects one snapshot of each topic ────────────────────────

class MapResultNode(Node):
    def __init__(self, timeout: float):
        super().__init__('map_result_viewer')
        self._timeout = timeout
        self._maps: dict[str, OccupancyGrid] = {}
        self._pois: dict[int, 'ArucoDetection'] = {}
        self._done = threading.Event()
        self._start = time.time()

        # TRANSIENT_LOCAL so we get the last latched OccupancyGrid immediately
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        for ns in ('d1', 'd2'):
            self.create_subscription(
                OccupancyGrid, f'/{ns}/projected_map',
                lambda msg, n=ns: self._map_cb(n, msg),
                map_qos)

        if _HAS_INTERFACES:
            self.create_subscription(
                ArucoDetection, '/poi/detections',
                self._poi_cb, 10)

        self.create_timer(0.5, self._check_done)

    def _map_cb(self, ns: str, msg: OccupancyGrid):
        if ns not in self._maps:
            self.get_logger().info(
                f'Received map from {ns} '
                f'({msg.info.width}×{msg.info.height} cells, '
                f'{msg.info.resolution*100:.1f} cm/cell)')
        self._maps[ns] = msg

    def _poi_cb(self, msg: 'ArucoDetection'):
        tid = msg.tag_id
        if tid not in self._pois:
            p = msg.world_pose.pose.position
            self.get_logger().info(
                f'POI  tag {tid}  at ({p.x:.2f}, {p.y:.2f})  '
                f'conf={msg.confidence:.2f}  by={msg.detected_by}')
        self._pois[tid] = msg

    def _check_done(self):
        elapsed = time.time() - self._start
        got_all = 'd1' in self._maps and 'd2' in self._maps
        if got_all or elapsed >= self._timeout:
            self._done.set()

    def wait(self):
        self._done.wait(timeout=self._timeout + 2.0)


# ── Map rendering helpers ──────────────────────────────────────────────────────

def _grid_to_image(grid: OccupancyGrid) -> tuple[np.ndarray, dict]:
    """Return (uint8 image, info-dict) from an OccupancyGrid.

    Colour convention:
      255  = free (white)
        0  = occupied (black)
      128  = unknown (mid-gray)
    """
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 128, dtype=np.uint8)
    img[data == 0]   = 255
    img[data == 100] = 0
    return img, {
        'resolution': grid.info.resolution,
        'origin_x':   grid.info.origin.position.x,
        'origin_y':   grid.info.origin.position.y,
        'width':       w,
        'height':      h,
    }


def _world_to_px(wx: float, wy: float, info: dict) -> tuple[float, float]:
    """Convert world (m) → image pixel coordinates."""
    px = (wx - info['origin_x']) / info['resolution']
    py = (wy - info['origin_y']) / info['resolution']
    return px, py


def render_png(maps: dict, pois: dict, output_path: str) -> None:
    n = len(maps)
    fig, axes = plt.subplots(1, n, figsize=(10 * n, 9), facecolor='#2a2a2a')
    if n == 1:
        axes = [axes]

    drone_colors = {'d1': '#00DC50', 'd2': '#FF8C00'}
    poi_color    = '#FF4444'

    for ax, (ns, grid) in zip(axes, sorted(maps.items())):
        img, info = _grid_to_image(grid)

        # origin='lower' keeps ROS y-up convention
        ax.imshow(img, cmap='gray', origin='lower',
                  vmin=0, vmax=255, interpolation='nearest')

        ax.set_title(
            f'Drone {ns.upper()} — Maze Map\n'
            f'{info["width"]}×{info["height"]} cells · '
            f'{info["resolution"]*100:.1f} cm/cell',
            color='white', fontsize=13, pad=10)
        ax.set_xlabel('X (cells)', color='lightgray')
        ax.set_ylabel('Y (cells)', color='lightgray')
        ax.tick_params(colors='lightgray')
        ax.set_facecolor('#1a1a1a')
        for spine in ax.spines.values():
            spine.set_edgecolor('#555')

        # Overlay POI markers
        for tag_id, det in pois.items():
            p   = det.world_pose.pose.position
            px, py = _world_to_px(p.x, p.y, info)
            if not (0 <= px < info['width'] and 0 <= py < info['height']):
                continue   # outside this drone's map extent
            ax.plot(px, py, 'o',
                    color=poi_color, markersize=14,
                    markeredgecolor='white', markeredgewidth=1.5, zorder=5)
            ax.annotate(
                f'Tag {tag_id}\n({det.detected_by})',
                (px, py), xytext=(8, 6), textcoords='offset points',
                color='white', fontsize=8,
                bbox=dict(boxstyle='round,pad=0.25',
                          facecolor='#AA0000', alpha=0.85),
                zorder=6)

        # Legend
        drone_patch = mpatches.Patch(
            color=drone_colors.get(ns, 'cyan'),
            label=f'{ns.upper()} mapped area')
        poi_patch = mpatches.Patch(
            color=poi_color,
            label=f'ArUco POIs ({len(pois)} tag{"s" if len(pois) != 1 else ""})')
        ax.legend(handles=[drone_patch, poi_patch],
                  loc='lower right', facecolor='#333',
                  labelcolor='white', fontsize=9, framealpha=0.9)

    fig.suptitle(
        'Maze Exploration — Final Map & Points of Interest',
        color='white', fontsize=15, y=0.99)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#2a2a2a')
    plt.close(fig)
    print(f'[OK] Map saved → {output_path}')


# ── Text report ────────────────────────────────────────────────────────────────

def print_report(maps: dict, pois: dict) -> None:
    sep = '=' * 62
    print(f'\n{sep}')
    print('  MAZE EXPLORATION — FINAL REPORT')
    print(sep)

    if not maps:
        print('\n  [!] No map data received.')
    else:
        for ns, grid in sorted(maps.items()):
            w, h  = grid.info.width, grid.info.height
            data  = np.array(grid.data, dtype=np.int16)
            total = w * h
            free  = int(np.sum(data == 0))
            occ   = int(np.sum(data == 100))
            unk   = int(np.sum(data == -1))
            print(f'\n  Drone {ns.upper()} map:')
            print(f'    Grid size    : {w} × {h}  ({total} cells)')
            print(f'    Resolution   : {grid.info.resolution * 100:.1f} cm/cell')
            print(f'    Free         : {free:6d} cells  ({100*free/total:5.1f} %)')
            print(f'    Occupied     : {occ:6d} cells  ({100*occ/total:5.1f} %)')
            print(f'    Unknown      : {unk:6d} cells  ({100*unk/total:5.1f} %)')
            area_m2 = free * grid.info.resolution ** 2
            print(f'    Explored area: {area_m2:.2f} m²')

    print(f'\n  ArUco / POI tags found: {len(pois)}')
    if pois:
        print(f'  {"ID":>4}  {"  X (m)":>8}  {"  Y (m)":>8}  '
              f'{"  Z (m)":>8}  {"Conf":>5}  Detected by')
        print('  ' + '-' * 52)
        for tid, det in sorted(pois.items()):
            p = det.world_pose.pose.position
            print(f'  {tid:4d}  {p.x:8.3f}  {p.y:8.3f}  {p.z:8.3f}  '
                  f'{det.confidence:5.2f}  {det.detected_by}')
    else:
        print('  (none detected — were ArUco markers visible during the mission?)')

    print(f'\n{sep}\n')


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Save maze map + POIs as a PNG after a full-stack mission.')
    parser.add_argument(
        '--output', default='/tmp/maze_map_result.png',
        help='Output PNG path  (default: /tmp/maze_map_result.png)')
    parser.add_argument(
        '--timeout', type=float, default=30.0,
        help='Seconds to wait for topic data  (default: 30)')
    args = parser.parse_args()

    if not _HAS_MPL:
        print('[ERROR] matplotlib is required:  pip3 install matplotlib')
        sys.exit(1)

    print(f'Waiting up to {args.timeout:.0f} s for map + POI data …')

    rclpy.init()
    node = MapResultNode(timeout=args.timeout)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.wait()

    maps = dict(node._maps)
    pois = dict(node._pois)

    # Orderly shutdown: stop executor first so the spin thread exits cleanly
    executor.shutdown(timeout_sec=1.0)
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()

    print_report(maps, pois)

    if maps:
        render_png(maps, pois, args.output)
    else:
        print(
            '[ERROR] No map received.\n'
            '\n'
            'Checklist:\n'
            '  1. Run this script WHILE the full stack is still running —\n'
            '     once ros2 launch is killed the latched map is gone.\n'
            '  2. Check the map pipeline:\n'
            '       ros2 topic hz /d1/lio_sam/mapping/cloud_registered  # ~1 Hz\n'
            '       ros2 topic hz /d1/projected_map                     # ~1 Hz\n'
            '  3. If both are 0 Hz, LIO-SAM has not initialised yet.\n'
            '     Wait ~40 s after launch and retry.\n'
            '  4. Try a longer timeout:\n'
            '       python3 scripts/show_map_result.py --timeout 60'
        )
        sys.exit(1)


if __name__ == '__main__':
    main()
