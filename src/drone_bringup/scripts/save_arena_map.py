#!/usr/bin/env python3
"""
save_arena_map.py

Subscribes to both drones' projected_maps and the POI manager's detected
ArUco tags, then saves a single PNG image of the explored arena with each
discovered tag marked and labelled.

Usage (after sourcing the workspace):
  python3 save_arena_map.py               # saves arena_map.png in CWD
  python3 save_arena_map.py --out /tmp/map.png --timeout 60

The script exits automatically after --timeout seconds or on Ctrl-C.
It merges d1 and d2 maps into one canvas using their OccupancyGrid origin
fields so the maps are rendered in the correct relative positions.

Pixel colour key:
  white   = free (explored, no obstacle)
  black   = occupied (wall / obstacle)
  grey    = unknown (never observed)
  star    = ArUco tag position (coloured by tag_id, labelled with id)
"""

import argparse
import math
import signal
import sys
import time
import threading

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("ERROR: OpenCV not found. Install: pip install opencv-python")

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from drone_interfaces.msg import ArucoDetection


# ── Colour palette for ArUco tags (BGR) ──────────────────────────────────────
_PALETTE = [
    (0,   0,   255),   # red
    (0,   200, 0  ),   # green
    (255, 100, 0  ),   # blue
    (0,   180, 255),   # yellow
    (180, 0,   255),   # magenta
    (0,   255, 200),   # lime
    (255, 0,   128),   # purple
    (20,  200, 255),   # orange
]


class MapSaverNode(Node):
    def __init__(self, output_path: str):
        super().__init__('map_saver')
        self._output_path = output_path
        self._maps: dict[str, OccupancyGrid] = {}           # ns → latest grid
        self._tags: dict[int, ArucoDetection] = {}          # tag_id → detection
        self._lock = threading.Lock()

        for ns in ('d1', 'd2'):
            self.create_subscription(
                OccupancyGrid,
                f'/{ns}/projected_map',
                lambda msg, n=ns: self._map_cb(n, msg),
                10,
            )
        self.create_subscription(
            ArucoDetection, '/poi/detections',
            self._tag_cb, 10)

        self.get_logger().info(
            f'MapSaver ready — waiting for maps… output → {output_path}')

    def _map_cb(self, ns: str, msg: OccupancyGrid):
        with self._lock:
            self._maps[ns] = msg

    def _tag_cb(self, msg: ArucoDetection):
        with self._lock:
            self._tags[msg.tag_id] = msg

    # ── Rendering ─────────────────────────────────────────────────────────
    def save(self):
        with self._lock:
            maps = dict(self._maps)
            tags = dict(self._tags)

        if not maps:
            self.get_logger().warn('No map data received yet — nothing to save.')
            return False

        # ── Step 1: find the bounding box that covers all grids ───────────
        # Use the grid with the finest resolution as the reference resolution.
        res = min(g.info.resolution for g in maps.values())

        # World-coordinate AABB across all grids
        world_min_x = world_min_y =  1e9
        world_max_x = world_max_y = -1e9

        for g in maps.values():
            ox = g.info.origin.position.x
            oy = g.info.origin.position.y
            wx = ox + g.info.width  * g.info.resolution
            wy = oy + g.info.height * g.info.resolution
            world_min_x = min(world_min_x, ox)
            world_min_y = min(world_min_y, oy)
            world_max_x = max(world_max_x, wx)
            world_max_y = max(world_max_y, wy)

        canvas_w = max(1, int(math.ceil((world_max_x - world_min_x) / res)))
        canvas_h = max(1, int(math.ceil((world_max_y - world_min_y) / res)))

        # Start with grey (unknown)
        canvas = np.full((canvas_h, canvas_w, 3), 128, dtype=np.uint8)

        # ── Step 2: paint each grid onto the canvas ───────────────────────
        for ns, g in maps.items():
            ox = g.info.origin.position.x
            oy = g.info.origin.position.y
            gw = g.info.width
            gh = g.info.height
            gr = g.info.resolution
            data = np.array(g.data, dtype=np.int16).reshape(gh, gw)

            # Pixel offset of this grid's origin on the canvas
            px_off_x = int((ox - world_min_x) / res)
            px_off_y = int((oy - world_min_y) / res)

            # Scale factor if grids have different resolutions
            scale = gr / res

            for row in range(gh):
                for col in range(gw):
                    v = data[row, col]
                    if v == -1:
                        colour = (128, 128, 128)   # unknown — keep grey
                    elif v == 0:
                        colour = (255, 255, 255)   # free — white
                    else:
                        colour = (30, 30, 30)      # occupied — near-black

                    # Canvas pixel (note: y axis flipped — row 0 = south)
                    cx = px_off_x + int(col * scale)
                    cy = canvas_h - 1 - (px_off_y + int(row * scale))

                    if 0 <= cx < canvas_w and 0 <= cy < canvas_h:
                        # Only overwrite grey (unknown) cells — free/occupied
                        # from any drone wins over unseen
                        if canvas[cy, cx, 0] == 128 or v != -1:
                            canvas[cy, cx] = colour

        # ── Step 3: draw a scale bar ──────────────────────────────────────
        bar_metres   = 5.0
        bar_px       = int(bar_metres / res)
        margin       = 20
        bar_y        = canvas_h - margin
        bar_x_start  = margin
        bar_x_end    = bar_x_start + bar_px
        cv2.line(canvas, (bar_x_start, bar_y), (bar_x_end, bar_y),
                 (0, 0, 0), 3)
        cv2.line(canvas, (bar_x_start, bar_y - 5), (bar_x_start, bar_y + 5),
                 (0, 0, 0), 2)
        cv2.line(canvas, (bar_x_end,   bar_y - 5), (bar_x_end,   bar_y + 5),
                 (0, 0, 0), 2)
        cv2.putText(canvas, f'{bar_metres:.0f} m',
                    (bar_x_start, bar_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

        # ── Step 4: draw drone start positions ────────────────────────────
        start_positions = {
            'd1': (-1.0, -8.0),
            'd2': ( 1.0, -8.0),
        }
        for ns, (wx, wy) in start_positions.items():
            px = int((wx - world_min_x) / res)
            py = canvas_h - 1 - int((wy - world_min_y) / res)
            if 0 <= px < canvas_w and 0 <= py < canvas_h:
                cv2.drawMarker(canvas, (px, py), (200, 100, 0),
                               cv2.MARKER_CROSS, 14, 2)
                cv2.putText(canvas, ns, (px + 6, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 100, 0), 1)

        # ── Step 5: overlay ArUco tags ────────────────────────────────────
        for tag_id, det in tags.items():
            wx = det.world_pose.pose.position.x
            wy = det.world_pose.pose.position.y
            frame = det.world_pose.header.frame_id   # e.g. 'd1/map'

            # We rendered everything in world coordinates, so the tag's
            # map-frame position IS its world position (LIO-SAM map ≈ world).
            px = int((wx - world_min_x) / res)
            py = canvas_h - 1 - int((wy - world_min_y) / res)

            if not (0 <= px < canvas_w and 0 <= py < canvas_h):
                self.get_logger().warn(
                    f'Tag {tag_id} at ({wx:.1f},{wy:.1f}) is outside canvas — skipped')
                continue

            colour = _PALETTE[tag_id % len(_PALETTE)]

            # Star marker
            cv2.drawMarker(canvas, (px, py), colour,
                           cv2.MARKER_STAR, 20, 2)
            # Circle outline
            cv2.circle(canvas, (px, py), 12, colour, 2)
            # Label
            label = f'ID {tag_id} (by {det.detected_by})'
            cv2.putText(canvas, label, (px + 14, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)

        # ── Step 6: legend ────────────────────────────────────────────────
        legend_x = canvas_w - 180
        legend_y = 20
        cv2.rectangle(canvas, (legend_x - 5, legend_y - 5),
                      (canvas_w - 5, legend_y + 90), (200, 200, 200), -1)
        cv2.rectangle(canvas, (legend_x - 5, legend_y - 5),
                      (canvas_w - 5, legend_y + 90), (0, 0, 0), 1)
        entries = [
            ((255, 255, 255), 'Free (explored)'),
            ((30,  30,  30 ), 'Occupied (wall)'),
            ((128, 128, 128), 'Unknown'),
            ((200, 100, 0  ), 'Drone start'),
            ((0,   0,   255), 'ArUco tag'),
        ]
        for i, (c, label) in enumerate(entries):
            y = legend_y + i * 17
            cv2.rectangle(canvas, (legend_x, y), (legend_x + 12, y + 12), c, -1)
            cv2.rectangle(canvas, (legend_x, y), (legend_x + 12, y + 12), (0, 0, 0), 1)
            cv2.putText(canvas, label, (legend_x + 16, y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 1)

        # ── Step 7: title ─────────────────────────────────────────────────
        n_tags = len(tags)
        title  = f'Arena map — {n_tags} ArUco tag(s) found'
        cv2.putText(canvas, title, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # ── Step 8: save ──────────────────────────────────────────────────
        cv2.imwrite(self._output_path, canvas)
        self.get_logger().info(
            f'Saved {canvas_w}x{canvas_h}px map → {self._output_path} '
            f'({n_tags} tags, {len(maps)} drone map(s))')
        return True

    def status(self):
        with self._lock:
            maps_ready = list(self._maps.keys())
            n_tags     = len(self._tags)
        return maps_ready, n_tags


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Save arena map with ArUco tags')
    parser.add_argument('--out', default='arena_map.png',
                        help='Output PNG path (default: arena_map.png in CWD)')
    parser.add_argument('--timeout', type=float, default=0.0,
                        help='Exit after this many seconds (0 = wait for Ctrl-C)')
    args = parser.parse_args()

    rclpy.init()
    node = MapSaverNode(args.out)

    done = threading.Event()

    def _shutdown(signum, frame):
        node.get_logger().info('Ctrl-C — saving and exiting…')
        done.set()

    signal.signal(signal.SIGINT, _shutdown)

    # Spin in a background thread so the main thread can monitor
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    start = time.time()
    last_status = 0.0

    while not done.is_set():
        elapsed = time.time() - start

        # Print status every 5 s
        if elapsed - last_status >= 5.0:
            maps_ready, n_tags = node.status()
            print(f'[{elapsed:.0f}s] Maps received: {maps_ready or "none"} | '
                  f'ArUco tags found: {n_tags}')
            last_status = elapsed

        if args.timeout > 0 and elapsed >= args.timeout:
            print(f'Timeout reached ({args.timeout:.0f}s) — saving map…')
            done.set()

        time.sleep(0.5)

    node.save()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
