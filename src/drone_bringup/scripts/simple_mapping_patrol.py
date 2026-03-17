#!/usr/bin/env python3
"""
Move both Gazebo drone models through a fixed set of survey viewpoints.

This mapping-only path bypasses PX4 entirely and repositions the simulated
models directly. That keeps the toolchain small and avoids the current arming /
EKF issues while still producing a usable area map from lidar.
"""

import argparse
import math
import subprocess
import time


WORLD = 'maze'
RATE_HZ = 1.0


def _parse_interest_points(spec: str):
    points = []
    if not spec.strip():
        return points
    for raw in spec.split(';'):
        raw = raw.strip()
        if not raw:
            continue
        name, x, y, z = raw.split(':')
        points.append((name, float(x), float(y), float(z)))
    return points


def _quat_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _set_pose(model: str, x: float, y: float, z: float, yaw_rad: float) -> None:
    qx, qy, qz, qw = _quat_from_yaw(yaw_rad)
    req = (
        f'name: "{model}", '
        f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
        f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}'
    )
    subprocess.run(
        [
            'gz', 'service',
            '-s', f'/world/{WORLD}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _route_points():
    return [
        (-6.0, -8.0, math.pi / 2.0),
        (-6.0, -1.5, math.pi / 2.0),
        (-6.0,  5.0, math.pi / 2.0),
        ( 0.0,  7.5, 0.0),
        ( 6.0,  5.0, -math.pi / 2.0),
        ( 6.0, -1.5, -math.pi / 2.0),
        ( 6.0, -8.0, -math.pi / 2.0),
        ( 0.0, -8.0, math.pi),
        ( 0.0,  0.0, 0.0),
    ]


def _scan_yaws(scan_steps: int):
    return [(-math.pi + (2.0 * math.pi * i / scan_steps)) for i in range(scan_steps)]


def main():
    parser = argparse.ArgumentParser(description='Fixed viewpoint survey for simple mapping')
    parser.add_argument('--hold-s', type=float, default=8.0,
                        help='Seconds to hold each viewpoint')
    parser.add_argument('--altitude-m', type=float, default=2.8,
                        help='Survey altitude in metres')
    parser.add_argument('--interest-points', default='',
                        help='Semicolon-separated name:x:y:z interest points')
    parser.add_argument('--interest-dwell-s', type=float, default=1.5,
                        help='Seconds to hold each yaw step at an interest point')
    parser.add_argument('--scan-steps', type=int, default=8,
                        help='How many yaw positions to use for a 360 scan')
    args = parser.parse_args()

    route = _route_points()
    interest_points = _parse_interest_points(args.interest_points)
    print(
        f'[simple_mapping_patrol] starting {len(route)} viewpoints '
        f'hold={args.hold_s:.1f}s altitude={args.altitude_m:.1f}m '
        f'interest_points={len(interest_points)}',
        flush=True,
    )
    time.sleep(2.0)

    for idx, (x, y, yaw) in enumerate(route, start=1):
        _set_pose('x500_d1', x, y, args.altitude_m, yaw)
        _set_pose('x500_d2', -x, y, args.altitude_m, yaw)
        print(
            f'[simple_mapping_patrol] viewpoint {idx}: '
            f'd1=({x:.1f},{y:.1f}) d2=({-x:.1f},{y:.1f})',
            flush=True,
        )
        time.sleep(args.hold_s)

    west_points = [pt for pt in interest_points if pt[1] <= 0.0]
    east_points = [pt for pt in interest_points if pt[1] > 0.0]
    max_points = max(len(west_points), len(east_points))

    for idx in range(max_points):
        if idx < len(west_points):
            name, x, y, z = west_points[idx]
            print(f'[simple_mapping_patrol] d1 interest point {name} at ({x:.1f},{y:.1f},{z:.1f})',
                  flush=True)
            for yaw in _scan_yaws(args.scan_steps):
                _set_pose('x500_d1', x, y, z, yaw)
                time.sleep(args.interest_dwell_s)
        if idx < len(east_points):
            name, x, y, z = east_points[idx]
            print(f'[simple_mapping_patrol] d2 interest point {name} at ({x:.1f},{y:.1f},{z:.1f})',
                  flush=True)
            for yaw in _scan_yaws(args.scan_steps):
                _set_pose('x500_d2', x, y, z, yaw)
                time.sleep(args.interest_dwell_s)

    print('[simple_mapping_patrol] survey complete', flush=True)


if __name__ == '__main__':
    main()
