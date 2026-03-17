#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time


def _build_request(args: argparse.Namespace) -> str:
    request = (
        f'sdf_filename: "{args.model_sdf}", '
        f'name: "{args.model_name}", '
        f'pose: {{position: {{x: {args.x}, y: {args.y}, z: {args.z}}}'
    )
    if any(abs(value) > 1e-6 for value in (args.roll, args.pitch, args.yaw)):
        raise ValueError('spawn_gz_model.py currently supports zero-rotation spawns only')
    request += '}'
    return request


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Retry Gazebo model spawning until it succeeds or times out.')
    parser.add_argument('--world', required=True)
    parser.add_argument('--model-sdf', required=True)
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--attempts', type=int, default=8)
    parser.add_argument('--timeout-ms', type=int, default=20000)
    parser.add_argument('--retry-delay-s', type=float, default=2.0)
    args = parser.parse_args()

    request = _build_request(args)
    command = [
        'gz', 'service',
        '-s', f'/world/{args.world}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(args.timeout_ms),
        '--req', request,
    ]

    for attempt in range(1, args.attempts + 1):
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        combined = '\n'.join(
            chunk.strip() for chunk in (result.stdout, result.stderr) if chunk.strip())
        if result.returncode == 0 and 'data: true' in result.stdout:
            print(
                f'[spawn_gz_model] spawned {args.model_name} in world {args.world} '
                f'on attempt {attempt}/{args.attempts}')
            if combined:
                print(combined)
            return 0

        print(
            f'[spawn_gz_model] attempt {attempt}/{args.attempts} failed for '
            f'{args.model_name}',
            file=sys.stderr,
        )
        if combined:
            print(combined, file=sys.stderr)
        if attempt < args.attempts:
            time.sleep(args.retry_delay_s)

    print(
        f'[spawn_gz_model] giving up on {args.model_name} after {args.attempts} attempts',
        file=sys.stderr,
    )
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
