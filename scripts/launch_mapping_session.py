#!/usr/bin/env python3
"""
launch_mapping_session.py

Opens two terminal windows for the mapping_hover stack:
  Terminal 1 — ros2 launch drone_bringup mapping_hover.launch.py
  Terminal 2 — verify_mapping_topics.py  (live pipeline health monitor)

Usage:
  python3 scripts/launch_mapping_session.py
  python3 scripts/launch_mapping_session.py --headless
  python3 scripts/launch_mapping_session.py --hover-alt 2.0
  python3 scripts/launch_mapping_session.py --rviz

Supported terminal emulators (auto-detected in priority order):
  gnome-terminal, xterm, konsole, xfce4-terminal, tilix
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
_SCRIPT_DIR  = Path(__file__).resolve().parent
_WS_ROOT     = _SCRIPT_DIR.parent          # /home/telemaque/ros_ws/drone
_ROS_SETUP   = '/opt/ros/humble/setup.bash'
_WS_SETUP    = _WS_ROOT / 'install' / 'setup.bash'
_VERIFIER    = _SCRIPT_DIR / 'verify_mapping_topics.py'

# ── Terminal emulator detection ───────────────────────────────────────────────
# Each entry: (binary, fn(title, cmd_str) → list[str])
_TERMINALS = [
    ('gnome-terminal', lambda title, cmd: [
        'gnome-terminal',
        f'--title={title}',
        '--',
        'bash', '-c', cmd,
    ]),
    ('xterm', lambda title, cmd: [
        'xterm',
        '-title', title,
        '-fa', 'Monospace',
        '-fs', '10',
        '-geometry', '180x40',
        '-e', f'bash -c {_shell_quote(cmd)}',
    ]),
    ('konsole', lambda title, cmd: [
        'konsole',
        '--title', title,
        '-e', 'bash', '-c', cmd,
    ]),
    ('xfce4-terminal', lambda title, cmd: [
        'xfce4-terminal',
        f'--title={title}',
        '-e', f'bash -c {_shell_quote(cmd)}',
    ]),
    ('tilix', lambda title, cmd: [
        'tilix',
        '--title', title,
        '-e', f'bash -c {_shell_quote(cmd)}',
    ]),
]

_G = '\033[32m'; _R = '\033[31m'; _Y = '\033[33m'; _B = '\033[1m'; _E = '\033[0m'


def _shell_quote(s: str) -> str:
    """Single-quote a string for passing to a shell."""
    return "'" + s.replace("'", "'\\''") + "'"


def _find_terminal() -> tuple[str, object] | None:
    for binary, builder in _TERMINALS:
        if shutil.which(binary):
            return binary, builder
    return None


def _source_prefix() -> str:
    """Return the bash source commands needed before any ros2 invocation."""
    sources = [f'source {_ROS_SETUP}']
    if _WS_SETUP.exists():
        sources.append(f'source {_WS_SETUP}')
    else:
        print(f'{_Y}[warn]{_E} Workspace install not found at {_WS_SETUP}')
        print(f'       Run: colcon build --symlink-install')
    return ' && '.join(sources)


def _open_terminal(builder, title: str, bash_cmd: str) -> subprocess.Popen:
    """Launch a new terminal window running bash_cmd."""
    argv = builder(title, bash_cmd)
    return subprocess.Popen(argv, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(
        description='Launch mapping_hover stack in two terminal windows')
    parser.add_argument('--headless',   action='store_true',
                        help='Run Gazebo without GUI (server-only)')
    parser.add_argument('--rviz',       action='store_true',
                        help='Open RViz2 in the simulation terminal')
    parser.add_argument('--hover-alt',  type=float, default=1.5,
                        help='Hover altitude in metres (default: 1.5)')
    parser.add_argument('--interval',   type=float, default=3.0,
                        help='Verifier refresh interval in seconds (default: 3)')
    parser.add_argument('--wait',       type=float, default=35.0,
                        help='Seconds verifier waits before first check (default: 35)')
    args = parser.parse_args()

    # ── Sanity checks ─────────────────────────────────────────────────────────
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        print(f'{_R}[error]{_E} No display detected ($DISPLAY / $WAYLAND_DISPLAY unset).')
        print('        Cannot open terminal windows in a headless environment.')
        print('        Run this script inside a desktop session.')
        sys.exit(1)

    result = _find_terminal()
    if result is None:
        print(f'{_R}[error]{_E} No supported terminal emulator found.')
        print('        Install one of: gnome-terminal, xterm, konsole, '
              'xfce4-terminal, tilix')
        sys.exit(1)

    binary, builder = result
    src = _source_prefix()

    # Both terminals must share the same CycloneDDS config (loopback only) so
    # that DDS discovery works between the simulation processes and the verifier.
    _cyclone = (
        'export CYCLONEDDS_URI='
        "'<CycloneDDS><Domain><General>"
        '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
        "</General></Domain></CycloneDDS>'"
    )

    # ── Build launch arguments ─────────────────────────────────────────────────
    launch_args: list[str] = []
    if args.headless:
        launch_args.append('headless:=true')
    if args.rviz:
        launch_args.append('use_rviz:=true')
    launch_args.append(f'hover_alt:={args.hover_alt}')
    launch_args_str = ' '.join(launch_args)

    # ── Terminal 1: simulation ─────────────────────────────────────────────────
    sim_cmd = (
        f'{src} && {_cyclone} && '
        f'echo "=== mapping_hover simulation ===" && '
        f'ros2 launch drone_bringup mapping_hover.launch.py {launch_args_str}; '
        f'echo ""; echo "=== Launch exited. Press Enter to close. ==="; read'
    )

    # ── Terminal 2: live topic verifier ───────────────────────────────────────
    verify_cmd = (
        f'{src} && {_cyclone} && '
        f'echo "=== mapping topic verifier ===" && '
        f'echo "Waiting {args.wait:.0f}s for pipeline to boot..." && '
        f'python3 {_VERIFIER} --interval {args.interval} --wait {args.wait}; '
        f'echo ""; echo "=== Verifier exited. Press Enter to close. ==="; read'
    )

    # ── Open terminals ────────────────────────────────────────────────────────
    print(f'{_B}launch_mapping_session.py{_E}')
    print(f'  Terminal emulator : {binary}')
    print(f'  Workspace         : {_WS_ROOT}')
    print(f'  Hover altitude    : {args.hover_alt} m')
    print(f'  Headless          : {args.headless}')
    print(f'  RViz              : {args.rviz}')
    print()

    print(f'  Opening {_B}Terminal 1{_E} — simulation …')
    _open_terminal(builder, 'mapping_hover — simulation', sim_cmd)

    # Small delay so the first terminal appears before the second opens
    time.sleep(1.0)

    print(f'  Opening {_B}Terminal 2{_E} — topic verifier …')
    _open_terminal(builder, 'mapping_hover — topic verifier', verify_cmd)

    print()
    print(f'{_G}Both terminals launched.{_E}')
    print()
    print('  Pipeline start sequence:')
    print('    t=0s   Gazebo + bridge')
    print('    t=10s  PX4 SITL + MicroXRCE')
    print('    t=12s  LIO-SAM + VIO bridges')
    print('    t=15s  OctoMap servers')
    print('    t=30s  Offboard controllers → arm → hover')
    print()
    print('  To stop everything:')
    print('    Ctrl-C in Terminal 1,  then:')
    print('    ps aux | grep -E "(gz sim|px4|MicroXRCE|offboard)" \\')
    print('      | grep -v grep | awk \'{print $2}\' | xargs -r kill -9')
    print()


if __name__ == '__main__':
    main()
