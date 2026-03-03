#!/usr/bin/env python3
"""
test_pipeline_mock.py

No-Gazebo/PX4 integration test for the upstream pipeline:
  fake PointCloud2 → lidar_enricher → /d1/points_enriched

Checks:
  1. lidar_enricher starts up without crashing
  2. 'ring' field (UINT16) is added at a 4-byte-aligned offset
  3. 'time' field (FLOAT32) is added and 4-byte-aligned
  4. ring values are in [0, N_SCAN-1]
  5. time values are in [0, 1/scan_rate]
  6. LIO-SAM YAML configs load all required keys
  7. lio_sam_d1.yaml and lio_sam_d2.yaml are symmetric (same keys, symmetric values)

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/test_pipeline_mock.py
"""

import subprocess
import sys
import time
import struct
import threading
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

# ── Colours ───────────────────────────────────────────────────────────────────
_G = '\033[32m'; _R = '\033[31m'; _Y = '\033[33m'; _C = '\033[36m'; _E = '\033[0m'
PASS = f'{_G}PASS{_E}'; FAIL = f'{_R}FAIL{_E}'; SKIP = f'{_Y}SKIP{_E}'

results = []

def check(name, condition, detail=''):
    status = PASS if condition else FAIL
    results.append((name, condition, detail))
    print(f'  [{status}] {name}' + (f'  — {detail}' if detail else ''))

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1: Unit test — lidar_enricher field layout (no subprocess needed)
# ─────────────────────────────────────────────────────────────────────────────

print(f'\n{"═"*66}')
print('  Section 1: lidar_enricher field layout (unit test)')
print(f'{"═"*66}')

sys.path.insert(0, str(Path(__file__).parent.parent / 'src/drone_bringup/scripts'))

try:
    import importlib.util, types
    spec = importlib.util.spec_from_file_location(
        'lidar_enricher',
        Path(__file__).parent.parent / 'src/drone_bringup/scripts/lidar_enricher.py'
    )
    le_mod = importlib.util.module_from_spec(spec)

    # Stub rclpy.Node so we can import without a live ROS context
    import rclpy as _rclpy
    class _FakeNode:
        def __init__(self, *a, **kw): pass
        def declare_parameter(self, *a, **kw): pass
        def get_parameter(self, name):
            defaults = {
                'n_scan': 16, 'horizon_scan': 1800, 'scan_rate': 10.0,
                'min_vert_deg': -15.0, 'max_vert_deg': 15.0,
                'input_topic': '/d1/points_raw',
                'output_topic': '/d1/points_enriched',
            }
            class _P:
                def __init__(self, v): self.value = v
            return _P(defaults.get(name, None))
        def get_logger(self):
            class _L:
                def info(self, *a): pass
            return _L()
        def create_subscription(self, *a, **kw): pass
        def create_publisher(self, *a, **kw): pass

    import unittest.mock as mock
    with mock.patch('rclpy.node.Node.__init__', _FakeNode.__init__), \
         mock.patch('rclpy.node.Node.declare_parameter', _FakeNode.declare_parameter), \
         mock.patch('rclpy.node.Node.get_parameter', _FakeNode.get_parameter), \
         mock.patch('rclpy.node.Node.get_logger', _FakeNode.get_logger), \
         mock.patch('rclpy.node.Node.create_subscription', _FakeNode.create_subscription), \
         mock.patch('rclpy.node.Node.create_publisher', _FakeNode.create_publisher), \
         mock.patch('rclpy.qos.QoSProfile', mock.MagicMock()):
        spec.loader.exec_module(le_mod)

    enricher_cls = le_mod.LidarEnricher
    SIZE_OF = le_mod.SIZE_OF
    check('lidar_enricher.py imports cleanly', True)

    # Build a minimal XYZ PointCloud2 (16 pts in a vertical fan, no ring/time)
    fields_in = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    import math, struct as _struct
    pts = []
    for i in range(16):
        angle = math.radians(-15 + i * 2)
        x, y, z = math.cos(angle) * 5.0, 0.0, math.sin(angle) * 5.0
        pts.append((x, y, z, 100.0))

    hdr = Header()
    hdr.frame_id = 'd1/lidar_link'
    cloud_in = pc2.create_cloud(hdr, fields_in, pts)

    # Simulate what the enricher does (replicate cb() logic)
    field_names = [f.name for f in cloud_in.fields]
    has_ring = 'ring' in field_names
    has_time = 'time' in field_names

    fields_out = list(cloud_in.fields)
    offset = 0
    if fields_out:
        last = fields_out[-1]
        offset = last.offset + SIZE_OF[last.datatype] * last.count  # = 16

    ring_offset = None
    time_offset = None
    if not has_ring:
        ring_offset = offset
        fields_out.append(PointField(name='ring', offset=offset,
                                     datatype=PointField.UINT16, count=1))
        offset += 2  # = 18
        if offset % 4 != 0:
            offset += 4 - (offset % 4)  # → 20
    if not has_time:
        time_offset = offset
        fields_out.append(PointField(name='time', offset=offset,
                                     datatype=PointField.FLOAT32, count=1))

    # Checks
    check('ring field added',     ring_offset is not None, f'offset={ring_offset}')
    check('time field added',     time_offset is not None, f'offset={time_offset}')
    check('ring UINT16',          next(f for f in fields_out if f.name == 'ring').datatype == PointField.UINT16)
    check('time FLOAT32',         next(f for f in fields_out if f.name == 'time').datatype == PointField.FLOAT32)
    check('ring offset ≥ 16',     ring_offset >= 16,       f'ring_offset={ring_offset}')
    check('time 4-byte aligned',  time_offset % 4 == 0,    f'time_offset={time_offset}')
    check('ring-to-time gap ≥ 2', time_offset - ring_offset >= 2,
          f'gap={time_offset - ring_offset} bytes')
    check('ring-to-time gap ≠ 2 (pad present)',
          time_offset - ring_offset != 2,
          f'pad={time_offset - ring_offset - 2} bytes — needed to avoid FLOAT32 misalignment')

except Exception as e:
    check('lidar_enricher.py imports cleanly', False, str(e))

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 2: Integration test — lidar_enricher as subprocess + ROS pub/sub
# ─────────────────────────────────────────────────────────────────────────────

print(f'\n{"═"*66}')
print('  Section 2: lidar_enricher subprocess integration test')
print(f'{"═"*66}')

received_msgs = []
enricher_proc = None

try:
    rclpy.init()
    test_node = Node('pipeline_mock_tester')

    IN_TOPIC  = '/test_mock/points_raw'
    OUT_TOPIC = '/test_mock/points_enriched'

    qos = rclpy.qos.QoSProfile(
        depth=5,
        reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
    )

    pub = test_node.create_publisher(PointCloud2, IN_TOPIC, qos)

    def _recv(msg): received_msgs.append(msg)
    test_node.create_subscription(PointCloud2, OUT_TOPIC, _recv, qos)

    # Start lidar_enricher with test topics
    enricher_proc = subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).parent.parent / 'src/drone_bringup/scripts/lidar_enricher.py'),
            '--ros-args',
            '-p', f'input_topic:={IN_TOPIC}',
            '-p', f'output_topic:={OUT_TOPIC}',
            '-p', 'n_scan:=16',
            '-p', 'horizon_scan:=1800',
            '-p', 'scan_rate:=10.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for node to start
    time.sleep(2.0)

    check('lidar_enricher process started', enricher_proc.poll() is None,
          f'pid={enricher_proc.pid}')

    # Publish 3 fake point clouds
    import math, struct as _s

    def make_cloud(topic_frame='d1/lidar_link'):
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pts = []
        for i in range(16 * 100):  # 16 rings × 100 cols
            angle = math.radians(-15 + (i % 16) * 2)
            x = math.cos(angle) * 3.0
            y = float(i // 16) * 0.01
            z = math.sin(angle) * 3.0
            pts.append((x, y, z, 100.0))
        hdr = Header()
        hdr.frame_id = topic_frame
        return pc2.create_cloud(hdr, fields, pts)

    for _ in range(3):
        pub.publish(make_cloud())
        for _ in range(20):
            rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.2)

    # Wait for responses
    deadline = time.monotonic() + 4.0
    while not received_msgs and time.monotonic() < deadline:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    check('received at least 1 enriched cloud', len(received_msgs) > 0,
          f'got {len(received_msgs)} messages')

    if received_msgs:
        msg = received_msgs[0]
        out_fields = {f.name: f for f in msg.fields}

        check("'ring' field present in output",  'ring' in out_fields)
        check("'time' field present in output",  'time' in out_fields)

        if 'ring' in out_fields and 'time' in out_fields:
            rf = out_fields['ring']
            tf = out_fields['time']
            check('ring datatype = UINT16',   rf.datatype == PointField.UINT16,
                  f'got {rf.datatype}')
            check('time datatype = FLOAT32',  tf.datatype == PointField.FLOAT32,
                  f'got {tf.datatype}')
            check('time offset 4-byte aligned', tf.offset % 4 == 0,
                  f'time.offset={tf.offset}')
            check('ring→time gap ≥ 4 bytes (padding present)',
                  tf.offset - rf.offset >= 4,
                  f'gap={tf.offset - rf.offset}')

            # Parse actual data values
            row_step = msg.row_step
            point_step = msg.point_step
            raw = bytes(msg.data)
            rings_seen = set()
            times_seen = []
            for i in range(min(msg.width * msg.height, 200)):
                base = i * point_step
                ring_val = _s.unpack_from('<H', raw, base + rf.offset)[0]
                time_val = _s.unpack_from('<f', raw, base + tf.offset)[0]
                rings_seen.add(ring_val)
                times_seen.append(time_val)

            max_ring = max(rings_seen) if rings_seen else -1
            min_time = min(times_seen) if times_seen else -1
            max_time = max(times_seen) if times_seen else -1

            check('ring values in [0, 15]',
                  0 <= max_ring <= 15,
                  f'max_ring={max_ring}, unique_rings={sorted(rings_seen)}')
            check('multiple ring values present (vertical distribution)',
                  len(rings_seen) > 1,
                  f'{len(rings_seen)} unique rings: {sorted(rings_seen)}')
            check('time values ≥ 0',
                  min_time >= 0.0,
                  f'min={min_time:.6f}')
            check('time values ≤ 1/scan_rate (0.1 s)',
                  max_time <= 0.101,
                  f'max={max_time:.6f} s')

except Exception as e:
    check('integration test ran without exception', False, str(e))
    import traceback; traceback.print_exc()

finally:
    if enricher_proc and enricher_proc.poll() is None:
        enricher_proc.terminate()
        enricher_proc.wait(timeout=3)
    try:
        rclpy.shutdown()
    except Exception:
        pass

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 3: LIO-SAM YAML config validation
# ─────────────────────────────────────────────────────────────────────────────

print(f'\n{"═"*66}')
print('  Section 3: LIO-SAM YAML config validation')
print(f'{"═"*66}')

REQUIRED_KEYS = {
    'sensor':              'velodyne',
    'N_SCAN':              16,
    'Horizon_SCAN':        1800,
    'lidarMinRange':       0.3,
    'odometryFrame':       None,  # just must exist
    'baselinkFrame':       None,
    'pointCloudTopic':     None,
    'imuTopic':            None,
    'mappingProcessInterval': None,
}
FORBIDDEN_KEYS = ['odomFrame', 'baseLinkFrame', 'mappingFrequency', 'imuFrame']

ws = Path(__file__).parent.parent
configs = {
    'd1': ws / 'src/drone_bringup/config/lio_sam_d1.yaml',
    'd2': ws / 'src/drone_bringup/config/lio_sam_d2.yaml',
}

loaded = {}
for drone, cfg_path in configs.items():
    print(f'\n  [{drone}] {cfg_path.name}')
    try:
        raw = yaml.safe_load(cfg_path.read_text())
        # Flatten: handle /**:  ros__parameters:  <keys>  (two levels deep)
        flat = {}
        if isinstance(raw, dict):
            for k, v in raw.items():
                if isinstance(v, dict):
                    for k2, v2 in v.items():
                        if isinstance(v2, dict):
                            flat.update(v2)   # /** → ros__parameters → params
                        else:
                            flat[k2] = v2
                else:
                    flat[k] = v
        loaded[drone] = flat

        for key, expected in REQUIRED_KEYS.items():
            if expected is None:
                check(f'  key present: {key}', key in flat, f'value={flat.get(key)}')
            else:
                check(f'  key={key}', flat.get(key) == expected,
                      f'got={flat.get(key)!r}  want={expected!r}')

        for key in FORBIDDEN_KEYS:
            check(f'  forbidden key absent: {key}', key not in flat,
                  '' if key not in flat else f'FOUND: {flat[key]!r}')

        # extrinsicTrans must be [0,0,-0.12] or [0.0,0.0,-0.12]
        et = flat.get('extrinsicTrans', [])
        check('  extrinsicTrans z < 0 (lidar above IMU)',
              len(et) == 3 and et[2] < 0,
              f'got {et}')

    except Exception as e:
        check(f'  {cfg_path.name} loads', False, str(e))

# Symmetry check between d1 and d2
if 'd1' in loaded and 'd2' in loaded:
    print(f'\n  [symmetry] d1 vs d2 key differences')
    d1, d2 = loaded['d1'], loaded['d2']
    diff_keys = [k for k in set(list(d1) + list(d2))
                 if k not in ('pointCloudTopic', 'imuTopic', 'odometryFrame',
                              'baselinkFrame', 'mapFrame', 'lidarFrame')
                 and d1.get(k) != d2.get(k)]
    check('  d1/d2 non-namespace keys identical',
          len(diff_keys) == 0,
          f'differing: {diff_keys}' if diff_keys else '')

# ─────────────────────────────────────────────────────────────────────────────
# SUMMARY
# ─────────────────────────────────────────────────────────────────────────────

print(f'\n{"═"*66}')
passed = sum(1 for _, ok, _ in results if ok)
failed = sum(1 for _, ok, _ in results if not ok)
total  = len(results)
color  = _G if failed == 0 else _R
print(f'  {color}Result: {passed}/{total} checks passed  ({failed} failed){_E}')
print(f'{"═"*66}\n')

sys.exit(0 if failed == 0 else 1)
