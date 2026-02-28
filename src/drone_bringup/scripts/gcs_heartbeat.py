#!/usr/bin/env python3
"""
gcs_heartbeat.py

Sends MAVLink HEARTBEAT messages to all PX4 SITL instances so that
the 'No connection to the ground control station' preflight check is
satisfied without needing QGroundControl.

PX4 SITL exposes a GCS MAVLink interface on UDP port (18570 + instance):
  instance 0 (d1) → localhost:18570
  instance 1 (d2) → localhost:18571

Usage:
  python3 gcs_heartbeat.py [n_instances]   default: 2
"""

import sys
import time
import socket
import struct

N_INSTANCES = int(sys.argv[1]) if len(sys.argv) > 1 else 2
PORTS = [18570 + i for i in range(N_INSTANCES)]
RATE_HZ = 2.0

# Minimal MAVLink 1 HEARTBEAT packet (type GCS = 6, autopilot INVALID = 8)
# Sequence, system ID, component ID are fixed for simplicity.
def _heartbeat_bytes(seq: int) -> bytes:
    payload = struct.pack('<IBBBBB',
        0,          # custom_mode (uint32)
        6,          # type: MAV_TYPE_GCS
        8,          # autopilot: MAV_AUTOPILOT_INVALID
        0,          # base_mode
        0,          # system_status: MAV_STATE_UNINIT
        3,          # mavlink_version = 3
    )
    length = len(payload)
    msg_id = 0  # HEARTBEAT
    sysid  = 255
    compid = 0
    header = struct.pack('BBBBBB', 0xFE, length, seq & 0xFF, sysid, compid, msg_id)
    crc_extra = 50  # HEARTBEAT CRC extra
    crc_data = bytes([length, seq & 0xFF, sysid, compid, msg_id]) + payload + bytes([crc_extra])
    crc = _crc16(crc_data)
    return header + payload + struct.pack('<H', crc)


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    print(f'[gcs_heartbeat] Sending HEARTBEAT to {N_INSTANCES} PX4 instance(s) '
          f'on ports {PORTS} at {RATE_HZ} Hz', flush=True)
    while True:
        pkt = _heartbeat_bytes(seq)
        for port in PORTS:
            try:
                sock.sendto(pkt, ('127.0.0.1', port))
            except OSError:
                pass
        seq += 1
        time.sleep(1.0 / RATE_HZ)


if __name__ == '__main__':
    main()
