#!/usr/bin/env python3
"""Compatibility entry: user policy now permits manual stopping only.

Pure legacy diagnostic predicates remain importable for historical tests.
They have no authority to signal recorder or mapper processes.
"""
from capture_monitor_only import main


def health_fault(now, last_imu, last_lidar, serial_present, imu_timeout=.2, lidar_timeout=.5):
    if not serial_present:
        return 'serial_device_missing'
    if last_imu is not None and now-last_imu > imu_timeout:
        return 'imu_receive_timeout'
    if last_lidar is not None and now-last_lidar > lidar_timeout:
        return 'lidar_receive_timeout'
    return None


def header_fault(topic, previous, current):
    if previous is None:return None
    step=current-previous
    if step < (-.02 if topic=='imu' else -.001):return topic+'_header_rewind'
    if step > (.2 if topic=='imu' else .25):return topic+'_header_jump'
    return None


def point_time_fault(max_offset_ns):
    # A bounded check for this setup's nominal 10 Hz / 0.1 s LiDAR frames.
    return 'lidar_point_time_over_150ms' if max_offset_ns>150_000_000 else None



if __name__ == '__main__':
    main()
