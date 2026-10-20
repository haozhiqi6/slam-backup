#!/usr/bin/env python3
"""Isolated replay with injected warnings; explicitly end this test afterward."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import signal
import subprocess
import time

home = Path('/home/orangepi')
workspace = home / 'dl/livox_ws'
logs = workspace / 'src/FASTLIVO2_ROS2-master/Log'
bag = home / 'slam_guarded_20261019_213002/raw'
out = home / ('monitor_only_validation_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
backup = out / 'previous_logs'
backup.mkdir()
assert os.environ.get('ROS_DOMAIN_ID') == '96'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
hashes = {}
for p in logs.glob('*.txt'):
    shutil.copy2(p, backup / p.name)
    hashes[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
print('OUTPUT ' + str(out), flush=True)
env = os.environ.copy()
monitor = player = None
result = {}
capture_log = (out / 'capture.log').open('w')
player_log = (out / 'player.log').open('w')
try:
    monitor = subprocess.Popen([
        '/usr/bin/python3', '-u', str(home / 'capture_monitor_only.py'),
        '--seconds', '2', '--ros-domain', '96', '--output-root', str(out),
        '--serial-path', '/tmp/intentionally_missing_test_serial_20261020'],
        env=env, stdout=capture_log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(1)
    player = subprocess.Popen([
        'ros2', 'bag', 'play', str(bag), '--clock', '-r', '1.0',
        '--disable-keyboard-controls', '--read-ahead-queue-size', '100',
        '--topics', '/livox/lidar', '/livox/imu'],
        env=env, stdout=player_log, stderr=subprocess.STDOUT, start_new_session=True)
    player.wait(timeout=38)
    # Inject an implausible displayed pose in this isolated test domain only.
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('isolated_test_pose_fault_injector')
    publisher = node.create_publisher(Odometry, '/aft_mapped_to_init', 10)
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        message = Odometry()
        message.pose.pose.position.x = 10000.0
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    run = next(out.glob('slam_manual_*'))
    manifest = json.loads((run / 'processes.json').read_text())
    events = [json.loads(line) for line in (run / 'events.jsonl').read_text().splitlines()]
    names = {row.get('reason') for row in events if row.get('event') == 'ALERT_ONLY'}
    alive = {}
    for name, pid in manifest['processes'].items():
        try:
            os.kill(pid, 0)
            state = Path(f'/proc/{pid}/stat').read_text().split(') ')[1].split()[0]
            alive[name] = state != 'Z'
        except ProcessLookupError:
            alive[name] = False
    result = {
        'directory': str(out), 'test_run': str(run), 'input_bag': str(bag),
        'isolated_domain': 96, 'monitor_alive_before_manual_stop': monitor.poll() is None,
        'children_alive_after_faults_and_input_eof': alive,
        'alerts_observed': sorted(names),
        'configured_reminder_elapsed_without_stop': any(row['event'] == 'REMINDER_TIME_REACHED_CONTINUING' for row in events),
        'automatic_stop_events': [row for row in events if row['event'] in ['STOP_REQUESTED', 'USER_STOP_REQUESTED']],
        'fake_pose_only_in_isolated_test_not_original_data': True}
    assert all(alive.values()) and len(alive) == 2 and result['monitor_alive_before_manual_stop']
    assert result['configured_reminder_elapsed_without_stop'] and not result['automatic_stop_events']
    assert {'serial_device_missing', 'imu_receive_timeout', 'lidar_receive_timeout',
            'position_nonfinite_or_over_50m', 'input_skipped_lidar'} <= names
finally:
    # Explicit test teardown, never a reaction to a sensor/pose warning.
    if monitor is not None and monitor.poll() is None:
        os.kill(monitor.pid, signal.SIGINT)
        monitor.wait(timeout=45)
    if player is not None and player.poll() is None:
        os.killpg(player.pid, signal.SIGINT)
        player.wait(timeout=10)
    capture_log.close()
    player_log.close()
    for p in backup.iterdir():
        shutil.copy2(p, logs / p.name)
    result['original_logs_restored'] = all(hashlib.sha256((logs / name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
    runs = list(out.glob('slam_manual_*'))
    if runs and (runs[0] / 'summary.json').exists():
        result['manual_stop_summary'] = json.loads((runs[0] / 'summary.json').read_text())
    (out / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
