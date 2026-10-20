#!/usr/bin/env python3
"""Record independently; observe faults without authority to stop children."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import struct
import subprocess
import time


class ManualControl:
    """Only an explicit manual request may signal recorder/mapper processes."""
    def __init__(self, emit):
        self.emit = emit
        self.children = {}
        self.requested = False
        self.reason = None
        self.signalled = set()
        self.last_alert = {}
        self.alert_counts = {}

    def alert(self, reason, **details):
        self.alert_counts[reason] = self.alert_counts.get(reason, 0) + 1
        now = time.monotonic()
        if now - self.last_alert.get(reason, -math.inf) >= 5:
            self.last_alert[reason] = now
            self.emit('ALERT_ONLY', reason=reason, action='continue', **details)

    def manual_stop(self, reason):
        if reason not in {'user_ctrl_c', 'user_stop_file'}:
            raise ValueError('A health warning is not authorization to stop')
        if not self.requested:
            self.requested = True
            self.reason = reason
            self.emit('USER_STOP_REQUESTED', reason=reason)
        for name in ['recorder', 'mapper']:
            child = self.children.get(name)
            if child is not None and child.poll() is None and child.pid not in self.signalled:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                    self.signalled.add(child.pid)
                    self.emit('USER_STOP_SIGNAL_SENT', process=name, pid=child.pid)
                except ProcessLookupError:
                    pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0,
                        help='Optional reminder time; never a stopping deadline')
    parser.add_argument('--mode', choices=['static', 'move'], default='move')
    parser.add_argument('--policy', choices=['monitor', 'strict', 'practical'], default='monitor',
                        help='Legacy labels accepted; all use manual stopping only')
    parser.add_argument('--ros-domain', type=int, default=30)
    parser.add_argument('--output-root', type=Path, default=Path('/home/orangepi'))
    parser.add_argument('--serial-path', default='/dev/ttyUSB0')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != str(args.ros_domain):
        raise RuntimeError('ROS_DOMAIN_ID does not match requested domain')
    home = Path('/home/orangepi')
    workspace = home / 'dl/livox_ws'
    source = workspace / 'src/FASTLIVO2_ROS2-master'
    config = workspace / 'install/fast_livo/share/fast_livo/config'
    binary = workspace / 'install/fast_livo/lib/fast_livo/fastlivo_mapping'
    # Refuse duplicate launches; never stop somebody else's existing process.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            words = proc.joinpath('cmdline').read_bytes().split(b'\0')
            if words and (os.fsdecode(words[0]).endswith('/fastlivo_mapping') or
                          (b'bag' in words and b'record' in words)):
                raise RuntimeError('Another mapper/recorder is active: ' + proc.name)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from livox_interfaces.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from sensor_screening import screen_lidar, screen_imu

    out = args.output_root / ('slam_manual_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True)
    (out / 'previous_logs').mkdir()
    for path in (source / 'Log').glob('*.txt'):
        shutil.copy2(path, out / 'previous_logs' / path.name)
    for name in ['avia.yaml', 'camera_pinhole.yaml']:
        shutil.copy2(config / name, out / name)
    events = (out / 'events.jsonl').open('w', buffering=1)
    stream = (out / 'streams.csv').open('w', buffering=1)
    stream.write('topic,elapsed_s,receive_wall,header_time\n')
    start = time.monotonic()

    def emit(event, **details):
        row = {'event': event, 'elapsed_s': time.monotonic()-start,
               'wall': time.time(), **details}
        line = json.dumps(row)
        events.write(line + '\n')
        try:
            print(line, flush=True)
        except (BrokenPipeError, OSError):
            # Losing a terminal is not permission to stop a recording.
            pass

    control = ManualControl(emit)
    state = {'imu_count': 0, 'lidar_count': 0, 'odom_count': 0,
             'last_imu': None, 'last_lidar': None, 'position': None,
             'filtered_points': 0, 'skipped_lidar': 0, 'skipped_imu': 0,
             'last_effective': None, 'last_positive_match': None}
    accepted = {'imu': None, 'lidar': None}
    previous_raw = {'imu': None, 'lidar': None}
    rclpy.init()
    node = rclpy.create_node('manual_only_capture_monitor')
    receive_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    publish_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    publishers = {
        'imu': node.create_publisher(Imu, '/' + out.name + '/imu', publish_qos),
        'lidar': node.create_publisher(CustomMsg, '/' + out.name + '/lidar', publish_qos)}

    def receive(kind, data):
        now = time.monotonic()
        old_receive = state['last_' + kind]
        state['last_' + kind] = now
        state[kind + '_count'] += 1
        try:
            sec, ns = struct.unpack_from('<iI', data, 4)
            head = sec * 10**9 + ns
            stream.write(f'{kind},{now-start:.9f},{time.time():.9f},{head/1e9:.9f}\n')
            if old_receive is not None and now-old_receive > (.2 if kind == 'imu' else .5):
                control.alert(kind + '_receive_gap', gap_s=now-old_receive)
            old_head = previous_raw[kind]
            if old_head is not None and (head <= old_head or head-old_head > (200_000_000 if kind == 'imu' else 500_000_000)):
                control.alert(kind + '_header_step', step_s=(head-old_head)/1e9)
            previous_raw[kind] = head
            processed, info = (screen_imu(data, accepted[kind]) if kind == 'imu' else
                               screen_lidar(data, accepted[kind], accepted['imu']))
            if processed is None:
                state['skipped_' + kind] += 1
                control.alert('input_skipped_' + kind, details=info)
                return
            accepted[kind] = info['header_ns']
            if kind == 'lidar':
                state['filtered_points'] += info['dropped_points']
            publishers[kind].publish(processed)
        except Exception as exc:
            control.alert('input_callback_error', topic=kind, error=repr(exc))

    def odom(msg):
        p = msg.pose.pose.position
        state['odom_count'] += 1
        state['position'] = [p.x, p.y, p.z]
        norm = math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z)
        if not math.isfinite(norm) or norm > 50:
            control.alert('position_nonfinite_or_over_50m', position=state['position'])

    subscriptions = [
        node.create_subscription(Imu, '/livox/imu', lambda d: receive('imu', d), receive_qos, raw=True),
        node.create_subscription(CustomMsg, '/livox/lidar', lambda d: receive('lidar', d), receive_qos, raw=True),
        node.create_subscription(Odometry, '/aft_mapped_to_init', odom, receive_qos)]
    # Ctrl+C or a deliberately created stop.request is an explicit manual stop.
    # HUP/TERM and unexpected monitor failures do not forward signals to children.
    monitor_terminate = [False]
    signal.signal(signal.SIGINT, lambda *_: control.manual_stop('user_ctrl_c'))
    signal.signal(signal.SIGHUP, lambda *_: control.alert('terminal_disconnected'))
    def terminate_monitor(*_):
        control.alert('monitor_received_SIGTERM_children_left_running')
        monitor_terminate[0] = True
    signal.signal(signal.SIGTERM, terminate_monitor)
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(out / 'ros_log')
    logs = {}
    emit('OUTPUT', directory=str(out), policy='manual_stop_only', automatic_stop=False,
         requested_seconds_is_reminder_only=args.seconds)
    commands = {
        'recorder': ['ros2', 'bag', 'record', '-o', str(out / 'raw'),
                     '/livox/lidar', '/livox/imu', '/aft_mapped_to_init'],
        'mapper': [str(binary), '--ros-args', '-r', '__node:=laserMapping_manual',
                   '--params-file', str(out / 'avia.yaml'), '--params-file', str(out / 'camera_pinhole.yaml'),
                   '-p', 'common.lid_topic:=/' + out.name + '/lidar',
                   '-p', 'common.imu_topic:=/' + out.name + '/imu']}
    for name in ['recorder', 'mapper']:
        if control.requested:
            break
        logs[name] = (out / (name + '.log')).open('w')
        try:
            control.children[name] = subprocess.Popen(commands[name], env=env, cwd=workspace,
                                                      stdout=logs[name], stderr=subprocess.STDOUT,
                                                      start_new_session=True)
        except Exception as exc:
            control.alert(name + '_start_failed', error=repr(exc))
    manifest = {'directory': str(out), 'automatic_stop': False,
                'processes': {k: p.pid for k, p in control.children.items()},
                'monitor_pid': os.getpid(), 'policy': 'manual_stop_only'}
    (out / 'processes.json').write_text(json.dumps(manifest, indent=2))
    last_status = 0
    log_offset = 0
    ready = False
    reminder_sent = False
    emit('BASELINE_START', seconds=10)
    try:
        while not control.requested and not monitor_terminate[0]:
            try:
                rclpy.spin_once(node, timeout_sec=.03)
                now = time.monotonic()
                if (out / 'stop.request').exists():
                    control.manual_stop('user_stop_file')
                    continue
                if now-last_status < 1:
                    continue
                last_status = now
                elapsed = now-start
                if args.seconds > 0 and elapsed >= args.seconds and not reminder_sent:
                    reminder_sent = True
                    emit('REMINDER_TIME_REACHED_CONTINUING', seconds=args.seconds)
                for kind in ['imu', 'lidar']:
                    last = state['last_' + kind]
                    if last is None or now-last > (.2 if kind == 'imu' else .5):
                        control.alert(kind + '_receive_timeout', silence_s=None if last is None else now-last)
                if not Path(args.serial_path).exists():
                    control.alert('serial_device_missing')
                for name, child in control.children.items():
                    if child.poll() is not None:
                        control.alert(name + '_exited_other_processes_continue', exit_code=child.returncode)
                with (out / 'mapper.log').open(errors='replace') as file:
                    file.seek(log_offset)
                    chunk = file.read()
                    log_offset = file.tell()
                matches = [int(v) for v in re.findall(r'effective feature num: (\d+)', chunk)]
                if matches:
                    state['last_effective'] = matches[-1]
                    if any(v > 0 for v in matches):
                        state['last_positive_match'] = now
                if state['last_effective'] == 0:
                    control.alert('zero_matching')
                mapper = control.children.get('mapper')
                rss = None
                if mapper is not None and mapper.poll() is None:
                    status_path = Path(f'/proc/{mapper.pid}/status')
                    try:
                        match = re.search(r'^VmRSS:\s+(\d+)', status_path.read_text(), re.M)
                        rss = int(match[1])/1024 if match else None
                        if rss is not None and rss > 900:
                            control.alert('mapper_rss_over_900MiB', rss_mib=rss)
                    except OSError:
                        pass
                free = shutil.disk_usage(out).free
                if free < 1024**3:
                    control.alert('disk_free_below_1GiB', free_bytes=free)
                subscribed = (out / 'recorder.log').read_text(errors='replace')
                recording_ready = all('Subscribed to topic ' + repr(t) in subscribed for t in ['/livox/lidar', '/livox/imu'])
                if not ready and elapsed >= 10 and recording_ready and state['last_positive_match'] is not None and now-state['last_positive_match'] < 2:
                    ready = True
                    emit('MOVE_READY')
                emit('STATUS', recording_alive=bool(control.children.get('recorder') and control.children['recorder'].poll() is None),
                     mapping_alive=bool(mapper and mapper.poll() is None), mapper_rss_mib=rss, **state)
            except Exception as exc:
                control.alert('monitor_exception_continuing', error=repr(exc))
    finally:
        # No automatic termination, forced kill or finally cleanup of children.
        if control.requested:
            control.manual_stop(control.reason)
            for name, child in control.children.items():
                try:
                    child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    control.alert('manual_stop_pending_no_forced_kill', process=name)
            for path in (source / 'Log').glob('*.txt'):
                shutil.copy2(path, out / path.name)
        summary = {'directory': str(out), 'manual_stop_requested': control.requested,
                   'manual_stop_reason': control.reason, 'automatic_stop': False,
                   'state': state, 'alerts': control.alert_counts,
                   'processes': {k: {'pid': p.pid, 'returncode': p.poll()} for k, p in control.children.items()},
                   'metadata_exists': (out / 'raw/metadata.yaml').exists()}
        (out / 'summary.json').write_text(json.dumps(summary, indent=2))
        emit('FINISHED' if control.requested else 'MONITOR_EXITED_CHILDREN_NOT_STOPPED', summary=summary)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stream.close()
        events.close()
        for file in logs.values():
            file.close()


if __name__ == '__main__':
    main()
