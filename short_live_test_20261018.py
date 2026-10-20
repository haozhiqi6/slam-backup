#!/usr/bin/env python3
"""Bounded live test using the installed mapper, with evidence backups."""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from livox_interfaces.msg import CustomMsg
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument('label', choices=['static', 'small_move'])
ap.add_argument('--seconds', type=float, default=60)
args = ap.parse_args()
assert 10 <= args.seconds <= 90
home = Path('/home/orangepi')
ws = home/'dl/livox_ws'
root = ws/'src/FASTLIVO2_ROS2-master'
out = home/('slam_short_'+time.strftime('%Y%m%d_%H%M%S')+'_'+args.label)
out.mkdir()
backup = out/'previous_logs'
backup.mkdir()
for file in (root/'Log').glob('*.txt'):
    shutil.copy2(file, backup/file.name)
env = os.environ.copy()
assert env.get('ROS_DOMAIN_ID') == '30'
env['ROS_LOG_DIR'] = str(out/'ros_log')
config = ws/'install/fast_livo/share/fast_livo/config'
for name in ['avia.yaml', 'camera_pinhole.yaml']:
    shutil.copy2(config/name, out/name)
binary = ws/'install/fast_livo/lib/fast_livo/fastlivo_mapping'
if not binary.exists():
    raise RuntimeError('Installed mapper not found')

def limits():
    os.setsid()
    resource.setrlimit(resource.RLIMIT_AS, (int(2.5*1024**3),)*2)

def stop(p):
    if p is None or p.poll() is not None:
        return
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)

rclpy.init()
node = rclpy.create_node('short_live_test_monitor')
rows = {key: [] for key in ['imu', 'lidar', 'odom', 'cloud']}
phase_start = time.monotonic()

def base(msg):
    return [time.monotonic()-phase_start, time.time(),
            msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9]

def imu(msg):
    rows['imu'].append(base(msg)+[
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

def lidar(data):
    sec, ns, string_len = struct.unpack_from('<iII', data, 4)
    after_header = 16 + string_len
    offset = 4 + ((after_header - 4 + 7)//8)*8
    point_num = struct.unpack_from('<I', data, offset+16)[0]
    start = offset+20
    offsets = np.ndarray((point_num,),dtype='<u4',buffer=data,offset=start,strides=(20,)).astype(np.int64)
    d = []
    if len(rows['lidar']) % 10 == 0:
        xyz = np.ndarray((point_num,3),dtype='<f4',buffer=data,offset=start+4,strides=(20,4))
        ranges = np.linalg.norm(xyz,axis=1)
        d = ranges[np.isfinite(ranges)&(ranges>0)]
    rows['lidar'].append([time.monotonic()-phase_start,time.time(),sec+ns*1e-9,
        point_num, int(offsets.max(initial=0))*1e-9,
        int((np.diff(offsets)<0).sum()),
        float(np.median(d)) if len(d) else None,
        float(np.mean(d>=2)) if len(d) else None])

def odom(msg):
    p=msg.pose.pose.position
    q=msg.pose.pose.orientation
    rows['odom'].append(base(msg)+[p.x,p.y,p.z,q.x,q.y,q.z,q.w])

subs = [node.create_subscription(Imu, '/livox/imu', imu, qos_profile_sensor_data),
        node.create_subscription(CustomMsg, '/livox/lidar', lidar, qos_profile_sensor_data,raw=True),
        node.create_subscription(Odometry, '/aft_mapped_to_init', odom, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/cloud_registered',
                                 lambda data: rows['cloud'].append([time.monotonic()-phase_start,time.time(),
                                     struct.unpack_from('<i',data,4)[0]+struct.unpack_from('<I',data,8)[0]*1e-9]),
                                 qos_profile_sensor_data,raw=True)]
mapper = recorder = None
reason = 'completed'
rss_peak = 0
exit_before_stop = None
started = time.monotonic()
print('OUTPUT '+str(out), flush=True)
log = (out/'mapper.log').open('w')
baglog = (out/'recorder.log').open('w')
try:
    recorder = subprocess.Popen(['ros2','bag','record','-o',str(out/'raw'),
        '/livox/lidar','/livox/imu','/aft_mapped_to_init'], env=env,
        stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
    discovery_end = time.monotonic()+3
    while time.monotonic()<discovery_end:
        rclpy.spin_once(node,timeout_sec=.05)
    mapper = subprocess.Popen([str(binary),'--ros-args','-r','__node:=laserMapping_short_test',
        '--params-file',str(config/'avia.yaml'),'--params-file',str(config/'camera_pinhole.yaml')],
        env=env,cwd=ws,stdout=log,stderr=subprocess.STDOUT,preexec_fn=limits)
    print('TEST_STARTED '+args.label+' '+time.strftime('%H:%M:%S'),flush=True)
    test_start = time.monotonic()
    checkpoint = test_start
    while time.monotonic()-test_start < args.seconds:
        rclpy.spin_once(node,timeout_sec=.03)
        now=time.monotonic()
        if mapper.poll() is not None:
            reason='mapper exited';break
        if recorder.poll() is not None:
            reason='recorder exited';break
        if now-checkpoint < 1:
            continue
        checkpoint=now
        status=Path(f'/proc/{mapper.pid}/status').read_text()
        match=re.search(r'^VmRSS:\s+(\d+)',status,re.M)
        rss=int(match[1]) if match else 0
        rss_peak=max(rss_peak,rss)
        p=rows['odom'][-1][3:6] if rows['odom'] else None
        print(json.dumps({'elapsed_s':round(now-test_start,1),
            'counts':{k:len(v) for k,v in rows.items()},'position':p,
            'mapper_rss_mib':round(rss/1024,1)}),flush=True)
        if rss>900*1024:
            reason='mapper RSS exceeded 900 MiB';break
        if p and (not all(math.isfinite(v) for v in p) or math.sqrt(sum(v*v for v in p))>5):
            reason='position exceeded 5 m or became nonfinite';break
finally:
    exit_before_stop=mapper.poll() if mapper else None
    stop(mapper)
    stop(recorder)
    log.close();baglog.close()
    node.destroy_node();rclpy.shutdown()
    for key,value in rows.items():
        (out/(key+'.json')).write_text(json.dumps(value))
    for file in (root/'Log').glob('*.txt'):
        shutil.copy2(file,out/file.name)

def stream_stats(key, threshold):
    x=rows[key]
    if len(x)<2:return {'count':len(x)}
    t=np.array([v[2] for v in x]); dt=np.diff(t)
    return {'count':len(x),'header_span_s':float(t[-1]-t[0]),
        'receive_span_s':x[-1][0]-x[0][0],
        'header_backsteps':int(sum(dt<0)), 'max_header_gap_s':float(dt.max()),
        'min_header_step_s':float(dt.min()), 'gaps_above_threshold':int(sum(dt>threshold)),
        'receive_minus_header_median_s':float(np.median([v[1]-v[2] for v in x]))}

text=(out/'mapper.log').read_text(errors='replace')
effect=[int(x) for x in re.findall('effective feature num: (\\d+)',text)]
positions=np.array([x[3:6] for x in rows['odom']])
summary={'label':args.label,'output':str(out),'wall_s':time.monotonic()-started,
    'stop_reason':reason,'mapper_exit_before_stop':exit_before_stop,
    'peak_mapper_rss_mib':rss_peak/1024,
    'streams':{k:stream_stats(k,.025 if k=='imu' else .25) for k in rows},
    'lidar_abnormal_frame_spans':sum(x[4]>.15 or x[5]>0 for x in rows['lidar']),
    'lidar_max_point_offset_s':max((x[4] for x in rows['lidar']),default=None),
    'lidar_median_range_m':float(np.median([x[6] for x in rows['lidar'] if x[6] is not None])) if rows['lidar'] else None,
    'sync_warning_count':text.count('not synced'),
    'lidar_rollback_count':text.count('lidar loop back'),
    'zero_match_iterations':effect.count(0),
    'effective_feature_median':float(np.median(effect)) if effect else None,
    'voxel_overflow_count':text.count('Integer indices would overflow')}
if len(positions):
    summary['position']={'first':positions[0].tolist(),'last':positions[-1].tolist(),
        'max_distance_from_first_m':float(np.linalg.norm(positions-positions[0],axis=1).max()),
        'max_step_m':float(np.linalg.norm(np.diff(positions,axis=0),axis=1).max()) if len(positions)>1 else 0}
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print('TEST_FINISHED '+json.dumps(summary),flush=True)
