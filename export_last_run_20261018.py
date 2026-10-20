#!/usr/bin/env python3
"""Reconstruct the valid portion of one saved bag and export a local PCD."""
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import sqlite3
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

HOME=Path('/home/orangepi')
WS=HOME/'dl/livox_ws'
SOURCE=WS/'src/FASTLIVO2_ROS2-master'
BAG=HOME/'slam_guarded_20261018_212204/raw'
OUT=HOME/('slam_export_212204_'+time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir()
ENV=os.environ.copy()
assert ENV.get('ROS_DOMAIN_ID')=='89' and ENV.get('ROS_LOCALHOST_ONLY')=='1'
ENV['ROS_LOG_DIR']=str(OUT/'ros_log')
CONFIG=WS/'install/fast_livo/share/fast_livo/config'
for name in ['avia.yaml','camera_pinhole.yaml']:
    shutil.copy2(CONFIG/name,OUT/name)
print('OUTPUT '+str(OUT),flush=True)

# No other mapper may write these fixed output paths while we borrow them.
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        argv=p.joinpath('cmdline').read_bytes().split(bytes([0]))
        if argv and os.fsdecode(argv[0]).endswith('/fastlivo_mapping'):
            raise RuntimeError('A mapper is active; export did not start')
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
backup=OUT/'previous_logs';backup.mkdir()
log_hashes={}
for path in (SOURCE/'Log').glob('*.txt'):
    shutil.copy2(path,backup/path.name)
    log_hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()

def filter_bag():
    db=sqlite3.connect('file:'+str(next(BAG.glob('*.db3')))+'?mode=ro',uri=True)
    topics={row[0]:row[1:] for row in db.execute('select id,name,type,serialization_format,offered_qos_profiles from topics')}
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(OUT/'valid_raw'),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('',''))
    for name,typ,fmt,qos in topics.values():
        if name not in ['/livox/lidar','/livox/imu']:continue
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=fmt,offered_qos_profiles=qos))
    counts={'/livox/lidar':0,'/livox/imu':0};excluded=[]
    for tid,stamp,data in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name=topics[tid][0]
        if name not in counts:continue
        if name=='/livox/lidar':
            string_len=struct.unpack_from('<I',data,12)[0]
            off=4+((16+string_len-4+7)//8)*8
            count=struct.unpack_from('<I',data,off+16)[0]
            offsets=np.ndarray((count,),dtype='<u4',buffer=data,offset=off+20,strides=(20,)).astype(np.int64)
            maximum=int(offsets.max(initial=0))
            if maximum>150_000_000 or (np.diff(offsets)<0).any():
                excluded.append({'receive_ns':stamp,'maximum_offset_ns':maximum})
                continue
        writer.write(name,data,stamp);counts[name]+=1
    del writer;gc.collect();db.close()
    return {'included':counts,'excluded_lidar_frames':excluded}

filtered=filter_bag()
print('FILTERED '+json.dumps(filtered),flush=True)
rclpy.init();node=rclpy.create_node('local_map_export_collector')
qos=QoSProfile(depth=40,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
clouds=[];poses=[];frames=[];last_cloud=None
types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}

def cloud(msg):
    global last_cloud
    fields={f.name:f for f in msg.fields}
    names=[n for n in ['x','y','z','intensity'] if n in fields]
    if not all(n in fields for n in ['x','y','z']):raise RuntimeError('Missing XYZ')
    order='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':names,'formats':[order+types[fields[n].datatype] for n in names],
        'offsets':[fields[n].offset for n in names],'itemsize':msg.point_step})
    data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    result=np.zeros((len(data),4),dtype=np.float32)
    for i,name in enumerate(['x','y','z','intensity']):
        if name in names:result[:,i]=data[name]
    result=result[np.isfinite(result).all(axis=1)]
    clouds.append(result);frames.append(msg.header.frame_id);last_cloud=time.monotonic()

def odom(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    poses.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])

subs=[node.create_subscription(PointCloud2,'/cloud_registered',cloud,qos),
      node.create_subscription(Odometry,'/aft_mapped_to_init',odom,qos)]

def limits():
    os.setsid();resource.setrlimit(resource.RLIMIT_AS,(int(2.5*1024**3),)*2)

def stop(p):
    if p is None or p.poll() is not None:return
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)

mapper=player=None;reason='completed';peak_rss=0;started=time.monotonic()
mlog=(OUT/'reconstruction.log').open('w');plog=(OUT/'player.log').open('w')
try:
    mapper=subprocess.Popen([str(WS/'install/fast_livo/lib/fast_livo/fastlivo_mapping'),
        '--ros-args','-r','__node:=laserMapping_export','--params-file',str(OUT/'avia.yaml'),
        '--params-file',str(OUT/'camera_pinhole.yaml'),'-p','use_sim_time:=true'],
        env=ENV,cwd=WS,stdout=mlog,stderr=subprocess.STDOUT,preexec_fn=limits)
    discovery=time.monotonic()+2
    while time.monotonic()<discovery:rclpy.spin_once(node,timeout_sec=.05)
    player=subprocess.Popen(['ros2','bag','play',str(OUT/'valid_raw'),'--clock','-r','0.5',
        '--read-ahead-queue-size','100','--disable-keyboard-controls','--wait-for-all-acked','3000'],
        env=ENV,stdout=plog,stderr=subprocess.STDOUT,start_new_session=True)
    player_done=None;checkpoint=time.monotonic()
    while time.monotonic()-started<85:
        rclpy.spin_once(node,timeout_sec=.03)
        now=time.monotonic()
        if mapper.poll() is not None:reason='mapper exited early';break
        if player.poll() is not None:
            if player_done is None:player_done=now
            if now-player_done>4 and (last_cloud is None or now-last_cloud>2):break
        if now-checkpoint>=5:
            checkpoint=now
            status=Path(f'/proc/{mapper.pid}/status').read_text()
            match=re.search(r'^VmRSS:\s+(\d+)',status,re.M);rss=int(match[1])/1024 if match else 0
            peak_rss=max(peak_rss,rss)
            print(json.dumps({'elapsed_s':now-started,'cloud_frames':len(clouds),'poses':len(poses),'mapper_rss_mib':rss}),flush=True)
            if rss>900:reason='mapper memory limit';break
            if poses and (not np.isfinite(poses[-1]).all() or np.linalg.norm(poses[-1][1:4])>20):
                reason='invalid or over-20m position';break
    else:reason='reconstruction timeout'
finally:
    stop(player);stop(mapper);mlog.close();plog.close()
    node.destroy_node();rclpy.shutdown()
    for name in ['mat_out.txt','mat_pre.txt','imu.txt']:
        q=SOURCE/'Log'/name
        if q.exists():shutil.copy2(q,OUT/('reconstructed_'+name))
    for path in backup.iterdir():shutil.copy2(path,SOURCE/'Log'/path.name)

if not clouds:raise RuntimeError('No point clouds reconstructed; see '+str(OUT))
if len(set(frames))!=1:raise RuntimeError('Mixed output frames')
points=np.concatenate(clouds)
# 3 cm voxel representatives preserve one measured point per occupied voxel.
keys=np.floor(points[:,:3]/.03).astype(np.int32)
_,indices=np.unique(keys,axis=0,return_index=True)
points=points[np.sort(indices)].astype('<f4',copy=False)
pcd=OUT/'local_map.pcd'
header=('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
        'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH '+str(len(points))+
        '\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(points))+'\nDATA binary\n')
with pcd.open('wb') as f:f.write(header.encode());f.write(points.tobytes())
np.savetxt(OUT/'trajectory.csv',np.asarray(poses),delimiter=',',header='timestamp,x,y,z,qx,qy,qz,qw',comments='')
text=(OUT/'reconstruction.log').read_text(errors='replace')
summary={'source_bag':str(BAG),'directory':str(OUT),'filtering':filtered,
    'stop_reason':reason,'cloud_frames':len(clouds),'pose_count':len(poses),'exported_points':len(points),
    'coordinate_frame':frames[0],'voxel_size_m':.03,'bounds_min':points[:,:3].min(axis=0).tolist(),
    'bounds_max':points[:,:3].max(axis=0).tolist(),'peak_mapper_rss_mib':peak_rss,
    'zero_match_prints':text.count('effective feature num: 0'),'sync_warning_count':text.count('not synced'),
    'integer_overflow_count':text.count('Integer indices'),'pcd_bytes':pcd.stat().st_size,
    'pcd_sha256':hashlib.sha256(pcd.read_bytes()).hexdigest(),
    'original_logs_restored':all(hashlib.sha256((SOURCE/'Log'/n).read_bytes()).hexdigest()==d for n,d in log_hashes.items())}
(OUT/'export_summary.json').write_text(json.dumps(summary,indent=2))
print('EXPORTED '+json.dumps(summary),flush=True)
