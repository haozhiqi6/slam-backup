#!/usr/bin/env python3
"""Offline, bounded comparisons with the installed mapper; no sensor changes."""
from pathlib import Path
import argparse,gc,hashlib,json,math,os,re,resource,shutil,signal,sqlite3,struct,subprocess,time
import numpy as np
import rclpy
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile,ReliabilityPolicy

H=Path('/home/orangepi');WS=H/'dl/livox_ws';SRC=WS/'src/FASTLIVO2_ROS2-master'
ap=argparse.ArgumentParser();ap.add_argument('--resume-from');args=ap.parse_args()
ROOT=H/('slam_review_'+time.strftime('%Y%m%d_%H%M%S'));ROOT.mkdir()
ENV=os.environ.copy();assert ENV.get('ROS_DOMAIN_ID')=='89' and ENV.get('ROS_LOCALHOST_ONLY')=='1'
CONFIG=WS/'install/fast_livo/share/fast_livo/config'
for name in ['avia.yaml','camera_pinhole.yaml']:shutil.copy2(CONFIG/name,ROOT/name)

def no_other_mapper():
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            argv=(p/'cmdline').read_bytes().split(bytes([0]))
            if argv and os.fsdecode(argv[0]).endswith('/fastlivo_mapping'):raise RuntimeError('Another mapper is active')
        except (FileNotFoundError,PermissionError):pass

no_other_mapper();backup=ROOT/'previous_logs';backup.mkdir();hashes={}
for p in (SRC/'Log').glob('*.txt'):
    shutil.copy2(p,backup/p.name);hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
print('OUTPUT '+str(ROOT),flush=True)

def screen_bag(source,dest):
    con=sqlite3.connect('file:'+str(next(source.glob('*.db3')))+'?mode=ro',uri=True)
    ts={x[0]:x[1:] for x in con.execute('select id,name,type,serialization_format,offered_qos_profiles from topics')}
    w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(dest),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for name,typ,fmt,qos in ts.values():
        if name in ['/livox/lidar','/livox/imu']:w.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=fmt,offered_qos_profiles=qos))
    stats={'lidar_written':0,'imu_written':0,'lidar_header_rejected':0,'imu_header_rejected':0,'original_points':0,'rejected_points':0,'reordered_frames':0}
    last={}
    for tid,stamp,data in con.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name=ts[tid][0]
        if name not in ['/livox/lidar','/livox/imu']:continue
        sec,ns=struct.unpack_from('<iI',data,4);head=sec*10**9+ns
        if name in last and head<=last[name]:
            stats['lidar_header_rejected' if name=='/livox/lidar' else 'imu_header_rejected']+=1;continue
        if name=='/livox/lidar':
            sl=struct.unpack_from('<I',data,12)[0];off=4+((16+sl-4+7)//8)*8;n=struct.unpack_from('<I',data,off+16)[0];start=off+20
            # CDR omits the final point's trailing alignment byte.
            padded=data+bytes(max(0,start+n*20-len(data)))
            pts=np.ndarray((n,),dtype='V20',buffer=padded,offset=start)
            times=np.ndarray((n,),dtype='<u4',buffer=data,offset=start,strides=(20,))
            keep=times<=150_000_000;stats['original_points']+=n;stats['rejected_points']+=int((~keep).sum())
            kept=pts[keep];tt=times[keep]
            if not len(kept):stats['lidar_header_rejected']+=1;continue
            if (np.diff(tt.astype(np.int64))<0).any():
                kept=kept[np.argsort(tt,kind='stable')];stats['reordered_frames']+=1
            if len(kept)!=n or not (times[keep]==np.sort(times[keep],kind='stable')).all():
                prefix=bytearray(data[:start]);struct.pack_into('<I',prefix,off+8,len(kept));struct.pack_into('<I',prefix,off+16,len(kept));data=bytes(prefix)+kept.tobytes()
            stats['lidar_written']+=1
        else:stats['imu_written']+=1
        last[name]=head;w.write(name,data,stamp)
    del w;gc.collect();con.close();return stats

bags={'last':H/'slam_guarded_20261018_212204/raw','raw02':H/'slam_raw_20261018_02'}
screening={}
for name,source in bags.items():
    screening[name]=screen_bag(source,ROOT/(name+'_input_screened'))
    print('SCREENED '+name+' '+json.dumps(screening[name]),flush=True)

def limits():
    os.setsid();resource.setrlimit(resource.RLIMIT_AS,(int(2.5*1024**3),)*2)

def stop(p):
    if p is None or p.poll() is not None:return
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)

results={}
if args.resume_from:
    prior=Path(args.resume_from);saved=json.loads((prior/'results.json').read_text())
    for name,value in saved['results'].items():
        if value['reason']=='completed':
            results[name]=value;shutil.copytree(prior/name,ROOT/name)
rclpy.init()
try:
    cases=[('last_original',bags['last'],45),('last_screened',ROOT/'last_input_screened',45),
           ('raw02_original',bags['raw02'],135),('raw02_screened',ROOT/'raw02_input_screened',135)]
    for label,bag,limit in cases:
        if label in results:
            print('REUSED_COMPLETED '+label,flush=True);continue
        no_other_mapper();out=ROOT/label;out.mkdir();env=ENV.copy();env['ROS_LOG_DIR']=str(out/'ros_log')
        node=rclpy.create_node('recording_review_'+label);poses=[];cloud_count=[0]
        def pose(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            poses.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
        subs=[node.create_subscription(Odometry,'/aft_mapped_to_init',pose,qos),
              node.create_subscription(PointCloud2,'/cloud_registered',lambda _:cloud_count.__setitem__(0,cloud_count[0]+1),qos,raw=True)]
        mlog=(out/'mapper.log').open('w');plog=(out/'player.log').open('w')
        mapper=player=None;started=time.monotonic();last_status=started;reason='completed';peak=0;done=None
        print('CASE_START '+label,flush=True)
        try:
            mapper=subprocess.Popen([str(WS/'install/fast_livo/lib/fast_livo/fastlivo_mapping'),'--ros-args','-r','__node:=review_'+label,
                '--params-file',str(ROOT/'avia.yaml'),'--params-file',str(ROOT/'camera_pinhole.yaml'),'-p','use_sim_time:=true'],
                env=env,cwd=WS,stdout=mlog,stderr=subprocess.STDOUT,preexec_fn=limits)
            till=time.monotonic()+2
            while time.monotonic()<till:rclpy.spin_once(node,timeout_sec=.05)
            player=subprocess.Popen(['ros2','bag','play',str(bag),'--clock','-r','1.0','--disable-keyboard-controls',
                '--read-ahead-queue-size','100','--topics','/livox/lidar','/livox/imu'],env=env,
                stdout=plog,stderr=subprocess.STDOUT,start_new_session=True)
            while time.monotonic()-started<limit:
                rclpy.spin_once(node,timeout_sec=.03);now=time.monotonic()
                if mapper.poll() is not None:reason='mapper exited';break
                if player.poll() is not None:
                    if done is None:done=now
                    if now-done>4:break
                if poses:
                    norm=math.sqrt(sum(v*v for v in poses[-1][1:4]))
                    if not math.isfinite(norm) or norm>100:reason='position nonfinite or over 100m';break
                if now-last_status>=10:
                    last_status=now;s=Path(f'/proc/{mapper.pid}/status').read_text();m=re.search(r'^VmRSS:\s+(\d+)',s,re.M);rss=int(m[1])/1024 if m else 0;peak=max(peak,rss)
                    print(json.dumps({'case':label,'wall_s':round(now-started,1),'poses':len(poses),'position':poses[-1][1:4] if poses else None,'rss_mib':rss}),flush=True)
                    if rss>900:reason='memory limit';break
            else:reason='time limit'
        finally:
            stop(player);stop(mapper);mlog.close();plog.close();node.destroy_node()
            for name in ['mat_out.txt','mat_pre.txt','imu.txt']:
                p=SRC/'Log'/name
                if p.exists():shutil.copy2(p,out/name)
        text=(out/'mapper.log').read_text(errors='replace');features=[int(x) for x in re.findall(r'effective feature num: (\d+)',text)]
        residuals=[float(x) for x in re.findall(r'average residual: ([0-9.eE+-]+)',text)]
        arr=np.asarray(poses)
        results[label]={'reason':reason,'wall_s':time.monotonic()-started,'pose_count':len(poses),'cloud_frames':cloud_count[0],'peak_rss_mib':peak,
            'last_position':poses[-1][1:4] if poses else None,'maximum_position_norm_m':float(np.linalg.norm(arr[:,1:4],axis=1).max()) if poses else None,
            'pose_time_span_s':float(arr[-1,0]-arr[0,0]) if len(poses)>1 else None,
            'effective_median':float(np.median(features)) if features else None,'zero_match_prints':features.count(0),
            'residual_median':float(np.median(residuals)) if residuals else None,'sync_warnings':text.count('not synced'),
            'lidar_rollback_messages':text.count('lidar loop back'),'voxel_overflow_messages':text.count('Integer indices')}
        (out/'poses.json').write_text(json.dumps(poses));(ROOT/'results.json').write_text(json.dumps({'screening':screening,'results':results},indent=2))
        print('CASE_FINISHED '+label+' '+json.dumps(results[label]),flush=True)
finally:
    rclpy.shutdown()
    for p in backup.iterdir():shutil.copy2(p,SRC/'Log'/p.name)
    restored=all(hashlib.sha256((SRC/'Log'/name).read_bytes()).hexdigest()==digest for name,digest in hashes.items())
    (ROOT/'restore.json').write_text(json.dumps({'original_logs_restored':restored},indent=2))
print('REVIEW_FINISHED '+str(ROOT),flush=True)
