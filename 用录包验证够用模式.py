#!/usr/bin/env python3
"""Run the practical capture end-to-end against a bag in isolated domain 94."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,signal,subprocess,time
ap=argparse.ArgumentParser();ap.add_argument('--bag',default='/home/orangepi/slam_guarded_20261018_212204/raw');args=ap.parse_args()
H=Path('/home/orangepi');WS=H/'dl/livox_ws';LOGS=WS/'src/FASTLIVO2_ROS2-master/Log'
out=H/('practical_validation_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir();backup=out/'previous_logs';backup.mkdir()
hashes={}
for p in LOGS.glob('*.txt'):
 shutil.copy2(p,backup/p.name);hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
env=os.environ.copy();env.update(ROS_DOMAIN_ID='94',ROS_LOCALHOST_ONLY='1')
guard=player=None;glog=(out/'capture.log').open('w');plog=(out/'player.log').open('w')
print('OUTPUT '+str(out),flush=True)
try:
 guard=subprocess.Popen(['python3','-u',str(H/'guarded_capture.py'),'--seconds','35','--mode','move','--policy','practical','--ros-domain','94'],env=env,stdout=glog,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(1)
 player=subprocess.Popen(['ros2','bag','play',args.bag,'--clock','-r','1.0','--disable-keyboard-controls','--read-ahead-queue-size','100','--topics','/livox/lidar','/livox/imu'],env=env,stdout=plog,stderr=subprocess.STDOUT,start_new_session=True)
 guard.wait(timeout=48)
finally:
 for p in [guard,player]:
  if p is not None and p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
 glog.close();plog.close()
 for p in backup.iterdir():shutil.copy2(p,LOGS/p.name)
text=(out/'capture.log').read_text(errors='replace');events=[]
for line in text.splitlines():
 try:events.append(json.loads(line))
 except ValueError:pass
done=[e for e in events if e.get('event')=='FINISHED'];summary=done[-1]['summary'] if done else None
result={'validation_directory':str(out),'input_bag':args.bag,'summary':summary,'original_logs_restored':all(hashlib.sha256((LOGS/n).read_bytes()).hexdigest()==v for n,v in hashes.items())}
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if summary is None or not summary['metadata_exists'] or summary['state']['odom_count']==0 or summary['state']['filtered_points']<1 or summary['recorder_exit']!=0:raise RuntimeError('Practical capture validation failed; inspect logs')
